"""音量(発話・BGM通常・BGM発話中を別々に設定、2026-10-09追加、docs/voice_levels.md)のテスト。

ロボット・スピーカー・Ollama・VOICEVOXには一切触れない(sounddevice/soundfileはスタブ、合成・再生は差し替え)。
1. voice_bridge 側の純ロジック: 範囲/型の検証(丸めず400相当)、ダッキング<=通常、部分更新・null=既定へ戻す、
   永続化(原子的・再起動後も維持)、ファイルなし/破損/不正値は既定値、優先順位(ファイル > 環境変数 > 既定値)、
   保存失敗でも反映+警告、ゲイン適用とクリップ防止の数値、VoiceMaster(全体ON/OFF)との独立、/bgmとの一本化
2. voice_bridge のHTTP: GET/POST /voice_levels、400で何も変えない、/status に現在値、/bgm と同じ値、
   OFF中でも保存でき次の発話から反映、発話の再生前に倍率がかかる
3. VoiceLevelsProxy とダッシュボードの /api/voice_levels(入力検証、権限: guestは閲覧のみ、未ログイン401、
   2台目不達はエラーJSON、未設定は非表示)
4. 画面(カード・緊急停止ドックが残っていること・JS構文・改行コード)

実行: python ros2_ws/src/nokosahen_core/test/test_voice_levels.py
(rclpy不要。voice_bridgeの依存の requests/numpy が入っていない環境ではその部分だけスキップ)
"""

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import types
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock

PKG = Path(__file__).resolve().parents[1]
REPO = PKG.parents[2]
sys.path.insert(0, str(PKG))

TMP = tempfile.mkdtemp(prefix="voice_levels_test_")
os.environ["VOICE_UTTERANCE_LOG_DB"] = os.path.join(TMP, "utt.db")
os.environ["VOICE_SETTINGS_FILE"] = os.path.join(TMP, "voice_settings.json")
os.environ["VOICE_MASTER_FILE"] = os.path.join(TMP, "master.json")
os.environ["VOICE_LEVELS_FILE"] = os.path.join(TMP, "levels_main.json")
os.environ["BGM_ENABLED"] = "0"
for _k in ("SPEECH_VOLUME", "BGM_VOLUME", "BGM_DUCK_VOLUME"):
    os.environ.pop(_k, None)

# ---- voice_bridge(sounddevice/soundfileはスタブ) ----
_sd = types.ModuleType("sounddevice")
_sd.PortAudioError = type("PortAudioError", (Exception,), {})
_sd.play = lambda *a, **k: None
_sd.wait = lambda: None
_sd.stop = lambda *a, **k: None
_sf = types.ModuleType("soundfile")
_sf.read = lambda path, dtype="float32": ([0.0] * 10, 24000)
sys.modules.setdefault("sounddevice", _sd)
sys.modules.setdefault("soundfile", _sf)
sys.path.insert(0, str(REPO / "voice_bridge"))
try:
    import numpy as np  # noqa: E402
    import voice_bridge as vb  # noqa: E402
except ImportError as _exc:  # requests/numpy等が無い環境
    vb = None
    np = None

# ---- ダッシュボード(rclpy系はスタブ) ----


def _install_stubs():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m

    class Msg:
        def __init__(self, *a, **k):
            self.data = None

    mod("geometry_msgs")
    mod("geometry_msgs.msg", PoseStamped=Msg, Twist=Msg)
    mod("nav_msgs")
    mod("nav_msgs.msg", Odometry=Msg)
    mod("std_msgs")
    mod("std_msgs.msg", Bool=Msg, Float32MultiArray=Msg, String=Msg, Empty=Msg)


try:
    import geometry_msgs.msg  # noqa: F401
    import nav_msgs.msg  # noqa: F401
    import std_msgs.msg as _std  # noqa: F401
except Exception:  # noqa: BLE001
    _install_stubs()

from nokosahen_core import dashboard_common as dc  # noqa: E402
from nokosahen_core.voice_levels_proxy import LEVEL_CONFIRM_ABOVE, LEVEL_LIMITS, LEVEL_MAX_ALL, VoiceLevelsProxy  # noqa: E402


def http(method, url, body=None, raw=None, headers=None, timeout=10):
    data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
    req = urllib.request.Request(url, data=data, method=method, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode("utf-8"))
    except urllib.error.HTTPError as e:
        try:
            return e.code, json.loads(e.read().decode("utf-8"))
        finally:
            e.close()


def serve(handler):
    srv = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, srv.server_address[1]


# ======================================================================
# 1. voice_bridge 純ロジック
# ======================================================================
@unittest.skipIf(vb is None, "voice_bridgeを読み込めません(requests/numpy未導入?)")
class LevelsLogicTest(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(TMP, "levels_logic.json")
        if os.path.exists(self.path):
            os.remove(self.path)

    def new(self, env=None):
        return vb.VoiceLevels(self.path, env={} if env is None else env)

    def test_defaults_without_file_are_louder_than_before(self):
        lv = self.new()
        self.assertEqual(lv.values(), vb.DEFAULT_LEVELS)
        # 旧既定(発話1.0・BGM 0.25・ダッキング0.07)より大きく、ダッキング<=通常
        self.assertGreater(lv.speech_volume, 1.0)
        self.assertGreater(lv.bgm_volume, 0.25)
        self.assertGreater(lv.bgm_duck_volume, 0.07)
        self.assertLessEqual(lv.bgm_duck_volume, lv.bgm_volume)
        self.assertEqual(lv.load_error, "")
        self.assertFalse(lv.from_file)

    def test_priority_file_over_env_over_default(self):
        env = {"SPEECH_VOLUME": "2.0", "BGM_VOLUME": "0.6", "BGM_DUCK_VOLUME": "0.2"}
        lv = self.new(env)
        self.assertEqual((lv.speech_volume, lv.bgm_volume, lv.bgm_duck_volume), (2.0, 0.6, 0.2))   # 環境変数 > 既定値
        lv.update({"speech_volume": 1.2})
        lv2 = self.new(env)
        self.assertEqual(lv2.speech_volume, 1.2)                                                    # ファイル > 環境変数
        self.assertEqual((lv2.bgm_volume, lv2.bgm_duck_volume), (0.6, 0.2))
        self.assertTrue(lv2.from_file)
        # null=既定に戻す は「環境変数 > 既定値」の値(ファイルの値ではなく)
        lv2.update({"speech_volume": None})
        self.assertEqual(lv2.speech_volume, 2.0)

    def test_bad_env_falls_back_to_default(self):
        lv = self.new({"SPEECH_VOLUME": "abc", "BGM_VOLUME": "5.5", "BGM_DUCK_VOLUME": "nan"})
        self.assertEqual(lv.values(), vb.DEFAULT_LEVELS)
        lv = self.new({"BGM_VOLUME": "0.05"})          # 既定ダッキングが通常を超えるなら通常に揃える
        self.assertEqual((lv.bgm_volume, lv.bgm_duck_volume), (0.05, 0.05))

    def test_persist_atomic_and_survives_restart(self):
        lv = self.new()
        out = lv.update({"speech_volume": 2.5, "bgm_volume": 0.7, "bgm_duck_volume": 0.3})
        self.assertTrue(out["ok"] and out["persisted"])
        text = Path(self.path).read_text(encoding="utf-8")
        self.assertNotIn("\r", text)
        doc = json.loads(text)
        self.assertEqual((doc["speech_volume"], doc["bgm_volume"], doc["bgm_duck_volume"]), (2.5, 0.7, 0.3))
        self.assertEqual([f for f in os.listdir(TMP) if f.startswith(".nokosahen_voice_levels_")], [])
        lv2 = self.new()
        self.assertEqual(lv2.values(), {"speech_volume": 2.5, "bgm_volume": 0.7, "bgm_duck_volume": 0.3, "announce_volume": 2.5})

    def test_missing_corrupt_or_invalid_file_uses_defaults(self):
        for content in ("{broken", "[]", "null", '{"speech_volume": "loud"}', '{"speech_volume": 9}',
                        '{"bgm_volume": -1}', '{"bgm_volume": true}', '{"speech_volume": NaN}'):
            Path(self.path).write_text(content, encoding="utf-8")
            lv = self.new()
            self.assertEqual(lv.values(), vb.DEFAULT_LEVELS, content)
            self.assertTrue(lv.load_error, content)
        # 一部だけ正しい値: 正しい項目は使い、不正な項目だけ既定値
        Path(self.path).write_text('{"speech_volume": 2.0, "bgm_volume": 99}', encoding="utf-8")
        lv = self.new()
        self.assertEqual(lv.speech_volume, 2.0)
        self.assertEqual(lv.bgm_volume, vb.DEFAULT_LEVELS["bgm_volume"])
        self.assertIn("bgm_volume", lv.load_error)

    def test_file_with_duck_above_volume_is_corrected(self):
        Path(self.path).write_text('{"bgm_volume": 0.2, "bgm_duck_volume": 0.5}', encoding="utf-8")
        lv = self.new()
        self.assertEqual((lv.bgm_volume, lv.bgm_duck_volume), (0.2, 0.2))
        self.assertTrue(lv.load_error)

    def test_old_saved_file_within_old_limits_stays_valid(self):
        # 前日までの上限(発話3・BGM1)内の保存値は、そのまま有効(上限を5に広げても値は変わらない)
        Path(self.path).write_text(json.dumps({"version": 1, "speech_volume": 2.9, "bgm_volume": 0.9,
                                               "bgm_duck_volume": 0.2, "updated_at": "2026-10-09T12:00:00"}), encoding="utf-8")
        lv = self.new()
        self.assertEqual(lv.values(), {"speech_volume": 2.9, "bgm_volume": 0.9, "bgm_duck_volume": 0.2, "announce_volume": 2.5})
        self.assertEqual(lv.load_error, "")
        self.assertTrue(lv.from_file)
        # 新しい上限いっぱいの値も保存・再読込できる
        lv.update({"speech_volume": 5.0, "bgm_volume": 5.0, "bgm_duck_volume": 5.0})
        self.assertEqual(self.new().values(), {"speech_volume": 5.0, "bgm_volume": 5.0, "bgm_duck_volume": 5.0, "announce_volume": 2.5})
        # 上限(5)を超える値は従来どおり不正
        Path(self.path).write_text('{"speech_volume": 5.5}', encoding="utf-8")
        self.assertTrue(self.new().load_error)

    def test_save_failure_still_applies_and_warns(self):
        lv = vb.VoiceLevels(os.path.join(TMP, "no_such_dir_file.txt", "sub", "levels.json"), env={})
        # 親が通常ファイルなのでmakedirsが失敗する
        Path(os.path.join(TMP, "no_such_dir_file.txt")).write_text("x", encoding="utf-8")
        out = lv.update({"speech_volume": 1.4})
        self.assertEqual(lv.speech_volume, 1.4)
        self.assertFalse(out["persisted"])
        self.assertIn("warning", out)

    def test_validate_ranges_types_and_duck_rule(self):
        cur = dict(vb.DEFAULT_LEVELS)
        ok, err = vb.validate_levels_update({"speech_volume": 5, "bgm_volume": 5, "bgm_duck_volume": 5}, cur)
        self.assertEqual(err, [])                                  # 上限は発話もBGMも5
        self.assertEqual(ok, {"speech_volume": 5.0, "bgm_volume": 5.0, "bgm_duck_volume": 5.0})
        ok, err = vb.validate_levels_update({"speech_volume": 0}, cur)
        self.assertEqual((ok, err), ({"speech_volume": 0.0}, []))
        bad = [None, [], "x", 1, {"speech_volume": 5.01}, {"speech_volume": -0.1}, {"bgm_volume": 5.01},
               {"bgm_duck_volume": 5.5}, {"speech_volume": "1.5"}, {"speech_volume": True},
               {"speech_volume": float("nan")}, {"speech_volume": float("inf")}, {"loud": 1},
               {"bgm_volume": [0.5]}, {"bgm_volume": 0.2, "bgm_duck_volume": 0.3}]
        for b in bad:
            ok, err = vb.validate_levels_update(b, cur)
            self.assertTrue(err, b)
            self.assertEqual(ok, {}, b)

    def test_duck_vs_current_value_in_partial_updates(self):
        cur = {"speech_volume": 1.0, "bgm_volume": 0.5, "bgm_duck_volume": 0.3}
        self.assertTrue(vb.validate_levels_update({"bgm_duck_volume": 0.6}, cur)[1])    # 現在の通常音量0.5を超える
        self.assertTrue(vb.validate_levels_update({"bgm_volume": 0.2}, cur)[1])         # 現在のダッキング0.3を下回る
        self.assertFalse(vb.validate_levels_update({"bgm_volume": 0.2, "bgm_duck_volume": 0.1}, cur)[1])
        self.assertFalse(vb.validate_levels_update({"bgm_duck_volume": 0.5}, cur)[1])   # 等しいのはOK
        # nullで既定(ダッキング0.12)に戻すときも、通常音量以下であること
        low = {"speech_volume": 1.0, "bgm_volume": 0.1, "bgm_duck_volume": 0.05}
        self.assertTrue(vb.validate_levels_update({"bgm_duck_volume": None}, low)[1])

    def test_partial_update_and_null_reset(self):
        lv = self.new()
        lv.update({"speech_volume": 2.0})
        self.assertEqual(lv.bgm_volume, vb.DEFAULT_LEVELS["bgm_volume"])
        lv.update({"speech_volume": None, "bgm_volume": 0.9})
        self.assertEqual(lv.speech_volume, vb.DEFAULT_LEVELS["speech_volume"])
        self.assertEqual(lv.bgm_volume, 0.9)

    def test_listener_called_and_failure_isolated(self):
        lv = self.new()
        seen = []
        lv.add_listener(lambda v: (_ for _ in ()).throw(RuntimeError("boom")))
        lv.add_listener(seen.append)
        lv.update({"bgm_volume": 0.8})
        self.assertEqual(seen[0]["bgm_volume"], 0.8)

    def test_bgm_compat_clamps_and_keeps_duck_below_volume(self):
        lv = self.new()
        lv.set_bgm_compat(volume=9, duck_volume=-1)
        self.assertEqual((lv.bgm_volume, lv.bgm_duck_volume), (5.0, 0.0))
        lv.set_bgm_compat(volume=0.1)                     # 通常を下げるとダッキングも揃う
        lv.update({"bgm_duck_volume": 0.1})
        lv.set_bgm_compat(duck_volume=0.9)                # 通常音量を超える指定は通常音量に揃う
        self.assertEqual(lv.bgm_duck_volume, lv.bgm_volume)
        for bad in (float("nan"), float("inf"), "x", [1]):
            with self.assertRaises((ValueError, TypeError)):
                lv.set_bgm_compat(volume=bad)
        self.assertEqual(self.new().bgm_volume, 0.1)      # /bgm経由の変更も保存されている


def _speechlike(sr=24000, sec=4.0, rms_db=-26.0):
    """発話っぽい合成信号(基本周波数のゆらぎ+倍音+音節の強弱)。実発話wav(RMS約-26dBFS・ピーク-10〜-5.6dBFS)に近づけてある。"""
    t = np.arange(int(sr * sec)) / sr
    ph = 2 * np.pi * np.cumsum(120 + 25 * np.sin(2 * np.pi * 0.7 * t)) / sr
    x = sum(np.sin(h * ph) / h ** 0.9 for h in range(1, 30))
    x = x * np.clip(np.sin(2 * np.pi * 3.3 * t) ** 2 * (0.6 + 0.4 * np.sin(2 * np.pi * 0.5 * t)), 0, None)
    return (x / np.sqrt(np.mean(x ** 2)) * 10 ** (rms_db / 20)).astype("float32")


def _rms_db(x):
    return 20 * np.log10(max(float(np.sqrt(np.mean(np.square(x, dtype="float64")))), 1e-12))


def _thd_percent(y, f, sr):
    spec = np.abs(np.fft.rfft(y * np.hanning(len(y))))
    fr = np.fft.rfftfreq(len(y), 1 / sr)

    def band(f0):
        i = int(np.argmin(abs(fr - f0)))
        return spec[max(i - 3, 0):i + 4].max()
    return 100 * np.sqrt(sum(band(f * k) ** 2 for k in range(2, 9))) / band(f)


@unittest.skipIf(vb is None, "voice_bridgeを読み込めません(requests/numpy未導入?)")
class GainTest(unittest.TestCase):
    def test_gain_one_is_identity(self):
        x = np.array([0.0, 0.3, -0.9, 1.0], dtype="float32")
        np.testing.assert_array_equal(vb.apply_speech_gain(x, 1.0), x)

    def test_quiet_signal_is_plain_multiplication(self):
        sr = 24000
        x = (0.1 * np.sin(2 * np.pi * 300 * np.arange(sr) / sr)).astype("float32")
        # 倍率<1は頭のフェードも無く、そのまま掛け算(波形を変えない)
        np.testing.assert_allclose(vb.apply_speech_gain(x, 0.5, sr), x * 0.5, rtol=1e-6)
        # 倍率>1でも天井(0.65)に届かない小さな信号は、頭のフェードインの後は単なる掛け算
        nf = int(vb.SPEECH_FADE_IN_SEC * sr)
        y = vb.apply_speech_gain(x, 2.0, sr)
        np.testing.assert_allclose(y[nf:], x[nf:] * 2.0, rtol=1e-6)

    def test_zero_gain_is_silence(self):
        self.assertEqual(float(np.abs(vb.apply_speech_gain(np.array([0.5, -1.0]), 0.0)).max()), 0.0)

    def test_soft_start_fade_in_only_when_boosting(self):
        sr = 24000
        x = np.full(sr, 0.1, dtype="float32")
        y = vb.apply_speech_gain(x, 3.0, sr)
        self.assertAlmostEqual(float(y[0]), 0.0, places=6)                  # 頭は0から
        nf = int(vb.SPEECH_FADE_IN_SEC * sr)
        self.assertTrue(np.all(np.diff(y[:nf]) >= -1e-9))                   # 単調に立ち上がる
        self.assertAlmostEqual(float(y[nf + 5]), 0.3, places=5)             # その後は倍率どおり
        self.assertAlmostEqual(float(vb.apply_speech_gain(x, 0.5, sr)[0]), 0.05, places=6)   # 下げる側は即座

    def test_output_never_reaches_ceiling_for_all_gains(self):
        sr = 24000
        sig = {"speech": _speechlike(sr), "full_scale_sine": np.sin(2 * np.pi * 200 * np.arange(sr) / sr).astype("float32"),
               "impulses": np.where(np.arange(sr) % 3000 == 0, 1.0, 0.02).astype("float32")}
        for name, x in sig.items():
            for g in (1.1, 1.8, 2.5, 3.0, 4.0, 5.0):
                y = vb.apply_speech_gain(x, g, sr)
                self.assertEqual(y.shape, x.shape)
                self.assertLessEqual(float(np.abs(y).max()), vb.SPEECH_CEILING + 1e-6, (name, g))
                self.assertLess(float(np.abs(y).max()), vb.MASTER_CEILING, (name, g))
                self.assertTrue(np.all(np.isfinite(y)))
        # ステレオ・リスト入力・空
        st = vb.apply_speech_gain(np.full((6000, 2), 0.9, dtype="float32"), 5.0, sr)
        self.assertEqual(st.shape, (6000, 2))
        self.assertLessEqual(float(np.abs(st).max()), vb.SPEECH_CEILING + 1e-6)
        self.assertEqual(vb.apply_speech_gain([0.1, 0.2], 2.0).dtype, np.float32)
        self.assertEqual(len(vb.apply_speech_gain(np.zeros(0, dtype="float32"), 3.0)), 0)

    def test_limit_peaks_passes_through_when_under_ceiling(self):
        x = _speechlike() * 1.0
        peak = float(np.abs(x).max())
        np.testing.assert_array_equal(vb.limit_peaks(x, peak + 0.01), x)       # 超えなければ1ビットも変えない

    def test_rms_rises_with_gain_but_with_diminishing_returns(self):
        # 数値実験(docs/voice_levels.md): 発話風の合成信号でリミッタ後のRMSがどれだけ上がるか
        sr = 24000
        x = _speechlike(sr)
        base = _rms_db(x)
        rise = {g: _rms_db(vb.apply_speech_gain(x, g, sr)) - base for g in (1.8, 2.5, 3.0, 4.0, 5.0)}
        gains = sorted(rise)
        for lo, hi in zip(gains, gains[1:]):
            self.assertGreaterEqual(rise[hi], rise[lo] - 0.05)                 # 倍率を上げて小さくはならない
        self.assertGreater(rise[2.5], 7.0)         # 2.5倍(+8dB)ほぼそのまま
        self.assertGreater(rise[5.0], 9.5)         # 5倍(+14dB)でもリミッタ後は約+11dB
        self.assertLess(rise[5.0], 14.0)           # 公称の+14dBには届かない(リミッタで頭打ち)
        self.assertGreater(rise[5.0] - rise[2.5], 1.5)   # それでも2.5倍より確実に大きい

    def test_sine_distortion_far_below_hard_clip(self):
        sr = 24000
        t = np.arange(sr * 2) / sr
        s = (0.25 * np.sin(2 * np.pi * 1000 * t)).astype("float32")
        for g in (3, 4, 5):
            y = vb.apply_speech_gain(s, g, sr)[int(0.1 * sr):]
            clipped = np.clip(s * g, -vb.SPEECH_CEILING, vb.SPEECH_CEILING)[int(0.1 * sr):]
            self.assertLess(_thd_percent(y, 1000, sr), 1.0, g)                 # リミッタ: ほぼ純音のまま
            self.assertGreater(_thd_percent(clipped, 1000, sr), 5.0, g)        # 単純クリップは割れる
            self.assertGreater(float(np.abs(y).max()), vb.SPEECH_CEILING - 0.02)   # 天井ぎりぎりまで使っている

    def test_far_samples_from_a_peak_are_untouched(self):
        sr = 24000
        x = np.full(sr, 0.1, dtype="float32")
        x[sr // 2] = 0.9                                                       # 1点だけ大きなピーク
        y = vb.apply_speech_gain(x, 2.0, sr)
        n = vb._attack_samples(sr)
        i = sr // 2
        np.testing.assert_allclose(y[i + 5 * n:], x[i + 5 * n:] * 2.0, rtol=1e-6)   # 数ms離れれば元の倍率
        np.testing.assert_allclose(y[int(0.06 * sr):i - 5 * n], x[int(0.06 * sr):i - 5 * n] * 2.0, rtol=1e-6)
        self.assertLessEqual(float(np.abs(y).max()), vb.SPEECH_CEILING + 1e-6)

    def test_lookahead_limiter_stream_is_block_size_independent_and_bounded(self):
        sr = 44100
        rng = np.random.default_rng(3)
        x = (rng.standard_normal((sr, 2)) * 0.4 + np.sin(2 * np.pi * 80 * np.arange(sr) / sr)[:, None] * 0.6).astype("float32")
        ceil = np.linspace(0.94, 0.29, len(x))
        one = vb.LookaheadLimiter(sr).process(x, ceil)
        lim = vb.LookaheadLimiter(sr)
        parts, pos, sizes, i = [], 0, (1024, 333, 1, 4096, 1000), 0
        while pos < len(x):
            k = sizes[i % len(sizes)]
            parts.append(lim.process(x[pos:pos + k], ceil[pos:pos + k]))
            pos += k
            i += 1
        many = np.concatenate(parts)
        np.testing.assert_allclose(many, one, atol=1e-6)                       # ブロックの切り方で結果が変わらない
        dd = 2 * vb._attack_samples(sr)                                        # 出力は入力より2*アタックぶん遅れる
        self.assertTrue(np.all(np.abs(one[dd:]) <= ceil[:-dd, None] + 1e-6))   # 各サンプルの(入力時点の)天井以下
        # 天井以下の信号は遅延だけで素通し
        q = (0.3 * np.sin(2 * np.pi * 440 * np.arange(sr) / sr)).astype("float32")[:, None].repeat(2, 1)
        d = 2 * vb._attack_samples(sr)
        out = vb.LookaheadLimiter(sr).process(q, 0.94)
        np.testing.assert_allclose(out[d:], q[:-d], atol=1e-7)
        self.assertEqual(float(np.abs(out[:d]).max()), 0.0)

    def test_master_budget_speech_plus_ducked_bgm_stays_below_ceiling(self):
        # 発話(5倍)と発話中のBGM(ダッキング音量も5)を、ピークが重なる最悪の組み合わせで足しても0.95未満
        self.assertLess(vb.SPEECH_CEILING + vb.BGM_DUCK_CEILING, vb.MASTER_CEILING)
        self.assertLess(vb.BGM_CEILING, vb.MASTER_CEILING)
        sr_s, sr_b = 24000, 44100
        sp = vb.apply_speech_gain(_speechlike(sr_s, 3.0), 5.0, sr_s)
        g = vb.BgmGain(volume=5.0, duck_volume=5.0)
        g.duck_begin()                                                         # 発話中=ダッキング中
        ceil = g.ceiling_for(g.duck_volume)
        self.assertAlmostEqual(ceil, vb.BGM_DUCK_CEILING)
        rng = np.random.default_rng(5)
        bgm_in = (np.clip(rng.standard_normal((sr_b * 3, 2)) * 0.6, -1, 1) * 5.0).astype("float32")
        bgm = vb.LookaheadLimiter(sr_b).process(bgm_in, ceil)
        worst = float(np.abs(sp).max()) + float(np.abs(bgm).max())
        self.assertLess(worst, vb.MASTER_CEILING)
        self.assertGreater(float(np.abs(bgm).max()), vb.BGM_DUCK_CEILING - 0.02)   # BGMも予算いっぱいまで使う

    def test_bgm_ceiling_follows_ducking_and_clamps_to_five(self):
        g = vb.BgmGain(volume=99, duck_volume=99)
        self.assertEqual((g.volume, g.duck_volume), (5.0, 5.0))                # 上限5
        g = vb.BgmGain(volume=3.0, duck_volume=0.3, clock=lambda: 100.0)
        self.assertAlmostEqual(g.ceiling_for(3.0), vb.BGM_CEILING)             # 通常
        self.assertAlmostEqual(g.ceiling_for(0.3), vb.BGM_DUCK_CEILING)        # 発話中
        mid = g.ceiling_for(1.65)
        self.assertTrue(vb.BGM_DUCK_CEILING < mid < vb.BGM_CEILING)            # ゲインの移動に合わせて滑らか
        self.assertAlmostEqual(g.ceiling_for(0.0), vb.BGM_DUCK_CEILING)
        self.assertAlmostEqual(g.ceiling_for(9.0), vb.BGM_CEILING)
        # ダッキングしない設定(通常=発話中)は状態で決める
        n = vb.BgmGain(volume=2.0, duck_volume=2.0, clock=lambda: 100.0)
        self.assertAlmostEqual(n.ceiling_for(2.0), vb.BGM_CEILING)
        n.duck_begin()
        self.assertAlmostEqual(n.ceiling_for(2.0), vb.BGM_DUCK_CEILING)

    def test_bgm_gain_change_is_smoothed(self):
        # 音量を上げても、BGMのゲインは一気に跳ねず(up_sec=1秒かけて)目標へ近づく
        t = [0.0]
        g = vb.BgmGain(volume=1.0, duck_volume=0.15, up_sec=1.0, clock=lambda: t[0])
        for _ in range(100):
            g.advance(0.05)                                                    # 通常音量1.0まで上げ切る
        self.assertAlmostEqual(g.gain, 1.0, places=6)
        g.volume = 5.0
        g0, g1 = g.advance(0.1)
        self.assertAlmostEqual(g0, 1.0)
        self.assertLess(g1, 1.0 + 0.1 * 4.85 + 1e-6)                            # 0.1秒で5までは行かない(約1.5)
        self.assertGreater(g1, g0)
        for _ in range(30):
            g.advance(0.1)
        self.assertAlmostEqual(g.gain, 5.0, places=6)

    def test_constants_for_five_x(self):
        self.assertEqual((vb.SPEECH_VOLUME_MAX, vb.BGM_VOLUME_MAX), (5.0, 5.0))
        self.assertEqual(vb.LEVEL_MAX, {"speech_volume": 5.0, "bgm_volume": 5.0, "bgm_duck_volume": 5.0, "announce_volume": 5.0})
        self.assertEqual(vb.LEVEL_CONFIRM_ABOVE, 2.0)
        d = vb.DEFAULT_LEVELS                                                  # 数値実験に基づく既定(前日の 1.8 / 0.45 / 0.12 より上)
        self.assertEqual((d["speech_volume"], d["bgm_volume"], d["bgm_duck_volume"]), (2.5, 1.0, 0.15))
        # 既定の発話中、BGMは発話より約10dB下(RMSの差で8〜13dB)
        sr = 24000
        x = _speechlike(sr)
        sp = _rms_db(vb.apply_speech_gain(x, d["speech_volume"], sr))
        bgm_ducked = -11.5 + 20 * np.log10(d["bgm_duck_volume"])               # BGMのmp3はRMS約-11.5dBFS
        self.assertTrue(8.0 <= sp - bgm_ducked <= 13.0, sp - bgm_ducked)


# ======================================================================
# 2. voice_bridge HTTP(+発話への適用)
# ======================================================================
@unittest.skipIf(vb is None, "voice_bridgeを読み込めません(requests/numpy未導入?)")
class BridgeHttpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv, cls.port = serve(vb.make_handler())
        cls.url = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        vb.voice_levels.update({"speech_volume": None, "bgm_volume": None, "bgm_duck_volume": None})
        vb.voice_master.set_enabled(True)

    def tearDown(self):
        vb.voice_master.set_enabled(True)

    def post(self, path, body=None, raw=None):
        return http("POST", self.url + path, body=body, raw=raw, headers={"Content-Type": "application/json"})

    def test_get_status_shape(self):
        code, d = http("GET", self.url + "/voice_levels")
        self.assertEqual(code, 200)
        for k in ("speech_volume", "bgm_volume", "bgm_duck_volume", "defaults", "limits", "persisted", "load_error"):
            self.assertIn(k, d)
        code, st = http("GET", self.url + "/status")
        self.assertEqual((st["speech_volume"], st["bgm_volume"], st["bgm_duck_volume"]),
                         (d["speech_volume"], d["bgm_volume"], d["bgm_duck_volume"]))

    def test_post_applies_to_bgm_and_persists(self):
        code, d = self.post("/voice_levels", {"speech_volume": 2.2, "bgm_volume": 0.6, "bgm_duck_volume": 0.2})
        self.assertEqual((code, d["ok"], d["speech_volume"]), (200, True, 2.2))
        self.assertEqual((vb.bgm.gain.volume, vb.bgm.gain.duck_volume), (0.6, 0.2))
        _, b = http("GET", self.url + "/bgm")
        self.assertEqual((b["volume"], b["duck_volume"]), (0.6, 0.2))
        doc = json.loads(Path(vb.VOICE_LEVELS_FILE).read_text(encoding="utf-8"))
        self.assertEqual(doc["speech_volume"], 2.2)

    def test_invalid_requests_are_400_and_change_nothing(self):
        before = vb.voice_levels.values()
        cases = [{}, {"speech_volume": 5.01}, {"speech_volume": "2"}, {"bgm_volume": 5.5}, {"bgm_duck_volume": -1},
                 {"volume": 0.5}, {"bgm_volume": 0.1, "bgm_duck_volume": 0.5}, {"speech_volume": True}]
        for c in cases:
            code, d = self.post("/voice_levels", c)
            self.assertEqual(code, 400, c)
            self.assertFalse(d["ok"])
            self.assertTrue(d["errors"] and d["message"])
        self.assertEqual(self.post("/voice_levels", raw=b"{bad")[0], 400)
        self.assertEqual(self.post("/voice_levels", raw=b'{"speech_volume": NaN}')[0], 400)   # NaNは構文の時点で拒否
        self.assertEqual(self.post("/voice_levels", raw=b"[1]")[0], 400)
        self.assertEqual(vb.voice_levels.values(), before)

    def test_five_is_accepted_and_reaches_bgm(self):
        code, d = self.post("/voice_levels", {"speech_volume": 5, "bgm_volume": 5, "bgm_duck_volume": 5})
        self.assertEqual((code, d["speech_volume"], d["bgm_volume"], d["bgm_duck_volume"]), (200, 5.0, 5.0, 5.0))
        self.assertEqual((vb.bgm.gain.volume, vb.bgm.gain.duck_volume), (5.0, 5.0))
        self.assertEqual(d["limits"]["speech_volume"], [0.0, 5.0])
        self.assertEqual(d["limits"]["bgm_volume"], [0.0, 5.0])
        for bad in ({"speech_volume": 5.01}, {"bgm_volume": 5.01}, {"bgm_duck_volume": 5.01}):
            self.assertEqual(self.post("/voice_levels", bad)[0], 400, bad)
        self.assertEqual(vb.voice_levels.speech_volume, 5.0)                   # 400では何も変わらない

    def test_bgm_endpoint_is_the_same_value(self):
        code, d = self.post("/bgm", {"volume": 0.8, "duck_volume": 0.3})
        self.assertEqual(code, 200)
        _, lv = http("GET", self.url + "/voice_levels")
        self.assertEqual((lv["bgm_volume"], lv["bgm_duck_volume"]), (0.8, 0.3))
        self.assertEqual(self.post("/bgm", {"volume": "x"})[0], 400)
        code, d = self.post("/bgm", {"duck_volume": 0.95})            # 通常音量0.8を超える分は揃えられる
        self.assertEqual(http("GET", self.url + "/voice_levels")[1]["bgm_duck_volume"], 0.8)

    def test_independent_of_voice_master(self):
        vb.voice_master.set_enabled(False)
        code, d = self.post("/voice_levels", {"speech_volume": 1.1, "bgm_volume": 0.5})   # OFF中でも保存できる
        self.assertEqual((code, d["speech_volume"]), (200, 1.1))
        self.assertFalse(vb.voice_master.enabled)                                           # 設定がマスターを変えない
        self.assertEqual(vb.voice_levels.bgm_volume, 0.5)
        self.assertEqual(vb.bgm.gain.target(), 0.0)                                         # OFFの間は無音のまま
        vb.voice_master.set_enabled(True)
        self.assertEqual(vb.voice_levels.speech_volume, 1.1)                                # ONに戻せば設定どおり
        vb.bgm.gain.enabled = True
        try:
            self.assertEqual(vb.bgm.gain.target(), 0.5)
        finally:
            vb.bgm.gain.enabled = False
        # 逆: マスターの切替で音量設定は変わらない
        vb.voice_master.set_enabled(False)
        vb.voice_master.set_enabled(True)
        self.assertEqual(vb.voice_levels.speech_volume, 1.1)

    def _speak_and_capture(self, samples):
        class Res:
            content = b"RIFF"

            def raise_for_status(self):
                pass

            def json(self):
                return {}
        played = []
        with mock.patch.object(vb.requests, "post", return_value=Res()), \
                mock.patch.object(vb.sf, "read", return_value=(samples.copy(), 24000)), \
                mock.patch.object(vb, "_play_audio", side_effect=lambda d, sr: played.append(np.array(d))):
            vb.speak_voicevox("テスト", {})
        return played

    def test_speech_gain_is_applied_before_playback(self):
        x = (0.1 * np.sin(np.arange(24000) / 7.0)).astype("float32")           # 24kHzで1秒(頭のフェードイン50msより十分長い)
        nf = int(vb.SPEECH_FADE_IN_SEC * 24000)
        self.post("/voice_levels", {"speech_volume": 2.0})
        played = self._speak_and_capture(x)
        np.testing.assert_allclose(played[0][nf:], x[nf:] * 2.0, rtol=1e-6)
        self.post("/voice_levels", {"speech_volume": 0.5})                    # 変更は次の発話から
        np.testing.assert_allclose(self._speak_and_capture(x)[0], x * 0.5, rtol=1e-6)
        code, _ = self.post("/voice_levels", {"speech_volume": 5.0})          # 上限の5倍も受け付ける
        self.assertEqual(code, 200)
        loud = np.tile(np.array([0.9, -1.0, 0.5], dtype="float32"), 8000)
        out = self._speak_and_capture(loud)[0]
        self.assertLess(float(np.abs(out).max()), vb.MASTER_CEILING)          # 割れない
        self.assertLessEqual(float(np.abs(out).max()), vb.SPEECH_CEILING + 1e-6)

    def test_speech_gain_applied_even_if_levels_changed_while_master_off(self):
        vb.voice_master.set_enabled(False)
        self.post("/voice_levels", {"speech_volume": 1.5})
        vb.voice_master.set_enabled(True)
        x = np.full(24000, 0.2, dtype="float32")
        nf = int(vb.SPEECH_FADE_IN_SEC * 24000)
        np.testing.assert_allclose(self._speak_and_capture(x)[0][nf:], x[nf:] * 1.5, rtol=1e-6)


# ======================================================================
# 3. proxy + dashboard
# ======================================================================
def _good_state(**kw):
    s = {"ok": True, "speech_volume": 1.8, "bgm_volume": 0.45, "bgm_duck_volume": 0.12,
         "persisted": True, "load_error": ""}
    s.update(kw)
    return s


class _Bridge(BaseHTTPRequestHandler):
    state = _good_state()
    posts = []
    fail = 0
    fail_body = {}
    garbage = False

    def log_message(self, *a):
        pass

    def _send(self, code, obj):
        body = obj if isinstance(obj, bytes) else json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.fail:
            self._send(self.fail, type(self).fail_body)
        elif self.garbage:
            self._send(200, b"not json")
        elif self.path == "/voice_levels":
            self._send(200, type(self).state)
        else:
            self._send(404, {})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0"))
        data = json.loads(self.rfile.read(n))
        type(self).posts.append(data)
        if self.fail:
            self._send(self.fail, type(self).fail_body)
            return
        new = dict(type(self).state)
        for k, v in data.items():
            new[k] = v if v is not None else vb_defaults[k]
        type(self).state = new
        self._send(200, new)


vb_defaults = {"speech_volume": 1.8, "bgm_volume": 0.45, "bgm_duck_volume": 0.12}


def _reset_bridge():
    _Bridge.state = _good_state()
    _Bridge.posts = []
    _Bridge.fail = 0
    _Bridge.fail_body = {}
    _Bridge.garbage = False


class ProxyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.srv, cls.port = serve(_Bridge)
        cls.url = f"http://127.0.0.1:{cls.port}"

    @classmethod
    def tearDownClass(cls):
        cls.srv.shutdown()
        cls.srv.server_close()

    def setUp(self):
        _reset_bridge()
        self.p = VoiceLevelsProxy(self.url)

    def test_limits_match_voice_bridge(self):
        if vb is None:
            self.skipTest("voice_bridgeを読み込めません")
        for k, (_label, hi) in LEVEL_LIMITS.items():
            self.assertEqual(hi, vb.LEVEL_MAX[k], k)

    def test_five_is_the_new_upper_limit(self):
        for k, (_label, hi) in LEVEL_LIMITS.items():
            self.assertEqual(hi, 5.0, k)
        code, r = self.p.set({"speech_volume": 5, "bgm_volume": 5, "bgm_duck_volume": 5})
        self.assertEqual(code, 200)
        self.assertEqual(_Bridge.posts[-1], {"speech_volume": 5.0, "bgm_volume": 5.0, "bgm_duck_volume": 5.0})
        self.assertEqual(self.p.set({"speech_volume": 5.0001})[0], 400)
        self.assertEqual(self.p.set({"bgm_volume": 6})[0], 400)
        self.assertEqual(self.p.set({"bgm_duck_volume": 5.5})[0], 400)

    def test_get_and_set(self):
        r = self.p.get_status()
        self.assertTrue(r["ok"] and r["configured"])
        self.assertEqual(r["status"]["speech_volume"], 1.8)
        code, r = self.p.set({"speech_volume": 2.5, "bgm_volume": 0.5})
        self.assertEqual((code, r["ok"], r["status"]["speech_volume"]), (200, True, 2.5))
        self.assertEqual(_Bridge.posts, [{"speech_volume": 2.5, "bgm_volume": 0.5}])

    def test_null_resets_and_ints_are_floats(self):
        code, r = self.p.set({"speech_volume": None, "bgm_volume": 1})
        self.assertEqual(code, 200)
        self.assertEqual(_Bridge.posts, [{"speech_volume": None, "bgm_volume": 1.0}])

    def test_validation_never_forwards(self):
        bad = [None, [], "x", 1, {}, {"speech_volume": "2"}, {"speech_volume": True}, {"speech_volume": 5.1},
               {"speech_volume": -0.5}, {"bgm_volume": 5.1}, {"bgm_duck_volume": 5.5}, {"volume": 0.5},
               {"bgm_volume": 0.2, "bgm_duck_volume": 0.3}, {"speech_volume": float("nan")},
               {"speech_volume": float("inf")}, {"enabled": True}]
        for b in bad:
            code, r = self.p.set(b)
            self.assertEqual(code, 400, b)
            self.assertFalse(r["ok"])
            self.assertTrue(r["message"])
        self.assertEqual(_Bridge.posts, [])

    def test_unreachable_is_error_json(self):
        dead = VoiceLevelsProxy("http://127.0.0.1:1", timeout_s=0.5)
        r = dead.get_status()
        self.assertFalse(r["ok"])
        self.assertIn("接続できません", r["message"])
        code, r = dead.set({"speech_volume": 2})
        self.assertEqual((code, r["ok"]), (200, False))

    def test_http_error_garbage_and_old_bridge(self):
        _Bridge.fail = 500
        self.assertIn("HTTP 500", self.p.get_status()["message"])
        _Bridge.fail = 404
        self.assertIn("未対応", self.p.get_status()["message"])
        _Bridge.fail = 400
        _Bridge.fail_body = {"ok": False, "message": "BGM発話中音量は通常音量以下にしてください"}
        code, r = self.p.set({"bgm_duck_volume": 0.3})
        self.assertIn("通常音量以下", r["message"])            # 2台目の400理由をそのまま表示に使う
        _Bridge.fail = 0
        _Bridge.garbage = True
        self.assertFalse(self.p.get_status()["ok"])
        _Bridge.garbage = False
        _Bridge.state = {"ok": True}                           # 数値が無い応答(古いbridgeや想定外)も不達扱い
        self.assertFalse(self.p.get_status()["ok"])

    def test_unconfigured(self):
        p = VoiceLevelsProxy("")
        self.assertFalse(p.get_status()["configured"])
        code, r = p.set({"speech_volume": 2})
        self.assertFalse(r["configured"])
        self.assertEqual(_Bridge.posts, [])

    def test_save_warning_passes_through(self):
        _Bridge.state = _good_state(warning="音量を保存できませんでした")
        code, r = self.p.set({"speech_volume": 2})
        self.assertTrue(r["ok"])
        self.assertIn("保存できません", r["message"])


class FakeTeleop:
    soft_estop_engaged = False


class DashboardApiTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge, cls.bport = serve(_Bridge)

    @classmethod
    def tearDownClass(cls):
        cls.bridge.shutdown()
        cls.bridge.server_close()

    def setUp(self):
        _reset_bridge()
        self.proxy = VoiceLevelsProxy(f"http://127.0.0.1:{self.bport}")
        handler = dc.make_dashboard_handler(
            lambda: b"<html></html>", lambda: {}, FakeTeleop(), voice_levels=self.proxy,
            auth_username="TL", auth_password="pw", enforce_single_controller=False)
        self.srv, self.port = serve(handler)
        self.tl = self.login("TL", "pw")
        self.guest = self.login("guest", "")

    def tearDown(self):
        self.srv.shutdown()
        self.srv.server_close()

    def login(self, user, pw):
        req = urllib.request.Request(
            f"http://127.0.0.1:{self.port}/api/login", data=json.dumps({"username": user, "password": pw}).encode(),
            method="POST", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.headers.get("Set-Cookie", "").split(";")[0]

    def call(self, method, path, body=None, cookie=None):
        h = {"Content-Type": "application/json"}
        c = self.tl if cookie is None else cookie
        if c:
            h["Cookie"] = c
        return http(method, f"http://127.0.0.1:{self.port}{path}", body=body, headers=h)

    def test_requires_login(self):
        self.assertEqual(self.call("GET", "/api/voice_levels", cookie="")[0], 401)
        self.assertEqual(self.call("POST", "/api/voice_levels", {"speech_volume": 2}, cookie="")[0], 401)
        self.assertEqual(_Bridge.posts, [])

    def test_guest_can_view_but_not_change(self):
        for ck in (self.tl, self.guest):
            code, d = self.call("GET", "/api/voice_levels", cookie=ck)
            self.assertEqual((code, d["ok"], d["status"]["speech_volume"]), (200, True, 1.8))
        self.assertEqual(self.call("POST", "/api/voice_levels", {"speech_volume": 2}, cookie=self.guest)[0], 403)
        self.assertEqual(_Bridge.posts, [])

    def test_tl_change_and_validation(self):
        code, d = self.call("POST", "/api/voice_levels", {"speech_volume": 2.4, "bgm_duck_volume": 0.1})
        self.assertEqual((code, d["ok"], d["status"]["speech_volume"]), (200, True, 2.4))
        self.assertEqual(self.call("POST", "/api/voice_levels", {"speech_volume": "2"})[0], 400)
        self.assertEqual(self.call("POST", "/api/voice_levels", {"speech_volume": 9})[0], 400)
        self.assertEqual(self.call("POST", "/api/voice_levels", {})[0], 400)
        self.assertEqual(self.call("POST", "/api/voice_levels", {"bgm_volume": 0.1, "bgm_duck_volume": 0.5})[0], 400)
        self.assertEqual(_Bridge.posts, [{"speech_volume": 2.4, "bgm_duck_volume": 0.1}])
        self.assertEqual(self.call("GET", "/api/voice_levels")[1]["status"]["speech_volume"], 2.4)

    def test_bridge_down_is_error_json(self):
        self.proxy.base_url = "http://127.0.0.1:1"
        self.proxy.timeout_s = 0.5
        code, d = self.call("GET", "/api/voice_levels")
        self.assertEqual((code, d["ok"]), (200, False))
        self.assertIn("接続できません", d["message"])
        code, d = self.call("POST", "/api/voice_levels", {"speech_volume": 2})
        self.assertEqual((code, d["ok"]), (200, False))

    def test_unconfigured_hides(self):
        self.proxy.base_url = ""
        self.assertFalse(self.call("GET", "/api/voice_levels")[1]["configured"])


# ======================================================================
# 4. 画面
# ======================================================================
class PageTest(unittest.TestCase):
    def setUp(self):
        src = (PKG / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8")
        self.html = src[:src.index("MOBILE_HTML = ")]

    def test_card_in_voice_tab_before_dock(self):
        h = self.html
        i_tab = h.index('id="tab-voice"')
        i_card = h.index('id="voiceLevelsBlock"')
        self.assertGreater(i_card, i_tab)
        self.assertLess(i_card, h.index('id="tab-logs"'))
        self.assertLess(i_card, h.index('id="estopDock"'))
        for el in ("vlNow", "vlSpeech", "vlSpeechNum", "vlBgm", "vlBgmNum", "vlDuck", "vlDuckNum",
                   "vlApplyBtn", "vlTestBtn", "vlResetBtn", "vlHint", "vlCaution", "vlConfirmBar", "vlConfirmText",
                   "vlYes", "vlNo", "vlTicks"):
            self.assertEqual(h.count(f'id="{el}"'), 1, el)
        self.assertIn("/api/voice_levels", h)
        self.assertIn('id="estopDock"', h)
        self.assertIn('id="voiceMasterBlock"', h)        # 既存の全体ON/OFFカードも残っている
        self.assertIn('id="bgmBlock"', h)

    def test_card_five_x_range_caution_and_confirm_bar(self):
        h = self.html
        card = h[h.index('id="voiceLevelsBlock"'):h.index('id="bgmBlock"')]
        for sid in ("vlSpeech", "vlSpeechNum", "vlBgm", "vlBgmNum", "vlDuck", "vlDuckNum"):
            m = re.search(r'<input id="%s"[^>]*>' % sid, card)
            self.assertIsNotNone(m, sid)
            self.assertIn('max="5"', m.group(0), sid)                           # スライダーも数値入力も最大5
            self.assertIn('min="0"', m.group(0), sid)
        for sid in ("vlSpeech", "vlBgm", "vlDuck"):
            self.assertIn('list="vlTicks"', re.search(r'<input id="%s"[^>]*>' % sid, card).group(0))   # 目盛り
        self.assertIn("音割れ", card)
        self.assertIn("スピーカーの故障", card)
        self.assertIn("人がいる場所で急に上げないでください", card)
        self.assertRegex(card, r'id="vlYes"[^>]*>実行する<')
        self.assertRegex(card, r'id="vlNo"[^>]*>やめる<')
        self.assertIn("既定に戻す", card)
        self.assertIn("const VL_MAX = 5, VL_CONFIRM_ABOVE = 2;", h)
        # 2倍超(今より上げる)のときは確認バーを出し、「実行する」まで送らない。既定に戻すは確認なし
        self.assertIn("req[f.key] > VL_CONFIRM_ABOVE", h)
        self.assertIn("vlPendingReq = req;", h)
        i_apply = h.index('vlEls.apply.addEventListener("click"')
        self.assertLess(h.index("vlPendingReq = req;", i_apply), h.index('await vlPost(req, "反映しました(保存済み)");', i_apply))

    def test_proxy_constants_feed_the_page(self):
        self.assertIn("const VL_MAX = %d, VL_CONFIRM_ABOVE = %d;" % (LEVEL_MAX_ALL, LEVEL_CONFIRM_ABOVE), self.html)
        if vb is not None:
            self.assertEqual((LEVEL_MAX_ALL, LEVEL_CONFIRM_ABOVE), (vb.SPEECH_VOLUME_MAX, vb.LEVEL_CONFIRM_ABOVE))

    def test_legacy_bgm_card_does_not_clobber_values_above_one(self):
        # 旧BGMカード(デスクトップ+スマホ8095)は0〜1のまま。触っていない欄は送らない(1超を1に丸めて下げない)
        src = (PKG / "nokosahen_core" / "full_dashboard_node.py").read_text(encoding="utf-8")
        self.assertEqual(src.count("Number(raw) === Number(el.dataset.orig)"), 2)
        self.assertEqual(src.count("bgmEls.vol.dataset.orig = String(s.volume)"), 2)

    def test_dock_untouched(self):
        nav = (PKG / "nokosahen_core" / "nav_layout.py").read_text(encoding="utf-8")
        self.assertIn("#estopDock { position:fixed;", nav)
        self.assertIn("z-index:2000", nav)

    def test_crlf_preserved_and_new_files_lf(self):
        for name in ("dashboard_common.py", "full_dashboard_node.py", "nav_layout.py"):
            b = (PKG / "nokosahen_core" / name).read_bytes()
            self.assertEqual(b.count(b"\r\n"), b.count(b"\n"), name)
        for name in ("voice_levels_proxy.py",):
            self.assertNotIn(b"\r", (PKG / "nokosahen_core" / name).read_bytes(), name)
        yaml = (REPO / "ros2_ws/src/nokosahen_bringup/config/nokosahen.yaml").read_text(encoding="utf-8")
        self.assertIn('voice_levels_url: "http://198.51.100.3:8100"', yaml)
        self.assertIn('bgm_url: "http://198.51.100.3:8100"', yaml)

    def test_js_syntax(self):
        node = shutil.which("node")
        if not node:
            self.skipTest("nodeが無い")
        scripts = []
        rest = self.html
        while "<script>" in rest:
            a = rest.index("<script>") + len("<script>")
            b = rest.index("</script>", a)
            scripts.append(rest[a:b])
            rest = rest[b:]
        js = [s for s in scripts if "loadVoiceLevels" in s]
        self.assertEqual(len(js), 1)
        p = Path(TMP) / "page.js"
        p.write_text(js[0].replace("__NAV_JS__", ""), encoding="utf-8")
        r = subprocess.run([node, "--check", str(p)], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)


def tearDownModule():
    shutil.rmtree(TMP, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
