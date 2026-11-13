#!/usr/bin/env python3
"""明電舎タブレット(POSレジの累計販売数集計画面)をWebカメラで撮影し、
Aランチ/Bランチ/Cランチ(タブレット画面の上から1〜3番目の行)の数字だけを
読み取って、のこさへんシステムへ送信する。

【設計方針】
- Raspberry Pi単体で動く(ROS 2不要)。のこさへん本体(Jetson)のweb_ui
  ノード(nokosahen_core/web_ui_node.py、2026-08-07追加)が公開している
  HTTP API(`/api/state`・`/api/sold_total`)にBasic認証でアクセスするだけ
  なので、このRaspberry PiにはROS 2をインストールする必要が無い。
- タブレット上の「Aランチ/Bランチ/Cランチ」は常に同じ画面レイアウトの
  上から1〜3番目に固定表示されている(2026-08-07時点でユーザーが現地で
  確認済み)。文字認識で行を探すのではなく、固定位置(ROI、画面比率で指定)
  を切り出して数字だけをOCRする方式にしている。一般的な物体検出・文字
  検出よりはるかに軽い処理で済み、Coral TPU等の高価なアクセラレータが
  無くてもRaspberry Pi単体(CPU)+Tesseract OCRで実用上十分な速度が出る
  想定(タブレットの数字は大きく・くっきりした画面表示文字であり、手書き
  文字や自然画像中の文字よりOCRとしてはかなり簡単な部類のため)。
- Coral Edge TPUによる数字認識に対応済み(2026-08-21追加、
  train_digit_model.py参照)。ただし学習データは実際のタブレット撮影画像
  ではなく、フォント・幾何変形・二値化ノイズをランダム化した合成データ
  なので、実機での精度は未検証。設定ファイルの"recognizer"を
  "edgetpu"にすると使われる(既定は引き続き"tesseract")。Edge TPU推論に
  失敗した場合は自動でTesseractへフォールバックする。
- のこさへん本体の「Aランチ/Bランチ/Cランチが今日は何という料理名か」の
  対応は、残数入力画面(初期食数)で1番目・2番目・3番目に入力された
  メニュー名がそのままA/B/Cに対応する運用にしている(web_ui_node.py側の
  UIにA/B/C表示を追加済み)。このスクリプトは`/api/state`から
  `menu_state.items[0]/[1]/[2]`を読み、その名前へ数字を紐付けて送信する。
- 誤読による誤送信を避けるため、同じ数字が連続で
  `stable_reads_required`回読めるまでは送信しない(タブレット画面の反射・
  遷移中のちらつき等による一時的な誤読を弾くため)。また前回送信した値と
  同じなら再送しない(無駄なログ・通信を避ける)。

使い方:
    python3 tablet_ocr_reader.py --config ~/.nokosahen_tablet_ocr_config.json
    python3 tablet_ocr_reader.py --calibrate   # ROI位置合わせ用の撮影のみ

事前準備(Raspberry Pi側、Debian/Ubuntu系):
    sudo apt install -y tesseract-ocr
    pip3 install --break-system-packages opencv-python-headless pytesseract requests

設定ファイル(既定 ~/.nokosahen_tablet_ocr_config.json)は
tablet_ocr_config.example.json をコピーして使う。roisの座標は
--calibrate で撮影したスナップショット(grid付き)を見ながら
画面比率(0.0〜1.0)で調整すること。
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import re
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, List, Optional, Tuple

import numpy as np
import requests

logger = logging.getLogger("tablet_ocr_reader")

# 【2026-10-10追加】カメラの画質まわり(鮮鋭度の定量・UVC自動調整・ソフトの画質改善)は同じ
# フォルダのocr_camera_quality.pyにある。単体ファイルだけ配備された場合でも従来機能は動くよう、
# 無ければ画質機能だけ無効にして続行する(cq=None)。
try:
    import ocr_camera_quality as cq
except ImportError:  # pragma: no cover - 配備漏れの保険
    cq = None


DEFAULT_CONFIG_PATH = "~/.nokosahen_tablet_ocr_config.json"

# A/B/Cの並び順は、のこさへん側(残数入力画面)の1〜3番目のメニューと
# 対応させる(web_ui_node.pyのSLOT_LABELSと揃えること)。
SLOT_ORDER = ["A", "B", "C"]

# 2026-09-25時点で実際に確認したタブレット画面の全7行の並び順(上から下)。
# roi_mode="auto_color"(タブレットの位置が動いても追従する自動検出モード)で、
# 検出した行を上から順にこの並びのスロット名へ割り当てるために使う。
FULL_ROW_ORDER = ["A", "B", "C", "curry", "noodles", "alacarte_b", "alacarte_100"]

# train_digit_model.py側の学習データ生成と必ず揃えること(値がずれると
# モデルが学習していない見た目の画像を渡すことになり精度が落ちる)。
DIGIT_CANVAS_SIZE = 32
DIGIT_PADDING_RATIO = 0.15

DEFAULT_CONFIG: Dict = {
    "server_base_url": "http://192.0.2.250:8080",
    "auth_username": "ADMIN_ACCOUNT",
    "auth_password": "",  # 実値は~/.nokosahen_tablet_ocr_config.json(リポジトリ外)か環境変数NOKOSAHEN_TABLET_OCR_PASSWORD
    "camera_index": "auto",
    # 【2026-09-25追記】Logitech C270の実際の最大解像度(v4l2-ctlで確認済み、
    # MJPG形式なら1280x960で30fpsまで出る)。カメラを差し替えた場合は
    # `v4l2-ctl -d /dev/videoX --list-formats-ext`で確認し直すこと。
    "width": 1280,
    "height": 960,
    "poll_interval_sec": 5.0,
    # 同じ数字がこの回数連続で読めるまでは確定させない(誤読対策)。
    "stable_reads_required": 3,
    # "edgetpu"(既定。2026-10-27: Coralを使った推論にするため、tesseractから変更)・"edgetpu_docker"・"tesseract"のいずれか。
    # "edgetpu": digit_model_pathのコンパイル済みモデルをこのプロセス内で
    # 直接推論する(pycoral/ai-edge-litertが動く環境向け)。
    # "edgetpu_docker": このRaspberry Pi(Debian 13)のようにpycoralが直接
    # 動かせない環境向け。coral_tpu_retry/start_docker_coral_server.shで
    # 起動した常駐Dockerコンテナ(edgetpu_docker_url)へHTTPで問い合わせる。
    # いずれも読み込み・接続に失敗した場合は自動的にtesseractへフォールバックする。
    "recognizer": "edgetpu",
    "edgetpu_docker_url": "http://127.0.0.1:5005",
    # "fixed"(既定): rois(下記)の固定比率で切り出す。カメラとタブレットの
    # 位置関係が変わらない設置を前提とする。
    # "auto_color": rois固定枠を使わず、毎周期detect_digit_rows()で数字の
    # 色(赤茶色)を手がかりに行位置を動的に検出する。タブレットを職員が
    # 手持ちして位置が動く場合向け(2026-09-25追加)。検出できた行数が
    # ちょうど7でない場合はその周期をスキップする(FULL_ROW_ORDER参照)。
    # "auto_lock"(2026-10-02追加): 起動直後はauto_colorと同じく毎周期検出するが、
    # ちょうど7行が連続auto_lock_required_cycles回・ほぼ同じ位置で検出できた
    # 時点でROIを「ロック」し、以降は再検出せずその固定領域だけを読む(CPU節約と、
    # 手・影・一部隠れによる周期スキップの回避)。全スロットが
    # auto_lock_reset_after_fail_cycles周期連続で読めなければ自動で再検出に戻る。
    # ダッシュボードの「ROIを再検出」ボタンでも手動で戻せる。
    "roi_mode": "fixed",
    # 【2026-10-09追加】カメラ映像を取り込んだ直後に、この角度(時計回り、度)だけ
    # 回転して「正立」に戻す。0(既定)=従来どおり何もしない。90/180/270のみ有効で、
    # それ以外は警告を出して0扱い。模擬POSタブレットをラズパイ画面へ90度回転表示
    # (?rot=90)して撮る場合は、その逆回転(rot=90なら270、rot=-90なら90)を指定する。
    # rois・locked_rois・プレビュー描画はすべて回転後のフレーム座標になる。
    # 確認用ダッシュボードの「設定」欄から実行中に切り替えることもできる。
    "frame_rotate_deg": 0,
    # 【2026-10-09追加】カメラのプロファイル切替。"camera_profile"("demo"|"production")を
    # 設定ファイルに書くとプロファイル方式になり、"camera_profiles"(各プロファイルの
    # width/height/fourcc/capture_fps/sharpness/preview_fps/preview_max_width/
    # preview_jpeg_quality/poll_interval_sec の上書き)が有効になる。キーが無い設定ファイルは
    # 従来どおり(上の width/height と preview_* だけを使う)。実行中に確認画面から切替可能。
    # 既定値は DEFAULT_CAMERA_PROFILES 参照。
    "auto_lock_required_cycles": 3,
    # 連続検出の位置一致の許容誤差(画面幅/高さに対する比率)。
    "auto_lock_tolerance": 0.04,
    # ロック時に各行の左右へ足す余白(行の幅に対する比率、桁数が増えた時用)。
    "auto_lock_margin_ratio": 0.05,
    "auto_lock_reset_after_fail_cycles": 12,
    # trueの時だけ、ロックしたROIをこの設定ファイルの"locked_rois"へ書き込む
    # (既定false=ユーザーの保存済み設定を一切書き換えない)。"locked_rois"が
    # 設定にある場合は、起動時にそのROIでロック済みとして開始する。
    "auto_lock_save": False,
    "locked_rois": None,
    # 【2026-10-05追加】動的ROI検出(auto_color/auto_lock)で同じ行の桁を1つの塊へ
    # 結合する横長CLOSEカーネルの幅(px)。0=無効(従来動作)。tab2.pyは20。
    # 1桁の行は、この設定が0でも縦列と整合すれば残る(detect_digit_rows参照)。
    "roi_row_merge_kernel_w": 0,
    # 【2026-10-05追加】チームメンバーのtab2.py風テキスト出力。空文字=無効。
    # 確定済みの全7スロットの値を FULL_ROW_ORDER(A,B,C,curry,noodles,
    # alacarte_b,alacarte_100)の順にカンマ区切り1行で追記する(未確定は"---")。
    # numbers_txt_on_change_only: trueなら値が変わった周期だけ追記、falseなら毎周期。
    # numbers_txt_with_timestamp: trueなら行頭に"YYYY-MM-DDTHH:MM:SS,"を付ける。
    # numbers_txt_include_estimated: falseなら推定値(TrendPredictor)は書かず、
    # 直近の実測確定値を書く(実測がまだ無ければ"---")。
    "numbers_txt_path": "",
    "numbers_txt_on_change_only": True,
    "numbers_txt_with_timestamp": False,
    "numbers_txt_include_estimated": False,
    # OCR値の急変フィルタ(2026-10-02追加)。累計販売数は増える一方なので、
    # 確定値から減る/この値以上に跳ね上がる読み取りは誤読として棄却する。
    # ただし同じ値がstable_reads_requiredの2倍回連続したら(日替わりリセット
    # や、過去の誤確定の訂正とみなして)受理する。0以下で無効。
    "ocr_max_jump": 50,
    # OCRで読み取った値が、残数入力画面で最初に入力した初期残数(initial)の
    # この倍率を超えて大きい場合は、明らかな誤読(縁の影を数字と誤認して
    # 桁が増える等)とみなして無視する(2026-09-25追加、ユーザー要望
    # 「初期残数より極端に大きい読み取り値は弾く」対応)。仕込み追加等で
    # 本当にinitialを超えて売れること自体はあり得るため、単純に
    # 「initialを超えたら弾く」のではなく、ある程度の倍率までは許容する。
    "ocr_sanity_max_ratio": 3.0,
    # OCRが同じスロットでこの回数だけ連続して読み取れなかった場合、直近の
    # 確定値の増減ペースから推測した値(TrendPredictor参照)を確定値の代わりに
    # 使う。既定6回×poll_interval_sec(既定5秒)=約30秒読み取れない状態が
    # 続いたら推測に切り替わる計算(2026-09-25追加)。
    "predict_after_consecutive_fails": 6,
    # 増減ペースの計算に使う直近の確定値の点数(多いほど平均化されて安定するが、
    # 直近の傾向の変化への追従は遅くなる)。
    "predict_history_size": 6,
    # OCR前処理(preprocess_for_ocr)でROI切り出し画像を何倍に拡大するか。
    # 実際にラズパイ・カメラ・タブレットを固定設置してみないと最適値が
    # 分からないため、確認用ダッシュボードの「設定」欄から実行中に変更できる
    # ようにしてある(2026-09-25追加、ユーザー要望)。変更するとこの設定
    # ファイルにも書き戻されるので、次回起動時も変更後の値が使われる。
    "ocr_scale_factor": 2.0,
    # 【2026-10-10追加】OCR前処理の画質改善(ocr_camera_quality.py、docs/ocr_camera_tuning.md参照)。
    # 既定(None/省略)=有効で控えめ。falseで全て無効(従来の前処理のみ)。辞書で個別に上書き:
    # best_of_n(直近N枚から鮮鋭なフレームを選ぶ。intかプロファイル別{"demo":3,"production":5})、
    # average_frames(静止中だけ平均してノイズ低減。同形式)、unsharp_amount/unsharp_radius、
    # clahe(clipLimit、0=無効)、auto_levels、interpolation("lanczos"|"cubic")、max_age_sec。
    "image_enhance": None,
    # ピント(鮮鋭度)の判定しきい値(表示スコア。good以上=良好、fair以上=やや悪い、未満=悪い)と、
    # 「悪い」が続いたらガイダンスを出すまでの秒数。実機の合焦時のスコアを見て調整する。
    "focus_thresholds": {"good": 60.0, "fair": 35.0},
    "focus_warn_after_sec": 20.0,
    # 「画質を自動調整」の結果(プロファイルごとの採用値と元の値)。確認画面から自動で書き込まれ、
    # 起動時・プロファイル切替時に再適用される。手で編集しない(元に戻すボタンで消える)。
    "camera_tuning": None,
    # カメラ映像(ROI枠・現在の読み取り値を重ねて描画したもの)をブラウザで
    # 確認できる簡易ダッシュボード(認証無し、ローカル/同一LAN内での確認用)。
    "preview_enabled": True,
    "preview_host": "127.0.0.1",  # 【2026-10-21】認証なしのため、1台目の内側だけ。映像・設定は、管理画面(ログイン必須)が中継する
    "preview_port": 8091,
    # プレビュー映像の撮影fps(OCRの実際の判定周期=poll_interval_secとは
    # 独立、2026-09-25追加)。撮影はこのスレッドに一本化されるので、
    # run_loop側の判定周期はこの値と無関係にそのまま(既定5秒)動く。
    # ラズパイCPUでの重ねて描く処理+JPEGエンコードが律速要因なので、
    # preview_max_width/preview_jpeg_qualityで画素数を減らして高速化する
    # (認識処理自体はフル解像度のフレームを使うので精度には影響しない)。
    "preview_fps": 10.0,
    "preview_max_width": 480,
    "preview_jpeg_quality": 55,
    # 確認用ダッシュボードのヘッダーから、のこさへん本体の総合ダッシュボード
    # (full_dashboard_node.py、ポート8090)へ戻れるようにするリンク先
    # (2026-09-25追加、ユーザー要望「メインの管理画面から飛んだり戻ったり
    # できるように」)。空文字(既定)なら、この確認画面を開いているホストと
    # 同じホストのポート8090へ向ける(2026-09-30変更: ネットワーク変更で
    # 固定IPが変わっても壊れないように)。別の機器(のこさへん本体が別ホスト)
    # の場合だけ "http://<IP>:8090/" を明示する。
    "dashboard_url": "",
    "digit_model_path": "./digit_model/digit_model_int8_edgetpu.tflite",
    # タブレット画面の中で、それぞれの「数字部分」だけを切り出すための矩形
    # (画面全体を0.0〜1.0とした比率)。実機のカメラ設置位置に合わせて
    # --calibrate で調整すること。ここにある値はあくまで初期値の例で、
    # 2026-09-24時点で実際に確認したタブレット画面(全7行:
    # Aランチ/Bランチ/Cランチ/カレー/麺類(昼)/アラカルトB/アラカルト100円)を
    # 均等7分割した比率(横方向はA/B/C用の初期値=0.55〜0.95を流用)。
    # のこさへん本体(残数管理)が対応しているのはA/B/Cの3項目のみ
    # (SLOT_ORDER参照)なので、そのうち送信されるのはA/B/Cだけ。残り4項目
    # (curry等)も読み取ってログには出すが、対応するメニュースロットが本体側に
    # 無いため現状は送信しない(必要になったら本体側のメニュー枠を増やす
    # 設計変更が別途必要)。
    "rois": {
        "A": {"x1": 0.55, "y1": 0.050, "x2": 0.95, "y2": 0.179},
        "B": {"x1": 0.55, "y1": 0.179, "x2": 0.95, "y2": 0.307},
        "C": {"x1": 0.55, "y1": 0.307, "x2": 0.95, "y2": 0.436},
        "curry": {"x1": 0.55, "y1": 0.436, "x2": 0.95, "y2": 0.564},
        "noodles": {"x1": 0.55, "y1": 0.564, "x2": 0.95, "y2": 0.693},
        "alacarte_b": {"x1": 0.55, "y1": 0.693, "x2": 0.95, "y2": 0.821},
        "alacarte_100": {"x1": 0.55, "y1": 0.821, "x2": 0.95, "y2": 0.950},
    },
}


@dataclass
class Roi:
    x1: float
    y1: float
    x2: float
    y2: float


VALID_ROTATE_DEGS = (0, 90, 180, 270)


def normalize_rotate_deg(value: object, warn: bool = True) -> int:
    """frame_rotate_degを0/90/180/270のいずれかへ正規化する(許可リスト方式)。
    不正な値は警告を出して0(回転なし)扱いにする。bool・文字列・90.5等は不正。
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        ok = False
    else:
        ok = float(value).is_integer() and int(value) in VALID_ROTATE_DEGS
    if ok:
        return int(value)  # type: ignore[arg-type]
    if warn:
        logger.warning(
            "frame_rotate_deg=%r は不正です(0/90/180/270のみ有効)。0(回転なし)として扱います", value
        )
    return 0


def rotate_frame(cv2, frame, deg: int):
    """撮影直後のフレームを時計回りにdeg度回転する(0なら同じオブジェクトをそのまま返す)。
    90/270では幅と高さが入れ替わる。以降のROI(比率)・プレビュー・検出は全て
    回転後のフレームのshapeから幅/高さを取ること。
    """
    if deg == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if deg == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if deg == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame


def load_config(path: str) -> Dict:
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        logger.warning(
            f"設定ファイルが見つからないため既定値で動作します: {path}"
            "(tablet_ocr_config.example.json を参考に作成してください)"
        )
        config = dict(DEFAULT_CONFIG)
        config["camera_profile"] = "demo"
        config["_config_path"] = path
        return config
    with open(path, "r", encoding="utf-8") as f:
        user_config = json.load(f)
    config = dict(DEFAULT_CONFIG)
    config.update(user_config)
    if "rois" in user_config:
        config["rois"] = user_config["rois"]
    # 確認用ダッシュボードから設定(拡大倍率等)を変更した際に、この設定
    # ファイルへ書き戻すためのパスを保持しておく(2026-09-25追加)。
    # "_"始まりのキーはDEFAULT_CONFIGにもtablet_ocr_config.example.jsonにも
    # 存在しないため、save_config_updates()側で書き戻す際に除外する。
    config["_config_path"] = path
    return config


def save_config_updates(config_path: str, updates: Dict) -> None:
    """確認用ダッシュボードから変更した設定値を、設定ファイルへ書き戻す。

    【背景】ユーザー要望「拡大する倍率は実際に固定しないとわからないから
    htmlの管理画面から変更できるようにしておいて」(2026-09-25)への対応。
    プロセス再起動後も変更が残るよう、メモリ上の状態(PreviewState)だけで
    なく設定ファイル自体も更新する。既存のファイル内容(rois等)は保ったまま、
    updatesで渡されたキーだけを上書きする。
    """
    path = os.path.expanduser(config_path)
    current: Dict = {}
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            current = json.load(f)
    current.update(updates)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(current, f, indent=2, ensure_ascii=False)
        f.write("\n")


def _candidate_video_devices() -> List[str]:
    """checkout_camera_node.pyと同じ考え方: 実際に開けるデバイスを自動探索する。"""
    devices: List[str] = []
    try:
        for name in sorted(os.listdir("/dev")):
            if name.startswith("video"):
                devices.append(f"/dev/{name}")
    except OSError:
        pass

    def _video_num(p: str) -> int:
        try:
            return int(p.rsplit("video", 1)[1])
        except (IndexError, ValueError):
            return 999

    devices.sort(key=_video_num)
    return devices


# 【2026-10-09追加】カメラのプロファイル。BUFFALO BSWHD06M(UVC、固定焦点)の実測仕様:
# MJPG 1280x960=最速12fps、1280x720=30fps、800x600/640x480=30fps(YUY2は遅いので必ずMJPG)。
# production=最高画質(撮影は数秒に1回でよいのでfps優先度は低い)、demo=少し画質を下げて滑らかに動かす。
DEFAULT_CAMERA_PROFILES: Dict[str, Dict] = {
    "demo": {
        "width": 800, "height": 600, "fourcc": "MJPG", "capture_fps": 30, "sharpness": None,
        "preview_fps": 12.0, "preview_max_width": 480, "preview_jpeg_quality": 55,
        "poll_interval_sec": 5.0,
    },
    "production": {
        "width": 1280, "height": 960, "fourcc": "MJPG", "capture_fps": 12, "sharpness": "max",
        "preview_fps": 1.0, "preview_max_width": 960, "preview_jpeg_quality": 90,
        "poll_interval_sec": 5.0,
    },
}
CAMERA_PROFILE_NAMES = ("demo", "production")
# 希望解像度が取れなかった場合の次点候補(大きい順)
_CAMERA_FALLBACK_SIZES = [(1280, 960), (1280, 720), (800, 600), (640, 480)]


def _valid_profile_value(key: str, value: object) -> bool:
    def num(v, lo, hi):
        return isinstance(v, (int, float)) and not isinstance(v, bool) and lo <= v <= hi

    if key in ("width", "height", "preview_max_width"):
        return isinstance(value, int) and not isinstance(value, bool) and 160 <= value <= 4096
    if key == "fourcc":
        return isinstance(value, str) and len(value) == 4
    if key == "capture_fps":
        return value is None or num(value, 1, 120)
    if key == "sharpness":
        return value is None or value == "max" or num(value, 0, 65535)
    if key == "preview_fps":
        return num(value, 0.5, 60)
    if key == "preview_jpeg_quality":
        return isinstance(value, int) and not isinstance(value, bool) and 1 <= value <= 100
    if key == "poll_interval_sec":
        return num(value, 1, 3600)
    return False


def resolve_camera_profiles(config: Dict) -> Dict[str, Dict]:
    """既定値へ設定の"camera_profiles"の上書きを重ねた、各プロファイルの有効値を返す。
    不正な値・未知のキー・未知のプロファイル名は警告して無視する(既定値のまま)。"""
    profiles = {name: dict(vals) for name, vals in DEFAULT_CAMERA_PROFILES.items()}
    overrides = config.get("camera_profiles")
    if overrides is None:
        return profiles
    if not isinstance(overrides, dict):
        logger.warning("camera_profiles は辞書ではないため無視します: %r", overrides)
        return profiles
    for name, ov in overrides.items():
        if name not in profiles:
            logger.warning("camera_profiles に未知のプロファイル名があるため無視します: %r", name)
            continue
        if not isinstance(ov, dict):
            logger.warning("camera_profiles[%s] は辞書ではないため無視します", name)
            continue
        for key, value in ov.items():
            if key in profiles[name] and _valid_profile_value(key, value):
                profiles[name][key] = value
            else:
                logger.warning("camera_profiles[%s][%s]=%r は不正なため無視します(既定値を使用)", name, key, value)
    return profiles


def select_camera_profile(config: Dict) -> Optional[str]:
    """設定のcamera_profileを返す。キーが無い=従来動作(None)。不正値は警告してdemo。"""
    if "camera_profile" not in config:
        return None
    value = config.get("camera_profile")
    if isinstance(value, str) and value in CAMERA_PROFILE_NAMES:
        return value
    logger.warning("camera_profile=%r は不正です(demo/productionのみ)。demoとして扱います", value)
    return "demo"


def legacy_open_params(config: Dict) -> Dict:
    """camera_profileが無い設定(従来動作)でカメラを開く時のパラメータ。"""
    return {
        "width": int(config["width"]), "height": int(config["height"]), "fourcc": "MJPG",
        "capture_fps": None, "sharpness": None, "size_fallback": False,
    }


def camera_size_candidates(width: int, height: int) -> List[Tuple[int, int]]:
    """希望解像度を先頭に、それより小さい標準解像度を大きい順に並べた試行リスト。"""
    out = [(width, height)]
    for w, h in _CAMERA_FALLBACK_SIZES:
        if (w, h) != (width, height) and w * h < width * height:
            out.append((w, h))
    return out


def _fourcc_to_str(value: float) -> str:
    v = int(value)
    chars = [chr((v >> (8 * i)) & 0xFF) for i in range(4)]
    return "".join(c for c in chars if c.isprintable()).strip() or "?"


def apply_sharpness(cv2, cap, value) -> Dict:
    """UVCのシャープネスを設定し、読み戻せたかを返す。"max"は上限が機種で違うため、
    大きい候補から順に試し、最初に設定が受理された値を使う。効かなくても動作は継続する。"""
    result: Dict = {"requested": value, "applied": False, "readback": None}
    prop = getattr(cv2, "CAP_PROP_SHARPNESS", None)
    if prop is None:
        return result
    candidates = [255, 128, 100, 64, 32, 16, 15, 10, 7, 6, 5, 4, 3, 2, 1] if value == "max" else [value]
    for cand in candidates:
        try:
            accepted = bool(cap.set(prop, float(cand)))
            back = float(cap.get(prop))
        except Exception:  # noqa: BLE001
            continue
        if value == "max":
            if accepted:
                result.update(applied=True, readback=back)
                return result
        else:
            result["readback"] = back
            result["applied"] = accepted and abs(back - float(cand)) < 0.5
    return result


def open_camera_ex(
    cv2, requested: str, width: int, height: int, fourcc: str = "MJPG",
    capture_fps: Optional[float] = None, sharpness=None, size_fallback: bool = False,
):
    """カメラを開き、(cap, info)を返す。infoは実際に取得できた解像度・fps・fourccなど
    (確認画面とAPIの camera_actual)。size_fallback=Trueで希望解像度が取れなければ
    次点の解像度へ順にフォールバックする(その場合 info["fallback"]=True)。"""
    candidates: List[str]
    if requested and requested.strip().lower() != "auto":
        candidates = [requested.strip()]
    else:
        candidates = _candidate_video_devices() or ["/dev/video0"]

    for candidate in candidates:
        cap = cv2.VideoCapture(candidate)
        if not cap.isOpened():
            cap.release()
            continue
        # 【2026-09-25追記】v4l2-ctl --list-formats-extで確認したところ、YUYV(既定)だと
        # 遅いが、MJPG形式を明示すると高いfpsが出せる(カメラ内部でJPEG圧縮されるため
        # 転送量が減る)。fourccはwidth/heightより先に設定する必要がある(V4L2の作法)。
        sizes = camera_size_candidates(width, height) if size_fallback else [(width, height)]
        got = None
        for (w, h) in sizes:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            if capture_fps:
                cap.set(cv2.CAP_PROP_FPS, float(capture_fps))
            # V4L2既定のバッファ(4枚)だと、間引き周期でread()した時に古いフレームが
            # 返るため、バッファを1枚にして常に最新に近い画像を得る。
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            frame = None
            for _ in range(3 if size_fallback else 1):
                ok, frame = cap.read()
                if ok and frame is not None and frame.size > 0:
                    break
                frame = None
            if frame is None:
                continue
            got = frame
            if (frame.shape[1], frame.shape[0]) == (w, h):
                break
        if got is None:
            cap.release()
            continue
        sharp_info = None
        if sharpness is not None:
            prop = getattr(cv2, "CAP_PROP_SHARPNESS", None)
            before = None
            if prop is not None:
                try:
                    before = float(cap.get(prop))
                except Exception:  # noqa: BLE001
                    before = None
            sharp_info = apply_sharpness(cv2, cap, sharpness)
            sharp_info["before"] = before
            logger.info(
                "シャープネス設定: 要求=%s 反映=%s 読み戻し=%s(効かなくても動作は継続します)",
                sharpness, "有効" if sharp_info["applied"] else "不明/無効", sharp_info["readback"],
            )
        try:
            fps_raw = float(cap.get(cv2.CAP_PROP_FPS))
        except Exception:  # noqa: BLE001
            fps_raw = 0.0
        try:
            fourcc_str = _fourcc_to_str(cap.get(cv2.CAP_PROP_FOURCC))
        except Exception:  # noqa: BLE001
            fourcc_str = "?"
        aw, ah = int(got.shape[1]), int(got.shape[0])
        info = {
            "width": aw, "height": ah, "fps": fps_raw if fps_raw > 0 else None, "fourcc": fourcc_str,
            "requested": {"width": width, "height": height, "fourcc": fourcc},
            "fallback": (aw, ah) != (width, height), "device": candidate, "sharpness": sharp_info,
        }
        logger.info(
            "Webカメラを開きました: %s 実際=%dx%d %s fps=%s(要求=%dx%d %s)",
            candidate, aw, ah, fourcc_str, info["fps"], width, height, fourcc,
        )
        if info["fallback"]:
            logger.warning("希望の解像度%dx%dが取得できなかったため%dx%dで動作します", width, height, aw, ah)
        return cap, info
    raise RuntimeError("Webカメラを開けませんでした(/dev/video*を自動探索しましたが見つかりません)")


def open_camera(cv2, requested: str, width: int, height: int):
    return open_camera_ex(cv2, requested, width, height)[0]


class CameraController:
    """カメラ(cv2.VideoCapture)の所有者。撮影スレッド以外からは触らない。
    プロファイル切替(switch)で release→新しい設定で開き直す。失敗したら元の設定へ戻す。
    openerはテスト用に差し替え可能: opener(cv2, camera_index, params) -> (cap, info)。"""

    def __init__(self, cv2, camera_index: str, opener=None) -> None:
        self.cv2 = cv2
        self.camera_index = camera_index
        self._opener = opener or self._default_opener
        self.cap = None
        self.info: Dict = {}
        self.params: Optional[Dict] = None
        self.last_error: Optional[str] = None
        self._orig_sharp: Optional[float] = None
        self._sharp_changed = False
        # 【2026-10-10追加】「画質を自動調整」の保存値(camera_tuning、検証済み)。開くたびに
        # tuning_profileのものを再適用する(UVCの設定はデバイスに残るので、保存値の無い
        # プロファイルへ切り替えた時は他プロファイルの「元の値」へ戻す)。
        self.tuning: Dict[str, Dict] = {}
        self.tuning_profile: Optional[str] = None

    def _default_opener(self, cv2, camera_index, params):
        return open_camera_ex(
            cv2, camera_index, params["width"], params["height"], params.get("fourcc", "MJPG"),
            params.get("capture_fps"), params.get("sharpness"), bool(params.get("size_fallback", False)),
        )

    def _open(self, params: Dict) -> None:
        eff = dict(params)
        # sharpness未指定(None)でも、以前に変更していれば元の値へ戻す(UVCの設定は閉じても残る)
        if eff.get("sharpness") is None and self._sharp_changed and self._orig_sharp is not None:
            eff["sharpness"] = self._orig_sharp
        cap, info = self._opener(self.cv2, self.camera_index, eff)
        self.cap, self.info, self.params = cap, info, params
        if cq is not None and self.tuning:
            tuned = cq.reapply_tuning(self.cv2, cap, self.tuning, self.tuning_profile)
            if tuned["mode"] != "none":
                bad = [k for k, ok in tuned["results"].items() if not ok]
                logger.info(
                    "カメラの保存済み調整を%sしました(%s)%s", "再適用" if tuned["mode"] == "applied" else "元の値へ復元",
                    self.tuning_profile or "default", f" 反映されなかった項目: {bad}" if bad else "",
                )
                self.info = dict(info, tuning=tuned["mode"])
        sh = info.get("sharpness")
        if sh:
            if self._orig_sharp is None and sh.get("before") is not None:
                self._orig_sharp = sh["before"]
            if params.get("sharpness") is not None and sh.get("applied"):
                self._sharp_changed = True

    def open(self, params: Dict) -> Dict:
        self._open(params)
        return self.info

    def release(self) -> None:
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:  # noqa: BLE001
                pass
            self.cap = None

    def switch(self, params: Dict, tuning_profile: Optional[str] = None) -> bool:
        """新しい設定で開き直す。失敗したら元の設定へ戻してFalse(戻せなければcap=None)。
        tuning_profile: 新しい設定の保存済み調整(camera_tuning)のプロファイル名(開き直し時に再適用)。"""
        old = self.params
        old_tuning_profile = self.tuning_profile
        if tuning_profile is not None:
            self.tuning_profile = tuning_profile
        self.release()
        try:
            self._open(params)
            self.last_error = None
            return True
        except Exception as exc:  # noqa: BLE001
            self.last_error = str(exc)
            self.tuning_profile = old_tuning_profile
            logger.error("カメラの切替に失敗しました。元の設定へ戻します: %s", exc)
        if old is not None:
            try:
                self._open(old)
            except Exception as exc2:  # noqa: BLE001
                self.cap = None
                logger.error("元の設定でもカメラを開けませんでした(再試行を続けます): %s", exc2)
        return False

    def recover(self) -> bool:
        """cap=None(開き直しに失敗した状態)から、直近の設定でもう一度開く。"""
        if self.cap is not None or self.params is None:
            return self.cap is not None
        try:
            self._open(self.params)
            return True
        except Exception as exc:  # noqa: BLE001
            logger.warning("カメラの再オープンに失敗(再試行します): %s", exc)
            return False


def crop_roi(cv2, frame, roi: Roi):
    h, w = frame.shape[0], frame.shape[1]
    x1 = max(0, min(w - 1, int(roi.x1 * w)))
    x2 = max(x1 + 1, min(w, int(roi.x2 * w)))
    y1 = max(0, min(h - 1, int(roi.y1 * h)))
    y2 = max(y1 + 1, min(h, int(roi.y2 * h)))
    return frame[y1:y2, x1:x2]


def crop_pixel_box(frame, box: Tuple[int, int, int, int]):
    x1, y1, x2, y2 = box
    return frame[y1:y2, x1:x2]


def _pixel_box_to_roi(frame, box: Tuple[int, int, int, int]) -> Roi:
    h, w = frame.shape[0], frame.shape[1]
    x1, y1, x2, y2 = box
    return Roi(x1=x1 / w, y1=y1 / h, x2=x2 / w, y2=y2 / h)


def detect_digit_rows(cv2, np, frame, merge_kernel_w: int = 0) -> List[Tuple[int, int, int, int]]:
    """タブレット画面の位置を仮定せず、数字表示部分の特徴的な色(赤茶色)を
    手がかりに「行」を動的に検出する。

    【背景】ユーザー要望「枠で括らずに、タブレットが動いた場合でも追従して
    読み取りできないか」(2026-09-25)への対応。実物のタブレットは職員が
    手持ちするため、固定ROI(rois設定)だと画面の位置がずれると読み取れない。

    実装は/home/pi/test_color_detect_e2e.py(ラズパイ上で実物写真を使って
    パラメータを調整済み)と同一のパイプライン。実物写真1枚での検証結果は
    行位置検出7/7、Edge TPU数字認識まで含めて6/7が正解(残り1行は縁の影が
    写り込んで余分な"1"が付く既知の制約)。静止画では同じ成因のアーティ
    ファクトが毎回同じフレームに再現するが、実運用では手持ちのタブレットが
    フレームごとに微妙に動くため、stable_reads_required(既定3回連続一致)
    の安定化ロジックで大部分は吸収される想定。

    戻り値は画面上端から下端の順に並んだ(x1, y1, x2, y2)のピクセル座標の
    リスト(検出できた行数がそのまま返る。7行ちょうどとは限らない。呼び出し
    側で期待行数と一致するかを確認すること)。

    【2026-10-05追記】輪郭1個だけの行(残数が1桁など)を無条件に捨てていたため、
    7行のうち1行でも1桁になると検出行数が6になり周期ごとスキップされていた。
    現在は、複数輪郭の行がなす縦列(右端x2または中心xが近く、高さが0.6〜1.6倍)
    と整合する1輪郭の行だけを残し、整合しないもの(ノイズ)は従来通り捨てる。
    merge_kernel_w>0なら、チームメンバーのtab2.pyと同様に赤マスクへ横長カーネル
    (merge_kernel_w x 3)のCLOSEを追加し、同じ行の桁を1つの塊へ結合する
    (0=従来動作)。
    """
    h_img, w_img = frame.shape[0], frame.shape[1]
    frame_area = h_img * w_img

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # タブレットの数字表示色(赤茶色)。OpenCVのHは0-179で、赤は0付近と
    # 180付近の両端にまたがるため2レンジを合成する。
    lower1 = np.array([0, 60, 60])
    upper1 = np.array([20, 255, 255])
    lower2 = np.array([165, 60, 60])
    upper2 = np.array([179, 255, 255])
    mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    if merge_kernel_w and merge_kernel_w > 1:
        hkernel = cv2.getStructuringElement(cv2.MORPH_RECT, (int(merge_kernel_w), 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, hkernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        area = w * h
        # 小さすぎるノイズと、背景全体・腕/服等の巨大な誤検出領域を除外する
        # (RETR_EXTERNALは画面端に触れた白領域を1つの巨大輪郭として拾うため)。
        if area < frame_area * 0.0003 or area > frame_area * 0.005:
            continue
        aspect = w / max(1, h)
        # 1桁の数字(縦長、aspect~0.4-0.7)〜数桁くっついたものまでを許容する。
        # これより狭めると単独の"1"のような細い数字を誤って除外してしまう。
        if aspect < 0.25 or aspect > 3.0:
            continue
        boxes.append((x, y, w, h))

    # y中心が近いものを同じ「行」としてグルーピングする。
    boxes.sort(key=lambda b: b[1])
    rows: List[dict] = []
    row_gap_thresh = h_img * 0.03
    for x, y, w, h in boxes:
        cy = y + h / 2
        placed = False
        for row in rows:
            if abs(row["cy"] - cy) < row_gap_thresh:
                row["items"].append((x, y, w, h))
                row["cy"] = sum(it[1] + it[3] / 2 for it in row["items"]) / len(row["items"])
                placed = True
                break
        if not placed:
            rows.append({"cy": cy, "items": [(x, y, w, h)]})
    rows.sort(key=lambda r: r["cy"])

    def row_bounds(row):
        xs1 = [it[0] for it in row["items"]]
        ys1 = [it[1] for it in row["items"]]
        xs2 = [it[0] + it[2] for it in row["items"]]
        ys2 = [it[1] + it[3] for it in row["items"]]
        return min(xs1), min(ys1), max(xs2), max(ys2)

    # 輪郭1個だけの「行」は、複数輪郭の行がなす縦列と整合する場合だけ残す
    # (残数が1桁の行は輪郭1個になる。整合しない孤立ノイズは従来通り除外)。
    multi_rows = [r for r in rows if len(r["items"]) >= 2]
    if not multi_rows:
        return []
    ref = [row_bounds(r) for r in multi_rows]
    ref_x2 = statistics.median(b[2] for b in ref)
    ref_cx = statistics.median((b[0] + b[2]) / 2.0 for b in ref)
    ref_h = statistics.median(b[3] - b[1] for b in ref)
    col_tol = max(w_img * 0.04, ref_h * 0.5)

    def single_fits_column(row) -> bool:
        x1, y1, x2, y2 = row_bounds(row)
        height = y2 - y1
        if ref_h <= 0 or not (0.6 * ref_h <= height <= 1.6 * ref_h):
            return False
        return abs(x2 - ref_x2) <= col_tol or abs((x1 + x2) / 2.0 - ref_cx) <= col_tol

    rows = [r for r in rows if len(r["items"]) >= 2 or single_fits_column(r)]
    rows.sort(key=lambda r: r["cy"])

    bounds_list = [row_bounds(r) for r in rows]

    # 数字はすべて同じ縦の列(タブレット右側の数字欄)に揃っているはずなので、
    # 各行の右端x2の中央値から大きく左にずれる行(指・手など)は誤検出として
    # 除外する。
    x2_values = sorted(b[2] for b in bounds_list)
    median_x2 = x2_values[len(x2_values) // 2]
    col_width_guess = median_x2 * 0.15

    kept = [
        (row, b)
        for row, b in zip(rows, bounds_list)
        if not (abs(b[2] - median_x2) > col_width_guess and b[2] < median_x2)
    ]

    result: List[Tuple[int, int, int, int]] = []
    for _row, (x1, y1, x2, y2) in kept:
        # 腕・手の混入で右端がはみ出す場合に備えてクランプする。
        x2 = min(x2, int(median_x2 + col_width_guess * 0.3))
        pad_x = int((x2 - x1) * 0.04)
        pad_y = int((y2 - y1) * 0.08)
        x1, y1 = max(0, x1 - pad_x), max(0, y1 - pad_y)
        x2, y2 = min(w_img, x2 + pad_x), min(h_img, y2 + pad_y)
        result.append((x1, y1, x2, y2))
    return result


def select_best_rows(
    boxes: List[Tuple[int, int, int, int]], expected: int = 7
) -> Optional[List[Tuple[int, int, int, int]]]:
    """検出した行(x1, y1, x2, y2)がexpected個より多い場合に、メインの縦列を
    なすexpected個を選ぶ。

    【背景】チームメンバーのtab.py(select_best_seven_boxes)の考え方を採用
    (2026-10-02)。数字欄は縦一列に揃っているので、x中心の中央値に近い
    expected個を残し、上から順に並べて返す。以前は7行ちょうどでないと
    その周期を丸ごとスキップしていた。expected未満はNone(呼び出し側でスキップ)。
    同距離の場合は上にある行を優先する。
    """
    if len(boxes) < expected:
        return None
    if len(boxes) == expected:
        return sorted(boxes, key=lambda b: (b[1], b[0]))
    x_centers = [(b[0] + b[2]) / 2.0 for b in boxes]
    median_x = statistics.median(x_centers)
    ranked = sorted(range(len(boxes)), key=lambda i: (abs(x_centers[i] - median_x), boxes[i][1]))
    chosen = [boxes[i] for i in ranked[:expected]]
    return sorted(chosen, key=lambda b: (b[1], b[0]))


class NumbersTxtWriter:
    """確定値をtab2.py風のカンマ区切り1行としてテキストへ追記する(2026-10-05追加)。

    values: スロット名->確定値(Noneは未確定)、estimated: スロット名->推定値か。
    推定値はinclude_estimated=falseなら使わず、直近の実測確定値を表示する。
    その周期の値がNone(未確定)でも、過去に値があればそれを保持する
    (SlotStabilizerが確定値を保持するのと同じ考え方)。まだ1つも無いスロットは"---"。
    例外はメインループへ出さない(書込み失敗は1回だけ警告)。
    """

    def __init__(
        self,
        path: str,
        on_change_only: bool = True,
        with_timestamp: bool = False,
        include_estimated: bool = False,
        order: Optional[List[str]] = None,
        clock=time.time,
    ) -> None:
        self.path = os.path.expanduser(path) if path else ""
        self.on_change_only = on_change_only
        self.with_timestamp = with_timestamp
        self.include_estimated = include_estimated
        self.order = list(order) if order else list(FULL_ROW_ORDER)
        self._clock = clock
        self._shown: Dict[str, Optional[int]] = {s: None for s in self.order}
        self._last_line: Optional[str] = None
        self._warned = False

    @property
    def enabled(self) -> bool:
        return bool(self.path)

    def update(self, values: Dict[str, Optional[int]], estimated: Optional[Dict[str, bool]] = None) -> Optional[str]:
        """行を追記したらその行(改行なし)を返す。書かなかった/失敗時はNone。"""
        if not self.enabled:
            return None
        estimated = estimated or {}
        for slot in self.order:
            v = values.get(slot)
            if v is None:
                continue
            if estimated.get(slot) and not self.include_estimated:
                continue
            self._shown[slot] = v
        if all(self._shown[s] is None for s in self.order):
            return None
        line = ",".join("---" if self._shown[s] is None else str(self._shown[s]) for s in self.order)
        if self.on_change_only and line == self._last_line:
            return None
        out = line
        if self.with_timestamp:
            out = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(self._clock())) + "," + line
        try:
            with open(self.path, "a", encoding="utf-8") as f:
                f.write(out + "\n")
        except Exception as exc:  # noqa: BLE001
            if not self._warned:
                self._warned = True
                logger.warning(f"numbers_txt_pathへの書き込みに失敗しました(以降の失敗は通知しません): {exc}")
            return None
        self._last_line = line
        return out


class JumpGuard:
    """OCR値の急変フィルタ(スロットごとに1つ)。

    【背景】tab.pyの「単調増加+飛びすぎ防止(curr >= prev かつ curr - prev < 50)」
    を採用(2026-10-02)。累計販売数は増える一方なので、確定済みの値(baseline)
    から減る読み取り、またはmax_jump以上増える読み取りは誤読として棄却
    (Noneを返す=「読み取れなかった」と同じ扱い)する。

    ただし棄却し続けると、日替わりリセットや、過去に誤って確定した値の訂正が
    永久に通らなくなる。同じ棄却値がoverride_reads回連続したら受理し、
    以降はその値を基準(override baseline)に判定する。baselineは呼び出し側が
    渡す「実測で確定した値」で、TrendPredictorの推定値は決して渡さないこと。
    """

    def __init__(self, max_jump: int = 50, override_reads: int = 6) -> None:
        self.max_jump = int(max_jump)
        self.override_reads = max(1, int(override_reads))
        self._reject_value: Optional[int] = None
        self._reject_streak = 0
        self._override_baseline: Optional[int] = None
        self._override_prev_baseline: Optional[int] = None

    def check(self, value: Optional[int], baseline: Optional[int]) -> Tuple[Optional[int], Optional[str]]:
        """(通す値 or None, 理由 or None)を返す。理由はログ用(棄却・強制受理時のみ)。"""
        if value is None or self.max_jump <= 0:
            return value, None
        if self._override_baseline is not None:
            if baseline != self._override_prev_baseline:
                # スタビライザーが新しい値を確定させた=強制受理後の基準はもう不要
                self._override_baseline = None
                self._override_prev_baseline = None
            else:
                baseline = self._override_baseline
        if baseline is None:
            return value, None
        diff = value - baseline
        if 0 <= diff < self.max_jump:
            self._reject_value = None
            self._reject_streak = 0
            return value, None
        if value == self._reject_value:
            self._reject_streak += 1
        else:
            self._reject_value = value
            self._reject_streak = 1
        kind = f"減少({baseline}->{value})" if diff < 0 else f"急増(+{diff}>={self.max_jump})"
        if self._reject_streak >= self.override_reads:
            self._override_prev_baseline = baseline if self._override_baseline is None else self._override_prev_baseline
            self._override_baseline = value
            streak = self._reject_streak
            self._reject_value = None
            self._reject_streak = 0
            return value, f"{kind}だが同じ値が{streak}回連続したため受理"
        return None, f"{kind}のため棄却(連続{self._reject_streak}/{self.override_reads})"


RatioBox = Tuple[float, float, float, float]


class RoiLocker:
    """roi_mode="auto_lock"用。detect_digit_rows()の結果が「ちょうど7行・ほぼ同じ
    位置」で連続required回得られたらROIをロックし、以降は再検出しない。

    【背景】tab.pyの「最初に7行を検出して位置を固定し、固定領域だけをOCRする」
    方式の採用(2026-10-02)。auto_colorは毎周期検出するため、手・影・一部隠れで
    行数が崩れると周期ごとスキップになっていた。ロック後は検出に左右されない。
    内部の座標は画面比率(0.0〜1.0)の(x1, y1, x2, y2)。
    """

    def __init__(
        self, expected: int = 7, required: int = 3, tolerance: float = 0.04,
        margin_ratio: float = 0.05, reset_after_fail_cycles: int = 12,
    ) -> None:
        self.expected = expected
        self.required = max(1, required)
        self.tolerance = tolerance
        self.margin_ratio = margin_ratio
        self.reset_after_fail_cycles = max(1, reset_after_fail_cycles)
        self.locked: Optional[List[RatioBox]] = None
        self._samples: List[List[RatioBox]] = []
        self._fail_cycles = 0

    @property
    def is_locked(self) -> bool:
        return self.locked is not None

    @property
    def progress(self) -> int:
        return len(self._samples)

    def reset(self) -> None:
        self.locked = None
        self._samples = []
        self._fail_cycles = 0

    def load(self, boxes: List[RatioBox]) -> None:
        """保存済みのロック(設定のlocked_rois)から、ロック済み状態で開始する。"""
        self.locked = [tuple(b) for b in boxes]  # type: ignore[misc]
        self._samples = []
        self._fail_cycles = 0

    def _consistent(self, a: List[RatioBox], b: List[RatioBox]) -> bool:
        return all(
            abs(p - q) <= self.tolerance
            for box_a, box_b in zip(a, b)
            for p, q in zip(box_a, box_b)
        )

    def observe(
        self, boxes_px: Optional[List[Tuple[int, int, int, int]]], width: int, height: int
    ) -> bool:
        """未ロック時の検出結果を渡す(select_best_rows後のexpected個、または
        検出失敗ならNone)。この呼び出しでロックが成立したらTrueを返す。"""
        if self.is_locked:
            return False
        if boxes_px is None or len(boxes_px) != self.expected:
            self._samples = []  # 「連続」で揃うことが条件なのでリセット
            return False
        norm: List[RatioBox] = [
            (x1 / width, y1 / height, x2 / width, y2 / height) for x1, y1, x2, y2 in boxes_px
        ]
        if self._samples and self._consistent(self._samples[-1], norm):
            self._samples.append(norm)
        else:
            self._samples = [norm]
        if len(self._samples) < self.required:
            return False
        locked: List[RatioBox] = []
        for i in range(self.expected):
            # 桁数の増減で幅が変わっても収まるよう、サンプルの和集合+余白
            x1 = min(smp[i][0] for smp in self._samples)
            y1 = min(smp[i][1] for smp in self._samples)
            x2 = max(smp[i][2] for smp in self._samples)
            y2 = max(smp[i][3] for smp in self._samples)
            margin = (x2 - x1) * self.margin_ratio
            locked.append((max(0.0, x1 - margin), y1, min(1.0, x2 + margin), y2))
        self.locked = locked
        self._samples = []
        self._fail_cycles = 0
        return True

    def note_cycle_result(self, any_valid_read: bool) -> bool:
        """ロック中の1周期ごとに、少なくとも1スロットが有効に読めたかを渡す。
        読めない周期がreset_after_fail_cycles連続したらロックを解除しTrueを返す。"""
        if not self.is_locked:
            return False
        if any_valid_read:
            self._fail_cycles = 0
            return False
        self._fail_cycles += 1
        if self._fail_cycles >= self.reset_after_fail_cycles:
            self.reset()
            return True
        return False


def ratio_boxes_to_rois(boxes: List[RatioBox]) -> Dict[str, Roi]:
    return {slot: Roi(x1=b[0], y1=b[1], x2=b[2], y2=b[3]) for slot, b in zip(FULL_ROW_ORDER, boxes)}


def rois_to_config_dict(boxes: List[RatioBox]) -> Dict[str, Dict[str, float]]:
    return {
        slot: {"x1": round(b[0], 4), "y1": round(b[1], 4), "x2": round(b[2], 4), "y2": round(b[3], 4)}
        for slot, b in zip(FULL_ROW_ORDER, boxes)
    }


def parse_locked_rois(value: object) -> Optional[List[RatioBox]]:
    """設定のlocked_rois({slot: {x1,y1,x2,y2}})をFULL_ROW_ORDER順の比率リストにする。
    形式が不正・不足ならNone(=ロック無しで通常通り検出から始める)。"""
    if not isinstance(value, dict):
        return None
    try:
        boxes = [
            (float(value[s]["x1"]), float(value[s]["y1"]), float(value[s]["x2"]), float(value[s]["y2"]))
            for s in FULL_ROW_ORDER
        ]
    except (KeyError, TypeError, ValueError):
        return None
    if any(not (0.0 <= v <= 1.0) for b in boxes for v in b):
        return None
    return boxes


def preprocess_for_ocr(cv2, crop, scale_factor: float = 2.0, enhance=None):
    """OCR精度を上げるための前処理(グレースケール化+二値化+拡大)。

    【2026-10-10追加】enhance(ocr_camera_quality.EnhanceSettings)を渡すと、ピンぼけ・低コントラスト
    対策として 自動レベル→拡大(Lanczos)→CLAHE→アンシャープ を経てから二値化する。None=従来どおり。

    タブレット画面の反射・照明ムラでコントラストが低いとTesseractが
    誤読しやすいため、大津の二値化で白黒をはっきりさせる。数字が小さい
    ままだと認識精度が落ちるため拡大してから渡す。最適な拡大倍率は実際の
    カメラ・タブレット・設置距離次第で変わるため、固定値ではなく設定
    (config["ocr_scale_factor"]、既定2.0)で調整できるようにしてある
    (2026-09-25変更、確認用ダッシュボードの「設定」欄からも変更可能)。
    """
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    if enhance is not None and cq is not None and enhance.enabled:
        gray = cq.enhance_gray_for_ocr(cv2, np, gray, scale_factor, enhance)
    else:
        gray = cv2.resize(gray, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_CUBIC)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return thresh


_DIGITS_RE = re.compile(r"\d+")


def segment_digits(cv2, binary_img) -> List["np.ndarray"]:
    """二値化済み画像(preprocess_for_ocrの出力)から、1文字ずつの矩形を
    輪郭検出で切り出し、左から右へ並べて返す(Edge TPU分類器は1桁ずつしか
    判定できないため)。文字の高さの30%未満の小さな輪郭はノイズとして除外する。
    """
    inverted = cv2.bitwise_not(binary_img)  # 文字を白、背景を黒にしてfindContoursへ
    contours, _ = cv2.findContours(inverted, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h_img = binary_img.shape[0]
    boxes = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if h < h_img * 0.3:
            continue
        boxes.append((x, y, w, h))
    boxes.sort(key=lambda b: b[0])
    return [binary_img[y : y + h, x : x + w] for x, y, w, h in boxes]


def _pad_and_resize_crop(cv2, crop, canvas_size: int = DIGIT_CANVAS_SIZE, padding_ratio: float = DIGIT_PADDING_RATIO):
    """1文字分の切り出し画像を、train_digit_model.py側の学習データと同じ
    「正方形に余白付きでパディング→canvas_sizeへ縮小」に正規化する。
    """
    h, w = crop.shape[:2]
    side = max(h, w)
    pad = int(side * padding_ratio)
    canvas = np.full((side + pad * 2, side + pad * 2), 255, dtype=np.uint8)
    y_off = pad + (side - h) // 2
    x_off = pad + (side - w) // 2
    canvas[y_off : y_off + h, x_off : x_off + w] = crop
    return cv2.resize(canvas, (canvas_size, canvas_size), interpolation=cv2.INTER_CUBIC)


_PROBE_SCRIPT = """
import sys
from ai_edge_litert.interpreter import Interpreter, load_delegate
import numpy as np

model_path = sys.argv[1]
delegate = load_delegate("libedgetpu.so.1")
interp = Interpreter(model_path=model_path, experimental_delegates=[delegate])
interp.allocate_tensors()
inp = interp.get_input_details()[0]
dummy = np.zeros(inp["shape"], dtype=inp["dtype"])
interp.set_tensor(inp["index"], dummy)
interp.invoke()
print("PROBE_OK")
"""


def _probe_edgetpu_delegate(model_path: str, timeout_sec: float = 15.0) -> bool:
    """ai-edge-litert + libedgetpuの組み合わせが、このマシンで実際に安全に
    動くかを子プロセスで検査する。

    既知の問題(2026-09-24確認): 組み合わせによってはPythonの例外ではなく
    プロセスごとのセグメンテーション違反(SIGSEGV)を起こす場合があり、
    その場合は本体プロセス側のtry/exceptでは検知できず、フォールバック
    処理まで巻き込んでプロセスごと落ちてしまう。子プロセスとして隔離して
    実行し、正常終了(returncode 0)した場合のみ本体プロセスでも同じ経路を
    使う。
    """
    import subprocess
    import sys as _sys

    try:
        result = subprocess.run(
            [_sys.executable, "-c", _PROBE_SCRIPT, model_path],
            capture_output=True,
            timeout=timeout_sec,
        )
    except Exception as exc:  # noqa: BLE001 - 検査自体が失敗した場合もEdge TPU不可扱いにする
        logger.warning("Edge TPUデリゲートの事前検査に失敗しました: %s", exc)
        return False

    if result.returncode != 0:
        logger.warning(
            "Edge TPUデリゲートの事前検査がクラッシュ/失敗しました"
            "(returncode=%s)。CPU推論にフォールバックします。stderr末尾: %s",
            result.returncode,
            result.stderr.decode("utf-8", errors="replace")[-500:],
        )
        return False
    return b"PROBE_OK" in result.stdout


class EdgeTpuDigitRecognizer:
    """Edge TPU向けにコンパイル済みの数字分類モデル(train_digit_model.py参照)
    による1桁ずつの推論。

    【重要】学習データは実際のタブレット撮影画像ではなく、フォント・
    幾何変形・二値化ノイズをランダム化した合成データのため、実機での精度は
    未検証。実際にRaspberry Pi+カメラ+本物のタブレットで動作確認できる
    ようになったら、必ず精度を確認すること。
    """

    def __init__(self, model_path: str) -> None:
        # 【2026-09-24追記】pycoral/tflite_runtimeはGoogleが実質的に開発終了して
        # おり、最近のPython(3.10+)・新しめのDebian/Ubuntu向けの配布物が無い
        # (Debian 13/Python 3.13の実機で確認済み)。代わりにGoogle公式の後継
        # ライブラリ`ai-edge-litert`(pip install可)+ libedgetpu.so.1(apt:
        # libedgetpu1-std)の組み合わせで同じことができるので、これも試す。
        # pycoralが使える環境(古いJetson等)ではそちらを優先し続ける。
        #
        # 【重要】ai-edge-litert(2026-09-24時点の最新2.1.0/2.2.0)と
        # libedgetpu.so.1(Google公式、2021年頃のビルドのまま更新停止)の
        # 組み合わせは、Debian 13/Python 3.13実機で検証したところ
        # Interpreter生成時に「Pythonの例外ではなくプロセスごとクラッシュする
        # セグメンテーション違反」を起こすことを確認した(try/exceptでは
        # 捕まえられない)。そのため本体プロセスで直接は試さず、まず別プロセス
        # で「デリゲート込みで実際に1回推論できるか」を安全に検査
        # (_probe_edgetpu_delegate)し、そちらが正常終了した場合のみ本体
        # プロセスでも同じ経路を使う。異常終了(クラッシュ)ならCPUへ
        # 安全にフォールバックする。
        self._interpreter = None
        try:
            from pycoral.utils.edgetpu import make_interpreter  # type: ignore

            self._interpreter = make_interpreter(model_path)
        except Exception:
            pass

        if self._interpreter is None and _probe_edgetpu_delegate(model_path):
            try:
                from ai_edge_litert.interpreter import Interpreter, load_delegate  # type: ignore

                delegate = load_delegate("libedgetpu.so.1")
                self._interpreter = Interpreter(
                    model_path=model_path, experimental_delegates=[delegate]
                )
            except Exception:
                pass

        if self._interpreter is None:
            # Coral実機/Edge TPUランタイムが無い開発環境でも動作確認できるよう、
            # CPUでのTFLite推論にフォールバックする。ただしEdge TPU向けに
            # コンパイル済みのモデル(edgetpu-custom-opを含む)はCPU側の
            # インタプリタでは読めないため、"_edgetpu.tflite"→".tflite"
            # (train_digit_model.py/edgetpu_compilerの命名規則)へ機械的に
            # 変換した、量子化のみでEdge TPU向けコンパイルはしていない
            # モデルを代わりに読む。
            cpu_model_path = model_path.replace("_edgetpu.tflite", ".tflite")
            try:
                from tflite_runtime.interpreter import Interpreter  # type: ignore
            except ImportError:
                try:
                    from tensorflow.lite.python.interpreter import Interpreter  # type: ignore
                except ImportError:
                    from ai_edge_litert.interpreter import Interpreter  # type: ignore
            self._interpreter = Interpreter(model_path=cpu_model_path)
            logger.warning(
                "pycoral/ai-edge-litert経由でEdge TPUデバイスが使えないため、"
                "CPUでのTFLite推論(%s)にフォールバックします"
                "(精度は同じだが速度は出ません)",
                cpu_model_path,
            )
        self._interpreter.allocate_tensors()
        self._input_index = self._interpreter.get_input_details()[0]["index"]
        self._output_index = self._interpreter.get_output_details()[0]["index"]

    def predict_digit(self, digit_img_32x32_uint8: "np.ndarray") -> int:
        data = digit_img_32x32_uint8.reshape(1, DIGIT_CANVAS_SIZE, DIGIT_CANVAS_SIZE, 1).astype(np.uint8)
        self._interpreter.set_tensor(self._input_index, data)
        self._interpreter.invoke()
        output = self._interpreter.get_tensor(self._output_index)[0]
        return int(np.argmax(output))


class DockerEdgeTpuDigitRecognizer:
    """実際にEdge TPUで推論する常駐Dockerコンテナ(coral_tpu_retry/server.py、
    公式pycoralが動くDebian bullseye環境)へHTTPで問い合わせる版。

    【背景】このRaspberry Pi(Debian 13/Python 3.13)では、`EdgeTpuDigitRecognizer`
    が試すpycoral/ai-edge-litert経由の直接推論はいずれも使えない
    (2026-09-24調査、ai-edge-litertはlibedgetpuとの組み合わせでクラッシュする)。
    2026-09-25、coral_tpu_retry/配下のDocker戦略で公式pycoralが実際に動く
    ことを確認できたため、そちらへ処理を投げる形にした
    (`coral_tpu_retry/start_docker_coral_server.sh`で起動した常駐サーバー、
    既定`http://127.0.0.1:5005/predict`)。`predict_digit()`のインターフェースを
    `EdgeTpuDigitRecognizer`と揃えてあるので、`recognize_digits_edgetpu()`側は
    どちらのクラスが渡されても同じように使える。
    """

    def __init__(self, server_url: str, timeout_sec: float = 3.0) -> None:
        self._server_url = server_url.rstrip("/")
        self._timeout_sec = timeout_sec
        # 起動時に一度だけ疎通確認しておく(サーバー未起動なら早期にエラーにして
        # build_recognizer()側でtesseractへフォールバックさせる)。
        resp = requests.get(f"{self._server_url}/health", timeout=timeout_sec)
        resp.raise_for_status()

    def predict_digit(self, digit_img_32x32_uint8: "np.ndarray") -> int:
        data = digit_img_32x32_uint8.astype(np.uint8).tobytes()
        resp = requests.post(
            f"{self._server_url}/predict",
            data=data,
            headers={"Content-Type": "application/octet-stream"},
            timeout=self._timeout_sec,
        )
        resp.raise_for_status()
        return int(resp.json()["digit"])


def recognize_digits_tesseract(cv2, crop, scale_factor: float = 2.0, enhance=None) -> Optional[int]:
    """1つのROI画像から数字を読み取る(Tesseract版)。"""
    try:
        import pytesseract  # type: ignore
    except ImportError:
        logger.error(
            "pytesseractが未導入です。'pip3 install --break-system-packages pytesseract' "
            "と 'sudo apt install tesseract-ocr' を実行してください。"
        )
        return None

    processed = preprocess_for_ocr(cv2, crop, scale_factor, enhance)
    # psm 7: 1行のテキストとして扱う。数字以外は認識しない(誤読防止)。
    config = "--psm 7 -c tessedit_char_whitelist=0123456789"
    text = pytesseract.image_to_string(processed, config=config)
    match = _DIGITS_RE.search(text)
    if not match:
        return None
    try:
        return int(match.group(0))
    except ValueError:
        return None


def recognize_digits_edgetpu(
    cv2, crop, recognizer: EdgeTpuDigitRecognizer, scale_factor: float = 2.0, enhance=None
) -> Optional[int]:
    """1つのROI画像から数字を読み取る(Edge TPU版)。1文字ずつ切り出して
    分類し、結果を連結する。文字が1つも見つからなければNoneを返す。
    """
    processed = preprocess_for_ocr(cv2, crop, scale_factor, enhance)
    digit_crops = segment_digits(cv2, processed)
    if not digit_crops:
        return None
    digits = []
    for digit_crop in digit_crops:
        normalized = _pad_and_resize_crop(cv2, digit_crop)
        digits.append(str(recognizer.predict_digit(normalized)))
    try:
        return int("".join(digits))
    except ValueError:
        return None


def recognize_digits(
    cv2, crop, recognizer: Optional[EdgeTpuDigitRecognizer] = None, scale_factor: float = 2.0, enhance=None
) -> Optional[int]:
    """1つのROI画像から数字を読み取る。recognizerが渡されればEdge TPUで、
    失敗した場合やrecognizerが無い場合はTesseractで読み取る。
    """
    if recognizer is not None:
        try:
            value = recognize_digits_edgetpu(cv2, crop, recognizer, scale_factor, enhance)
            if value is not None:
                return value
        except Exception as exc:  # noqa: BLE001
            logger.warning(f"Edge TPU推論に失敗、Tesseractへフォールバックします: {exc}")
    return recognize_digits_tesseract(cv2, crop, scale_factor, enhance)


class SlotStabilizer:
    """同じ数字がstable_reads_required回連続で読めるまで確定させない。

    1フレームだけの誤読(反射・遷移中のちらつき等)で誤送信しないための
    デバウンス処理。読み取り値が変わるたびカウンタをリセットする。

    OCRが数字を読み取れなかったフレーム(value=None、反射・ピント外れ等)は
    「情報無し」として無視するだけにし、streakはリセットしない。
    ここでリセットしてしまうと、既に確定済みの値を安定して読み続けている
    最中にたまたま1フレームだけ読み損ねただけで、また
    stable_reads_required回分読み直すまで確定値が(既に分かっているのに)
    戻せなくなる。読み取り値が実際に変わった場合(誤読・表示更新)は
    そちらの分岐でstreakを1から数え直すため、この方式でも別の値への
    切り替わりはこれまで通り検知できる。
    """

    def __init__(self, required: int) -> None:
        self.required = max(1, required)
        self._last_value: Optional[int] = None
        self._streak = 0
        self._confirmed: Optional[int] = None

    def update(self, value: Optional[int]) -> Optional[int]:
        if value is None:
            return self._confirmed
        if value == self._last_value:
            self._streak += 1
        else:
            self._last_value = value
            self._streak = 1
        if self._streak >= self.required:
            self._confirmed = value
        return self._confirmed


class TrendPredictor:
    """過去の確定値の増減ペースから、今後の値を外挿予測する。

    【背景】ユーザー要望「最悪連続で読めないことがあった場合は、それまでの
    増減比から今後の推移を予測する関数を作って、それをもとに計算して推測
    するようにしておいて」(2026-09-25)への対応。タブレットが一時的に
    カメラから外れる・ピントが合わない等でOCRが連続して読み取れない間、
    確定値が完全に止まったままにならないよう、直近の確定値の変化ペース
    (個/秒)から現在時刻での推定値を計算する。

    のこさへんが送るsold_total(累計販売数)は基本的に増える一方の値なので、
    ペースは0未満にはクランプする(タブレット側の表示が何かの理由で一時的に
    減って見えた場合の異常なペースをそのまま将来へ延長しないため)。
    """

    def __init__(self, history_size: int = 6) -> None:
        self._history: "deque[Tuple[float, int]]" = deque(maxlen=max(2, history_size))

    def add_sample(self, timestamp: float, value: int) -> None:
        self._history.append((timestamp, value))

    def predict(self, at_timestamp: float) -> Optional[int]:
        if len(self._history) < 2:
            return None  # 傾向を計算するにはサンプルが最低2点必要
        t0, v0 = self._history[0]
        t1, v1 = self._history[-1]
        dt = t1 - t0
        if dt <= 0:
            return None
        rate = max(0.0, (v1 - v0) / dt)  # 個/秒
        elapsed = at_timestamp - t1
        if elapsed <= 0:
            return v1
        return int(round(v1 + rate * elapsed))


def fetch_slot_info(base_url: str, auth: Tuple[str, str]) -> Dict[str, Dict[str, object]]:
    """のこさへん本体の`/api/state`から、A/B/Cに対応する本日のメニュー名と、
    残数入力画面(web_ui_node.py)で最初に入力された「初期残数」(initial)を
    取得する。

    残数入力画面の1〜3番目に入力されたメニューが、そのままA/B/Cに対応する
    運用(SLOT_ORDERと揃えること)。initialはOCRの明らかな誤読
    (2026-09-25確認: タブレット縁の影を数字と誤認して"178"→"1781"のように
    桁が増える等)を弾くための妥当性チェックに使う(run_loop参照)。
    """
    resp = requests.get(f"{base_url}/api/state", auth=auth, timeout=10)
    resp.raise_for_status()
    data = resp.json()
    items = (data.get("menu_state") or {}).get("items") or []
    info: Dict[str, Dict[str, object]] = {}
    for slot, item in zip(SLOT_ORDER, items):
        name = str(item.get("name", "")).strip()
        if not name:
            continue
        entry: Dict[str, object] = {"name": name}
        initial = item.get("initial")
        if isinstance(initial, (int, float)) and initial > 0:
            entry["initial"] = int(initial)
        info[slot] = entry
    return info


def post_sold_total(
    base_url: str, auth: Tuple[str, str], menu: str, sold_total: int, estimated: bool = False
) -> None:
    """のこさへん本体へ累計販売数を送る。

    estimated=TrueはOCRが連続して読み取れず、TrendPredictorの外挿予測値を
    使った送信であることを示す(2026-09-25追加)。本体側(inventory_manager.py)
    がconsumption_logのestimated列に記録し、消費推移グラフで実測と区別
    できるようにするためのフラグ。
    """
    resp = requests.post(
        f"{base_url}/api/sold_total",
        json={
            "menu": menu,
            "sold_total": sold_total,
            "source": "tablet_ocr_reader",
            "estimated": bool(estimated),
        },
        auth=auth,
        timeout=10,
    )
    resp.raise_for_status()


def run_calibrate(config: Dict) -> None:
    """ROI位置合わせ用に、10%刻みのグリッド線を重ねた1枚を保存するだけのモード。

    保存された画像を見ながら、tablet_ocr_config.example.jsonのrois
    (x1/y1/x2/y2、0.0〜1.0の比率)を実際のタブレット表示位置に合わせて
    調整する。
    """
    import cv2  # type: ignore

    cal_profile = select_camera_profile(config)
    cal_params = (
        dict(resolve_camera_profiles(config)[cal_profile], size_fallback=True)
        if cal_profile else legacy_open_params(config)
    )
    cap, _cal_info = open_camera_ex(
        cv2, config["camera_index"], cal_params["width"], cal_params["height"], cal_params["fourcc"],
        cal_params["capture_fps"], cal_params["sharpness"], cal_params["size_fallback"],
    )
    try:
        ok, frame = cap.read()
    finally:
        cap.release()
    if not ok or frame is None:
        logger.error("キャリブレーション用の撮影に失敗しました")
        return
    # frame_rotate_degの回転後の画像にグリッド・ROI枠を重ねる(roisは回転後の座標のため)
    frame = rotate_frame(cv2, frame, normalize_rotate_deg(config.get("frame_rotate_deg", 0)))

    h, w = frame.shape[0], frame.shape[1]
    for i in range(1, 10):
        x = int(w * i / 10)
        y = int(h * i / 10)
        cv2.line(frame, (x, 0), (x, h), (0, 255, 0), 1)
        cv2.line(frame, (0, y), (w, y), (0, 255, 0), 1)
        cv2.putText(frame, f"{i/10:.1f}", (x + 2, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.putText(frame, f"{i/10:.1f}", (2, y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    # 現在の設定のROI枠も重ねて描画し、そのまま位置確認できるようにする。
    for slot, roi_dict in config.get("rois", {}).items():
        roi = Roi(**roi_dict)
        x1, y1 = int(roi.x1 * w), int(roi.y1 * h)
        x2, y2 = int(roi.x2 * w), int(roi.y2 * h)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.putText(frame, slot, (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    out_path = os.path.expanduser("~/tablet_ocr_calibrate.jpg")
    cv2.imwrite(out_path, frame)
    logger.info(f"キャリブレーション画像を保存しました: {out_path}")
    logger.info("この画像を見ながら設定ファイルのroisを調整してください(比率0.0〜1.0)。")


_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def _resolve_config_path(path: str) -> str:
    """設定ファイル中の相対パスを解決する。

    既定値("./digit_model/...")はこのスクリプトと同じフォルダに置く想定
    だが、カレントディレクトリはスクリプトの起動場所によって変わるため、
    そのままだとCWD相対で解決されて見つからないことがある(2026-09-15
    レビューで発見)。相対パスは常にこのスクリプト自身のフォルダを基準に
    解決する。
    """
    expanded = os.path.expanduser(path)
    if os.path.isabs(expanded):
        return expanded
    return os.path.join(_SCRIPT_DIR, expanded)


def build_recognizer(config: Dict):
    mode = str(config.get("recognizer", "tesseract")).lower()
    if mode not in ("edgetpu", "edgetpu_docker"):
        return None

    if mode == "edgetpu_docker":
        server_url = str(config.get("edgetpu_docker_url", "http://127.0.0.1:5005"))
        try:
            recognizer = DockerEdgeTpuDigitRecognizer(server_url)
            logger.info(f"Edge TPU推論サーバー(Docker)に接続しました: {server_url}")
            return recognizer
        except Exception as exc:  # noqa: BLE001
            logger.warning(
                f"Edge TPU推論サーバー(Docker、{server_url})に接続できません"
                f"(coral_tpu_retry/start_docker_coral_server.shで起動済みか確認してください): {exc}"
                " tesseractにフォールバックします。"
            )
            return None

    raw_path = str(config.get("digit_model_path", ""))
    model_path = _resolve_config_path(raw_path) if raw_path else ""
    if not model_path or not os.path.exists(model_path):
        logger.warning(
            f"recognizer=edgetpuですが、モデルファイルが見つかりません: {model_path}"
            " tesseractにフォールバックします。"
        )
        return None
    try:
        recognizer = EdgeTpuDigitRecognizer(model_path)
        logger.info(f"Edge TPU数字認識モデルを読み込みました: {model_path}")
        return recognizer
    except Exception as exc:  # noqa: BLE001
        logger.warning(f"Edge TPUモデルの読み込みに失敗、tesseractにフォールバックします: {exc}")
        return None


class PreviewState:
    """カメラ映像(ROI枠・現在の読み取り値を重ねて描画したJPEG)と、スロット
    ごとの最新の読み取り状況を保持する。run_loop()側(メインスレッド)が
    毎周期書き込み、HTTPハンドラ側(ThreadingHTTPServerによりリクエスト毎に
    別スレッド)が読み出すため、dashboard_common.pyの各コントローラと同じく
    threading.Lockで保護する。
    """

    def __init__(
        self, scale_factor: float = 2.0, rotate_deg: int = 0, focus_thresholds: Optional[Dict] = None,
        focus_warn_after_sec: float = 20.0,
    ) -> None:
        self._lock = threading.Lock()
        self._frame_jpeg: Optional[bytes] = None
        self._updated_at: Optional[float] = None
        self._readings: Dict[str, dict] = {}
        self._raw_frame: Optional["np.ndarray"] = None
        self._dynamic_rois: Dict[str, Roi] = {}
        # OCR前処理の拡大倍率。実際に固定設置してみないと最適値が分からない
        # ため、確認用ダッシュボードの「設定」欄から実行中に変更できる
        # (2026-09-25追加)。run_loop側は周期ごとにget_scale_factor()を
        # 読むので、変更は次の周期からすぐ反映される。
        self._scale_factor = scale_factor
        # 撮影直後のフレーム回転角(2026-10-09追加)。ダッシュボードの「設定」欄から
        # 実行中に切り替えられる。撮影スレッドが毎フレームget_rotate_deg()を読む。
        self._rotate_deg = rotate_deg
        # カメラのプロファイル(2026-10-09追加)。_cam_profileは実際に適用済みの名前
        # (None=camera_profileの無い従来動作)、_cam_pendingは切替要求中の名前。
        # 切替自体はカメラの所有者である撮影スレッドが行う(take_camera_request)。
        self._cam_profile: Optional[str] = None
        self._cam_profiles: Dict[str, Dict] = {}
        self._cam_params: Optional[Dict] = None
        self._cam_actual: Dict = {}
        self._cam_pending: Optional[str] = None
        self._cam_error: Optional[str] = None
        self._cam_measured_fps: Optional[float] = None
        # 直近にプレビュー画像(/api/preview.jpg)が要求された時刻。
        # 2026-09-28追加: 誰もダッシュボードを見ていない間は撮影スレッドの
        # 描画・JPEGエンコードを間引いてCPUを節約するために使う(ユーザー
        # 要望「軽くできないか検討して」対応)。0.0のままなら「まだ誰も
        # 見ていない」を意味する。
        self._last_viewed_at: float = 0.0
        # roi_mode="auto_lock"の状態表示と、「ROIを再検出」ボタンの要求
        # (2026-10-02追加)。
        self._roi_lock: dict = {"mode": None, "locked": False, "progress": 0, "required": 0}
        self._relock_requested = False
        # 【2026-10-10追加】ピント(鮮鋭度)モニタ・直近フレーム履歴・画質の自動調整の状態
        # (ocr_camera_quality.py。モジュールが無い配備では全て無効=None)。
        self.focus = (
            cq.FocusMonitor(thresholds=focus_thresholds, warn_after_sec=focus_warn_after_sec) if cq else None
        )
        self.history = cq.FrameHistory() if cq else None
        self._at_state: Dict = {
            "state": "idle", "action": None, "progress": {"done": 0, "total": 0, "label": ""},
            "result": None, "message": "", "profile": None, "started_at": None, "finished_at": None,
        }
        self._at_request: Optional[str] = None
        self._at_cancel = False
        self._tuning_saved: Dict[str, Dict] = {}

    def set_roi_lock(self, mode: str, locked: bool, progress: int, required: int) -> None:
        with self._lock:
            self._roi_lock = {"mode": mode, "locked": locked, "progress": progress, "required": required}

    def snapshot_roi_lock(self) -> dict:
        with self._lock:
            return dict(self._roi_lock)

    def request_relock(self) -> None:
        with self._lock:
            self._relock_requested = True

    def consume_relock_request(self) -> bool:
        with self._lock:
            requested = self._relock_requested
            self._relock_requested = False
            return requested

    def mark_viewed(self) -> None:
        with self._lock:
            self._last_viewed_at = time.time()

    # 【2026-09-30追加】認識に実際に渡している前処理後の画像(各スロットのROIを
    # 拡大→二値化したもの)を並べた1枚のJPEG。拡大倍率の効果は内部の画像にしか
    # 出ず、カメラ映像やROI枠は変わらないため「変えても変わらない」ように見えて
    # いた。閲覧中だけrun_loop側が作る(見ていない間は作らない)。
    def update_processed(self, jpeg_bytes: Optional[bytes]) -> None:
        with self._lock:
            self._processed_jpeg = jpeg_bytes
            self._processed_at = time.time() if jpeg_bytes is not None else 0.0

    def snapshot_processed(self) -> Optional[bytes]:
        with self._lock:
            return getattr(self, "_processed_jpeg", None)

    def discard_processed_if_stale(self, max_age_sec: float) -> None:
        """作成からmax_age_sec秒より古い前処理後画像を破棄する。
        見ていない間は更新しないため、最初の1枚を放置すると古い画像が
        「現在の認識画像」として出続けてしまう。破棄するとrun_loop側が
        (画像が無い状態を検知して)次の周期で1枚作り直す。"""
        with self._lock:
            if getattr(self, "_processed_jpeg", None) is None:
                return
            if time.time() - getattr(self, "_processed_at", 0.0) > max_age_sec:
                self._processed_jpeg = None
                self._processed_at = 0.0

    def seconds_since_viewed(self) -> float:
        with self._lock:
            if self._last_viewed_at <= 0.0:
                return float("inf")
            return time.time() - self._last_viewed_at

    def get_scale_factor(self) -> float:
        with self._lock:
            return self._scale_factor

    def set_scale_factor(self, value: float) -> None:
        with self._lock:
            self._scale_factor = value

    def get_rotate_deg(self) -> int:
        with self._lock:
            return self._rotate_deg

    def set_rotate_deg(self, value: int) -> bool:
        """回転角を変更する。変わった場合は、古い角度の撮影フレームとROIを捨てて
        ROIの再検出を要求し(/api/relockと同じ扱い)Trueを返す。"""
        with self._lock:
            if value == self._rotate_deg:
                return False
            self._rotate_deg = value
            self._raw_frame = None
            self._dynamic_rois = {}
            self._relock_requested = True
        self.reset_quality_history()
        return True

    # ---- カメラのプロファイル(2026-10-09追加) ----
    def init_camera(self, profile: Optional[str], profiles: Dict[str, Dict], actual: Dict) -> None:
        with self._lock:
            self._cam_profile = profile
            self._cam_profiles = {k: dict(v) for k, v in profiles.items()}
            self._cam_params = dict(profiles[profile]) if profile else None
            self._cam_actual = dict(actual)

    def get_camera_profile(self) -> Optional[str]:
        with self._lock:
            return self._cam_profile

    def get_camera_params(self) -> Optional[Dict]:
        """適用中のプロファイルの有効値(従来動作ならNone)。"""
        with self._lock:
            return None if self._cam_params is None else dict(self._cam_params)

    def request_camera_profile(self, name: str) -> bool:
        """切替を要求する。既にそのプロファイルで動作中なら何もしない(False)。"""
        with self._lock:
            if name == self._cam_profile and self._cam_pending is None:
                return False
            self._cam_pending = name
            self._cam_error = None
            return True

    def take_camera_request(self) -> Optional[Tuple[str, Dict]]:
        with self._lock:
            name = self._cam_pending
            if name is None or name not in self._cam_profiles:
                self._cam_pending = None
                return None
            return name, dict(self._cam_profiles[name], size_fallback=True)

    def finish_camera_switch(self, name: str, actual: Dict) -> None:
        """切替成功。古い解像度のフレーム・ROIを捨て、再検出を要求する(/api/relockと同じ扱い)。"""
        with self._lock:
            self._cam_profile = name
            self._cam_params = dict(self._cam_profiles[name])
            self._cam_actual = dict(actual)
            self._cam_pending = None
            self._cam_error = None
            self._cam_measured_fps = None
            self._raw_frame = None
            self._dynamic_rois = {}
            self._relock_requested = True
        self.reset_quality_history()

    def fail_camera_switch(self, error: str, actual: Dict) -> None:
        """切替失敗(元のプロファイルのまま)。"""
        with self._lock:
            self._cam_pending = None
            self._cam_error = error
            if actual:
                self._cam_actual = dict(actual)

    def set_measured_fps(self, value: Optional[float]) -> None:
        with self._lock:
            self._cam_measured_fps = value

    # ---- 画質の自動調整(2026-10-10追加) ----
    AUTOTUNE_ACTIONS = ("start", "revert", "cancel")

    def request_autotune(self, action: str) -> Tuple[bool, str]:
        """POST /api/autotune。(受理したか, 理由)。実行は撮影スレッド(take_autotune_request)。"""
        with self._lock:
            busy = self._at_state["state"] in ("requested", "running", "reverting")
            if action == "cancel":
                if not busy:
                    return False, "実行中の自動調整はありません"
                if self._at_state["state"] == "requested":
                    self._at_request = None
                    self._at_state.update(state="idle", message="中断しました")
                else:
                    self._at_cancel = True
                return True, ""
            if busy:
                return False, "自動調整(または元に戻す処理)を実行中です"
            if self._cam_pending is not None:
                return False, "カメラのプロファイル切替中です。完了してから実行してください"
            self._at_request = action
            self._at_cancel = False
            self._at_state.update(
                state="requested", action=action, result=None, message="開始を待っています",
                progress={"done": 0, "total": 0, "label": ""}, started_at=time.time(), finished_at=None,
            )
            return True, ""

    def take_autotune_request(self) -> Optional[str]:
        with self._lock:
            action = self._at_request
            if action is None:
                return None
            self._at_request = None
            self._at_state.update(
                state="running" if action == "start" else "reverting", profile=self._cam_profile,
                message="画質を自動調整しています(完了まで読み取りの確定を一時停止します)"
                if action == "start" else "元の設定へ戻しています",
            )
            return action

    def autotune_busy(self) -> bool:
        with self._lock:
            return self._at_state["state"] in ("requested", "running", "reverting")

    def autotune_cancelled(self) -> bool:
        with self._lock:
            return self._at_cancel

    def set_autotune_progress(self, done: int, total: int, label: str) -> None:
        with self._lock:
            self._at_state["progress"] = {"done": int(done), "total": int(total), "label": str(label)}

    def finish_autotune(self, action: str, result: Optional[Dict], state: str, message: str) -> None:
        with self._lock:
            self._at_cancel = False
            self._at_state.update(state=state, result=result, message=message, finished_at=time.time())
            if result is not None:
                self._at_state["progress"] = {
                    "done": self._at_state["progress"].get("total", 0), "total": self._at_state["progress"].get("total", 0),
                    "label": "完了",
                }

    def set_tuning_saved(self, tuning: Dict[str, Dict]) -> None:
        with self._lock:
            self._tuning_saved = {k: dict(v) for k, v in tuning.items()}

    def _snapshot_autotune_locked(self) -> Dict:
        out = {k: (dict(v) if isinstance(v, dict) else v) for k, v in self._at_state.items()}
        prof = cq.tuning_profile_key(self._cam_profile) if cq else None
        rec = self._tuning_saved.get(prof) if prof else None
        out["available"] = cq is not None
        out["saved"] = None if not rec else {
            "profile": prof, "applied": dict(rec.get("applied", {})), "original": dict(rec.get("original", {})),
            "tuned_at": rec.get("tuned_at", ""),
        }
        return out

    def snapshot_autotune(self) -> Dict:
        with self._lock:
            return self._snapshot_autotune_locked()

    def snapshot_focus(self) -> Dict:
        """GET /api/focus.json: ピント(鮮鋭度)・明るさ・ガイダンス + 自動調整の状態の要約。"""
        out: Dict = {"available": self.focus is not None}
        if self.focus is not None:
            out.update(self.focus.snapshot())
        with self._lock:
            out["autotune_state"] = self._at_state["state"]
        return out

    def diagnose(self) -> Dict:
        """POST /api/diagnose: 現在のフレームで解像度・ピント・明るさを診断する(HTTPスレッドで実行)。"""
        if cq is None:
            return {"ok": False, "warnings": ["画質モジュール(ocr_camera_quality.py)が見つかりません"]}
        import cv2  # type: ignore

        frame = self.get_raw_frame()
        return cq.diagnose(cv2, np, frame, self.focus, self.get_focus_box())

    def set_focus_box(self, box: Optional[Tuple[float, float, float, float]]) -> None:
        """鮮鋭度を測る領域(ROI外接枠の比率。Noneなら画面中央)。撮影スレッドが更新する。"""
        with self._lock:
            self._focus_box = box

    def get_focus_box(self) -> Optional[Tuple[float, float, float, float]]:
        with self._lock:
            return getattr(self, "_focus_box", None)

    def reset_quality_history(self) -> None:
        """回転・プロファイル切替・カメラ設定変更の後: 古い条件のフレーム履歴と鮮鋭度の平均/ピークを捨てる。"""
        if self.history is not None:
            self.history.clear()
        if self.focus is not None:
            self.focus.reset()

    def snapshot_settings(self) -> Dict:
        """GET /api/settings.json の内容(回転・倍率・カメラのプロファイル)。"""
        with self._lock:
            actual = dict(self._cam_actual)
            if self._cam_measured_fps is not None:
                actual["measured_fps"] = round(self._cam_measured_fps, 1)
            return {
                "ocr_scale_factor": self._scale_factor,
                "frame_rotate_deg": self._rotate_deg,
                "camera_profile": self._cam_pending or self._cam_profile,
                "camera_profiles": {k: dict(v) for k, v in self._cam_profiles.items()},
                "camera_actual": actual,
                "camera_switching": self._cam_pending is not None,
                "camera_error": self._cam_error,
                "autotune": self._snapshot_autotune_locked(),
            }

    def update_frame(self, jpeg_bytes: bytes) -> None:
        with self._lock:
            self._frame_jpeg = jpeg_bytes
            self._updated_at = time.time()

    def update_raw_frame(self, frame: "np.ndarray", rotate_deg: Optional[int] = None) -> None:
        # OCR側(run_loop)が自分でcap.read()せず、プレビュー用の撮影スレッドが
        # 撮った最新フレームをそのまま使い回すための受け渡し。カメラへの
        # アクセスをこのスレッド1つに一本化する(cv2.VideoCaptureは複数
        # スレッドから同時に.read()されるのを想定していないため)。
        # rotate_degを渡した場合、現在の回転角と違えば(回転切替の最中に古い角度で
        # 回したフレーム)捨てる。
        with self._lock:
            if rotate_deg is not None and rotate_deg != self._rotate_deg:
                return
            self._raw_frame = frame

    def get_raw_frame(self) -> Optional["np.ndarray"]:
        with self._lock:
            return None if self._raw_frame is None else self._raw_frame.copy()

    def update_reading(
        self, slot: str, raw_value: Optional[int], confirmed_value: Optional[int],
        menu_name: Optional[str], sent: bool, estimated: bool = False,
    ) -> None:
        with self._lock:
            self._readings[slot] = {
                "raw_value": raw_value,
                "confirmed_value": confirmed_value,
                "menu_name": menu_name,
                "sent": sent,
                # OCRが連続して読み取れず、TrendPredictorの外挿予測値を
                # confirmed_valueとして使っている場合はTrueになる
                # (2026-09-25追加。ダッシュボードで実測値と区別して表示する)。
                "estimated": estimated,
            }

    def update_dynamic_rois(self, rois: Dict[str, Roi]) -> None:
        # roi_mode="auto_color"のとき、直近の周期でdetect_digit_rows()が
        # 見つけた行位置をプレビュー描画用に共有する(検出に失敗した周期は
        # 呼ばない=直前の枠を描き続け、プレビューが毎周期チラつかないようにする)。
        with self._lock:
            self._dynamic_rois = dict(rois)

    def snapshot_dynamic_rois(self) -> Dict[str, Roi]:
        with self._lock:
            return dict(self._dynamic_rois)

    def snapshot_frame(self) -> Optional[bytes]:
        with self._lock:
            return self._frame_jpeg

    def snapshot_readings(self) -> dict:
        with self._lock:
            return {
                "updated_at": self._updated_at,
                "readings": dict(self._readings),
                "roi_lock": dict(self._roi_lock),
                "frame_rotate_deg": self._rotate_deg,
                "autotune_state": self._at_state["state"],
                "focus": None if self.focus is None else self.focus.snapshot(),
            }


_PREVIEW_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>タブレットOCR 確認画面</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", sans-serif; background:#f4f6f5; color:#16211d; }
  html, body { margin:0; }
  header { box-sizing:border-box; padding:10px 18px; background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; gap:10px; flex-wrap:wrap; }
  header h1 { margin:0; font-size:16px; }
  header a.backlink { color:#cfe9df; text-decoration:none; font-size:13px; }
  main { max-width:820px; margin:0 auto; padding:16px; display:flex; flex-direction:column; gap:16px; }
  section { background:#fff; border:1px solid #dce4df; border-radius:8px; padding:12px; }
  h2 { margin:0 0 8px; font-size:13px; color:#53635c; }
  img#preview { width:100%; border-radius:6px; background:#0c1b16; display:block; }
  table { width:100%; border-collapse:collapse; font-size:13px; }
  th, td { border-bottom:1px solid #e4ebe7; padding:6px 8px; text-align:left; }
  .sent-yes { color:#0f6e56; font-weight:600; }
  .sent-no { color:#8a7a3a; }
  .estimated-badge { display:inline-block; margin-left:6px; padding:1px 6px; border-radius:10px; background:#f0e6fb; color:#6a3fa0; font-size:11px; font-weight:600; }
  .hint { font-size:12px; color:#6b6a64; }
  .settings-row { display:flex; gap:8px; align-items:center; flex-wrap:wrap; }
  .settings-row input[type="number"] { width:80px; padding:6px; font-size:13px; border:1px solid #cfd8d3; border-radius:6px; }
  .settings-row button { padding:7px 14px; font-size:13px; border:0; border-radius:6px; background:#176b56; color:#fff; cursor:pointer; }
  #settingsMsg { font-size:12px; margin-left:4px; }
  .rot-btn { background:#e4ebe7 !important; color:#16211d !important; }
  .rot-btn.active { background:#176b56 !important; color:#fff !important; }
  .cam-btn { background:#e4ebe7 !important; color:#16211d !important; }
  .cam-btn.active { background:#176b56 !important; color:#fff !important; }
  .focus-line { font-size:14px; margin:8px 0 4px; font-weight:600; }
  .focus-bar { position:relative; height:18px; background:#e4ebe7; border-radius:9px; overflow:hidden; }
  .focus-bar .fill { position:absolute; left:0; top:0; bottom:0; width:0; background:#c0392b; transition:width .15s, background .15s; }
  .focus-bar .peak { position:absolute; top:0; bottom:0; width:3px; background:#16211d; left:0; }
  .focus-bar.near .fill { background:#0f9d58; }
  .focus-bar.good .fill { background:#2e9c6a; }
  .focus-bar.fair .fill { background:#d9a21b; }
  .focus-bar.bad .fill { background:#c0392b; }
  .grade-good { color:#0f6e56; } .grade-fair { color:#9a6b00; } .grade-bad { color:#b3261e; }
  .advice { margin:8px 0 0; padding:8px 10px; border-radius:6px; background:#fdecea; color:#8a1c14; font-size:13px; display:none; }
  .warnbox { margin:8px 0 0; padding:8px 10px; border-radius:6px; background:#fff6dd; color:#6b4e00; font-size:13px; display:none; }
  .progress { height:10px; background:#e4ebe7; border-radius:5px; overflow:hidden; margin-top:6px; display:none; }
  .progress .bar { height:100%; width:0; background:#176b56; transition:width .3s; }
  .btn-sub { background:#e4ebe7 !important; color:#16211d !important; }
  .focus-on { background:#b3261e !important; }
</style>
</head>
<body>
<header>
  <h1>タブレットOCR 確認画面</h1>
  <a class="backlink" href="__DASHBOARD_URL__">&larr; 総合ダッシュボードに戻る</a>
</header>
<main>
  <section>
    <h2>カメラ映像(ROI枠・読み取り値を重ねて表示)</h2>
    <img id="preview" src="/api/preview.jpg" alt="カメラ映像" />
    <p class="hint" id="updatedAt">-</p>
    <div class="focus-line" id="focusLine">ピント: -</div>
    <div class="advice" id="focusAdvice"></div>
    <div class="warnbox" id="focusWarn"></div>
  </section>
  <section>
    <h2>ピント合わせ・画質</h2>
    <div class="settings-row">
      <button id="focusModeBtn">ピント合わせモード開始</button>
      <button id="peakResetBtn" class="btn-sub">最高値をリセット</button>
      <label class="hint"><input type="checkbox" id="beepOn" checked /> 音で知らせる</label>
      <button id="diagBtn" class="btn-sub">診断</button>
    </div>
    <div id="focusPanel" style="display:none;margin-top:10px;">
      <div class="focus-bar" id="focusBar"><div class="fill" id="focusFill"></div><div class="peak" id="focusPeakMark"></div></div>
      <p class="hint" id="focusNums">-</p>
    </div>
    <p class="hint">カメラの距離を前後にゆっくり動かして、バーが最も伸びる(最高値に近づくと緑色・音が高くなる)位置を探してください。固定焦点のカメラは近すぎるとぼやけます(目安: 画面から20〜40cm以上)。レンズの回転リングがある機種ならそれでも合わせられます。</p>
    <p class="hint" id="diagOut"></p>
    <div class="settings-row" style="margin-top:10px;">
      <button id="autotuneBtn">画質を自動調整</button>
      <button id="autotuneRevertBtn" class="btn-sub">元に戻す</button>
      <button id="autotuneCancelBtn" class="btn-sub" style="display:none;">中断</button>
      <span id="autotuneMsg" class="hint"></span>
    </div>
    <div class="progress" id="autotuneProgress"><div class="bar" id="autotuneBar"></div></div>
    <p class="hint" id="autotuneDetail"></p>
    <p class="hint">シャープネス・コントラスト・ガンマ・明るさ・逆光補正・ホワイトバランスを少しずつ変えて、数字がいちばんくっきり読める組み合わせを選びます(1分前後)。実行中は読み取りの確定が一時停止します。結果は設定ファイルに保存され、再起動後も再適用されます。カメラは文字(数字の画面)が写る位置に置いてから押してください。</p>
  </section>
  <section>
    <h2>読み取り状況</h2>
    <table>
      <thead><tr><th>スロット</th><th>メニュー名</th><th>読み取り中</th><th>確定値</th><th>送信</th></tr></thead>
      <tbody id="rows"><tr><td colspan="5">読み込み中...</td></tr></tbody>
    </table>
    <div class="settings-row" id="lockRow" style="display:none;margin-top:8px;">
      <span class="hint" id="lockStatus">-</span>
      <button id="relockBtn">ROIを再検出</button>
      <span class="hint" id="relockMsg"></span>
    </div>
  </section>
  <section>
    <h2>設定</h2>
    <div class="settings-row">
      <label for="scaleFactor">OCR拡大倍率</label>
      <input id="scaleFactor" type="number" min="1.0" max="4.0" step="0.1" />
      <button id="applySettingsBtn">適用</button>
      <span id="settingsMsg"></span>
    </div>
    <div class="settings-row" style="margin-top:10px;">
      <label>カメラ</label>
      <button class="cam-btn" data-cam="demo">デモ(滑らか)</button>
      <button class="cam-btn" data-cam="production">本番(高画質)</button>
      <span id="camMsg" class="hint"></span>
    </div>
    <p class="hint" id="camActual">実効: -</p>
    <p class="hint">デモは少し画質を下げて(800x600相当)プレビューを滑らかに、本番は最高画質(1280x960、撮影は数秒に1回でfps低め)にします。切り替えるとカメラを開き直し、ROIは再検出になります。選択は設定ファイルにも保存されます。</p>
    <div class="settings-row" style="margin-top:10px;">
      <label>映像の回転(時計回り)</label>
      <button class="rot-btn" data-rot="0">0°</button>
      <button class="rot-btn" data-rot="90">90°</button>
      <button class="rot-btn" data-rot="180">180°</button>
      <button class="rot-btn" data-rot="270">270°</button>
      <span id="rotMsg" class="hint"></span>
    </div>
    <p class="hint">カメラ映像を取り込んだ直後に回転して正立に戻します。模擬画面を ?rot=90 で表示して撮る場合は270、?rot=-90なら90を選びます。切り替えるとROIは再検出になり、設定ファイルにも保存されます。</p>
    <p class="hint">数字が小さく写る/大きく写りすぎる場合に調整してください。適用すると設定ファイルにも保存され、次回起動時も引き継がれます。倍率はカメラ映像には出ません。下の「認識に渡している画像」で効果を確認してください。</p>
  </section>
  <section>
    <h2>認識に渡している画像(ROIを拡大→二値化したもの、2.5秒ごとに更新)</h2>
    <img id="processed" src="/api/processed.jpg" alt="認識に渡している画像" style="max-width:100%;background:#0c1b16;border-radius:6px;display:block;" />
    <p class="hint">各段の見出しは「スロット 画像サイズ ×倍率」です。数字の形がつぶれず、余計な線や隣の文字が入っていなければ良い状態です(白黒がギザギザなら倍率を上げ、数字が欠けるなら下げます)。</p>
  </section>
</main>
<script>
  (function () {
    const back = document.querySelector("a.backlink");
    const href = back && back.getAttribute("href");
    if (href && href.indexOf("__AUTOHOST__") !== -1) {
      back.setAttribute("href", href.replace("__AUTOHOST__", location.hostname));
    }
  })();
  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    }[ch]));
  }
  // 【2026-10-02修正】前の画像の読み込みが終わる前にsrcを差し替えると、遅い回線では
  // 1枚も表示完了しなくなる(飢餓)。onload/onerrorを待ってから次を予約する。
  // 応答が返らない場合に止まらないよう、保険のタイムアウトも置く。
  function refreshImageLoop(imgId, url, intervalMs) {
    const img = document.getElementById(imgId);
    let timer = null;
    function next(delay) { clearTimeout(timer); timer = setTimeout(load, delay); }
    function load() {
      next(10000); // 保険
      img.src = url + "?t=" + Date.now();
    }
    img.addEventListener("load", () => next(intervalMs));
    img.addEventListener("error", () => next(Math.max(intervalMs, 500)));
    load();
  }
  async function fetchSettings() {
    try {
      const res = await fetch("/api/settings.json");
      const data = await res.json();
      document.getElementById("scaleFactor").value = data.ocr_scale_factor;
      markRotation(data.frame_rotate_deg);
      markCamera(data);
      renderAutotune(data.autotune);
    } catch (e) { /* 通信エラー時は前回表示のまま */ }
  }
  async function applySettings() {
    const msg = document.getElementById("settingsMsg");
    const value = parseFloat(document.getElementById("scaleFactor").value);
    if (!Number.isFinite(value) || value < 1.0 || value > 4.0) {
      msg.textContent = "1.0〜4.0の範囲で入力してください";
      return;
    }
    try {
      const res = await fetch("/api/settings", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({ocr_scale_factor: value}),
      });
      msg.textContent = res.ok ? "適用しました" : "適用に失敗しました";
    } catch (e) {
      msg.textContent = "通信に失敗しました";
    }
    setTimeout(() => { msg.textContent = ""; }, 3000);
  }
  document.getElementById("applySettingsBtn").addEventListener("click", applySettings);
  // 【2026-10-09追加】カメラのプロファイル(デモ/本番)と実効解像度の表示。
  function markCamera(data) {
    document.querySelectorAll(".cam-btn").forEach((b) => {
      b.classList.toggle("active", b.dataset.cam === data.camera_profile);
    });
    const a = data.camera_actual || {};
    let txt = "実効: " + (a.width ? a.width + "x" + a.height : "-") + " " + (a.fourcc || "")
      + " fps=" + (a.fps != null ? a.fps : "-")
      + (a.measured_fps != null ? "(実測" + a.measured_fps + ")" : "");
    if (a.fallback && a.requested) txt += " ※希望" + a.requested.width + "x" + a.requested.height + "が取得できずフォールバック";
    if (data.camera_switching) txt += " (切替中...)";
    if (data.camera_error) txt += " 切替失敗: " + data.camera_error;
    if (!data.camera_profile) txt += " (プロファイル未設定=従来動作)";
    document.getElementById("camActual").textContent = txt;
  }
  async function pollCamera() {
    try {
      const res = await fetch("/api/settings.json");
      const data = await res.json();
      markCamera(data);
      markRotation(data.frame_rotate_deg);
      renderAutotune(data.autotune);
    } catch (e) { /* 通信エラー時は前回表示のまま */ }
  }
  document.querySelectorAll(".cam-btn").forEach((btn) => {
    btn.addEventListener("click", async () => {
      const msg = document.getElementById("camMsg");
      try {
        const res = await fetch("/api/settings", {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({camera_profile: btn.dataset.cam}),
        });
        msg.textContent = res.ok ? "切り替え中です(数秒かかります)" : "切り替えに失敗しました";
        if (res.ok) markCamera(await res.json());
      } catch (e) {
        msg.textContent = "通信に失敗しました";
      }
      setTimeout(() => { msg.textContent = ""; }, 5000);
    });
  });
  setInterval(pollCamera, 2000);
  // 【2026-10-09追加】映像の回転角(0/90/180/270)。
  function markRotation(deg) {
    document.querySelectorAll(".rot-btn").forEach((b) => {
      b.classList.toggle("active", Number(b.dataset.rot) === Number(deg));
    });
  }
  document.querySelectorAll(".rot-btn").forEach((btn) => {
    btn.addEventListener("click", async () => {
      const msg = document.getElementById("rotMsg");
      const deg = Number(btn.dataset.rot);
      try {
        const res = await fetch("/api/settings", {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({frame_rotate_deg: deg}),
        });
        msg.textContent = res.ok ? deg + "°に切り替えました(ROIは再検出します)" : "切り替えに失敗しました";
        if (res.ok) markRotation(deg);
      } catch (e) {
        msg.textContent = "通信に失敗しました";
      }
      setTimeout(() => { msg.textContent = ""; }, 4000);
    });
  });
  // 【2026-10-02追加】押しても何も表示が変わらなかったため、結果を表示する。
  document.getElementById("relockBtn").addEventListener("click", async () => {
    const msg = document.getElementById("relockMsg");
    msg.textContent = "再検出を要求しています...";
    try {
      const res = await fetch("/api/relock", {method: "POST"});
      msg.textContent = res.ok
        ? "再検出を要求しました(次の読み取り周期でロックを解除して位置を検出し直します)"
        : "再検出の要求に失敗しました(HTTP " + res.status + ")";
    } catch (e) {
      msg.textContent = "通信に失敗しました。再検出は要求できていません";
    }
    setTimeout(() => { msg.textContent = ""; }, 6000);
  });
  // ===== 【2026-10-10追加】ピント合わせ・画質の自動調整 =====
  const GRADE_CLS = {good: "grade-good", fair: "grade-fair", bad: "grade-bad"};
  let focusMode = false, audioCtx = null, lastBeepAt = 0, atFastPoll = null;
  function renderFocus(f) {
    if (!f || f.available === false) { document.getElementById("focusLine").textContent = "ピント: (画質モジュール無し)"; return; }
    const line = document.getElementById("focusLine");
    line.className = "focus-line " + (GRADE_CLS[f.grade] || "");
    const sc = f.avg != null ? f.avg : null;
    let t = "ピント: " + (f.grade_label || "測定不可") + (sc != null ? "(スコア " + sc + ")" : "");
    if (f.resolution) t += "  解像度 " + f.resolution.width + "x" + f.resolution.height;
    line.textContent = t;
    const adv = document.getElementById("focusAdvice");
    adv.style.display = f.advice ? "block" : "none"; adv.textContent = f.advice || "";
    const warn = document.getElementById("focusWarn");
    const w = f.warnings || [];
    warn.style.display = w.length ? "block" : "none"; warn.textContent = w.join(" / ");
    if (focusMode) {
      const peak = f.peak != null ? f.peak : 0, v = sc != null ? sc : 0;
      const full = Math.max(120, peak * 1.15);
      const bar = document.getElementById("focusBar");
      document.getElementById("focusFill").style.width = Math.min(100, v / full * 100) + "%";
      document.getElementById("focusPeakMark").style.left = Math.min(99, peak / full * 100) + "%";
      bar.className = "focus-bar " + (f.near_peak ? "near" : (f.grade || ""));
      document.getElementById("focusNums").textContent =
        "現在 " + (sc != null ? sc : "-") + " / 最高 " + (f.peak != null ? f.peak : "-")
        + (f.near_peak ? "  ← 最高に近い位置です!" : "  (カメラをゆっくり前後に動かしてください)");
      beep(v, f.near_peak, full);
    }
  }
  function beep(v, near, full) {
    if (!audioCtx || !document.getElementById("beepOn").checked) return;
    const now = performance.now();
    if (now - lastBeepAt < (near ? 250 : 400)) return;
    lastBeepAt = now;
    try {
      const o = audioCtx.createOscillator(), g = audioCtx.createGain();
      o.frequency.value = 300 + Math.min(1, v / full) * 900 + (near ? 300 : 0);
      g.gain.value = 0.05; o.connect(g); g.connect(audioCtx.destination);
      o.start(); o.stop(audioCtx.currentTime + (near ? 0.18 : 0.08));
    } catch (e) { /* 音が出せなくても続行 */ }
  }
  async function pollFocus() {
    try {
      const res = await fetch("/api/focus.json");
      renderFocus(await res.json());
    } catch (e) { /* 通信エラー時は前回表示のまま */ }
    setTimeout(pollFocus, focusMode ? 300 : 1000);
  }
  document.getElementById("focusModeBtn").addEventListener("click", () => {
    focusMode = !focusMode;
    const btn = document.getElementById("focusModeBtn");
    btn.textContent = focusMode ? "ピント合わせモード終了" : "ピント合わせモード開始";
    btn.classList.toggle("focus-on", focusMode);
    document.getElementById("focusPanel").style.display = focusMode ? "block" : "none";
    if (focusMode && !audioCtx) {
      try { audioCtx = new (window.AudioContext || window.webkitAudioContext)(); } catch (e) { audioCtx = null; }
    }
  });
  document.getElementById("peakResetBtn").addEventListener("click", async () => {
    try { await fetch("/api/focus", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({action: "reset_peak"})}); } catch (e) {}
  });
  document.getElementById("diagBtn").addEventListener("click", async () => {
    const out = document.getElementById("diagOut");
    out.textContent = "診断中...";
    try {
      const res = await fetch("/api/diagnose", {method: "POST"});
      const d = await res.json();
      const parts = [];
      if (d.resolution) parts.push("解像度 " + d.resolution.width + "x" + d.resolution.height);
      if (d.focus) parts.push("ピント " + d.focus.grade_label + (d.focus.score != null ? "(スコア " + d.focus.score + ")" : ""));
      if (d.brightness) parts.push("明るさ平均 " + d.brightness.mean + " / コントラスト " + d.brightness.contrast);
      (d.warnings || []).forEach((w) => parts.push("警告: " + w));
      if (d.advice) parts.push(d.advice);
      out.textContent = parts.join(" | ") || "診断できませんでした";
    } catch (e) { out.textContent = "診断の通信に失敗しました"; }
  });
  async function postAutotune(action) {
    const msg = document.getElementById("autotuneMsg");
    try {
      const res = await fetch("/api/autotune", {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify({action: action})});
      const d = await res.json();
      if (!res.ok) { msg.textContent = d.error || ("失敗(HTTP " + res.status + ")"); return; }
      renderAutotune(d.autotune);
      fastPollAutotune();
    } catch (e) { msg.textContent = "通信に失敗しました"; }
  }
  document.getElementById("autotuneBtn").addEventListener("click", () => {
    if (confirm("画質の自動調整を始めます(1分前後、この間は読み取りの確定が止まります)。カメラに数字の画面が写っていますか?")) postAutotune("start");
  });
  document.getElementById("autotuneRevertBtn").addEventListener("click", () => {
    if (confirm("自動調整の前の設定へ戻します。よろしいですか?")) postAutotune("revert");
  });
  document.getElementById("autotuneCancelBtn").addEventListener("click", () => postAutotune("cancel"));
  function fastPollAutotune() { if (!atFastPoll) { atFastPoll = setTimeout(() => { atFastPoll = null; pollCamera(); }, 700); } }
  function renderAutotune(at) {
    if (!at) return;
    const busy = at.state === "requested" || at.state === "running" || at.state === "reverting";
    document.getElementById("autotuneBtn").disabled = busy || at.available === false;
    document.getElementById("autotuneRevertBtn").disabled = busy || !at.saved;
    document.getElementById("autotuneCancelBtn").style.display = busy ? "inline-block" : "none";
    const pg = document.getElementById("autotuneProgress");
    pg.style.display = busy ? "block" : "none";
    const p = at.progress || {};
    document.getElementById("autotuneBar").style.width = (p.total ? Math.min(100, p.done / p.total * 100) : 5) + "%";
    let m = at.message || "";
    if (busy && p.label) m += " [" + p.label + "]";
    document.getElementById("autotuneMsg").textContent = m || (at.saved ? "調整済み(" + (at.saved.tuned_at || "") + ")" : "未調整");
    const lines = [];
    const r = at.result;
    if (!busy && r && r.changes) {
      Object.values(r.changes).forEach((c) => lines.push(c.label + ": " + c.before + " → " + c.after));
      Object.entries(r.skipped || {}).forEach(([k, why]) => lines.push(k + ": スキップ(" + why + ")"));
      if (r.baseline && r.final) lines.push("評価値: " + r.baseline.value + " → " + r.final.value);
    }
    document.getElementById("autotuneDetail").textContent = lines.join(" / ");
    if (busy) fastPollAutotune();
  }
  pollFocus();
  fetchSettings();
  async function fetchReadings() {
    try {
      const res = await fetch("/api/readings.json");
      const data = await res.json();
      const rows = Object.entries(data.readings || {}).map(([slot, r]) => {
        const sentCls = r.sent ? "sent-yes" : "sent-no";
        const sentText = r.sent ? "送信済み" : (r.menu_name ? "未送信" : "送信先未対応");
        const estimatedBadge = r.estimated ? '<span class="estimated-badge">推定</span>' : "";
        return `<tr>
          <td>${slot}</td>
          <td>${r.menu_name ? escapeHtml(r.menu_name) : "-"}</td>
          <td>${r.raw_value ?? "-"}</td>
          <td>${r.confirmed_value ?? "-"}${estimatedBadge}</td>
          <td class="${sentCls}">${sentText}</td>
        </tr>`;
      });
      document.getElementById("rows").innerHTML = rows.join("") || '<tr><td colspan="5">データがありません</td></tr>';
      const updated = data.updated_at ? new Date(data.updated_at * 1000).toLocaleTimeString("ja-JP") : "-";
      document.getElementById("updatedAt").textContent = "最終更新: " + updated;
      const lk = data.roi_lock || {};
      document.getElementById("lockRow").style.display = lk.mode === "auto_lock" ? "flex" : "none";
      document.getElementById("lockStatus").textContent = lk.locked
        ? "ROI: ロック済み(固定位置で読み取り中)"
        : "ROI: 位置を検出中 (" + (lk.progress || 0) + "/" + (lk.required || 0) + ")";
    } catch (e) { /* 通信エラー時は前回表示のまま */ }
  }
  refreshImageLoop("processed", "/api/processed.jpg", 2500);
  refreshImageLoop("preview", "/api/preview.jpg", 110);
  setInterval(fetchReadings, 1000);
  fetchReadings();
</script>
</body>
</html>
"""


def make_preview_handler(state: PreviewState, config_path: Optional[str], dashboard_url: str):
    # dashboard_urlが空なら、閲覧中のホスト名+ポート8090をブラウザ側(JS)で組み立てる。
    page = _PREVIEW_HTML.replace(
        "__DASHBOARD_URL__", dashboard_url or "http://__AUTOHOST__:8090/"
    ).encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # noqa: A003 - アクセスログを静かにする
            pass

        def _send(self, status: int, body: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802
            if self.path in ("/", "/index.html"):
                self._send(200, page, "text/html; charset=utf-8")
                return
            if self.path.startswith("/api/preview.jpg"):
                state.mark_viewed()
                frame = state.snapshot_frame()
                if frame is None:
                    self._send(404, b"{}", "application/json")
                    return
                self._send(200, frame, "image/jpeg")
                return
            if self.path.startswith("/api/processed.jpg"):
                # 見ていない間に放置された古い画像は、閲覧開始の前に破棄する
                # (破棄→404→run_loopが次の周期で作り直す→UI側が再取得)。
                if state.seconds_since_viewed() > _PREVIEW_IDLE_TIMEOUT_SEC:
                    state.discard_processed_if_stale(8.0)
                state.mark_viewed()
                jpeg = state.snapshot_processed()
                if jpeg is None:
                    self._send(404, b"{}", "application/json")
                    return
                self._send(200, jpeg, "image/jpeg")
                return
            if self.path == "/api/readings.json":
                body = json.dumps(state.snapshot_readings(), ensure_ascii=False).encode("utf-8")
                self._send(200, body, "application/json; charset=utf-8")
                return
            if self.path == "/api/settings.json":
                body = json.dumps(state.snapshot_settings(), ensure_ascii=False).encode("utf-8")
                self._send(200, body, "application/json; charset=utf-8")
                return
            if self.path == "/api/focus.json":
                # 【2026-10-10追加】ピント合わせモード用(軽量。ブラウザが0.3秒ごとに読む)
                state.mark_viewed()
                body = json.dumps(state.snapshot_focus(), ensure_ascii=False).encode("utf-8")
                self._send(200, body, "application/json; charset=utf-8")
                return
            self._send(404, b"{}", "application/json")

        def _read_json_body(self):
            """POSTボディをJSONオブジェクトとして読む(空は{})。不正ならNone。"""
            try:
                length = int(self.headers.get("Content-Length", "0") or "0")
                data = json.loads(self.rfile.read(min(length, 65536)) or b"{}")
            except (ValueError, json.JSONDecodeError):
                return None
            return data if isinstance(data, dict) else None

        def do_POST(self) -> None:  # noqa: N802
            if self.path == "/api/settings":
                # 【2026-09-25追加】ユーザー要望「拡大する倍率は実際に固定しないと
                # 分からないからhtmlの管理画面から変更できるようにしておいて」対応。
                length = int(self.headers.get("Content-Length", "0") or "0")
                try:
                    data = json.loads(self.rfile.read(length) or b"{}")
                    if not isinstance(data, dict):
                        raise TypeError("body must be an object")
                    if not any(k in data for k in ("ocr_scale_factor", "frame_rotate_deg", "camera_profile")):
                        raise KeyError("ocr_scale_factor")
                    scale_factor = float(data["ocr_scale_factor"]) if "ocr_scale_factor" in data else None
                except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                    self._send(400, b'{"error": "ocr_scale_factor must be a number"}', "application/json")
                    return
                if scale_factor is not None and not (1.0 <= scale_factor <= 4.0):
                    self._send(
                        400, b'{"error": "ocr_scale_factor must be between 1.0 and 4.0"}', "application/json"
                    )
                    return
                # 【2026-10-09追加】frame_rotate_deg: 許可リスト(0/90/180/270)の整数だけ受理。
                rotate_deg = None
                if "frame_rotate_deg" in data:
                    raw_deg = data["frame_rotate_deg"]
                    if (
                        isinstance(raw_deg, bool) or not isinstance(raw_deg, int)
                        or raw_deg not in VALID_ROTATE_DEGS
                    ):
                        self._send(
                            400, b'{"error": "frame_rotate_deg must be one of 0, 90, 180, 270"}',
                            "application/json",
                        )
                        return
                    rotate_deg = raw_deg
                # 【2026-10-09追加】camera_profile: "demo"|"production"のみ(許可リスト)。
                camera_profile = None
                if "camera_profile" in data:
                    raw_prof = data["camera_profile"]
                    if not isinstance(raw_prof, str) or raw_prof not in CAMERA_PROFILE_NAMES:
                        self._send(
                            400, b'{"error": "camera_profile must be one of demo, production"}',
                            "application/json",
                        )
                        return
                    camera_profile = raw_prof
                updates: Dict = {}
                if camera_profile is not None:
                    # 切替は撮影スレッドが行い、成功後に設定ファイルへ書き戻す(apply_camera_switch)
                    if state.request_camera_profile(camera_profile):
                        logger.info("カメラのプロファイルを%sへ切り替える要求を受けました", camera_profile)
                if scale_factor is not None:
                    state.set_scale_factor(scale_factor)
                    updates["ocr_scale_factor"] = scale_factor
                if rotate_deg is not None:
                    # 変わった場合は古い角度のROIを捨てて再検出に戻る(/api/relockと同じ扱い)
                    if state.set_rotate_deg(rotate_deg):
                        logger.info("映像の回転角を%d度に切り替えました。ROIを再検出します", rotate_deg)
                    updates["frame_rotate_deg"] = rotate_deg
                if config_path:
                    try:
                        save_config_updates(config_path, updates)
                    except OSError as exc:
                        logger.warning(f"設定ファイルへの書き戻しに失敗しました(次回起動時は反映されません): {exc}")
                payload = dict(state.snapshot_settings(), ok=True)
                self._send(200, json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json")
                return
            if self.path == "/api/autotune":
                # 【2026-10-10追加】画質の自動調整・元に戻す・中断。実行は撮影スレッド。許可リスト方式。
                data = self._read_json_body()
                action = "start" if data is not None and "action" not in data else (data or {}).get("action")
                if data is None or not isinstance(action, str) or action not in PreviewState.AUTOTUNE_ACTIONS:
                    self._send(
                        400, b'{"error": "action must be one of start, revert, cancel"}', "application/json"
                    )
                    return
                if cq is None:
                    self._send(503, b'{"error": "ocr_camera_quality.py is not installed"}', "application/json")
                    return
                ok, reason = state.request_autotune(action)
                if not ok:
                    self._send(
                        409, json.dumps({"error": reason}, ensure_ascii=False).encode("utf-8"),
                        "application/json; charset=utf-8",
                    )
                    return
                logger.info("画質の自動調整の要求を受けました: %s", action)
                payload = {"ok": True, "autotune": state.snapshot_autotune()}
                self._send(200, json.dumps(payload, ensure_ascii=False).encode("utf-8"), "application/json")
                return
            if self.path == "/api/focus":
                data = self._read_json_body()
                if data is None or data.get("action") != "reset_peak":
                    self._send(400, b'{"error": "action must be reset_peak"}', "application/json")
                    return
                if state.focus is not None:
                    state.focus.reset_peak()
                self._send(200, b'{"ok": true}', "application/json")
                return
            if self.path == "/api/diagnose":
                body = json.dumps(state.diagnose(), ensure_ascii=False).encode("utf-8")
                self._send(200, body, "application/json; charset=utf-8")
                return
            if self.path == "/api/relock":
                # 【2026-10-02追加】roi_mode="auto_lock"のROIロックを解除して再検出させる。
                state.request_relock()
                self._send(200, b'{"ok": true}', "application/json")
                return
            self._send(404, b"{}", "application/json")

    return Handler


def start_preview_server(
    state: PreviewState, host: str, port: int, config_path: Optional[str] = None,
    dashboard_url: str = "",
) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), make_preview_handler(state, config_path, dashboard_url))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    # 【2026-10-02追加】この確認画面は単体動作を前提に認証を付けていない。POST /api/settings
    # (OCR倍率の変更)と POST /api/relock(ROIの再検出)も誰でも実行できるため、
    # 信頼できるLAN内だけで使い、インターネットや来場者用Wi-Fiへ公開しないこと。
    logger.warning(
        "確認用ダッシュボード(ポート%s)は認証なしです。LAN内専用で、外部ネットワークへ公開しないでください"
        "(POST /api/settings・/api/relock・/api/autotune が誰でも実行できます)。", port,
    )
    logger.info(f"確認用ダッシュボードを起動しました: http://{host}:{port}")
    return server


def _draw_preview_overlay(
    cv2, frame, roi: Roi, slot: str, value: Optional[int], confirmed: Optional[int],
    locked: bool = False,
) -> None:
    """確認用ダッシュボードのプレビュー画像に、ROI枠と現在の読み取り値を
    重ねて描く(--calibrate用のグリッド描画と同じ考え方)。confirmed(確定済み)
    ならROI枠を緑、まだ確定していない/読み取れていないなら黄色にする。
    """
    h, w = frame.shape[0], frame.shape[1]
    x1, y1 = int(roi.x1 * w), int(roi.y1 * h)
    x2, y2 = int(roi.x2 * w), int(roi.y2 * h)
    color = (0, 200, 0) if confirmed is not None else (0, 200, 220)
    # ロック済み(roi_mode="auto_lock")の枠は太く描いて、検出中と見分けられるようにする。
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 4 if locked else 2)
    label = f"{slot}: {confirmed if confirmed is not None else (value if value is not None else '?')}"
    cv2.putText(frame, label, (x1, max(0, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def _build_processed_montage(cv2, tiles, scale_factor: float, tile_width: int = 360) -> Optional[bytes]:
    """各スロットの前処理後(拡大→二値化)画像を、見出し付きで縦に並べた1枚のJPEGにする。

    表示幅は揃えるが、補間はINTER_NEARESTにして、拡大倍率によるピクセルの粗さ・
    画像サイズの違いが目で見えるようにする(見出しに実際の画像サイズと倍率も出す)。
    """
    rows = []
    for slot, img in tiles:
        h, w = img.shape[:2]
        disp = cv2.resize(
            img, (tile_width, max(1, int(h * tile_width / max(1, w)))), interpolation=cv2.INTER_NEAREST
        )
        disp = cv2.cvtColor(disp, cv2.COLOR_GRAY2BGR)
        bar = np.full((22, tile_width, 3), 40, dtype=np.uint8)
        cv2.putText(
            bar, f"{slot}  {w}x{h}px  x{scale_factor:g}", (6, 16),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
        )
        rows.append(np.vstack([bar, disp]))
    if not rows:
        return None
    ok, jpeg = cv2.imencode(".jpg", np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 70])
    return jpeg.tobytes() if ok else None


# 確認用ダッシュボードへ何秒アクセスが無かったら「見ている人がいない」とみなすか、
# その間の低頻度撮影周期(秒)。2026-09-28追加。
_PREVIEW_IDLE_TIMEOUT_SEC = 8.0
_PREVIEW_IDLE_PERIOD_SEC = 2.0


def apply_camera_switch(preview_state: PreviewState, ctl: CameraController, persist=None) -> bool:
    """確認画面から要求されたカメラのプロファイル切替があれば実行する(撮影スレッドから毎周期呼ぶ)。
    例外でループを止めない。失敗したら元のプロファイルのまま(ctl.switchが元の設定を開き直す)。
    成功したらpersist(name)で設定ファイルへ書き戻す。切替があればTrueを返す。"""
    req = preview_state.take_camera_request()
    if req is None:
        return False
    name, params = req
    try:
        ok = ctl.switch(params, tuning_profile=name)
    except Exception as exc:  # noqa: BLE001
        ctl.last_error = str(exc)
        ok = False
    if ok:
        preview_state.finish_camera_switch(name, ctl.info)
        logger.info(
            "カメラのプロファイルを%sへ切り替えました(実際=%sx%s fps=%s)",
            name, ctl.info.get("width"), ctl.info.get("height"), ctl.info.get("fps"),
        )
        if persist is not None:
            try:
                persist(name)
            except Exception as exc:  # noqa: BLE001
                logger.warning(f"カメラのプロファイルの設定ファイルへの書き戻しに失敗しました: {exc}")
    else:
        preview_state.fail_camera_switch(ctl.last_error or "unknown", ctl.info if ctl.cap is not None else {})
    return True


def run_autotune_request(preview_state: PreviewState, ctl: CameraController, cv2, persist_tuning=None) -> bool:
    """確認画面から要求された「画質の自動調整」「元に戻す」があれば実行する(撮影スレッドから毎周期呼ぶ)。
    実行中は読み取りの確定を止める(run_loopがpreview_state.autotune_busy()を見る)。
    どこで失敗しても例外を外へ出さない(撮影ループを止めない)。実行したらTrueを返す。
    結果はctl.tuning[プロファイル名]へ入れ、persist_tuning(dict)で設定ファイル(camera_tuning)へ保存する。"""
    action = preview_state.take_autotune_request()
    if action is None:
        return False
    key = cq.tuning_profile_key(preview_state.snapshot_settings().get("camera_profile")) if cq else "default"
    if preview_state.focus is not None:
        preview_state.focus.set_suspended(True)
    try:
        cap = ctl.cap
        if cq is None or cap is None:
            preview_state.finish_autotune(action, None, "failed", "カメラが使えないため実行できません")
            return True
        rotate_deg = preview_state.get_rotate_deg()
        last_pub = [0.0]

        def read_frame():
            ok, fr = cap.read()
            if not ok or fr is None or fr.size == 0:
                return None
            fr = rotate_frame(cv2, fr, rotate_deg)
            preview_state.update_raw_frame(fr, rotate_deg)
            now = time.monotonic()
            if now - last_pub[0] >= 0.5:  # 調整中も映像が止まって見えないよう、簡易プレビューを更新
                last_pub[0] = now
                small = cv2.resize(fr, (480, max(1, int(fr.shape[0] * 480 / fr.shape[1]))))
                cv2.putText(small, "AUTOTUNE...", (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 220), 2)
                ok_enc, jpeg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 60])
                if ok_enc:
                    preview_state.update_frame(jpeg.tobytes())
            return fr

        if action == "revert":
            rec = ctl.tuning.get(key)
            if not rec:
                preview_state.finish_autotune(action, None, "failed", "元に戻す調整がありません(このプロファイルは未調整です)")
                return True
            res = cq.apply_values(cv2, cap, rec.get("original", {}))
            bad = [k for k, ok in res.items() if not ok]
            del ctl.tuning[key]
            if persist_tuning is not None:
                persist_tuning(dict(ctl.tuning))
            preview_state.set_tuning_saved(ctl.tuning)
            for _ in range(5):  # 設定が映像に反映されるまで数フレーム読んでrawを更新
                read_frame()
            msg = "調整前の設定へ戻しました" + (f"(反映されなかった項目: {bad})" if bad else "")
            logger.info("画質の自動調整を元に戻しました(%s): %s", key, rec.get("original"))
            preview_state.finish_autotune(action, {"reverted": rec.get("original", {}), "failed": bad}, "reverted", msg)
            return True

        tuner = cq.CameraTuner(
            cv2, np, cap, read_frame, box_provider=preview_state.get_focus_box,
            progress=preview_state.set_autotune_progress, cancelled=preview_state.autotune_cancelled,
        )
        result = tuner.run()
        status = result.get("status", "failed")
        if status == "improved" and result.get("applied"):
            prior = ctl.tuning.get(key) or {}
            ctl.tuning[key] = {
                "applied": dict(prior.get("applied", {}), **result["applied"]),
                # 元の値は「一番最初の値」を守る(2回目以降の調整でも、元に戻すと調整前へ戻る)
                "original": dict(result["original"], **prior.get("original", {})),
                "tuned_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            }
            if persist_tuning is not None:
                persist_tuning(dict(ctl.tuning))
            preview_state.set_tuning_saved(ctl.tuning)
        for ch_key, ch in result.get("changes", {}).items():
            logger.info("画質の自動調整: %s %s -> %s", ch.get("label", ch_key), ch.get("before"), ch.get("after"))
        for sk_key, why in result.get("skipped", {}).items():
            logger.info("画質の自動調整: %s はスキップ(%s)", sk_key, why)
        logger.info(
            "画質の自動調整が終わりました: %s %s 評価値 %s -> %s (%s回評価、%s秒)", status, result.get("message"),
            (result.get("baseline") or {}).get("value"), (result.get("final") or {}).get("value"),
            result.get("evals"), result.get("elapsed_sec"),
        )
        for _ in range(3):
            read_frame()
        state = {"improved": "done", "unchanged": "done", "cancelled": "cancelled"}.get(status, "failed")
        preview_state.finish_autotune(action, result, state, result.get("message", ""))
    except Exception as exc:  # noqa: BLE001
        logger.exception("画質の自動調整の実行中に想定外のエラー(継続します)")
        preview_state.finish_autotune(action, None, "failed", f"自動調整に失敗しました: {exc}")
    finally:
        preview_state.reset_quality_history()
        if preview_state.focus is not None:
            preview_state.focus.set_suspended(False)
    return True


def _camera_capture_loop(
    ctl: CameraController, cv2, preview_state: PreviewState, rois: Dict[str, Roi], preview_fps: float,
    preview_max_width: int, preview_jpeg_quality: int, roi_mode: str = "fixed", persist=None,
    persist_tuning=None,
) -> None:
    """プレビュー用の撮影を専用スレッドで行う。

    【背景】以前はOCRのポーリング周期(既定5秒)に1回しか撮影しておらず、
    確認用ダッシュボードの映像も実質0.2fps程度しか更新されなかった
    (ユーザー要望「プレビューのfpsをもう少し上げたい」への対応)。カメラへの
    アクセスはこのスレッドだけが行い(cv2.VideoCaptureは複数スレッドからの
    同時読み出しを想定していない)、run_loop()側は撮影済みの最新フレームを
    `preview_state.get_raw_frame()`経由で受け取るだけにする。
    重ねて描く読み取り値は、直近にrun_loop側が確定させた値
    (`preview_state.snapshot_readings()`)をそのまま使う(この時点では
    このスレッド自身は認識処理をしない、映像を速く見せることだけが目的)。

    【2026-09-25追記】さらにfpsを上げるため、person_target_detector.py
    (Jetson側)と同じ考え方で、プレビュー配信用の画像だけ幅を縮小してから
    エンコードする(認識処理自体は縮小前のフル解像度フレームを使うので、
    OCR精度には影響しない)。ラズパイCPUでの1〜720p画像の重ねて描く処理+
    JPEGエンコードが実質的なfpsの律速要因だったため、画素数を減らすことで
    速くする。

    【2026-09-25追記】roi_mode="auto_color"のときは、固定rois枠の代わりに
    run_loop側がdetect_digit_rows()で見つけた直近の枠
    (`preview_state.snapshot_dynamic_rois()`)を描く。detect_digit_rows()
    自体はOCR判定周期(既定5秒に1回)でしか呼ばないので、このスレッド自身が
    毎フレーム動的検出をやり直すことはしない(fpsを落とさないため)。

    【2026-09-28追記】ダッシュボードを誰も見ていない間は、撮影周期を
    大きく落とす(重ねて描く処理+JPEGエンコードが主なCPU消費源のため)。
    OCR本体(run_loop側)はpreview_state.get_raw_frame()経由でこのスレッドの
    最新フレームを使うが、判定周期(既定5秒に1回)に対して間引き後の周期
    (既定2秒に1回)でも十分新しいフレームを渡せるため、精度への影響は無い
    (ユーザー要望「軽くできないか検討して」対応)。
    """
    active_period = 1.0 / max(0.5, preview_fps)
    last_decode_at = 0.0
    fps_count, fps_since = 0, time.monotonic()
    while True:
        start = time.monotonic()
        # 【2026-10-09追加】カメラのプロファイル切替(要求があれば開き直す)と、適用中の
        # プロファイルのプレビュー設定の反映。
        try:
            if apply_camera_switch(preview_state, ctl, persist):
                fps_count, fps_since = 0, time.monotonic()
        except Exception as exc:  # noqa: BLE001
            logger.error(f"カメラ切替処理で想定外のエラー(継続します): {exc}")
        # 【2026-10-10追加】画質の自動調整/元に戻す(要求があれば実行。この間は読み取りの確定が止まる)
        try:
            if run_autotune_request(preview_state, ctl, cv2, persist_tuning):
                fps_count, fps_since = 0, time.monotonic()
        except Exception as exc:  # noqa: BLE001
            logger.error(f"画質の自動調整処理で想定外のエラー(継続します): {exc}")
        cam_params = preview_state.get_camera_params()
        if cam_params is not None:
            preview_fps = float(cam_params["preview_fps"])
            preview_max_width = int(cam_params["preview_max_width"])
            preview_jpeg_quality = int(cam_params["preview_jpeg_quality"])
            active_period = 1.0 / max(0.5, preview_fps)
        cap = ctl.cap
        if cap is None:
            # 開き直しに失敗した状態。2秒ごとに直近の設定で再オープンを試みる。
            time.sleep(2.0)
            ctl.recover()
            continue
        idle = preview_state.seconds_since_viewed() > _PREVIEW_IDLE_TIMEOUT_SEC
        # 間引き中はデコード/描画/エンコードの周期だけを落とし、ループ自体
        # (grab)は通常周期で回す。
        period = _PREVIEW_IDLE_PERIOD_SEC if idle else active_period
        if idle:
            # 間引き中もバッファは通常周期でgrab()して捨て(デコード無しで軽い)、
            # 間引き周期でretrieve()する画像が古いフレームにならないようにする。
            grabbed = cap.grab()
            if grabbed and start - last_decode_at >= period:
                last_decode_at = start
                ok, frame = cap.retrieve()
            else:
                ok, frame = False, None
        else:
            ok, frame = cap.read()
            last_decode_at = start
        if not idle and ok:
            # 実測fps(確認画面の表示用、3秒ごとに更新)
            fps_count += 1
            if start - fps_since >= 3.0:
                preview_state.set_measured_fps(fps_count / (start - fps_since))
                fps_count, fps_since = 0, start
        if ok and frame is not None and frame.size > 0:
            # 【2026-10-09追加】最初の段階で回転して正立に戻す。以降(ROI・プレビュー縮小・
            # OCR側へ渡すフレーム)は全て回転後のshape(90/270では幅と高さが入れ替わる)。
            rotate_deg = preview_state.get_rotate_deg()
            frame = rotate_frame(cv2, frame, rotate_deg)
            preview_state.update_raw_frame(frame, rotate_deg)
            if preview_state.history is not None and preview_state.get_rotate_deg() == rotate_deg:
                preview_state.history.push(frame)  # ベストフレーム選択/平均用の直近フレーム
            preview_frame = frame
            h, w = preview_frame.shape[:2]
            if w > preview_max_width:
                # 【2026-09-30変更】1280→480を直接INTER_AREAで縮小すると、Jetsonでの
                # py-spy実測でプレビュー表示中のCPUの約11%を占めていた。先に整数倍で
                # 間引いて(ビュー、コピー無し)から縮小することで軽くする(プレビュー
                # 専用なので多少の画質差は許容。認識側は間引き前のフレームを使う)。
                step = max(1, w // preview_max_width)
                small = preview_frame[::step, ::step] if step > 1 else preview_frame
                sh, sw = small.shape[:2]
                if sw > preview_max_width:
                    small = cv2.resize(
                        small, (preview_max_width, int(sh * preview_max_width / sw)),
                        interpolation=cv2.INTER_LINEAR,
                    )
                preview_frame = small.copy()
            else:
                preview_frame = preview_frame.copy()
            readings = preview_state.snapshot_readings().get("readings", {})
            active_rois = (
                preview_state.snapshot_dynamic_rois() if roi_mode in ("auto_color", "auto_lock") else rois
            )
            lock_info = preview_state.snapshot_roi_lock() if roi_mode == "auto_lock" else {}
            is_locked = bool(lock_info.get("locked"))
            # 【2026-10-10追加】ピント(鮮鋭度)の測定(ROIの外接枠、無ければ画面中央)。確認画面を見ている間は
            # 0.3秒ごと、見ていない間は2秒ごと(ガイダンス用)。測定の失敗でループを止めない。
            focus = preview_state.focus
            if focus is not None:
                focus_box = cq.rois_bounding_box(active_rois)
                preview_state.set_focus_box(focus_box)
                if focus.due(2.0 if idle else 0.3):
                    try:
                        focus.update(cv2, np, frame, focus_box)
                        if focus.startup_report_due():
                            snap = focus.snapshot()
                            bri = snap.get("brightness") or {}
                            logger.info(
                                "カメラ診断(起動時): 解像度=%s ピント=%s(スコア%s) 明るさ平均=%s コントラスト=%s %s %s",
                                snap.get("resolution"), snap.get("grade_label"), snap.get("avg"),
                                bri.get("mean"), bri.get("contrast"), " / ".join(snap.get("warnings") or []),
                                snap.get("advice") or "",
                            )
                    except Exception as exc:  # noqa: BLE001
                        logger.warning(f"鮮鋭度の測定に失敗(継続します): {exc}")
            for slot, roi in active_rois.items():
                r = readings.get(slot, {})
                _draw_preview_overlay(
                    cv2, preview_frame, roi, slot, r.get("raw_value"), r.get("confirmed_value"),
                    locked=is_locked,
                )
            if lock_info:
                status_text = (
                    "ROI LOCKED" if is_locked
                    else f"ROI DETECTING {lock_info.get('progress', 0)}/{lock_info.get('required', 0)}"
                )
                cv2.putText(
                    preview_frame, status_text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 200, 0) if is_locked else (0, 200, 220), 2,
                )
            ok_enc, jpeg = cv2.imencode(
                ".jpg", preview_frame, [cv2.IMWRITE_JPEG_QUALITY, preview_jpeg_quality]
            )
            if ok_enc:
                preview_state.update_frame(jpeg.tobytes())
        elapsed = time.monotonic() - start
        time.sleep(max(0.0, active_period - elapsed))


def run_loop(config: Dict) -> None:
    import cv2  # type: ignore

    base_url = str(config["server_base_url"]).rstrip("/")
    auth_password = str(config["auth_password"])
    if not auth_password or auth_password.startswith("CHANGE_ME"):
        # 実値はgit管理外の設定ファイル(~/.nokosahen_tablet_ocr_config.json)か環境変数で与える
        auth_password = os.environ.get("NOKOSAHEN_TABLET_OCR_PASSWORD", "")
    if not auth_password:
        print(
            "[警告] auth_password が未設定です(設定ファイルか環境変数 "
            "NOKOSAHEN_TABLET_OCR_PASSWORD で指定してください)。残数入力画面への送信は認証エラーになります。",
            flush=True,
        )
    auth = (str(config["auth_username"]), auth_password)
    poll_interval = max(1.0, float(config["poll_interval_sec"]))
    camera_profile = select_camera_profile(config)
    camera_profiles = resolve_camera_profiles(config)
    if camera_profile:
        poll_interval = max(1.0, float(camera_profiles[camera_profile]["poll_interval_sec"]))
        logger.info("カメラのプロファイル: %s", camera_profile)
    rois = {slot: Roi(**roi_dict) for slot, roi_dict in config["rois"].items()}
    roi_mode = str(config.get("roi_mode", "fixed"))
    sanity_max_ratio = float(config.get("ocr_sanity_max_ratio", 3.0))
    stabilizers = {slot: SlotStabilizer(int(config["stable_reads_required"])) for slot in rois}
    last_sent: Dict[str, int] = {}
    predict_after_fails = int(config.get("predict_after_consecutive_fails", 6))
    predict_history_size = int(config.get("predict_history_size", 6))
    predictors = {slot: TrendPredictor(predict_history_size) for slot in rois}
    fail_streak: Dict[str, int] = {slot: 0 for slot in rois}
    last_confirmed_value: Dict[str, int] = {}
    recognizer = build_recognizer(config)
    initial_scale_factor = float(config.get("ocr_scale_factor", 2.0))
    initial_rotate_deg = normalize_rotate_deg(config.get("frame_rotate_deg", 0))
    if initial_rotate_deg:
        logger.info("撮影直後にフレームを時計回りに%d度回転します(frame_rotate_deg)", initial_rotate_deg)

    # OCR値の急変フィルタ(JumpGuard参照)。0以下で無効。
    max_jump = int(config.get("ocr_max_jump", 50))
    jump_override_reads = max(1, int(config["stable_reads_required"])) * 2
    jump_guards = {slot: JumpGuard(max_jump, jump_override_reads) for slot in rois}
    row_merge_kernel_w = int(config.get("roi_row_merge_kernel_w", 0) or 0)
    enhance_cache: Dict[Optional[str], object] = {}  # プロファイル別の画質改善設定(検証警告を毎周期出さない)
    numbers_writer = NumbersTxtWriter(
        str(config.get("numbers_txt_path", "") or ""),
        on_change_only=bool(config.get("numbers_txt_on_change_only", True)),
        with_timestamp=bool(config.get("numbers_txt_with_timestamp", False)),
        include_estimated=bool(config.get("numbers_txt_include_estimated", False)),
    )

    # roi_mode="auto_lock"のROIロック(RoiLocker参照)。
    locker: Optional[RoiLocker] = None
    auto_lock_save = bool(config.get("auto_lock_save", False))
    config_path_for_lock = config.get("_config_path")
    if roi_mode == "auto_lock":
        locker = RoiLocker(
            expected=len(FULL_ROW_ORDER),
            required=int(config.get("auto_lock_required_cycles", 3)),
            tolerance=float(config.get("auto_lock_tolerance", 0.04)),
            margin_ratio=float(config.get("auto_lock_margin_ratio", 0.05)),
            reset_after_fail_cycles=int(config.get("auto_lock_reset_after_fail_cycles", 12)),
        )
        saved_boxes = parse_locked_rois(config.get("locked_rois"))
        if saved_boxes is not None:
            locker.load(saved_boxes)
            logger.info("設定のlocked_roisからROIをロック済みで開始します")

    def _persist_locked_rois(boxes: Optional[List[RatioBox]]) -> None:
        # auto_lock_save=trueの時だけ設定ファイルへ書く(既定では何も書かない)。
        if not auto_lock_save or not config_path_for_lock:
            return
        try:
            save_config_updates(
                config_path_for_lock,
                {"locked_rois": rois_to_config_dict(boxes) if boxes is not None else None},
            )
        except OSError as exc:
            logger.warning(f"ロックしたROIの設定ファイルへの保存に失敗しました: {exc}")

    preview_state: Optional[PreviewState] = None
    if bool(config.get("preview_enabled", True)):
        preview_state = PreviewState(
            initial_scale_factor, initial_rotate_deg,
            cq.resolve_focus_thresholds(config.get("focus_thresholds")) if cq else None,
            float(config.get("focus_warn_after_sec", 20.0) or 20.0),
        )
        start_preview_server(
            preview_state,
            str(config.get("preview_host", "127.0.0.1")),
            int(config.get("preview_port", 8091)),
            config.get("_config_path"),
            str(config.get("dashboard_url", "")),
        )

    ctl = CameraController(cv2, config["camera_index"])
    # 【2026-10-10追加】保存済みの「画質の自動調整」(camera_tuning)を、カメラを開いた直後に再適用する。
    if cq is not None:
        ctl.tuning = cq.resolve_camera_tuning(config.get("camera_tuning"))
        ctl.tuning_profile = cq.tuning_profile_key(camera_profile)
    ctl.open(
        dict(camera_profiles[camera_profile], size_fallback=True) if camera_profile
        else legacy_open_params(config)
    )
    cap = ctl.cap
    if preview_state is not None:
        preview_state.init_camera(camera_profile, camera_profiles, ctl.info)

    def _persist_camera_profile(name: str) -> None:
        if config_path_for_lock:
            save_config_updates(config_path_for_lock, {"camera_profile": name})

    def _persist_tuning(tuning: Dict[str, Dict]) -> None:
        # 自動調整の結果(camera_tuning)。空になったら(元に戻した)キーをnullへ。
        if config_path_for_lock:
            try:
                save_config_updates(config_path_for_lock, {"camera_tuning": tuning or None})
            except OSError as exc:
                logger.warning(f"画質の自動調整結果の設定ファイルへの保存に失敗しました(次回起動時は再適用されません): {exc}")

    if preview_state is not None:
        preview_state.set_tuning_saved(ctl.tuning)

    if preview_state is not None and locker is not None:
        preview_state.set_roi_lock("auto_lock", locker.is_locked, locker.progress, locker.required)
        if locker.is_locked and locker.locked is not None:
            preview_state.update_dynamic_rois(ratio_boxes_to_rois(locker.locked))

    if preview_state is not None:
        # カメラへのアクセス(cap.read())は、プレビュー用の撮影スレッドに
        # 一本化する(fpsを上げるため。詳細は_camera_capture_loop参照)。
        # 以降run_loop側は自分でcap.read()せず、撮影済みの最新フレームを
        # preview_state経由で受け取るだけにする。
        preview_fps = max(0.5, float(config.get("preview_fps", 10.0)))
        preview_max_width = int(config.get("preview_max_width", 480))
        preview_jpeg_quality = int(config.get("preview_jpeg_quality", 55))
        threading.Thread(
            target=_camera_capture_loop,
            args=(
                ctl, cv2, preview_state, rois, preview_fps, preview_max_width, preview_jpeg_quality,
                roi_mode, _persist_camera_profile, _persist_tuning,
            ),
            daemon=True,
        ).start()

    logger.info("読み取りループを開始します(Ctrl+Cで終了)")
    try:
        while True:
            start = time.monotonic()
            try:
                if preview_state is not None and preview_state.autotune_busy():
                    # 【2026-10-10追加】画質の自動調整/元に戻す中は、カメラ設定を振っていて映像が不安定なので
                    # 読み取り・確定・送信を丸ごと止める(確定値・連続失敗カウントも触らない)。
                    time.sleep(0.5)
                    continue
                if preview_state is not None:
                    frame = preview_state.get_raw_frame()
                    ok = frame is not None
                else:
                    ok, frame = cap.read()
                    if ok and frame is not None and frame.size > 0:
                        frame = rotate_frame(cv2, frame, initial_rotate_deg)
                if not ok or frame is None or frame.size == 0:
                    logger.warning("フレーム取得に失敗しました。次の周期に再試行します")
                    frame = None

                # 【2026-10-10追加】OCR前処理の画質改善(image_enhance)。直近フレームから鮮鋭なものを選び、
                # 静止中は平均してノイズを減らす(以降の検出・切り出し・認識はこのフレームを使う)。
                enhance = None
                if cq is not None and frame is not None:
                    prof_now = preview_state.get_camera_profile() if preview_state is not None else camera_profile
                    if prof_now not in enhance_cache:
                        enhance_cache[prof_now] = cq.resolve_image_enhance(config, prof_now)
                    enhance = enhance_cache[prof_now]
                    if (
                        enhance.any_frame_stage and preview_state is not None and preview_state.history is not None
                    ):
                        built, enh_info = cq.build_enhanced_frame(
                            cv2, np, preview_state.history.snapshot(), preview_state.get_focus_box(), enhance
                        )
                        if built is not None and built.shape == frame.shape:
                            frame = built
                            if enh_info.get("used", 1) > 1 or enh_info.get("considered", 1) > 1:
                                logger.debug("フレーム選択/平均: %s", enh_info)
                    if not enhance.enabled:
                        enhance = None

                try:
                    slot_info = fetch_slot_info(base_url, auth) if frame is not None else {}
                except requests.RequestException as exc:
                    logger.warning(f"のこさへん本体との通信に失敗(続行): {exc}")
                    slot_info = {}

                cycle_rois: Dict[str, Roi] = {}
                if (
                    locker is not None and preview_state is not None
                    and preview_state.consume_relock_request()
                ):
                    # ダッシュボードの「ROIを再検出」ボタン
                    locker.reset()
                    preview_state.update_dynamic_rois({})
                    _persist_locked_rois(None)
                    logger.info("ROIの再検出が要求されました。ロックを解除します")
                if frame is not None:
                    if locker is not None and locker.is_locked and locker.locked is not None:
                        # ロック済み: 検出せず固定位置をそのまま使う(CPU節約)
                        cycle_rois = ratio_boxes_to_rois(locker.locked)
                    elif roi_mode in ("auto_color", "auto_lock"):
                        detected_boxes = detect_digit_rows(cv2, np, frame, row_merge_kernel_w)
                        # 7行より多ければ、メインの縦列に近い7行へ絞り込む
                        # (tab.pyのselect_best_seven_boxesの考え方、2026-10-02)。
                        selected_boxes = select_best_rows(detected_boxes, len(FULL_ROW_ORDER))
                        if selected_boxes is None and enhance is not None:
                            # 検出が期待行数に届かない(ぼけ・低コントラスト等で赤茶色の彩度が落ちた)時は、
                            # コントラストを伸ばす/輪郭を締めた強化版で順に再検出する(成功した版を以降も使う)。
                            for vname, vframe in cq.detection_variants(cv2, np, frame, enhance):
                                vboxes = detect_digit_rows(cv2, np, vframe, row_merge_kernel_w)
                                vsel = select_best_rows(vboxes, len(FULL_ROW_ORDER))
                                if vsel is not None:
                                    logger.info("動的ROI検出: 画質強化(%s)で%d行を検出しました", vname, len(vsel))
                                    detected_boxes, selected_boxes, frame = vboxes, vsel, vframe
                                    break
                        if selected_boxes is None:
                            # 期待した行数(7)に満たない周期は、タブレットの
                            # 一部が隠れている/映っていない等の可能性が高いため、
                            # 誤った並び替えで数字を送信しないようスキップする
                            # (フレーム取得失敗時の扱いと同じ考え方。この周期は
                            # 全スロットがfail_streakにカウントされ、続くと
                            # TrendPredictorによる推測に切り替わる)。
                            logger.warning(
                                "動的ROI検出: 期待した行数(%d)に満たません(検出=%d件)。"
                                "この周期はスキップします",
                                len(FULL_ROW_ORDER), len(detected_boxes),
                            )
                            if locker is not None:
                                locker.observe(None, frame.shape[1], frame.shape[0])
                        else:
                            if len(detected_boxes) > len(FULL_ROW_ORDER):
                                logger.info(
                                    "動的ROI検出: %d件検出したため、縦列に近い%d行へ絞り込みました",
                                    len(detected_boxes), len(FULL_ROW_ORDER),
                                )
                            cycle_rois = {
                                slot: _pixel_box_to_roi(frame, box)
                                for slot, box in zip(FULL_ROW_ORDER, selected_boxes)
                            }
                            if locker is not None and locker.observe(
                                selected_boxes, frame.shape[1], frame.shape[0]
                            ):
                                assert locker.locked is not None
                                cycle_rois = ratio_boxes_to_rois(locker.locked)
                                logger.info(
                                    "ROIをロックしました(連続%d周期でほぼ同じ位置に7行を検出)。"
                                    "以降は再検出せず固定位置で読み取ります", locker.required,
                                )
                                _persist_locked_rois(locker.locked)
                            if preview_state is not None:
                                preview_state.update_dynamic_rois(cycle_rois)
                    else:
                        cycle_rois = rois

                # 【2026-09-25追記】常にrois全スロット分をループする(cycle_rois
                # に無いスロット=このフレームでは撮れなかった/検出できなかった
                # スロットもvalue=Noneとして扱い、fail_streakとTrendPredictorへの
                # 連携を必ず行うため。ユーザー要望「連続で読めない場合は増減比から
                # 推測する」対応)。
                # 確認画面を見ている間だけ、認識に渡す前処理後の画像を集める(見ていない間は
                # 余計な前処理をしない)。
                # ただし、まだ1枚も作っていない間は見られていなくても作る(curl等の単発
                # アクセスや、ページを開いた直後の最初のリクエストが404になるのを防ぐ。
                # 以降は最後の1枚を保持するので、見ていない間は作らない)。
                collect_processed = preview_state is not None and (
                    preview_state.seconds_since_viewed() <= _PREVIEW_IDLE_TIMEOUT_SEC
                    or preview_state.snapshot_processed() is None
                )
                processed_tiles: List[Tuple[str, "np.ndarray"]] = []
                shown_scale = initial_scale_factor
                any_valid_read = False
                cycle_confirmed: Dict[str, Optional[int]] = {}
                cycle_estimated: Dict[str, bool] = {}
                for slot in rois:
                    value: Optional[int] = None
                    if frame is not None and slot in cycle_rois:
                        crop = crop_roi(cv2, frame, cycle_rois[slot])
                        scale_factor = (
                            preview_state.get_scale_factor()
                            if preview_state is not None
                            else initial_scale_factor
                        )
                        shown_scale = scale_factor
                        if collect_processed:
                            processed_tiles.append(
                                (slot, preprocess_for_ocr(cv2, crop, scale_factor, enhance))
                            )
                        value = recognize_digits(cv2, crop, recognizer, scale_factor, enhance)

                    entry = slot_info.get(slot, {})
                    menu_name = entry.get("name")
                    initial_count = entry.get("initial")
                    if (
                        value is not None
                        and isinstance(initial_count, int)
                        and value > initial_count * sanity_max_ratio
                    ):
                        # 初期残数に対して極端に大きい読み取り値は、縁の影を
                        # 数字と誤認する等の明らかな誤読とみなし、「読み取れ
                        # なかった」のと同じ扱いにしてスタビライザーに渡さない
                        # (誤読1回でstreakが乱れて確定が遅れるのを防ぐ)。
                        logger.warning(
                            "読み取り値が初期残数に対して極端に大きいため無視します: "
                            "%s = %s (初期残数=%s, 許容倍率=%.1f)",
                            slot, value, initial_count, sanity_max_ratio,
                        )
                        value = None

                    # 急変フィルタ(確定済みの実測値から減る/急増する読み取りを弾く)。
                    # 基準はlast_confirmed_value(実測の確定値のみ。TrendPredictorの
                    # 推定値は入らない)。棄却時は「読み取れなかった」扱いで、
                    # 同じ値が長く続けば強制受理される(JumpGuard参照)。
                    if value is not None:
                        before_guard = value
                        value, guard_reason = jump_guards[slot].check(
                            value, last_confirmed_value.get(slot)
                        )
                        if guard_reason:
                            logger.info(
                                "急変フィルタ: %s 読み取り=%s 確定値=%s: %s",
                                slot, before_guard, last_confirmed_value.get(slot), guard_reason,
                            )
                    if value is not None:
                        any_valid_read = True

                    confirmed = stabilizers[slot].update(value)
                    estimated = False
                    now_ts = time.time()
                    if value is not None:
                        fail_streak[slot] = 0
                        if confirmed is not None and last_confirmed_value.get(slot) != confirmed:
                            predictors[slot].add_sample(now_ts, confirmed)
                            last_confirmed_value[slot] = confirmed
                    else:
                        fail_streak[slot] += 1
                        if fail_streak[slot] >= predict_after_fails:
                            predicted = predictors[slot].predict(now_ts)
                            if predicted is not None:
                                # 増減ペースからの推測値を確定値の代わりに使う。
                                # 実測で読めるようになった時点でvalue is not None
                                # の分岐に戻り、この推測状態は自然に解除される。
                                logger.info(
                                    "推定値を使用します(%sが%d回連続で読み取れないため): "
                                    "%s = %d (傾向からの外挿)",
                                    slot, fail_streak[slot], slot, predicted,
                                )
                                confirmed = predicted
                                estimated = True

                    sent = False
                    if confirmed is not None:
                        if not menu_name:
                            # のこさへん本体はA/B/Cの3項目しか対応していないため、
                            # それ以外のスロット(カレー等)は送信先が無い。ただし
                            # 読み取れていること自体は分かるよう、値はログに出す
                            # (2026-09-24、ユーザー要望「全部の値を読み取りたい」対応)。
                            logger.info(f"読み取り(送信先未対応のため未送信): {slot} = {confirmed}")
                        elif last_sent.get(slot) == confirmed:
                            sent = True  # 前回と同じ値は再送しないが、送信済み扱いにする
                        else:
                            try:
                                post_sold_total(base_url, auth, menu_name, confirmed, estimated)
                                last_sent[slot] = confirmed
                                sent = True
                                logger.info(f"送信しました: {slot}({menu_name}) = {confirmed}")
                            except requests.RequestException as exc:
                                logger.warning(f"送信に失敗(次回再試行): {slot}={confirmed}: {exc}")

                    cycle_confirmed[slot] = confirmed
                    cycle_estimated[slot] = estimated
                    if preview_state is not None:
                        preview_state.update_reading(slot, value, confirmed, menu_name, sent, estimated)

                if numbers_writer.enabled and frame is not None:
                    numbers_writer.update(cycle_confirmed, cycle_estimated)

                if locker is not None and frame is not None:
                    if locker.note_cycle_result(any_valid_read):
                        logger.warning(
                            "全スロットが%d周期連続で読み取れないため、ROIのロックを解除して再検出します",
                            locker.reset_after_fail_cycles,
                        )
                        if preview_state is not None:
                            preview_state.update_dynamic_rois({})
                        _persist_locked_rois(None)
                if preview_state is not None and locker is not None:
                    preview_state.set_roi_lock("auto_lock", locker.is_locked, locker.progress, locker.required)

                if preview_state is not None and processed_tiles:
                    preview_state.update_processed(
                        _build_processed_montage(cv2, processed_tiles, shown_scale)
                    )
            except Exception as exc:  # noqa: BLE001
                # 1周期分の処理がどこで失敗してもループ自体は止めない
                # (person_target_detector.py等、他ノードと同じ縮退方針)。
                logger.error(f"読み取り処理中にエラー(継続します): {exc}")

            elapsed = time.monotonic() - start
            cur_params = preview_state.get_camera_params() if preview_state is not None else None
            if cur_params is not None:
                poll_interval = max(1.0, float(cur_params["poll_interval_sec"]))
            time.sleep(max(0.0, poll_interval - elapsed))
    finally:
        ctl.release()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH, help="設定ファイルのパス")
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="ROI位置合わせ用のグリッド付きスナップショットを1枚保存して終了する",
    )
    parser.add_argument("--verbose", action="store_true", help="デバッグログを出す")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    config = load_config(args.config)

    if args.calibrate:
        run_calibrate(config)
        return

    run_loop(config)


if __name__ == "__main__":
    main()
