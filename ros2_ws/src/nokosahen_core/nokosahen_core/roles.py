"""ダッシュボードのロール(権限レベル)と権限(capability)の定義(純Python、ROS非依存)。

【2026-10-13追加】docs/accounts.md。弱いアカウント(operator)の枠組み。

ロール(強さの順: viewer < operator < admin)
- admin   : 既存の強いアカウント(例: TL/ADMIN_ACCOUNT)。全権限。従来の動作と完全に同じ(このモジュールは何も制限しない)。
- operator: 新しい弱いアカウント。**デフォルト拒否の許可リスト方式**。何も許可していない間は「閲覧 +
            緊急停止(止める方向)+ 目的地キャンセル」だけ。許可は `operator_permissions`(設定)で1つずつ足す。
- viewer  : `guest`(閲覧のみ。従来どおり書き込み系は全て403、緊急停止ボタンも従来どおり押せない)。

権限(capability)は3段階
- GRANTABLE : operatorに付与できる。**付与しても、その変更は一時的(プログラム再起動で消える)** —
              書き込みの実行者のロールが admin 以外なら永続化しない(ephemeral_overrides.py)。
              このため「再起動で消える仕組みを実装済み」の権限だけがここに入る。
- DEFERRED  : 将来operatorに付与したいが、一時的な(再起動で消える)反映の仕組みが未実装のため、**現状は付与不可**
              (admin_only と同じ扱いで、設定に書いても起動時に弾く)。
- ADMIN_ONLY: ロボットの根幹・安全に関わる。**コードで固定。設定では絶対に付与できない**。

APIの分類表 `API_RULES`(メソッド+パス → 必要な権限)にない書き込みAPIは **admin専用**(デフォルト拒否)。
新しいAPIが後から増えても、表に足すまでは安全側(operatorは403)に倒れる。
"""
from __future__ import annotations

from typing import Any, Callable, Dict, FrozenSet, Iterable, List, NamedTuple, Optional, Set, Tuple

ROLE_ADMIN = "admin"
ROLE_OPERATOR = "operator"
ROLE_VIEWER = "viewer"
# 【2026-11-04追加】スマホ版の制限付きアカウント(restricted_account)専用ロール。以前はadmin扱いで、人の方へ動くスイッチ等・緊急停止の解除まで通っていた。
# 許可は「ログアウト・手動運転・緊急停止をかける」だけ(解除はTL=adminだけ)。operatorより狭いが、手動運転はoperatorに無い権限を持つ。
ROLE_RESTRICTED = "restricted"
ROLE_RANK: Dict[str, int] = {ROLE_VIEWER: 0, ROLE_RESTRICTED: 1, ROLE_OPERATOR: 2, ROLE_ADMIN: 3}

TIER_GRANTABLE = "grantable"
TIER_DEFERRED = "deferred"
TIER_ADMIN_ONLY = "admin_only"
TIER_SAFE = "safe"  # 全ロール(operator含む)が常に使える。止める方向・ログアウトだけ
# 【2026-11-06追加】ロールの既定で持たせる権限(operator・restricted・admin。viewer/guestは除く)。付与の対象ではない(設定で外せず、
# 許可リストの選択肢にも出ない)。文化祭で、弱いアカウントすべてが「人の方を向く」スイッチだけを切り替えられるようにするため。
TIER_ROLE_DEFAULT = "role_default"


class Capability(NamedTuple):
    name: str
    tier: str
    label: str  # 日本語の説明(管理画面・docs用)
    risk: str  # リスクの要約
    recommend: str  # 親がユーザーに確認するときの推奨: "許可" / "保留" / "-"


def _c(name: str, tier: str, label: str, risk: str = "", recommend: str = "-") -> Tuple[str, Capability]:
    return name, Capability(name, tier, label, risk, recommend)


CAPABILITIES: Dict[str, Capability] = dict([
    # ---- 常に許可(止める方向・ログアウト) ----
    _c("session.logout", TIER_SAFE, "ログアウト"),
    _c("safe.estop_engage", TIER_SAFE, "ソフト緊急停止をかける(止める方向だけ。解除は robot.estop_release)"),
    _c("safe.goal_cancel", TIER_SAFE, "移動中の目的地をキャンセルする(止める方向)"),
    _c("safe.schedule_stop", TIER_SAFE, "スケジュールの実行を止める(その場で待機し、ルール表の動作へ戻る。止める方向だけ。実行は schedule.run)"),

    # ---- 弱いアカウント(operator・restricted)が最初から持つ(2026-11-06。docs/accounts.md) ----
    _c("behavior.face_switch", TIER_ROLE_DEFAULT, "「人の方を向く」スイッチだけを切り替える(POST /api/face_enabled)",
       "ロボットがその場で旋回して人の方を向く(前には進まない。移動の安全層・緊急停止は従来どおり効く)。"
       "近づく・見回す・待機中の揺動・デモ強制起動は behavior.control(管理者専用)のままで、この権限では操作できない。", "-"),

    # 【2026-11-06】手動運転(WASD・スマホのボタン)。文化祭で班員(operator)も必要なときに動かせるように、ロールの既定にした(admin_onlyから移動)。
    # 安全上の前提: 手動運転は人が目で見て操作する。operatorの運転も、緊急停止(中は動かない)・速度上限(manual_linear/angular。速度設定は管理者専用のまま)・
    # watchdog(指令が途切れたら停止)・手動運転の人避け(drive.person_guardがONのとき前進停止)に従い、TLの運転中は破棄される(優先度が低い)。
    # viewer(guest)は持たない。緊急停止の解除は管理者専用のまま。
    _c("robot.manual_drive", TIER_ROLE_DEFAULT, "手動運転(PC版のWASD・スマホ版のボタン。POST /api/teleop)",
       "ロボットが実際に動く。人が目で見て操作する前提。緊急停止中は動かず、速度は管理者が決めた上限まで、TLの運転中は指令が破棄され、"
       "指令が途切れると止まる。人避け(drive.person_guard)がONなら人の近くで前進しない。緊急停止の解除・速度設定・自律走行へ戻す操作はできない。", "-"),

    # ---- operatorに付与できる(変更は一時的) ----
    _c("voice.levels", TIER_GRANTABLE, "発話・BGMの音量(通常/発話中)を変える",
       "音量を上げすぎると来場者に迷惑。範囲は0〜5倍で、既存の確認と上限はそのまま。", "許可"),
    _c("voice.master", TIER_GRANTABLE, "音声の全体ON/OFF(話す・BGM・文面生成をまとめて止める/再開)",
       "OFFにすると呼びかけが全て止まる(安全側の操作)。ONにし忘れると無音のまま。", "許可"),
    _c("voice.announce_interval", TIER_GRANTABLE, "呼びかけの発話間隔(10〜600秒)を変える",
       "短くしすぎると話しっぱなしになる(下限10秒、ディレクタ側にクールダウンもある)。", "許可"),
    _c("voice.announce_text", TIER_GRANTABLE, "言う内容(AIへの指示文・温度など)を変える/お試し生成",
       "不適切な指示文を入れると来場者に不適切な発話をするおそれ。お試し生成はCPUを使う。", "保留"),
    _c("voice.test_speak", TIER_GRANTABLE, "試し発話(その場でロボットに実際に喋らせる)",
       "来場者に聞こえる。連打すると騒がしい。", "保留"),
    _c("nav.goto_pin", TIER_GRANTABLE, "登録済みのピン(場所)へ移動させる",
       "ロボットが実際に動く。走行可能範囲・緊急停止中の拒否は従来どおり効くが、人の近くで動かすリスクは残る。", "保留"),
    _c("nav.wall_escape", TIER_GRANTABLE, "壁際で動けないとき、壁から離れる(通常の走行と同じ速さ0.7 m/sで、壁と逆向きへ短く動く。ボタン1回で最長10秒)",
       "ロボットが実際に少し動く。LiDARで角の衝突を先読みして、動ける向きがなければ動かない。緊急停止・手動運転・近接停止中は動かない(速度制限と衝突監視も通る)。速いので、人の近くでは管理者が許可するかを決める。", "保留"),
    _c("announce.test", TIER_GRANTABLE, "状況アナウンスを「試しに鳴らす」(その場で1回再生)",
       "来場者に聞こえる。設定は変えない。連打すると騒がしい。", "保留"),
    _c("display.survey_mode", TIER_GRANTABLE, "頭部ディスプレイの表示切替(お客さん向け/アンケート/OCRテスト/顔)",
       "アンケートに切り替えると回答中はロボットが停止する。", "許可"),
    _c("display.ocr_mock", TIER_GRANTABLE, "模擬POS(OCRテスト画面)の数字を変える/ランダム化/元に戻す",
       "OCRテスト用の画面だけ。実業務への影響は小さい。", "許可"),
    _c("display.menu_edit", TIER_GRANTABLE, "メニューの追加・名前の変更・削除・初期食数の変更(設定タブの「メニュー編集」)",
       "本日のメニュー一覧が変わると、お客さん向け表示(iPad)の品名と残数、ロボットの推奨メニューの判断が変わる。新しい一覧の全体を送って置き換える"
       "(削除したメニューは一覧から消え、残数・販売済み数も戻らない)。名前を変えると、旧名で送る別の入力元(厨房サイトの古い画面など)の残数は無視される。"
       "メモリ上の一覧だけ(再起動で消える)。iPad側の編集は厨房サイトと同じアカウントでログインしたiPadからで、この権限とは別。docs/ipad_display.md", "保留"),

    _c("schedule.run", TIER_GRANTABLE, "保存済みのスケジュール(何時にどこへ行きどうするかの時間割)を「実行」する(止めるのは誰でも可)",
       "ロボットが時間割どおりに実際に動く(ピンへの移動、走行範囲の解除を含む)。実行すると木のファイルが上書きされる。"
       "衝突監視・立入禁止・緊急停止などの安全層は常に有効。スケジュールの編集・保存は管理者のみ。docs/schedule.md", "保留"),

    _c("drive.person_guard", TIER_GRANTABLE, "手動運転の「人が近いとき前進を止める」安全機能のON/OFFを変える(OFFにすると安全が下がるため、この権限で変更全体を許可する)",
       "OFFにすると、手動運転で人の近くへ前進できてしまう(既定がOFFなので、効くのはONにした後だけ)。変更は一時的(再起動で元に戻る)。docs/manual_person_guard.md", "保留"),

    _c("system.shutdown", TIER_GRANTABLE, "プログラム終了(1台目のロボット関連プログラムを全部止める。既定ではラズパイ・2台目・1台目のOSも停止して電源を落とす。再起動は一式の電源の入れ直し)",
       "ロボット・管理画面が全て止まり、再起動は人がSSH/PCのスクリプトで行う。確認文字列の入力は従来どおり必要。"
       "モーターは先に止まる。未保存の地図を捨てる選択だけは管理者専用。docs/system_shutdown.md", "許可"),

    # ---- 将来付与したいが、一時的反映が未実装のため現状は付与不可 ----
    _c("voice.bgm_onoff", TIER_DEFERRED, "BGMのON/OFF(/api/bgmのenabled)",
       "voice_bridge側がメモリ上の状態のため、ダッシュボード再起動では戻らない。"),
    _c("announce.events", TIER_DEFERRED, "状況アナウンスの設定(言う内容・ON/OFF・間隔・音量。/api/announce_events)",
       "設定ファイルとvoice_bridgeの音量へ永続保存される。一時的な反映は未実装。"),
    _c("announce.pr_intro", TIER_DEFERRED, "文化祭の出店紹介の設定(文章・間隔・ON/OFF・音量)の保存と「音声を作る」(/api/pr_intro, /api/pr_intro/prepare)",
       "設定ファイル(~/.nokosahen_pr_intro.json)へ永続保存され、2台目のVOICEVOXで音声を合成する(CPU・時間を使う)。来場者に聞こえる文章を変えられる。一時的な反映は未実装。", "保留"),
    _c("display.ocr_quality", TIER_DEFERRED, "OCRカメラの画質プロファイル/自動調整",
       "OCR側(8091)が設定を保存する。一時的な反映は未実装。自動調整中はOCRが止まる。"),
    _c("nav.goto_point", TIER_DEFERRED, "地図上の任意の地点へ移動させる(/api/goal)",
       "ロボットが実際に動く。ピン以外の任意地点は危険度が高い。"),

    # 【2026-11-06】ソフト緊急停止の「解除」(POST /api/soft_estop で engaged が厳密に false のとき)。ユーザーの明示の決定で、operatorにもロール既定で許可
    # (以前は管理者専用)。restricted(restricted_account)は据え置き(解除不可)。viewer(guest)・未ログインは不可。
    # 安全上の注意: 物理の緊急停止スイッチは現状ファームで無効なので、ソフトの緊急停止が唯一の停止手段。解除は、人が機体から離れていることを
    # 目で確認してから行うこと。解除しても、運転の優先度(TL優先)・速度上限・人避け・watchdogは従来どおり。
    _c("robot.estop_release", TIER_ROLE_DEFAULT, "ソフト緊急停止の解除(POST /api/soft_estop で engaged:false。admin・operatorのみ。restrictedは不可)",
       "解除すると、止まっていた機体が再び動ける状態になる。人が機体から離れていることを確認してから行う。物理の緊急停止スイッチは現状無効。", "-"),

    # 【2026-11-07】文化祭で班員が電源投入のたびに「保存済みの地図(festival1)を読み込む」→「現在位置を指定」をするため、ユーザーの明示の決定で
    # 読み込みと初期位置の指定だけを operator にロール既定で許可した(restricted=restricted_accountは不可。guest・未ログインも不可)。
    # 作成開始/停止・保存・削除・リセット(map.lifecycle)、グローバル再位置合わせ(map.localize。ロボットを走らせる)は管理者専用のまま。
    # 安全上の注意: 読み込みはロボット全体の位置推定を作り直す。別の地図を選ぶと位置がずれる(festival1を選ぶ)。
    # 初期位置を大きく間違えると、走行可能範囲の保護や「向く」の基準がずれる。ロボットは動かさない(cmd_velは出ない)。
    _c("map.load", TIER_ROLE_DEFAULT, "保存済みの地図を読み込む(POST /api/mapping/load。位置推定を起動し直す)",
       "ロボット全体の位置推定を作り直す。別の地図を選ぶと位置がずれ、走行可能範囲・ピンの位置も合わなくなる。作成・保存・削除・リセットはできない。", "-"),
    _c("map.initial_pose", TIER_ROLE_DEFAULT, "現在位置を指定する(初期位置。POST /api/initial_pose。ロボットは動かない)",
       "位置を大きく間違えると、走行可能範囲の保護や「向く」の基準がずれる。「自動で位置を探す」(ロボットを走らせる)はできない。", "-"),

    # ---- ロボットの根幹・安全(operatorには絶対に付与できない) ----
    _c("robot.estop_config", TIER_ADMIN_ONLY, "緊急停止の設定変更(2026-11-06に解除と分離。現状は対応するAPIが無く、将来のために固定)",
       "緊急停止のしくみそのものが変わる。"),
    _c("robot.speed", TIER_ADMIN_ONLY, "走行速度・加減速・Nav2最高速度・自動(director)の速度", "衝突・暴走に直結。"),
    _c("robot.pid_firmware", TIER_ADMIN_ONLY, "PIDゲイン/ファーム/足回り/テスト走行/ステップ応答", "足回りの制御そのもの。"),
    _c("robot.exploration", TIER_ADMIN_ONLY, "探索ON/OFF", "ロボットが自走して動き回る。"),
    _c("nav.autonomy_resume", TIER_ADMIN_ONLY, "手動介入の解除(自律走行へ戻す)", "自律走行が再開する。"),
    _c("map.pins_edit", TIER_ADMIN_ONLY, "ピンの追加・更新・削除", "移動先の定義そのもの(永続データ)。"),
    _c("map.localize", TIER_ADMIN_ONLY, "グローバル再位置合わせ(「自動で位置を探す」。ロボットを少し走らせて探す。初期位置の指定は map.initial_pose に分離)", "自己位置が狂うと壁や人に衝突。"),
    _c("map.view", TIER_ADMIN_ONLY, "地図の向き(表示回転)の設定・保存", "地図の座標の見え方。永続データ。"),
    _c("map.lifecycle", TIER_ADMIN_ONLY, "地図の作成開始/停止/保存/削除/リセット(保存済みの地図の読み込みは map.load に分離)", "地図が壊れると運用不能。"),
    _c("map.boundary", TIER_ADMIN_ONLY, "走行可能範囲の設定・解除", "安全境界。"),
    _c("map.keepout", TIER_ADMIN_ONLY, "立入禁止エリアの追加・削除", "安全境界。"),
    _c("behavior.rules_edit", TIER_ADMIN_ONLY, "動作ルール(時間帯・状況)の保存/再読み込み", "動作条件が変わる(安全に関わる)。"),
    _c("behavior.tree_edit", TIER_ADMIN_ONLY, "behavior treeの保存/検証/リセット", "動作ロジックそのもの。"),
    _c("behavior.control", TIER_ADMIN_ONLY, "推奨発話ON/OFF・アイドル動作・デモ強制起動", "ロボットの動作モード。"),
    _c("crowd.calibrate", TIER_ADMIN_ONLY, "混雑判定のキャリブレーション", "混雑判定が狂うと動作判断が狂う。"),
    _c("camera.assign", TIER_ADMIN_ONLY, "決済横カメラの割り当て", "消費カウントの根拠。"),
    _c("accounts.manage", TIER_ADMIN_ONLY, "アカウント・権限の管理", "権限の昇格につながる。"),
    _c("ephemeral.manage", TIER_ADMIN_ONLY, "一時的な上書きの破棄", "管理者が判断する。"),
    _c("system.single_controller", TIER_ADMIN_ONLY, "同時1端末制限(enforce_single_controller)の設定", "操作衝突の防止。"),
    _c("system.shutdown_discard_map", TIER_ADMIN_ONLY, "プログラム終了時に「未保存の地図を捨てて終了」を選ぶ",
       "未保存の地図が消える。operatorは未保存の地図があるときは終了できない(先に管理者が保存する)。"),
    _c("system.secrets", TIER_ADMIN_ONLY, "パスワード・秘密情報(secrets)の設定", "API経由では存在しない。固定のため列挙。"),
    _c("admin.unclassified", TIER_ADMIN_ONLY, "未分類のAPI(表に載っていない書き込みAPIは全てここ)", "デフォルト拒否。"),
])

GRANTABLE: FrozenSet[str] = frozenset(n for n, c in CAPABILITIES.items() if c.tier == TIER_GRANTABLE)
DEFERRED: FrozenSet[str] = frozenset(n for n, c in CAPABILITIES.items() if c.tier == TIER_DEFERRED)
ADMIN_ONLY: FrozenSet[str] = frozenset(n for n, c in CAPABILITIES.items() if c.tier == TIER_ADMIN_ONLY)
SAFE_FOR_ALL: FrozenSet[str] = frozenset(n for n, c in CAPABILITIES.items() if c.tier == TIER_SAFE)
ROLE_DEFAULT: FrozenSet[str] = frozenset(n for n, c in CAPABILITIES.items() if c.tier == TIER_ROLE_DEFAULT)
# 設定に書いても付与できない(ADMIN_ONLY + DEFERRED)
NOT_GRANTABLE: FrozenSet[str] = ADMIN_ONLY | DEFERRED

# ロール別の権限(operatorは「常に許可」だけ。設定の許可リストは別途足す)
ROLE_BASE_CAPS: Dict[str, FrozenSet[str]] = {
    ROLE_VIEWER: frozenset(),
    ROLE_OPERATOR: SAFE_FOR_ALL | ROLE_DEFAULT,  # 【2026-11-06】+「人の方を向く」スイッチ
    # 【2026-11-04】restricted(restricted_account): ログアウト・緊急停止をかける・手動運転だけ(設定では増やせない。固定)。2026-11-06: +「人の方を向く」スイッチ(behavior.face_switch)
    ROLE_RESTRICTED: frozenset({"session.logout", "safe.estop_engage", "robot.manual_drive", "behavior.face_switch"}),  # 2026-11-06: manual_driveはoperatorも既定で持つ(TIER_ROLE_DEFAULT)
    ROLE_ADMIN: frozenset(CAPABILITIES),
}


class RoleTableError(AssertionError):
    pass


def validate_tables() -> None:
    """ロール表・分類表の整合を検証する(import時に実行。壊れていたら起動できない)。"""
    if not (ROLE_RANK[ROLE_VIEWER] < ROLE_RANK[ROLE_OPERATOR] < ROLE_RANK[ROLE_ADMIN]):
        raise RoleTableError("ロールの強さの順序が壊れています")
    admin = ROLE_BASE_CAPS[ROLE_ADMIN]
    # operatorは「付与しうる最大」でもadminを超えない
    if not (ROLE_BASE_CAPS[ROLE_OPERATOR] | GRANTABLE) <= admin:
        raise RoleTableError("operatorの権限がadminを超えています")
    if not ROLE_BASE_CAPS[ROLE_RESTRICTED] <= admin:
        raise RoleTableError("restrictedの権限がadminを超えています")
    if ROLE_BASE_CAPS[ROLE_RESTRICTED] & {"robot.estop_release", "behavior.control", "robot.speed", "accounts.manage"}:
        raise RoleTableError("restrictedに緊急停止の解除・動作スイッチ・速度・アカウント管理を持たせてはいけません")
    if "behavior.face_switch" in ROLE_BASE_CAPS[ROLE_VIEWER]:
        raise RoleTableError("viewer(guest)に「人の方を向く」スイッチを持たせてはいけません(読み取り専用)")
    if not ({"behavior.face_switch"} <= ROLE_BASE_CAPS[ROLE_OPERATOR] and {"behavior.face_switch"} <= ROLE_BASE_CAPS[ROLE_RESTRICTED]):
        raise RoleTableError("operator/restrictedに「人の方を向く」スイッチ(behavior.face_switch)がありません")
    # 【2026-11-06】手動運転: operatorとrestrictedは持ち、viewer(guest)は持たない
    # 【2026-11-06】緊急停止の解除: operatorは持つ。viewer・restrictedは持たない(restrictedは上の検査でも弾く)
    if "robot.estop_release" not in ROLE_BASE_CAPS[ROLE_OPERATOR]:
        raise RoleTableError("operatorに緊急停止の解除(robot.estop_release)がありません")
    if "robot.estop_release" in ROLE_BASE_CAPS[ROLE_VIEWER]:
        raise RoleTableError("viewer(guest)に緊急停止の解除を持たせてはいけません")
    if "robot.estop_config" in (ROLE_BASE_CAPS[ROLE_OPERATOR] | GRANTABLE):
        raise RoleTableError("緊急停止の設定変更(robot.estop_config)はadmin専用です")
    # 【2026-11-07】地図の読み込み・初期位置: operatorは持つ。viewerとrestrictedは持たない。作成/保存/リセット/グローバル再位置合わせはadmin専用
    for _cap in ("map.load", "map.initial_pose"):
        if _cap not in ROLE_BASE_CAPS[ROLE_OPERATOR]:
            raise RoleTableError(f"operatorに{_cap}がありません")
        if _cap in ROLE_BASE_CAPS[ROLE_VIEWER] or _cap in ROLE_BASE_CAPS[ROLE_RESTRICTED]:
            raise RoleTableError(f"viewer/restrictedに{_cap}を持たせてはいけません")
    if ({"map.lifecycle", "map.localize"} & (ROLE_BASE_CAPS[ROLE_OPERATOR] | GRANTABLE)):
        raise RoleTableError("map.lifecycle/map.localizeはadmin専用です")
    if "robot.manual_drive" in ROLE_BASE_CAPS[ROLE_VIEWER]:
        raise RoleTableError("viewer(guest)に手動運転(robot.manual_drive)を持たせてはいけません(読み取り専用)")
    if not ("robot.manual_drive" in ROLE_BASE_CAPS[ROLE_OPERATOR] and "robot.manual_drive" in ROLE_BASE_CAPS[ROLE_RESTRICTED]):
        raise RoleTableError("operator/restrictedに手動運転(robot.manual_drive)がありません")
    if not ROLE_BASE_CAPS[ROLE_VIEWER] <= ROLE_BASE_CAPS[ROLE_OPERATOR]:
        raise RoleTableError("viewerの権限がoperatorを超えています")
    if (ROLE_BASE_CAPS[ROLE_OPERATOR] | GRANTABLE) & NOT_GRANTABLE:
        raise RoleTableError("operatorに付与できる権限にadmin_only/deferredが混ざっています")
    for key, rule in API_RULES.items():
        for cap in _rule_static_caps(rule):
            if cap not in CAPABILITIES:
                raise RoleTableError(f"APIの分類表に未定義の権限があります: {key} -> {cap}")


# ---------------------------------------------------------------------------
# 書き込みAPIの分類表: (メソッド, パス) -> 権限名 / 動的ルール(本文を見て権限の一覧を返す関数)
# 表に無い書き込みAPIは admin 専用(admin.unclassified)。
# ---------------------------------------------------------------------------
Rule = Any  # str | Callable[[Dict[str, Any]], List[str]]


def _estop_rule(data: Dict[str, Any]) -> List[str]:
    # 【2026-11-06】解除(robot.estop_release)はoperatorもロール既定で可。restricted・guestは不可。厳密判定は変えない。
    # 【2026-11-04】解除は `engaged` が厳密に false のときだけ(ダッシュボードの実装 `data.get("engaged") is not False` と同じ式)。
    # 以前は bool(data.get("engaged", True)) で null・0・""・[] が「解除」になっていた。かける=安全側(全員)、解除=admin・operator
    return ["robot.estop_release"] if data.get("engaged") is False else ["safe.estop_engage"]


def _bgm_rule(data: Dict[str, Any]) -> List[str]:
    caps = []
    if "enabled" in data:
        caps.append("voice.bgm_onoff")  # 現状DEFERRED=operatorは不可
    if "volume" in data or "duck_volume" in data or not caps:
        caps.append("voice.levels")
    return caps


def _voice_settings_save_rule(data: Dict[str, Any]) -> List[str]:
    caps = []
    if "announce" in data:
        caps.append("voice.announce_interval")
    if "voice" in data:
        caps.append("voice.announce_text")
    return caps or ["voice.announce_text"]


API_RULES: Dict[Tuple[str, str], Rule] = {
    # セッション
    ("POST", "/api/logout"): "session.logout",
    # 止める方向(全員可)/解除(admin)
    ("POST", "/api/soft_estop"): _estop_rule,
    ("POST", "/api/goal/cancel"): "safe.goal_cancel",
    # 足回り・速度・手動
    ("POST", "/api/teleop"): "robot.manual_drive",
    # 手動運転の人避け(2026-10-24、docs/manual_person_guard.md)。ONは安全側だがOFFは安全を下げるので、POST全体を管理者専用にし、
    # operatorには許可リストの drive.person_guard で付与できる(guestは書込み扱いで403)。
    ("POST", "/api/manual_person_guard"): "drive.person_guard",
    ("POST", "/api/speed_settings"): "robot.speed",
    ("POST", "/api/pid_gains"): "robot.pid_firmware",
    ("POST", "/api/drive_test"): "robot.pid_firmware",
    ("POST", "/api/step_response/run"): "robot.pid_firmware",
    ("POST", "/api/exploration/start"): "robot.exploration",
    ("POST", "/api/exploration/stop"): "robot.exploration",
    ("POST", "/api/resume_autonomy"): "nav.autonomy_resume",
    # 移動
    ("POST", "/api/goal"): "nav.goto_point",
    ("POST", "/api/pins/goto"): "nav.goto_pin",
    ("POST", "/api/wall_escape/trigger"): "nav.wall_escape",
    ("POST", "/api/pins/add"): "map.pins_edit",
    ("POST", "/api/pins/update"): "map.pins_edit",
    ("POST", "/api/pins/delete"): "map.pins_edit",
    # 位置合わせ・地図・範囲
    ("POST", "/api/initial_pose"): "map.initial_pose",  # 2026-11-07: operatorも可
    ("POST", "/api/global_localization"): "map.localize",
    ("POST", "/api/map_view/set"): "map.view",
    ("POST", "/api/map_view/estimate"): "map.view",
    ("POST", "/api/map_view/save"): "map.view",
    ("POST", "/api/mapping/save"): "map.lifecycle",
    ("POST", "/api/mapping/start"): "map.lifecycle",
    ("POST", "/api/mapping/stop"): "map.lifecycle",
    ("POST", "/api/mapping/reset"): "map.lifecycle",
    ("POST", "/api/mapping/load"): "map.load",  # 2026-11-07: operatorも可。他のmapping/*はmap.lifecycle(管理者専用)
    ("POST", "/api/boundary/set"): "map.boundary",
    ("POST", "/api/boundary/clear"): "map.boundary",
    ("POST", "/api/keepout_zones/add"): "map.keepout",
    ("POST", "/api/keepout_zones/remove"): "map.keepout",
    ("POST", "/api/keepout_zones/clear"): "map.keepout",
    # 音声
    ("POST", "/api/voice_levels"): "voice.levels",
    ("POST", "/api/bgm"): _bgm_rule,
    ("POST", "/api/voice_master"): "voice.master",
    ("POST", "/api/voice_settings/save"): _voice_settings_save_rule,
    ("POST", "/api/voice_settings/preview"): "voice.announce_text",
    ("POST", "/api/voice_settings/speak"): "voice.test_speak",
    # 状況アナウンス
    ("POST", "/api/announce_events"): "announce.events",
    ("POST", "/api/announce_events/test"): "announce.test",
    # 文化祭の出店紹介(2026-10-30、docs/announcements.md)。文章の保存と音声の作成は保留(付与不可)、保存済み音声の試し再生は announce.test
    ("POST", "/api/pr_intro"): "announce.pr_intro",
    ("POST", "/api/pr_intro/prepare"): "announce.pr_intro",
    ("POST", "/api/pr_intro/test"): "announce.test",
    # 表示(頭部ディスプレイ・OCR)
    ("POST", "/api/survey/mode"): "display.survey_mode",
    ("POST", "/api/survey/auto"): "display.survey_mode",
    ("POST", "/api/survey/ocr_mock"): "display.ocr_mock",
    # メニュー編集(2026-10-27、docs/ipad_display.md)。TLの管理画面の設定タブから。iPad側(/display 配下のPOST)は、厨房サイトと同じアカウントで
    # ログインしたiPad専用のセッションで display_routes.py が扱い、ここ(管理画面の権限表)には載せない。
    ("POST", "/api/menu/edit"): "display.menu_edit",
    ("POST", "/api/ocr/profile"): "display.ocr_quality",
    ("POST", "/api/ocr/autotune"): "display.ocr_quality",
    # 動作
    ("POST", "/api/behavior_rules/save"): "behavior.rules_edit",
    ("POST", "/api/behavior_rules/reload"): "behavior.rules_edit",
    ("POST", "/api/behavior_tree"): "behavior.tree_edit",
    ("POST", "/api/behavior_tree/validate"): "behavior.tree_edit",
    ("POST", "/api/behavior_tree/reset"): "behavior.tree_edit",
    # スケジュール(2026-10-23、docs/schedule.md): 編集・保存・検証は管理者のみ(木のファイルに書かれる動作ロジック)。
    # 実行は許可リストで operator に付与できる(schedule.run)。停止は止める方向なので全員可。
    ("POST", "/api/schedule"): "behavior.tree_edit",
    ("POST", "/api/schedule/preview"): "behavior.tree_edit",
    ("POST", "/api/schedule/run"): "schedule.run",
    ("POST", "/api/schedule/stop"): "safe.schedule_stop",
    ("POST", "/api/recommend_enabled"): "behavior.control",
    ("POST", "/api/idle_wiggle"): "behavior.control",
    ("POST", "/api/person_interaction"): "behavior.control",
    ("POST", "/api/scan_enabled"): "behavior.control",
    # 【2026-11-06】向くだけは弱いアカウントにも許可(behavior.face_switch)。近づく・見回す・揺動・デモ強制は behavior.control(管理者専用)のまま
    ("POST", "/api/face_enabled"): "behavior.face_switch",
    ("POST", "/api/force_demo"): "behavior.control",
    ("POST", "/api/crowd_background/calibrate"): "crowd.calibrate",
    ("POST", "/api/checkout_cam_assign"): "camera.assign",
    # アカウント・一時的な上書きの管理
    ("POST", "/api/accounts/permissions"): "accounts.manage",
    ("POST", "/api/ephemeral/clear"): "ephemeral.manage",
    # プログラム終了(全停止)。operatorには許可リストで付与できる(2026-10-16)。未保存の地図を捨てる選択だけadmin専用。docs/system_shutdown.md
    ("POST", "/api/system/shutdown"): lambda d: ["system.shutdown"] + (["system.shutdown_discard_map"] if d.get("discard_map") else []),
}

# ログイン前に処理する(認可の対象外)
PUBLIC_POST_PATHS: FrozenSet[str] = frozenset({"/api/login"})


def _rule_static_caps(rule: Rule) -> List[str]:
    """検証用: ルールが返しうる権限(動的ルールは代表的な入力で列挙)。"""
    if isinstance(rule, str):
        return [rule]
    out: List[str] = []
    for sample in ({}, {"engaged": True}, {"engaged": False}, {"enabled": True}, {"volume": 1},
                   {"announce": {}}, {"voice": {}}):
        out.extend(rule(sample))
    return out


def required_capabilities(method: str, path: str, data: Optional[Dict[str, Any]] = None) -> List[str]:
    """このAPI呼び出しに必要な権限の一覧(全部持っている必要がある)。表に無ければ admin.unclassified。"""
    rule = API_RULES.get((method.upper(), path))
    if rule is None:
        return ["admin.unclassified"]
    if isinstance(rule, str):
        return [rule]
    try:
        return list(rule(data if isinstance(data, dict) else {}))
    except Exception:  # noqa: BLE001 - 本文が壊れていても安全側(admin専用)
        return ["admin.unclassified"]


class PermissionConfigError(ValueError):
    pass


def sanitize_operator_permissions(
    names: Iterable[Any], strict: bool = False
) -> Tuple[FrozenSet[str], List[str]]:
    """設定の許可リストを検証する。(有効な権限, 問題点の一覧)。

    - 未知の名前・admin_only・deferred は**付与しない**(無視して問題点に記録)。strict=Trueなら例外。
    - SAFE(常に許可)は重複なので問題にせず、そのまま受け入れる(効果なし)。
    """
    granted: Set[str] = set()
    problems: List[str] = []
    for raw in names or []:
        name = str(raw).strip()
        cap = CAPABILITIES.get(name)
        if cap is None:
            problems.append(f"未知の権限名のため無視: {name}")
        elif cap.tier in (TIER_ADMIN_ONLY, TIER_DEFERRED):
            kind = "管理者専用(ロボットの根幹)" if cap.tier == TIER_ADMIN_ONLY else "一時的反映が未実装で現状は付与不可"
            problems.append(f"{name} は operator に付与できません({kind})。無視しました")
        else:
            granted.add(name)
    if strict and problems:
        raise PermissionConfigError(" / ".join(problems))
    return frozenset(granted), problems


def role_caps(role: str, operator_permissions: Iterable[str] = ()) -> FrozenSet[str]:
    """ロールの持つ権限。operatorは常時許可 + 検証済みの許可リスト(admin_onlyは必ず除外)。"""
    if role == ROLE_ADMIN:
        return ROLE_BASE_CAPS[ROLE_ADMIN]
    if role == ROLE_OPERATOR:
        extra, _ = sanitize_operator_permissions(operator_permissions)
        return ROLE_BASE_CAPS[ROLE_OPERATOR] | extra
    return ROLE_BASE_CAPS.get(role, frozenset())


class Decision(NamedTuple):
    allowed: bool
    missing: Tuple[str, ...]  # 足りない権限
    required: Tuple[str, ...]  # このAPIに必要な権限
    admin_only: bool  # 不足分にadmin_only/deferred(=設定では付与できない)が含まれる


def authorize(
    role: str, operator_permissions: Iterable[str], method: str, path: str,
    data: Optional[Dict[str, Any]] = None,
) -> Decision:
    """書き込みAPIを実行してよいか。adminは常に許可(従来動作のまま)。"""
    req = tuple(required_capabilities(method, path, data))
    if role == ROLE_ADMIN:
        return Decision(True, (), req, False)
    have = role_caps(role, operator_permissions)
    missing = tuple(c for c in req if c not in have)
    locked = any(CAPABILITIES[c].tier in (TIER_ADMIN_ONLY, TIER_DEFERRED) for c in missing)
    return Decision(not missing, missing, req, locked)


def describe_capabilities() -> List[Dict[str, Any]]:
    """管理画面・docs用: 全権限の一覧。"""
    return [
        {"name": c.name, "tier": c.tier, "label": c.label, "risk": c.risk, "recommend": c.recommend}
        for c in CAPABILITIES.values()
    ]


validate_tables()
