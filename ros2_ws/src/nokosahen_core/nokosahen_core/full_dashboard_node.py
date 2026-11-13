"""完成版(ESP32+Jetson+LiDAR+Webカメラ+決済横カメラによる残数カウント有効)
専用の監視・操作ダッシュボード。全体監視用の総合HTMLで、Stage2ダッシュボードの
内容(混雑・行動・発話・estop・地図クリック運転)に加えて、メニューの残数
カウント状況も表示する。
"""
from __future__ import annotations

import math
import time
from typing import Any, Dict

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .assets import LOGO_DATA_URI
from .consumption_log import ConsumptionLog
from .detection_log import DetectionLog
from .bgm_proxy import BgmProxy
from .voice_master_proxy import VoiceMasterProxy
from .voice_levels_proxy import VoiceLevelsProxy
from .survey_admin_proxy import SurveyAdminProxy
from .display_hub import DEFAULT_MODE_FILE as _DISPLAY_MODE_FILE, DisplayHub, LocalDisplayAdmin, build_customer_payload
from .display_survey import SurveyStore
from .ocr_admin_proxy import OcrAdminProxy
from .announce_settings import AnnounceSettingsStore
from .announce_events import AnnounceEventsStore
from .announce_events_proxy import AnnounceBridgeProxy, AnnounceEventsController
from .pr_intro import PrIntroController, PrIntroStore
from .announce_settings_page import build_announce_settings_html
from .voice_settings_proxy import VoiceSettingsProxy
from .voice_settings_controller import VoiceSettingsController
from .voice_settings_page import build_voice_settings_html
from .engagement_log import EngagementLog
from .utterance_log import UtteranceLogReader
from .secrets_loader import resolve_password
from .dashboard_common import (
    DASHBOARD_ODOM_TOPIC,
    AutoSpeedController,
    BehaviorDirectorControlController,
    BoundaryController,
    CameraPreviewController,
    CheckoutCamRegistry,
    CrowdBackgroundController,
    FrontierExplorerController,
    KeepOutZoneController,
    LocalizationAssistController,
    MapGoalController,
    WallEscapeController,
    MapLifecycleController,
    MappingController,
    PidGainsController,
    PinController,
    RobotPoseController,
    RouteTrailController,
    ScanController,
    StepResponseResultsController,
    TeleopController,
    build_activity_report_html,
    build_audience_display_html,
    build_consumption_graph_html,
    build_detection_log_html,
    build_map_editor_html,
    build_recommendation_effectiveness_html,
    build_step_response_html,
    build_utterance_log_html,
    BehaviorRulesController,
    make_dashboard_handler,
    make_mobile_handler,
    start_dashboard_server,
)
from .behavior_rules_page import build_behavior_rules_html
from .json_tools import dumps, loads
from .moving_notice import MovingNotice
from .nav_speed import NavSpeedController
from .map_view import MAP_VIEW_JS
from .map_pins import get_pin_store
from .map_pins import pins_payload as _tree_pins_payload
from .behavior_tree_page import build_behavior_tree_html
from .behavior_tree_store import BehaviorTreeController
from .schedule_page import build_schedule_html
from .schedule_controller import ScheduleController
from .schedule_display import DEFAULT_BASELINE_FILE as _SCHEDULE_DISPLAY_BASELINE, DisplayScheduleController
from .dashboard_common import LazySubscription as _TreeLazySub
from .dashboard_common import String as _TreeString
from .dashboard_common import resolve_enforce_single_controller
from .nav_layout import NAV_CSS, NAV_JS, NK_FIELD_JS, link_cards_html, nav_drawer_html, nav_header_html
from .accounts import AccountStore
from .accounts_ui import ACCOUNTS_CARD_HTML, ACCOUNTS_CARD_JS
from .system_shutdown import (
    SystemShutdownController,
    build_map_state,
    build_motor_stopper,
    build_nav_state,
)
from .system_shutdown_ui import SHUTDOWN_CARD_HTML, SHUTDOWN_CARD_JS
from .ephemeral_overrides import DEFAULT_JOURNAL_FILE, EphemeralOverrides

HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1" />
<title>のこさへん - 完成版 総合ダッシュボード</title>
<style>
__NAV_CSS__
</style>
</head>
<body>
__NAV_HEADER__
__NAV_DRAWER__
<main>

  <!-- ===== タブ1: ダッシュボード(いまの状況を一目で。詳しい操作は各タブへ) ===== -->
  <!-- 【2026-10-10再設計】先頭に「いまの状況」(ロボット/行動/位置/音声/BGM/アンケート/OCR)。管理メニューは重複していたので折りたたみに。 -->
  <div id="tab-dash" class="tabpane" data-pane="dash">
    <section class="card s-summary" id="statusStrip">
      <div class="card-head"><h2>いまの状況</h2><span class="card-desc" style="margin:0;">押すと詳しいタブへ移動します</span></div>
      <div class="sumgrid">
        <a class="sumitem" href="#tab-dash" data-goto="dash"><span class="sum-l">ロボットの状態</span><b id="sumStop" class="sbadge sb-mute">-</b></a>
        <a class="sumitem" href="#tab-dash" data-goto="dash"><span class="sum-l">行動</span><b id="sumMode" class="sbadge sb-mute">-</b></a>
        <a class="sumitem" href="#tab-map" data-goto="map"><span class="sum-l">位置の確かさ</span><b id="sumLoc" class="sbadge sb-mute">-</b></a>
        <a class="sumitem" href="#tab-voice" data-goto="voice"><span class="sum-l">音声(全体)</span><b id="sumVoice" class="sbadge sb-mute">-</b></a>
        <a class="sumitem" href="#tab-voice" data-goto="voice"><span class="sum-l">BGM</span><b id="sumBgm" class="sbadge sb-mute">-</b></a>
        <a class="sumitem" href="#tab-settings" data-goto="settings"><span class="sum-l">頭部ディスプレイ</span><b id="sumSurvey" class="sbadge sb-mute">-</b></a>
        <a class="sumitem" href="#tab-settings" data-goto="settings"><span class="sum-l">タブレットOCR</span><b id="sumOcr" class="sbadge sb-mute">-</b></a>
      </div>
    </section>
    <div class="dash-col">
      <section class="card s-state">
        <div class="card-head"><h2>ロボットの状態</h2></div>
        <div class="metrics">
          <div class="metric"><span>混雑度</span><b id="crowdLevel">-</b></div>
          <div class="metric"><span>行動モード</span><b id="mode">-</b><em id="modeJa" class="metric-sub"></em></div>
          <div class="metric"><span>適用中ルール</span><b id="activeRule">-</b></div>
          <div class="metric"><span>発話中</span><b id="audioBusy">-</b></div>
          <div class="metric"><span>手動運転</span><b id="manualOverride">-</b></div>
          <div class="metric"><span>ロボットの位置(odom x,y)</span><b id="odomXY">-</b></div>
        </div>
      </section>
      <section class="card s-speech">
        <div class="card-head"><h2>いま話している内容</h2></div>
        <p id="announce">-</p>
      </section>
    </div>
    <div class="dash-col">
      <section class="card s-menu">
        <div class="card-head"><h2>メニュー残数</h2></div>
        <div class="metrics">
          <div class="metric"><span>フェーズ</span><b id="menuPhase">-</b></div>
          <div class="metric"><span>宣伝対象</span><b id="promoteItem">-</b></div>
        </div>
        <div class="tblwrap">
          <table>
            <thead><tr><th>メニュー</th><th>初期</th><th>残数</th><th>販売</th></tr></thead>
            <tbody id="menuRows"><tr><td colspan="4">-</td></tr></tbody>
          </table>
        </div>
        <p class="hint">提供数の登録・訂正は厨房用UI(ポート8080)で行います。</p>
      </section>
    </div>
    <details class="card s-links">
      <summary>管理メニュー(すべての画面へのリンク。右上の「メニュー」からも開けます)</summary>
      __NAV_LINKS_ALL__
    </details>
  </div>

  <!-- ===== タブ2: 地図・走行(地図が主役。地図+運転+センサーを1画面に) ===== -->
  <div id="tab-map" class="tabpane" data-pane="map">
    <section class="card s-map">
      <div class="map-head">
        <h2>地図 <small>(クリックでそこへ自動走行)</small></h2>
        <span class="map-mode">地図モード: <b id="mapModeLabel">-</b></span>
        <span class="map-loc"><span id="locChip" class="chip chip-mute">位置の確かさ: -</span> <span id="locDetail" class="loc-detail"></span></span>
      </div>
      <!-- 【2026-10-08】地図ツールは1本のツールバーに集約(全体表示・回転はmap_view.pyが先頭に差し込む)。長い説明は「?」/title、細かい設定はポップオーバー。 -->
      <div id="mapToolbar" class="map-toolbar" role="toolbar" aria-label="地図ツール">
        <button id="poseToolBtn" type="button" class="tb-btn" data-cap="map.initial_pose" title="保存した地図を読み込んだら、まずロボットの実際の位置と向きを教えてください(ロボットは動きません)">現在位置を指定</button>
        <button id="globalLocBtn" type="button" class="tb-btn" title="地図全体から位置を探し直します(全体位置推定)。ロボットを少し走らせると位置が定まります">自動で位置を探す</button>
        <button id="goalHeadBtn" type="button" class="tb-btn" title="ONの間、地図をドラッグすると到着時の向きも指定します(地図の移動はできません)">向き指定: OFF</button>
        <span class="tb-wrap">
          <button id="pinMenuBtn" type="button" class="tb-btn" data-pop="pinPop" aria-expanded="false" aria-controls="pinPop" title="地図ピン(名前付きの場所)を立てる">ピン &#9662;</button>
          <!-- 【2026-10-07追加】地図ピン: 現在位置に立てる / 地図で置く。一覧は右カラムの「ピン」カード。 -->
          <div id="pinPop" class="tb-pop" hidden>
            <div class="tb-row">
              <input id="pinNewName" type="text" maxlength="40" placeholder="ピンの名前(空欄で自動)" aria-label="ピンの名前" />
              <details id="pinColorDd" class="color-dd">
                <summary aria-label="ピンの色"><span id="pinColorDot" class="pin-dot"></span>色: <b id="pinColorTxt">自動</b></summary>
                <span id="pinNewSwatches" class="pin-swatches-row" aria-label="ピンの色"></span>
              </details>
            </div>
            <div class="tb-row">
              <button id="pinHereBtn" type="button" class="primary" data-close="1">現在位置にピンを立てる</button>
              <button id="pinToolBtn" type="button" data-close="1">地図でピンを置く</button>
            </div>
            <p class="hint" style="margin:0;">「向き指定」をONにすると、地図クリックの目的地にも到着時の向きを付けられます(ツールバーのボタン)。</p>
          </div>
        </span>
        <span class="tb-wrap tb-right">
          <button id="mapHelpBtn" type="button" class="tb-btn tb-icon" data-pop="mapHelpPop" aria-expanded="false" aria-controls="mapHelpPop" aria-label="地図の使い方" title="地図の使い方">?</button>
          <div id="mapHelpPop" class="tb-pop tb-help" hidden>
            <p><b>地図の見方</b> 緑の線=実際に通った軌跡、青の破線=Nav2の計画経路。クリックした地点へNav2が経路計画して移動します。ホイール/ピンチで拡大縮小、ドラッグで移動できます。</p>
            <p><b>現在位置を指定</b> 保存した地図を読み込んだら、まずロボットの実際の位置と向きを教えてください(ロボットは動きません)。<b>自動で位置を探す</b>は、地図全体から探し直します(ロボットを少し走らせると位置が定まります)。</p>
            <p><b>回転</b> 表示だけを回します(地図・Nav2の座標は変わりません)。走行範囲は表示で水平・垂直に見える矩形になります。</p>
            <p><b>ピン</b> ピンは地図(mapフレーム)上の位置と向きで保存され、「行く」でその姿勢へそのまま向かいます(現在位置からNav2が経路を計画)。地図を保存すると一緒に保存され、読み込むと復元されます。地図上のピンをタップしても選べます。位置の精度は自己位置推定(SLAM/AMCL)の誤差が限界です。</p>
          </div>
        </span>
      </div>
      <div id="mapStage" class="map-stage">
        <canvas id="mapCanvas"></canvas>
        <!-- 通知ストリップ: 状況・確認バーは地図下端に重ねる(キャンバスの大きさを変えない)。緊急停止ドックより下の層。 -->
        <div id="mapNotes" class="map-notes" aria-live="polite">
          <p id="poseHint" class="map-note" title="クリックで消す"></p>
          <p id="pinHint" class="map-note" title="クリックで消す"></p>
          <p id="wallStuckNote" class="map-note wall-stuck-note" role="alert"></p>
          <div id="pinPlaceBar" class="loc-bar" hidden>
            <span id="pinPlaceText">-</span>
            <button id="pinPlaceOkBtn" type="button" class="primary">置く</button>
            <button id="pinPlaceRetryBtn" type="button">やり直し</button>
            <button id="pinPlaceCancelBtn" type="button">やめる</button>
          </div>
          <div id="pinSelBar" class="loc-bar pin-sel-bar" hidden>
            <span id="pinSelText">-</span>
            <button id="pinSelGoBtn" type="button" class="primary" data-cap="nav.goto_pin">このピンへ行く</button>
            <button id="pinSelCloseBtn" type="button">閉じる</button>
          </div>
          <div id="goalProgressBar" class="loc-bar loc-bar-info" hidden>
            <span id="goalProgressText">-</span>
            <button id="goalCancelRunBtn" type="button" class="danger">中止</button>
            <button id="goalCloseBtn" type="button">閉じる</button>
          </div>
          <p id="locWarn" class="loc-warn" hidden>クリックで走行するのは位置の確かさが「良好」になってからにしてください</p>
          <div id="poseConfirmBar" class="loc-bar" data-cap="map.initial_pose" hidden>
            <span id="poseConfirmText">-</span>
            <button id="poseSendBtn" type="button" class="primary">送信</button>
            <button id="poseRetryBtn" type="button">やり直し</button>
          </div>
          <div id="goalConfirmBar" class="loc-bar loc-bar-warn" hidden>
            <span id="goalConfirmText">位置が不確かです。このまま走行しますか？</span>
            <button id="goalGoBtn" type="button" data-cap="nav.goto_pin">走行する</button>
            <button id="goalCancelBtn" type="button" class="primary">やめる</button>
          </div>
        </div>
      </div>
    </section>

    <div class="sidecol">
      <!-- 【2026-10-08】ピン一覧は右カラムの独立カード(常時表示・折りたたみ可)。地図上のピンを選ぶと一覧でも強調される。 -->
      <section class="card s-pins">
        <details id="pinsCard" class="pins-card" open>
          <summary>ピン(名前付きの場所) <small id="pinCount">0 / 200</small></summary>
          <div class="pins-body">
            <p id="pinNote" class="hint" hidden></p>
            <div id="pinList" class="pin-list"></div>
          </div>
        </details>
      </section>

      <section class="card s-drive">
        <div class="card-head"><h2>運転</h2><span id="driveBadge" class="sbadge sb-mute">-</span></div>
        <p class="card-desc">手動運転になっているときは、ここから自律運転に戻します。</p>
        <button id="resumeBtn" class="resume-btn">自律運転に戻す</button>
        <!-- 【2026-10-22】壁際で動けないとき、壁から低速で離れる(/api/wall_escape/trigger)。壁が近く動けない(stuck)ときは目立たせる。 -->
        <button id="wallEscapeBtn" type="button" class="resume-btn we-btn" data-cap="nav.wall_escape" disabled>壁から離れる</button>
        <p id="wallEscapeNote" class="hint" role="status" aria-live="polite">-</p>
        <!-- 【2026-11-02追加】位置が不確かなときの走行前の確認のON/OFF(既定OFF)。この端末のブラウザだけの設定。 -->
        <button id="locConfirmBtn" type="button" aria-pressed="false" style="margin-top:var(--s2);">位置が不確かなときの走行前の確認: OFF</button>
        <p class="hint">ONにすると、位置の確かさが「良好」でないときに、目的地クリック・ピンの「行く」の前に「このまま走行しますか?」と聞きます。OFFのときは、聞かずに走行します(この端末のブラウザだけの設定)。</p>
        <p class="hint">手動運転(D-pad)はスマホ版から: <b id="mobileUrl">-</b>(同じログイン)。PCはW/A/S/Dキー、Spaceで緊急停止。</p>
        <!-- 【2026-10-24】手動運転の人避け(既定OFF)。ONで、LiDARが前方0.5 m以内に人を見つけている間は手動運転でも前進しない。docs/manual_person_guard.md -->
        <div class="mpg" id="mpgBlock" data-cap="drive.person_guard" style="margin-top:var(--s3);padding-top:var(--s3);border-top:1px solid var(--line);">
          <div class="card-head" style="margin-bottom:var(--s2);">
            <label for="mpgToggle" style="display:flex;align-items:center;gap:var(--s2);font-weight:700;cursor:pointer;">
              <input id="mpgToggle" type="checkbox" role="switch" style="width:18px;height:18px;"> 手動運転で人が近いとき前進を止める
            </label>
            <span id="mpgBadge" class="sbadge sb-mute">-</span>
          </div>
          <p class="hint" style="margin-top:0;">ONにすると、LiDARで前方0.5 m以内に人を見つけたとき、手動運転でも前進しません(後退・旋回はできます)。人物検出が動いていない構成では効きません。基本はOFFです。</p>
          <p id="mpgBlocked" class="sbadge sb-bad" role="status" aria-live="polite" style="display:none;">人が近いので前進を止めています</p>
          <p id="mpgWarn" class="sbadge sb-warn" role="status" aria-live="polite" style="display:none;">人物検出が動いていないため人避けは効きません</p>
          <p id="mpgNote" class="hint" role="status" aria-live="polite" style="display:none;"></p>
        </div>
        <details class="fold" id="speedFold" style="margin-top:var(--s3);">
          <summary>速度の設定(手動・自動・Nav2)</summary>
          <div class="fold-body">
        <h3 style="margin-top:4px;">最高速度設定 <small style="font-weight:400;">(手動の上限: 直進1.2m/s・旋回4rad/s、自動の上限: 直進1.1m/s・旋回3rad/s。超える値は丸められます)</small></h3>
        <div class="speed-grid">
          <label>手動直進(m/s) <input id="manualLinear" type="number" step="0.01" min="0.01" max="1.2"></label>
          <label>手動旋回(rad/s) <input id="manualAngular" type="number" step="0.01" min="0.01" max="4"></label>
          <label>自動直進(m/s) <input id="autoLinear" type="number" step="0.01" min="0.01" max="1.1"></label>
          <label>自動旋回(rad/s) <input id="autoAngular" type="number" step="0.01" min="0.01" max="3"></label>
        </div>
        <div class="btns"><button id="speedSaveBtn" class="primary" style="flex:0 0 96px;">保存</button><p class="hint" id="speedSaveHint" style="flex:1 1 140px;margin:0;align-self:center;">&nbsp;</p></div>
        <div id="speedConfirmBar" role="alert" style="display:none;margin-top:8px;padding:8px;border:2px solid #b45309;border-radius:8px;background:#fff7e6;color:#5c2d00;">
          <span id="speedConfirmText"></span>
          <div class="btns" style="margin-top:6px;"><button id="speedConfirmYes" type="button" class="primary">上げる</button><button id="speedConfirmNo" type="button">やめる</button></div>
        </div>
        <p class="hint">手動の直進が1.1m/s、旋回が3rad/sを超える値は確認が必要です。広い場所で、電源を切れる状態で試してください。急な旋回は転倒やLiDARの歪みの原因になります。立ち上がりの速さはファームの加速制限で決まり、上限を上げても変わりません。</p>
        <!-- 【2026-10-07追加】手動直進・director自動直進・Nav2最高速度を1回で同じ値にする(旋回は変えない)。 -->
        <h3 style="margin-top:12px;">全て同じ速度にする <small style="font-weight:400;">(手動・director自動・Nav2の直進。旋回は変わりません)</small></h3>
        <div class="speed-grid">
          <label>全て同じ速度にする [m/s] <input id="allSpeed" type="number" step="0.05" min="0.1" max="1.1"></label>
        </div>
        <div class="btns"><button id="allSpeedBtn" class="primary" style="flex:0 0 160px;">全て同じ速度にする</button><p class="hint" id="allSpeedHint" style="flex:1 1 140px;margin:0;align-self:center;">&nbsp;</p></div>
        <ul id="allSpeedResult" class="hint" style="margin:4px 0 0 16px;padding:0;"></ul>
        <!-- 【2026-10-07追加】Nav2(ゴール追従)の最高速度。上の「自動直進」(behavior_directorの接近/見回り速度)とは別の設定。 -->
        <h3 style="margin-top:12px;">自動走行(Nav2)の最高速度 <small style="font-weight:400;">(既定0.5。手動と同程度の1.1m/sまで)</small></h3>
        <div class="speed-grid">
          <label>自動走行(Nav2)の最高速度 [m/s] <input id="navSpeed" type="number" step="0.05" min="0.1" max="1.1"></label>
        </div>
        <div class="btns"><button id="navSpeedBtn" class="primary" style="flex:0 0 96px;">適用</button><p class="hint" id="navSpeedHint" style="flex:1 1 140px;margin:0;align-self:center;">&nbsp;</p></div>
        <div id="navConfirmBar" role="alert" style="display:none;margin-top:8px;padding:8px;border:2px solid #b45309;border-radius:8px;background:#fff7e6;color:#5c2d00;">
          <span id="navConfirmText"></span>
          <div class="btns" style="margin-top:6px;"><button id="navConfirmYes" type="button" class="primary">上げる</button><button id="navConfirmNo" type="button">やめる</button></div>
        </div>
        <p class="hint" id="navSpeedInfo">Nav2の速度(地図クリック移動・探索・ルート巡回など)と、上の「自動直進」(behavior_directorの接近・見回り速度)は別の設定です。</p>
          </div>
        </details>
      </section>

      <section class="card s-folds">
        <div class="card-head"><h2>地図の操作</h2></div>
        <p class="card-desc">地図の作成・保存・読み込み、走行範囲、自律探索。</p>
        <details class="fold">
          <summary>地図の作成・保存・読込</summary>
          <div class="fold-body">
            <div class="btns">
              <button id="mapStartBtn" class="primary">地図作成を開始</button>
              <button id="mapStopBtn">停止</button>
            </div>
            <div class="btns">
              <input id="mapSaveName" type="text" placeholder="保存名(英数字・_・-、空欄なら自動命名)" />
              <button id="mapSaveBtn">地図を保存</button>
            </div>
            <!-- 【2026-11-07】目印(data-cap)map.load: 弱いアカウント(operator)も保存済みの地図を読み込める。別の地図を選ぶと位置がずれるので、festival1を選ぶ -->
            <div class="btns" data-cap="map.load">
              <select id="mapLoadSelect"></select>
              <button id="mapLoadBtn">この地図を読み込む</button>
            </div>
            <p id="mapSaveHint" class="hint"></p>
            <!-- 危険な操作は他の操作と離して最後に置く(押すと確認が出る)。 -->
            <div class="btns danger-zone">
              <button id="mapResetBtn" class="warnbtn" title="作成中の地図を破棄します(確認が出ます)">作成中の地図をリセット(破棄)</button>
            </div>
            <p class="hint" style="margin-top:4px;">リセットすると、保存していない作成中の地図は失われます。</p>
          </div>
        </details>
        <details class="fold">
          <summary>走行範囲・立入禁止エリア</summary>
          <div class="fold-body">
            <div class="btns">
              <button id="boundarySetBtn">走行範囲を設定(地図を2回クリック)</button>
              <button id="boundaryClearBtn">範囲解除</button>
            </div>
            <p id="boundaryHint" class="hint">黄色い点線が現在の走行可能範囲(自律動作と地図クリックの目的地に適用、WASD手動運転は対象外)。四隅の□をドラッグで調整できます。</p>
            <div class="btns">
              <a class="linkbtn" href="/map_editor" target="_blank">地図を大きく開いて範囲を編集</a>
            </div>
            <p class="hint">走行可能範囲(外周)に加えて、地図内の「立入禁止エリア」を複数指定できます(別タブで開きます)。</p>
          </div>
        </details>
        <details class="fold">
          <summary>自律探索</summary>
          <div class="fold-body">
            <div class="btns">
              <button id="explorationStartBtn" class="primary">自律探索を開始</button>
              <button id="explorationStopBtn">停止</button>
            </div>
            <p id="explorationHint" class="hint">地図作成中に自分で走り回って探索する(手動運転が入ったら即座にそちらを優先)。</p>
          </div>
        </details>
      </section>

      <section class="card s-sensors">
        <div class="card-head"><h2>センサー(LiDAR・カメラ)</h2></div>
        <div class="sensor-row">
          <figure class="sensor-lidar">
            <canvas id="lidarCanvas" width="360" height="360"></canvas>
            <figcaption>LiDAR(自機中心・前方が上)。緑の三角=自機、赤点=障害物、円=1m刻み。</figcaption>
          </figure>
          <figure>
            <img id="cameraView" alt="カメラ映像なし" />
            <figcaption>正面カメラ。緑枠=人物として認識した範囲(数字は確度)。数百ms毎の静止画更新です。</figcaption>
          </figure>
          <figure class="sensor-half">
            <img id="cameraViewLeft" alt="左後ろカメラ映像なし" />
            <figcaption>左後ろ。緑枠=人物として認識した範囲。</figcaption>
          </figure>
          <figure class="sensor-half">
            <img id="cameraViewRight" alt="右後ろカメラ映像なし" />
            <figcaption>右後ろ。緑枠=人物として認識した範囲。</figcaption>
          </figure>
        </div>
      </section>
    </div>
  </div>

  <!-- ===== タブ3: 発話・音声(全体ON/OFF → 音量・BGM → 発話の履歴/設定 → 実験・デモ) ===== -->
  <div id="tab-voice" class="tabpane" data-pane="voice">
    <!-- 【2026-10-09追加】音声の全体ON/OFF(voice_bridgeの/voice_masterをダッシュボード経由で操作)。
         OFFの間は発話・AIによる文面生成・BGMがすべて止まる(docs/voice_master.md)。voice_master_url空なら非表示。 -->
    <!-- 【2026-10-09追加】音量(発話・BGM通常・BGM発話中を別々に調整、voice_bridgeの/voice_levelsをダッシュボード経由で操作)。
         【2026-10-10】旧BGMカードをこのカードの下半分に統合(BGMの音量が2か所に分かれていたため)。docs/voice_levels.md。 -->
    <div class="grp" id="grpVoiceState">
      <div class="grp-head"><h2 class="grp-title">音声の状態と音量</h2><p class="grp-note">先に全体ON/OFF、次に音量とBGM。OFFの間は発話もBGMも止まります。</p></div>
      <div class="grp-grid">
        <section class="card" id="voiceMasterBlock" data-cap="voice.master" style="display:none;">
      <div class="card-head"><h2>音声 全体ON/OFF</h2><span id="voiceMasterBadge" class="sbadge sb-mute">-</span></div>
      <p class="card-desc">OFFにすると、発話・AIによる文面生成・BGMがすべて止まります(ヘッダーにも「音声OFF」が出ます)。</p>
      <div id="voiceMasterState" role="status" style="font-size:22px;font-weight:700;padding:10px 12px;border-radius:8px;margin-bottom:8px;background:#eee;">-</div>
      <p class="hint" id="voiceMasterDetail" style="margin-top:0;">-</p>
      <div class="btns"><button id="voiceMasterBtn" type="button" class="primary" style="min-height:48px;font-size:16px;">-</button></div>
      <div id="voiceMasterConfirmBar" role="alert" style="display:none;margin-top:8px;padding:8px;border:2px solid #b45309;border-radius:8px;background:#fff7e6;color:#5c2d00;">
        <span id="voiceMasterConfirmText"></span>
        <div class="btns" style="margin-top:6px;"><button id="voiceMasterYes" type="button" class="primary">実行する</button><button id="voiceMasterNo" type="button">やめる</button></div>
      </div>
      <p class="hint" id="voiceMasterHint">&nbsp;</p>
    </section>
        <style>.vl-row{display:flex;flex-wrap:wrap;align-items:center;gap:8px;margin:8px 0;}.vl-row label{flex:1 1 100%;font-size:13px;}.vl-row input[type=range]{flex:1 1 140px;min-width:0;}.vl-row input[type=number]{width:84px;}.vl-ticks{display:flex;justify-content:space-between;font-size:13px;color:#667;margin:-4px 92px 0 0;}</style>
    <section class="card" id="voiceLevelsBlock" data-cap="voice.levels" style="display:none;">
      <div class="card-head"><h2>音量・BGM</h2><span id="vlBadge" class="sbadge sb-mute">-</span></div>
      <p class="card-desc">発話の音量、BGMの音量、発話中にBGMを下げる量(ダッキング)をここでまとめて調整します。</p>
      <p class="hint" id="vlNow" style="margin-top:0;">-</p>
      <p class="hint" id="vlCaution" style="margin:0 0 6px;color:#7a4a00;">注意: 2倍を超える大きい値は、音割れやスピーカーの故障の恐れがあります。人がいる場所で急に上げないでください(0.5ずつ上げて、テスト発話で確かめてください)。</p>
      <datalist id="vlTicks"><option value="0"></option><option value="1"></option><option value="2"></option><option value="3"></option><option value="4"></option><option value="5"></option></datalist>
      <div class="vl-row"><label for="vlSpeechNum">発話音量(倍率 0〜5)</label>
        <input id="vlSpeech" type="range" min="0" max="5" step="0.05" list="vlTicks" aria-label="発話音量スライダー">
        <input id="vlSpeechNum" type="number" min="0" max="5" step="0.05" inputmode="decimal"></div>
      <div class="vl-ticks" aria-hidden="true"><span>0</span><span>1</span><span>2</span><span>3</span><span>4</span><span>5</span></div>
      <div class="vl-row"><label for="vlBgmNum">BGM通常音量(0〜5)</label>
        <input id="vlBgm" type="range" min="0" max="5" step="0.05" list="vlTicks" aria-label="BGM通常音量スライダー">
        <input id="vlBgmNum" type="number" min="0" max="5" step="0.05" inputmode="decimal"></div>
      <div class="vl-row"><label for="vlDuckNum">BGM発話中(発話中にBGMを下げる音量。0〜通常音量、最大5)</label>
        <input id="vlDuck" type="range" min="0" max="5" step="0.01" list="vlTicks" aria-label="BGM発話中音量スライダー">
        <input id="vlDuckNum" type="number" min="0" max="5" step="0.01" inputmode="decimal"></div>
      <div id="vlConfirmBar" role="alert" style="display:none;margin-top:8px;padding:8px;border:2px solid #b45309;border-radius:8px;background:#fff7e6;color:#5c2d00;">
        <span id="vlConfirmText"></span>
        <div class="btns" style="margin-top:6px;"><button id="vlYes" type="button" class="primary">実行する</button><button id="vlNo" type="button">やめる</button></div>
      </div>
      <div class="btns">
        <button id="vlApplyBtn" type="button" class="primary">適用</button>
        <button id="vlTestBtn" type="button" data-cap="voice.test_speak">テスト発話</button>
        <button id="vlResetBtn" type="button">既定に戻す</button>
      </div>
      <p class="hint" id="vlHint">&nbsp;</p>
      <!-- 【2026-10-10統合】旧「BGM」カード。BGMの音量(0〜5)は上のスライダーと同じ値なので、簡易入力は折りたたみに入れた。ON/OFFと状態はここ。 -->
      <div class="subpanel" id="bgmBlock" style="display:none;">
        <h3>BGMの再生</h3>
        <p class="hint" id="bgmStatus" style="margin-top:0;">-</p>
        <div class="btns"><button id="bgmToggleBtn" type="button" data-cap="voice.bgm_onoff">-</button></div>
        <details class="fold" style="margin-top:8px;">
          <summary>0〜5の簡易入力(上のスライダーと同じ値)</summary>
          <div class="fold-body">
            <p class="hint" style="margin-top:0;">上の「BGM通常音量」「BGM発話中」と同じ値を、0〜5の範囲で手早く直すための入力です。通常は上のスライダーを使ってください。</p>
            <div class="speed-grid" style="grid-template-columns:1fr 1fr;">
              <label>通常音量(0〜5) <input id="bgmVolume" type="number" step="0.05" min="0" max="5"></label>
              <label>発話中(0〜5) <input id="bgmDuckVolume" type="number" step="0.05" min="0" max="5"></label>
            </div>
            <div class="btns"><button id="bgmSaveBtn" type="button" class="primary">音量を保存</button></div>
          </div>
        </details>
        <p class="hint" id="bgmHint">&nbsp;</p>
      </div>
    </section>
      </div>
    </div>
    <div class="grp" id="grpVoiceLinks">
      <div class="grp-head"><h2 class="grp-title">発話・呼びかけ</h2><p class="grp-note">発話の履歴確認と、呼びかけ音声の設定(発話の間隔・AIへの指示文・応答時間の記録など)。</p></div>
      <div class="grp-grid">
        <section class="card">
          __NAV_LINKS_SPEECH__
        </section>
      </div>
    </div>
    <div class="grp" id="grpVoiceDemo">
      <div class="grp-head"><h2 class="grp-title">実験・デモ</h2><p class="grp-note">普段は使いません。</p></div>
      <div class="grp-grid">
    <section class="card s-ab">
      <div class="card-head"><h2>推奨発話のA/Bテスト・デモ</h2></div>
      <p class="card-desc">推奨発話のON/OFFを切り替えて効果を比べます。「デモ実行」はロボットが実際に動きます(周りを確認してから押してください)。</p>
      <div class="btns">
        <button id="recommendOnBtn">推奨ON</button>
        <button id="recommendOffBtn">推奨OFF</button>
      </div>
      <p class="hint" id="recommendState"><label><input type="checkbox" id="recommendDirectorValue" disabled> director側の現在値: <b id="recommendStateText">確認中…</b></label></p>
      <div class="btns">
        <button id="forceDemoBtn" class="warnbtn">デモ実行(目の前の人へ確実に接近)</button>
      </div>
      <p class="hint" id="behaviorControlHint">&nbsp;</p>
    </section>
      </div>
    </div>
  </div>

  <!-- ===== タブ4: ログ・分析 ===== -->
  <div id="tab-logs" class="tabpane" data-pane="logs">
    <div class="grp">
      <div class="grp-head"><h2 class="grp-title">ログ・分析</h2><p class="grp-note">記録の確認とグラフ・レポート。ここでは何も変更しません。</p></div>
      <div class="grp-grid">
        <section class="card">
          __NAV_LINKS_LOGS__
        </section>
      </div>
    </div>
  </div>

  <!-- ===== タブ5: 設定(頭部ディスプレイ → タブレットOCR → 動作ルール → メンテナンス) ===== -->
  <div id="tab-settings" class="tabpane" data-pane="settings">
    <nav class="set-toc" aria-label="設定の目次">
      <span class="set-toc-l">ここへ移動</span>
      <a href="#grpDisplay">頭部ディスプレイ</a>
      <a href="#grpOcr">タブレットOCR</a>
      <a href="#grpRules">動作ルール</a>
      <a href="#grpMaint">メンテナンス</a>
      <a href="#grpAccounts" id="tocAccounts" hidden>アカウントと権限</a>
    </nav>
    <div class="set-cols">
    <div class="set-col">
    <div class="grp" id="grpDisplay">
      <div class="grp-head"><h2 class="grp-title">頭部ディスプレイ(iPad)</h2><p class="grp-note">お客さん向け・アンケート・残数入力・OCRテスト(模擬POS)・顔の切り替え。iPadは固定URL <code>/display</code> を開いておくだけで、ここの切り替えに合わせて中身が変わります。</p></div>
      <div class="grp-grid">
        <section class="card" id="surveyBlock" data-cap="display.survey_mode" style="display:none;">
      <details class="cf" id="surveyFold" open>
      <summary class="card-head cf-sum"><h2>アンケート(頭部ディスプレイ)</h2><span id="surveyBadge" class="sbadge sb-mute">-</span><span class="cf-note" id="surveyNote"></span></summary>
      <p class="card-desc">頭部のiPadを「お客さん向け」「アンケート」「残数入力」「OCRテスト」「顔」に切り替えます。回答中はロボットが停止し、その間は切り替えできません。</p>
      <div class="metrics kv">
        <div class="metric"><span>いまの表示</span><b id="surveyMode">-</b></div>
        <div class="metric"><span>回答件数</span><b id="surveyCount">-</b></div>
        <div class="metric"><span>回答中</span><b id="surveyActive">-</b></div>
        <div class="metric"><span>回答の状態</span><b id="surveySession">-</b></div>
        <div class="metric"><span>最終の操作</span><b id="surveyLast">-</b></div>
        <div class="metric"><span>実験条件</span><b id="surveyCond">-</b></div>
      </div>
      <p class="hint" id="surveyErr" style="color:var(--bad);font-weight:700;" hidden></p>
      <div class="btns">
        <button id="surveyToSurveyBtn" type="button" data-mode="survey">アンケートを表示</button>
        <button id="surveyToCustomerBtn" type="button" data-mode="customer">お客さん向けに戻す</button>
        <button id="surveyToInvBtn" type="button" data-mode="inventory">残数入力を表示</button>
        <button id="surveyToOcrBtn" type="button" data-mode="ocrtest">OCRテスト(模擬POS)</button>
        <button id="surveyToFaceBtn" type="button" data-mode="face">顔を表示</button>
        <button id="surveyAutoBtn" type="button" title="普段は顔を出し、人が近づいたときだけアンケートにします">自動切替(顔⇄アンケート): -</button>
        <a id="surveyCsvLink" class="linkbtn" href="/api/survey/responses.csv" download>回答CSVをダウンロード</a>
      </div>
      <div class="loc-bar loc-bar-warn" id="surveyConfirm" hidden>
        <span id="surveyConfirmText"></span>
        <button id="surveyConfirmYes" type="button" class="primary">切り替える</button>
        <button id="surveyConfirmNo" type="button">やめる</button>
      </div>
      <p class="hint" id="surveyHint">iPadの表示は約1秒で切り替わります。回答中は切り替えできません(旧方式のラズパイ表示のときは、数秒暗くなって切り替わり、回答中でも切り替わります)。<span id="surveyIpad"></span></p>
      <!-- 【2026-10-09追加】模擬POS(OCRテスト)の数字変更。ラズパイのOCRテスト画面の数字を、ランダム化・元に戻す・指定できる。
           OCRテストモードでないときは無効。押すと確認なしで即実行(テスト用)。docs/ocr_test_admin.md -->
      <style>.ocrm-sub{margin-top:14px;}.ocrm-sub h3{margin:0 0 6px;font-size:14px;}.ocrm-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(120px,1fr));gap:6px 10px;margin:8px 0;}.ocrm-grid label{display:flex;flex-direction:column;gap:2px;font-size:13px;}.ocrm-grid input{width:100%;font-size:16px;padding:6px;box-sizing:border-box;}</style>
      <details class="ocrm-sub subpanel cf cf-sub" id="ocrMockBlock" data-cap="display.ocr_mock" open>
        <summary class="card-head cf-sum"><h3 style="margin:0;">模擬POS(OCRテスト用)</h3><span id="ocrMockBadge" class="sbadge sb-mute">-</span><span class="cf-note" id="ocrMockNote"></span></summary>
        <p class="card-desc">開発・試験用。OCRテスト画面に出る数字を変えて、読み取りを確かめます。</p>
        <p class="hint" id="ocrMockWhy" style="margin-top:0;" hidden></p>
        <div class="btns">
          <button id="ocrMockRandBtn" type="button">数字をランダム化</button>
          <button id="ocrMockResetBtn" type="button">元の値に戻す</button>
        </div>
        <div class="ocrm-grid">
          <label><span>Aランチ</span><input class="ocrm-in" type="text" inputmode="numeric" maxlength="4" value="118"></label>
          <label><span>Bランチ</span><input class="ocrm-in" type="text" inputmode="numeric" maxlength="4" value="129"></label>
          <label><span>Cランチ</span><input class="ocrm-in" type="text" inputmode="numeric" maxlength="4" value="353"></label>
          <label><span>カレー</span><input class="ocrm-in" type="text" inputmode="numeric" maxlength="4" value="166"></label>
          <label><span>麺類(昼)</span><input class="ocrm-in" type="text" inputmode="numeric" maxlength="4" value="211"></label>
          <label><span>アラカルトB</span><input class="ocrm-in" type="text" inputmode="numeric" maxlength="4" value="178"></label>
          <label><span>アラカルト100円</span><input class="ocrm-in" type="text" inputmode="numeric" maxlength="4" value="103"></label>
        </div>
        <div class="btns"><button id="ocrMockSetBtn" type="button" class="primary">この値を指定して表示</button></div>
        <p class="hint" id="ocrMockHint">&nbsp;</p>
      </details>
      </details>
    </section>
        <!-- 【2026-10-27追加】メニュー編集(追加・名前の変更・削除・初期食数)。iPadの残数入力の「メニュー編集」と同じ操作(DisplayHub.edit_menu)。docs/ipad_display.md -->
        <section class="card" id="menuEditBlock" data-cap="display.menu_edit" style="display:none;">
          <details class="cf" id="menuFold" open>
          <summary class="card-head cf-sum"><h2>メニュー編集</h2><span id="menuBadge" class="sbadge sb-mute">-</span><span class="cf-note" id="menuNote"></span></summary>
          <p class="card-desc">本日のメニューの追加・名前の変更・削除・初期食数の変更をします。iPadの「残数入力」画面の「メニュー編集」でも同じことができます。変更は、お客さん向けの表示(iPad)にすぐ反映されます。</p>
          <style>.mn-row{display:grid;grid-template-columns:minmax(0,3fr) minmax(70px,1fr) auto auto;gap:6px 8px;align-items:center;margin:8px 0;padding:8px;border:1px solid var(--line,#d9e2dd);border-radius:8px;}.mn-row.mn-add{border-style:dashed;}.mn-row input{width:100%;font-size:16px;padding:6px;box-sizing:border-box;}.mn-row .mn-info{grid-column:1 / -1;font-size:13px;color:var(--mute,#5b6b64);}.mn-row.mn-dirty{border-color:#d9a21b;background:#fffaf0;}</style>
          <div id="menuList"></div>
          <div class="loc-bar loc-bar-warn" id="menuConfirm" hidden>
            <span id="menuConfirmText"></span>
            <button id="menuConfirmYes" type="button" class="primary">削除する</button>
            <button id="menuConfirmNo" type="button">やめる</button>
          </div>
          <p class="hint" id="menuHint">&nbsp;</p>
          <p class="hint">名前を変えても、売れた数と残数は引き継ぎます。初期食数を変えると、残数 = 初期食数 − 売れた数 になります。削除したメニューは一覧から消えます(残数・売れた数の記録も、この一覧からなくなります)。メニューは最大12品、名前は40文字まで、初期食数は0〜99999です。決済横タブレットのOCRが動いている間は、残数は数秒で読み取り値に上書きされます(メニューの追加・削除・名前・初期食数は上書きされません)。</p>
          </details>
        </section>
        <section class="card">
          <div class="card-head"><h2>案内表示(観客向け)</h2></div>
          __NAV_LINKS_DISPLAY__
        </section>
      </div>
    </div>
    <div class="grp" id="grpOcr">
      <div class="grp-head"><h2 class="grp-title">タブレットOCR(読み取り)</h2><p class="grp-note">決済横のタブレットの数字を読み取る機能の状態と画質。</p></div>
      <div class="grp-grid">
        <style>.ocr-prof{font-size:26px;font-weight:800;margin:4px 0 8px;}.ocr-tbl{width:100%;border-collapse:collapse;font-size:13px;margin-top:8px;}.ocr-tbl th,.ocr-tbl td{border-bottom:1px solid var(--line,#d9e2dd);padding:4px 6px;text-align:left;}.ocr-tbl td.n{text-align:right;font-variant-numeric:tabular-nums;}</style>
    <section class="card" id="ocrBlock" style="display:none;">
      <details class="cf" id="ocrFold" open>
      <summary class="card-head cf-sum"><h2>タブレットOCR(読み取り)</h2><span id="ocrBadge" class="sbadge sb-mute">-</span><span class="cf-note" id="ocrNote"></span></summary>
      <p class="card-desc">決済横のタブレットの数字を読み取る機能。画質(デモ/本番)の切り替えとピントの調整ができます。</p>
      <div class="ocr-prof" id="ocrProfile">-</div>
      <div class="metrics kv">
        <div class="metric"><span>実効解像度</span><b id="ocrRes">-</b></div>
        <div class="metric"><span>実効fps</span><b id="ocrFps">-</b></div>
        <div class="metric"><span>撮影間隔</span><b id="ocrInterval">-</b></div>
        <div class="metric"><span>回転</span><b id="ocrRot">-</b></div>
        <div class="metric"><span>ROI</span><b id="ocrRoi">-</b></div>
        <div class="metric"><span>最終更新</span><b id="ocrAge">-</b></div>
      </div>
      <p class="hint" id="ocrErr" style="color:var(--bad);font-weight:700;" hidden></p>
      <div class="btns">
        <button id="ocrToDemoBtn" type="button" data-profile="demo">デモ(滑らか)</button>
        <button id="ocrToProdBtn" type="button" data-profile="production">本番(最高画質)</button>
        <a id="ocrLink" class="linkbtn" href="#" role="button" aria-pressed="false">映像を表示</a>
      </div>
      <div id="ocrViewWrap" hidden style="margin-top:8px;">
        <img id="ocrPreview" alt="OCRのカメラ映像(ROI枠・読み取り値つき)" style="max-width:100%;background:#0c1b16;border-radius:6px;display:block;">
        <p class="hint" id="ocrViewHint" style="margin:4px 0;">カメラ映像(約1秒ごとに更新)。見ている間は、OCRの負荷が少し上がります。</p>
        <img id="ocrProcessed" alt="認識に渡している画像" style="max-width:100%;background:#0c1b16;border-radius:6px;display:block;">
      </div>
      <div class="loc-bar loc-bar-warn" id="ocrConfirm" hidden>
        <span id="ocrConfirmText"></span>
        <button id="ocrConfirmYes" type="button" class="primary">切り替える</button>
        <button id="ocrConfirmNo" type="button">やめる</button>
      </div>
      <p class="hint" id="ocrHint">デモ=解像度を少し下げてfpsを確保 / 本番=最高画質で数秒に1回だけ撮影(fpsは低くてよい)。切替にはカメラの再設定で数秒かかります。</p>
      <h3 style="margin:12px 0 4px;">ピントと画質の自動調整</h3>
      <p class="hint" id="ocrQuality" style="margin-top:0;">ピント: -</p>
      <div class="btns">
        <button id="ocrTuneBtn" type="button">画質を自動調整</button>
        <button id="ocrTuneRevertBtn" type="button">調整を元に戻す</button>
      </div>
      <details class="cf cf-sub" id="ocrRowsFold" open>
      <summary class="cf-sum"><h3 style="margin:0;">読み取り結果</h3></summary>
      <table class="ocr-tbl"><thead><tr><th>メニュー</th><th>確定値</th><th>生値</th><th>送信</th></tr></thead><tbody id="ocrRows"><tr><td colspan="4">-</td></tr></tbody></table>
      </details>
      </details>
    </section>
      </div>
    </div>
    </div>
    <div class="set-col">
    <div class="grp" id="grpInteract">
      <div class="grp-head"><h2 class="grp-title">人の方へ動く(接近)</h2><p class="grp-note">ONの間だけ、人を見つけると近づきます。OFFのときは、人がいてもその場で待ちます(発話は続きます)。「人の方を向く」「見回す」は、下の別のスイッチです。</p></div>
      <div class="grp-grid">
        <section class="card">
          <div class="card-head"><h2>人の方へ動く(近づく)</h2><span id="interactBadge" class="sbadge sb-mute">-</span></div>
          <p class="card-desc">時間割・ビヘイビアツリー・ルールで「接近を許可」にした間は、OFFでも動きます(「接近しない」の間は、ONでも動きません)。ロボットを再起動すると、OFFに戻ります。</p>
          <div class="btns">
            <button id="interactOnBtn" type="button">ON(動く)</button>
            <button id="interactOffBtn" type="button">OFF(待つ)</button>
          </div>
          <p class="hint" id="interactHint">-</p>
        </section>
        <section class="card" data-cap="behavior.face_switch">
          <div class="card-head"><h2>人の方を向く</h2><span id="faceBadge" class="sbadge sb-mute">-</span></div>
          <p class="card-desc">人を見つけると、その場で旋回して、人の多い方を向きます(前には進みません)。時間割・ビヘイビアツリー・ルールで「向くを許可」にした間は、OFFでも動きます。ロボットを再起動すると、OFFに戻ります。</p>
          <div class="btns">
            <button id="faceOnBtn" type="button">ON(向く)</button>
            <button id="faceOffBtn" type="button">OFF</button>
          </div>
          <p class="hint" id="faceHint">-</p>
        </section>
        <section class="card">
          <div class="card-head"><h2>見回す</h2><span id="scanBadge" class="sbadge sb-mute">-</span></div>
          <p class="card-desc">人がいないとき、旋回して人を探します。「待機中の左右振動」も、このスイッチに従います。ロボットを再起動すると、OFFに戻ります。</p>
          <div class="btns">
            <button id="scanOnBtn" type="button">ON(見回す)</button>
            <button id="scanOffBtn" type="button">OFF</button>
          </div>
          <p class="hint" id="scanHint">-</p>
        </section>
        <section class="card">
          <div class="card-head"><h2>待機中の左右振動</h2><span id="wiggleBadge" class="sbadge sb-mute">-</span></div>
          <p class="card-desc">人がまだ見つかっていない間、センサーを使わず左右へ1秒ずつ交互に回転するだけの動きに切り替えます。「見回す」がONのときだけ動きます(ビヘイビアツリー・ルールで「左右に揺れる」を指定した間は、OFFでも動きます)。ロボットを再起動すると、OFFに戻ります。</p>
          <div class="btns">
            <button id="wiggleOnBtn" type="button">ON</button>
            <button id="wiggleOffBtn" type="button">OFF</button>
          </div>
          <p class="hint" id="wiggleHint">-</p>
        </section>
      </div>
    </div>
    <div class="grp" id="grpRules">
      <div class="grp-head"><h2 class="grp-title">動作ルール</h2><p class="grp-note">いつ・どんな動きをするかの決まりを編集します(別タブで開きます)。</p></div>
      <div class="grp-grid">
        <section class="card">
          <div class="card-head"><h2>かんたん時間割 / 時間帯・状況ルール / ビヘイビアツリー</h2></div>
          __NAV_LINKS_RULES__
        </section>
      </div>
    </div>
    <div class="grp" id="grpMaint">
      <div class="grp-head"><h2 class="grp-title">メンテナンス</h2><p class="grp-note">たまにだけ使う調整。地図のリセットは「地図・走行」タブの「地図の操作」にあります(確認が出ます)。</p></div>
      <div class="grp-grid">
        <section class="card s-calib">
      <div class="card-head"><h2>混雑判定の調整</h2><span class="sbadge sb-warn">実行前に確認</span></div>
      <p class="card-desc">混雑(人数)の判定に使う「背景」を取り直します。部屋が無人のときだけ実行してください。</p>
      <div class="btns"><button id="calibrateBgBtn">混雑判定の背景をキャリブレーション</button></div>
      <p id="calibrateBgHint" class="hint">壁・柱等が近い設置場所での誤判定(無人でも「混雑」になる)対策。部屋を無人にしてから押してください。</p>
    </section>
        <div class="set-danger">
          <p class="set-danger-title">危険な操作(ロボット全体が止まります)</p>
__SHUTDOWN_CARD__
        </div>
      </div>
    </div>
__ACCOUNTS_CARD__
    </div>
    </div>
  </div>

</main>
<div id="estopDock">
  <button id="estopBtn" class="stop-btn">緊急停止(ソフト)</button>
</div>
<script>
__NAV_JS__
</script>
<script>
  // 401(未ログイン/セッション切れ)を受け取ったら自動でログイン画面へ誘導する。
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") {
        location.href = "/login";
      }
      return res;
    };
  })();

  const state = { estopEngaged: false, forward: 0, turn: 0 };

  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    }[ch]));
  }

  function showError(message) {
    let banner = document.getElementById("errorBanner");
    if (!banner) {
      banner = document.createElement("div");
      banner.id = "errorBanner";
      banner.style.cssText = "background:#faece7;color:#993c1d;padding:6px 10px;font-size:13px;text-align:center;";
      document.body.insertBefore(banner, document.body.firstChild);
    }
    banner.textContent = message;
    banner.style.display = message ? "block" : "none";
  }

  // 【2026-10-02追加】結果のメッセージを返すPOST。通信エラー・非JSON応答・HTTPエラーでも
  // 例外を投げず、必ず{ok, message}を返す(「〜しています...」の表示のまま止まるのを防ぐ)。
  async function postForMessage(path, body) {
    try {
      const res = await fetch(path, { method: "POST", headers: {"Content-Type":"application/json"}, body: body === undefined ? "{}" : body });
      let d = null;
      try { d = await res.json(); } catch (e2) { /* 本文がJSONでない */ }
      if (!res.ok) return { ok: false, message: "失敗: " + ((d && d.message) || `操作エラー(${res.status})`) };
      return { ok: !(d && d.ok === false), message: (d && d.message) || "" };
    } catch (e) {
      return { ok: false, message: "失敗: 通信エラー(サーバーに接続できません)" };
    }
  }

  // 【2026-10-02追加】読み取り専用(guest)アカウントでは運転コマンドを送らない
  // (送り続けても403が返るだけで、エラーバナーが出続けてしまうため)。
  let readOnlyAccount = false;
  let driveTimer = null;

  async function postJson(path, payload, opts) {
    try {
      const res = await fetch(path, Object.assign({ method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify(payload || {}) }, opts || {}));
      if (!res.ok) {
        let msg = `操作エラー(${res.status})`;
        try { const data = await res.json(); if (data && data.message) msg = data.message; } catch (e2) { /* 本文がJSONでなくても既定メッセージのまま続行 */ }
        showError(msg);
        return false;
      }
      showError("");
      return true;
    } catch (e) {
      showError("通信エラー: サーバーに接続できません");
      return false;
    }
  }

  document.getElementById("mobileUrl").textContent = `http://${location.hostname}:8095`;

  // PC側は画面上にD-pad/WASDボタンは出さない(スマホ専用ページに移動済み)が、
  // 物理キーボードのW/A/S/Dキーには反応する(ボタンが無いだけでキー入力は使える)。
  // 【2026-11-06】operator(弱いアカウント)もWASDで運転できる(robot.manual_drive はロール既定)。ここで権限による送信停止はしない
  // (送信を止めるのはguest=readOnlyAccountだけ。サーバー側で /api/teleop の権限・緊急停止・優先度を必ず判定する)。
  const keyMap = { "w":[1,0], "s":[-1,0], "a":[0,1], "d":[0,-1] };
  const pressed = new Set();
  function recomputeFromKeys() {
    let f = 0, t = 0;
    for (const k of pressed) {
      const [kf, kt] = keyMap[k] || [0,0];
      f += kf; t += kt;
    }
    state.forward = Math.max(-1, Math.min(1, f));
    state.turn = Math.max(-1, Math.min(1, t));
  }
  const teleopTabId = (() => {
    try { const a = new Uint8Array(8); crypto.getRandomValues(a); return Array.from(a, (b) => b.toString(16).padStart(2, "0")).join(""); }
    catch (e) { return (Math.random().toString(36).slice(2, 14) || "t"); }
  })();
  let teleopSeq = 0;
  let driveInFlightSince = 0;
  let driveInFlightSeq = 0;
  // 【2026-10-03修正】seq(単調増加)+tab_id(ページ固有)を付け、サーバー側で古い前進が
  // 最後の停止を追い越すのを防ぐ。非停止の前回リクエストが未完了の間は周期送信を
  // 見送る(停止は常に送る。1秒で見送り解除=通信ハング時に操作不能のままにしない)。
  function sendDriveKeyboard() {
    if (readOnlyAccount) return;
    const stop = state.forward === 0 && state.turn === 0;
    if (!stop && driveInFlightSince && (Date.now() - driveInFlightSince) < 1000) return;
    const seq = ++teleopSeq;
    const p = postJson("/api/teleop", { forward: state.forward, turn: state.turn, seq, tab_id: teleopTabId }, { keepalive: true });
    if (stop) { driveInFlightSince = 0; return; }
    driveInFlightSince = Date.now(); driveInFlightSeq = seq;
    p.finally(() => { if (driveInFlightSeq === seq) driveInFlightSince = 0; });
  }
  // 【2026-10-02追加】入力欄(INPUT/TEXTAREA/SELECT)での文字入力(例: 「west」)で
  // 走行しないよう、これらにフォーカスがある間・修飾キー併用・キーリピートは無視する。
  function isTypingTarget(e) {
    const t = e.target;
    if (!t) return false;
    const tag = t.tagName || "";
    return tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT" || !!t.isContentEditable;
  }
  // 押しっぱなし状態を全て破棄して停止を送る。Alt+Tab・別ウィンドウのクリック・
  // confirm()ダイアログ等でkeyupを受け取れないと、pressedが残ったまま走り続けて
  // しまうため、フォーカス喪失・非表示・ダイアログ表示の前に必ず呼ぶ。
  function releaseAllKeys() {
    pressed.clear();
    recomputeFromKeys();
    sendDriveKeyboard();
  }
  window.addEventListener("keydown", (e) => {
    if (e.repeat || e.ctrlKey || e.metaKey || e.altKey) return;
    if (isTypingTarget(e)) return;
    const k = e.key.toLowerCase();
    if (!(k in keyMap)) return;
    pressed.add(k);
    recomputeFromKeys();
    sendDriveKeyboard(); // キー入力直後に即送信(最大100ms周期を待たずに反映する)
  });
  window.addEventListener("keyup", (e) => {
    const k = e.key.toLowerCase();
    if (!(k in keyMap)) return;
    pressed.delete(k);
    recomputeFromKeys();
    sendDriveKeyboard(); // キーを離した瞬間も即送信(離してから最大100ms止まらない遅延を無くす)
  });
  window.addEventListener("blur", releaseAllKeys);
  window.addEventListener("pagehide", releaseAllKeys);
  document.addEventListener("visibilitychange", () => { if (document.hidden) releaseAllKeys(); });
  // confirm()はkeyupを飲み込むことがあるため、表示前に必ず停止する。
  const rawConfirm = window.confirm.bind(window);
  window.confirm = (msg) => { releaseAllKeys(); return rawConfirm(msg); };
  // alert()もkeyupを飲み込むため、表示前に必ず停止する。
  const rawAlert = window.alert.bind(window);
  window.alert = (m) => { releaseAllKeys(); rawAlert(m); };
  driveTimer = setInterval(sendDriveKeyboard, 100);

  const estopBtn = document.getElementById("estopBtn");
  // 【2026-10-02修正】ボタン表示はサーバー側の実際の状態(/api/stateのsoft_estop_engaged)に
  // 同期させる。以前は押した瞬間に表示だけ反転しており、POSTが409(in_use)/403/通信エラーで
  // 失敗しても「停止中」と表示され、再読み込み・他端末での操作とも食い違っていた。
  let estopPending = false;
  let estopIgnoreSyncUntil = 0;
  function renderEstop(engaged) {
    state.estopEngaged = engaged;
    estopBtn.textContent = engaged ? "緊急停止を解除" : "緊急停止(ソフト)";
    estopBtn.classList.toggle("engaged", engaged);
  }
  function syncEstop(s) {
    if (estopPending || Date.now() < estopIgnoreSyncUntil) return;
    if (typeof s.soft_estop_engaged === "boolean") renderEstop(s.soft_estop_engaged);
  }
  function showEstopWarning(message) {
    let banner = document.getElementById("estopWarning");
    if (!banner) {
      banner = document.createElement("div");
      banner.id = "estopWarning";
      banner.style.cssText = "background:#a32d2d;color:#fff;padding:8px 10px;font-size:13px;font-weight:700;text-align:center;";
      document.body.insertBefore(banner, document.body.firstChild);
    }
    banner.textContent = message;
    banner.style.display = message ? "block" : "none";
  }
  async function setEstop(engaged) {
    if (readOnlyAccount) return;
    if (engaged) releaseAllKeys(); // 停止側へ倒す(alert/通信失敗でkeyupを失っても走り続けないように)
    if (estopPending) return; // 二重送信防止
    const prev = state.estopEngaged;
    estopPending = true;
    estopBtn.disabled = true;
    renderEstop(engaged); // 押した直後は即時に表示を変える(失敗したら下で戻す)
    let ok = false;
    try {
      ok = await postJson("/api/soft_estop", { engaged });
    } finally {
      estopPending = false;
      estopBtn.disabled = false;
      estopIgnoreSyncUntil = Date.now() + 1500; // 送信前に始まった古い/api/state応答で表示が戻るのを避ける
    }
    if (ok) { showEstopWarning(""); return; }
    renderEstop(prev);
    if (engaged) {
      showEstopWarning("緊急停止の送信に失敗しました。機体は止まっていない可能性があります。物理の非常停止を使ってください。");
      alert("緊急停止の送信に失敗しました。機体は止まっていない可能性があります。物理の非常停止を使ってください。");
    } else {
      showEstopWarning("緊急停止の解除に失敗しました(緊急停止中のままです)。");
    }
  }
  estopBtn.addEventListener("click", async () => {
    if (state.estopEngaged) {
      if (!confirm("本当に解除しますか？")) return;
      await setEstop(false);
    } else {
      await setEstop(true);
    }
  });
  // スペースキーでどこからでも緊急停止できるようにする(入力欄にフォーカスが
  // あるときは通常のスペース入力を優先し、発動させない)。解除は誤操作防止の
  // ため必ずボタン+確認ダイアログ経由のみとし、スペースキーでは解除しない。
  window.addEventListener("keydown", (e) => {
    if (e.code !== "Space") return;
    const tag = (e.target && e.target.tagName) || "";
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    if (state.estopEngaged) return;
    e.preventDefault();
    setEstop(true);
  });
  document.getElementById("resumeBtn").addEventListener("click", () => postJson("/api/resume_autonomy", {}));

  // --- 【2026-10-22】壁から離れる(壁際で動けないとき)。状態は /api/wall_escape/status を定期取得 ---
  const weBtn = document.getElementById("wallEscapeBtn");
  const weNote = document.getElementById("wallEscapeNote");
  const weStuck = document.getElementById("wallStuckNote");
  const WE_STUCK_MSG = "壁が近く動けません。『壁から離れる』を押してください";
  let weLast = null;      // 最後に取得した状態(取得できなければnull=不明扱い)
  let weMsg = "", weMsgUntil = 0;   // 操作エラーの表示
  let weAskedAt = 0, weBusy = false;
  function renderWallEscape() {
    if (!weBtn || !weNote) return;
    const s = weLast || { state: "unknown", reason: "壁の状態を取得できません", stuck: false, min_range_m: null };
    const unknown = s.state === "unknown", escaping = s.state === "escaping";
    const stuck = !!s.stuck && !unknown;
    const dis = unknown || escaping || weBusy || readOnlyAccount;
    if (weBtn.dataset.weDis !== String(dis)) { weBtn.dataset.weDis = String(dis); weBtn.disabled = dis; }
    const label = escaping ? "離脱中…" : "壁から離れる";
    if (weBtn.textContent !== label) weBtn.textContent = label;
    weBtn.classList.toggle("we-urgent", stuck && !escaping);
    if (weStuck) { const t = (stuck && !escaping) ? WE_STUCK_MSG : ""; if (weStuck.textContent !== t) weStuck.textContent = t; }
    const asked = Date.now() - weAskedAt;
    let text;
    if (weMsg && Date.now() < weMsgUntil) text = weMsg;
    else if (escaping) text = "離脱中…(壁から離れています。最長6秒)";
    else if (unknown) text = s.reason || "壁の状態が届いていません";
    else if (asked < 6000 && s.reason) text = "開始できません: " + s.reason;
    else if (asked < 2000) text = "依頼を送りました…";
    else if (s.state === "failed") text = "離れられませんでした: " + (s.reason || "");
    else if (s.state === "blocked") text = "停止中・手動運転中のため使えません";
    else if (stuck) text = WE_STUCK_MSG;
    else text = s.min_range_m == null ? "壁までの距離: -" : "壁までの距離: " + Number(s.min_range_m).toFixed(2) + " m";
    if (weNote.textContent !== text) weNote.textContent = text;
  }
  async function fetchWallEscape() {
    try {
      const res = await fetch("/api/wall_escape/status");
      weLast = res.ok ? await res.json() : null;
    } catch (e) {
      weLast = null;
    }
    renderWallEscape();
  }
  if (weBtn) weBtn.addEventListener("click", async () => {
    if (weBusy) return;
    weBusy = true; weMsg = ""; renderWallEscape();
    try {
      const res = await fetch("/api/wall_escape/trigger", { method: "POST", headers: {"Content-Type":"application/json"}, body: "{}" });
      let data = {};
      try { data = await res.json(); } catch (e2) { /* 本文がJSONでなくても既定メッセージのまま続行 */ }
      if (res.ok) { weAskedAt = Date.now(); }
      else { weMsg = (data && data.message) || `操作エラー(${res.status})`; weMsgUntil = Date.now() + 8000; }
    } catch (e) {
      weMsg = "通信エラー: サーバーに接続できません"; weMsgUntil = Date.now() + 8000;
    }
    weBusy = false;
    await fetchWallEscape();
    setTimeout(fetchWallEscape, 1300);   // ノード側の「断った理由」は1Hzの状態に載る
  });
  NKField.poll(fetchWallEscape, 1500);
  fetchWallEscape();

  // --- 【2026-10-24】手動運転の人避け(既定OFF)。状態は GET /api/manual_person_guard を定期取得、切替は POST {"enabled": bool} ---
  const mpgToggle = document.getElementById("mpgToggle");
  const mpgBadge = document.getElementById("mpgBadge");
  const mpgBlockedEl = document.getElementById("mpgBlocked");
  const mpgWarnEl = document.getElementById("mpgWarn");
  const mpgNote = document.getElementById("mpgNote");
  let mpgLast = null, mpgBusy = false, mpgMsg = "", mpgMsgUntil = 0;
  function mpgShow(el, on) { const v = on ? "" : "none"; if (el && el.style.display !== v) el.style.display = v; }
  function renderMpg() {
    if (!mpgToggle) return;
    const s = mpgLast;
    if (s && !mpgBusy && mpgToggle.checked !== !!s.enabled) mpgToggle.checked = !!s.enabled;
    const canChange = !!(s && s.can_change) && !readOnlyAccount;
    const lock = !canChange || mpgBusy;
    if (lock) { if (!mpgToggle.disabled) mpgToggle.disabled = true; }
    else if (mpgToggle.disabled && mpgToggle.dataset.nkLocked !== "1") mpgToggle.disabled = false;
    const on = !!(s && s.enabled);
    if (mpgBadge) {
      const t = s ? (on ? "ON" : "OFF") : "-";
      if (mpgBadge.textContent !== t) mpgBadge.textContent = t;
      const c = "sbadge " + (s ? (on ? "sb-ok" : "sb-mute") : "sb-mute");
      if (mpgBadge.className !== c) mpgBadge.className = c;
    }
    mpgShow(mpgBlockedEl, !!(s && s.enabled && s.blocked));
    mpgShow(mpgWarnEl, !!(s && s.enabled && !s.detector_alive));
    let note = "";
    if (mpgMsg && Date.now() < mpgMsgUntil) note = mpgMsg;
    else if (!s) note = "人避けの状態を取得できません";
    else if (!canChange) note = "この設定を変えるには管理者の許可が必要です(現在は" + (on ? "ON" : "OFF") + "のまま)";
    if (mpgNote.textContent !== note) mpgNote.textContent = note;
    mpgShow(mpgNote, !!note);
  }
  async function fetchMpg() {
    try {
      const res = await fetch("/api/manual_person_guard");
      mpgLast = res.ok ? await res.json() : null;
    } catch (e) {
      mpgLast = null;
    }
    renderMpg();
  }
  if (mpgToggle) mpgToggle.addEventListener("change", async () => {
    if (mpgBusy) return;
    const want = mpgToggle.checked;
    mpgBusy = true; mpgMsg = ""; renderMpg();
    try {
      const res = await fetch("/api/manual_person_guard", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify({ enabled: want }) });
      let data = {};
      try { data = await res.json(); } catch (e2) { /* 本文がJSONでなくても既定メッセージのまま続行 */ }
      if (!res.ok) { mpgMsg = (data && data.message) || `操作エラー(${res.status})`; mpgMsgUntil = Date.now() + 8000; }
    } catch (e) {
      mpgMsg = "通信エラー: サーバーに接続できません"; mpgMsgUntil = Date.now() + 8000;
    }
    mpgBusy = false;
    await fetchMpg();   // 失敗したときは、サーバー側の実際の状態にトグルを戻す
  });
  NKField.poll(fetchMpg, 1500);
  fetchMpg();

  // --- 地図モードの開始/停止/リセット/読み込み ---
  const mapModeLabel = document.getElementById("mapModeLabel");
  const mapLoadSelect = document.getElementById("mapLoadSelect");
  const mapSaveHint = document.getElementById("mapSaveHint");
  document.getElementById("mapStartBtn").addEventListener("click", async () => {
    mapSaveHint.textContent = "地図作成を開始しています...";
    const d = await postForMessage("/api/mapping/start", "{}");
    mapSaveHint.textContent = d.message;
  });
  document.getElementById("mapStopBtn").addEventListener("click", async () => {
    mapSaveHint.textContent = "停止しています...";
    const d = await postForMessage("/api/mapping/stop", "{}");
    mapSaveHint.textContent = d.message;
  });
  document.getElementById("mapResetBtn").addEventListener("click", async () => {
    if (!confirm("地図をリセットします(保存していない作成中の地図は失われます)。本当によろしいですか？")) return;
    mapSaveHint.textContent = "リセットしています...";
    const d = await postForMessage("/api/mapping/reset", "{}");
    mapSaveHint.textContent = d.message;
  });
  document.getElementById("mapLoadBtn").addEventListener("click", async () => {
    const name = mapLoadSelect.value;
    if (!name) { mapSaveHint.textContent = "読み込む地図を選択してください"; return; }
    mapSaveHint.textContent = "読み込んでいます...";
    const d = await postForMessage("/api/mapping/load", JSON.stringify({ name }));
    mapSaveHint.textContent = d.message;
  });
  async function refreshMapList() {
    try {
      const res = await fetch("/api/mapping/list");
      if (!res.ok) return;
      const d = await res.json();
      NKField.setOptions(mapLoadSelect, d.maps || []);  // 中身が同じ/操作中なら作り直さない
    } catch (e) { /* 無視 */ }
  }
  async function pollMapLifecycleStatus() {
    try {
      const res = await fetch("/api/mapping/lifecycle_status");
      if (!res.ok) return;
      const s = await res.json();
      const modeLabel = { idle: "停止中", mapping: "作成中(SLAM)", localization: "読み込み済み(localization)" }[s.mode] || s.mode;
      let label = s.loaded_map ? `${modeLabel}: ${s.loaded_map}` : modeLabel;
      // 【2026-10-02追加】プロセス稼働状態と、異常終了などのlast_errorも表示する。
      if (s.mode !== "idle") label += s.running ? "(プロセス稼働中)" : "(プロセス停止)";
      if (s.last_error) label += ` / エラー: ${s.last_error}`;
      mapModeLabel.textContent = label;
      mapModeLabel.style.color = s.last_error ? "#993c1d" : "";
    } catch (e) { /* 無視 */ }
  }
  NKField.guard(["mapSaveName", "mapLoadSelect"]);
  NKField.poll(refreshMapList, 5000);
  setInterval(pollMapLifecycleStatus, 2000);
  refreshMapList();
  pollMapLifecycleStatus();

  // --- 地図の保存(SLAMで作成中の地図をボタン1つで書き出す) ---
  const mapSaveBtn = document.getElementById("mapSaveBtn");
  mapSaveBtn.addEventListener("click", async () => {
    const name = document.getElementById("mapSaveName").value.trim();
    mapSaveBtn.disabled = true;
    mapSaveHint.textContent = "保存中...";
    try {
      const res = await fetch("/api/mapping/save", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name }),
      });
      const data = await res.json();
      mapSaveHint.textContent = data.message || (data.ok ? "保存を開始しました" : "保存に失敗しました");
    } catch (e) {
      mapSaveHint.textContent = "通信エラー: サーバーに接続できません";
    } finally {
      mapSaveBtn.disabled = false;
    }
  });
  let lastMapSaveTime = null;
  async function pollMappingStatus() {
    try {
      const res = await fetch("/api/mapping/status");
      if (!res.ok) return;
      const s = await res.json();
      // 【2026-10-02修正】last_result.timeが変わった時だけ表示を更新する
      // (3秒毎に上書きすると、他の操作メッセージが消えてしまうため)。
      if (s.last_result && s.last_result.time && s.last_result.time !== lastMapSaveTime) {
        lastMapSaveTime = s.last_result.time;
        const r = s.last_result;
        mapSaveHint.textContent = r.ok
          ? `保存済み: ${r.name}(${r.path}.yaml / .pgm)`
          : `保存失敗: ${r.message}`;
      }
    } catch (e) { /* 無視 */ }
  }
  setInterval(pollMappingStatus, 3000);

  __MAP_VIEW_JS__

  // --- 走行可能範囲(矩形)の設定: 地図を2回クリックして対角の2点を指定する ---
  // 状態: {enabled, center_x, center_y, width, height, yaw[rad], yaw_deg, min_x, max_x, min_y, max_y}
  // (min/maxは回転矩形の軸平行外接矩形。描画・判定は回転矩形で行う)
  let currentBoundary = null;
  let boundaryPickMode = false;
  let boundaryFirstPoint = null;
  const boundaryHint = document.getElementById("boundaryHint");

  function drawBoundary() {
    drawBoundaryShape(currentBoundary);  // 回転矩形+四隅のハンドル(map_view.py)
  }

  async function fetchBoundary() {
    const tk = NKField.ticket("boundary");
    try {
      const res = await fetch("/api/boundary");
      if (!res.ok) return;
      const b = await res.json();
      if (!NKField.fresh(tk) || mapBoundaryDrag) return;  // 送信前に始まった古い結果・角のドラッグ中は反映しない
      currentBoundary = b;
      renderMap();
    } catch (e) { /* 無視 */ }
  }

  document.getElementById("boundarySetBtn").addEventListener("click", () => {
    if (poseMode) setPoseMode(false);  // 現在位置の指定中は終了してから範囲指定へ
    if (pinMode) setPinMode(false);
    if (goalHeadMode) setGoalHeadMode(false);
    boundaryPickMode = true;
    boundaryFirstPoint = null;
    boundaryHint.textContent = "地図上で範囲の1つ目の角をクリックしてください(表示の向きに沿った矩形になります)";
  });
  document.getElementById("boundaryClearBtn").addEventListener("click", async () => {
    boundaryPickMode = false;
    boundaryFirstPoint = null;
    NKField.bump("boundary");
    const d = await postForMessage("/api/boundary/clear", "{}");
    boundaryHint.textContent = d.message;
    NKField.bump("boundary");
    await fetchBoundary();
    if (d.ok && currentBoundary && currentBoundary.enabled) boundaryHint.textContent = "範囲の解除が反映されていません(サーバーでは範囲が有効のままです)。もう一度押してください";
  });

  canvas.addEventListener("click", (ev) => {
    if (!mapMeta || mapCssW < 2 || mapCssH < 2) return;
    // ドラッグ/ピンチ操作の直後のclickは目的地指定として扱わない
    if (Date.now() < mapClickSuppressUntil) return;
    const [px, py] = mapEventToPixel(ev);
    const [x, y] = pixelToWorld(px, py);

    if (boundaryPickMode) {
      if (!boundaryFirstPoint) {
        boundaryFirstPoint = [x, y];
        boundaryHint.textContent = "2つ目の角(対角)をクリックしてください";
      } else {
        // 表示の向きで水平・垂直に見える矩形(世界座標では表示回転ぶん回った矩形)
        const rect = boundaryRectFromViewCorners(boundaryFirstPoint, [x, y], mapView.rot || 0);
        boundaryPickMode = false;
        boundaryFirstPoint = null;
        if (rect.width < 0.2 || rect.height < 0.2) {
          boundaryHint.textContent = "範囲が小さすぎます(対角の2点を離してクリックしてください)";
          return;
        }
        (async () => {
          NKField.bump("boundary");
          const d = await postForMessage("/api/boundary/set", JSON.stringify(rect));
          boundaryHint.textContent = d.message;
          NKField.bump("boundary");
          if (d.ok) await boundaryVerify(rect, boundaryHint); else fetchBoundary();
        })();
      }
      return;
    }
    // 位置の確かさが「良好」でない(localization中)時は、確認バーを挟む(map_view.pyのrequestGoal)
    requestGoal(x, y);
  });
  setInterval(fetchMap, 2000);
  NKField.poll(fetchBoundary, 5000);
  fetchMap();
  fetchBoundary();

  // --- 自律探索マッピング(frontier_explorer)の開始/停止 ---
  const explorationHint = document.getElementById("explorationHint");
  document.getElementById("explorationStartBtn").addEventListener("click", async () => {
    const d = await postForMessage("/api/exploration/start", "{}");
    explorationHint.textContent = d.message;
  });
  document.getElementById("explorationStopBtn").addEventListener("click", async () => {
    const d = await postForMessage("/api/exploration/stop", "{}");
    explorationHint.textContent = d.message;
  });

  // --- A/Bテスト(推奨発話ON/OFF)・デモ用強制トリガー ---
  const behaviorControlHint = document.getElementById("behaviorControlHint");
  // 【2026-10-13追加】director側の現在値を読み戻して表示し、切替後は実際の値が一致するまで確認する
  // (以前は「送りました」と出すだけで、directorに届いたか・再起動で既定に戻っていないかが分からなかった)。
  // 値は/api/stateのrecommend_enabled(behavior_commandに載る)。指定後は5秒ごとに再送される。
  const recEls = { box: document.getElementById("recommendDirectorValue"), text: document.getElementById("recommendStateText") };
  NKField.guard(recEls.box);
  function recFresh(s) { return typeof s.recommend_enabled === "boolean" && typeof s.behavior_age_s === "number" && s.behavior_age_s < 5; }
  function updateRecommendState(s) {
    if (!recEls.box) return;
    if (!recFresh(s)) {
      recEls.text.textContent = "未確認(directorから現在値を受信できていません)";
      recEls.text.style.color = "#b3261e";
      return;
    }
    NKField.set(recEls.box, s.recommend_enabled);
    const req = typeof s.recommend_requested === "boolean" ? s.recommend_requested : null;
    const v = s.recommend_enabled ? "ON" : "OFF";
    if (req === null) { recEls.text.textContent = v + "(起動時の既定。画面からは未指定)"; recEls.text.style.color = ""; }
    else if (req === s.recommend_enabled) { recEls.text.textContent = v + "(確認済み)"; recEls.text.style.color = "#0f6e56"; }
    else { recEls.text.textContent = v + "(未確認: 指定は" + (req ? "ON" : "OFF") + "ですが、directorに届いていません。5秒ごとに再送中)"; recEls.text.style.color = "#b3261e"; }
  }
  async function recSend(enabled) {
    const tx = NKField.begin("recommend", [recEls.box]);
    const d = await postForMessage("/api/recommend_enabled", JSON.stringify({ enabled: enabled }));
    if (!d.ok) { tx.fail(); behaviorControlHint.textContent = d.message; return; }
    behaviorControlHint.textContent = d.message + "(directorでの反映を確認しています…)";
    const r = await tx.verify({
      items: [{ el: recEls.box, sent: enabled, label: "推奨発話", read: (s) => (recFresh(s) ? s.recommend_enabled : undefined) }],
      load: async () => { try { return await (await fetch("/api/state")).json(); } catch (e) { return null; } },
      delays: [300, 700, 1500, 3000, 4000],
    });
    const name = enabled ? "ON" : "OFF";
    if (r.state === "ok") behaviorControlHint.textContent = d.message + "(directorが" + name + "になったことを確認しました)";
    else if (r.state === "mismatch") behaviorControlHint.textContent = "directorに届いていません(director側の現在値は " + (enabled ? "OFF" : "ON") + " のままです)。directorが止まっているか再起動中の可能性があります。5秒ごとに再送しています";
    else behaviorControlHint.textContent = "届いたか確認できません(directorから現在値を受信できません。directorは起動していますか?古い版の可能性もあります)";
  }
  document.getElementById("recommendOnBtn").addEventListener("click", () => recSend(true));
  document.getElementById("recommendOffBtn").addEventListener("click", () => recSend(false));
  document.getElementById("forceDemoBtn").addEventListener("click", async () => {
    const d = await postForMessage("/api/force_demo", "{}");
    behaviorControlHint.textContent = d.message;
  });

  async function pollExplorationStatus() {
    try {
      const res = await fetch("/api/exploration/status");
      if (!res.ok) return;
      const s = await res.json();
      if (!s.status) return;
      const label = { idle: "未実行", exploring: "探索中", paused: "一時停止中", complete: "探索完了" }[s.status] || s.status;
      explorationHint.textContent = `状態: ${label}` + (s.reason ? `(${s.reason})` : "");
    } catch (e) { /* 無視 */ }
  }
  setInterval(pollExplorationStatus, 2000);
  pollExplorationStatus();

  // --- LiDARの見え方(自機中心のレーダー風表示) ---
  const lidarCanvas = document.getElementById("lidarCanvas");
  const lidarCtx = lidarCanvas.getContext("2d");

  function drawScan(s) {
    lidarCtx.clearRect(0, 0, lidarCanvas.width, lidarCanvas.height);
    const cx = lidarCanvas.width / 2, cy = lidarCanvas.height / 2;
    const maxRange = Math.min(s.range_max, 6.0);
    const scale = Math.min(cx, cy) / maxRange;

    lidarCtx.strokeStyle = "#234339";
    lidarCtx.lineWidth = 1;
    for (let r = 1; r <= maxRange; r += 1) {
      lidarCtx.beginPath();
      lidarCtx.arc(cx, cy, r * scale, 0, Math.PI * 2);
      lidarCtx.stroke();
    }

    lidarCtx.fillStyle = "#ff5a5a";
    let angle = s.angle_min;
    for (const r of s.ranges) {
      if (r !== null && r <= maxRange) {
        const px = cx - r * Math.sin(angle) * scale;
        const py = cy - r * Math.cos(angle) * scale;
        lidarCtx.fillRect(px - 1.5, py - 1.5, 3, 3);
      }
      angle += s.angle_increment;
    }

    lidarCtx.fillStyle = "#3fd68c";
    lidarCtx.beginPath();
    lidarCtx.moveTo(cx, cy - 9);
    lidarCtx.lineTo(cx - 6, cy + 6);
    lidarCtx.lineTo(cx + 6, cy + 6);
    lidarCtx.closePath();
    lidarCtx.fill();
  }

  async function fetchScan() {
    try {
      const res = await fetch("/api/scan");
      if (!res.ok) return;
      drawScan(await res.json());
    } catch (e) { /* 無視、次のポーリングで再試行 */ }
  }
  setInterval(fetchScan, 500);

  // --- 混雑判定の背景キャリブレーション(壁・柱等の静的な近さを誤検出しないように) ---
  document.getElementById("calibrateBgBtn").addEventListener("click", async () => {
    if (!confirm("周囲に人がいない状態にしてから実行してください。今キャリブレーションしますか？")) return;
    const btn = document.getElementById("calibrateBgBtn");
    const hint = document.getElementById("calibrateBgHint");
    // 【2026-10-13変更】要求にrequest_idが付き、crowd_monitorが返す「受理(started)→完了(done)/失敗(failed)」を
    // 読み戻して表示する。以前はcrowd_monitorが止まっていても成功と表示していた。
    const tx = NKField.begin("crowdcal", [btn]);
    hint.textContent = "要求を送信中…";
    let d = null;
    try {
      const res = await fetch("/api/crowd_background/calibrate", { method: "POST", headers: {"Content-Type":"application/json"}, body: "{}" });
      d = await res.json().catch(() => null);
      if (!res.ok) d = { ok: false, message: "失敗: " + ((d && d.message) || `操作エラー(${res.status})`) };
    } catch (e) { d = { ok: false, message: "失敗: 通信エラー(サーバーに接続できません)" }; }
    if (!d || !d.ok || !d.request_id) { tx.fail(); hint.textContent = (d && d.message) || "失敗: 要求を送れませんでした"; return; }
    hint.textContent = "要求を送りました。crowd_monitorの応答を待っています…";
    const rid = d.request_id;
    let started = false, fin = null;
    for (let i = 0; i < 20 && !fin; i++) {   // 最大約20秒(受理は数秒以内、完了は次のLiDARスキャン)
      await new Promise((r) => setTimeout(r, 1000));
      let s = null;
      try { s = await (await fetch("/api/crowd_background/state")).json(); } catch (e) { continue; }
      if (!s || s.request_id !== rid) continue;
      if (s.state === "started") { started = true; hint.textContent = "crowd_monitorが受け付けました。次のLiDARスキャンを待っています…"; }
      else if (s.state === "done" || s.state === "failed") fin = s;
    }
    tx.done();
    if (fin && fin.state === "done") hint.textContent = "混雑判定の背景を記録しました(確認済み。LiDARの有効点 " + fin.background_frames + ")";
    else if (fin) hint.textContent = "背景の記録に失敗しました: " + (fin.error || "原因不明");
    else if (started) hint.textContent = "crowd_monitorは受け付けましたが、LiDARスキャンが来ないため完了を確認できません(LiDARは動いていますか?)";
    else hint.textContent = "crowd_monitorが応答しません(起動していますか?)。届いたか確認できません";
  });

  // --- 正面カメラの見え方(静止画ポーリング。滑らかな動画ではない) ---
  const cameraView = document.getElementById("cameraView");
  function fetchCamera() {
    clearTimeout(cameraTimer);
    cameraTimer = setTimeout(fetchCamera, 10000); // 応答が来ない場合の保険
    cameraView.src = "/api/camera?t=" + Date.now();
  }
  // 【2026-10-02修正】前の画像の読み込みが終わる前にsrcを差し替えると、遅い回線では
  // いつまでも1枚も表示完了しない(飢餓)。onload/onerrorを待ってから次を予約する。
  let cameraTimer = null;
  function scheduleCamera() { clearTimeout(cameraTimer); cameraTimer = setTimeout(fetchCamera, 400); }
  cameraView.addEventListener("load", scheduleCamera);
  cameraView.addEventListener("error", scheduleCamera);
  fetchCamera();
  // 【2026-10-22】後ろ向きカメラ(左後ろ・右後ろ)。同じ方式(onload/onerrorを待ってから次を予約)。
  // 後ろ向きカメラが無い/外した構成でも画面が崩れないように、映像が5回続けて取れなければ、その枠を隠す(30秒おきに再確認し、取れたら表示)。
  function startRearCamera(id, which) {
    const el = document.getElementById(id);
    if (!el) return;
    const fig = el.closest("figure");
    let timer = null, errors = 0;
    function fetchIt() {
      clearTimeout(timer);
      timer = setTimeout(fetchIt, 10000);
      el.src = "/api/camera?which=" + which + "&t=" + Date.now();
    }
    function schedule() { clearTimeout(timer); timer = setTimeout(fetchIt, 700); }
    el.addEventListener("load", function () { errors = 0; if (fig) fig.style.display = ""; schedule(); });
    el.addEventListener("error", function () {
      errors += 1;
      if (errors >= 5) { if (fig) fig.style.display = "none"; clearTimeout(timer); timer = setTimeout(fetchIt, 30000); }
      else schedule();
    });
    fetchIt();
  }
  startRearCamera("cameraViewLeft", "left");
  startRearCamera("cameraViewRight", "right");
  // 【2026-10-22】カメラ映像をクリックで拡大/縮小(Escでも戻る)
  const camZoomEls = ["cameraView", "cameraViewLeft", "cameraViewRight"].map(function (id) { return document.getElementById(id); }).filter(Boolean);
  camZoomEls.forEach(function (el) { el.addEventListener("click", function () { el.classList.toggle("cam-zoom"); }); });
  document.addEventListener("keydown", function (e) { if (e.key === "Escape") camZoomEls.forEach(function (el) { el.classList.remove("cam-zoom"); }); });
  fetchScan();

  // 決済場所カメラ(メニュー割り当て)欄は2026-10-05に撤去(タブレットOCRに置き換え)。
  // バックエンド(CheckoutCamRegistryと決済カメラ用API)は残してあるが、
  // このページからは一切リクエストしない。
  let lastMenuItems = [];

  // 安全装置(estop・立入禁止エリア等)で自律動作が止まっている時、何が原因かを
  // ヘッダーのバッジに分かりやすく表示する(2026-07-17、ユーザー要望:
  // 「なぜ動かないか」をAPIを叩かなくても画面で分かるようにしたい)。
  const HOLD_REASONS = {
    NEAR_PERSON_HOLD: "人物接近(50cm以内)で自律停止中",
    OUT_OF_BOUNDS_HOLD: "走行可能範囲外のため自律停止中",
    KEEPOUT_ZONE_HOLD: "立入禁止エリアのため自律停止中",
    CAMERA_OBSTACLE_HOLD: "カメラが障害物を検知、自律停止中",
    MAP_NOT_LOADED_HOLD: "地図が読み込まれていません。地図を読み込んでください(自律動作は停止中)",
    // 【2026-10-02追加】これらも自律が止まっている状態なのにバッジが「正常」になっていた。
    MAPPING_ACTIVE_HOLD: "地図作成/探索中のため自律停止中",
    SURVEY_ACTIVE_HOLD: "アンケート回答中のため停止中",
    WAIT_FOR_SENSOR: "センサー待ちのため自律停止中",
  };
  // 【2026-10-06追加】時間帯・状況ルール(方針)による待機/移動。安全装置のホールドではないので
  // 赤ではなく黄色のバッジで表示する(behavior_rules、docs/behavior_rules.md)。
  const POLICY_REASONS = {
    // 【2026-11-05】「向く」機能の通常動作(安全装置のホールドではない)。赤ではなく黄色で表示する。
    FACE_PERSON: "人の方を向いています(その場で旋回)",
    FACE_SWEEP: "人を探して見回しています(その場で旋回)",
    POLICY_STAY: "方針により待機中(その場で待機)",
    POLICY_GO_ENTRANCE: "方針により入口へ移動中",
    POLICY_AT_ENTRANCE: "方針により入口で待機中",
    POLICY_GO_PIN: "方針によりピンへ移動中",
    POLICY_AT_PIN: "方針によりピンで待機中",
    POLICY_PIN_UNAVAILABLE: "方針のピンが使えないため待機中",
    POLICY_NO_SCAN: "方針により見回し停止中(待機)",
  };
  // behavior_directorからの指令が数秒来ていない(落ちている/通信断)場合は、
  // 古いmode/estop_activeを信用せず「無応答」と表示する。
  function isBehaviorStale(s) {
    return s.behavior_age_s === null || s.behavior_age_s === undefined || s.behavior_age_s > 5;
  }
  function updateStatusBadge(s) {
    const badge = document.getElementById("estopBadge");
    if (isBehaviorStale(s)) {
      badge.textContent = "behavior_director無応答";
      badge.className = "estop-badge estop-bad";
    } else if (s.estop_active) {
      badge.textContent = "緊急停止中";
      badge.className = "estop-badge estop-bad";
    } else if (HOLD_REASONS[s.mode]) {
      badge.textContent = HOLD_REASONS[s.mode];
      badge.className = "estop-badge estop-bad";
    } else if (POLICY_REASONS[s.mode]) {
      badge.textContent = POLICY_REASONS[s.mode];
      badge.className = "estop-badge estop-policy";
    } else {
      badge.textContent = "正常";
      badge.className = "estop-badge estop-ok";
    }
  }

  // 【2026-11-02追加】人の方へ動く動作(接近・見回し)のスイッチ。表示はdirectorの報告値(読み戻し)。
  function updatePersonInteraction(s) {
    const badge = document.getElementById("interactBadge");
    const hint = document.getElementById("interactHint");
    if (!badge || !hint) return;
    if (s.person_interaction_gate === false) {
      badge.textContent = "常に許可"; badge.className = "sbadge sb-warn";
      hint.textContent = "現在: 常に許可(従来動作。このロボットのプログラムは、まだ新しいスイッチに対応していません)";
    } else if (s.person_interaction_enabled) {
      badge.textContent = "ON"; badge.className = "sbadge sb-ok";
      hint.textContent = "現在: ON(人を見つけると近づきます)";
    } else {
      badge.textContent = "OFF"; badge.className = "sbadge sb-mute";
      hint.textContent = "現在: OFF(人がいてもその場で待ちます)";
    }
  }
  document.getElementById("interactOnBtn").addEventListener("click", () => postJson("/api/person_interaction", { enabled: true }));
  document.getElementById("interactOffBtn").addEventListener("click", () => postJson("/api/person_interaction", { enabled: false }));
  // 【2026-11-03追加】見回す(scan)・人の方を向く(face)のスイッチ。表示はdirectorの報告値(読み戻し)。
  // 古いdirector(scan_enabled/face_enabledを報告しない)は「未対応」と表示する。
  function updateScan(s) {
    const badge = document.getElementById("scanBadge");
    const hint = document.getElementById("scanHint");
    if (!badge || !hint) return;
    if (s.scan_face_supported === false) {
      badge.textContent = "未対応"; badge.className = "sbadge sb-warn";
      hint.textContent = "未対応(ロボット側が古い)";
    } else if (s.scan_enabled) {
      badge.textContent = "ON"; badge.className = "sbadge sb-ok";
      hint.textContent = "現在: ON(人がいないとき、旋回して探します)";
    } else {
      badge.textContent = "OFF"; badge.className = "sbadge sb-mute";
      hint.textContent = "現在: OFF(人がいなくても、その場で待ちます)";
    }
  }
  function updateFace(s) {
    const badge = document.getElementById("faceBadge");
    const hint = document.getElementById("faceHint");
    if (!badge || !hint) return;
    if (s.scan_face_supported === false) {
      badge.textContent = "未対応"; badge.className = "sbadge sb-warn";
      hint.textContent = "未対応(ロボット側が古い)";
    } else if (s.face_enabled) {
      badge.textContent = s.face_active ? (s.face_aligned ? "向いている" : "旋回中") : "ON";
      badge.className = "sbadge sb-ok";
      const b = (typeof s.face_bearing_deg === "number") ? `、方位 ${s.face_bearing_deg.toFixed(0)}度` : "";
      hint.textContent = "現在: ON" + (s.face_active ? (s.face_aligned ? "(人の方を向いています" : "(人の方へ旋回中") + b + ")" : "(人がいると、その場で向きます)");
    } else {
      badge.textContent = "OFF"; badge.className = "sbadge sb-mute";
      hint.textContent = "現在: OFF";
    }
  }
  document.getElementById("scanOnBtn").addEventListener("click", () => postJson("/api/scan_enabled", { enabled: true }));
  document.getElementById("scanOffBtn").addEventListener("click", () => postJson("/api/scan_enabled", { enabled: false }));
  document.getElementById("faceOnBtn").addEventListener("click", () => postJson("/api/face_enabled", { enabled: true }));
  document.getElementById("faceOffBtn").addEventListener("click", () => postJson("/api/face_enabled", { enabled: false }));
  // 【2026-11-02追加】待機中の左右振動(スマホ版と同じ。/api/idle_wiggle)。表示はdirectorの報告値。
  function updateWiggle(s) {
    const badge = document.getElementById("wiggleBadge");
    const hint = document.getElementById("wiggleHint");
    if (!badge || !hint) return;
    const on = !!s.idle_wiggle_enabled;
    badge.textContent = on ? "ON" : "OFF"; badge.className = "sbadge " + (on ? "sb-ok" : "sb-mute");
    hint.textContent = on ? "現在: ON(「見回す」がONのとき、左右に振動します)" : "現在: OFF";
  }
  document.getElementById("wiggleOnBtn").addEventListener("click", () => postJson("/api/idle_wiggle", { enabled: true }));
  document.getElementById("wiggleOffBtn").addEventListener("click", () => postJson("/api/idle_wiggle", { enabled: false }));

  async function poll() {
    try {
      const res = await fetch("/api/state");
      const s = await res.json();
      syncEstop(s);
      updateRecommendState(s);   // 推奨発話のdirector側の現在値(読み戻し)
      updatePersonInteraction(s);   // 人の方へ動くスイッチのdirector側の現在値(読み戻し)
      updateScan(s);                // 見回すスイッチのdirector側の現在値
      updateFace(s);                // 人の方を向くスイッチのdirector側の現在値
      updateWiggle(s);              // 待機中の左右振動のdirector側の現在値(読み戻し)
      document.getElementById("crowdLevel").textContent = s.crowd_level || "-";
      document.getElementById("mode").textContent = s.mode || "-";
      // 【2026-10-06】時間帯・状況ルールの適用状況(ルールid / 方針)。ルールが無ければ「なし」。
      document.getElementById("activeRule").textContent = s.active_rule
        ? `${s.active_rule}(${s.policy || "normal"})`
        : "なし(現行動作)";
      document.getElementById("audioBusy").textContent = s.audio_busy ? "はい" : "いいえ";
      document.getElementById("manualOverride").textContent = s.manual_override ? "はい" : "いいえ";
      document.getElementById("odomXY").textContent = s.odom ? `${s.odom.x.toFixed(2)}, ${s.odom.y.toFixed(2)}` : "-";
      document.getElementById("announce").textContent = s.announce_text || "";
      const menu = s.menu_state || {};
      document.getElementById("menuPhase").textContent = menu.phase || "-";
      document.getElementById("promoteItem").textContent = menu.promote_item || "なし";
      const items = menu.items || [];
      lastMenuItems = items;
      document.getElementById("menuRows").innerHTML = items.length ? items.map(item => `
        <tr class="${item.name === menu.promote_item ? 'promote' : ''}">
          <td>${escapeHtml(item.name)}</td><td>${item.initial}</td><td>${item.remaining}</td><td>${item.sold}</td>
        </tr>
      `).join("") : '<tr><td colspan="4">未登録</td></tr>';
      updateStatusBadge(s);
      // 【2026-10-07】tfが無い/古い(localizationでAMCLの位置合わせ前・停止後)時は、
      // 古い位置にマーカーを残さず消す(地図上に「TFなし(位置合わせ前)」を出す)。
      const rp = s.robot_pose;
      robotPose = (rp && !(rp.age_s !== null && rp.age_s !== undefined && rp.age_s > 5)) ? rp : null;
      drawRobot();
      routeTrail = s.trail || [];
      plannedPath = s.planned_path || [];
    } catch (e) {
      document.getElementById("estopBadge").textContent = "接続断";
    }
  }
  setInterval(poll, 500);
  poll();

  async function loadSpeedSettings() {
    try {
      const res = await fetch("/api/speed_settings");
      const s = await res.json();
      // 入力中・未保存の編集(dirty)・保存確認中の欄は書き換えない(NKField)
      NKField.set("manualLinear", s.manual_linear_mps);
      NKField.set("manualAngular", s.manual_angular_rps);
      NKField.set("autoLinear", s.auto_linear_mps);
      NKField.set("autoAngular", s.auto_angular_rps);
      return s;
    } catch (e) { /* 未取得ならinput空のまま */ return null; }
  }
  const SPEED_KEY_ID = { manual_linear_mps: "manualLinear", manual_angular_rps: "manualAngular", auto_linear_mps: "autoLinear", auto_angular_rps: "autoAngular" };
  NKField.guard(Object.values(SPEED_KEY_ID), { clamp: true });  // 範囲はmin/max属性(サーバーが丸める範囲と同じ)。範囲外は入力時に表示
  async function speedRead() { try { return await (await fetch("/api/speed_settings")).json(); } catch (e) { return null; } }
  // サーバーが上限/下限へ丸めた値か(limitsは丸め後の上限。下限は0.01)
  function speedAccept(key) {
    return (sent, v, srv) => {
      const lim = (srv && srv.limits) || {}, base = key.replace(/_(mps|rps)$/, "");
      const mx = lim[base + "_max"], mn = lim[base + "_min"] !== undefined ? lim[base + "_min"] : 0.01;
      return (typeof mx === "number" && sent > mx && Math.abs(v - mx) < 1e-6) || (sent < mn && Math.abs(v - mn) < 1e-6);
    };
  }
  NKField.poll(loadSpeedSettings, 15000);  // 他の端末で変えた速度も反映する(編集中の欄は上書きしない)
  // 【2026-10-10】手動の高い領域(直進>1.1 / 旋回>3)はサーバーがconfirm_requiredで返す。インラインの確認バーで「上げる」を押すまで保存しない。
  let speedPendingReq = null;
  function speedConfirmHide() { speedPendingReq = null; document.getElementById("speedConfirmBar").style.display = "none"; }
  document.getElementById("speedConfirmNo").addEventListener("click", speedConfirmHide);
  document.getElementById("speedConfirmYes").addEventListener("click", () => { const r = speedPendingReq; speedConfirmHide(); if (r) saveSpeed(r, true); });
  document.getElementById("speedSaveBtn").addEventListener("click", async () => {
    // 【2026-10-02修正】空欄/数値でない欄はリクエストに含めない(以前はNumber("")=0を送り、
    // 最小値0.01m/sに丸めて保存されていた)。
    const req = {};
    for (const [key, id] of [["manual_linear_mps", "manualLinear"], ["manual_angular_rps", "manualAngular"], ["auto_linear_mps", "autoLinear"], ["auto_angular_rps", "autoAngular"]]) {
      const raw = document.getElementById(id).value.trim();
      if (raw === "" || !NKField.isDirty(id)) continue;  // 触った欄だけ送る(触っていない欄は、他の端末で変えられていても上書きしない)
      const v = Number(raw);
      if (Number.isFinite(v)) req[key] = v;
    }
    const hint = document.getElementById("speedSaveHint");
    if (!Object.keys(req).length) { hint.textContent = "保存する値が入力されていません(値を変えてから保存してください)"; return; }
    speedConfirmHide();
    saveSpeed(req, false);
  });
  async function saveSpeed(req, confirmHigh) {
    const hint = document.getElementById("speedSaveHint");
    let ok = false;
    const tx = NKField.begin("speed", Object.keys(req).map((k) => SPEED_KEY_ID[k]));  // 送信中・確認中は定期取得で入力欄を上書きしない
    try {
      const res = await fetch("/api/speed_settings", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify(confirmHigh ? Object.assign({ confirm_high: true }, req) : req) });
      let d = {};
      try { d = await res.json(); } catch (e2) { /* 本文なし */ }
      if (res.status === 400 && d.confirm_required) {
        speedPendingReq = req;
        document.getElementById("speedConfirmText").textContent = "手動の速度を高い値に上げます(直進1.1m/s超または旋回3rad/s超)。物理の緊急停止は未配線です。広い場所で、電源を切る手段を手元に置いて確認してください";
        document.getElementById("speedConfirmBar").style.display = "block";
        hint.textContent = " ";
        tx.fail();
        return;
      }
      if (!res.ok) { showError(d.message || `操作エラー(${res.status})`); } else { showError(""); ok = true; }
    } catch (e) { showError("通信エラー: サーバーに接続できません"); }
    if (!ok) { tx.fail(); hint.textContent = "保存できませんでした(入力した値は残しています)"; return; }
    // 保存後にサーバーが実際に適用した値を読み直して入力欄へ反映する
    // (上限を超える値はサーバー側で丸められるため、入力した値のまま残さない)。送った値と違う(丸めでもない)ときは警告して入力値を残す。
    const labels = { manual_linear_mps: "手動直進", manual_angular_rps: "手動旋回", auto_linear_mps: "自動移動直進", auto_angular_rps: "自動移動旋回" };
    const vr = await tx.verify({
      items: Object.keys(req).map((k) => ({ el: SPEED_KEY_ID[k], sent: req[k], label: labels[k], read: (srv) => srv[k], accept: speedAccept(k) })),
      load: speedRead, delays: [0, 800, 2000],
    });
    const s = vr.server;
    if (vr.state === "mismatch") { hint.textContent = "反映されていません: " + vr.rows.filter((x) => x.status === "mismatch").map((x) => `${x.label} 送信 ${x.sent} / サーバー ${x.server}`).join("、"); return; }
    if (vr.state === "unknown") { hint.textContent = "保存を送信しましたが、反映を確認できませんでした(サーバーから値を読めません)"; return; }
    const clamped = [];
    if (s) {
      for (const k of Object.keys(req)) {
        if (s[k] !== undefined && Math.abs(Number(req[k]) - Number(s[k])) > 1e-6) {
          clamped.push(`${labels[k]} ${req[k]}→${s[k]}`);
        }
      }
    }
    if (clamped.length) {
      const lim = s.limits || {};
      hint.textContent = `上限/下限に丸めて保存しました: ${clamped.join("、")}(上限 手動: 直進${lim.manual_linear_max}m/s・旋回${lim.manual_angular_max}rad/s / 自動: 直進${lim.auto_linear_max}m/s・旋回${lim.auto_angular_max}rad/s)`;
      setTimeout(() => { hint.textContent = " "; }, 8000);
    } else {
      hint.textContent = "保存しました";
      setTimeout(() => { hint.textContent = " "; }, 2000);
    }
  }
  loadSpeedSettings();

  // --- Nav2(自律走行)の最高速度(2026-10-07追加) ---
  // 0.6m/sを超える値は、インラインの確認バーで「上げる」を押すまで送らない(ブラウザのダイアログは使わない)。
  const NAV_CONFIRM_ABOVE = 0.6, NAV_MAX = 1.1, NAV_MIN = 0.1;
  const navEls = {
    input: document.getElementById("navSpeed"), btn: document.getElementById("navSpeedBtn"),
    hint: document.getElementById("navSpeedHint"), bar: document.getElementById("navConfirmBar"),
    text: document.getElementById("navConfirmText"), yes: document.getElementById("navConfirmYes"),
    no: document.getElementById("navConfirmNo"), info: document.getElementById("navSpeedInfo"),
  };
  let navPending = null, navPendingKind = "nav";
  function navHideConfirm() { navPending = null; navEls.bar.style.display = "none"; }
  function navShowConfirm(v, kind) {
    navPending = v; navPendingKind = kind || "nav";
    navEls.text.textContent = `自動走行を ${v} m/s に上げます。物理の緊急停止は未配線です。広い場所で、電源を切る手段を手元に置いて確認してください`;
    navEls.bar.style.display = "block";
  }
  async function loadNavSpeed() {
    try {
      const res = await fetch("/api/nav_speed");
      if (!res.ok) return;
      const s = await res.json();
      const rb = s.read_back || {};
      const c = (rb.nodes || {}).controller_server || {};
      const live = rb.running && typeof c["FollowPath.max_vel_x"] === "number";
      // Nav2が止まっている間は「最後に反映した値」はもう効いていない(起動し直すと既定値になる)ので、その値を現在値として見せない
      const stale = !rb.running && s.nav_max_speed_source === "live" && typeof s.nav_speed_default === "number";
      const cur = live ? c["FollowPath.max_vel_x"] : (stale ? s.nav_speed_default : s.nav_max_speed_mps);
      NKField.set(navEls.input, cur);  // 入力中・未適用の編集があれば上書きしない
      const auto = document.getElementById("autoLinear").value;
      const src = live ? "実行中のNav2の値" : (rb.running ? "Nav2に問い合わせできませんでした(最後に設定した値)" : (stale ? "Nav2は未起動。起動し直すと既定値に戻るため、起動後にもう一度「適用」してください" : "Nav2は未起動(次回起動時の既定値)"));
      navEls.info.textContent = `現在: Nav2 ${cur} m/s(${src}) / director自動(接近・見回り) ${auto || "-"} m/s。両者は別の設定です。`;
    } catch (e) { /* 取得できなければ入力欄は空のまま */ }
  }
  async function navRead() {
    try {
      const s = await (await fetch("/api/nav_speed")).json();
      const c = ((s.read_back || {}).nodes || {}).controller_server || {};
      return { v: typeof c["FollowPath.max_vel_x"] === "number" ? c["FollowPath.max_vel_x"] : undefined };
    } catch (e) { return null; }
  }
  async function sendNavSpeed(v, confirmHigh) {
    navHideConfirm();
    navEls.hint.textContent = "反映中…";
    const tx = NKField.begin("nav", [navEls.input]);
    try {
      const res = await fetch("/api/speed_settings", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify({ nav_max_speed_mps: v, confirm_high: confirmHigh }) });
      let d = {};
      try { d = await res.json(); } catch (e2) { /* 本文なし */ }
      if (res.status === 400 && d.confirm_required) { navShowConfirm(d.nav_max_speed_mps); navEls.hint.textContent = " "; tx.fail(); return; }
      navEls.hint.textContent = d.message || (res.ok ? "反映しました" : `反映できませんでした(${res.status})`);
      if (Array.isArray(d.nodes)) {
        const ng = d.nodes.filter(n => !n.ok).map(n => `${n.label}: ${n.detail}`);
        if (ng.length) navEls.hint.textContent += " [失敗] " + ng.join(" / ");
      }
      if (!res.ok) { tx.fail(); return; }  // 反映できていない(Nav2未起動など): 入力した値は残す
      // 実行中のNav2から値を読み直して、送った値と一致したことを確かめてから表示を更新する
      const vr = await tx.verify({ items: [{ el: navEls.input, sent: Math.max(NAV_MIN, Math.min(NAV_MAX, v)), label: "Nav2最高速度", read: (srv) => srv.v }], load: navRead, delays: [0, 1200] });
      if (vr.state === "mismatch") navEls.hint.textContent += " 反映されていません(実行中のNav2は " + vr.rows[0].server + " m/s のままです)";
      await loadNavSpeed();
    } catch (e) { tx.fail(); navEls.hint.textContent = "通信エラー: サーバーに接続できません"; }
  }
  navEls.btn.addEventListener("click", () => {
    const raw = navEls.input.value.trim();
    const v = Number(raw);
    if (raw === "" || !Number.isFinite(v)) { navEls.hint.textContent = "数値を入力してください"; return; }
    const clamped = Math.max(NAV_MIN, Math.min(NAV_MAX, v));
    if (clamped > NAV_CONFIRM_ABOVE) { navShowConfirm(clamped); navEls.hint.textContent = " "; return; }
    sendNavSpeed(v, false);
  });
  navEls.yes.addEventListener("click", () => {
    if (navPending === null) return;
    if (navPendingKind === "all") sendAllSpeed(navPending, true); else sendNavSpeed(navPending, true);
  });
  // 全て同じ速度にする(手動・director自動・Nav2。1リクエスト、結果は対象ごとに表示)
  const allEls = { input: document.getElementById("allSpeed"), btn: document.getElementById("allSpeedBtn"), hint: document.getElementById("allSpeedHint"), list: document.getElementById("allSpeedResult") };
  async function sendAllSpeed(v, confirmHigh) {
    navHideConfirm();
    allEls.hint.textContent = "反映中…";
    allEls.list.textContent = "";
    NKField.bump("speed");  // 送信前に始まった古い取得結果を捨てる
    try {
      const res = await fetch("/api/speed_settings", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify({ all_linear_mps: v, confirm_high: confirmHigh }) });
      let d = {};
      try { d = await res.json(); } catch (e2) { /* 本文なし */ }
      if (res.status === 400 && d.confirm_required) { navShowConfirm(d.all_linear_mps, "all"); allEls.hint.textContent = " "; return; }
      const u = d.unified;
      if (!u) { allEls.hint.textContent = d.message || `反映できませんでした(${res.status})`; return; }
      allEls.hint.textContent = u.clamped ? `指定値${u.requested}を${u.applied}m/sに丸めました` : `${u.applied} m/s を指定しました`;
      for (const t of u.targets) {
        const li = document.createElement("li");
        li.textContent = `${t.label}: ${t.status}` + (t.status === "成功" ? ` (${t.applied} m/s)` : (t.detail ? ` - ${t.detail}` : ""));
        allEls.list.appendChild(li);
      }
      NKField.clean(allEls.input);  // 反映済み(結果は下の一覧で対象ごとに確認する)
      await loadSpeedSettings();
      await loadNavSpeed();
    } catch (e) { allEls.hint.textContent = "通信エラー: サーバーに接続できません"; }
  }
  allEls.btn.addEventListener("click", () => {
    const raw = allEls.input.value.trim();
    const v = Number(raw);
    if (raw === "" || !Number.isFinite(v)) { allEls.hint.textContent = "数値を入力してください"; return; }
    const clamped = Math.max(NAV_MIN, Math.min(NAV_MAX, v));
    if (clamped > NAV_CONFIRM_ABOVE) { navShowConfirm(clamped, "all"); allEls.hint.textContent = " "; return; }
    sendAllSpeed(v, false);
  });
  navEls.no.addEventListener("click", () => { navHideConfirm(); navEls.hint.textContent = "やめました(速度は変更していません)"; });
  navEls.input.addEventListener("input", navHideConfirm);
  NKField.guard([navEls.input, allEls.input], { clamp: true });  // 範囲はmin/max属性(0.1〜1.1)。範囲外は入力時に表示
  loadNavSpeed();
  NKField.poll(loadNavSpeed, 30000);  // Nav2の再起動などで実際の値が変わったら表示も追従(入力中は上書きしない)

  // --- BGM(voice_bridgeの/bgmをダッシュボード経由で操作、2026-10-03追加) ---
  const bgmEls = {
    block: document.getElementById("bgmBlock"), status: document.getElementById("bgmStatus"),
    toggle: document.getElementById("bgmToggleBtn"), vol: document.getElementById("bgmVolume"),
    duck: document.getElementById("bgmDuckVolume"), save: document.getElementById("bgmSaveBtn"),
    hint: document.getElementById("bgmHint"),
  };
  let bgmEnabled = null; // サーバーから読んだ実際の状態(null=不明/接続不可)

  function bgmFileNames(s) {
    const cur = s.playing || s.current || s.file || s.now_playing;
    if (cur) return [String(cur)];
    return Array.isArray(s.files) ? s.files.map((f) => String((f && f.name) || f)) : [];
  }

  // 状態を読み直して画面へ反映する(楽観更新はしない)。force=trueなら入力中の欄も上書きする。
  async function loadBgm(force) {
    const tk = NKField.ticket("bgm");
    let d = null;
    try {
      const res = await fetch("/api/bgm");
      d = await res.json();
    } catch (e) { d = { ok: false }; }
    if (!NKField.fresh(tk)) return;  // 送信前に始まった古い取得結果は捨てる(画面が一瞬古い値に戻らないように)
    if (d && d.configured === false) { bgmEls.block.style.display = "none"; return; }
    bgmEls.block.style.display = "";
    if (!d || !d.ok || !d.status) {
      bgmEnabled = null;
      bgmEls.status.textContent = "BGM: 接続できません";
      bgmEls.toggle.textContent = "-";
      bgmEls.toggle.disabled = true;
      bgmEls.save.disabled = true;
      return;
    }
    const s = d.status;
    bgmEnabled = !!s.enabled;
    const names = bgmFileNames(s);
    let line = `BGM: <b>${bgmEnabled ? "ON" : "OFF"}</b>`;
    if (s.available === false) line += " / 再生できるBGMファイルがありません";
    else if (s.playing) line += ` / 再生中${names.length ? ": " + names.map(escapeHtml).join("、") : ""}`;
    else line += " / 停止中";
    if (s.muted) line += "(音声の全体OFFのため停止中)";
    if (s.ducked) line += "(発話中のため音量を下げています)";
    if (s.stream_ok === false) line += " / 音声出力に失敗しています";
    bgmEls.status.innerHTML = line;
    bgmEls.toggle.textContent = bgmEnabled ? "BGMをOFFにする" : "BGMをONにする";
    bgmEls.toggle.disabled = readOnlyAccount;
    bgmEls.save.disabled = readOnlyAccount;
    // 入力中・未保存の編集(dirty)・保存確認中の欄は書き換えない(NKField)。force=trueのときだけ上書きする
    if (typeof s.volume === "number") { NKField.set(bgmEls.vol, s.volume, { force: !!force }); bgmEls.vol.dataset.orig = String(s.volume); }
    if (typeof s.duck_volume === "number") { NKField.set(bgmEls.duck, s.duck_volume, { force: !!force }); bgmEls.duck.dataset.orig = String(s.duck_volume); }
  }
  async function bgmRead() {
    try { const d = await (await fetch("/api/bgm")).json(); return d && d.ok && d.status ? d.status : null; } catch (e) { return null; }
  }
  NKField.guard([bgmEls.vol, bgmEls.duck]);
  bgmEls.toggle.addEventListener("click", async () => {
    if (bgmEnabled === null) return;
    const want = !bgmEnabled;
    NKField.bump("bgm");
    const d = await postForMessage("/api/bgm", JSON.stringify({ enabled: want }));
    bgmEls.hint.textContent = d.ok ? "" : (d.message || "失敗しました");
    NKField.bump("bgm");
    await loadBgm(false);
    if (d.ok && bgmEnabled !== want) bgmEls.hint.textContent = "切り替えを送りましたが、まだ反映が確認できません(BGMは" + (bgmEnabled ? "ON" : "OFF") + "のまま)。数秒後に表示が更新されます";
  });
  bgmEls.save.addEventListener("click", async () => {
    const req = {}, sentEls = [];
    for (const [key, el] of [["volume", bgmEls.vol], ["duck_volume", bgmEls.duck]]) {
      if (!NKField.isDirty(el)) continue;  // 触った欄だけ送る(触っていない欄は、他の端末で変えられていても上書きしない/1超の値を丸めない)
      const raw = el.value.trim();
      if (raw === "" || (el.dataset.orig !== undefined && Number(raw) === Number(el.dataset.orig))) continue;  // 元の値のままなら送らない
      const v = Number(raw);
      if (Number.isFinite(v)) { req[key] = v; sentEls.push([key, el]); }
    }
    if (!Object.keys(req).length) { bgmEls.hint.textContent = "保存する値が入力されていません(値を変えてから押してください)"; return; }
    const tx = NKField.begin("bgm", sentEls.map((x) => x[1]));
    const d = await postForMessage("/api/bgm", JSON.stringify(req));
    if (!d.ok) { tx.fail(); bgmEls.hint.textContent = d.message || "保存できませんでした(入力した値は残しています)"; return; }
    const r = await tx.verify({ items: sentEls.map(([key, el]) => ({ el: el, sent: req[key], label: key === "volume" ? "通常音量" : "発話中音量", read: (s) => s[key] })), load: bgmRead });
    bgmEls.hint.textContent = r.state === "ok" ? "保存しました" : (r.state === "clamped" ? "保存しました(範囲外の値は丸められました)"
      : (r.state === "mismatch" ? "反映されていません: " + r.rows.filter((x) => x.status === "mismatch").map((x) => x.label + " 送信 " + x.sent + " / サーバー " + x.server).join("、")
        : "送信しましたが、反映を確認できませんでした"));
    await loadBgm(false);
  });
  loadBgm(true);
  NKField.poll(() => loadBgm(false), 5000);

  // --- 音声 全体ON/OFF(voice_bridgeの/voice_masterをダッシュボード経由で操作、2026-10-09追加) ---
  // OFFの間は発話・AIによる文面生成・BGMがすべて止まる。切替は確認バーで「実行する」を押すまで送らない。
  // 状態は5秒ごと+切替直後に取り直す。2台目に届かないときは状態を「不明」にしてエラーを出す。
  const vmEls = {
    block: document.getElementById("voiceMasterBlock"), state: document.getElementById("voiceMasterState"),
    detail: document.getElementById("voiceMasterDetail"), btn: document.getElementById("voiceMasterBtn"),
    bar: document.getElementById("voiceMasterConfirmBar"), text: document.getElementById("voiceMasterConfirmText"),
    yes: document.getElementById("voiceMasterYes"), no: document.getElementById("voiceMasterNo"),
    hint: document.getElementById("voiceMasterHint"), chip: document.getElementById("voiceOffChip"),
  };
  let vmEnabled = null; // サーバーから読んだ実際の状態(null=不明/接続不可)
  let vmPending = null;
  function vmHideConfirm() { vmPending = null; vmEls.bar.style.display = "none"; }
  async function loadVoiceMaster() {
    const tk = NKField.ticket("vm");
    let d = null;
    try {
      const res = await fetch("/api/voice_master");
      d = await res.json();
    } catch (e) { d = null; }
    if (!NKField.fresh(tk)) return;  // 切り替え前に始まった古い取得結果は捨てる
    if (d && d.configured === false) { vmEls.block.style.display = "none"; vmEls.chip.hidden = true; return; }
    vmEls.block.style.display = "";
    if (!d || !d.ok || !d.status || typeof d.status.enabled !== "boolean") {
      vmEnabled = null;
      vmEls.chip.hidden = true;
      vmEls.state.textContent = "音声の状態を取得できません";
      vmEls.state.style.background = "#fff6df";
      vmEls.state.style.color = "#7a5c00";
      vmEls.detail.textContent = (d && d.message) || "通信エラー(サーバーに接続できません)";
      vmEls.btn.textContent = "-";
      vmEls.btn.disabled = true;
      return;
    }
    const s = d.status;
    vmEnabled = s.enabled;
    vmEls.chip.hidden = vmEnabled;
    vmEls.state.textContent = vmEnabled ? "音声: ON" : "音声: OFF(完全に無音)";
    vmEls.state.style.background = vmEnabled ? "var(--ok-bg, #e3f4ea)" : "var(--bad-bg, #faece7)";
    vmEls.state.style.color = vmEnabled ? "var(--ok, #15603a)" : "var(--bad, #993c1d)";
    let detail;
    if (vmEnabled) {
      detail = "発話・AIによる文面生成・BGM(BGM個別設定がONのとき)が通常どおり動いています。";
    } else {
      detail = "発話(合成・再生)、AI(Ollama)による文面生成、BGMをすべて止めています。呼びかけの依頼は受け付けても何もしません。"
        + (bgmEnabled === null ? "" : `BGM個別設定: ${bgmEnabled ? "ON" : "OFF"}(ONに戻すとこの設定で再開)。`);
      if (s.muted_requests) detail += ` OFF中に見送った依頼: ${s.muted_requests}件。`;
    }
    if (s.persisted === false) detail += " 注意: 状態を保存できていません(2台目の再起動で元に戻ります)。";
    if (s.load_error) detail += " " + s.load_error;
    vmEls.detail.textContent = detail;
    vmEls.btn.textContent = vmEnabled ? "音声を全体OFFにする" : "音声を全体ONに戻す";
    vmEls.btn.disabled = readOnlyAccount;
  }
  vmEls.btn.addEventListener("click", () => {
    if (vmEnabled === null) return;
    vmPending = !vmEnabled;
    vmEls.text.textContent = vmPending
      ? "音声を全体ONに戻します。発話が再開し、BGMもBGM個別設定がONなら再開します。よろしいですか？"
      : "音声を全体OFFにします。発話・AIによる文面生成・BGMがすべて止まり、再生中の音もその場で止まります。よろしいですか？";
    vmEls.bar.style.display = "block";
  });
  vmEls.no.addEventListener("click", vmHideConfirm);
  vmEls.yes.addEventListener("click", async () => {
    const want = vmPending;
    vmHideConfirm();
    if (want === null) return;
    vmEls.hint.textContent = "切り替え中…";
    NKField.bump("vm");
    let d = null;
    try {
      const res = await fetch("/api/voice_master", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify({ enabled: want }) });
      try { d = await res.json(); } catch (e2) { /* 本文がJSONでない */ }
      if (!res.ok) d = { ok: false, message: (d && d.message) || `操作エラー(${res.status})` };
    } catch (e) {
      d = { ok: false, message: "通信エラー(サーバーに接続できません)" };
    }
    if (d && d.ok) vmEls.hint.textContent = (want ? "音声をONに戻しました" : "音声を全体OFFにしました") + (d.message ? "。" + d.message : "");
    else vmEls.hint.textContent = "失敗: " + ((d && d.message) || "切り替えできませんでした");
    let reflected = false;
    for (const w of [0, 800, 1700, 3000]) {  // 2台目の反映遅れを待って、状態が切り替わったことを確かめる
      if (w) await new Promise((r) => setTimeout(r, w));
      await loadVoiceMaster();
      if (vmEnabled === want) { reflected = true; break; }
    }
    if (d && d.ok && !reflected) vmEls.hint.textContent = "切り替えを送りましたが、まだ反映が確認できません(音声は" + (vmEnabled === null ? "状態不明" : (vmEnabled ? "ON" : "OFF")) + ")。数秒後に表示が更新されます";
    await loadBgm(false);
  });
  loadVoiceMaster();
  NKField.poll(loadVoiceMaster, 5000);

  // --- 音量(発話・BGM通常・BGM発話中)(voice_bridgeの/voice_levelsをダッシュボード経由で操作、2026-10-09追加) ---
  // 「適用」で3項目まとめて送る(部分更新)。保存は2台目側で行われ再起動後も維持。発話音量は次の発話から反映。
  // 5秒ごとに現在値を取り直すが、入力中(未適用の編集あり)は入力欄を上書きしない。
  const vlEls = {
    block: document.getElementById("voiceLevelsBlock"), now: document.getElementById("vlNow"),
    apply: document.getElementById("vlApplyBtn"), test: document.getElementById("vlTestBtn"),
    reset: document.getElementById("vlResetBtn"), hint: document.getElementById("vlHint"),
    bar: document.getElementById("vlConfirmBar"), barText: document.getElementById("vlConfirmText"),
    yes: document.getElementById("vlYes"), no: document.getElementById("vlNo"),
  };
  // 上限(倍率)と、確認バーを出す境目。voice_levels_proxy.LEVEL_MAX_ALL / LEVEL_CONFIRM_ABOVE と同じ
  const VL_MAX = 5, VL_CONFIRM_ABOVE = 2;
  const VL_FIELDS = [
    { key: "speech_volume", label: "発話音量", max: VL_MAX, range: document.getElementById("vlSpeech"), num: document.getElementById("vlSpeechNum") },
    { key: "bgm_volume", label: "BGM通常音量", max: VL_MAX, range: document.getElementById("vlBgm"), num: document.getElementById("vlBgmNum") },
    { key: "bgm_duck_volume", label: "BGM発話中音量", max: VL_MAX, range: document.getElementById("vlDuck"), num: document.getElementById("vlDuckNum") },
  ];
  let vlCurrent = null;  // サーバーで適用中の値(確認バーを出すか=今より上げるか、の判定用)
  let vlPendingReq = null;
  function vlHideConfirm() { vlPendingReq = null; vlEls.bar.style.display = "none"; }
  function vlRound(v) { return Math.round(v * 100) / 100; }
  function vlDb(v) { return v > 0 ? (20 * Math.log10(v)).toFixed(1) : "-∞"; }
  // スライダーと数値欄は1組(link)。編集中・ドラッグ中・未適用の編集(dirty)・適用の確認中は、取得した値で上書きしない(NKField)
  VL_FIELDS.forEach((f) => { NKField.guard([f.range, f.num]); NKField.link(f.range, f.num); });
  function vlDirtyFields() { return VL_FIELDS.filter((f) => NKField.isDirty(f.num) || NKField.isDirty(f.range)); }
  function vlSetInputs(s, force) {
    VL_FIELDS.forEach((f) => { if (typeof s[f.key] === "number") NKField.set(f.num, vlRound(s[f.key]), { force: !!force }); });
  }
  VL_FIELDS.forEach((f) => {
    f.range.addEventListener("input", () => { f.num.value = vlRound(Number(f.range.value)); vlHideConfirm(); });
    f.num.addEventListener("input", () => { if (f.num.value !== "" && Number.isFinite(Number(f.num.value))) f.range.value = f.num.value; vlHideConfirm(); });
  });
  async function vlRead() {
    try { const d = await (await fetch("/api/voice_levels")).json(); return d && d.ok && d.status ? d.status : null; } catch (e) { return null; }
  }
  async function loadVoiceLevels(force) {
    const tk = NKField.ticket("vl");
    let d = null;
    try { d = await (await fetch("/api/voice_levels")).json(); } catch (e) { d = null; }
    if (!NKField.fresh(tk)) return;  // 適用の前に始まった古い取得結果は捨てる(スライダーが一瞬古い値に戻らないように)
    if (d && d.configured === false) { vlEls.block.style.display = "none"; return; }
    vlEls.block.style.display = "";
    if (!d || !d.ok || !d.status) {
      vlEls.now.textContent = "音量を取得できません: " + ((d && d.message) || "通信エラー(サーバーに接続できません)");
      vlEls.apply.disabled = true; vlEls.test.disabled = true; vlEls.reset.disabled = true;
      return;
    }
    const s = d.status;
    vlCurrent = s;
    vlEls.now.textContent = `現在値: 発話 ×${vlRound(s.speech_volume)}(${vlDb(s.speech_volume)}dB) / BGM ${vlRound(s.bgm_volume)}(${vlDb(s.bgm_volume)}dB) / BGM発話中 ${vlRound(s.bgm_duck_volume)}(${vlDb(s.bgm_duck_volume)}dB)`
      + (s.persisted === false ? "(注意: 保存できていません。2台目の再起動で元に戻ります)" : "")
      + (s.load_error ? " " + s.load_error : "");
    vlEls.apply.disabled = readOnlyAccount; vlEls.test.disabled = readOnlyAccount; vlEls.reset.disabled = readOnlyAccount;
    vlSetInputs(s, force);
  }
  async function vlPost(payload, okText) {
    const reset = Object.values(payload).some((v) => v === null);
    const fields = VL_FIELDS.filter((f) => f.key in payload);
    vlEls.hint.textContent = "反映中…";
    const tx = NKField.begin("vl", fields.map((f) => [f.range, f.num]));
    const d = await postForMessage("/api/voice_levels", JSON.stringify(payload));
    if (!d.ok) {
      tx.fail();
      vlEls.hint.textContent = d.message || "反映できませんでした(入力した値は残しています)";
      await loadVoiceLevels(false);
      return;
    }
    if (reset) {  // 既定に戻す: 既定値は2台目が決めるので、読み直した値をそのまま表示する
      tx.done();
      NKField.clean(fields.map((f) => [f.range, f.num]));
      vlEls.hint.textContent = d.message || okText;
      await loadVoiceLevels(true);
      await loadBgm(false);
      return;
    }
    // 2台目の保存値を読み直して、送った値と一致したことを確かめてから表示を更新する(不一致なら入力値を残して警告)
    const r = await tx.verify({
      items: fields.map((f) => ({ el: f.num, sent: payload[f.key], label: f.label, read: (s) => s[f.key] })),
      load: vlRead,
    });
    if (r.state === "ok" || r.state === "clamped") vlEls.hint.textContent = d.message || okText;
    else if (r.state === "mismatch") vlEls.hint.textContent = "反映されていません: " + r.rows.filter((x) => x.status === "mismatch").map((x) => x.label + " 送信 " + x.sent + " / サーバー " + x.server).join("、");
    else vlEls.hint.textContent = "送信しましたが、反映を確認できませんでした(2台目から値を読めません)";
    await loadVoiceLevels(false);
    await loadBgm(false);
  }
  vlEls.apply.addEventListener("click", async () => {
    const dirty = vlDirtyFields();
    if (!dirty.length) { vlEls.hint.textContent = "変更がありません(値を変えてから「適用」を押してください)"; return; }
    const req = {};  // 触った欄だけ送る(触っていない欄は、他の端末で変えられていても上書きしない)
    for (const f of dirty) {
      const raw = f.num.value.trim();
      const v = Number(raw);
      if (raw === "" || !Number.isFinite(v)) { vlEls.hint.textContent = `${f.label}は数値で入力してください`; return; }
      if (v < 0 || v > f.max) { vlEls.hint.textContent = `${f.label}は0〜${f.max}で入力してください`; return; }
      req[f.key] = v;
    }
    const eff = (key) => (req[key] !== undefined ? req[key] : (vlCurrent ? vlCurrent[key] : undefined));
    if (eff("bgm_duck_volume") > eff("bgm_volume")) { vlEls.hint.textContent = "BGM発話中音量は通常音量以下にしてください"; return; }
    // 2倍を超えて「今より上げる」項目があるときは確認バー(「実行する」を押すまで送らない)。下げる/据え置きは確認なし
    const loud = VL_FIELDS.filter((f) => req[f.key] > VL_CONFIRM_ABOVE && req[f.key] > ((vlCurrent && vlCurrent[f.key]) || 0) + 1e-9);
    if (loud.length) {
      vlPendingReq = req;
      vlEls.barText.textContent = `${loud.map((f) => `${f.label} ${vlRound(req[f.key])}(約+${vlDb(req[f.key])}dB)`).join("、")} に上げます。`
        + "大きい音になり、音割れ・スピーカーの故障の恐れがあります。近くに人がいないか確認してください。よろしいですか？";
      vlEls.bar.style.display = "block";
      vlEls.hint.textContent = "確認バーで「実行する」を押すと反映します";
      return;
    }
    await vlPost(req, "反映しました(保存済み)");
  });
  vlEls.no.addEventListener("click", () => { vlHideConfirm(); vlEls.hint.textContent = "やめました(音量は変えていません)"; });
  vlEls.yes.addEventListener("click", async () => {
    const req = vlPendingReq;
    vlHideConfirm();
    if (req) await vlPost(req, "反映しました(保存済み)。発話は次の発話から、BGMは数秒かけて変わります");
  });
  vlEls.reset.addEventListener("click", async () => {
    vlHideConfirm();
    await vlPost({ speech_volume: null, bgm_volume: null, bgm_duck_volume: null }, "既定値に戻しました");
  });
  // テスト発話: 発話・音声ページの「この文面を試しに話す」と同じ /api/voice_settings/speak で短い固定文を1回流す。
  vlEls.test.addEventListener("click", async () => {
    vlEls.hint.textContent = "テスト発話を依頼中…(適用済みの音量で話します)";
    const d = await postForMessage("/api/voice_settings/speak", JSON.stringify({ text: "音量のテストです。聞こえますか？" }));
    vlEls.hint.textContent = d.ok ? (d.message || "テスト発話を依頼しました") : (d.message || "テスト発話できませんでした(2台目に届かない可能性があります)");
  });
  loadVoiceLevels(true);
  NKField.poll(() => loadVoiceLevels(false), 5000);

  // --- アンケート(頭部ディスプレイ): ラズパイのkiosk_admin.pyをダッシュボード経由で操作(2026-10-09追加) ---
  const svEls = {
    block: document.getElementById("surveyBlock"), mode: document.getElementById("surveyMode"),
    count: document.getElementById("surveyCount"), active: document.getElementById("surveyActive"),
    session: document.getElementById("surveySession"),
    last: document.getElementById("surveyLast"), cond: document.getElementById("surveyCond"),
    err: document.getElementById("surveyErr"), hint: document.getElementById("surveyHint"),
    toSurvey: document.getElementById("surveyToSurveyBtn"), toCustomer: document.getElementById("surveyToCustomerBtn"), toOcr: document.getElementById("surveyToOcrBtn"),
    toInv: document.getElementById("surveyToInvBtn"), toFace: document.getElementById("surveyToFaceBtn"), auto: document.getElementById("surveyAutoBtn"), ipad: document.getElementById("surveyIpad"),
    confirm: document.getElementById("surveyConfirm"), confirmText: document.getElementById("surveyConfirmText"),
    yes: document.getElementById("surveyConfirmYes"), no: document.getElementById("surveyConfirmNo"),
  };
  let svMode = null, svPending = null;
  function svAge(sec) {
    if (typeof sec !== "number") return "なし";
    if (sec < 90) return Math.round(sec) + "秒前";
    if (sec < 5400) return Math.round(sec / 60) + "分前";
    return Math.round(sec / 3600) + "時間前";
  }
  // --- 模擬POS(OCRテスト)の数字変更(2026-10-09追加): kiosk_admin.py の /ocr_mock をダッシュボード経由で操作 ---
  const ocrmEls = {
    why: document.getElementById("ocrMockWhy"), hint: document.getElementById("ocrMockHint"),
    rand: document.getElementById("ocrMockRandBtn"), reset: document.getElementById("ocrMockResetBtn"),
    set: document.getElementById("ocrMockSetBtn"), ins: Array.from(document.querySelectorAll(".ocrm-in")),
  };
  const OCRM_DEFAULTS = [118, 129, 353, 166, 211, 178, 103];
  function ocrmEnable(mode) {   // OCRテストモードのときだけ操作可(mode=nullは未取得)
    const ok = mode === "ocrtest" && !readOnlyAccount;
    [ocrmEls.rand, ocrmEls.reset, ocrmEls.set].forEach((b) => { b.disabled = !ok; });
    ocrmEls.ins.forEach((i) => { i.disabled = readOnlyAccount; });
    ocrmEls.why.hidden = mode === "ocrtest" || mode === null;
    if (!ocrmEls.why.hidden) ocrmEls.why.textContent = mode === "unknown"
      ? "頭部ディスプレイの状態を取得できないため操作できません。"
      : "いまの頭部ディスプレイは「OCRテスト(模擬POS)」ではないため操作できません。上の「OCRテスト(模擬POS)」ボタンで切り替えてください。";
  }
  ocrmEnable(null);
  NKField.guardAll(".ocrm-in");  // 7つの数字欄(この欄はポーリングでは書き換えない。入力中の値を守るために登録)
  // 【2026-10-13変更】送信後は、模擬画面(ラズパイ)が実行した結果の読み戻し(kiosk_adminのocr_mock.ack_seq/shown_values)で
  // 「送った番号を適用した」「画面に実際に出ている7つの数字が指定と一致する」を確認してから、反映済みと表示する。
  // 以前は「送りました」と出すだけで、模擬画面を開いていなくても成功に見えた。
  const OCRM_DELAYS = [1000, 1000, 1500, 2000, 3000, 3000];
  async function ocrmLoad(seq) {
    try {
      const d = await (await fetch("/api/survey/state")).json();
      const m = d && d.ok && d.state && d.state.ocr_mock;
      if (m && typeof m.ack_seq === "number" && m.ack_seq >= seq && Array.isArray(m.shown_values) && m.shown_values.length === OCRM_DEFAULTS.length) return m;
    } catch (e) { /* 次の確認で再試行 */ }
    return null;
  }
  async function ocrmSend(body, okText, expect) {
    ocrmEls.hint.textContent = "送信中…";
    const tx = NKField.begin("ocrmock", ocrmEls.ins);
    let d = null;
    try {
      const res = await fetch("/api/survey/ocr_mock", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify(body) });
      d = await res.json().catch(() => null);
      if (!res.ok) d = { ok: false, message: "失敗: " + ((d && d.message) || `操作エラー(${res.status})`) };
    } catch (e) { d = { ok: false, message: "失敗: 通信エラー(サーバーに接続できません)" }; }
    if (!d || !d.ok) { tx.fail(); ocrmEls.hint.textContent = (d && d.message) || "変更できませんでした"; return; }
    const seq = d.state && d.state.ocr_mock && d.state.ocr_mock.seq;
    if (typeof seq !== "number") { tx.fail(); ocrmEls.hint.textContent = "送りましたが、届いたか確認できません(ラズパイが番号を返しませんでした)"; return; }
    ocrmEls.hint.textContent = "ラズパイへ送りました。模擬画面への反映を確認しています…";
    const shownText = (m) => m.shown_values.join("/");
    if (expect) {
      const r = await tx.verify({
        items: ocrmEls.ins.map((el, k) => ({ el: el, sent: expect[k], label: el.parentElement.firstChild.textContent, read: (m) => m.shown_values[k] })),
        load: () => ocrmLoad(seq), delays: OCRM_DELAYS,
      });
      if (r.state === "ok") ocrmEls.hint.textContent = "模擬画面に反映されました(表示: " + shownText(r.server) + ")";
      else if (r.state === "mismatch") ocrmEls.hint.textContent = "模擬画面の表示が指定と違います(表示: " + shownText(r.server) + ")。画面上で書き換えられた可能性があります";
      else ocrmEls.hint.textContent = "反映が確認できません(模擬画面を開いていますか?iPad(旧方式はラズパイ)の表示が止まっている可能性があります)";
    } else {
      let m = null;
      for (const w of OCRM_DELAYS) { await new Promise((r) => setTimeout(r, w)); m = await ocrmLoad(seq); if (m) break; }
      if (m) ocrmEls.ins.forEach((el, k) => NKField.set(el, String(m.shown_values[k]), { force: true }));   // 実際の表示を入力欄に写す
      tx.done();
      ocrmEls.hint.textContent = m ? "模擬画面に反映されました(" + okText + "。表示: " + shownText(m) + ")" : "反映が確認できません(模擬画面を開いていますか?iPad(旧方式はラズパイ)の表示が止まっている可能性があります)";
    }
  }
  ocrmEls.rand.addEventListener("click", () => ocrmSend({ action: "randomize" }, "ランダム化", null));
  ocrmEls.reset.addEventListener("click", () => {
    ocrmEls.ins.forEach((i, k) => { i.value = String(OCRM_DEFAULTS[k]); });
    NKField.clean(ocrmEls.ins);
    ocrmSend({ action: "reset" }, "元の値に戻しました", OCRM_DEFAULTS.slice());
  });
  ocrmEls.set.addEventListener("click", () => {
    const values = [];
    for (let k = 0; k < ocrmEls.ins.length; k++) {
      const raw = ocrmEls.ins[k].value.trim();
      if (!/^[0-9]{1,4}$/.test(raw)) { ocrmEls.hint.textContent = "数字は0〜9999の整数で入力してください(" + ocrmEls.ins[k].parentElement.firstChild.textContent + ")"; return; }
      values.push(parseInt(raw, 10));
    }
    ocrmSend({ action: "set", values }, "指定した値", values.slice());
  });

  let svBusy = false;  // 切り替え確認中は、定期取得で表示を戻さない
  async function loadSurvey(fromVerify) {
    if (svBusy && !fromVerify) return;
    const tk = NKField.ticket("sv");
    let d = null;
    try { d = await (await fetch("/api/survey/state")).json(); } catch (e) { d = { ok: false, error: "ダッシュボードに届きません" }; }
    if (!NKField.fresh(tk)) return;  // 切り替え前に始まった古い取得結果は捨てる
    if (d && d.configured === false) { svEls.block.style.display = "none"; return; }
    svEls.block.style.display = "";
    if (!d || !d.ok || !d.state) {
      svMode = null;
      svEls.err.hidden = false;
      svEls.err.textContent = (d && (d.message || d.error)) || "頭部ディスプレイの状態を取得できません";
      [svEls.mode, svEls.count, svEls.active, svEls.session, svEls.last, svEls.cond].forEach((e) => { e.textContent = "-"; });
      svEls.toSurvey.disabled = svEls.toCustomer.disabled = svEls.toOcr.disabled = svEls.toInv.disabled = svEls.toFace.disabled = svEls.auto.disabled = true;
      ocrmEnable("unknown");
      return;
    }
    const s = d.state;
    svMode = s.mode;
    svEls.err.hidden = s.receiver !== false;
    if (s.receiver === false) svEls.err.textContent = "アンケート受信サーバが止まっています(件数はファイルから数えました)";
    svEls.mode.textContent = s.mode === "survey" ? "アンケート表示中" : (s.mode === "ocrtest" ? "OCRテスト(模擬POS)" : (s.mode === "inventory" ? "残数入力" : (s.mode === "face" ? "顔" : "お客さん向け")))
      + (s.pending_mode ? "(回答が終わったら「" + ({ survey: "アンケート", customer: "お客さん向け", inventory: "残数入力", ocrtest: "OCRテスト", face: "顔" })[s.pending_mode] + "」へ切り替え)" : "");
    // iPad版のとき: 状態を読みに来ているiPadの台数(0台なら、iPadが開いていない/つながっていない)
    if (s.ipad && typeof s.ipad.connected === "number") svEls.ipad.textContent = " いまiPadの接続: " + s.ipad.connected + "台" + (s.ipad.connected === 0 ? "(iPadで /display を開いているか確認してください)" : "");
    else svEls.ipad.textContent = "";
    svEls.count.textContent = (typeof s.count === "number" ? s.count : "-") + (typeof s.count === "number" ? " 件" : "");
    svEls.active.textContent = s.active ? "回答中のためロボット停止中" : "いいえ";
    // 回答の状態(ラズパイのsession_state。古いラズパイには無い)。回答中は保存完了までロボットが動かない
    svEls.session.textContent = ({ idle: "待機(誰も操作していない)", answering: "回答中(保存完了まで停止)", saved: "保存完了", abandoned: "放置で終了(停止を解除)" })[s.session_state] || "-";
    svEls.last.textContent = svAge(s.last_activity_age_s);
    svEls.cond.textContent = s.condition || "未設定";
    svEls.toSurvey.disabled = readOnlyAccount || s.mode === "survey";
    svEls.toCustomer.disabled = readOnlyAccount || s.mode === "customer";
    svEls.toOcr.disabled = readOnlyAccount || s.mode === "ocrtest";
    svEls.toInv.disabled = readOnlyAccount || s.mode === "inventory";
    svEls.toFace.disabled = readOnlyAccount || s.mode === "face";
    svAutoOn = s.auto_face === true;
    svEls.auto.textContent = "自動切替(顔⇄アンケート): " + (svAutoOn ? "ON" : "OFF");
    svEls.auto.classList.toggle("primary", svAutoOn);
    svEls.auto.disabled = readOnlyAccount || typeof s.auto_face !== "boolean";
    ocrmEnable(s.mode);
  }
  let svAutoOn = false;
  svEls.auto.addEventListener("click", async () => {
    const want = !svAutoOn;
    if (!confirm(want ? "自動切替をONにします。普段は顔を出し、人が近づいたときだけアンケートにします(顔・アンケート以外の表示だったときは顔になります)。よろしいですか?"
                      : "自動切替をOFFにします。表示は今のまま固定されます。よろしいですか?")) return;
    svEls.auto.disabled = true;
    NKField.bump("sv");
    const d = await postForMessage("/api/survey/auto", JSON.stringify({ enabled: want }));
    svEls.hint.textContent = d.ok ? ("自動切替を" + (want ? "ONにしました。" : "OFFにしました。")) : (d.message || "自動切替を変更できませんでした");
    await loadSurvey(true);
  });
  function svAsk(mode) {
    svPending = mode;
    svEls.confirmText.textContent = mode === "survey"
      ? "頭部ディスプレイをアンケート画面に切り替えます。よろしいですか?"
      : mode === "inventory" ? "頭部ディスプレイを残数入力の画面に切り替えます(スタッフがログインして入力します。終わったらお客さん向けに戻してください)。よろしいですか?"
      : mode === "face" ? "頭部ディスプレイにロボットの顔を出します(お客さん向け画面は出なくなります。戻すときは「お客さん向けに戻す」)。よろしいですか?"
      : mode === "ocrtest" ? "頭部ディスプレイをOCRテスト用の模擬POS画面に切り替えます(お客さん向け画面は出なくなります)。よろしいですか?"
      : "頭部ディスプレイをお客さん向け画面に戻します。よろしいですか?" + (svEls.active.textContent.indexOf("回答中") === 0 ? "(いま回答中の人がいます)" : "");
    svEls.confirm.hidden = false;
  }
  svEls.toSurvey.addEventListener("click", () => svAsk("survey"));
  svEls.toCustomer.addEventListener("click", () => svAsk("customer"));
  svEls.toOcr.addEventListener("click", () => svAsk("ocrtest"));
  svEls.toInv.addEventListener("click", () => svAsk("inventory"));
  svEls.toFace.addEventListener("click", () => svAsk("face"));
  svEls.no.addEventListener("click", () => { svPending = null; svEls.confirm.hidden = true; });
  svEls.yes.addEventListener("click", async () => {
    const mode = svPending;
    svPending = null; svEls.confirm.hidden = true;
    if (!mode) return;
    svEls.hint.textContent = "切り替え中…";
    NKField.bump("sv");
    svBusy = true;
    const d = await postForMessage("/api/survey/mode", JSON.stringify({ mode }));
    if (!d.ok) { svBusy = false; svEls.hint.textContent = d.message || "切り替えに失敗しました"; await loadSurvey(true); return; }
    // iPad(旧方式はラズパイ)の画面は少し遅れて切り替わるので、状態が変わったと確認できるまで取り直す(最大約13秒)
    let reflected = false;
    for (const w of [0, 1500, 2500, 4000, 5000]) {
      if (w) await new Promise((r) => setTimeout(r, w));
      await loadSurvey(true);
      if (svMode === mode) { reflected = true; break; }
    }
    svBusy = false;
    svEls.hint.textContent = reflected ? "切り替えました。" : "切り替えを依頼しましたが、まだ反映が確認できません(いまの表示: " + svEls.mode.textContent + ")。少し待ってから再表示してください";
  });
  loadSurvey();
  NKField.poll(loadSurvey, 5000);

  // --- メニュー編集(2026-10-27追加): 追加・名前の変更・削除・初期食数。iPadと同じ DisplayHub.edit_menu(POST /api/menu/edit、権限 display.menu_edit) ---
  const mnEls = {
    block: document.getElementById("menuEditBlock"), list: document.getElementById("menuList"), badge: document.getElementById("menuBadge"),
    note: document.getElementById("menuNote"), hint: document.getElementById("menuHint"),
    confirm: document.getElementById("menuConfirm"), confirmText: document.getElementById("menuConfirmText"),
    yes: document.getElementById("menuConfirmYes"), no: document.getElementById("menuConfirmNo"),
  };
  let mnItems = [], mnMax = 12, mnDirty = false, mnBusy = false, mnDelName = null;
  function mnMsg(t, err) { mnEls.hint.textContent = t || "\u00a0"; mnEls.hint.style.color = err ? "var(--bad)" : ""; mnEls.hint.style.fontWeight = err ? "700" : ""; }
  async function mnPost(op, tries) {
    let res, d = null;
    try {
      res = await fetch("/api/menu/edit", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify(op) });
      d = await res.json().catch(() => null);
    } catch (e) { return { ok: false, message: "通信エラー(サーバーに接続できません)" }; }
    if (res.status === 409 && tries > 0) { await new Promise((r) => setTimeout(r, 1500)); return mnPost(op, tries - 1); }   // 直前の変更の反映待ち
    if (res.ok && d && d.ok) return { ok: true, message: d.summary || "" };
    return { ok: false, message: (d && (d.message || d.error)) || `操作エラー(${res.status})` };
  }
  function mnMkInput(value, attrs) {
    const el = document.createElement("input");
    Object.keys(attrs).forEach((k) => el.setAttribute(k, attrs[k]));
    el.value = value;
    return el;
  }
  function mnMkBtn(label, cls) {
    const b = document.createElement("button");
    b.type = "button"; b.textContent = label; if (cls) b.className = cls;
    return b;
  }
  function mnRender() {
    mnEls.list.textContent = "";
    mnItems.forEach((it) => {
      const row = document.createElement("div"); row.className = "mn-row";
      const nm = mnMkInput(it.name, { type: "text", maxlength: "40", "aria-label": "メニュー名", autocomplete: "off" });
      const ini = mnMkInput(String(it.initial), { type: "text", inputmode: "numeric", maxlength: "5", "aria-label": "初期食数", autocomplete: "off" });
      const save = mnMkBtn("保存", "primary"); save.disabled = true;
      const del = mnMkBtn("削除");
      const info = document.createElement("div"); info.className = "mn-info";
      info.textContent = "いま " + it.remaining + " 食 / 売れた " + it.sold + " 食";
      const onInput = () => {
        ini.value = ini.value.replace(/[^0-9]/g, "").slice(0, 5);
        const changed = nm.value.trim() !== it.name || ini.value !== String(it.initial);
        row.classList.toggle("mn-dirty", changed); save.disabled = !changed || mnBusy;
        mnDirty = true;
      };
      nm.addEventListener("input", onInput); ini.addEventListener("input", onInput);
      save.addEventListener("click", () => mnSave(it, nm.value.trim(), ini.value));
      del.addEventListener("click", () => { mnDelName = it.name; mnEls.confirmText.textContent = "「" + it.name + "」を削除します(売れた数 " + it.sold + " 食・残数 " + it.remaining + " 食の記録も一覧から消えます)。よろしいですか?"; mnEls.confirm.hidden = false; });
      row.append(nm, ini, save, del, info);
      mnEls.list.appendChild(row);
    });
    if (!mnItems.length) { const p = document.createElement("p"); p.className = "hint"; p.textContent = "メニューは登録されていません。下の欄から追加してください。"; mnEls.list.appendChild(p); }
    // 追加の行
    const add = document.createElement("div"); add.className = "mn-row mn-add";
    const an = mnMkInput("", { type: "text", maxlength: "40", placeholder: "新しいメニュー名", "aria-label": "追加するメニュー名", autocomplete: "off" });
    const ai = mnMkInput("", { type: "text", inputmode: "numeric", maxlength: "5", placeholder: "初期食数", "aria-label": "追加するメニューの初期食数", autocomplete: "off" });
    const ab = mnMkBtn("追加", "primary");
    const full = mnItems.length >= mnMax;
    ab.disabled = full; an.disabled = ai.disabled = full;
    if (full) an.placeholder = "メニューは最大" + mnMax + "品までです";
    ai.addEventListener("input", () => { ai.value = ai.value.replace(/[^0-9]/g, "").slice(0, 5); mnDirty = true; });
    an.addEventListener("input", () => { mnDirty = true; });
    ab.addEventListener("click", async () => {
      const name = an.value.trim();
      if (!name) { mnMsg("メニュー名を入力してください", true); return; }
      if (!/^[0-9]{1,5}$/.test(ai.value)) { mnMsg("初期食数は0〜99999の数字で入力してください", true); return; }
      mnBusy = true; mnMsg("追加しています…");
      const r = await mnPost({ op: "add", name: name, initial: parseInt(ai.value, 10) }, 6);
      mnBusy = false; mnDirty = false;
      mnMsg(r.ok ? (r.message || "追加しました") : "追加できませんでした: " + r.message, !r.ok);
      await mnLoad(true);
    });
    add.append(an, ai, ab);
    mnEls.list.appendChild(add);
    mnEls.badge.textContent = mnItems.length + "品"; mnEls.badge.className = "sbadge sb-ok";
  }
  async function mnSave(it, name, initRaw) {
    if (!name) { mnMsg("メニュー名を入力してください", true); return; }
    if (!/^[0-9]{1,5}$/.test(initRaw)) { mnMsg("初期食数は0〜99999の数字で入力してください", true); return; }
    const init = parseInt(initRaw, 10), ops = [];
    if (name !== it.name) ops.push({ op: "rename", name: it.name, new_name: name });
    if (init !== it.initial) ops.push({ op: "set_initial", name: name, initial: init });
    if (!ops.length) { mnMsg("変更がありません"); return; }
    mnBusy = true; mnMsg("送信中…");
    let last = null;
    for (const op of ops) { last = await mnPost(op, 6); if (!last.ok) break; }
    mnBusy = false; mnDirty = false;
    mnMsg(last.ok ? "反映しました" : "反映できませんでした: " + last.message, !last.ok);
    await mnLoad(true);
  }
  mnEls.no.addEventListener("click", () => { mnDelName = null; mnEls.confirm.hidden = true; });
  mnEls.yes.addEventListener("click", async () => {
    const name = mnDelName; mnDelName = null; mnEls.confirm.hidden = true;
    if (!name) return;
    mnBusy = true; mnMsg("削除しています…");
    const r = await mnPost({ op: "delete", name: name }, 6);
    mnBusy = false; mnDirty = false;
    mnMsg(r.ok ? (r.message || "削除しました") : "削除できませんでした: " + r.message, !r.ok);
    await mnLoad(true);
  });
  async function mnLoad(force) {
    if (!force && (mnDirty || mnBusy || !mnEls.confirm.hidden)) return;   // 入力中・送信中・確認中は、定期取得で書き換えない
    let d = null;
    try { d = await (await fetch("/api/menu/state")).json(); } catch (e) { d = null; }
    if (!d || d.configured === false) { mnEls.block.style.display = "none"; return; }
    mnEls.block.style.display = "";
    if (!d.ok) { mnEls.badge.textContent = "-"; mnEls.badge.className = "sbadge sb-mute"; return; }
    mnItems = Array.isArray(d.items) ? d.items : [];
    mnMax = d.max_items || 12;
    mnEls.note.textContent = d.editable === false ? "(残数管理につながっていません)" : "";
    mnRender();
  }
  mnLoad(true);
  NKField.poll(() => mnLoad(false), 5000);

  // --- タブレットOCR(読み取り): 画質プロファイル切替と読み取り状況(OCRの8091 APIをダッシュボード経由で操作、2026-10-09追加) ---
  const ocrEls = {
    block: document.getElementById("ocrBlock"), prof: document.getElementById("ocrProfile"),
    res: document.getElementById("ocrRes"), fps: document.getElementById("ocrFps"), interval: document.getElementById("ocrInterval"),
    rot: document.getElementById("ocrRot"), roi: document.getElementById("ocrRoi"), age: document.getElementById("ocrAge"),
    err: document.getElementById("ocrErr"), hint: document.getElementById("ocrHint"), rows: document.getElementById("ocrRows"),
    toDemo: document.getElementById("ocrToDemoBtn"), toProd: document.getElementById("ocrToProdBtn"), link: document.getElementById("ocrLink"),
    confirm: document.getElementById("ocrConfirm"), confirmText: document.getElementById("ocrConfirmText"),
    yes: document.getElementById("ocrConfirmYes"), no: document.getElementById("ocrConfirmNo"),
    quality: document.getElementById("ocrQuality"), tune: document.getElementById("ocrTuneBtn"), tuneRevert: document.getElementById("ocrTuneRevertBtn"),
  };
  const OCR_PROFILE_NAMES = { demo: "デモ(滑らか)", production: "本番(最高画質)" };
  let ocrProfile = null, ocrPending = null, ocrSwitching = false;
  // 【2026-10-21】OCR確認画面(8091)は1台目の内側だけになったため、映像は、ダッシュボードが中継する(/api/ocr/preview.jpg 等、ログイン必須)。
  // 見ている間だけOCRの負荷が上がるので、「映像を表示」を押したときだけ、約1秒ごとに取りにいく(非表示のタブ・非表示のカードでは止める)。
  const ocrView = { wrap: document.getElementById("ocrViewWrap"), prev: document.getElementById("ocrPreview"),
                    proc: document.getElementById("ocrProcessed"), hint: document.getElementById("ocrViewHint"), on: false, timer: null, n: 0 };
  function ocrViewTick() {
    if (!ocrView.on || document.hidden || ocrEls.block.style.display === "none") return;
    ocrView.n += 1;
    ocrView.prev.src = "/api/ocr/preview.jpg?t=" + Date.now();
    if (ocrView.n % 3 === 1) ocrView.proc.src = "/api/ocr/processed.jpg?t=" + Date.now();  // 認識に渡している画像は、3秒に1回
  }
  ocrView.prev.addEventListener("error", () => { ocrView.hint.textContent = "映像を取得できません(OCRが停止中か、まだ映像がありません)。"; });
  ocrView.prev.addEventListener("load", () => { ocrView.hint.textContent = "カメラ映像(約1秒ごとに更新)。見ている間は、OCRの負荷が少し上がります。"; });
  ocrEls.link.addEventListener("click", (ev) => {
    ev.preventDefault();
    ocrView.on = !ocrView.on;
    ocrEls.link.textContent = ocrView.on ? "映像を隠す" : "映像を表示";
    ocrEls.link.setAttribute("aria-pressed", ocrView.on ? "true" : "false");
    ocrView.wrap.hidden = !ocrView.on;
    if (ocrView.timer) { clearInterval(ocrView.timer); ocrView.timer = null; }
    if (ocrView.on) { ocrView.n = 0; ocrViewTick(); ocrView.timer = setInterval(ocrViewTick, 1000); }
  });
  function ocrNum(v, unit) { return typeof v === "number" && isFinite(v) ? (Math.round(v * 10) / 10) + unit : "-"; }
  async function loadOcr() {
    const tk = NKField.ticket("ocr");
    let d = null;
    try { d = await (await fetch("/api/ocr/state")).json(); } catch (e) { d = { ok: false, error: "ダッシュボードに届きません" }; }
    if (!NKField.fresh(tk)) return;  // 切り替え前に始まった古い取得結果は捨てる
    if (d && d.configured === false) { ocrEls.block.style.display = "none"; return; }
    ocrEls.block.style.display = "";
    if (!d || !d.ok || !d.state) {
      ocrProfile = null;
      ocrEls.err.hidden = false;
      ocrEls.err.textContent = (d && (d.message || d.error)) || "OCRの状態を取得できません";
      ocrEls.prof.textContent = "OCRに届きません";
      [ocrEls.res, ocrEls.fps, ocrEls.interval, ocrEls.rot, ocrEls.roi, ocrEls.age].forEach((e) => { e.textContent = "-"; });
      ocrEls.rows.innerHTML = '<tr><td colspan="4">-</td></tr>';
      ocrEls.toDemo.disabled = ocrEls.toProd.disabled = true;
      return;
    }
    const s = d.state;
    ocrProfile = s.profile;
    ocrEls.err.hidden = !(s.readings_error || !s.profile_supported);
    ocrEls.err.textContent = !s.profile_supported ? "このOCRは画質プロファイルに未対応です(OCR側の更新が必要です)" : (s.readings_error || "");
    ocrEls.prof.textContent = OCR_PROFILE_NAMES[s.profile] || "不明";
    const ca = s.camera_actual, pf = (s.profiles && s.profiles[s.profile]) || null;
    ocrEls.res.textContent = ca && ca.width && ca.height ? ca.width + "×" + ca.height : "-";
    ocrEls.fps.textContent = ca ? ocrNum(ca.fps, " fps") : "-";
    ocrEls.interval.textContent = pf && typeof pf.poll_interval_sec === "number" ? pf.poll_interval_sec + " 秒に1回" : "-";
    ocrEls.rot.textContent = typeof s.frame_rotate_deg === "number" ? s.frame_rotate_deg + "°" : "-";
    const lk = s.roi_lock;
    ocrEls.roi.textContent = !lk ? "-" : (lk.mode !== "auto_lock" ? "固定" : (lk.locked ? "ロック済み" : "検出中 " + (lk.progress || 0) + "/" + (lk.required || 0)));
    ocrEls.age.textContent = typeof s.updated_age_s === "number" ? Math.round(s.updated_age_s) + "秒前" : "-";
    ocrEls.rows.innerHTML = "";
    (s.readings || []).forEach((r) => {
      const tr = document.createElement("tr");
      const cells = [r.label, r.confirmed === null || r.confirmed === undefined ? "-" : String(r.confirmed) + (r.estimated ? "(推定)" : ""),
                     r.raw === null || r.raw === undefined ? "-" : String(r.raw), r.seen ? (r.sent ? "送信済み" : "未送信") : "-"];
      cells.forEach((t, k) => { const td = document.createElement("td"); if (k === 1 || k === 2) td.className = "n"; td.textContent = t; tr.appendChild(td); });
      ocrEls.rows.appendChild(tr);
    });
    if (!(s.readings || []).length) ocrEls.rows.innerHTML = '<tr><td colspan="4">読み取りデータがありません</td></tr>';
    // 画質(ピント・自動調整、2026-10-10追加。古いOCRはqualityが無い=非表示)
    const q = s.quality, qf = q && q.focus, qa = q && q.autotune;
    const tuning = !!(qa && ["requested", "running", "reverting"].includes(qa.state));
    ocrEls.quality.hidden = !q;
    ocrEls.quality.textContent = !q ? "" : "ピント: " + ((qf && qf.grade_label) || "測定不可") + (qf && qf.avg !== null && qf.avg !== undefined ? "(スコア " + qf.avg + ")" : "")
      + (tuning ? " / 自動調整中…(読み取りは一時停止) " + (qa.message || "") : (qa && qa.state === "done" ? " / " + (qa.message || "調整しました") : ""))
      + (qf && qf.advice ? " / " + qf.advice : "") + ((qf && qf.warnings) || []).map((w) => " / " + w).join("");
    ocrEls.tune.hidden = ocrEls.tuneRevert.hidden = !q;
    ocrEls.tune.disabled = readOnlyAccount || tuning;
    ocrEls.tuneRevert.disabled = readOnlyAccount || tuning || !(qa && qa.has_saved);
    const lock = readOnlyAccount || ocrSwitching || !s.profile_supported;
    ocrEls.toDemo.disabled = lock || s.profile === "demo";
    ocrEls.toProd.disabled = lock || s.profile === "production";
  }
  function ocrAsk(profile) {
    ocrPending = profile;
    ocrEls.confirmText.textContent = profile === "production"
      ? "OCRカメラを「本番(最高画質)」に切り替えます(数秒に1回の撮影になります)。よろしいですか?"
      : "OCRカメラを「デモ(滑らか)」に切り替えます(解像度を下げてfpsを確保します)。よろしいですか?";
    ocrEls.confirm.hidden = false;
  }
  ocrEls.toDemo.addEventListener("click", () => ocrAsk("demo"));
  ocrEls.toProd.addEventListener("click", () => ocrAsk("production"));
  ocrEls.no.addEventListener("click", () => { ocrPending = null; ocrEls.confirm.hidden = true; });
  ocrEls.yes.addEventListener("click", async () => {
    const profile = ocrPending;
    ocrPending = null; ocrEls.confirm.hidden = true;
    if (!profile) return;
    ocrSwitching = true;
    NKField.bump("ocr");
    ocrEls.toDemo.disabled = ocrEls.toProd.disabled = true;
    ocrEls.hint.textContent = "切替中…(カメラの再設定に数秒かかります)";
    const d = await postForMessage("/api/ocr/profile", JSON.stringify({ camera_profile: profile }));
    if (!d.ok) {
      ocrSwitching = false;
      ocrEls.hint.textContent = d.message || "切り替えに失敗しました";
      await loadOcr();
      return;
    }
    // 反映には数秒かかるので、少し待って取り直す(切替が終わったと確認できるまで最大約10秒)
    for (const wait of [1500, 2500, 3000, 3000]) {
      await new Promise((r) => setTimeout(r, wait));
      await loadOcr();
      if (ocrProfile === profile) break;
    }
    ocrSwitching = false;
    await loadOcr();
    ocrEls.hint.textContent = ocrProfile === profile ? (OCR_PROFILE_NAMES[profile] + " に切り替えました。") : "切替を依頼しましたが、まだ反映が確認できません。少し待ってから再表示してください。";
  });
  async function ocrTune(action, ask) {
    if (!confirm(ask)) return;
    const d = await postForMessage("/api/ocr/autotune", JSON.stringify({ action: action }));
    ocrEls.hint.textContent = d.ok ? (action === "start" ? "画質の自動調整を始めました(1分前後、読み取りは一時停止します)" : "元に戻しました") : (d.message || "実行できませんでした");
    await loadOcr();
  }
  ocrEls.tune.addEventListener("click", () => ocrTune("start", "OCRカメラの画質を自動調整します(1分前後、その間OCRの読み取りが止まります)。カメラに数字の画面が写っていますか?"));
  ocrEls.tuneRevert.addEventListener("click", () => ocrTune("revert", "自動調整の前のカメラ設定へ戻します。よろしいですか?"));
  loadOcr();
  NKField.poll(loadOcr, 5000);
__ACCOUNTS_JS__
__SHUTDOWN_JS__

  // --- guest(閲覧専用)アカウント対応: 操作系UIを無効化して見た目上も
  // 「モックアップ・見るだけ」と分かるようにする ---
  async function checkAccount() {
    try {
      const res = await fetch("/api/whoami");
      const w = await res.json();
      document.getElementById("accountLabel").textContent = w.account ? `(${w.account})` : "";
      if (w.account === "guest") {
        // 閲覧専用: 運転コマンドの定期送信を止める(403が返り続けるため)。
        readOnlyAccount = true;
        if (driveTimer !== null) { clearInterval(driveTimer); driveTimer = null; }
        document.querySelectorAll("button, input, select").forEach((el) => { el.disabled = true; });
        const banner = document.createElement("div");
        banner.textContent = "閲覧専用(guestアカウント): ここでは操作できません";
        banner.style.cssText = "background:#fff6df;color:#7a5c00;padding:6px 10px;font-size:13px;text-align:center;";
        document.body.insertBefore(banner, document.body.firstChild);
      }
    } catch (e) { /* 取得失敗時は通常通り(操作可能のまま)にしておく */ }
  }
  checkAccount();

  // ログアウト: サーバー側セッションを破棄し、専用ログインページへ移動する。
  document.getElementById("logoutBtn").addEventListener("click", async () => {
    try {
      await fetch("/api/logout", { method: "POST" });
    } catch (e) { /* 失敗してもログイン画面へは移動する */ }
    location.href = "/login";
  });
</script>
</body>
</html>
""".replace("__NOKOSAHEN_LOGO__", LOGO_DATA_URI).replace("__NAV_CSS__", NAV_CSS).replace("__NAV_HEADER__", nav_header_html()).replace("__NAV_DRAWER__", nav_drawer_html()).replace("__NAV_LINKS_ALL__", link_cards_html()).replace("__NAV_LINKS_SPEECH__", link_cards_html(("speech",), False)).replace("__NAV_LINKS_LOGS__", link_cards_html(("logs",), False)).replace("__NAV_LINKS_DISPLAY__", link_cards_html(("display",), False)).replace("__NAV_LINKS_RULES__", link_cards_html(("rules",), False)).replace("__NAV_JS__", NAV_JS).replace("__MAP_VIEW_JS__", MAP_VIEW_JS).replace("__ACCOUNTS_CARD__", ACCOUNTS_CARD_HTML).replace("__ACCOUNTS_JS__", ACCOUNTS_CARD_JS).replace("__SHUTDOWN_CARD__", SHUTDOWN_CARD_HTML).replace("__SHUTDOWN_JS__", SHUTDOWN_CARD_JS)

# スマホ(6.2インチ想定)から機体の移動制御だけを行うための最小限のページ。
# カメラ・LiDAR・地図・残数・決済カメラ等はここには置かない
# (それらはPC版の総合ダッシュボードを参照する)。ログインはPC版と共通。
MOBILE_HTML = """<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8" />
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no" />
<title>のこさへん - スマホ操作</title>
<style>
  :root { color-scheme: light; font-family: system-ui, -apple-system, "Segoe UI", "Hiragino Sans", sans-serif; background:#f2f5f3; color:#16211d;
    --s1:4px; --s2:8px; --s3:12px; --s4:16px; --line:#d9e2dd; --mute:#566860; }
  * { box-sizing:border-box; }
  html, body { margin:0; }
  body { background:#f2f5f3; font-size:14px; line-height:1.45; overflow-x:hidden; padding-bottom:env(safe-area-inset-bottom); }
  header { padding:var(--s2) var(--s3); background:#124739; color:#fff; display:flex; justify-content:space-between; align-items:center; gap:var(--s2); flex-wrap:wrap; }
  .logo-title { display:flex; align-items:center; gap:var(--s2); }
  .logo-title img { height:28px; width:28px; border-radius:6px; }
  h1 { margin:0; font-size:16px; }
  /* 緊急停止バー: 画面上端に固定(スクロールしても常に押せる)。運転→設定の順に並べる。 */
  #estopBar { position:sticky; top:0; z-index:1000; padding:var(--s2) var(--s3); background:#124739; box-shadow:0 2px 8px rgba(0,0,0,.3); }
  #estopBar .stop-btn { margin:0; border:2px solid #fff; }
  main { padding:var(--s3); display:flex; flex-direction:column; gap:var(--s3); max-width:480px; margin:0 auto; }
  section { background:#fff; border:1px solid var(--line); border-radius:12px; padding:var(--s3) var(--s4); }
  h2 { margin:0 0 var(--s2); font-size:15px; }
  .group-title { margin:var(--s2) var(--s1) 0; font-size:13px; font-weight:700; color:var(--mute); letter-spacing:.05em; }
  .estop-badge { padding:3px 10px; border-radius:999px; font-weight:700; font-size:13px; max-width:100%; }
  .estop-ok { background:#e1f5ee; color:#0f6e56; }
  .estop-bad { background:#faece7; color:#993c1d; }
  .estop-policy { background:#fff3cd; color:#7a5b00; }
  .header-right { display:flex; align-items:center; gap:var(--s2); flex-wrap:wrap; }
  #accountLabel { font-size:13px; opacity:0.85; }
  .logout-btn { padding:6px 10px; font-size:13px; border:1px solid rgba(255,255,255,0.5); border-radius:8px; background:transparent; color:#fff; cursor:pointer; min-height:32px; }
  .dpad { display:grid; grid-template-columns:repeat(3, minmax(0,84px)); grid-template-rows:repeat(3, 84px); gap:var(--s2); justify-content:center; margin:0 auto; }
  .dpad button { font-size:26px; border:0; border-radius:12px; background:#176b56; color:#fff; touch-action:none; user-select:none; -webkit-user-select:none; -webkit-touch-callout:none; }
  .dpad button:active, .dpad button.pressed { background:#0c4a3a; }
  .stop-btn { width:100%; padding:16px; font-size:17px; font-weight:700; border:0; border-radius:12px; background:#a32d2d; color:#fff; }
  .stop-btn.engaged { background:#5c1717; }
  .row { display:flex; gap:var(--s2); flex-wrap:wrap; margin-bottom:var(--s2); }
  .row label { flex:1 1 45%; font-size:13px; color:var(--mute); display:flex; flex-direction:column; gap:var(--s1); }
  .row input { font-size:16px; padding:8px; border:1px solid #bccbc3; border-radius:8px; width:100%; }
  .save-btn { width:100%; padding:12px; font-size:15px; font-weight:700; border:0; border-radius:12px; background:#176b56; color:#fff; }
  .row .save-btn { flex:1 1 45%; width:auto; }
  .hint { font-size:13px; color:var(--mute); margin-top:var(--s2); }
</style>
</head>
<body>
<header>
  <div class="logo-title">
    <img src="__NOKOSAHEN_LOGO__" alt="のこさへん" />
    <h1>のこさへん: スマホ操作</h1>
  </div>
  <div class="header-right">
    <span id="estopBadge" class="estop-badge estop-ok">--</span>
    <span id="accountLabel"></span>
    <button id="logoutBtn" class="logout-btn">ログアウト</button>
  </div>
</header>
<div id="estopBar">
  <button id="estopBtn" class="stop-btn">緊急停止(ソフト)</button>
</div>
<main>
  <div class="group-title">運転</div>
  <!-- 【2026-11-06】目印(data-cap)robot.manual_drive: 弱いアカウント(operator)も運転できる(ロール既定。guestは不可)。operatorは他に目印の無い操作が全て無効になるので、運転ボタンにだけ目印を付ける。
       安全: 手動運転は人が目で見て操作する前提。緊急停止中は動かず、速度上限・人避け・指令が途切れたら停止(watchdog)は従来どおり。TLの運転中は指令が破棄される。 -->
  <section data-cap="robot.manual_drive">
    <h2>手動運転</h2>
    <div class="dpad">
      <span></span><button data-fwd="1" data-turn="0">▲</button><span></span>
      <button data-fwd="0" data-turn="1">◀</button>
      <button data-fwd="0" data-turn="0">■</button>
      <button data-fwd="0" data-turn="-1">▶</button>
      <span></span><button data-fwd="-1" data-turn="0">▼</button><span></span>
    </div>
    <p class="hint">ボタンを押している間だけ動きます。操作している間は自律行動が一時停止します。</p>
  </section>

  <div class="group-title">設定</div>
  <section id="speedSection">
    <h2>最高速度設定</h2>
    <div class="row">
      <label>手動直進(m/s) <input id="manualLinear" type="number" step="0.01" min="0.01" max="1.2"></label>
      <label>手動旋回(rad/s) <input id="manualAngular" type="number" step="0.01" min="0.01" max="4"></label>
    </div>
    <div class="row">
      <label>自動移動直進(m/s) <input id="autoLinear" type="number" step="0.01" min="0.01" max="1.1"></label>
      <label>自動移動旋回(rad/s) <input id="autoAngular" type="number" step="0.01" min="0.01" max="3"></label>
    </div>
    <button id="speedSaveBtn" class="save-btn">保存</button>
    <div id="speedConfirmBar" role="alert" style="display:none;margin-top:8px;padding:8px;border:2px solid #b45309;border-radius:8px;background:#fff7e6;color:#5c2d00;">
      <span id="speedConfirmText"></span>
      <div class="row" style="margin-top:6px;"><button id="speedConfirmYes" type="button" class="save-btn">上げる</button><button id="speedConfirmNo" type="button" class="save-btn">やめる</button></div>
    </div>
    <p class="hint">上限: 手動は直進1.2m/s・旋回4rad/s、自動は直進1.1m/s・旋回3rad/s(ファーム側の車輪速制限に合わせた値)。超える値は丸められます。手動の直進が1.1m/s、旋回が3rad/sを超える値は確認が必要です。広い場所で、電源を切れる状態で試してください。</p>
    <p class="hint" id="speedSaveHint">&nbsp;</p>
  </section>

  <!-- 【2026-10-02追加】docs/mobile_control.mdの記載(restricted_accountも切り替え可)に合わせ、
       Stage2のスマホ版と同じ「待機中の左右振動」ON/OFFを完成版にも追加。 -->
  <!-- 【2026-11-02追加】人の方へ動く動作(接近・見回し)を「指定したときだけ」にするスイッチ。既定はOFF。 -->
  <section>
    <h2>人の方へ動く(近づく)</h2>
    <p class="hint">ONの間だけ、人を見つけると近づきます。OFFのときは、人がいてもその場で待ちます(発話は続きます)。時間割・ビヘイビアツリー・ルールで「接近を許可」にした間は、OFFでも動きます。ロボットを再起動すると、OFFに戻ります。</p>
    <div class="row">
      <button id="interactOnBtn" class="save-btn">ON(動く)</button>
      <button id="interactOffBtn" class="save-btn" style="background:#6b6a64;">OFF(待つ)</button>
    </div>
    <p class="hint" id="interactHint">&nbsp;</p>
  </section>

  <!-- 【2026-11-03追加】人の方を向く(face)・見回す(scan)の独立スイッチ。 -->
  <section data-cap="behavior.face_switch">
    <h2>人の方を向く</h2>
    <p class="hint">人を見つけると、その場で旋回して、人の多い方を向きます(前には進みません)。時間割・ビヘイビアツリー・ルールで「向くを許可」にした間は、OFFでも動きます。ロボットを再起動すると、OFFに戻ります。</p>
    <div class="row">
      <button id="faceOnBtn" class="save-btn">ON(向く)</button>
      <button id="faceOffBtn" class="save-btn" style="background:#6b6a64;">OFF</button>
    </div>
    <p class="hint" id="faceHint">&nbsp;</p>
  </section>

  <section>
    <h2>見回す</h2>
    <p class="hint">人がいないとき、旋回して人を探します。「待機中の左右振動」も、このスイッチに従います。ロボットを再起動すると、OFFに戻ります。</p>
    <div class="row">
      <button id="scanOnBtn" class="save-btn">ON(見回す)</button>
      <button id="scanOffBtn" class="save-btn" style="background:#6b6a64;">OFF</button>
    </div>
    <p class="hint" id="scanHint">&nbsp;</p>
  </section>

  <section>
    <h2>待機中の左右振動</h2>
    <p class="hint">人がまだ見つかっていない間、センサーを使わず左右へ1秒ずつ交互に回転するだけの動きに切り替えます。「見回す」がONのときだけ動きます(ビヘイビアツリー・ルールで「左右に揺れる」を指定した間は、OFFでも動きます)。</p>
    <div class="row">
      <button id="wiggleOnBtn" class="save-btn">ON</button>
      <button id="wiggleOffBtn" class="save-btn" style="background:#6b6a64;">OFF</button>
    </div>
    <p class="hint" id="wiggleHint">&nbsp;</p>
  </section>

  <!-- 【2026-10-03追加】BGM ON/OFF・音量(voice_bridgeの/bgmをダッシュボード経由で操作)。
       bgm_urlが空なら非表示。restrictedアカウントも操作可、guestは状態の閲覧のみ。 -->
  <section id="bgmBlock" style="display:none;">
    <h2>BGM</h2>
    <p class="hint" id="bgmStatus" style="margin-top:0;margin-bottom:10px;">-</p>
    <button id="bgmToggleBtn" class="save-btn" type="button">-</button>
    <div class="row" style="margin-top:10px;">
      <label>通常音量(0〜5) <input id="bgmVolume" type="number" step="0.05" min="0" max="5" inputmode="decimal"></label>
      <label>発話中(0〜5) <input id="bgmDuckVolume" type="number" step="0.05" min="0" max="5" inputmode="decimal"></label>
    </div>
    <button id="bgmSaveBtn" class="save-btn" type="button">音量を保存</button>
    <p class="hint" id="bgmHint">&nbsp;</p>
  </section>
</main>
<script>
__NK_FIELD_JS__
</script>
<script>
  // 401(未ログイン/セッション切れ)を受け取ったら自動でログイン画面へ誘導する。
  (function () {
    const rawFetch = window.fetch.bind(window);
    window.fetch = async (...args) => {
      const res = await rawFetch(...args);
      if (res.status === 401 && location.pathname !== "/login") {
        location.href = "/login";
      }
      return res;
    };
  })();

  const state = { forward: 0, turn: 0, estopEngaged: false };

  function showError(message) {
    let banner = document.getElementById("errorBanner");
    if (!banner) {
      banner = document.createElement("div");
      banner.id = "errorBanner";
      banner.style.cssText = "background:#faece7;color:#993c1d;padding:6px 10px;font-size:13px;text-align:center;";
      document.body.insertBefore(banner, document.body.firstChild);
    }
    banner.textContent = message;
    banner.style.display = message ? "block" : "none";
  }

  // 【2026-10-02追加】読み取り専用(guest)アカウントでは運転コマンドを送らない
  // (送り続けても403が返るだけで、エラーバナーが出続けてしまうため)。
  let readOnlyAccount = false;
  let driveTimer = null;

  async function postJson(path, payload, opts) {
    try {
      const res = await fetch(path, Object.assign({ method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify(payload || {}) }, opts || {}));
      if (!res.ok) {
        let msg = `操作エラー(${res.status})`;
        try { const data = await res.json(); if (data && data.message) msg = data.message; } catch (e2) { /* 本文がJSONでなくても既定メッセージのまま続行 */ }
        showError(msg);
        return false;
      }
      showError("");
      return true;
    } catch (e) {
      showError("通信エラー: サーバーに接続できません");
      return false;
    }
  }

  // 【2026-07-17追加】restricted(restricted_account)アカウントの運転コマンドがTL優先で
  // 破棄された場合、ここだけ専用のバナーで「TLが操作中です」と知らせる
  // (通常のエラーバナーとは別枠、TLの操作が終われば自動的に消える)。
  function showTlActiveBanner(active) {
    let banner = document.getElementById("tlActiveBanner");
    if (active) {
      if (!banner) {
        banner = document.createElement("div");
        banner.id = "tlActiveBanner";
        banner.style.cssText = "background:#fff6df;color:#7a5c00;padding:6px 10px;font-size:13px;text-align:center;";
        document.body.insertBefore(banner, document.body.firstChild);
      }
      banner.textContent = "TLが操作中です";
      banner.style.display = "block";
    } else if (banner) {
      banner.style.display = "none";
    }
  }

  const teleopTabId = (() => {
    try { const a = new Uint8Array(8); crypto.getRandomValues(a); return Array.from(a, (b) => b.toString(16).padStart(2, "0")).join(""); }
    catch (e) { return (Math.random().toString(36).slice(2, 14) || "t"); }
  })();
  let teleopSeq = 0;
  let driveInFlightSince = 0;
  let driveInFlightSeq = 0;
  async function sendDrive() {
    if (readOnlyAccount) return;
    // 【2026-10-03修正】seq/tab_idを付与し、非停止の前回リクエストが未完了なら周期送信を見送る(停止は常に送る)。
    const stop = state.forward === 0 && state.turn === 0;
    if (!stop && driveInFlightSince && (Date.now() - driveInFlightSince) < 1000) return;
    const seq = ++teleopSeq;
    if (stop) { driveInFlightSince = 0; } else { driveInFlightSince = Date.now(); driveInFlightSeq = seq; }
    try {
      const res = await fetch("/api/teleop", {
        method: "POST", headers: {"Content-Type":"application/json"},
        body: JSON.stringify({ forward: state.forward, turn: state.turn, seq, tab_id: teleopTabId }),
        keepalive: true, // pagehide時の停止送信がページ破棄で中断されないようにする
      });
      if (!res.ok) {
        showError(`操作エラー(${res.status})`);
        return;
      }
      const data = await res.json();
      showTlActiveBanner(data.reason === "tl_active");
      if (data.reason === "estop_engaged") {
        showError(data.message || "緊急停止中です。解除してから操作してください。");
      } else if (data.reason !== "tl_active") {
        showError("");
      }
    } catch (e) {
      showError("通信エラー: サーバーに接続できません");
    } finally {
      if (!stop && driveInFlightSeq === seq) driveInFlightSince = 0;
    }
  }

  // 【2026-10-02修正】Pointer Events+setPointerCaptureに統一し、タッチのキャンセル
  // (通知の割り込み・システムジェスチャ等でtouchendが来ない)・画面ロック・アプリ切替・
  // タブ非表示でも必ず停止を送る。以前はtouchend/mouseupしか見ておらず、キャンセルされると
  // state.forward=1のまま100ms毎に前進を送り続けた。
  let activePointerId = null;
  function stopDrive() {
    activePointerId = null;
    state.forward = 0; state.turn = 0;
    document.querySelectorAll(".dpad button.pressed").forEach((b) => b.classList.remove("pressed"));
    sendDrive();
  }
  document.querySelectorAll(".dpad button").forEach((btn) => {
    const fwd = Number(btn.dataset.fwd), turn = Number(btn.dataset.turn);
    btn.addEventListener("pointerdown", (ev) => {
      ev.preventDefault();
      if (readOnlyAccount) return;
      // 既に別の指で操作中に2本目が触れた場合は、安全側として一旦停止する。
      if (activePointerId !== null && activePointerId !== ev.pointerId) { stopDrive(); return; }
      activePointerId = ev.pointerId;
      try { btn.setPointerCapture(ev.pointerId); } catch (e) { /* 取れない場合はpointerleaveで停止する */ }
      state.forward = fwd; state.turn = turn; btn.classList.add("pressed"); sendDrive();
    });
    btn.addEventListener("pointerup", stopDrive);
    btn.addEventListener("pointercancel", stopDrive);
    btn.addEventListener("lostpointercapture", stopDrive);
    btn.addEventListener("pointerleave", (ev) => {
      let captured = false;
      try { captured = btn.hasPointerCapture(ev.pointerId); } catch (e) { captured = false; }
      if (!captured && activePointerId === ev.pointerId) stopDrive();
    });
    btn.addEventListener("touchcancel", stopDrive);
    btn.addEventListener("contextmenu", (ev) => ev.preventDefault());
  });
  window.addEventListener("blur", stopDrive);
  window.addEventListener("pagehide", stopDrive);
  document.addEventListener("visibilitychange", () => { if (document.hidden) stopDrive(); });
  driveTimer = setInterval(sendDrive, 100);
  // alert()もタッチ/キー入力のキャンセルを飲み込むため、表示前に必ず停止する。
  const rawAlert = window.alert.bind(window);
  window.alert = (m) => { stopDrive(); rawAlert(m); };

  const estopBtn = document.getElementById("estopBtn");
  // 【2026-10-02修正】ボタン表示はサーバー側の実際の状態(/api/stateのsoft_estop_engaged)に
  // 同期させる。以前は押した瞬間に表示だけ反転しており、POSTが409(in_use)/403/通信エラーで
  // 失敗しても「停止中」と表示され、再読み込み・他端末での操作とも食い違っていた。
  let estopPending = false;
  let estopIgnoreSyncUntil = 0;
  function renderEstop(engaged) {
    state.estopEngaged = engaged;
    estopBtn.textContent = engaged ? "緊急停止を解除" : "緊急停止(ソフト)";
    estopBtn.classList.toggle("engaged", engaged);
  }
  function syncEstop(s) {
    if (estopPending || Date.now() < estopIgnoreSyncUntil) return;
    if (typeof s.soft_estop_engaged === "boolean") renderEstop(s.soft_estop_engaged);
  }
  function showEstopWarning(message) {
    let banner = document.getElementById("estopWarning");
    if (!banner) {
      banner = document.createElement("div");
      banner.id = "estopWarning";
      banner.style.cssText = "background:#a32d2d;color:#fff;padding:8px 10px;font-size:13px;font-weight:700;text-align:center;";
      document.body.insertBefore(banner, document.body.firstChild);
    }
    banner.textContent = message;
    banner.style.display = message ? "block" : "none";
  }
  async function setEstop(engaged) {
    if (readOnlyAccount) return;
    if (engaged) stopDrive(); // 停止側へ倒す(alert/通信失敗でkeyupを失っても走り続けないように)
    if (estopPending) return; // 二重送信防止
    const prev = state.estopEngaged;
    estopPending = true;
    estopBtn.disabled = true;
    renderEstop(engaged); // 押した直後は即時に表示を変える(失敗したら下で戻す)
    let ok = false;
    try {
      ok = await postJson("/api/soft_estop", { engaged });
    } finally {
      estopPending = false;
      estopBtn.disabled = false;
      estopIgnoreSyncUntil = Date.now() + 1500; // 送信前に始まった古い/api/state応答で表示が戻るのを避ける
    }
    if (ok) { showEstopWarning(""); return; }
    renderEstop(prev);
    if (engaged) {
      showEstopWarning("緊急停止の送信に失敗しました。機体は止まっていない可能性があります。物理の非常停止を使ってください。");
      alert("緊急停止の送信に失敗しました。機体は止まっていない可能性があります。物理の非常停止を使ってください。");
    } else {
      showEstopWarning("緊急停止の解除に失敗しました(緊急停止中のままです)。");
    }
  }
  estopBtn.addEventListener("click", async () => {
    if (state.estopEngaged) {
      if (!confirm("本当に解除しますか？")) return;
      await setEstop(false);
    } else {
      await setEstop(true);
    }
  });
  window.addEventListener("keydown", (e) => {
    if (e.code !== "Space") return;
    const tag = (e.target && e.target.tagName) || "";
    if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
    if (state.estopEngaged) return;
    e.preventDefault();
    setEstop(true);
  });

  const HOLD_REASONS = {
    NEAR_PERSON_HOLD: "人物接近(50cm以内)で自律停止中",
    OUT_OF_BOUNDS_HOLD: "走行可能範囲外のため自律停止中",
    KEEPOUT_ZONE_HOLD: "立入禁止エリアのため自律停止中",
    CAMERA_OBSTACLE_HOLD: "カメラが障害物を検知、自律停止中",
    MAP_NOT_LOADED_HOLD: "地図が読み込まれていません。地図を読み込んでください(自律動作は停止中)",
    // 【2026-10-02追加】これらも自律が止まっている状態なのにバッジが「正常」になっていた。
    MAPPING_ACTIVE_HOLD: "地図作成/探索中のため自律停止中",
    SURVEY_ACTIVE_HOLD: "アンケート回答中のため停止中",
    WAIT_FOR_SENSOR: "センサー待ちのため自律停止中",
  };
  // 【2026-10-06追加】時間帯・状況ルール(方針)による待機/移動。安全装置のホールドではないので
  // 赤ではなく黄色のバッジで表示する(behavior_rules、docs/behavior_rules.md)。
  const POLICY_REASONS = {
    // 【2026-11-05】「向く」機能の通常動作(安全装置のホールドではない)。赤ではなく黄色で表示する。
    FACE_PERSON: "人の方を向いています(その場で旋回)",
    FACE_SWEEP: "人を探して見回しています(その場で旋回)",
    POLICY_STAY: "方針により待機中(その場で待機)",
    POLICY_GO_ENTRANCE: "方針により入口へ移動中",
    POLICY_AT_ENTRANCE: "方針により入口で待機中",
    POLICY_GO_PIN: "方針によりピンへ移動中",
    POLICY_AT_PIN: "方針によりピンで待機中",
    POLICY_PIN_UNAVAILABLE: "方針のピンが使えないため待機中",
    POLICY_NO_SCAN: "方針により見回し停止中(待機)",
  };
  // behavior_directorからの指令が数秒来ていない(落ちている/通信断)場合は、
  // 古いmode/estop_activeを信用せず「無応答」と表示する。
  function isBehaviorStale(s) {
    return s.behavior_age_s === null || s.behavior_age_s === undefined || s.behavior_age_s > 5;
  }
  async function poll() {
    try {
      const res = await fetch("/api/state");
      const s = await res.json();
      syncEstop(s);
      const badge = document.getElementById("estopBadge");
      if (isBehaviorStale(s)) {
        badge.textContent = "behavior_director無応答";
        badge.className = "estop-badge estop-bad";
      } else if (s.estop_active) {
        badge.textContent = "緊急停止中";
        badge.className = "estop-badge estop-bad";
      } else if (HOLD_REASONS[s.mode]) {
        badge.textContent = HOLD_REASONS[s.mode];
        badge.className = "estop-badge estop-bad";
      } else if (POLICY_REASONS[s.mode]) {
        badge.textContent = POLICY_REASONS[s.mode];
        badge.className = "estop-badge estop-policy";
      } else {
        badge.textContent = "正常";
        badge.className = "estop-badge estop-ok";
      }
      const interactHint = document.getElementById("interactHint");
      if (interactHint) {
        interactHint.textContent = s.person_interaction_gate === false
          ? "現在: 常に許可(従来動作)"
          : (s.person_interaction_enabled ? "現在: ON(人の方へ動く)" : "現在: OFF(人がいてもその場で待つ)");
      }
      const scanHint = document.getElementById("scanHint");
      if (scanHint) {
        scanHint.textContent = s.scan_face_supported === false
          ? "未対応(ロボット側が古い)"
          : (s.scan_enabled ? "現在: ON(人がいないとき、旋回して探す)" : "現在: OFF(人がいなくても、その場で待つ)");
      }
      const faceHint = document.getElementById("faceHint");
      if (faceHint) {
        faceHint.textContent = s.scan_face_supported === false
          ? "未対応(ロボット側が古い)"
          : (s.face_enabled
              ? "現在: ON" + (s.face_active ? (s.face_aligned ? "(人の方を向いている)" : "(人の方へ旋回中)") : "(人がいると、その場で向く)")
              : "現在: OFF");
      }
      const wiggleHint = document.getElementById("wiggleHint");
      if (wiggleHint) {
        wiggleHint.textContent = `現在: ${s.idle_wiggle_enabled ? "ON" : "OFF"}`;
      }
    } catch (e) {
      document.getElementById("estopBadge").textContent = "接続断";
    }
  }
  setInterval(poll, 500);
  poll();

  async function loadSpeedSettings() {
    try {
      const res = await fetch("/api/speed_settings");
      const s = await res.json();
      // 入力中・未保存の編集(dirty)・保存確認中の欄は書き換えない(NKField)
      NKField.set("manualLinear", s.manual_linear_mps);
      NKField.set("manualAngular", s.manual_angular_rps);
      NKField.set("autoLinear", s.auto_linear_mps);
      NKField.set("autoAngular", s.auto_angular_rps);
      return s;
    } catch (e) { /* 未取得ならinput空のまま */ return null; }
  }
  const SPEED_KEY_ID = { manual_linear_mps: "manualLinear", manual_angular_rps: "manualAngular", auto_linear_mps: "autoLinear", auto_angular_rps: "autoAngular" };
  NKField.guard(Object.values(SPEED_KEY_ID), { clamp: true });  // 範囲はmin/max属性(サーバーが丸める範囲と同じ)。範囲外は入力時に表示
  async function speedRead() { try { return await (await fetch("/api/speed_settings")).json(); } catch (e) { return null; } }
  // サーバーが上限/下限へ丸めた値か(limitsは丸め後の上限。下限は0.01)
  function speedAccept(key) {
    return (sent, v, srv) => {
      const lim = (srv && srv.limits) || {}, base = key.replace(/_(mps|rps)$/, "");
      const mx = lim[base + "_max"], mn = lim[base + "_min"] !== undefined ? lim[base + "_min"] : 0.01;
      return (typeof mx === "number" && sent > mx && Math.abs(v - mx) < 1e-6) || (sent < mn && Math.abs(v - mn) < 1e-6);
    };
  }
  NKField.poll(loadSpeedSettings, 15000);  // 他の端末で変えた速度も反映する(編集中の欄は上書きしない)
  // 【2026-10-10】手動の高い領域(直進>1.1 / 旋回>3)はサーバーがconfirm_requiredで返す。インラインの確認バーで「上げる」を押すまで保存しない。
  let speedPendingReq = null;
  function speedConfirmHide() { speedPendingReq = null; document.getElementById("speedConfirmBar").style.display = "none"; }
  document.getElementById("speedConfirmNo").addEventListener("click", speedConfirmHide);
  document.getElementById("speedConfirmYes").addEventListener("click", () => { const r = speedPendingReq; speedConfirmHide(); if (r) saveSpeed(r, true); });
  document.getElementById("speedSaveBtn").addEventListener("click", async () => {
    // 【2026-10-02修正】空欄/数値でない欄はリクエストに含めない(以前はNumber("")=0を送り、
    // 最小値0.01m/sに丸めて保存されていた)。
    const req = {};
    for (const [key, id] of [["manual_linear_mps", "manualLinear"], ["manual_angular_rps", "manualAngular"], ["auto_linear_mps", "autoLinear"], ["auto_angular_rps", "autoAngular"]]) {
      const raw = document.getElementById(id).value.trim();
      if (raw === "" || !NKField.isDirty(id)) continue;  // 触った欄だけ送る(触っていない欄は、他の端末で変えられていても上書きしない)
      const v = Number(raw);
      if (Number.isFinite(v)) req[key] = v;
    }
    const hint = document.getElementById("speedSaveHint");
    if (!Object.keys(req).length) { hint.textContent = "保存する値が入力されていません(値を変えてから保存してください)"; return; }
    speedConfirmHide();
    saveSpeed(req, false);
  });
  async function saveSpeed(req, confirmHigh) {
    const hint = document.getElementById("speedSaveHint");
    let ok = false;
    const tx = NKField.begin("speed", Object.keys(req).map((k) => SPEED_KEY_ID[k]));  // 送信中・確認中は定期取得で入力欄を上書きしない
    try {
      const res = await fetch("/api/speed_settings", { method: "POST", headers: {"Content-Type":"application/json"}, body: JSON.stringify(confirmHigh ? Object.assign({ confirm_high: true }, req) : req) });
      let d = {};
      try { d = await res.json(); } catch (e2) { /* 本文なし */ }
      if (res.status === 400 && d.confirm_required) {
        speedPendingReq = req;
        document.getElementById("speedConfirmText").textContent = "手動の速度を高い値に上げます(直進1.1m/s超または旋回3rad/s超)。物理の緊急停止は未配線です。広い場所で、電源を切る手段を手元に置いて確認してください";
        document.getElementById("speedConfirmBar").style.display = "block";
        hint.textContent = " ";
        tx.fail();
        return;
      }
      if (!res.ok) { showError(d.message || `操作エラー(${res.status})`); } else { showError(""); ok = true; }
    } catch (e) { showError("通信エラー: サーバーに接続できません"); }
    if (!ok) { tx.fail(); hint.textContent = "保存できませんでした(入力した値は残しています)"; return; }
    // 保存後にサーバーが実際に適用した値を読み直して入力欄へ反映する
    // (上限を超える値はサーバー側で丸められるため、入力した値のまま残さない)。送った値と違う(丸めでもない)ときは警告して入力値を残す。
    const labels = { manual_linear_mps: "手動直進", manual_angular_rps: "手動旋回", auto_linear_mps: "自動移動直進", auto_angular_rps: "自動移動旋回" };
    const vr = await tx.verify({
      items: Object.keys(req).map((k) => ({ el: SPEED_KEY_ID[k], sent: req[k], label: labels[k], read: (srv) => srv[k], accept: speedAccept(k) })),
      load: speedRead, delays: [0, 800, 2000],
    });
    const s = vr.server;
    if (vr.state === "mismatch") { hint.textContent = "反映されていません: " + vr.rows.filter((x) => x.status === "mismatch").map((x) => `${x.label} 送信 ${x.sent} / サーバー ${x.server}`).join("、"); return; }
    if (vr.state === "unknown") { hint.textContent = "保存を送信しましたが、反映を確認できませんでした(サーバーから値を読めません)"; return; }
    const clamped = [];
    if (s) {
      for (const k of Object.keys(req)) {
        if (s[k] !== undefined && Math.abs(Number(req[k]) - Number(s[k])) > 1e-6) {
          clamped.push(`${labels[k]} ${req[k]}→${s[k]}`);
        }
      }
    }
    if (clamped.length) {
      const lim = s.limits || {};
      hint.textContent = `上限/下限に丸めて保存しました: ${clamped.join("、")}(上限 手動: 直進${lim.manual_linear_max}m/s・旋回${lim.manual_angular_max}rad/s / 自動: 直進${lim.auto_linear_max}m/s・旋回${lim.auto_angular_max}rad/s)`;
      setTimeout(() => { hint.textContent = " "; }, 8000);
    } else {
      hint.textContent = "保存しました";
      setTimeout(() => { hint.textContent = " "; }, 2000);
    }
  }
  loadSpeedSettings();

  document.getElementById("interactOnBtn").addEventListener("click", () => postJson("/api/person_interaction", { enabled: true }));
  document.getElementById("interactOffBtn").addEventListener("click", () => postJson("/api/person_interaction", { enabled: false }));
  document.getElementById("scanOnBtn").addEventListener("click", () => postJson("/api/scan_enabled", { enabled: true }));
  document.getElementById("scanOffBtn").addEventListener("click", () => postJson("/api/scan_enabled", { enabled: false }));
  document.getElementById("faceOnBtn").addEventListener("click", () => postJson("/api/face_enabled", { enabled: true }));
  document.getElementById("faceOffBtn").addEventListener("click", () => postJson("/api/face_enabled", { enabled: false }));
  document.getElementById("wiggleOnBtn").addEventListener("click", () => postJson("/api/idle_wiggle", { enabled: true }));
  document.getElementById("wiggleOffBtn").addEventListener("click", () => postJson("/api/idle_wiggle", { enabled: false }));

  // --- BGM(voice_bridgeの/bgmをダッシュボード経由で操作、2026-10-03追加) ---
  function escapeHtml(value) {
    return String(value).replace(/[&<>"']/g, (ch) => ({
      "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
    }[ch]));
  }
  const bgmEls = {
    block: document.getElementById("bgmBlock"), status: document.getElementById("bgmStatus"),
    toggle: document.getElementById("bgmToggleBtn"), vol: document.getElementById("bgmVolume"),
    duck: document.getElementById("bgmDuckVolume"), save: document.getElementById("bgmSaveBtn"),
    hint: document.getElementById("bgmHint"),
  };
  let bgmEnabled = null; // サーバーから読んだ実際の状態(null=不明/接続不可)

  function bgmFileNames(s) {
    const cur = s.playing || s.current || s.file || s.now_playing;
    if (cur) return [String(cur)];
    return Array.isArray(s.files) ? s.files.map((f) => String((f && f.name) || f)) : [];
  }

  // 状態を読み直して画面へ反映する(楽観更新はしない)。force=trueなら入力中の欄も上書きする。
  async function loadBgm(force) {
    const tk = NKField.ticket("bgm");
    let d = null;
    try {
      const res = await fetch("/api/bgm");
      d = await res.json();
    } catch (e) { d = { ok: false }; }
    if (!NKField.fresh(tk)) return;  // 送信前に始まった古い取得結果は捨てる(画面が一瞬古い値に戻らないように)
    if (d && d.configured === false) { bgmEls.block.style.display = "none"; return; }
    bgmEls.block.style.display = "";
    if (!d || !d.ok || !d.status) {
      bgmEnabled = null;
      bgmEls.status.textContent = "BGM: 接続できません";
      bgmEls.toggle.textContent = "-";
      bgmEls.toggle.disabled = true;
      bgmEls.save.disabled = true;
      return;
    }
    const s = d.status;
    bgmEnabled = !!s.enabled;
    const names = bgmFileNames(s);
    let line = `BGM: <b>${bgmEnabled ? "ON" : "OFF"}</b>`;
    if (s.available === false) line += " / 再生できるBGMファイルがありません";
    else if (s.playing) line += ` / 再生中${names.length ? ": " + names.map(escapeHtml).join("、") : ""}`;
    else line += " / 停止中";
    if (s.ducked) line += "(発話中のため音量を下げています)";
    if (s.stream_ok === false) line += " / 音声出力に失敗しています";
    bgmEls.status.innerHTML = line;
    bgmEls.toggle.textContent = bgmEnabled ? "BGMをOFFにする" : "BGMをONにする";
    bgmEls.toggle.disabled = readOnlyAccount;
    bgmEls.save.disabled = readOnlyAccount;
    // 入力中・未保存の編集(dirty)・保存確認中の欄は書き換えない(NKField)。force=trueのときだけ上書きする
    if (typeof s.volume === "number") { NKField.set(bgmEls.vol, s.volume, { force: !!force }); bgmEls.vol.dataset.orig = String(s.volume); }
    if (typeof s.duck_volume === "number") { NKField.set(bgmEls.duck, s.duck_volume, { force: !!force }); bgmEls.duck.dataset.orig = String(s.duck_volume); }
  }
  async function bgmRead() {
    try { const d = await (await fetch("/api/bgm")).json(); return d && d.ok && d.status ? d.status : null; } catch (e) { return null; }
  }
  NKField.guard([bgmEls.vol, bgmEls.duck]);
  bgmEls.toggle.addEventListener("click", async () => {
    if (bgmEnabled === null) return;
    const want = !bgmEnabled;
    NKField.bump("bgm");
    const ok = await postJson("/api/bgm", { enabled: want });
    bgmEls.hint.textContent = ok ? "" : "操作できませんでした";
    NKField.bump("bgm");
    await loadBgm(false);
    if (ok && bgmEnabled !== want) bgmEls.hint.textContent = "切り替えを送りましたが、まだ反映が確認できません(BGMは" + (bgmEnabled ? "ON" : "OFF") + "のまま)";
  });
  bgmEls.save.addEventListener("click", async () => {
    const req = {}, sentEls = [];
    for (const [key, el] of [["volume", bgmEls.vol], ["duck_volume", bgmEls.duck]]) {
      if (!NKField.isDirty(el)) continue;  // 触った欄だけ送る(触っていない欄は、他の端末で変えられていても上書きしない/1超の値を丸めない)
      const raw = el.value.trim();
      if (raw === "" || (el.dataset.orig !== undefined && Number(raw) === Number(el.dataset.orig))) continue;  // 元の値のままなら送らない
      const v = Number(raw);
      if (Number.isFinite(v)) { req[key] = v; sentEls.push([key, el]); }
    }
    if (!Object.keys(req).length) { bgmEls.hint.textContent = "保存する値が入力されていません(値を変えてから押してください)"; return; }
    const tx = NKField.begin("bgm", sentEls.map((x) => x[1]));
    const ok = await postJson("/api/bgm", req);
    if (!ok) { tx.fail(); bgmEls.hint.textContent = "保存できませんでした(入力した値は残しています)"; return; }
    const r = await tx.verify({ items: sentEls.map(([key, el]) => ({ el: el, sent: req[key], label: key === "volume" ? "通常音量" : "発話中音量", read: (s) => s[key] })), load: bgmRead });
    bgmEls.hint.textContent = r.state === "ok" ? "保存しました" : (r.state === "clamped" ? "保存しました(範囲外の値は丸められました)"
      : (r.state === "mismatch" ? "反映されていません: " + r.rows.filter((x) => x.status === "mismatch").map((x) => x.label + " 送信 " + x.sent + " / サーバー " + x.server).join("、")
        : "送信しましたが、反映を確認できませんでした"));
    await loadBgm(false);
  });
  loadBgm(true);
  NKField.poll(() => loadBgm(false), 5000);

  // --- guest(閲覧専用)アカウント対応: 操作系UIを無効化して見た目上も
  // 「モックアップ・見るだけ」と分かるようにする ---
  // --- restricted(機体操作+緊急停止のみ)アカウント対応: 最高速度設定の
  // セクションだけを無効化する(2026-07-17追加)。 ---
  async function checkAccount() {
    try {
      const res = await fetch("/api/whoami");
      const w = await res.json();
      document.getElementById("accountLabel").textContent = w.account ? `(${w.account})` : "";
      if (w.account === "guest") {
        // 閲覧専用: 運転コマンドの定期送信を止める(403が返り続けるため)。
        readOnlyAccount = true;
        if (driveTimer !== null) { clearInterval(driveTimer); driveTimer = null; }
        document.querySelectorAll("button, input, select").forEach((el) => { el.disabled = true; });
        const banner = document.createElement("div");
        banner.textContent = "閲覧専用(guestアカウント): ここでは操作できません";
        banner.style.cssText = "background:#fff6df;color:#7a5c00;padding:6px 10px;font-size:13px;text-align:center;";
        document.body.insertBefore(banner, document.body.firstChild);
      } else if (w.restricted) {
        // 消す(非表示)。disabledで残すだけだと項目が見えたままなので、
        // 2026-07-17のユーザー指示で完全に隠す形に変更した。
        const section = document.getElementById("speedSection");
        if (section) {
          section.style.display = "none";
        }
        const banner = document.createElement("div");
        banner.textContent = "このアカウントは機体操作・緊急停止のみ利用できます(速度設定は変更できません)";
        banner.style.cssText = "background:#fff6df;color:#7a5c00;padding:6px 10px;font-size:13px;text-align:center;";
        document.body.insertBefore(banner, document.body.firstChild);
      }
    } catch (e) { /* 取得失敗時は通常通り(操作可能のまま)にしておく */ }
  }
  checkAccount();

  // ログアウト: サーバー側セッションを破棄し、専用ログインページへ移動する。
  document.getElementById("logoutBtn").addEventListener("click", async () => {
    try {
      await fetch("/api/logout", { method: "POST" });
    } catch (e) { /* 失敗してもログイン画面へは移動する */ }
    location.href = "/login";
  });
</script>
</body>
</html>
""".replace("__NOKOSAHEN_LOGO__", LOGO_DATA_URI)


class FullDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("full_dashboard")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8090)
        # スマホ操作用ページ(移動制御のみ)のポート。ログインはPC版と共通。
        self.declare_parameter("mobile_port", 8095)
        # 【2026-10-21】ソフト緊急停止の状態の保存先(空=保存しない)。再起動後も、停止中のままにする。docs/code_review_2026-10-21_opus.md
        self.declare_parameter("soft_estop_state_file", "~/.nokosahen_soft_estop.json")
        # 【2026-10-24】手動運転の人避け(既定OFF)の保存先。再起動後も保つ。docs/manual_person_guard.md
        self.declare_parameter("manual_person_guard_state_file", "~/.nokosahen_manual_person_guard.json")
        self.declare_parameter("linear_speed_mps", 0.2)
        self.declare_parameter("angular_speed_rps", 0.8)
        # SLAMで作成中の地図を「地図を保存」ボタンで書き出す先。マシンローカルの
        # 保存先なので、他の~/.nokosahen_*設定と同じ考え方でホーム配下を既定にする。
        self.declare_parameter("maps_dir", "~/.nokosahen_maps")
        # 【2026-08-06追加】Jetson 2台分散構成向け。空文字なら従来通り
        # mapping_mode.launch.py/localization_mode.launch.pyをこのダッシュボード
        # と同じマシンでローカル起動する。分散構成(このダッシュボードは
        # Jetson B=アプリ系)では、Nav2/SLAM/frontier_explorerが同居する
        # Jetson AのSSHホスト名/IPを設定すると、そちらへSSH経由で起動する
        # (要: Jetson B→AへのSSH鍵認証設定)。
        self.declare_parameter("mapping_control_remote_host", "")
        # 操作盤(管理者用)のBasic認証。両方とも空にすれば従来通り認証無しになる。
        self.declare_parameter("auth_username", "TL")
        self.declare_parameter("auth_password", "")
        # 【2026-07-17追加】スマホ版限定の、機体操作+緊急停止だけできる
        # 制限付きアカウント(最高速度設定は変更不可)。空文字なら無効。
        self.declare_parameter("restricted_username", "")
        self.declare_parameter("restricted_password", "")
        # inventory_manager(別プロセス)が書き込むのと同じSQLiteファイルを
        # 読み取り専用で開き、消費推移グラフ(/consumption_graph)に使う
        # (2026-07-22追加)。inventory_manager側のconsumption_db_fileと
        # 同じパスにすること。
        self.declare_parameter("consumption_db_file", "~/.nokosahen_consumption.db")
        # voice_bridge.py(Jetsonのホスト側、別プロセス)が書き込む発話履歴
        # SQLiteを読み取り専用で開く(2026-07-22追加、/utterance_logが使う)。
        # voice_bridge.py側の環境変数VOICE_UTTERANCE_LOG_DBと同じパスにすること。
        self.declare_parameter("utterance_log_db", "~/.nokosahen_utterance_log.db")
        # voice_bridgeが別マシンで動く場合の取得先URL(2026-10-02追加)。
        # 空ならutterance_log_dbのローカルファイルを読む。
        # 【2026-10-05追加】同時1端末制限(同じアカウントを複数の端末/アドレスから同時に操作できなく
        # する機能)の有効/無効。本番(実際の食堂運用)ではTrue、開発・試験中はFalseにする
        # (方針: 試験中は制限しない)。既定は従来どおりTrue。
        self.declare_parameter("enforce_single_controller", True)
        # 【2026-10-13追加】管理画面の「プログラム終了(全停止)」。OS停止(include_poweroff)を含む。docs/system_shutdown.md
        self.declare_parameter("system_shutdown_enabled", True)
        self.declare_parameter("shutdown_script", "~/mirs2603_nokosahen/scripts/shutdown_all.sh")
        self.declare_parameter("utterance_log_url", "")
        # voice_bridgeのBGM API(GET/POST {bgm_url}/bgm)の中継先(2026-10-03追加)。
        # 空ならBGM操作UIは非表示。voice_bridge_url/utterance_log_urlと同じホストにすること。
        self.declare_parameter("bgm_url", "")
        # voice_bridgeの音声「全体ON/OFF」API(GET/POST {voice_master_url}/voice_master)の中継先(2026-10-09追加)。
        # 空ならbgm_urlと同じホストを使う(同じvoice_bridgeなので)。両方空なら全体ON/OFFのUIは非表示。
        self.declare_parameter("voice_master_url", "")
        # 音量(発話・BGM通常・BGM発話中)API(GET/POST {voice_levels_url}/voice_levels)の中継先(2026-10-09追加)。
        # 空ならbgm_urlと同じホストを使う。両方空なら音量カードは非表示。
        self.declare_parameter("voice_levels_url", "")
        # 腹部ディスプレイ(ラズパイ)の管理API(kiosk_admin.py)の中継先(2026-10-09追加)。空ならアンケートカードは非表示。
        self.declare_parameter("survey_admin_url", "http://198.51.100.1:8083")
        # 【2026-10-26追加】頭部ディスプレイを「本体に付けるiPad(Safari)」にする(docs/ipad_display.md)。true: 1台目の中の表示の状態機械
        # (DisplayHub)を使い、iPadは固定URL http://<1台目>:8090/display を開きっぱなしにする。ROSからは /nokosahen/display_mode で切り替える。
        # false: 従来のラズパイ版(survey_admin_urlの kiosk_admin.py へ中継)。切り戻しのときは survey_monitor の survey_url も戻すこと。
        self.declare_parameter("ipad_display_enabled", True)
        self.declare_parameter("display_mode_file", _DISPLAY_MODE_FILE)   # iPad表示の現在のモードの保存先(再起動後も残す)
        self.declare_parameter("display_auto_near_m", 1.5)   # 自動切替で「近くに来た」とみなす距離[m](これより遠い人は無視)
        self.declare_parameter("display_auto_file", "~/.nokosahen_display_auto_face")   # 顔⇄アンケート自動切替のON/OFFの保存先
        # 【2026-10-27追加】iPad表示 /display の「最初に1回だけ」のログイン。アカウント名・パスワードは厨房サイト(web_ui、8080)と同じものを、
        # web_ui_node.py と同じ仕組み(secrets_loader.resolve_password の web_ui / auth_password)で解決する。パスワードはここ・ログ・yamlに書かない
        # (config/secrets.local.yaml の web_ui: auth_password か、環境変数 NOKOSAHEN_WEB_UI_AUTH_PASSWORD)。display_auth_password は一時的な上書き用(通常は空のまま)。
        self.declare_parameter("display_auth_username", "ADMIN_ACCOUNT")
        self.declare_parameter("display_auth_password", "")
        # アンケートの保存先。空なら環境変数 SURVEY_DATA_DIR、それも無ければ ~/survey_data(ラズパイ版と同じ名前)
        self.declare_parameter("survey_data_dir", "")
        # タブレットOCR(tablet_ocr_reader、1台目の8091)の管理API中継先(2026-10-09追加)。画質プロファイル切替・読み取り状況。空ならOCRカードは非表示。
        self.declare_parameter("ocr_admin_url", "http://127.0.0.1:8091")
        # 呼びかけ音声設定(2026-10-06追加): voice_bridgeの設定/実績APIの中継先。空なら設定ページ非表示。
        # voice_bridge_url/utterance_log_url/bgm_urlと同じホストにすること(アドレス変更時は4つ連動して直す)。
        self.declare_parameter("voice_settings_url", "")
        # 発話間隔の設定ファイル(behavior_directorのannounce_settings_fileと同じパスにすること)。
        self.declare_parameter("announce_settings_file", "~/.nokosahen_announce_settings.json")
        # 状況アナウンスの定義表(announce_nodeのannounce_events_fileと同じパスにすること、2026-10-13追加、docs/announcements.md)。
        self.declare_parameter("announce_events_file", "~/.nokosahen_announce_events.json")
        # 文化祭の出店紹介(announce_nodeのpr_intro_fileと同じパスにすること、2026-10-30追加、docs/announcements.md)。
        self.declare_parameter("pr_intro_file", "~/.nokosahen_pr_intro.json")
        # behavior_director.py(別プロセス)が書き込む接客履歴・A/Bモード履歴
        # SQLiteを読み取り専用で開く(2026-07-22追加)。behavior_director側の
        # engagement_db_fileと同じパスにすること。
        self.declare_parameter("engagement_db_file", "~/.nokosahen_engagement.db")
        # person_target_detector(別プロセス)が書き込む人物検知ログを
        # 読み取り専用で開く(2026-07-24追加、/detection_logが使う)。
        self.declare_parameter("detection_db_file", "~/.nokosahen_detection_log.db")
        # 【2026-10-06追加】時間帯・状況ルール(docs/behavior_rules.md)の編集ページ(/behavior_rules)が
        # 読み書きするルールファイル。behavior_directorの`rules_file`と同じパスにすること
        # (nokosahen.yamlの同名キー。既定値も同じ)。時刻の扱い(utc_offset_*)も同じ設定に揃える。
        self.declare_parameter("behavior_rules_file", "~/.nokosahen_behavior_rules.json")
        # 空なら自動探索(config/behavior_rules.example.json)。
        self.declare_parameter("behavior_rules_example_file", "")
        # 【2026-10-09追加】ビヘイビアツリー(/behavior_tree)の木ファイル。behavior_directorの`tree_file`と同じパスにする。
        self.declare_parameter("behavior_tree_file", "~/.nokosahen_behavior_tree.json")
        # 【2026-10-23追加】スケジュール(時間割、docs/schedule.md)の保存先。「実行」で上のbehavior_tree_fileへコンパイルして書く。
        self.declare_parameter("schedule_file", "~/.nokosahen_schedule.json")
        self.declare_parameter("utc_offset_override_enabled", False)
        self.declare_parameter("utc_offset_hours", 9.0)
        # scripts/step_response_test/step_response_test.pyが結果を書き込む
        # ディレクトリを読み取り専用で見に行く(2026-09-24追加、/step_response
        # が使う)。CLI側の既定出力先(--out)と同じパスにすること。
        self.declare_parameter("step_response_results_dir", "~/.nokosahen_step_response_tests")

        self._crowd_state: Dict[str, Any] = {}
        self._behavior_command: Dict[str, Any] = {}
        # behavior_directorの指令を最後に受信した時刻(time.monotonic())。
        # 無応答(落ちている/通信断)をUIバッジに出すために使う(2026-10-02追加)。
        self._behavior_command_rx = 0.0
        self._menu_state: Dict[str, Any] = {}
        self._announce_text = ""
        self._audio_busy = False
        self._odom: Dict[str, float] = {}

        self.create_subscription(String, "/nokosahen/crowd_state", self._on_crowd_state, 10)
        self.create_subscription(String, "/nokosahen/behavior_command", self._on_behavior_command, 10)
        self.create_subscription(String, "/nokosahen/menu_state", self._on_menu_state, 10)
        self.create_subscription(String, "/nokosahen/announce_text", self._on_announce_text, 10)
        self.create_subscription(Bool, "/audio/busy", self._on_audio_busy, 10)
        self.create_subscription(Odometry, DASHBOARD_ODOM_TOPIC, self._on_odom, 10)
        # 【2026-10-27追加】「移動中です。ご注意ください」(iPad画面+状況アナウンスの音)の判定。表示と音の通知だけで、動作には一切影響しない。
        # 判定は moving_notice.py(純Python)。画面は _display_customer、音は /nokosahen/moving_notice を announce_node が読む。
        self._moving_notice = MovingNotice()
        self._moving_result: Dict[str, Any] = {}
        self._moving_result_t = 0.0
        self._moving_pub = self.create_publisher(String, "/nokosahen/moving_notice", 5)
        self.create_timer(0.25, self._moving_tick)

        self.teleop = TeleopController(
            self,
            linear_speed=float(self.get_parameter("linear_speed_mps").value),
            angular_speed=float(self.get_parameter("angular_speed_rps").value),
            soft_estop_state_file=str(self.get_parameter("soft_estop_state_file").value),
            manual_person_guard_state_file=str(self.get_parameter("manual_person_guard_state_file").value),
        )
        self.map_goal = MapGoalController(self)
        self.robot_pose = RobotPoseController(self)
        # 【2026-10-07】到着誤差(ロボット姿勢と目的地の差)の計測に使う。
        self.map_goal.robot_pose = self.robot_pose
        self.wall_escape = WallEscapeController(self)  # 【2026-10-22】「壁から離れる」ボタンと壁際の状態
        self.route_trail = RouteTrailController(self, self.robot_pose)
        self.scan = ScanController(self)
        self.camera = CameraPreviewController(self)
        # 【2026-10-22】後ろ向きカメラ(左後ろ・右後ろ)の映像。閲覧者がいる間だけ購読する
        self.camera_rear = {
            "left": CameraPreviewController(self, "/nokosahen/rear_camera/left_preview"),
            "right": CameraPreviewController(self, "/nokosahen/rear_camera/right_preview"),
        }
        self.checkout_cams = CheckoutCamRegistry(self)
        self.auto_speed = AutoSpeedController(self)
        # 【2026-10-07追加】Nav2の最高速度を実行時に変更(パラメータサービス経由。cmd_velは出さない)。
        self.nav_speed = NavSpeedController(node=self)
        self.mapping = MappingController(
            self, str(self.get_parameter("maps_dir").value), robot_pose=self.robot_pose
        )
        self.map_lifecycle = MapLifecycleController(
            self,
            str(self.get_parameter("maps_dir").value),
            on_stopped=self._on_map_lifecycle_stopped,
            on_started=self._on_map_lifecycle_started,
            remote_ssh_host=str(self.get_parameter("mapping_control_remote_host").value),
        )
        # 【2026-10-07】ロボットに現在位置を教える(AMCLの初期位置・全体位置推定・位置の確かさ)。
        self.localization = LocalizationAssistController(
            self, self.map_lifecycle, map_goal=self.map_goal, robot_pose=self.robot_pose
        )
        # 【2026-10-07】地図ピン(名前付きの場所)。mapping/map_lifecycleと同じストアを共有する。
        self.pins = PinController(
            self,
            get_pin_store(str(self.get_parameter("maps_dir").value)),
            robot_pose=self.robot_pose,
            map_goal=self.map_goal,
            map_lifecycle=self.map_lifecycle,
        )
        self.boundary = BoundaryController(self, "~/.nokosahen_travel_boundary.json")
        self.keepout_zones = KeepOutZoneController(self, "~/.nokosahen_keepout_zones.json")
        self.crowd_background = CrowdBackgroundController(self)
        self.frontier_explorer = FrontierExplorerController(self)
        self.consumption_log = ConsumptionLog(
            str(self.get_parameter("consumption_db_file").value)
        )
        self.utterance_log = UtteranceLogReader(
            str(self.get_parameter("utterance_log_db").value),
            remote_url=str(self.get_parameter("utterance_log_url").value),
        )
        self.engagement_log = EngagementLog(
            str(self.get_parameter("engagement_db_file").value)
        )
        self.detection_log = DetectionLog(
            str(self.get_parameter("detection_db_file").value)
        )
        self.behavior_control = BehaviorDirectorControlController(self)
        self.behavior_rules = BehaviorRulesController(
            self,
            str(self.get_parameter("behavior_rules_file").value),
            example_file=str(self.get_parameter("behavior_rules_example_file").value),
            pin_controller=self.pins,
            utc_offset_hours=(
                float(self.get_parameter("utc_offset_hours").value)
                if bool(self.get_parameter("utc_offset_override_enabled").value)
                else None
            ),
        )
        self.behavior_tree = BehaviorTreeController(
            self,
            str(self.get_parameter("behavior_tree_file").value),
            _TreeString,
            _TreeLazySub,
            pin_controller=self.pins,
            pins_payload_fn=_tree_pins_payload,
        )
        self.schedule = ScheduleController(
            self, str(self.get_parameter("schedule_file").value), self.behavior_tree, _TreeString, _TreeLazySub,
        )
        self.bgm = BgmProxy(str(self.get_parameter("bgm_url").value))
        self.voice_master = VoiceMasterProxy(
            str(self.get_parameter("voice_master_url").value).strip()
            or str(self.get_parameter("bgm_url").value)
        )
        self.voice_levels = VoiceLevelsProxy(
            str(self.get_parameter("voice_levels_url").value).strip()
            or str(self.get_parameter("bgm_url").value)
        )
        self.announce_events = AnnounceEventsController(
            AnnounceEventsStore(str(self.get_parameter("announce_events_file").value)),
            AnnounceBridgeProxy(
                str(self.get_parameter("voice_levels_url").value).strip()
                or str(self.get_parameter("bgm_url").value)
            ),
        )
        self.pr_intro = PrIntroController(
            PrIntroStore(str(self.get_parameter("pr_intro_file").value)),
            AnnounceBridgeProxy(
                str(self.get_parameter("voice_levels_url").value).strip()
                or str(self.get_parameter("bgm_url").value)
            ),
        )
        # 【2026-10-26追加】iPad表示(DisplayHub)。有効なら survey_admin をラズパイへの中継から、1台目の中の DisplayHub へ差し替える
        # (SurveyAdminProxy と同じ呼び口。設定タブのカード・スケジュールの切替・一時的な上書きの復元が、コードを変えずに動く)。
        self.display_hub = None
        if bool(self.get_parameter("ipad_display_enabled").value):
            self.display_hub = DisplayHub(
                mode_file=str(self.get_parameter("display_mode_file").value),
                auto_file=str(self.get_parameter("display_auto_file").value),
                survey_store=SurveyStore(str(self.get_parameter("survey_data_dir").value).strip() or None),
                customer_provider=self._display_customer,
                menu_state_provider=lambda: self._menu_state,
                inventory_sink=self._display_inventory_sink,
                logger=self.get_logger(),
            )
            self.survey_admin = LocalDisplayAdmin(self.display_hub)
            self._display_menu_init_pub = self.create_publisher(String, "/nokosahen/menu_init", 10)
            self._display_sold_pub = self.create_publisher(String, "/nokosahen/sold_total_event", 20)
            self._display_state_pub = self.create_publisher(String, "/nokosahen/display_state", 10)
            self.create_subscription(String, "/nokosahen/display_mode", self._on_display_mode, 10)
            self.create_timer(1.0, self._display_tick)
            self.get_logger().info(
                f"iPad表示を有効にしました(固定URL: http://<このPC>:{int(self.get_parameter('port').value)}/display、"
                f"切替: /nokosahen/display_mode、現在のモード: {self.display_hub.mode})")
        else:
            self.survey_admin = SurveyAdminProxy(str(self.get_parameter("survey_admin_url").value))
        # 【2026-10-23追加】スケジュールによる頭部ディスプレイの画面の切替。directorの /nokosahen/display_request を受け、
        # 既存の切替(survey_admin.set_mode)を、アンケート回答中でないときだけ呼ぶ。docs/schedule.md
        self.schedule_display = DisplayScheduleController(
            self.survey_admin, self, _TreeString, baseline_file=_SCHEDULE_DISPLAY_BASELINE)
        self.schedule.display = self.schedule_display
        self.schedule.map_source = self.map_goal  # 動ける範囲の警告用(地図の範囲・機体の元の範囲)
        self.schedule.boundary_source = self.boundary
        self.ocr_admin = OcrAdminProxy(str(self.get_parameter("ocr_admin_url").value))
        _vs_url = str(self.get_parameter("voice_settings_url").value)
        self.voice_settings = (
            VoiceSettingsController(
                AnnounceSettingsStore(str(self.get_parameter("announce_settings_file").value)),
                VoiceSettingsProxy(_vs_url),
            )
            if _vs_url.strip()
            else None
        )
        # 【2026-09-24追加】走行距離のステップ応答テスト用。run_drive_test()は
        # manual_override経由でNav2/behavior_directorとのcmd_vel競合を回避する
        # ため完成版でも安全に使える(dashboard_common.py PidGainsControllerの
        # docstring参照)。ESP32 PIDゲイン編集UI自体は完成版には追加しない。
        self.pid_gains = PidGainsController(self, teleop=self.teleop)
        # 走行テスト中だけ購読する高頻度odomを、このダッシュボードのstate用odomにも
        # 流す(CLIが/api/stateから読むyaw/x/yを、間引き前の精度にするため)。
        self.pid_gains.add_odom_listener(self._on_odom)
        self.step_response_results = StepResponseResultsController(
            str(self.get_parameter("step_response_results_dir").value)
        )

        # 【2026-10-13追加】operator(弱いアカウント)の枠組み。アカウント定義・許可リスト・一時上書きの台帳はPC版とスマホ版で共有する。
        # 既定ではoperatorは無効(プレースホルダー)。docs/accounts.md
        self.account_store = AccountStore()
        self.ephemeral = EphemeralOverrides(DEFAULT_JOURNAL_FILE)
        # 【2026-10-13追加】プログラム終了。モーター停止(ソフト緊急停止・目的地キャンセル・探索停止)は他を止める前に必ず行う。
        self.system_shutdown = SystemShutdownController(
            script_path=str(self.get_parameter("shutdown_script").value),
            enabled=bool(self.get_parameter("system_shutdown_enabled").value),
            stop_motors=build_motor_stopper(self.teleop, self.map_goal, self.frontier_explorer),
            map_state=build_map_state(self.map_lifecycle, self.mapping),
            nav_active=build_nav_state(self.map_goal),
        )
        handler = make_dashboard_handler(
            html_getter=lambda: HTML.encode("utf-8"),
            state_getter=self._state,
            teleop=self.teleop,
            map_goal=self.map_goal,
            wall_escape=self.wall_escape,
            scan=self.scan,
            camera=self.camera,
            camera_rear=self.camera_rear,
            checkout_cams=self.checkout_cams,
            auto_speed=self.auto_speed,
            nav_speed=self.nav_speed,
            mapping=self.mapping,
            map_lifecycle=self.map_lifecycle,
            localization=self.localization,
            pins=self.pins,
            boundary=self.boundary,
            keepout_zones=self.keepout_zones,
            map_editor_html_getter=build_map_editor_html,
            crowd_background=self.crowd_background,
            frontier_explorer=self.frontier_explorer,
            consumption_log=self.consumption_log,
            consumption_graph_html_getter=build_consumption_graph_html,
            utterance_log=self.utterance_log,
            utterance_log_html_getter=build_utterance_log_html,
            detection_log=self.detection_log,
            detection_log_html_getter=build_detection_log_html,
            pid_gains=self.pid_gains,
            step_response_results=self.step_response_results,
            step_response_html_getter=build_step_response_html,
            engagement_log=self.engagement_log,
            recommendation_effectiveness_html_getter=build_recommendation_effectiveness_html,
            activity_report_html_getter=build_activity_report_html,
            audience_display_html_getter=build_audience_display_html,
            behavior_control=self.behavior_control,
            behavior_rules=self.behavior_rules,
            behavior_rules_html_getter=build_behavior_rules_html,
            behavior_tree=self.behavior_tree,
            behavior_tree_html_getter=build_behavior_tree_html,
            schedule_html_getter=build_schedule_html,
            schedule=self.schedule,
            voice_settings=self.voice_settings,
            voice_settings_html_getter=build_voice_settings_html if self.voice_settings else None,
            bgm=self.bgm,
            voice_master=self.voice_master,
            voice_levels=self.voice_levels,
            announce_events=self.announce_events,
            pr_intro=self.pr_intro,
            announce_settings_html_getter=build_announce_settings_html,
            survey_admin=self.survey_admin,
            display_hub=self.display_hub,
            display_auth_username=str(self.get_parameter("display_auth_username").value),
            display_auth_password=resolve_password(
                self.get_logger(), "web_ui", "auth_password",
                str(self.get_parameter("display_auth_password").value),
                str(self.get_parameter("display_auth_username").value),
            ),
            ocr_admin=self.ocr_admin,
            auth_username=str(self.get_parameter("auth_username").value),
            auth_password=resolve_password(
                self.get_logger(), "full_dashboard", "auth_password",
                str(self.get_parameter("auth_password").value),
                str(self.get_parameter("auth_username").value),
            ),
            # 完成版(実運用のダッシュボード)のみ同時1台制限を適用する。
            enforce_single_controller=resolve_enforce_single_controller(bool(self.get_parameter("enforce_single_controller").value), None, self.get_logger()),
            account_store=self.account_store,
            ephemeral=self.ephemeral,
            system_shutdown=self.system_shutdown,
            startup_reset=True,  # 起動時にoperatorの一時的な変更を元に戻す(voice_bridgeのメモリ上の上書きも破棄)
        )
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self._server = start_dashboard_server(self, host, port, handler)

        mobile_handler = make_mobile_handler(
            html_getter=lambda: MOBILE_HTML.replace("__NK_FIELD_JS__", NK_FIELD_JS).encode("utf-8"),
            state_getter=self._state,
            teleop=self.teleop,
            auto_speed=self.auto_speed,
            auth_username=str(self.get_parameter("auth_username").value),
            auth_password=resolve_password(
                self.get_logger(), "full_dashboard", "auth_password",
                str(self.get_parameter("auth_password").value),
                str(self.get_parameter("auth_username").value),
            ),
            restricted_username=str(self.get_parameter("restricted_username").value),
            account_store=self.account_store,
            ephemeral=self.ephemeral,
            restricted_password=resolve_password(
                self.get_logger(), "full_dashboard", "restricted_password",
                str(self.get_parameter("restricted_password").value),
                str(self.get_parameter("restricted_username").value),
            ),
            enforce_single_controller=resolve_enforce_single_controller(bool(self.get_parameter("enforce_single_controller").value), None, self.get_logger()),
            # 【2026-10-02追加】待機中の左右振動(/api/idle_wiggle)をスマホ版でも使えるようにする
            # (guestは/api/idle_wiggleがPOST扱いのため403で拒否される)。
            behavior_control=self.behavior_control,
            # 【2026-10-03追加】BGM ON/OFF・音量(/api/bgm)。restrictedアカウントも操作可、guestは403。
            bgm=self.bgm,
        )
        mobile_port = int(self.get_parameter("mobile_port").value)
        self._mobile_server = start_dashboard_server(self, host, mobile_port, mobile_handler)

    def destroy_node(self) -> bool:
        self._server.shutdown()
        self._server.server_close()
        self._mobile_server.shutdown()
        self._mobile_server.server_close()
        return super().destroy_node()

    # --- 【2026-10-26追加】iPad表示(docs/ipad_display.md) ---
    def _display_customer(self) -> Dict[str, Any]:
        """お客さん向け画面の中身(今の状態の文言 + 残数)。director の指令・残数管理の状態から作る。"""
        age = (time.monotonic() - self._behavior_command_rx) if self._behavior_command_rx else None
        moving = self._moving_result if (time.monotonic() - self._moving_result_t) <= 1.5 else None   # 判定が止まっていたら「移動中」を出さない
        return build_customer_payload(self._menu_state, self._behavior_command, age, self._announce_text, moving)

    def _moving_tick(self) -> None:
        """4Hzで「移動中」を判定して、画面用に保持し、音用に /nokosahen/moving_notice へ流す。例外が出てもノードを止めない。"""
        try:
            now = time.monotonic()
            hub = self.display_hub
            answering = bool(hub.status_for_ros().get("active")) if hub is not None else False
            # 【2026-10-30】iPad表示が有効なら、表示モードが customer のときだけ出す(画面と音を同じ判定で。無効=旧ラズパイ方式は従来どおり)
            r = self._moving_notice.evaluate(now, survey_answering=answering, speaking=bool(self._audio_busy),
                                             display_enabled=hub is not None, display_mode=(hub.mode if hub is not None else None))
            self._moving_result, self._moving_result_t = r, now
            out = String()
            out.data = dumps({"stamp": time.time(), **r})
            self._moving_pub.publish(out)
        except Exception as exc:  # noqa: BLE001
            self._moving_result = {}
            self.get_logger().warn(f"移動中の判定でエラー(表示と音の通知のみ。動作には影響しません): {exc!r}", throttle_duration_sec=60.0)

    def _display_inventory_sink(self, topic: str, payload: Dict[str, Any]) -> bool:
        """スタッフが入力した残数を、既存の残数管理(inventory_manager)の入力トピックへ流す。"""
        pub = {"/nokosahen/sold_total_event": getattr(self, "_display_sold_pub", None),
               "/nokosahen/menu_init": getattr(self, "_display_menu_init_pub", None)}.get(topic)
        if pub is None:
            return False
        msg = String()
        msg.data = dumps(payload)
        pub.publish(msg)
        return True

    def _on_display_mode(self, msg: String) -> None:
        """ROSからの「これを表示」。"survey" か {"mode":"survey"}。アンケート回答中は保留して、終わってから切り替える。"""
        hub = self.display_hub
        if hub is None:
            return
        r = hub.request_mode(msg.data, "ros", defer=True)
        if not r["ok"]:
            self.get_logger().warn(f"/nokosahen/display_mode の要求を無視しました: {msg.data!r} ({r['message']})")

    def _display_tick(self) -> None:
        hub = self.display_hub
        if hub is None:
            return
        hub.tick()
        # 【2026-11-08】顔⇄アンケートの自動切替用に「人が見えているか」を渡す(crowd_state の person_target が新しいとき=見えている)
        try:
            fresh = (time.monotonic() - getattr(self, "_crowd_state_rx", 0.0)) <= 3.0
            pt = self._crowd_state.get("person_target") if fresh else None
            near = False
            if isinstance(pt, dict):
                try:
                    near = math.hypot(float(pt.get("x", 99.0)), float(pt.get("y", 99.0))) <= float(self.get_parameter("display_auto_near_m").value)
                except (TypeError, ValueError):
                    near = False
            hub.note_presence(near)
        except Exception as exc:  # noqa: BLE001  (自動切替の不具合で表示の更新を止めない)
            self.get_logger().warn(f"自動切替の判定でエラー: {exc!r}", throttle_duration_sec=60.0)
        out = String()
        out.data = dumps(hub.status_for_ros())
        self._display_state_pub.publish(out)

    def _on_crowd_state(self, msg: String) -> None:
        data = loads(msg.data, {})
        if isinstance(data, dict):
            self._crowd_state = data
            self._crowd_state_rx = time.monotonic()

    def _on_behavior_command(self, msg: String) -> None:
        data = loads(msg.data, {})
        if isinstance(data, dict):
            self._behavior_command = data
            self._behavior_command_rx = time.monotonic()
            self._moving_notice.on_behavior(data, self._behavior_command_rx)

    def _on_menu_state(self, msg: String) -> None:
        data = loads(msg.data, {})
        if isinstance(data, dict):
            self._menu_state = data

    def _on_announce_text(self, msg: String) -> None:
        self._announce_text = msg.data

    def _on_audio_busy(self, msg: Bool) -> None:
        self._audio_busy = bool(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        # 【2026-09-24追加】yaw_degはstep_response_test.py CLIの回転(rotate_left/
        # rotate_right)ステップ応答テストが/api/stateから読む(stage1_dashboard_node
        # の_on_odom()と同じ計算方法)。
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom = {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw_deg": math.degrees(yaw),
        }
        try:   # 「移動中」の判定用に、車輪の前後速度(オドメトリのtwist)を渡す。失敗しても本体の処理は続ける
            self._moving_notice.on_odom(msg.twist.twist.linear.x, time.monotonic())
        except Exception:  # noqa: BLE001
            pass

    def _on_map_lifecycle_stopped(self) -> None:
        # 【2026-09-15修正】lambdaでのtuple評価は前者が例外を投げると後者が
        # 実行されないため、try/finallyで両方を確実に呼ぶようにする。
        try:
            self.map_goal.clear()
        finally:
            self.route_trail.clear()

    def _on_map_lifecycle_started(self) -> None:
        try:
            self.mapping.clear_camera_obstacle_points()
        finally:
            self.route_trail.clear()

    def _state(self) -> Dict[str, Any]:
        state = {
            "crowd_level": self._crowd_state.get("level", "unknown"),
            "person_target": self._crowd_state.get("person_target"),
            "mode": self._behavior_command.get("mode"),
            "estop_active": self._behavior_command.get("estop_active", False),
            "manual_override": self._behavior_command.get("manual_override", False),
            "idle_wiggle_enabled": self._behavior_command.get("idle_wiggle_enabled", False),
            # 【2026-11-02追加】人の方へ動く動作のスイッチ(directorの報告値。古いdirectorは未報告=従来どおり常に許可)。
            "person_interaction_enabled": self._behavior_command.get("person_interaction_enabled", False),
            "person_interaction_gate": self._behavior_command.get("person_interaction_gate", False),
            # 【2026-11-03追加】見回す・向くのスイッチ(directorの報告値)。古いdirectorは未報告=False。
            # scan_face_supported は画面が「未対応(ロボット側が古い)」を出すための印(両キーとも報告されたときだけtrue)。
            "scan_enabled": bool(self._behavior_command.get("scan_enabled", False)),
            "face_enabled": bool(self._behavior_command.get("face_enabled", False)),
            "face_active": bool(self._behavior_command.get("face_active", False)),
            "face_aligned": bool(self._behavior_command.get("face_aligned", False)),
            "face_bearing_deg": self._behavior_command.get("face_bearing_deg"),
            "scan_face_supported": ("scan_enabled" in self._behavior_command and "face_enabled" in self._behavior_command),
            # 【2026-10-13追加】推奨発話ON/OFFのdirector側の現在値(読み戻し確認用。古いdirectorは未報告=None)と、
            # ダッシュボードが最後に指定した値(未指定=None)。
            "recommend_enabled": self._behavior_command.get("recommend_enabled"),
            "recommend_updated_at": self._behavior_command.get("recommend_updated_at"),
            "recommend_requested": self.behavior_control.recommend_requested,
            # 【2026-10-06】時間帯・状況ルールの適用状況(behavior_commandのactive_rule/policy)。
            "active_rule": self._behavior_command.get("active_rule"),
            "policy": self._behavior_command.get("policy"),
            # 最後にbehavior_directorの指令を受信してからの経過秒(未受信はNone)。
            "behavior_age_s": (
                time.monotonic() - self._behavior_command_rx
                if self._behavior_command_rx else None
            ),
            "announce_text": self._announce_text,
            "audio_busy": self._audio_busy,
            "odom": self._odom or None,
            "robot_pose": self.robot_pose.get_pose(),
            "menu_state": self._menu_state or None,
        }
        state.update(self.teleop.status())
        state.update(self.route_trail.status())
        return state


def main() -> None:
    rclpy.init()
    node = FullDashboardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
