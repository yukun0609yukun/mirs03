#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <esp_system.h>
#include <esp_task_wdt.h>

#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>

#include "config.h"

// 通信リンク断からの自己復旧用。連続してこの回数だけpublishに失敗したら
// (エージェント再起動・USB切断等を疑い)自己リセットしてsetup()からやり直す。
// リセット後も/cmd_velが来るまでtargetは0のままなので、突然動き出す心配はない。
#define MAX_CONSECUTIVE_PUBLISH_FAILURES 100
static uint32_t consecutive_publish_failures = 0;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t cmd_sub;
rcl_subscription_t pid_gains_sub;
rcl_publisher_t ticks_pub;
rcl_publisher_t estop_pub;
rcl_publisher_t pid_gains_state_pub;
rcl_publisher_t noise_guard_pub;
rcl_timer_t control_timer;
rclc_executor_t executor;

geometry_msgs__msg__Twist cmd_msg;
std_msgs__msg__Int32MultiArray ticks_msg;
std_msgs__msg__Bool estop_msg;
std_msgs__msg__Float32MultiArray pid_gains_msg;
std_msgs__msg__Float32MultiArray pid_gains_state_msg;
std_msgs__msg__Int32MultiArray noise_guard_msg;

// 実行時に/nokosahen/pid_gains(Float32MultiArray, 12要素)で書き換え可能な
// PIDゲイン。再書き込み不要でHTML側からゲイン調整できるようにするための
// 可変変数(config.hのDEFAULT_*は起動直後の初期値のみ)。
//
// 前進(target_rad_s >= 0)と後退(< 0)で別のゲインを持てるようにしている。
// 実機テストで「後退すると直進からずれる」ことが分かったため、モーター/
// キャタピラの前後非対称性を吸収する目的で追加した(車輪ごとの向きの符号は
// setMotor()のinvertが別途処理するので、ここのfwd/bwdはあくまで
// 「その車輪が前向き回転か後ろ向き回転か」で選ぶ)。
// 並び順: [rkp_fwd,rki_fwd,rkd_fwd, rkp_bwd,rki_bwd,rkd_bwd,
//          lkp_fwd,lki_fwd,lkd_fwd, lkp_bwd,lki_bwd,lkd_bwd]
double rkp_fwd = DEFAULT_RKP_FWD, rki_fwd = DEFAULT_RKI_FWD, rkd_fwd = DEFAULT_RKD_FWD;
double rkp_bwd = DEFAULT_RKP_BWD, rki_bwd = DEFAULT_RKI_BWD, rkd_bwd = DEFAULT_RKD_BWD;
double lkp_fwd = DEFAULT_LKP_FWD, lki_fwd = DEFAULT_LKI_FWD, lkd_fwd = DEFAULT_LKD_FWD;
double lkp_bwd = DEFAULT_LKP_BWD, lki_bwd = DEFAULT_LKI_BWD, lkd_bwd = DEFAULT_LKD_BWD;
float pid_gains_state_data[12] = {
    (float)DEFAULT_RKP_FWD, (float)DEFAULT_RKI_FWD, (float)DEFAULT_RKD_FWD,
    (float)DEFAULT_RKP_BWD, (float)DEFAULT_RKI_BWD, (float)DEFAULT_RKD_BWD,
    (float)DEFAULT_LKP_FWD, (float)DEFAULT_LKI_FWD, (float)DEFAULT_LKD_FWD,
    (float)DEFAULT_LKP_BWD, (float)DEFAULT_LKI_BWD, (float)DEFAULT_LKD_BWD};
uint32_t last_pid_gains_state_publish_ms = 0;
// 前回の制御周期で使った向き(前進/後退)。向きが切り替わった瞬間に積分項が
// 不連続に飛ぶのを避けるため、切り替わりを検知したら積分・前回誤差をリセットする。
bool last_left_was_forward = true;
bool last_right_was_forward = true;

// Debounced state of the emergency-stop (drive main-current cutoff) switch.
// This only reflects hardware state; the ESP32 does not control the cutoff.
bool estop_active = false;
int estop_debounce_counter = 0;
// /estop_stateを最後にpublishした値と時刻(値が変化した時+心拍でのみpublishする)。
bool last_estop_published = false;
uint32_t last_estop_publish_ms = 0;

int32_t ticks_data[2] = {0, 0};

volatile int32_t enc_left = 0;
volatile int32_t enc_right = 0;

int32_t last_left = 0;
int32_t last_right = 0;
uint32_t last_control_ms = 0;
uint32_t last_cmd_ms = 0;

double target_left_rad_s = 0.0;
double target_right_rad_s = 0.0;

// 【2026-07-17追加】/cmd_velから受け取った生の目標値(体座標系、左右輪への
// 混合前)。controlCallback側で台形加減速+臨界減衰フィルタを通してから
// target_left_rad_s/target_right_rad_sへ混合する。
double cmd_v = 0.0;
double cmd_w = 0.0;
// 台形加減速(スルーレート制限)の出力段。臨界減衰フィルタへの入力になる。
double ramp_v_target = 0.0;
double ramp_w_target = 0.0;

// 臨界減衰(ζ=1)の2次系フィルタ状態。
// a = ωn^2 * (target - pos) - 2*ωn*vel という運動方程式を毎周期積分する
// (質点ダンパ系、ζ=1固定なので理論上オーバーシュートしない)。
struct CriticalDampedState {
  double pos;
  double vel;
};
CriticalDampedState cd_v = {0.0, 0.0};
CriticalDampedState cd_w = {0.0, 0.0};

// モーター通電時に配線からノイズが乗る/電源電圧が瞬間的に落ちるなどで
// エンコーダ割り込みが誤発火した場合の被害を抑えるための防御策。
// 1) 1制御周期(20ms)でここまでのカウント変化は物理的にありえない、という
//    上限を超えたらノイズとみなしてクランプする(この値を大きく超える回転
//    速度はこの足回りでは出ない想定)。
// 2) モーター出力(PWM相当値)自体も1周期あたりの変化量に上限を設け、
//    ノイズでPID出力が瞬間的に跳んでもモーターが急反転/急加速しないようにする。
constexpr int32_t MAX_TICK_DELTA_PER_CONTROL = 400;
constexpr double MAX_CMD_STEP_PER_CONTROL = 40.0;
double last_left_cmd_out = 0.0;
double last_right_cmd_out = 0.0;

// 上記2つの安全弁が実際に何回発動したかのカウンタ。/nokosahen/noise_guard_countsへ
// 1秒毎にpublishし、次回の実機テストで「本当にノイズが多発しているか」を
// 定量的に確認できるようにする(診断用途。多ければハード側の配線/電源を疑う根拠になる)。
uint32_t tick_clamp_events = 0;
uint32_t slew_limit_events = 0;
int32_t noise_guard_data[2] = {0, 0};
uint32_t last_noise_guard_publish_ms = 0;

// ハードウェアウォッチドッグ: loop()が一定時間呼ばれなくなったら(シリアル
// 通信の詰まり等でrclc_executor_spin_someがブロックし続けた場合等)、
// ソフトウェア側の安全策(WATCHDOG_TIMEOUTによる自動停止)も道連れで
// 効かなくなってしまう。ハードウェアタイマーで強制リセットすることで、
// 通信状態に関係なく「止まらなくなる」事態を防ぐ最終防衛ラインとする。
constexpr uint32_t HW_WATCHDOG_TIMEOUT_MS = 3000;

struct PidState {
  double integral;
  double last_error;
};

PidState pid_left = {0.0, 0.0};
PidState pid_right = {0.0, 0.0};

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { errorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void errorLoop() {
  while (true) {
    delay(100);
  }
}

// publish失敗(エージェント切断・USB切断等)が連続した場合に自己リセットする。
// 単発の失敗は無視し(RCSOFTCHECKと同じ扱い)、しきい値を超えたときだけ
// esp_restart()でsetup()からやり直す。これにより/estop_stateが永久に
// publishされなくなる状態を自動的に脱する。
void trackPublish(rcl_ret_t rc) {
  if (rc == RCL_RET_OK) {
    consecutive_publish_failures = 0;
    return;
  }
  consecutive_publish_failures++;
  if (consecutive_publish_failures >= MAX_CONSECUTIVE_PUBLISH_FAILURES) {
    esp_restart();
  }
}

void IRAM_ATTR onEncLeftA() {
  bool a = digitalRead(PIN_ENC_A_L);
  bool b = digitalRead(PIN_ENC_B_L);
  int32_t delta = (a == b) ? 1 : -1;
  enc_left += ENC_INVERT_L ? -delta : delta;
}

void IRAM_ATTR onEncRightA() {
  bool a = digitalRead(PIN_ENC_A_R);
  bool b = digitalRead(PIN_ENC_B_R);
  int32_t delta = (a == b) ? 1 : -1;
  enc_right += ENC_INVERT_R ? -delta : delta;
}

void setMotor(int dir_pin, int pwm_channel, double command, bool invert) {
  // NaN/Infは(int)へのキャストが未定義動作になるため、駆動前に弾く。
  // constrain()は比較演算ベースでNaNを素通ししてしまうため防御にならない。
  if (isnan(command) || isinf(command)) {
    command = 0.0;
  }
  if (invert) {
    command = -command;
  }
  bool forward = command >= 0.0;
  int pwm = constrain((int)fabs(command), 0, 255);
  digitalWrite(dir_pin, forward ? HIGH : LOW);
  ledcWrite(pwm_channel, pwm);
}

double updatePid(PidState &state, double target, double measured,
                 double kp, double ki, double kd, double dt) {
  double error = target - measured;
  state.integral += error * dt;
  state.integral = constrain(state.integral, -2.0, 2.0);
  double derivative = (error - state.last_error) / max(dt, 0.001);
  state.last_error = error;
  return kp * error + ki * state.integral + kd * derivative;
}

// 【2026-07-17追加】台形加減速(非対称スルーレート制限)。
// |target|が|current|より大きい(=速度の大きさが増える=加速)ならaccel_rate、
// 小さい(=減速/停止/反転)ならdecel_rateを使う。減速側を強めにすることで
// 「止まれ」に対する反応の速さ(ラジコン感)を保ちつつ、発進は滑らかにする。
double rampToward(double current, double target, double accel_rate,
                   double decel_rate, double dt) {
  double diff = target - current;
  bool speeding_up = fabs(target) > fabs(current);
  double max_step = (speeding_up ? accel_rate : decel_rate) * dt;
  if (max_step < 0.0) {
    max_step = 0.0;
  }
  if (diff > max_step) {
    diff = max_step;
  } else if (diff < -max_step) {
    diff = -max_step;
  }
  return current + diff;
}

// 【2026-07-17追加】臨界減衰(ζ=1)の2次系フィルタを1周期分積分する。
// バネ質点ダンパ系でζ=1(臨界減衰)にすると、理論上オーバーシュートせず
// かつ振動もしない最速の収束になる。rampToward()で作った台形の角を
// なめらかに丸める後段フィルタとして使う。
double stepCriticalDamped(CriticalDampedState &state, double target,
                           double omega_n, double dt) {
  double accel = omega_n * omega_n * (target - state.pos) - 2.0 * omega_n * state.vel;
  state.vel += accel * dt;
  state.pos += state.vel * dt;
  return state.pos;
}

// PIN_ESTOP is wired HIGH when the drive main-current cutoff is engaged.
// GPIO34-39 have no internal pull resistor, so this relies on the external
// pull-down described in config.h; a few consecutive stable samples are
// required before the state is trusted (software debounce).
void updateEstop() {
#ifndef ESTOP_PIN_ENABLED
  // config.hでESTOP_PIN_ENABLEDがコメントアウトされている(=物理スイッチ
  // 未配線でのベンチテスト中)。GPIO34は見ず、常に非作動として扱う。
  estop_active = false;
  estop_debounce_counter = 0;
  return;
#endif
  bool raw = digitalRead(PIN_ESTOP) == HIGH;
  if (raw == estop_active) {
    estop_debounce_counter = 0;
    return;
  }
  estop_debounce_counter++;
  if (estop_debounce_counter >= ESTOP_DEBOUNCE_SAMPLES) {
    estop_active = raw;
    estop_debounce_counter = 0;
    if (estop_active) {
      // Motors cannot physically respond while the cutoff is engaged;
      // reset PID state so it doesn't wind up while targets are ignored.
      pid_left.integral = 0.0;
      pid_left.last_error = 0.0;
      pid_right.integral = 0.0;
      pid_right.last_error = 0.0;
    }
  }
}

void cmdVelCallback(const void *msgin) {
  // 多重防御: estop_active中はここでも0にしておく。安全性がcontrolCallback側の
  // 実行順序だけに依存しないようにするため(将来のリファクタでのリグレッション防止)。
  // 【2026-07-17変更】ここで受け取るのは体座標系のv/ω(左右輪への混合前)の
  // 生の目標値。台形加減速+臨界減衰フィルタ(controlCallback側)を通してから
  // target_left_rad_s/target_right_rad_sへ混合するようになったため、
  // ここでの混合・target_*_rad_sへの直接代入はしない。
  if (estop_active) {
    cmd_v = 0.0;
    cmd_w = 0.0;
    last_cmd_ms = millis();
    return;
  }
  const geometry_msgs__msg__Twist *twist =
      (const geometry_msgs__msg__Twist *)msgin;
  cmd_v = twist->linear.x;
  cmd_w = twist->angular.z;
  last_cmd_ms = millis();
}

// /nokosahen/pid_gains(Float32MultiArray, 12要素: [rkp_fwd,rki_fwd,rkd_fwd,
// rkp_bwd,rki_bwd,rkd_bwd, lkp_fwd,lki_fwd,lkd_fwd, lkp_bwd,lki_bwd,lkd_bwd])
// を受信するたびにゲインを上書きする。再書き込み無しでゲイン調整用HTMLから
// 即座に反映できるようにするため。ゲインが変わった瞬間の積分項/微分項の
// 不連続な飛びを避けるため、積分・前回誤差はリセットする。
void pidGainsCallback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *msg =
      (const std_msgs__msg__Float32MultiArray *)msgin;
  if (msg->data.size < 12) {
    return;
  }
  rkp_fwd = msg->data.data[0];
  rki_fwd = msg->data.data[1];
  rkd_fwd = msg->data.data[2];
  rkp_bwd = msg->data.data[3];
  rki_bwd = msg->data.data[4];
  rkd_bwd = msg->data.data[5];
  lkp_fwd = msg->data.data[6];
  lki_fwd = msg->data.data[7];
  lkd_fwd = msg->data.data[8];
  lkp_bwd = msg->data.data[9];
  lki_bwd = msg->data.data[10];
  lkd_bwd = msg->data.data[11];
  pid_left.integral = 0.0;
  pid_left.last_error = 0.0;
  pid_right.integral = 0.0;
  pid_right.last_error = 0.0;
}

void controlCallback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) {
    return;
  }

  uint32_t now = millis();
  double dt = (now - last_control_ms) / 1000.0;
  if (dt <= 0.0) {
    dt = 0.02;
  }
  // 制御ループが一時的に遅延した場合の異常なdt急増を防ぐ上限クランプ
  // (積分項はconstrainで別途保護されているが、速度推定側の保険として)。
  if (dt > 0.5) {
    dt = 0.5;
  }
  last_control_ms = now;

  updateEstop();

  // 【2026-07-17追加】台形加減速+臨界減衰(ζ=1)フィルタ。estop/通信断時は
  // ランプをバイパスして即座にゼロにし(安全優先、従来通りの即時停止)、
  // 内部状態もリセットして復帰直後に「古い速度へ戻ろうとする」ことが
  // ないようにする。通常時はcmd_v/cmd_w(体座標系)をランプに通してから
  // 左右輪のPID目標値へ混合する。
  if (now - last_cmd_ms >= WATCHDOG_TIMEOUT || estop_active) {
    cmd_v = 0.0;
    cmd_w = 0.0;
    ramp_v_target = 0.0;
    ramp_w_target = 0.0;
    cd_v.pos = 0.0;
    cd_v.vel = 0.0;
    cd_w.pos = 0.0;
    cd_w.vel = 0.0;
    target_left_rad_s = 0.0;
    target_right_rad_s = 0.0;
  } else {
    ramp_v_target = rampToward(ramp_v_target, cmd_v, RAMP_ACCEL_LINEAR_MPS2,
                                RAMP_DECEL_LINEAR_MPS2, dt);
    ramp_w_target = rampToward(ramp_w_target, cmd_w, RAMP_ACCEL_ANGULAR_RPS2,
                                RAMP_DECEL_ANGULAR_RPS2, dt);
    double ramped_v = stepCriticalDamped(cd_v, ramp_v_target, RAMP_OMEGA_N_LINEAR, dt);
    double ramped_w = stepCriticalDamped(cd_w, ramp_w_target, RAMP_OMEGA_N_ANGULAR, dt);
    target_left_rad_s = (ramped_v - (ramped_w * WHEEL_BASE / 2.0)) / WHEEL_RADIUS;
    target_right_rad_s = (ramped_v + (ramped_w * WHEEL_BASE / 2.0)) / WHEEL_RADIUS;
  }

  noInterrupts();
  int32_t left = enc_left;
  int32_t right = enc_right;
  interrupts();

  // int32_tの符号付きオーバーフローは未定義動作のため、差分計算はunsigned
  // 演算で行い意図的にラップさせる(2の補数でのモジュラー演算を保証する)。
  int32_t d_left = (int32_t)((uint32_t)left - (uint32_t)last_left);
  int32_t d_right = (int32_t)((uint32_t)right - (uint32_t)last_right);
  last_left = left;
  last_right = right;

  // ノイズによる誤カウント対策: 1周期でここまでの変化は物理的にありえない
  // という上限でクランプする(本物の高速回転をこの上限より先に頭打ちに
  // してしまう副作用より、ノイズ由来の桁違いな暴走を防ぐ方を優先する)。
  int32_t d_left_clamped = constrain(d_left, -MAX_TICK_DELTA_PER_CONTROL, MAX_TICK_DELTA_PER_CONTROL);
  int32_t d_right_clamped = constrain(d_right, -MAX_TICK_DELTA_PER_CONTROL, MAX_TICK_DELTA_PER_CONTROL);
  if (d_left_clamped != d_left || d_right_clamped != d_right) {
    tick_clamp_events++;
  }
  d_left = d_left_clamped;
  d_right = d_right_clamped;

  double left_rad_s = ((double)d_left / COUNTS_PER_REV) * 2.0 * PI / dt;
  double right_rad_s = ((double)d_right / COUNTS_PER_REV) * 2.0 * PI / dt;

  // 前進(target>=0)か後退(target<0)かで、その車輪のゲインセットを選ぶ。
  // 向きが切り替わった瞬間はゲインが不連続に変わるので、積分/前回誤差の
  // 飛びを避けるためPID状態をリセットする(pidGainsCallbackでのゲイン
  // 変更時と同じ考え方)。
  bool left_is_forward = target_left_rad_s >= 0.0;
  bool right_is_forward = target_right_rad_s >= 0.0;
  if (left_is_forward != last_left_was_forward) {
    pid_left.integral = 0.0;
    pid_left.last_error = 0.0;
    last_left_was_forward = left_is_forward;
  }
  if (right_is_forward != last_right_was_forward) {
    pid_right.integral = 0.0;
    pid_right.last_error = 0.0;
    last_right_was_forward = right_is_forward;
  }
  double left_kp = left_is_forward ? lkp_fwd : lkp_bwd;
  double left_ki = left_is_forward ? lki_fwd : lki_bwd;
  double left_kd = left_is_forward ? lkd_fwd : lkd_bwd;
  double right_kp = right_is_forward ? rkp_fwd : rkp_bwd;
  double right_ki = right_is_forward ? rki_fwd : rki_bwd;
  double right_kd = right_is_forward ? rkd_fwd : rkd_bwd;

  double left_cmd = estop_active ? 0.0 : updatePid(
      pid_left, target_left_rad_s, left_rad_s, left_kp, left_ki, left_kd, dt);
  double right_cmd = estop_active ? 0.0 : updatePid(
      pid_right, target_right_rad_s, right_rad_s, right_kp, right_ki, right_kd, dt);

  // スルーレート制限: PID出力(ノイズで瞬間的に跳ぶ可能性がある)を、
  // 1周期あたりの変化量で制限してからモーターへ渡す。estop中は即座に
  // 0まで制限なしで下げる(安全優先)。
  if (estop_active) {
    last_left_cmd_out = 0.0;
    last_right_cmd_out = 0.0;
  } else {
    double left_cmd_limited = constrain(
        left_cmd, last_left_cmd_out - MAX_CMD_STEP_PER_CONTROL,
        last_left_cmd_out + MAX_CMD_STEP_PER_CONTROL);
    double right_cmd_limited = constrain(
        right_cmd, last_right_cmd_out - MAX_CMD_STEP_PER_CONTROL,
        last_right_cmd_out + MAX_CMD_STEP_PER_CONTROL);
    if (left_cmd_limited != left_cmd || right_cmd_limited != right_cmd) {
      slew_limit_events++;
    }
    left_cmd = left_cmd_limited;
    right_cmd = right_cmd_limited;
    last_left_cmd_out = left_cmd;
    last_right_cmd_out = right_cmd;
  }

  setMotor(PIN_DIR_L, l_Channel, left_cmd, INVERT_L);
  setMotor(PIN_DIR_R, r_Channel, right_cmd, INVERT_R);

  ticks_data[0] = left;
  ticks_data[1] = right;
  trackPublish(rcl_publish(&ticks_pub, &ticks_msg, NULL));

  // 【2026-09-30変更】/estop_stateは以前は制御周期(約50Hz)ごとに毎回publishして
  // いたが、値が変わらないのに購読側(behavior_director等4ノード)が毎回rclpyの
  // 待ち受け処理を回すため、Jetson側のCPUを大きく消費していた(実測: 1ノードの
  // 購読だけで1コアの約11%)。値が変化した周期は即座に(従来と同じ遅延で)、
  // 変化が無い間はESTOP_HEARTBEAT_MSごとにだけpublishする。緊急停止でモーターを
  // 止める処理(estop_active)はこのファーム内で完結しており、ROS側への通知の
  // 頻度には依存しない。
  if (estop_active != last_estop_published ||
      now - last_estop_publish_ms >= ESTOP_HEARTBEAT_MS) {
    estop_msg.data = estop_active;
    trackPublish(rcl_publish(&estop_pub, &estop_msg, NULL));
    last_estop_published = estop_active;
    last_estop_publish_ms = now;
  }

  // ゲイン調整UIが「今設定されている値」を表示できるよう、1秒に1回
  // 現在のゲインをpublishする(毎tick送るのは無駄なのでスロットルする)。
  if (now - last_pid_gains_state_publish_ms >= 1000) {
    last_pid_gains_state_publish_ms = now;
    pid_gains_state_data[0] = (float)rkp_fwd;
    pid_gains_state_data[1] = (float)rki_fwd;
    pid_gains_state_data[2] = (float)rkd_fwd;
    pid_gains_state_data[3] = (float)rkp_bwd;
    pid_gains_state_data[4] = (float)rki_bwd;
    pid_gains_state_data[5] = (float)rkd_bwd;
    pid_gains_state_data[6] = (float)lkp_fwd;
    pid_gains_state_data[7] = (float)lki_fwd;
    pid_gains_state_data[8] = (float)lkd_fwd;
    pid_gains_state_data[9] = (float)lkp_bwd;
    pid_gains_state_data[10] = (float)lki_bwd;
    pid_gains_state_data[11] = (float)lkd_bwd;
    trackPublish(rcl_publish(&pid_gains_state_pub, &pid_gains_state_msg, NULL));
  }

  // ノイズ対策(ティッククランプ/スルーレート制限)が実際に何回発動したかを
  // 1秒に1回publishする(診断用途)。
  if (now - last_noise_guard_publish_ms >= 1000) {
    last_noise_guard_publish_ms = now;
    noise_guard_data[0] = (int32_t)tick_clamp_events;
    noise_guard_data[1] = (int32_t)slew_limit_events;
    trackPublish(rcl_publish(&noise_guard_pub, &noise_guard_msg, NULL));
  }

  // ハードウェアウォッチドッグに「制御ループは生きている」ことを伝える。
  // ここまで到達できているならloop()もrclc_executor_spin_someもブロック
  // していないということなので、この行が実行される限りリセットされない。
  esp_task_wdt_reset();
}

void setupMessages() {
  ticks_msg.data.data = ticks_data;
  ticks_msg.data.size = 2;
  ticks_msg.data.capacity = 2;

  pid_gains_state_msg.data.data = pid_gains_state_data;
  pid_gains_state_msg.data.size = 12;
  pid_gains_state_msg.data.capacity = 12;

  noise_guard_msg.data.data = noise_guard_data;
  noise_guard_msg.data.size = 2;
  noise_guard_msg.data.capacity = 2;

  // 受信用バッファも固定確保しておく(micro-ROS Arduinoの既定アロケータは
  // 動的確保しないため、subscriptionを登録する前に容量を用意しておく必要がある)。
  static float pid_gains_recv_buf[12];
  pid_gains_msg.data.data = pid_gains_recv_buf;
  pid_gains_msg.data.size = 0;
  pid_gains_msg.data.capacity = 12;
}

void setupPins() {
  pinMode(PIN_ENC_A_L, INPUT_PULLUP);
  pinMode(PIN_ENC_B_L, INPUT_PULLUP);
  pinMode(PIN_ENC_A_R, INPUT_PULLUP);
  pinMode(PIN_ENC_B_R, INPUT_PULLUP);

  pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);
  // GPIO34-39 have no internal pull resistor; wiring must supply an
  // external pull-down (see config.h) so this reads LOW when not cut off.
  pinMode(PIN_ESTOP, INPUT);

  ledcSetup(l_Channel, pwmFrequency, pwmResolution);
  ledcSetup(r_Channel, pwmFrequency, pwmResolution);
  ledcAttachPin(PIN_PWM_L, l_Channel);
  ledcAttachPin(PIN_PWM_R, r_Channel);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_L), onEncLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_R), onEncRightA, CHANGE);
}

void setupRos() {
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  // このライブラリにはrmw_uros_options_set_domain_id()が無く(古い/別系統の
  // API)、domain idはrcl_init_options側から設定する必要がある。
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_drive", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
      &pid_gains_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/nokosahen/pid_gains"));

  RCCHECK(rclc_publisher_init_default(
      &ticks_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "/wheel_ticks"));

  RCCHECK(rclc_publisher_init_default(
      &estop_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/estop_state"));

  RCCHECK(rclc_publisher_init_default(
      &pid_gains_state_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/nokosahen/pid_gains_state"));

  RCCHECK(rclc_publisher_init_default(
      &noise_guard_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "/nokosahen/noise_guard_counts"));

  const unsigned int timer_timeout_ms = 20;
  RCCHECK(rclc_timer_init_default(
      &control_timer,
      &support,
      RCL_MS_TO_NS(timer_timeout_ms),
      controlCallback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &cmd_sub, &cmd_msg, &cmdVelCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &pid_gains_sub, &pid_gains_msg, &pidGainsCallback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
}

void setup() {
  Serial.begin(115200);
  delay(500);
  // 導入している生のmicro_ros_arduino(#humble)にはSerial引数を取る
  // set_microros_serial_transports()が無く、既定でSerialを使う
  // set_microros_transports()が正しいAPI(platformio.ini参照)。
  set_microros_transports();
  setupPins();
  setupMessages();

  while (rmw_uros_ping_agent(1000, 3) != RMW_RET_OK) {
    delay(500);
  }
  setupRos();

  // ハードウェアウォッチドッグはsetupRos()完了後(=通常運転に入ってから)
  // 有効化する。エージェント接続待ちの間はここでの遅延は正常なので、
  // その間は監視対象にしない。以後loop()内のesp_task_wdt_reset()が
  // HW_WATCHDOG_TIMEOUT_ms以内に呼ばれ続けないと自動リセットされる。
  esp_task_wdt_init(HW_WATCHDOG_TIMEOUT_MS / 1000, true);
  esp_task_wdt_add(NULL);

  // setupRos()完了直後に基準時刻を取る(初期化に要した時間をdt/ウォッチドッグの
  // 計算に含めないため)。
  last_control_ms = millis();
  last_cmd_ms = millis();
  consecutive_publish_failures = 0;
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
  delay(2);
}

