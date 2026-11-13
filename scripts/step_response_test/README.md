# 走行距離のステップ応答テスト

目標走行距離(ステップ入力)を与え、実際の走行距離が時間とともにどう追従
するかを記録・プロットし、目標値に対してオーバーシュートしていないかを
自動判定するテストツール。ゲイン調整(ESP32 PID・速度プロファイル等)の
前後比較に使う。

## 設計方針

- 走行指令自体は新しく作らず、既存の操作盤ダッシュボード
  (`stage1_dashboard_node.py`/`stage2_dashboard_node.py`/
  `full_dashboard_node.py`、いずれも`dashboard_common.py`の
  `PidGainsController.run_drive_test()`)に既に実装済みの走行テスト機能
  (`/api/drive_test`)へ相乗りする。ソフト緊急停止・速度/時間クランプ・
  同時実行防止・Nav2/behavior_directorとの`/cmd_vel`競合回避
  (`/nokosahen/manual_override`)といった既存の安全機構をそのまま再利用する
  ため、このスクリプト自身は新たに`/cmd_vel`をpublishしたりROS 2ノードを
  起動したりしない。
- 記録(時刻・走行距離のサンプリング)は、このスクリプト側が`/api/state`を
  `--rate`Hzで自前でポーリングして行う。
- `--sim`はロボット・ネットワーク・ROS 2無しで動く。1次遅れ/2次系の疑似
  応答をローカルで生成し、判定・プロット処理だけを試せる。
- 判定・指標計算(`analysis.py`)はロボット非依存の純粋関数にしてあり、
  `test_analysis.py`で単体テストできる。

## インストール

```bash
cd scripts/step_response_test
pip3 install --break-system-packages -r requirements.txt   # Raspberry Pi/Jetson等
# または
pip3 install -r requirements.txt                           # 通常のPython環境
```

Linux(Jetson等)で日本語ラベルを文字化けさせたくない場合、CJK対応フォント
を入れておくと良い(無くてもエラーにはならず、`DejaVu Sans`にフォール
バックして英数字以外が四角(豆腐)になるだけ):

```bash
sudo apt install -y fonts-noto-cjk
```

## 使い方

### ロボット無しで判定・プロット処理だけ試す(--sim)

```bash
python3 step_response_test.py --sim --target 1.0
```

オーバーシュートする2次系(既定)とオーバーシュートしない1次遅れの両方を
試せる:

```bash
python3 step_response_test.py --sim --target 1.0 --sim-kind first_order
python3 step_response_test.py --sim --target 1.0 --sim-kind second_order --sim-zeta 0.3
```

### 実機で走らせる

ダッシュボード(stage1/stage2/完成版のいずれか)が起動している状態で:

```bash
python3 step_response_test.py --target 1.0 \
    --api-base http://169.254.135.10:8090 \
    --username TL --password <パスワード(TL)>
```

必ず車輪を浮かせた状態、またはすぐ止められる状態で最初に試すこと
(`docs/field_ssh_operations.md`の運用ルールに準ずる)。

### 主なオプション

| オプション | 既定値 | 説明 |
|---|---|---|
| `--target` | 1.0 | 目標走行距離[m] |
| `--duration` | 8.0 | 記録時間[s](目標到達後の整定を見られる長さ) |
| `--rate` | 10.0 | サンプリング周期[Hz] |
| `--os-threshold` | 5.0 | オーバーシュート判定のしきい値[%] |
| `--tol` | 2.0 | 整定判定の許容幅[%](±tol%) |
| `--out` | `~/.nokosahen_step_response_tests` | 出力ディレクトリ(ダッシュボードの`step_response_results_dir`パラメータと合わせること) |
| `--max-target` | 3.0 | 目標距離の安全上限[m]。超過はエラーで終了 |
| `--sim` | (無し) | ロボットを動かさず疑似応答で試す |
| `--direction` | forward | forward/backward(`--sim`では無視) |
| `--speed` | 0.3 | 実走行時の目安速度[m/s](`--sim`では無視) |
| `--api-base` | `http://127.0.0.1:8090` | ダッシュボードのURL(実走行時のみ) |
| `--username`/`--password` | `TL`/(空) | ダッシュボードのログイン情報(`/api/login`でセッションCookieを取得する。HTTP Basic認証ではない) |

終了コードは判定がOKなら`0`、NGなら`1`(自動テストから使える)。

## 出力

`<出力ディレクトリ>/<実行日時>_<sim または forward/backward>_<目標距離>m/` に
以下を保存する:

- `data.csv` — 時刻[s]・走行距離[m]の生データ
- `result.json` — パラメータ・時系列・指標・判定をまとめたもの(ダッシュ
  ボードが読むのはこのファイル)
- `plot.png` — プロット画像(目標線・許容帯・ピーク点・判定バッジ入り)

`--sim`実行で作った`sample_output/`配下のサンプル一式も参照可能。

## ダッシュボードでの表示

ダッシュボード(stage1/stage2/完成版のいずれか)のトップページから
「ステップ応答テスト結果を見る」を開くか、直接`/step_response`へアクセス
する。一覧から結果を最大2件チェックすると、Canvasに重ねてプロットし、
指標を並べて比較表示する(グラフはCDNに頼らずCanvas 2Dへ自前描画して
いるので、インターネットに繋がらない現場でも動く)。

ダッシュボード側は`~/.nokosahen_step_response_tests`(ROS 2パラメータ
`step_response_results_dir`で変更可)を読みに行くだけの読み取り専用実装
なので、このCLIと同じマシンで実行する必要がある(通常はロボット本体
=Jetson上でCLIとダッシュボードの両方を動かす想定)。

## 単体テスト

```bash
python3 -m pytest test_analysis.py -v
```

ロボット・ネットワーク不要(`analysis.py`は純粋関数のみ)。

## 未実装・要確認事項

- ダッシュボードから直接テストを実行する「実行ボタン」は、安全面の
  リスクがあるため未実装。必要であれば別途確認の上で追加する
  (実装する場合は確認ダイアログ・目標距離の上限チェック・非常停止
  ボタンを必須にする)。現状はこのCLIから実行し、結果だけをダッシュ
  ボードで見る運用。
