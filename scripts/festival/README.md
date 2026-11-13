# 文化祭モード(2026-11-07・08)

2026-11-06追加。**PC上で作っただけで、ロボット・2台目には配置していない。実機未検証。**

班員が電源を2個入れる -> 1台目が起動 -> 約2分後にWi-Fi `nokosahen` が出る -> 管理画面(`http://192.0.2.1:8090/`)で
**「人の方を向く」だけをONにして放置**、という運用のための起動構成。発話(出店紹介)は自動で始まる。BGMはOFF。

## 1. 何が起動するか

| 項目 | 文化祭モード | 従来の手動モード(boot_start.sh manual) |
|---|---|---|
| 自律行動(`behavior_director`) | **あり**(ただし下のスイッチはすべてOFF) | なし |
| 近づく・見回す・向く・待機の揺動 | **すべてOFF**(再起動でOFFに戻る)。**「向く」だけ、人が管理画面でONにする**(自動ではONにしない) | -- |
| ホーム帰還 | 「近づく」の後にだけ起きる -> 近づくがOFFなので動かない | -- |
| 探索・Nav2 | 起動しない(人が「地図・走行」で開始するまで動かない) | 同じ |
| アンケート入力中の監視 | **ON**(入力中は走行を完全に止める) | ON |
| 人物検出 | ON | launchの既定(ON) |
| 後ろ向きカメラ | **OFF**(未搭載の構成。付けたら `rear_cameras`、または `--rear-cameras`) | launchの既定(ON) |
| ROS通信 | 有線NIC限定(boot_start.sh と同じ。ハブ接続が前提) | 同じ |
| BGM | **OFF**(2台目に届いた時点でOFFにし、20秒おきに見張る) | 2台目の既定(ON) |
| 出店紹介 | **ON**(出店の説明 -> ロボットの説明を交互に繰り返す) | OFF |
| OCR/Coral | 起動しない(boot_start.sh と同じ) | 同じ |

自律行動の入口は「人の方を向く」スイッチだけで、`behavior_director` のスイッチ(`person_interaction_enabled`・`scan_enabled`・`face_enabled`・
`idle_wiggle_enabled`)は既定がすべてOFFで、このスクリプトはどれもONにしない。呼びかけ(メニュー紹介)は `require_announce_opt_in`(既定ON)で
指示が無い限り出ず、さらに出店紹介が `speech_mode: pr` でメニュー紹介を止める。

## 2. 出店紹介の「30秒ごとに交互」の意味

`config/pr_intro.kousensai.json` と同じ2つの文章を使う(パート1=D4出店(62文字)、パート2=ロボットの特徴(110文字))。
設定は `pr_intro.festival.json`(`interval_sec: 30`、`gap_sec: 30`、`enabled: true`、`speech_mode: pr`)。

- 間隔は**「1つ話し終わってから30秒あけて、次を話し始める」**。これは `pr_intro`/`announce_engine` の仕様(`gap_sec`=パートの再生が終わってからの間隔、`interval_sec`=最後のパートが終わってから最初に戻るまでの間隔)で、「話し始めてから30秒」は選べない。
- 流れ: 出店(再生) -> 30秒 -> ロボット(再生) -> 30秒 -> 出店(再生) -> ... (一巡 = 2つの再生時間 + 60秒)。
- **文章が30秒を超えても重ならない**(再生が終わってから数えるので、長いほど周期が延びるだけ)。いまの文章は、62文字で約10秒、110文字で約20秒の見込み(実機の合成後に確認)。
- 音声は2台目が**最初に1回だけ合成してWAVに保存**し、以後は再生するだけ。2台目が遅れて起動しても、1台目が2分おきに作成を依頼し直すので、自動で追いつく(作り終わるまで、そのパートは飛ばされる)。
- 次のときは流れない(既存の優先順位): 移動中の注意・探索・後退・進めず待機・人への接近・地図作成・手動運転・緊急停止・**アンケート回答中**。解けて5秒後に再開。「向く」(その場の旋回)では止まらない。
- 間隔を変える: `festival_mode.sh enable interval=45`(10〜600秒)、または `festival_start.sh --interval 45`。
- 文章を変える: 管理画面「状況アナウンス」の「文化祭の出店紹介」で編集して保存。ただし、**festival_start.sh を実行するたびに `~/.nokosahen_pr_intro.json` が `pr_intro.festival.json` の内容に置き換わる**(前のファイルは `.bak_before_festival_<日時>` に退避)。管理画面で直した文章を使い続けるなら `--no-pr-intro` を付ける(電源投入のフラグファイルでは使えないので、文章を変えるなら `pr_intro.festival.json` を直す)。

## 3. 地図は自動で読み込まれない(最重要)

**電源投入だけでは、保存済みの地図は読み込まれない。** 「人の方を向く」は地図が読み込まれていないと `MAP_NOT_LOADED_HOLD`(止まって「地図を読み込んでください」)になる。

- Nav2(AMCL・map_server)は、管理画面の「地図・走行」タブで地図を選んで「読み込み」を押したときに初めて `localization_mode.launch.py` として起動する(`MapLifecycleController.load_map`、API `POST /api/mapping/load`)。起動時に自動で読む仕組みは無い。
- 自己位置の初期化もない(AMCLの初期位置パラメータは未設定)。読み込んだあとに「現在位置を指定」で位置を教える必要がある(`docs/mapping_and_calibration.md`)。
- したがって、**起動後に管理画面で ①地図を読み込む ②現在位置を指定する ③「人の方を向く」をONにする**、の3手順が要る(班員の手順書に入れること)。
- 自動化しなかった理由: 読み込みのAPIはログインが要り(パスワードを扱えない)、直接 `localization_mode.launch.py` を起動するとダッシュボードの状態(読み込み中の地図名・ピン・表示の向き)と食い違う。さらに、自己位置は機体を置いた場所に依存し、固定の座標では危険。確実でないので実装していない。
- 地図の保存場所は `~/.nokosahen_maps/`(1台目)。

## 4. 使い方

### 4.1 手動で起動(SSH。まず試すとき)

```
bash ~/mirs2603_nokosahen/scripts/festival/festival_start.sh --dry-run                      # やることの表示だけ(何も変えない)
bash ~/mirs2603_nokosahen/scripts/festival/festival_start.sh                                # 全停止 -> 設定 -> 起動
bash ~/mirs2603_nokosahen/scripts/festival/festival_start.sh --reset-survey-once            # アンケートを1回だけ退避してから起動
```

オプションは `festival_start.sh -h`。全停止は `bash ~/mirs2603_nokosahen/scripts/stop_everything.sh`。

### 4.2 電源投入で自動起動する(オプトイン)

前提: `sudo bash ~/mirs2603_nokosahen/scripts/autostart/install_autostart.sh jetson1`(`nokosahen-robot.service` の登録)が済んでいること(2026-10-19に1台目は登録済み)。
このサービスは変えていない。`boot_start.sh` が、フラグファイル `~/.nokosahen_festival_mode` があるときだけ `festival_start.sh --boot` に切り替える。**フラグが無ければ従来どおり手動モード**。

```
# 有効にする(7日の朝、起動の前に。アンケートを1回だけ退避するなら reset_survey_once を付ける)
bash ~/mirs2603_nokosahen/scripts/festival/festival_mode.sh enable reset_survey_once
bash ~/mirs2603_nokosahen/scripts/festival/festival_mode.sh status
# 無効にする(次の電源投入から手動モードに戻る)
bash ~/mirs2603_nokosahen/scripts/festival/festival_mode.sh disable
```

フラグファイルに書ける語: `reset_survey_once` / `rear_cameras` / `no_person_detector` / `no_dds_wired` / `interval=秒`。

必要なこと(配置のあと): 最新コードの配置(`scripts/pc/deploy_jetson1.sh`。配置だけなら `--no-restart`)。ビルドは不要(シェルスクリプトとJSONだけ)。
配置のとき、手動モードで再起動されて地図が消えるので、再起動したくなければ `--no-restart`。

### 4.3 2台目(音声)

- 2台目の自動起動(`nokosahen-voice.service` -> `start_voice_stack.sh`)の**仕組みはリポジトリにあるが、2台目で登録済みかは未確認**(作業日報の記録では、2026-10-19〜11-05の時点で「2台目の自動起動の登録は未実施」)。2台目の `systemctl is-enabled nokosahen-voice.service` で確認する。
- 登録されていなければ、2台目の電源を入れたあと、**手動で `bash ~/start_voice_stack.sh`**(1台目から `ssh j2 'bash ~/start_voice_stack.sh'`)。登録するなら `sudo bash ~/mirs2603_nokosahen/scripts/autostart/install_autostart.sh jetson2`。
- BGMは2台目の起動のたびにONで始まる。1台目の見張りが、2台目が上がり次第OFFにする(通常20秒以内)。
- 本番の2台目では `voice_bridge_merged_20261030.py` を使っている(作業日報)。出店紹介のAPI(`/announce_prepare`・プレイリスト)に対応した版であること。

## 5. アンケートの1回きりのリセット

- 保存先: 1台目の `~/survey_data/`(`SURVEY_DATA_DIR` で変更されていればそこ)。`responses.jsonl`(正本)・`responses.csv`・`condition.txt`(実験条件)。
- `--reset-survey-once`(またはフラグの `reset_survey_once`)のとき、**削除ではなく移動**する: `responses.*` を `~/survey_data_backup_before_festival_<日時>/` へ移し、`condition.txt` はコピーを退避先に置いたうえで**元の場所に残す**(実験条件が消えると、回答の条件が空になるため)。移したあと、空の状態で回答が始まる。
- マーカー `~/.nokosahen_festival_survey_reset_done`(中身は日時と退避先)があれば、2回目以降は何もしない。**8日の起動では消えない。**
- 管理画面(8090)が動いている間は退避しない(回答の件数がメモリにあるため。全停止のあと、起動前に行う。電源投入のときは何も動いていない)。マーカーも書かない。
- フラグファイルに `reset_survey_once` を入れたまま放置してよい(マーカーがあるので繰り返さない)。やり直したいときだけマーカーを手で消す。
- iPad画面の「今日 N人がこたえてくれました」は、**iPadのブラウザ内に記録した数**で、このリセットでは変わらない(必要ならiPadのSafariでそのサイトのデータを消す)。
- ラズパイ側の古い `~/survey_data/`(旧方式)は触らない。

## 6. 起動後の確認

1. 電源投入の約2分後にWi-Fi `nokosahen`。`http://192.0.2.1:8090/` にログイン。
2. 「地図・走行」で地図を読み込み、現在位置を指定(第3章)。
3. 「人の方へ動く」の各スイッチが **すべてOFF** であること。そのあと「向く」だけをON。
4. アンケート監視が動いていること(アンケートカードの「回答の状態」)。
5. 出店紹介: 「状況アナウンス」の「文化祭の出店紹介」が ON で、音声が「作成済み」。約1分以内に、出店の説明が聞こえる。BGMが鳴っていない。
6. ログ: `~/.nokosahen_festival.log`(この起動の経過)、`~/.nokosahen_boot.log`、`~/.nokosahen_festival_keeper.log`(BGM見張り)、`~/.nokosahen_supervisor_wrapper.log`。
7. 起動前の注意: `~/.nokosahen_behavior_tree.json`・`~/.nokosahen_behavior_rules.json` が1台目にあると、中身が approach/scan/wiggle を許可している場合、**スイッチOFFでも近づく/見回す**ことがある(`docs/behavior_tree.md`)。festival_start.sh は存在を警告する(`~/.nokosahen_festival.log`)。当日は、無いか、`enabled:false` であることを確認する。時間割(スケジュール)の `run_on_startup` も同様。

### 軽量の位置推定モード(任意。既定は使わない)

地図の読み込み時に Nav2の走行系(約120〜150%のCPU)を起動せず、map_server・amcl・pose_relay・motion_arbiter だけにする。その場で向くだけの文化祭向け。走行(目的地・探索・壁際脱出)は使えない。
手順・確認項目・戻し方は `docs/light_localization.md`。**朝に実機で確認してから有効にする。**

- 手動起動: `festival_start.sh --light-localization`(`NOKOSAHEN_LOCALIZATION_LIGHT=1` を出して起動する)
- 電源投入: `festival_mode.sh enable light_localization ...`(フラグファイルに `light_localization` の行)
- 使わない(既定): 何も付けない。`--no-light-localization` でも明示できる。

## 7. トラブル時

| 症状 | 確認・対処 |
|---|---|
| 「地図を読み込んでください」で止まる | 第3章。地図を読み込み、現在位置を指定 |
| 勝手に動いた | 管理画面の緊急停止、または電源を切る。木・ルール・時間割のファイル(第6章7)を確認 |
| 声が出ない | 2台目が起動しているか(`curl http://198.51.100.3:8100/status`)。登録されていなければ手動で `start_voice_stack.sh`。管理画面で音声の全体スイッチ、紹介の「音声: 作成済み」 |
| BGMが鳴る | 2台目が再起動した直後なら20秒以内に止まる。止まらないとき: `bash ~/mirs2603_nokosahen/scripts/festival/festival_voice_keeper.sh --once`。見張りが死んでいれば `festival_start.sh` を再実行 |
| 出店紹介が流れない | 「状況アナウンス」画面の理由表示(アンケート回答中・移動中の注意 など、止める状況)。`ros2 topic echo /nokosahen/announce_state` の `pr_intro` |
| 文化祭モードをやめたい | `festival_mode.sh disable` -> 再起動(または `stop_everything.sh` -> `start_all.sh manual`)。BGMの見張りは、再起動か `kill $(cat ~/.nokosahen_festival_keeper.pid)` で止まる |
| アンケートを元に戻したい | `~/survey_data_backup_before_festival_<日時>/` から `responses.*` を `~/survey_data/` へ戻す(管理画面を止めてから) |

## 8. 安全上の注意

- 物理の緊急停止は未配線(電源を切る)。
- このスクリプトはパスワード・秘密ファイルを読まない。`sudo` も使わない。
- 文化祭モードの電源投入起動は、**自律行動のノードを動かす**(従来の自動起動は動かさなかった)。動きの許可は、管理画面で人が「向く」をONにした時だけ。
