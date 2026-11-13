# Coral Edge TPUを実際に使えるようにする再挑戦手順

## これまでに分かったこと(2026-09-24時点)

対象のRaspberry Pi(Debian 13/trixie、Python 3.13、aarch64)で確認した内容:

- Coral USBアクセラレータ自体はハードウェアとして正常(`lsusb`でモード切り替え
  `1a6e:089a`→`18d1:9302`まで確認できている)。
- `libedgetpu1-std`(Google公式apt、バージョン16.0、2021年頃のビルドのまま
  更新停止)はインストールでき、共有ライブラリ`libedgetpu.so.1`も存在する。
- `pycoral`・`tflite_runtime`はどちらもPython 3.10以降・最近のOS向けの配布が
  無く、そのままでは使えない。
- 代わりにGoogle公式の後継ライブラリ`ai-edge-litert`(pip)は
  インストールできる(2.1.0/2.2.0で確認)。
- しかし`ai-edge-litert` + `libedgetpu.so.1`を組み合わせてEdge TPUデリゲート付きの
  `Interpreter`を生成しようとすると、**Pythonの例外ではなくプロセスごと
  セグメンテーション違反(SIGSEGV)を起こす**。CPU推論のみ(デリゲート無し)は
  問題なく動く。バージョンを変えても同じ症状。

→ 現状は「ドライバ(2021年ビルド)」と「新しいライブラリ/OS(2026年)」の間の
ABI不一致が濃厚。`tablet_ocr_reader.py`側は既にこのクラッシュを検知して
自動でCPU推論にフォールバックするようになっている(`_probe_edgetpu_delegate`)ので、
**最悪でもTPUが使えないだけでOCR自体は動く**。以下はTPUを実際に使えるように
するための再挑戦手順。上から順に試すことを推奨(簡単・成功率が高そうな順)。

## 使い方

ラズパイに繋がったら、まず現在の状態を再確認:

```bash
ssh pi@<ラズパイのIP>
cd ~/tablet_ocr_reader/coral_tpu_retry   # このフォルダをラズパイへ転送しておくこと
bash check_current_state.sh
```

その後、下の戦略を順番に試す。**それぞれ独立して試せる**ので、うまくいかなければ
次を試すだけでよい。どれか1つでも`PROBE_OK`が出れば、
`tablet_ocr_reader.py`の設定はそのまま(`recognizer: edgetpu`)で
自動的にそちらが使われるようになる(`_probe_edgetpu_delegate`が
成功を検知すれば本採用するため、コード変更は不要)。

## 戦略1: Dockerで「当時のOS」を再現する(最有力候補)

`try_docker_coral.sh` — ラズパイ本体のOS(trixie)はそのままに、Dockerコンテナ
内だけ`libedgetpu`が正式サポートしていた頃のOS(Debian bullseye)を動かし、
その中で公式`pycoral`を使う。USBデバイスをコンテナへパススルーする。
ホストOSを一切変更しないので、失敗しても副作用が無い。Dockerさえ入れれば
確認できる。

## 戦略2: 有志が再ビルドしたlibedgetpuを試す

`try_alt_libedgetpu.sh` — Google公式ビルドの代わりに、コミュニティが今の
glibc/gcc向けに再ビルドしたlibedgetpu(GitHub上に複数存在する)を試す。
公式ビルドとの相性問題が原因なら、これで直る可能性がある。

## 戦略3(最終手段): OSを入れ替える

`libedgetpu`が公式サポートしていたOS世代(Debian bullseye/Ubuntu 20.04相当、
Raspberry Pi OS bullseye)へ入れ替えれば、`pycoral`公式手順がそのまま
使えるはず。ただし他のセットアップ(Python 3.13前提で入れたopencv等)も
やり直しになるため、影響が一番大きい。戦略1・2がどちらもダメだった場合の
最終手段として位置づける。手順は`README.md`末尾に記載。

### 戦略3の詳細手順

1. Raspberry Pi Imagerで「Raspberry Pi OS (Legacy, 64-bit)」相当
   (bullseyeベース)を新しいmicroSDカードに書き込む(今のカードは
   そのまま残しておき、比較・切り戻しできるようにする)。
2. 起動後、SSH有効化・Wi-Fi設定は今回と同じ手順。
3. `install_edgetpu_compiler.sh`と同様の手順でCoral公式apt repoを追加し、
   `sudo apt install python3-pycoral libedgetpu1-std`(公式にサポート
   されている組み合わせなので、今回のようなクラッシュは起きないはず)。
4. `scripts/tablet_ocr_reader/`一式を転送し、`recognizer: edgetpu`のまま
   動作確認。
5. うまくいったら、そのmicroSDカードを本番用として採用する。

デメリット: OpenCV等、今回Python 3.13向けに入れたパッケージをすべて
入れ直す必要があり、影響範囲が一番大きい。戦略1・2がどちらも失敗した
場合の最終手段として位置づける。

## 結果の記録

どれを試して結果がどうだったかは、このREADMEの下に追記していく想定。
(このセッション終了時点では未実施、次回接続時に実施予定)

---

## 実施結果ログ(ここに追記していく)

- 2026-09-24: 戦略1〜3、まだ未実施。
- 2026-09-25: **戦略1(Docker)で成功。** `try_docker_coral.sh`実行で
  `PROBE_OK (Docker + 公式pycoral)`を確認。実際に試して見つかった追加の
  不具合(スクリプトは修正済み):
  1. `sudo`で実行すると`$HOME`が`/root`になりモデルパスが見つからない
     → sudoを使わず`pi`ユーザーを`docker`グループに入れて実行する方式に変更。
  2. `debian:bullseye-slim`がEOL済みで`deb.debian.org`からは404
     → `archive.debian.org`(mainのみ、bullseye-security/updatesは使わない
     シンプルな1行構成)へ切り替え。
  3. Coral公式apt repoの鍵検証(`apt-key`)がgnupgのバージョン不整合を誘発
     → `gnupg`自体を入れず、Coral repoの行だけ`[trusted=yes]`で検証省略に変更
     (HTTPS経由でGoogle公式ドメインから直接取得するため許容)。
  - 次の段階(未着手、着手前にユーザーと相談する): `tablet_ocr_reader.py`本体を
    このDockerコンテナ経由で推論するように作り替える(画像1枚ごとにコンテナを
    起動していては遅すぎるので、コンテナを常駐させて標準入出力かHTTPで
    やり取りする設計にする必要がある)。
