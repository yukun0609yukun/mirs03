"""iPad表示の画面HTML(`/display`)。Pythonの文字列として持つ(dashboard_common.py / schedule_page.py などと同じ流儀)。docs/ipad_display.md。

1枚のページが5つのモード(customer / survey / inventory / ocrtest / face)を持ち、サーバ(1台目)の現在のモードを
`GET /display/api/state` で1秒おきに読んで、中身を切り替える。URLは固定(人がURLを変えない)。

**横向き(landscape)が既定・本命**。本番の12.9インチも、テスト用の11インチも、横向きに付ける。縦向きは「崩れない程度のフォールバック」。

画面サイズへの対応(勝手に判断して切り替える)
  - 基準の長さ `--u` = 画面の短辺の 1/100(横向きなら高さの1/100。12.9横なら約10.2px、11横なら約8.3px)。文字・余白・ボタンはすべて `--u` の倍数。
    12.9 も 11 も縦横比がほぼ同じ(約4:3)なので、拡大縮小しただけの同じ見え方になる(11インチでテストした見え方が本番に近い)。
  - `100dvh` と `env(safe-area-inset-*)` で、アドレスバーやホームバーに隠れない。JSが実寸(innerWidth/Height)を `--app-w/--app-h` に入れる。
  - 向き(縦/横)・機種の区分は JS が `<html data-orient data-size>` に入れる。既定は横向き。`?size=11` / `?size=12.9` で区分を上書きできる。
    縦向きのときは、薄い「横向きにしてください」を出す(押せない・動作は止めない)。
  - customer: 左=今の状態(大きな見出し)、右=残数(品数で列数・文字の大きさを自動調整、はみ出すときは縮める)。
    survey: 左=設問、右=大きな選択肢ボタン(顔の5段階は横いっぱい)。inventory: 左=メニュー一覧、右=テンキー。
    inventory の「メニュー編集」: 左=メニュー一覧(+追加)、右=名前・初期食数の入力(キーボードで隠れないよう上寄せ)。削除は確認ダイアログ。
  - ログイン: 最初に開いたときだけ、ログイン画面(build_display_login_html。状態を含まない)。成功するとCookieを受け取り、以降はこの本体の画面。
    状態の取得が401になったら(ロボットの再起動でセッションが消えたとき)自動で読み込み直してログイン画面に戻る。
  - OCRテスト画面(模擬POS)だけは、`--u` ではなく**実物の長さ(mm)**で描く(カメラで撮って数字を読むため)。縦向きの7インチ端末を、
    横向きのiPadの中央に、縦向きのまま同じ実寸で置く。計算は display_sizes.py と docs/ipad_display.md。式は `NKSIZE-BEGIN`〜`NKSIZE-END` の中にあり、
    test_ipad_display.py が node で動かして Python と照合する。
  - 顔(face。2026-11-08): ユーザーが用意したロボットの顔のHTML(face_html.py。一字も変えていない)を、`<iframe sandbox="allow-scripts" srcdoc>` に入れて
    画面いっぱいに出す。iframe に隔離するので、このページの id・グローバル変数・CSS(`html,body{overflow:hidden}` など)・ポーリングと衝突しない。
    iframe は face モードに入ったときだけ作り、出るときに取り除く(顔の requestAnimationFrame / setTimeout も一緒に止まる)。
    状態のポーリング・ログイン切れで読み込み直す処理・接続状態の表示・画面ロック防止は、親のページがそのまま続ける。
"""
from __future__ import annotations

import functools
import hashlib
import json

from .assets import LOGO_DATA_URI
from .display_survey import SURVEY_DEF
from .face_html import FACE_HTML

_TEMPLATE = r"""<!doctype html>
<html lang="ja" data-size="l" data-orient="landscape">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,minimum-scale=1,maximum-scale=1,viewport-fit=cover,user-scalable=no">
<!-- ホーム画面に追加して起動すると、Safariのバーが無い全画面表示になる(キオスクの代わり) -->
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="のこさへん">
<meta name="format-detection" content="telephone=no">
<meta name="theme-color" content="#0f2e24">
<link rel="manifest" href="/display/manifest.webmanifest">
<link rel="apple-touch-icon" href="/display/icon.png">
<title>のこさへん 頭部ディスプレイ</title>
<style>
:root{
  --app-w:100vw; --app-h:100vh;
  /* 基準の長さ: 画面の短辺の1/100。横向きなら高さの1/100(12.9横 1366x1024 で10.24px、11横 1194x834 で8.34px、1180x820 で8.20px)。
     12.9 と 11 は縦横比がほぼ同じ(約4:3)なので、u の倍数で組めば拡大縮小しただけの同じ見え方になる */
  --u:calc(min(var(--app-w),var(--app-h)) / 100);
  --sat:env(safe-area-inset-top,0px); --sar:env(safe-area-inset-right,0px);
  --sab:env(safe-area-inset-bottom,0px); --sal:env(safe-area-inset-left,0px);
  --f:"Hiragino Sans","Hiragino Kaku Gothic ProN","Noto Sans CJK JP","Noto Sans JP","Yu Gothic UI",system-ui,-apple-system,sans-serif;
  --mono:ui-monospace,"SF Mono",Menlo,"DejaVu Sans Mono",monospace;
  --pad:calc(var(--u) * 3); --gap:calc(var(--u) * 2); --r:calc(var(--u) * 2);
  --t-s:max(13px,calc(var(--u) * 1.6)); --t-m:max(15px,calc(var(--u) * 2.3)); --t-l:calc(var(--u) * 3.2);
  --t-xl:calc(var(--u) * 5.2); --t-hero:calc(var(--u) * 8);
  --tap:max(56px,calc(var(--u) * 7));
  --g0:#0f2e24; --g1:#124739; --g2:#176b56; --ink:#16211d; --muted:#53635c;
  --bg:#eef2f4; --surface:#fff; --line:#c9d3d8; --accent:#0d6b58; --accent-soft:#d5ece5; --hi:#f2b43c; --warn:#b4432d;
  --cols:1; --k:1; --numsz:calc(var(--u) * 14 * var(--k)); --nmsz:calc(var(--u) * 3.4 * var(--k));
}
@supports (height:100dvh){ :root{ --app-h:100dvh; } }
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
[hidden]{display:none!important}
html,body{margin:0;height:100%;overflow:hidden;overscroll-behavior:none;-webkit-text-size-adjust:100%}
body{position:fixed;inset:0;font-family:var(--f);font-weight:500;color:var(--ink);background:var(--g0);
  -webkit-user-select:none;user-select:none;-webkit-touch-callout:none;touch-action:manipulation}
button{font:inherit;color:inherit;border:0;background:none;cursor:pointer}
button:focus-visible,input:focus-visible{outline:4px solid var(--hi);outline-offset:3px}
.view{position:fixed;inset:0;display:none;flex-direction:column;
  padding:calc(var(--pad) + var(--sat)) calc(var(--pad) + var(--sar)) calc(var(--pad) + var(--sab)) calc(var(--pad) + var(--sal))}
body[data-mode=customer] #v-customer,body[data-mode=survey] #v-survey,
body[data-mode=inventory] #v-inventory,body[data-mode=ocrtest] #v-ocr,body[data-mode=loading] #v-loading,body[data-mode=face] #v-face{display:flex}
body[data-mode=customer] #v-customer{display:grid}
body[data-mode=survey],body[data-mode=inventory]{background:var(--bg)}
body[data-mode=ocrtest]{background:#17181a}
/* ---- face(顔。元のHTMLをiframeに隔離して画面いっぱいに出す。親の余白は付けない) ---- */
body[data-mode=face]{background:#fff}
#v-face{padding:0;background:#fff}
#v-face iframe{position:absolute;inset:0;width:100%;height:100%;border:0;display:block;background:#fff}

/* ---- 接続状態(右下。つながらないときだけ) ---- */
#conn{position:fixed;right:calc(var(--u) * 2 + var(--sar));bottom:calc(var(--u) * 1.5 + var(--sab));z-index:50;
  background:#7a2d1a;color:#fff;border-radius:99px;padding:.35em .9em;font-size:var(--t-s);font-weight:700;opacity:.92}
body[data-mode=ocrtest] #conn{display:none}
/* 縦向きのときだけ出す薄い案内(押せない・動作は止めない。OCRテストの撮影には写さない) */
#rotate{position:fixed;left:50%;top:calc(var(--u) * .6 + var(--sat));transform:translateX(-50%);z-index:55;pointer-events:none;
  background:rgba(20,30,26,.78);color:#fff;border-radius:99px;padding:.25em 1em;font-size:max(13px,calc(var(--u) * 1.8));font-weight:700;white-space:nowrap}
body[data-mode=ocrtest] #rotate{display:none}
body[data-mode=face] #rotate{display:none}
/* ?debug=1 のとき、どのモードでも左上に判定結果を出す */
#dbg{position:fixed;left:8px;top:8px;z-index:90;pointer-events:none;color:#9fe;background:rgba(0,0,0,.72);padding:4px 8px;border-radius:6px;
  font:12px/1.4 var(--mono);white-space:pre}

/* ---- 読み込み中 ---- */
#v-loading{align-items:center;justify-content:center;color:#cfe5dc;font-size:var(--t-l);gap:var(--gap)}
#v-loading img{width:calc(var(--u) * 14);height:calc(var(--u) * 14);border-radius:var(--r)}

/* ---- customer(お客さん向け。暗い緑)。横向き: 左=今の状態、右=残数リスト ---- */
#v-customer{background:var(--g0);color:#fff;column-gap:calc(var(--u) * 3);
  grid-template-columns:minmax(0,5fr) minmax(0,7fr);grid-template-rows:auto minmax(0,1fr) auto auto;
  grid-template-areas:"bar cards" "status cards" "promote cards" "announce cards"}
.cbar{grid-area:bar;display:flex;align-items:center;gap:var(--gap);margin-bottom:var(--gap)}
.cbar img{width:calc(var(--u) * 6.5);height:calc(var(--u) * 6.5);border-radius:calc(var(--u) * 1.4)}
.cbar h1{margin:0;font-size:calc(var(--u) * 4.6);font-weight:800}
.cbar .clock{margin-left:auto;font-size:calc(var(--u) * 3.4);font-variant-numeric:tabular-nums;opacity:.85}
.cstatus{grid-area:status;min-height:0;overflow:hidden;display:flex;flex-direction:column;justify-content:center;align-items:flex-start;gap:calc(var(--u) * 2.4);
  padding:calc(var(--u) * 4);border-radius:calc(var(--u) * 2.5);background:var(--g1)}
.cstatus .ic{flex:none;width:calc(var(--u) * 15);height:calc(var(--u) * 15);border-radius:50%;display:grid;place-items:center;background:#fff;color:var(--g2)}
.cstatus .ic svg{width:60%;height:60%}
.cstatus .tt{font-size:calc(var(--u) * 6.4);font-weight:900;line-height:1.2;text-wrap:balance}
.cstatus .dd{margin-top:calc(var(--u) * 1.2);font-size:calc(var(--u) * 3.6);opacity:.9;line-height:1.35}
.cstatus[data-tone=warn]{background:#6b2b17}.cstatus[data-tone=warn] .ic{color:#b4432d}
.cstatus[data-tone=info]{background:#1c4d63}.cstatus[data-tone=info] .ic{color:#1c4d63}
/* 移動中の注意(caution=琥珀色)。数m離れても読める: 明るい琥珀の地+黒に近い大きな文字+上下の斜めの警戒帯+太い枠。切替は点滅させない(ゆっくり脈打つ枠だけ) */
.cstatus[data-tone=caution]{position:relative;background:#ffb400;color:#1b1300;border:calc(var(--u) * 1) solid #1b1300;
  background-image:repeating-linear-gradient(-45deg,#1b1300 0 calc(var(--u) * 2),#ffb400 calc(var(--u) * 2) calc(var(--u) * 4));
  background-size:100% calc(var(--u) * 2.6);background-repeat:no-repeat;background-position:0 0}
.cstatus[data-tone=caution]::after{content:"";position:absolute;left:0;right:0;bottom:0;height:calc(var(--u) * 2.6);
  background:repeating-linear-gradient(-45deg,#1b1300 0 calc(var(--u) * 2),#ffb400 calc(var(--u) * 2) calc(var(--u) * 4))}
.cstatus[data-tone=caution] .ic{color:#c2410c;background:#fff;width:calc(var(--u) * 19);height:calc(var(--u) * 19)}
.cstatus[data-tone=caution]{padding-left:calc(var(--u) * 2);padding-right:calc(var(--u) * 2)}
.cstatus[data-tone=caution] .tt{font-size:calc(var(--u) * 6.2);line-height:1.2;color:#1b1300;white-space:pre-line}
.cstatus[data-tone=caution] .dd{color:#1b1300;opacity:1}
@keyframes nkpulse{0%,100%{box-shadow:0 0 0 0 rgba(255,180,0,.0)}50%{box-shadow:0 0 0 calc(var(--u) * 1.2) rgba(255,180,0,.55)}}
.cstatus[data-tone=caution]{animation:nkpulse 2.4s ease-in-out infinite}
@media (prefers-reduced-motion:reduce){.cstatus[data-tone=caution]{animation:none}}
#promote{grid-area:promote;margin-top:var(--gap);padding:calc(var(--u) * 1.6) calc(var(--u) * 2.4);border-radius:calc(var(--u) * 2);background:#ffce54;color:#4a2e00;
  font-size:calc(var(--u) * 2.8);font-weight:800;text-align:left;line-height:1.35}
#cards{grid-area:cards;min-height:0;display:grid;grid-template-columns:repeat(var(--cols),minmax(0,1fr));grid-auto-rows:minmax(0,1fr);gap:var(--gap)}
.card{background:#fff;color:var(--ink);border-radius:var(--r);padding:calc(var(--u) * 1.8) calc(var(--u) * 2.4);
  display:flex;flex-direction:column;min-height:0;min-width:0;overflow:hidden;box-shadow:0 2px 8px rgba(0,0,0,.18)}
/* 縦中央寄せ。はみ出すときは下へはみ出す(justify-content:center だと上が切れて品名が見えなくなる) */
.card>:first-child{margin-top:auto}.card>:last-child{margin-bottom:auto}
.card .nm{font-size:var(--nmsz);font-weight:800;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.card .rm{font-size:var(--numsz);font-weight:900;line-height:1.05;color:var(--g2);font-variant-numeric:tabular-nums}
.card .rm small{font-size:max(12px,calc(var(--numsz) * .3));font-weight:600;margin-left:.3em;color:var(--ink)}
.card .bar{margin-top:calc(var(--u) * 1);height:calc(var(--u) * 1.2);border-radius:99px;background:#e4ebe7;overflow:hidden}
.card .bar i{display:block;height:100%;border-radius:99px;background:var(--g2)}
.card .lb{margin-top:calc(var(--u) * .6);font-size:var(--t-m);font-weight:800;min-height:1.2em}
.card.low .rm,.card.low .lb{color:#c07a00}.card.low .bar i{background:#e0a418}
.card.out .rm,.card.out .lb{color:#9a3f1d}.card.out .bar i{background:#9a3f1d}.card.out{opacity:.88}
/* 横向きの並べ方(JS が #cards[data-lay] に入れる): h=品名と数字を横に並べる(品数が少ない・1列)、v=品名の下に数字(2〜3列)、v2=v で品名を2行まで */
#cards[data-lay=h] .card{display:grid;grid-template-columns:minmax(0,1fr) auto;grid-template-areas:"nm rm" "bar rm" "lb rm";align-content:center;column-gap:calc(var(--u) * 2)}
#cards[data-lay=h] .card>:first-child,#cards[data-lay=h] .card>:last-child{margin:0}
#cards[data-lay=h] .nm{grid-area:nm;white-space:normal;line-height:1.15;overflow-wrap:anywhere}
#cards[data-lay=h] .rm{grid-area:rm;align-self:center}#cards[data-lay=h] .bar{grid-area:bar}
#cards[data-lay=h] .lb{grid-area:lb;min-height:0;margin-top:2px}#cards[data-lay=h] .lb:empty{display:none}
#cards[data-lay=v] .nm,#cards[data-lay=v2] .nm{white-space:normal;line-height:1.15;overflow-wrap:anywhere;max-height:2.35em}
#cards[data-lay=v] .card,#cards[data-lay=v2] .card{padding:calc(var(--u) * 1.4) calc(var(--u) * 2)}
#cards[data-lay=v2] .lb{min-height:0;margin-top:0;font-size:max(13px,calc(var(--u) * 1.9))}#cards[data-lay=v2] .lb:empty{display:none}
#cempty{grid-column:1/-1;align-self:center;text-align:center;font-size:var(--t-xl);opacity:.85}
#announce{grid-area:announce;margin-top:calc(var(--u) * 1.2);font-size:var(--t-m);opacity:.8;text-align:left}

/* ---- survey(アンケート。明るい色。食堂の明るい照明下で読む)。横向き: 左=設問、右=選択肢(大きなボタン) ---- */
#v-survey{padding:0;background:var(--bg);color:var(--ink)}
#v-survey .screen{position:absolute;inset:0;display:flex;flex-direction:column;
  padding:calc(var(--pad) + var(--sat)) calc(var(--pad) + var(--sar)) calc(var(--pad) + var(--sab)) calc(var(--pad) + var(--sal))}
#v-survey #idle{justify-content:center;cursor:pointer;display:grid;grid-template-columns:minmax(0,3fr) minmax(0,2fr);grid-template-rows:auto auto auto auto;
  grid-template-areas:"tag go" "h go" "note go" "cnt go";align-content:center;column-gap:calc(var(--u) * 5);row-gap:calc(var(--u) * 3)}
#idle .tag{grid-area:tag;font-family:var(--mono);font-size:var(--t-s);letter-spacing:.12em;color:var(--muted)}
#idle h1{grid-area:h;margin:0;font-weight:900;letter-spacing:-.01em;font-size:calc(var(--u) * 8.2);line-height:1.2;white-space:pre-line}
#idle .note{grid-area:note;font-size:calc(var(--u) * 3);color:var(--muted)}
#idle .idlecount{grid-area:cnt;justify-self:start;background:var(--accent-soft);color:var(--accent);font-weight:900;font-size:calc(var(--u) * 2.8);padding:calc(var(--u) * .8) calc(var(--u) * 2.4);border-radius:999px}
#idle .go{grid-area:go;align-self:center;justify-self:center;display:flex;flex-direction:column;align-items:center;justify-content:center;text-align:center;gap:calc(var(--u) * 2);
  background:var(--accent);color:#fff;font-weight:900;font-size:calc(var(--u) * 3.6);line-height:1.3;
  width:calc(var(--u) * 36);height:calc(var(--u) * 36);border-radius:50%;animation:pulse 2.4s ease-in-out infinite}
#idle .go b{display:grid;place-items:center;width:calc(var(--u) * 9);height:calc(var(--u) * 9);border-radius:50%;background:#fff;color:var(--accent);font-size:calc(var(--u) * 5)}
@keyframes pulse{0%,100%{box-shadow:0 0 0 0 rgba(13,107,88,.45)}50%{box-shadow:0 0 0 calc(var(--u) * 1.8) rgba(13,107,88,0)}}
.stop{display:flex;align-items:center;gap:calc(var(--u) * 2);min-height:calc(var(--u) * 5)}
.dots{display:flex;gap:calc(var(--u) * .8)}
.dots i{width:calc(var(--u) * 5.6);height:calc(var(--u) * .9);min-height:8px;border-radius:99px;background:var(--line)}
.dots i.on{background:var(--accent)}
.qno{font-family:var(--mono);font-weight:600;font-size:var(--t-m);color:var(--muted)}
.stop .bubble{margin-left:auto;background:#fff;border:3px solid var(--line);border-radius:calc(var(--u) * 1.6);padding:calc(var(--u) * .6) calc(var(--u) * 1.8);font-weight:700;font-size:var(--t-m)}
.qwrap{flex:1;min-height:0;display:grid;grid-template-columns:minmax(0,2fr) minmax(0,3fr);grid-template-rows:minmax(0,1fr) auto;
  grid-template-areas:"q a" "q e";column-gap:calc(var(--u) * 5);row-gap:calc(var(--u) * 1.2);align-items:center}
.qlabel{grid-area:q;margin:0;font-weight:900;font-size:calc(var(--u) * 5.8);line-height:1.3}
.tiles{grid-area:a;--n:5;display:grid;grid-template-columns:minmax(0,1fr);grid-auto-rows:minmax(var(--tap),1fr);gap:calc(var(--u) * 1.4);padding:4px;align-self:center;
  height:min(100%,calc(var(--n) * var(--u) * 14.5));overflow-y:auto}
.tile{min-height:var(--tap);padding:calc(var(--u) * 1.2) calc(var(--u) * 2.6);border-radius:calc(var(--u) * 2);background:#fff;border:3px solid var(--line);
  font-weight:700;font-size:calc(var(--u) * 3.6);text-align:left;display:flex;align-items:center;gap:calc(var(--u) * 1.8);transition:transform .08s,background .12s}
.tile:active{transform:scale(.98)}
.tile .ck{flex:none;width:calc(var(--u) * 4);height:calc(var(--u) * 4);border-radius:8px;border:3px solid var(--line);display:grid;place-items:center;font-size:calc(var(--u) * 2.6);color:transparent}
.tile.sel{background:var(--accent-soft);border-color:var(--accent)}
.tile.sel .ck{background:var(--accent);border-color:var(--accent);color:#fff}
.tile:disabled{opacity:.4}
/* 立体ボタン(下に厚みを付け、押すと沈む。kousensai_survey.html と同じ) */
.tile{box-shadow:0 4px 0 #b7c4ca,0 8px 12px rgba(16,32,42,.1)}
.tile:active{transform:translateY(3px);box-shadow:0 1px 0 #b7c4ca,0 2px 4px rgba(16,32,42,.1)}
.tile.sel{box-shadow:0 4px 0 #0a5545,0 8px 12px rgba(13,107,88,.2)}
.btn{box-shadow:0 4px 0 #b7c4ca}.btn:active{transform:translateY(3px);box-shadow:0 1px 0 #b7c4ca}
.btn.primary{box-shadow:0 5px 0 #0a5545}.btn.primary:active{box-shadow:0 1px 0 #0a5545}
.btn.ghost,.btn:disabled{box-shadow:none}
/* 5段階の顔: 設問を上に、5つの大きなボタンを横いっぱいに並べる */
.qwrap.is-scale{grid-template-columns:minmax(0,1fr);grid-template-rows:auto auto auto;grid-template-areas:"q" "a" "e";align-items:stretch;align-content:center;row-gap:calc(var(--u) * 2.4)}
.scale{grid-area:a;display:grid;grid-template-columns:repeat(var(--n),1fr);gap:calc(var(--u) * 1.6);align-self:stretch}
.scale .tile{flex-direction:column;justify-content:center;text-align:center;gap:calc(var(--u) * 1.2);min-height:calc(var(--u) * 34);padding:calc(var(--u) * 1.2) 6px;font-family:var(--mono);font-size:calc(var(--u) * 5.4)}
.scale svg{width:calc(var(--u) * 12);height:auto}
.ends{grid-area:e;display:flex;justify-content:space-between;font-size:calc(var(--u) * 3);color:var(--ink);font-weight:900;padding:0 4px}
/* 各段階の色(1=赤 … 5=緑。選ぶと濃くなる) */
.scale .tile{border-width:3px}
.scale .tile[data-v="1"]{background:#ffe3e3;color:#9c1c1c;border-color:#f5b0b0}
.scale .tile[data-v="2"]{background:#fff3d6;color:#8f5800;border-color:#f7d68f}
.scale .tile[data-v="3"]{background:#f0f4f8;color:#334e68;border-color:#cbd5e1}
.scale .tile[data-v="4"]{background:#e0f2fe;color:#0369a1;border-color:#93c5fd}
.scale .tile[data-v="5"]{background:#dcfce7;color:#15803d;border-color:#86efac}
.scale .tile.sel[data-v="1"]{background:#ffb8b8;border-color:#d91e1e;box-shadow:0 6px 0 #9c1c1c}
.scale .tile.sel[data-v="2"]{background:#fde08a;border-color:#d97706;box-shadow:0 6px 0 #8f5800}
.scale .tile.sel[data-v="3"]{background:#cbd5e1;border-color:#475569;box-shadow:0 6px 0 #334e68}
.scale .tile.sel[data-v="4"]{background:#93c5fd;border-color:#0284c7;box-shadow:0 6px 0 #0369a1}
.scale .tile.sel[data-v="5"]{background:#86efac;border-color:#16a34a;box-shadow:0 6px 0 #15803d}
.sbottom{display:flex;gap:calc(var(--u) * 1.4);align-items:center;min-height:calc(var(--tap) + 8px);flex-wrap:wrap}
.sbottom .btn.nav{flex:none;width:max(56px,calc(var(--u) * 7.4));height:max(56px,calc(var(--u) * 7.4));min-height:56px;padding:0;margin-right:calc(var(--u) * 1.6);display:inline-grid;place-items:center;color:var(--ink)}   /* 「1つ前」「さいしょ」は線画アイコンのボタン(56px以上)。「つぎへ」(右端)から離し、2つの間も空ける */
.sbottom .btn.nav svg{width:58%;height:58%;fill:none;stroke:currentColor;stroke-width:2.4;stroke-linecap:round;stroke-linejoin:round;pointer-events:none}
.sbottom .btn.nav:disabled{opacity:.3}
.btn{min-height:var(--tap);padding:0 calc(var(--u) * 3);border-radius:calc(var(--u) * 1.8);font-weight:700;font-size:calc(var(--u) * 2.8);border:3px solid var(--line);background:#fff}
.btn.primary{margin-left:auto;background:var(--accent);border-color:var(--accent);color:#fff;min-width:calc(var(--u) * 24)}
.btn.ghost{background:none;border-color:transparent;color:var(--muted)}
.btn:disabled{opacity:.35}
.btn.danger{color:var(--warn);border-color:var(--warn)}
#done{justify-content:center;align-items:center;text-align:center;gap:calc(var(--u) * 2.4)}
#done .ok{width:calc(var(--u) * 14);height:calc(var(--u) * 14);border-radius:50%;background:var(--accent);color:#fff;display:grid;place-items:center;font-size:calc(var(--u) * 7);font-weight:900}
#done h2{margin:0;font-weight:900;font-size:calc(var(--u) * 6.4)}
#done p{margin:0;color:var(--ink);font-weight:700;font-size:var(--t-l)}
#done .count{background:var(--accent);color:#fff;font-weight:900;font-size:calc(var(--u) * 3.2);padding:calc(var(--u) * 1.2) calc(var(--u) * 3.2);border-radius:999px;box-shadow:0 5px 0 #0a5545}
#done .tapback{color:var(--muted);font-size:var(--t-m);font-weight:700;margin-top:calc(var(--u) * .8)}
#condbar{position:absolute;left:calc(var(--pad) + var(--sal));bottom:calc(var(--u) * 1 + var(--sab));font-family:var(--mono);font-size:var(--t-s);color:var(--muted);opacity:.7}
#adminHit{position:absolute;top:0;right:0;width:calc(var(--u) * 9);height:calc(var(--u) * 9);z-index:5}
/* ダイアログ。横向きは、キーボードが下半分を隠すので上寄せにする */
.sheet{position:fixed;inset:0;background:rgba(16,32,42,.55);display:grid;place-items:start center;z-index:60;padding:calc(var(--u) * 3 + var(--sat)) calc(var(--u) * 2) calc(var(--u) * 2)}
.panel{background:#fff;color:var(--ink);border-radius:calc(var(--u) * 2.4);padding:calc(var(--u) * 3);width:min(calc(var(--u) * 70),100%);max-height:100%;overflow-y:auto;display:grid;gap:calc(var(--u) * 2);
  -webkit-user-select:text;user-select:text}
.panel h3{margin:0;font-size:calc(var(--u) * 3)}
.panel label{display:grid;gap:6px;font-weight:700;font-size:var(--t-m)}
.panel select,.panel input{font:inherit;font-size:max(16px,calc(var(--u) * 2.4));padding:calc(var(--u) * 1.4);border-radius:12px;border:2px solid var(--line);background:#fff;color:var(--ink);width:100%}
.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
.stats div{background:var(--bg);border-radius:12px;padding:calc(var(--u) * 1.2);display:grid}
.stats b{font-family:var(--mono);font-size:calc(var(--u) * 3.2);font-variant-numeric:tabular-nums}
.stats span{font-size:var(--t-s);color:var(--muted)}
.prow{display:flex;flex-wrap:wrap;gap:10px}.prow .btn{flex:1 1 auto;font-size:var(--t-m)}
.msg{font-size:var(--t-s);color:var(--muted);min-height:1.4em}.msg.err{color:var(--warn);font-weight:700}
.pin{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}.pin .btn{font-family:var(--mono);font-size:calc(var(--u) * 3)}
.pinshow{font-family:var(--mono);font-size:calc(var(--u) * 3.6);letter-spacing:.4em;text-align:center;min-height:1.4em}

/* ---- inventory(残数入力。スタッフ用)。横向き: 左=メニュー一覧、右=選んだ品の数字入力(テンキー) ---- */
#v-inventory{background:var(--bg);color:var(--ink);gap:calc(var(--u) * 1.6)}
.ihead{display:flex;align-items:center;gap:var(--gap);flex:0 0 auto}
.ihead h1{margin:0;font-size:calc(var(--u) * 4.6);font-weight:900}
.ihead .who{margin-left:auto;display:flex;align-items:center;gap:calc(var(--u) * 1.4);font-size:var(--t-m);color:var(--muted)}
#istate{flex:0 0 auto;font-size:var(--t-m);color:var(--muted);min-height:0}
#istate:empty{display:none}
#istate.err{color:var(--warn);font-weight:700}
#ibody{flex:1 1 auto;min-height:0;display:grid;grid-template-columns:minmax(0,2fr) minmax(0,3fr);column-gap:calc(var(--u) * 3)}
#ilist{min-height:0;overflow-y:auto;-webkit-overflow-scrolling:touch;display:grid;grid-template-columns:minmax(0,1fr);gap:calc(var(--u) * 1.2);align-content:start}
.irow{background:#fff;border-radius:var(--r);padding:calc(var(--u) * 1.4) calc(var(--u) * 2);display:grid;grid-template-columns:minmax(0,1fr);gap:calc(var(--u) * 1.4);border:3px solid transparent;cursor:pointer;min-height:calc(var(--u) * 9.5);align-content:center}
.irow.dirty{border-color:var(--hi)}
.irow.sel{border-color:var(--accent);background:var(--accent-soft)}
.irow .l1{display:flex;flex-wrap:nowrap;align-items:baseline;gap:calc(var(--u) * 1.6)}
.irow .nm{font-size:calc(var(--u) * 3.4);font-weight:900;flex:1 1 0;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.irow .cur{flex:none;white-space:nowrap;font-size:calc(var(--u) * 2.6);color:var(--muted);font-variant-numeric:tabular-nums}
.irow .cur .ini{display:none}
.irow .cur b{font-size:calc(var(--u) * 4.2);color:var(--ink)}
.irow .l2{display:none}
.irow .hint{display:none}
.irow.empty{cursor:default;display:block;font-size:var(--t-m)}
/* 右の入力パネル */
#ipane{min-height:0;background:#fff;border-radius:var(--r);padding:calc(var(--u) * 2.4);display:grid;gap:calc(var(--u) * 1.4);
  grid-template-columns:minmax(0,3fr) minmax(0,1.25fr);grid-template-rows:auto auto minmax(0,1fr) auto;grid-template-areas:"pn pn" "pv pv" "keys side" "ph ph"}
#ipName{grid-area:pn;display:flex;align-items:baseline;gap:calc(var(--u) * 2);flex-wrap:wrap}
#ipName b{font-size:calc(var(--u) * 4.2);font-weight:900}#ipName span{font-size:calc(var(--u) * 2.6);color:var(--muted);font-variant-numeric:tabular-nums}
#ipVal{grid-area:pv;display:flex;align-items:baseline;justify-content:flex-end;gap:calc(var(--u) * 1.4);padding:calc(var(--u) * .6) calc(var(--u) * 2.4);border:3px solid var(--line);border-radius:calc(var(--u) * 1.4);
  font-family:var(--mono);font-weight:800;font-size:calc(var(--u) * 8.4);line-height:1.15;font-variant-numeric:tabular-nums}
#ipVal small{font-family:var(--f);font-size:calc(var(--u) * 2.8);font-weight:600;color:var(--muted)}
#ipVal.dirty{border-color:var(--hi);background:#fff8e6}#ipVal.idle{color:#8b9a94}
#ipKeys{grid-area:keys;display:grid;grid-template-columns:repeat(3,1fr);grid-template-rows:repeat(4,minmax(var(--tap),1fr));gap:calc(var(--u) * 1.2);min-height:0}
#ipSide{grid-area:side;display:grid;grid-template-rows:repeat(4,minmax(var(--tap),1fr)) minmax(var(--tap),1.7fr);gap:calc(var(--u) * 1.2);min-height:0}
.ikey{border-radius:calc(var(--u) * 1.4);background:var(--bg);border:3px solid var(--line);font-family:var(--mono);font-weight:800;font-size:calc(var(--u) * 4.4);font-variant-numeric:tabular-nums}
.ikey:active{background:#dde5e9}
.ikey.st{background:var(--accent-soft);border-color:#b9dccf;color:var(--accent);font-size:calc(var(--u) * 3.4)}
.ikey.go{background:var(--accent);border-color:var(--accent);color:#fff;font-family:var(--f);font-size:calc(var(--u) * 3.6);font-weight:900}
.ikey:disabled{opacity:.3}
#ipHint{grid-area:ph;font-size:var(--t-s);color:var(--muted);min-height:1.3em}#ipHint.err{color:var(--warn);font-weight:700}
/* メニュー編集(inventory の中のもうひとつの状態)。右の入力パネルは上寄せ(下半分はiPadのキーボードが隠すので、入力欄とボタンは上に置く) */
#v-inventory[data-edit=on] .irow{cursor:pointer}
#v-inventory[data-edit=on] .irow .cur .ini{display:inline}
/* 編集中は名前を省略しない(名前を直す画面なので)。品名の下に「いま・初期」を出す */
#v-inventory[data-edit=on] .irow .l1{flex-wrap:wrap;row-gap:calc(var(--u) * .4)}
#v-inventory[data-edit=on] .irow .nm{flex:1 1 100%;white-space:normal;overflow-wrap:anywhere;text-overflow:clip}
#v-inventory[data-edit=on] .irow .cur{flex:1 1 100%;white-space:normal}
#v-inventory[data-edit=on] .irow .l2,#v-inventory[data-edit=on] .irow .hint{display:none!important}
.irow.addrow{justify-content:center;align-content:center;text-align:center;font-weight:900;font-size:calc(var(--u) * 3.2);color:var(--accent);border:3px dashed var(--accent);background:transparent}
.irow.addrow.sel{background:var(--accent-soft)}
.irow.addrow.full{opacity:.4;cursor:default}
#epane{min-height:0;overflow-y:auto;background:#fff;border-radius:var(--r);padding:calc(var(--u) * 2.4);display:grid;gap:calc(var(--u) * 1.5);align-content:start;grid-auto-rows:min-content;
  -webkit-user-select:text;user-select:text}
#epane .eh{display:flex;align-items:baseline;gap:calc(var(--u) * 2);flex-wrap:wrap}
#epane .eh b{font-size:calc(var(--u) * 3.6);font-weight:900}#epane .eh span{font-size:calc(var(--u) * 2.4);color:var(--muted);font-variant-numeric:tabular-nums}
#epane label{display:grid;gap:calc(var(--u) * .6);font-weight:800;font-size:var(--t-m)}
#epane input{font:inherit;font-weight:700;font-size:max(18px,calc(var(--u) * 3.4));min-height:var(--tap);padding:calc(var(--u) * .8) calc(var(--u) * 1.6);border-radius:calc(var(--u) * 1.4);
  border:3px solid var(--line);background:#fff;color:var(--ink);width:100%;-webkit-user-select:text;user-select:text;-webkit-appearance:none;appearance:none}
#epane input:focus{border-color:var(--accent);outline:none}
#epane input.bad{border-color:var(--warn)}
#epane .eact{display:flex;gap:calc(var(--u) * 1.6)}
#epane .eact .btn{flex:1 1 0;margin:0;min-width:0}
#eMsg{font-size:var(--t-m);color:var(--muted);min-height:1.4em}#eMsg.err{color:var(--warn);font-weight:700}#eMsg.ok{color:var(--accent);font-weight:800}
#epane .enote{font-size:var(--t-s);color:var(--muted);line-height:1.5}

/* ---- ocrtest(模擬POS。実物の長さ[mm]で描く。--mm = 1mmが何CSS pxか)。画面の中央に、本物の7インチ端末と同じ縦向き・同じ実寸で置く ---- */
#v-ocr{background:#17181a;padding:var(--sat) var(--sar) var(--sab) var(--sal);align-items:center;justify-content:center}
#mockwrap{position:relative;flex:none}
#mock{position:absolute;left:0;top:0;transform-origin:0 0;background:#fff;overflow:hidden;display:flex;flex-direction:column;
  width:calc(var(--mm) * 103.35);height:calc(var(--mm) * 144.68)}
#mock .row{flex:1 1 0;display:flex;align-items:center;gap:calc(var(--mm) * 2.65);padding:0 calc(var(--mm) * 4.1) 0 calc(var(--mm) * 3.1);border-bottom:calc(var(--mm) * .26) solid #fff}
#mock .row:nth-child(odd){background:#eaf2fb}#mock .row:nth-child(even){background:#dbe9f8}
#mock .badge{flex:0 0 auto;width:calc(var(--mm) * 7.69);height:calc(var(--mm) * 7.69);border-radius:50%;display:flex;align-items:center;justify-content:center;
  color:#fff;font-weight:700;font-size:calc(var(--mm) * 2.69)}
#mock .row:nth-child(1) .badge{background:#4a3f8c}#mock .row:nth-child(2) .badge{background:#3d4e9e}
#mock .row:nth-child(3) .badge{background:#8b3a86}#mock .row:nth-child(4) .badge{background:#b0447a}
#mock .row:nth-child(5) .badge{background:#6a4aa8}#mock .row:nth-child(6) .badge{background:#3f5aa8}
#mock .row:nth-child(7) .badge{background:#4a4a4a}
#mock .nm{flex:1 1 auto;min-width:0;font-size:calc(var(--mm) * 3.2);color:#1c1c1e;font-weight:500;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
#mock .num{flex:0 0 auto;width:calc(var(--mm) * 35.1);text-align:right;font-family:var(--mono);font-variant-numeric:tabular-nums;font-weight:700;
  font-size:calc(var(--mm) * 7.44);line-height:1;color:#b6452c}
#ocrnote{position:fixed;left:calc(var(--u) * 2 + var(--sal));top:calc(var(--u) * 2 + var(--sat));color:#e8c27a;font-size:max(13px,calc(var(--u) * 1.7));line-height:1.45;font-weight:700}

/* ================= 縦向きのフォールバック(横向きが本命。縦でも切れずに読める程度) ================= */
html[data-orient=portrait] body[data-mode=customer] #v-customer{display:flex;flex-direction:column;gap:var(--gap)}
html[data-orient=portrait] .cbar{margin:0;flex:0 0 auto}
html[data-orient=portrait] .cbar h1{font-size:var(--t-xl)}
html[data-orient=portrait] .cbar .clock{font-size:var(--t-l)}
html[data-orient=portrait] .cstatus{flex:0 0 auto;flex-direction:row;align-items:center;justify-content:flex-start;gap:calc(var(--u) * 3);padding:calc(var(--u) * 3);border-radius:calc(var(--u) * 2.5)}
html[data-orient=portrait] .cstatus .ic{width:calc(var(--u) * 13);height:calc(var(--u) * 13)}
html[data-orient=portrait] .cstatus .tt{font-size:calc(var(--u) * 6)}
html[data-orient=portrait] .cstatus[data-tone=caution] .tt{font-size:calc(var(--u) * 6.4)}
html[data-orient=portrait] .cstatus[data-tone=caution] .ic{width:calc(var(--u) * 15);height:calc(var(--u) * 15)}
html[data-orient=portrait] .cstatus .dd{margin-top:calc(var(--u) * .8);font-size:var(--t-l)}
html[data-orient=portrait] #promote{flex:0 0 auto;margin:0;font-size:var(--t-l);text-align:center}
html[data-orient=portrait] #cards{flex:1 1 auto}
html[data-orient=portrait] #announce{flex:0 0 auto;margin:0;text-align:center;min-height:1.3em}
/* 縦で4行以上: 品名と数字を横に並べて背を低くする */
html[data-orient=portrait] #cards.compact .card{display:grid;grid-template-columns:minmax(0,1fr) auto;grid-template-areas:"nm rm" "bar bar" "lb lb";align-content:center;column-gap:calc(var(--u) * 1)}
html[data-orient=portrait] #cards.compact .card>:first-child,html[data-orient=portrait] #cards.compact .card>:last-child{margin:0}
html[data-orient=portrait] #cards.compact .nm{grid-area:nm;align-self:center;white-space:normal;line-height:1.15;overflow-wrap:anywhere}
html[data-orient=portrait] #cards.compact .rm{grid-area:rm}html[data-orient=portrait] #cards.compact .bar{grid-area:bar}
html[data-orient=portrait] #cards.compact .lb{grid-area:lb;min-height:0;margin-top:2px}html[data-orient=portrait] #cards.compact .lb:empty{display:none}

html[data-orient=portrait] #v-survey #idle{display:flex;flex-direction:column;gap:calc(var(--u) * 4)}
html[data-orient=portrait] #idle h1{font-size:calc(var(--u) * 8.4)}
html[data-orient=portrait] #idle .note{font-size:calc(var(--u) * 2.8)}
html[data-orient=portrait] #idle .go{align-self:flex-start;flex-direction:row;width:auto;height:auto;border-radius:999px;gap:calc(var(--u) * 1.6);font-size:calc(var(--u) * 3.4);padding:calc(var(--u) * 2.2) calc(var(--u) * 4)}
html[data-orient=portrait] #idle .go b{width:calc(var(--u) * 4.4);height:calc(var(--u) * 4.4);font-size:calc(var(--u) * 2.6)}
html[data-orient=portrait] .qwrap,html[data-orient=portrait] .qwrap.is-scale{display:flex;flex-direction:column;justify-content:center;align-items:stretch;gap:calc(var(--u) * 3.5)}
html[data-orient=portrait] .qlabel{font-size:calc(var(--u) * 4.8)}
html[data-orient=portrait] .tiles{align-self:stretch;height:auto;display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,calc(var(--u) * 30)),1fr));grid-auto-rows:auto;gap:calc(var(--u) * 1.4)}
html[data-orient=portrait] .tile{min-height:max(var(--tap),calc(var(--u) * 10));font-size:calc(var(--u) * 3)}
html[data-orient=portrait] .tile .ck{width:calc(var(--u) * 3.2);height:calc(var(--u) * 3.2);font-size:calc(var(--u) * 2.2)}
html[data-orient=portrait] .scale .tile{min-height:calc(var(--u) * 17);font-size:calc(var(--u) * 4)}
html[data-orient=portrait] .scale svg{width:calc(var(--u) * 8)}
html[data-orient=portrait] .ends{font-size:calc(var(--u) * 2.2)}
html[data-orient=portrait] .sheet{place-items:center}

html[data-orient=portrait] #ibody{display:contents}
html[data-orient=portrait] #ipane{display:none}
html[data-orient=portrait] #ilist{flex:1 1 auto;gap:calc(var(--u) * 1.6)}
html[data-orient=portrait] .irow{cursor:default;padding:calc(var(--u) * 2);min-height:0;border-color:transparent;background:#fff}
html[data-orient=portrait] .irow.dirty{border-color:var(--hi)}
html[data-orient=portrait] .irow .l2{display:flex;align-items:center;gap:calc(var(--u) * 1.2)}
html[data-orient=portrait] .irow .hint{display:block;font-size:var(--t-s);color:var(--muted);min-height:1.2em}
html[data-orient=portrait] .irow .hint.err{color:var(--warn);font-weight:700}
html[data-orient=portrait] .irow .l1{flex-wrap:wrap}html[data-orient=portrait] .irow .nm{flex:1 1 40%}
html[data-orient=portrait] .irow .cur{font-size:calc(var(--u) * 3);white-space:normal}html[data-orient=portrait] .irow .cur .ini{display:inline}
html[data-orient=portrait] .irow .cur b{font-size:calc(var(--u) * 5)}
html[data-orient=portrait] .irow .st{flex:none;min-width:var(--tap);min-height:var(--tap);padding:0 calc(var(--u) * 1.2);border-radius:calc(var(--u) * 1.4);background:var(--accent-soft);color:var(--accent);font-weight:900;font-size:calc(var(--u) * 2.8);font-variant-numeric:tabular-nums}
html[data-orient=portrait] .irow .st:active{background:#b9dccf}
html[data-orient=portrait] .irow input{flex:1 1 0;width:0;min-width:0;min-height:var(--tap);text-align:center;font-family:var(--mono);font-weight:800;font-size:calc(var(--u) * 4.4);border:3px solid var(--line);border-radius:calc(var(--u) * 1.4);background:#fff;color:var(--ink);-webkit-user-select:text;user-select:text}
html[data-orient=portrait] .irow .go{flex:none;min-height:var(--tap);padding:0 calc(var(--u) * 2.4);border-radius:calc(var(--u) * 1.4);background:var(--accent);color:#fff;font-weight:900;font-size:calc(var(--u) * 2.8)}
html[data-orient=portrait] .irow .go:disabled{opacity:.3}
html[data-orient=portrait] #istate{min-height:1.3em}
html[data-orient=portrait] #epane{position:fixed;z-index:40;left:calc(var(--pad) + var(--sal));right:calc(var(--pad) + var(--sar));top:calc(var(--pad) + var(--sat) + var(--u) * 9);
  max-height:calc(var(--app-h) * .5);box-shadow:0 8px 30px rgba(0,0,0,.35)}
html[data-orient=portrait] #v-inventory[data-edit=on] .irow.sel{border-color:var(--accent);background:var(--accent-soft)}
html[data-orient=portrait] #istate:empty{display:block}
</style>
</head>
<body data-mode="loading">

<!-- 読み込み中 -->
<section class="view" id="v-loading"><img src="__LOGO__" alt=""><div>つなげています…</div></section>

<!-- customer -->
<section class="view" id="v-customer" aria-live="polite">
  <div class="cbar"><img src="__LOGO__" alt=""><h1>のこさへん</h1><span class="clock" id="clock"></span></div>
  <div class="cstatus" id="cstatus" data-tone="info"><div class="ic" id="cic"></div><div><div class="tt" id="ctt">準備中です</div><div class="dd" id="cdd"></div></div></div>
  <div id="promote" hidden></div>
  <div id="cards"></div>
  <div id="announce"></div>
</section>

<!-- survey -->
<section class="view" id="v-survey">
  <section class="screen" id="idle" role="button" tabindex="0" aria-label="アンケートをはじめる">
    <div class="tag">MIRS2603 ・ QUESTIONNAIRE</div>
    <h1 id="idleTitle"></h1>
    <div class="note" id="idleNote"></div>
    <div class="idlecount" id="idleCount" hidden></div>
    <div class="go"><b aria-hidden="true">→</b><span id="startLabel"></span></div>
  </section>
  <section class="screen" id="ask" hidden>
    <div class="stop"><div class="dots" id="dots"></div><span class="qno" id="qno"></span><div class="bubble" id="bubble"></div></div>
    <div class="qwrap" id="qwrap"></div>
    <div class="sbottom">
      <button class="btn nav" id="back" type="button" aria-label="1つ前にもどる" title="1つ前にもどる"><svg viewBox="0 0 24 24" aria-hidden="true" focusable="false"><path d="M20 12H5M11 5l-7 7 7 7"/></svg></button>
      <button class="btn nav" id="first" type="button" aria-label="さいしょにもどる" title="さいしょにもどる"><svg viewBox="0 0 24 24" aria-hidden="true" focusable="false"><path d="M4 5v14M20 6l-6 6 6 6M13 6l-6 6 6 6"/></svg></button>
      <button class="btn ghost" id="skip" type="button">とばす</button>
      <button class="btn primary" id="next" type="button">つぎへ →</button>
    </div>
  </section>
  <section class="screen" id="done" hidden>
    <div class="ok" aria-hidden="true">✓</div><h2 id="thanks"></h2><div class="count" id="doneCount"></div><p id="doneNote"></p><div class="tapback">タッチして もどる</div>
  </section>
  <div id="condbar"></div>
  <button id="adminHit" type="button" aria-label="管理"></button>
</section>

<!-- inventory -->
<section class="view" id="v-inventory">
  <div class="ihead"><h1 id="ihTitle">残数入力</h1>
    <div class="who"><button class="btn" id="editToggle" type="button">メニュー編集</button></div></div>
  <div id="istate"></div>
  <div id="ibody">
    <div id="ilist"></div>
    <div id="ipane" hidden>
      <div id="ipName"><b id="ipNm"></b><span id="ipCur"></span></div>
      <div id="ipVal"><span id="ipNum"></span><small>食</small></div>
      <div id="ipKeys"></div>
      <div id="ipSide"></div>
      <div id="ipHint"></div>
    </div>
    <div id="epane" hidden>
      <div class="eh"><b id="eTitle">メニューの編集</b><span id="eSub"></span></div>
      <label>メニュー名<input id="eName" type="text" maxlength="40" autocomplete="off" autocapitalize="off" autocorrect="off" spellcheck="false" enterkeyhint="done"></label>
      <label>初期食数<input id="eInit" type="text" inputmode="numeric" pattern="[0-9]*" maxlength="5" autocomplete="off" enterkeyhint="done"></label>
      <div class="eact"><button class="btn danger" id="eDel" type="button">削除…</button><button class="btn primary" id="eGo" type="button">確定</button></div>
      <div id="eMsg" role="status"></div>
      <div class="enote" id="eNote"></div>
    </div>
  </div>
</section>

<!-- face(中身の iframe は face モードに入ったときだけ JS が作る) -->
<section class="view" id="v-face"></section>

<!-- ocrtest -->
<section class="view" id="v-ocr"><div id="mockwrap"><div id="mock" aria-label="模擬POS"></div></div><div id="ocrnote" hidden></div></section>

<div id="conn" hidden>つながっていません</div>
<div id="rotate" hidden>横向きにしてください</div>
<div id="dbg" hidden></div>

<!-- メニュー削除の確認(誤タップ防止の2段階) -->
<div class="sheet" id="delSheet" hidden><div class="panel" role="alertdialog" aria-labelledby="delTitle">
  <h3 id="delTitle">このメニューを削除しますか?</h3>
  <div class="msg" id="delText"></div>
  <div class="prow"><button class="btn primary" id="delNo" type="button">やめる</button><button class="btn danger" id="delYes" type="button">削除する</button></div>
</div></div>
<!-- アンケートの隠し管理(右上の角を5回タップ → PIN) -->
<div class="sheet" id="pinSheet" hidden><div class="panel"><h3>管理用 PIN</h3><div class="pinshow" id="pinShow"></div><div class="pin" id="pinPad"></div><div class="msg" id="pinMsg"></div></div></div>
<div class="sheet" id="adminSheet" hidden><div class="panel">
  <h3>アンケートの管理</h3>
  <div class="stats"><div><b id="stTotal">0</b><span>保存した回答</span></div><div><b id="stToday">0</b><span>今日の回答(この端末)</span></div><div><b id="stPending">0</b><span>未送信</span></div></div>
  <label>実験条件 (以後の回答に記録)<select id="condSel"></select></label>
  <label>設置場所・メモ<input id="locIn" type="text" maxlength="80" placeholder="例: 食堂入口 / 第1展示室"></label>
  <div class="prow"><button class="btn" id="aSync" type="button">今すぐ再送</button></div>
  <div class="msg" id="aMsg"></div>
  <div class="prow"><button class="btn primary" id="aClose" type="button">閉じる</button></div>
</div></div>

<script>
window.NK_SURVEY = __SURVEY_JSON__;
window.NK_FACE = __FACE_JSON__;   /* 顔のHTML(文字列)。face モードのときだけ iframe の srcdoc に入れる */
window.NK_BUILD = "__BUILD__";
</script>
<script>
/*NKSIZE-BEGIN*/
/* 画面サイズの判定と、模擬POS(OCRテスト)の実寸の計算。DOMを使わない純粋な関数(node でも動く)。
 * 同じ式が display_sizes.py にある。式を変えたら両方直す(test_ipad_display.py が照合する)。
 *
 * iPadのCSS px は 1/132 インチ(264ppiの半分)。12.9インチも11インチも同じなので、同じCSS px数は同じ実寸になる。
 * iPad mini 6/7 だけ 163。CSSの "mm" 単位は96dpi換算で実物の長さではないので使わず、px = mm * ppi / 25.4 で自分で換算する。 */
var NK_MOCK = {w: 103.35, h: 144.68, rows: 7, fontRatio: 0.072, digitEm: 0.698};
var NK_ALIASES = {"12.9": "12.9", "13": "12.9", "12": "12.9", "xl": "12.9", "11": "11", "10.9": "11", "10.5": "11", "10.2": "11", "l": "11",
  "9.7": "small", "8.3": "small", "mini": "small", "small": "small", "m": "small", "phone": "phone", "s": "phone"};
var NK_LAYOUT_KEY = {"12.9": "xl", "11": "l", "small": "m", "phone": "s", "other": "l"};
function nkParseQuery(search) {
  var o = {};
  String(search || "").replace(/^\?/, "").split("&").forEach(function (p) {
    if (!p) return;
    var i = p.indexOf("=");
    try {
      var k = decodeURIComponent(i < 0 ? p : p.slice(0, i));
      o[k] = i < 0 ? "" : decodeURIComponent(p.slice(i + 1).replace(/\+/g, " "));
    } catch (e) { /* 壊れた引数は無視 */ }
  });
  return o;
}
function nkNum(v) { var n = parseFloat(v); return isFinite(n) ? n : null; }
function nkClassify(shortCss, override) {
  var f = override == null ? null : NK_ALIASES[String(override).trim().toLowerCase()];
  if (f) return f;
  if (shortCss >= 1000) return "12.9";
  if (shortCss >= 800) return "11";
  if (shortCss >= 700) return "small";
  return "phone";
}
function nkSize(env) {
  var q = nkParseQuery(env.search);
  var ua = String(env.ua || "");
  var isIPad = /iPad/.test(ua) || (/Macintosh/.test(ua) && (env.touchPoints || 0) > 1);
  var innerShort = Math.min(env.innerW, env.innerH);
  var deviceShort = isIPad ? Math.min(env.screenW || env.innerW, env.screenH || env.innerH) : innerShort;
  var forced = q.size != null && !!NK_ALIASES[String(q.size).trim().toLowerCase()];
  var key = nkClassify(deviceShort, q.size);
  var ppiQ = nkNum(q.ppi);
  var ppi = (ppiQ != null && ppiQ > 0) ? ppiQ : ((!isIPad && !forced) ? 96 : ((deviceShort >= 740 && deviceShort <= 750) ? 163 : 132));
  var calib = nkNum(q.calib); calib = calib == null ? 1 : Math.max(0.5, Math.min(2, calib));
  return {key: key, layout: NK_LAYOUT_KEY[key] || "l", forced: forced, isIPad: isIPad,
    orient: env.innerW > env.innerH ? "landscape" : "portrait", short: innerShort, long: Math.max(env.innerW, env.innerH),
    ppi: ppi, calib: calib, ppmm: ppi / 25.4 * calib, fit: nkNum(q.ocrfit), rot: nkRot(q.ocrrot), debug: q.debug === "1"};
}
/* ?ocrrot=90|270|-90: 模擬POSを横向きの端末として回して出す(既定は0=本物と同じ縦向き)。raspi版の mock_pos_tablet.html?rot= と同じ考え方 */
function nkRot(v) {
  var n = nkNum(v);
  return n === 90 ? 90 : ((n === 270 || n === -90) ? 270 : 0);
}
/* 模擬POSの描画寸法(CSS px)。rot=90/270 のとき縦横を入れ替えた外形で数える。mx/my=左右・上下の余白(セーフエリア)。
 * fit>0: 画面(余白を除く)の何割まで拡大/縮小して出すか(実寸ではなくなる)。shrink=true: 実寸が入らないときだけ、入る大きさまで縮める。 */
function nkOcrGeometry(vw, vh, ppmm, fit, rot, mx, my, shrink) {
  mx = mx || 0; my = my || 0;
  var pw = NK_MOCK.w * ppmm, ph = NK_MOCK.h * ppmm;
  var sideways = rot === 90 || rot === 270;
  var w = sideways ? ph : pw, h = sideways ? pw : ph, scale = 1;
  var aw = Math.max(0, vw - 2 * mx), ah = Math.max(0, vh - 2 * my);
  if (fit != null && fit > 0) scale = Math.min(aw * fit / w, ah * fit / h);
  else if (shrink && (w > aw + 0.5 || h > ah + 0.5)) scale = Math.min(aw / w, ah / h);
  w *= scale; h *= scale;
  var digitMm = NK_MOCK.w * NK_MOCK.fontRatio * NK_MOCK.digitEm * scale;
  return {ppmm: ppmm * scale, screen_w: w, screen_h: h, scale: scale, digit_mm: digitMm, digit_px: digitMm * ppmm,
    frac_w: w / vw, frac_h: h / vh, fits: w <= aw + 0.5 && h <= ah + 0.5, rot: sideways ? rot : 0, shrunk: scale < 1 && !(fit != null && fit > 0)};
}
/*NKSIZE-END*/
</script>
<script>
(function () {
  "use strict";
  var S = window.NK_SURVEY;
  var $ = function (id) { return document.getElementById(id); };
  var body = document.body, root = document.documentElement;

  /* =========================== 画面サイズ =========================== */
  var size = null, lastOcrKey = "";
  function applySize() {
    var vw = window.innerWidth, vh = window.innerHeight;
    size = nkSize({innerW: vw, innerH: vh, screenW: window.screen && screen.width, screenH: window.screen && screen.height,
      search: location.search, ua: navigator.userAgent, touchPoints: navigator.maxTouchPoints});
    root.style.setProperty("--app-w", vw + "px");
    root.style.setProperty("--app-h", vh + "px");
    root.setAttribute("data-size", size.layout);
    root.setAttribute("data-size-key", size.key);
    root.setAttribute("data-orient", size.orient);
    root.style.setProperty("--mm", size.ppmm + "px");
    $("rotate").hidden = size.orient !== "portrait";    /* 縦向きの薄い案内(動作は止めない) */
    layoutCards();
    drawMock(true);
    paintDbg();
  }
  /* ?debug=1: どのモードでも左上に、判定した区分・向き・px を出す(OCRテストのときは模擬POSの寸法も) */
  var dbgOcr = "";
  function paintDbg() {
    var d = $("dbg");
    d.hidden = !(size && size.debug);
    if (d.hidden) return;
    var u = Math.min(window.innerWidth, window.innerHeight) / 100;
    d.textContent = "size=" + size.key + " (" + size.layout + ")" + (size.forced ? " [?size=]" : "") + "  " + size.orient + "  viewport " + window.innerWidth + "x" + window.innerHeight +
      "  u=" + u.toFixed(2) + "px  mode=" + mode + (mode === "ocrtest" ? "\n" + dbgOcr : "");
  }
  var rzT = null;
  function onResize() { clearTimeout(rzT); rzT = setTimeout(applySize, 60); }
  window.addEventListener("resize", onResize);
  window.addEventListener("orientationchange", onResize);

  /* =========================== 状態の取得(ポーリング) =========================== */
  var state = null, lastOk = 0, failCount = 0, pollTimer = null, polling = false, mode = "loading";
  function setConn() {
    var bad = lastOk && (Date.now() - lastOk > 8000);
    $("conn").hidden = !(bad || (!lastOk && failCount >= 3));
  }
  function schedule(ms) { clearTimeout(pollTimer); pollTimer = setTimeout(poll, ms); }
  /* ログインが切れた(ロボットのプログラムが再起動してセッションが消えた)。読み込み直すとログイン画面になる。10秒以内に続けては読み込み直さない */
  function authLost() {
    try { var t = +sessionStorage.getItem("nk_re401") || 0; if (Date.now() - t < 10000) return; sessionStorage.setItem("nk_re401", String(Date.now())); } catch (e) { /* 使えなくても読み込み直す */ }
    location.reload();
  }
  function poll() {
    if (polling) return;
    polling = true;
    var q = (mode === "ocrtest" && ocr && ocr.seq) ? "?seen=" + ocr.seq : "";
    var ctl = ("AbortController" in window) ? new AbortController() : null;
    var to = setTimeout(function () { if (ctl) ctl.abort(); }, 6000);
    fetch("/display/api/state" + q, {cache: "no-store", signal: ctl ? ctl.signal : undefined})
      .then(function (r) { if (r.status === 401) { authLost(); throw new Error("401"); } if (!r.ok) throw new Error("HTTP " + r.status); return r.json(); })
      .then(function (d) { clearTimeout(to); polling = false; failCount = 0; lastOk = Date.now(); onState(d); setConn(); schedule(1000); })
      .catch(function () { clearTimeout(to); polling = false; failCount++; setConn(); schedule(Math.min(5000, 1000 + failCount * 750)); });
  }
  function pollNow() { schedule(0); }
  document.addEventListener("visibilitychange", function () { if (!document.hidden) { pollNow(); wakeLock(); } });
  window.addEventListener("pageshow", function () { pollNow(); wakeLock(); });
  window.addEventListener("focus", pollNow);
  window.addEventListener("online", pollNow);
  setInterval(setConn, 2000);

  function onState(d) {
    if (!d || d.ok !== true) return;
    if (window.NK_BUILD && d.build && d.build !== window.NK_BUILD) { location.reload(); return; }   /* 画面のプログラムが更新された */
    state = d;
    var m = d.mode;
    if (m !== "customer" && m !== "survey" && m !== "inventory" && m !== "ocrtest" && m !== "face") return;
    if (m !== mode) { leaveMode(mode); mode = m; body.setAttribute("data-mode", m); enterMode(m); }
    if (m === "customer") renderCustomer(d.customer);
    else if (m === "inventory") renderInventory(d.customer);
    else if (m === "survey") surveyState(d);
    else if (m === "ocrtest") { ocr = d.ocr_mock || ocr; drawMock(false); }
    $("condbar").textContent = [store.condition, store.location].filter(Boolean).join(" ・ ");
  }
  function leaveMode(m) {
    if (m === "survey") { surveyLeave(); }
    if (m === "inventory") { invLeave(); }
    if (m === "face") { faceLeave(); }
  }
  /* 顔: iframe を作る/取り除く。取り除くと顔のアニメーション(requestAnimationFrame・タイマー)も止まる */
  function faceEnter() {
    var host = $("v-face");
    if (host.firstChild) return;
    var f = document.createElement("iframe");
    f.id = "faceFrame";
    f.title = "ロボットの顔";
    f.setAttribute("sandbox", "allow-scripts");
    f.setAttribute("scrolling", "no");
    f.srcdoc = window.NK_FACE;
    host.appendChild(f);
  }
  function faceLeave() {
    var host = $("v-face");
    while (host.firstChild) { host.removeChild(host.firstChild); }
  }
  function enterMode(m) {
    if (m === "survey") surveyEnter();
    if (m === "inventory") invEnter();
    if (m === "face") faceEnter();
    if (m === "ocrtest") { drawMock(true); }
    applySize();
  }

  /* =========================== customer =========================== */
  var ICONS = {
    ok: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M8 5v14l11-7z"/></svg>',
    info: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M7 5h4v14H7zM13 5h4v14h-4z"/></svg>',
    warn: '<svg viewBox="0 0 24 24" fill="currentColor"><path d="M10.5 4h3l-.6 10h-1.8zM12 16.4a1.7 1.7 0 110 3.4 1.7 1.7 0 010-3.4z"/></svg>',
    caution: '<svg viewBox="0 0 24 24" fill="currentColor" fill-rule="evenodd"><path d="M12 2.2l10.8 19H1.2zM10.8 9v6.2h2.4V9zM10.8 16.6V19h2.4v-2.4z"/></svg>'
  };
  var lastTone = "";
  function renderCustomer(c) {
    c = c || {};
    var st = c.status || {title: "準備中です", detail: "", tone: "info"};
    var tone = ICONS[st.tone] ? st.tone : "info";
    $("cstatus").setAttribute("data-tone", tone);
    if (tone !== lastTone) { $("cic").innerHTML = ICONS[tone]; lastTone = tone; }   /* 固定の図形だけ(サーバの文字は入れない) */
    $("ctt").textContent = tone === "caution" ? String(st.title || "").replace(/。(?=.)/g, "。\n") : (st.title || "");   /* 注意の文言は「。」で改行(2行で読める) */
    $("cdd").textContent = st.detail || "";
    $("cdd").hidden = !st.detail;
    var menu = c.menu || {items: []};
    var pr = $("promote");
    if (menu.promote_item) { pr.hidden = false; pr.textContent = "本日は「" + menu.promote_item + "」にまだ余裕があります。できたてのうちにぜひどうぞ！"; }
    else pr.hidden = true;
    $("announce").textContent = c.announce || "";
    var host = $("cards"), items = menu.items || [];
    var key = items.map(function (i) { return i.name; }).join("\u0001");
    if (host.getAttribute("data-key") !== key) {
      host.setAttribute("data-key", key);
      host.textContent = "";
      if (!items.length) { var e = document.createElement("div"); e.id = "cempty"; e.textContent = "本日のメニューは準備中です"; host.appendChild(e); }
      items.forEach(function () {
        var card = document.createElement("div"); card.className = "card";
        card.innerHTML = '<div class="nm"></div><div class="rm"><span class="n"></span><small>食</small></div><div class="bar"><i></i></div><div class="lb"></div>';
        host.appendChild(card);
      });
    }
    var cards = host.querySelectorAll(".card");
    items.forEach(function (it, i) {
      var card = cards[i]; if (!card) return;
      var ratio = it.initial > 0 ? Math.max(0, Math.min(1, it.remaining / it.initial)) : 0;
      var cls = it.remaining <= 0 ? "out" : (ratio <= 0.2 ? "low" : "");
      card.className = "card" + (cls ? " " + cls : "");
      card.querySelector(".nm").textContent = it.name;
      card.querySelector(".n").textContent = it.remaining;
      card.querySelector(".bar i").style.width = Math.round(ratio * 100) + "%";
      card.querySelector(".lb").textContent = it.remaining <= 0 ? "本日完売" : (ratio <= 0.2 ? "残りわずかです" : "");
    });
    layoutCards(items.length);
  }
  var lastN = 0, lastFitSig = "";
  /* 横向き(既定): 右の領域(幅約3/5・高さ約94u)に品数ごとに並べる。[列数, 並べ方, 数字の大きさ(u), 品名の大きさ(u)]
     h=品名と数字を横に並べる(1列・背が高い)、v=品名の下に数字(2列)、v2=3列以上で品名を2行まで。入りきらなければ fitCards が縮める */
  function layoutCards(n) {
    if (n == null) n = lastN; lastN = n;
    var land = root.getAttribute("data-orient") !== "portrait", cols, lay = "", compact = false, num, nm = 3;
    if (land) {
      var P = n <= 1 ? [1, "h", 16, 5.4] : n === 2 ? [1, "h", 15, 5] : n === 3 ? [1, "h", 13, 4.4] : n === 4 ? [1, "h", 10.5, 4]
        : n <= 6 ? [2, "v", 13, 3.8] : n <= 8 ? [2, "v", 9, 3.2] : n <= 12 ? [3, "v2", 7.5, 2.7] : [4, "v2", 5.5, 2.4];
      cols = P[0]; lay = P[1]; num = P[2]; nm = P[3];
    } else {
      cols = n <= 3 ? 1 : 2;
      var rows = Math.ceil(Math.max(1, n) / cols);
      compact = rows >= 4;
      num = compact ? (rows >= 5 ? 6 : 8) : (cols === 1 ? (n <= 2 ? 15 : 12) : 11);
    }
    root.style.setProperty("--cols", cols);
    root.style.setProperty("--numsz", "calc(var(--u) * " + num + " * var(--k))");
    root.style.setProperty("--nmsz", "calc(var(--u) * " + nm + " * var(--k))");
    var host = $("cards");
    host.classList.toggle("compact", compact);
    if (lay) host.setAttribute("data-lay", lay); else host.removeAttribute("data-lay");
    fitCards();
  }
  /* カードの中身が切れる(はみ出す)ときは、文字を 0.9 倍ずつ縮める(--k)。内容・画面サイズが変わったときだけ計算し直す */
  function fitCards() {
    if (mode !== "customer") return;
    var host = $("cards");
    var sig = root.getAttribute("data-orient") + window.innerWidth + "x" + window.innerHeight + "|" + lastN + "|" + host.getAttribute("data-lay") + "|" + host.textContent;
    if (sig === lastFitSig) return;
    lastFitSig = sig;
    var k = 1;
    root.style.setProperty("--k", "1");
    for (var i = 0; i < 10; i++) {
      var bad = false, cs = host.children;
      for (var j = 0; j < cs.length; j++) {
        var c = cs[j];
        if (c.scrollHeight > c.clientHeight + 1 || c.scrollWidth > c.clientWidth + 1) { bad = true; break; }
      }
      if (!bad) break;
      k *= 0.9; root.style.setProperty("--k", k.toFixed(3));
    }
  }
  function tickClock() {
    var d = new Date(), p = function (x) { return (x < 10 ? "0" : "") + x; };
    $("clock").textContent = p(d.getHours()) + ":" + p(d.getMinutes());
  }
  setInterval(tickClock, 5000); tickClock();

  /* =========================== OCRテスト(模擬POS) =========================== */
  var ocr = null, ROWS = [["1", "Aランチ"], ["2", "Bランチ"], ["3", "Cランチ"], ["7", "カレー"], ["5", "麺類(昼)"], ["4", "アラカルトB"], ["6", "アラカルト100円"]];
  var mockBuilt = false;
  function drawMock(force) {
    if (mode !== "ocrtest" && !force) return;
    var host = $("mock");
    if (!mockBuilt) {
      ROWS.forEach(function (r) {
        var row = document.createElement("div"); row.className = "row";
        var b = document.createElement("div"); b.className = "badge"; b.textContent = r[0];
        var n = document.createElement("div"); n.className = "nm"; n.textContent = r[1];
        var v = document.createElement("div"); v.className = "num"; v.textContent = "";
        row.appendChild(b); row.appendChild(n); row.appendChild(v); host.appendChild(row);
      });
      mockBuilt = true;
    }
    if (!size) return;
    var vw = parseFloat(root.style.getPropertyValue("--app-w")) || window.innerWidth;
    var vh = parseFloat(root.style.getPropertyValue("--app-h")) || window.innerHeight;
    /* セーフエリア(ホームバー・ステータスバー)を避けた、使える大きさで考える。左右・上下は大きい方を両側に取って中央に置く */
    var cs = getComputedStyle($("v-ocr")), px = function (x) { return parseFloat(x) || 0; };
    var mx = Math.max(px(cs.paddingLeft), px(cs.paddingRight)), my = Math.max(px(cs.paddingTop), px(cs.paddingBottom));
    var g = nkOcrGeometry(vw, vh, size.ppmm, size.fit, size.rot, mx, my, true);
    host.style.setProperty("--mm", g.ppmm + "px");        /* 実寸なら size.ppmm のまま。?ocrfit= か、実寸が入らず縮めたときだけ変わる */
    var wrap = $("mockwrap");
    wrap.style.width = g.screen_w + "px"; wrap.style.height = g.screen_h + "px";
    var pw = NK_MOCK.w * g.ppmm, ph = NK_MOCK.h * g.ppmm;
    host.style.transform = g.rot === 90 ? "translate(" + ph + "px,0) rotate(90deg)" : (g.rot === 270 ? "translate(0," + pw + "px) rotate(-90deg)" : "none");
    if (ocr && ocr.values) {
      var nums = host.querySelectorAll(".num");
      for (var i = 0; i < nums.length && i < ocr.values.length; i++) nums[i].textContent = String(ocr.values[i]);
    }
    /* 実寸が入らなくて縮めたときは、横の余白に小さく知らせる(模擬画面には重ねない) */
    var note = $("ocrnote"), side = (vw - g.screen_w) / 2 - 24;
    if (g.shrunk && g.scale < 0.97 && side >= 150) {
      note.hidden = false; note.style.maxWidth = side + "px";
      note.textContent = "実寸が画面に入らないため " + Math.round(g.scale * 100) + "% に縮小中(数字 " + g.digit_mm.toFixed(2) + "mm)。ホーム画面から開くか、?ocrrot=90 を試す";
    } else note.hidden = true;
    dbgOcr = "ppi=" + size.ppi + " calib=" + size.calib + " px/mm=" + g.ppmm.toFixed(3) + (size.fit ? " fit=" + size.fit : (g.shrunk ? " 縮小 x" + g.scale.toFixed(3) : " 実寸")) + (g.rot ? " rot=" + g.rot : " 縦向き") + "\n" +
      "mock " + g.screen_w.toFixed(1) + "x" + g.screen_h.toFixed(1) + "px (" + (g.frac_w * 100).toFixed(0) + "% x " + (g.frac_h * 100).toFixed(0) + "%)  余白 " + mx.toFixed(0) + "," + my.toFixed(0) + "\n" +
      "digit " + g.digit_mm.toFixed(2) + "mm = " + g.digit_px.toFixed(1) + "px  seq=" + (ocr && ocr.seq) + (g.fits ? "" : "  ** はみ出し **");
    paintDbg();
  }

  /* =========================== 残数入力 / メニュー編集 =========================== */
  /* ログインは最初の1回だけ(ログイン画面は別のHTML)。ここに来ている=ログイン済みなので、入力は常にできる */
  var who = {account: "staff", can: true};
  var inv = {rows: {}, drafts: {}, items: [], sel: null};
  function invEnter() { invPaintAll(); }
  function invLeave() { edExit(); }
  function renderInventory(c) {
    var items = (c && c.menu && c.menu.items) || [];
    inv.items = items;
    var host = $("ilist"), key = items.map(function (i) { return i.name; }).join("\u0001") + (ed.on ? "\u0002E" + items.length : "");
    if (host.getAttribute("data-key") !== key) {
      host.setAttribute("data-key", key); host.textContent = ""; inv.rows = {};
      items.forEach(function (it) { host.appendChild(invRow(it.name)); });
      if (!items.length) { var e = document.createElement("div"); e.className = "irow empty"; e.textContent = "本日のメニューが登録されていません。右上の「メニュー編集」から追加するか、厨房の画面(ポート8080)で登録してください。"; host.appendChild(e); }
      if (ed.on) edBuildAddRow(host, items.length);
    }
    items.forEach(function (it) {
      var r = inv.rows[it.name]; if (!r) return;
      r.item = it;
      r.cur.textContent = "";
      var b = document.createElement("b"); b.textContent = it.remaining;
      var ini = document.createElement("span"); ini.className = "ini"; ini.textContent = " / 初期 " + it.initial;   /* 横向きの一覧では省く(右のパネルに出る) */
      r.cur.appendChild(document.createTextNode("いま ")); r.cur.appendChild(b); r.cur.appendChild(document.createTextNode(" 食")); r.cur.appendChild(ini);
      if (!(it.name in inv.drafts) && document.activeElement !== r.input) r.input.value = String(it.remaining);
      invPaint(r);
    });
    if (!inv.sel || !inv.rows[inv.sel]) invSel(items.length ? items[0].name : null);
    if (ed.on) edSync();
    paintPane();
  }
  /* 横向き: 一覧の行を押すと右のパネルでその品の数字を入力する(縦向きは各行で直接入力) */
  function invSel(name) {
    inv.sel = name;
    Object.keys(inv.rows).forEach(function (k) { inv.rows[k].el.classList.toggle("sel", k === name); });
    paintPane();
  }
  function paintPane() {
    var r = inv.sel && inv.rows[inv.sel], pane = $("ipane");
    pane.hidden = ed.on || !r;
    if (ed.on || !r) return;
    var can = !!(who.account && who.can), it = r.item, has = r.name in inv.drafts;
    var dirty = has && it && inv.drafts[r.name] !== it.remaining;
    $("ipNm").textContent = r.name;
    $("ipCur").textContent = it ? ("いま " + it.remaining + " 食 / 初期 " + it.initial) : "";
    $("ipNum").textContent = has ? String(inv.drafts[r.name]) : (it ? String(it.remaining) : "");
    $("ipVal").classList.toggle("dirty", !!dirty); $("ipVal").classList.toggle("idle", !has);
    Array.prototype.forEach.call(pane.querySelectorAll(".ikey"), function (b) { b.disabled = !can; });
    $("ipGo").disabled = !(can && dirty);
    $("ipHint").className = r.hint.className.indexOf("err") >= 0 ? "err" : "";
    $("ipHint").textContent = r.hint.textContent;
  }
  function paneKey(k) {
    var r = inv.sel && inv.rows[inv.sel];
    if (!r || !(who.account && who.can)) return;
    var cur = (r.name in inv.drafts) ? String(inv.drafts[r.name]) : "";
    if (k === "C") { r.setDraft(null); return; }
    if (k === "B") { cur = cur.slice(0, -1); r.setDraft(cur === "" ? null : parseInt(cur, 10)); return; }
    cur = (cur + k).replace(/^0+(?=[0-9])/, "").slice(0, 5);
    r.setDraft(parseInt(cur, 10));
  }
  (function buildPane() {
    [["7", "7"], ["8", "8"], ["9", "9"], ["4", "4"], ["5", "5"], ["6", "6"], ["1", "1"], ["2", "2"], ["3", "3"], ["C", "C"], ["0", "0"], ["B", "\u232b"]].forEach(function (k) {
      var b = document.createElement("button"); b.type = "button"; b.className = "ikey"; b.textContent = k[1]; b.setAttribute("aria-label", k[0] === "B" ? "ひとつ消す" : (k[0] === "C" ? "入力を取り消す" : k[0]));
      b.addEventListener("click", function () { paneKey(k[0]); });
      $("ipKeys").appendChild(b);
    });
    [["10", "+10"], ["1", "+1"], ["-1", "-1"], ["-10", "-10"]].forEach(function (k) {
      var b = document.createElement("button"); b.type = "button"; b.className = "ikey st"; b.textContent = k[1];
      b.addEventListener("click", function () {
        var r = inv.sel && inv.rows[inv.sel]; if (!r || !(who.account && who.can)) return;
        var base = (r.name in inv.drafts) ? inv.drafts[r.name] : (r.item ? r.item.remaining : 0);
        r.setDraft(Math.max(0, Math.min(99999, base + parseInt(k[0], 10))));
      });
      $("ipSide").appendChild(b);
    });
    var go = document.createElement("button"); go.type = "button"; go.className = "ikey go"; go.id = "ipGo"; go.textContent = "確定";
    go.addEventListener("click", function () { var r = inv.sel && inv.rows[inv.sel]; if (r) invSend(r); });
    $("ipSide").appendChild(go);
  })();
  function invRow(name) {
    var el = document.createElement("div"); el.className = "irow";
    el.innerHTML = '<div class="l1"><div class="nm"></div><div class="cur"></div></div>' +
      '<div class="l2"><button class="st" data-d="-10" type="button">-10</button><button class="st" data-d="-1" type="button">-1</button>' +
      '<input inputmode="numeric" pattern="[0-9]*" maxlength="5" aria-label="残数"><button class="st" data-d="1" type="button">+1</button><button class="st" data-d="10" type="button">+10</button>' +
      '<button class="go" type="button">確定</button></div><div class="hint"></div>';
    el.querySelector(".nm").textContent = name;
    var r = {el: el, name: name, cur: el.querySelector(".cur"), input: el.querySelector("input"), go: el.querySelector(".go"), hint: el.querySelector(".hint"), item: null};
    inv.rows[name] = r;
    el.addEventListener("click", function () { if (ed.on) edSel(name); else invSel(name); });
    var setDraft = function (v) {
      if (v === null) { delete inv.drafts[name]; r.input.value = r.item ? String(r.item.remaining) : ""; }
      else { inv.drafts[name] = v; r.input.value = String(v); }
      invPaint(r);
    };
    r.setDraft = setDraft;
    Array.prototype.forEach.call(el.querySelectorAll(".st"), function (b) {
      b.addEventListener("click", function () {
        var base = (name in inv.drafts) ? inv.drafts[name] : (r.item ? r.item.remaining : 0);
        setDraft(Math.max(0, Math.min(99999, base + parseInt(b.getAttribute("data-d"), 10))));
      });
    });
    r.input.addEventListener("input", function () {
      var t = r.input.value.replace(/[^0-9]/g, "").slice(0, 5);
      if (t !== r.input.value) r.input.value = t;
      if (t === "") { r.hint.textContent = ""; delete inv.drafts[name]; invPaint(r); return; }
      inv.drafts[name] = parseInt(t, 10); invPaint(r);
    });
    r.go.addEventListener("click", function () { invSend(r); });
    return el;
  }
  function invPaint(r) {
    var can = who.account && who.can;
    var dirty = (r.name in inv.drafts) && r.item && inv.drafts[r.name] !== r.item.remaining;
    r.el.classList.toggle("dirty", !!dirty);
    r.go.disabled = !(can && dirty);
    Array.prototype.forEach.call(r.el.querySelectorAll(".st"), function (b) { b.disabled = !can; });
    r.input.disabled = !can;
    if (!can && !dirty) r.hint.className = "hint";
    if (dirty && r.item && inv.drafts[r.name] > r.item.initial) { r.hint.className = "hint"; r.hint.textContent = "初期数(" + r.item.initial + ")より多いので、初期数も増やします"; }
    else if (!dirty && r.hint.getAttribute("data-sent") !== "1") r.hint.textContent = can ? "" : "入力するにはログインしてください";
    if (r.name === inv.sel) paintPane();
  }
  function invPaintAll() { Object.keys(inv.rows).forEach(function (k) { invPaint(inv.rows[k]); }); }

  /* ---- メニュー編集: 追加 / 名前の変更 / 初期食数の変更 / 削除。確定を押すまで送らない。送るのは POST /display/api/menu/edit(1回に1操作) ---- */
  var ed = {on: false, sel: undefined, busy: false, grace: 0};     /* sel: undefined=未選択, null=新しく追加, 文字=その名前のメニューを編集 */
  function edItems() { return inv.items || []; }
  function edFind(name) { var l = edItems(); for (var i = 0; i < l.length; i++) if (l[i].name === name) return l[i]; return null; }
  function edMsg(t, cls) { $("eMsg").textContent = t || ""; $("eMsg").className = cls || ""; }
  function edBuildAddRow(host, n) {
    var el = document.createElement("div"); el.className = "irow addrow"; el.id = "addRow";
    var full = n >= 12;
    el.textContent = full ? "メニューは最大12品までです" : "＋ メニューを追加";
    if (full) el.classList.add("full");
    el.addEventListener("click", function () { if (!full) edSel(null); });
    host.appendChild(el);
  }
  function edEnter() {
    ed.on = true; ed.sel = undefined; $("v-inventory").setAttribute("data-edit", "on");
    $("ihTitle").textContent = "メニュー編集"; $("editToggle").textContent = "← 残数入力にもどる";
    $("ilist").setAttribute("data-key", "");     /* 一覧を作り直す(追加の行を足す) */
    renderInventory({menu: {items: edItems()}});
    var l = edItems(); if (l.length) edSel(l[0].name); else edSel(null);
  }
  function edExit() {
    if (!ed.on) return;
    ed.on = false; ed.sel = undefined; $("v-inventory").removeAttribute("data-edit");
    $("ihTitle").textContent = "残数入力"; $("editToggle").textContent = "メニュー編集";
    $("epane").hidden = true; $("delSheet").hidden = true;
    try { document.activeElement && document.activeElement.blur(); } catch (e) {}
    $("ilist").setAttribute("data-key", "");
    renderInventory({menu: {items: edItems()}});
  }
  $("editToggle").addEventListener("click", function () { if (ed.on) edExit(); else edEnter(); });
  function edSel(name, keepMsg) {
    ed.sel = name; if (!keepMsg) edMsg("");
    $("eName").value = name === null ? "" : name;
    var it = name === null ? null : edFind(name);
    $("eInit").value = it ? String(it.initial) : "";
    $("eName").classList.remove("bad"); $("eInit").classList.remove("bad");
    edSync();
    if (name === null) setTimeout(function () { try { $("eName").focus(); } catch (e) {} }, 60);
  }
  /* 一覧の選択のしるし・右パネルの見出しを、いまの状態に合わせる(入力中の文字は書き換えない) */
  function edSync() {
    var rows = inv.rows;
    Object.keys(rows).forEach(function (k) { rows[k].el.classList.toggle("sel", k === ed.sel); });
    var add = $("addRow"); if (add) add.classList.toggle("sel", ed.sel === null);
    var pane = $("epane");
    if (ed.sel === undefined || (ed.sel !== null && !edFind(ed.sel))) {
      if (ed.sel !== undefined && ed.sel !== null && !edFind(ed.sel)) {
        if (Date.now() < ed.grace) return;     /* いま送った変更が、状態に届くのを待っている */
        ed.sel = undefined;                    /* 他で削除された・名前が変わった */
      }
      pane.hidden = true; return;
    }
    pane.hidden = false;
    var it = ed.sel === null ? null : edFind(ed.sel);
    $("eTitle").textContent = it ? "メニューを編集" : "メニューを追加";
    $("eSub").textContent = it ? ("いま " + it.remaining + " 食 / 売れた " + it.sold + " 食") : "";
    $("eDel").hidden = !it; $("eGo").textContent = it ? "確定" : "追加する";
    $("eNote").textContent = it
      ? "名前を変えても、売れた数と残数は引き継ぎます。初期食数を変えると、残数 = 初期食数 − 売れた数 になります。"
      : "追加したメニューは、売れた数0・残数 = 初期食数で始まります。";
  }
  function edPost(op, tries) {
    return fetch("/display/api/menu/edit", {method: "POST", headers: {"Content-Type": "application/json"}, credentials: "same-origin", body: JSON.stringify(op)})
      .then(function (res) { return res.json().catch(function () { return {}; }).then(function (d) { return [res.status, d]; }); })
      .then(function (a) {
        if (a[0] === 409 && tries > 0) return new Promise(function (ok) { setTimeout(ok, 1500); }).then(function () { return edPost(op, tries - 1); });   /* 直前の変更の反映待ち */
        return a;
      });
  }
  function edFail(a) {
    if (a[0] === 401) { edMsg("ログインが切れました。画面を読み込み直します", "err"); authLost(); return; }
    edMsg((a[1] && (a[1].message || a[1].error)) || ("失敗しました(" + a[0] + ")"), "err");
  }
  /* 送った変更が状態に届いたら(最大約8秒)、入力欄を最新の値にそろえる */
  function edAfter(name, init) {
    var n = 0;
    (function t() {
      if (!ed.on || ed.sel !== name) return;
      var it = edFind(name);
      if ((it && it.initial === init) || ++n > 13) { if (it) edSel(name, true); return; }
      setTimeout(t, 600);
    })();
  }
  function edBusy(b) { ed.busy = b; $("eGo").disabled = b; $("eDel").disabled = b; $("editToggle").disabled = b; }
  $("eGo").addEventListener("click", function () {
    if (ed.busy || ed.sel === undefined) return;
    var nm = $("eName").value.replace(/^\s+|\s+$/g, ""), iv = $("eInit").value.replace(/\s+/g, "");
    $("eName").classList.toggle("bad", !nm || nm.length > 40); $("eInit").classList.toggle("bad", !/^[0-9]{1,5}$/.test(iv));
    if (!nm) { edMsg("メニュー名を入力してください", "err"); return; }
    if (nm.length > 40) { edMsg("メニュー名は40文字までです", "err"); return; }
    if (!/^[0-9]{1,5}$/.test(iv)) { edMsg("初期食数は0〜99999の数字で入力してください", "err"); return; }
    var init = parseInt(iv, 10), old = ed.sel, it = old === null ? null : edFind(old), ops = [];
    if (!it) ops.push({op: "add", name: nm, initial: init});
    else {
      if (nm !== old) ops.push({op: "rename", name: old, new_name: nm});
      if (init !== it.initial) ops.push({op: "set_initial", name: nm, initial: init});
      if (!ops.length) { edMsg("変更がありません", ""); return; }
    }
    edBusy(true); edMsg("送信中…", "");
    var step = function (k) {
      if (k >= ops.length) return Promise.resolve(null);
      return edPost(ops[k], 6).then(function (a) {
        if (a[0] !== 200 || !a[1].ok) return a;
        if (ops[k].op === "rename") ed.sel = ops[k].new_name;
        if (ops[k].op === "add") ed.sel = ops[k].name;
        return step(k + 1);
      });
    };
    step(0).then(function (a) {
      edBusy(false);
      if (a) { edFail(a); return; }
      edMsg(it ? "反映しました" : "追加しました", "ok"); pollNow();
      ed.grace = Date.now() + 9000; edAfter(ed.sel, init);
    }).catch(function () { edBusy(false); edMsg("つながりません。もう一度押してください", "err"); });
  });
  $("eDel").addEventListener("click", function () {
    if (ed.busy || !ed.sel) return;
    var it = edFind(ed.sel); if (!it) return;
    $("delText").textContent = "「" + it.name + "」を一覧から消します(売れた数 " + it.sold + " 食・残数 " + it.remaining + " 食の記録も、この一覧からなくなります)。元に戻すには、同じ名前で追加し直します。";
    $("delSheet").hidden = false;
  });
  $("delNo").addEventListener("click", function () { $("delSheet").hidden = true; });
  $("delYes").addEventListener("click", function () {
    var name = ed.sel; $("delSheet").hidden = true;
    if (!name || ed.busy) return;
    edBusy(true); edMsg("削除しています…", "");
    edPost({op: "delete", name: name}, 6).then(function (a) {
      edBusy(false);
      if (a[0] === 200 && a[1].ok) { ed.sel = undefined; $("epane").hidden = true; edMsg(""); pollNow(); }
      else edFail(a);
    }).catch(function () { edBusy(false); edMsg("つながりません。もう一度押してください", "err"); });
  });
  ["eName", "eInit"].forEach(function (id) {
    $(id).addEventListener("input", function () {
      if (id === "eInit") { var t = $(id).value.replace(/[^0-9]/g, "").slice(0, 5); if (t !== $(id).value) $(id).value = t; }
      $(id).classList.remove("bad"); edMsg("");
    });
    $(id).addEventListener("keydown", function (e) { if (e.key === "Enter") { e.preventDefault(); $(id).blur(); } });
    $(id).addEventListener("focus", function () { setTimeout(function () { try { $(id).scrollIntoView({block: "nearest"}); } catch (e) {} }, 300); });
  });
  /* キーボードを閉じたあと、iOSが押し上げたままにする表示位置を戻す */
  document.addEventListener("focusout", function () { setTimeout(function () { if (!/INPUT|TEXTAREA/.test((document.activeElement || {}).tagName || "")) window.scrollTo(0, 0); }, 80); });
  function invSend(r) {
    var v = inv.drafts[r.name];
    if (typeof v !== "number") return;
    r.go.disabled = true; r.hint.className = "hint"; r.hint.textContent = "送信中…"; r.hint.setAttribute("data-sent", "1"); paintPane();
    fetch("/display/api/inventory", {method: "POST", headers: {"Content-Type": "application/json"}, credentials: "same-origin", body: JSON.stringify({menu: r.name, remaining: v})})
      .then(function (res) { return res.json().catch(function () { return {}; }).then(function (d) { return [res.status, d]; }); })
      .then(function (a) {
        var code = a[0], d = a[1];
        if (code === 200 && d.ok) { delete inv.drafts[r.name]; r.hint.textContent = "反映しました(" + v + "食)"; pollNow(); setTimeout(function () { r.hint.removeAttribute("data-sent"); }, 4000); }
        else if (code === 401) { r.hint.className = "hint err"; r.hint.textContent = "ログインが切れました。画面を読み込み直します"; authLost(); }
        else { r.hint.className = "hint err"; r.hint.textContent = d.message || d.error || ("失敗しました(" + code + ")"); }
        invPaint(r);
      }).catch(function () { r.hint.className = "hint err"; r.hint.textContent = "つながりません。もう一度押してください"; invPaint(r); });
  }

  /* =========================== survey(アンケート。ラズパイ版 mirs_robot_survey.html の移植) =========================== */
  var KEY = "mirs_robot_survey";
  function loadStore() {
    var def = {records: [], condition: (S.conditions[0] || ""), location: ""};
    try { return Object.assign(def, JSON.parse(localStorage.getItem(KEY) || "{}")); } catch (e) { return def; }
  }
  function saveStore() { try { localStorage.setItem(KEY, JSON.stringify(store)); } catch (e) {} }
  var store = loadStore();
  if (S.conditions.indexOf(store.condition) < 0) store.condition = S.conditions[0] || "";
  var ans = {}, idx = 0, idleTimer = null, doneTimer = null, beatTimer = null, startedAt = null, lastFinishedId = null, advTimer = null;
  $("idleTitle").textContent = S.idleTitle; $("idleNote").textContent = S.idleNote || ""; $("startLabel").textContent = S.startLabel; $("thanks").textContent = S.thanks;

  function post(url, obj) {
    return fetch(url, {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(obj), credentials: "same-origin"});
  }
  function hook(event) { post("/display/api/survey/session", {event: event, condition: store.condition || ""}).catch(function () {}); }
  function showSv(id) { ["idle", "ask", "done"].forEach(function (s) { $(s).hidden = s !== id; }); }
  function todayCount() {
    var today = new Date().toDateString();
    return store.records.filter(function (r) { return new Date(r.submitted_at).toDateString() === today; }).length;
  }
  function updateIdle() {
    var n = todayCount();
    $("idleCount").hidden = n === 0;
    $("idleCount").textContent = "今日 " + n + "人が こたえてくれました！";
  }
  function surveyEnter() { updateIdle(); showSv("idle"); sync(); }
  function surveyLeave() { clearTimeout(idleTimer); clearTimeout(doneTimer); clearInterval(beatTimer); beatTimer = null; ans = {}; idx = 0; }
  function surveyState(d) {
    $("stTotal").textContent = typeof d.count === "number" ? d.count : "-";
    /* サーバが放棄(通知途絶え・最長)にしたのにまだ質問画面が出ているときは、待機へ戻す */
    if (!$("ask").hidden && d.session_state !== "answering" && d.session_state !== "saved" && Date.now() - startedAt > 15000) toIdle();
  }
  function poke() {
    clearTimeout(idleTimer);
    if (!$("ask").hidden) idleTimer = setTimeout(function () { hook("end"); toIdle(); }, S.idleTimeoutSec * 1000);
  }
  ["pointerdown", "keydown"].forEach(function (ev) { document.addEventListener(ev, function () { if (mode === "survey") poke(); }, {passive: true}); });
  function toIdle() { clearTimeout(idleTimer); clearTimeout(doneTimer); clearInterval(beatTimer); beatTimer = null; ans = {}; idx = 0; updateIdle(); showSv("idle"); }
  function start() {
    ans = {}; idx = 0; startedAt = Date.now();
    hook("start");
    clearInterval(beatTimer); beatTimer = setInterval(function () { if (!$("ask").hidden) hook("beat"); }, 5000);   /* 回答中は5秒ごとに「まだ回答中」と知らせる */
    showSv("ask"); render(); poke();
  }
  $("idle").addEventListener("click", start);
  $("idle").addEventListener("keydown", function (e) { if (e.key === "Enter" || e.key === " ") start(); });

  function face(level) {
    var curve = [-10, -5, 0, 5, 10][level - 1], mouth = "M30 " + (66 - curve / 2) + " Q50 " + (66 + curve) + " 70 " + (66 - curve / 2);
    return '<svg viewBox="0 0 100 100" aria-hidden="true"><circle cx="50" cy="50" r="44" fill="none" stroke="currentColor" stroke-width="6"/><circle cx="35" cy="40" r="6" fill="currentColor"/><circle cx="65" cy="40" r="6" fill="currentColor"/><path d="' + mouth + '" fill="none" stroke="currentColor" stroke-width="6" stroke-linecap="round"/></svg>';
  }
  function render() {
    var qs = S.questions, q = qs[idx];
    $("dots").innerHTML = qs.map(function (_, i) { return '<i class="' + (i <= idx ? "on" : "") + '"></i>'; }).join("");
    $("qno").textContent = (idx + 1) + " / " + qs.length;
    $("bubble").textContent = idx === qs.length - 1 ? "ラスト！" : "あと" + (qs.length - idx) + "問！";
    var w = $("qwrap"); w.textContent = "";
    w.className = "qwrap" + (q.type === "scale" ? " is-scale" : "");
    var h = document.createElement("h2"); h.className = "qlabel"; h.textContent = q.label; w.appendChild(h);
    if (q.type === "scale") {
      var min = q.min != null ? q.min : 1, max = q.max != null ? q.max : 5, useFace = q.faces && min === 1 && max === 5;
      var g = document.createElement("div"); g.className = "scale"; g.style.setProperty("--n", max - min + 1);
      for (var v = min; v <= max; v++) (function (v) {
        var b = document.createElement("button"); b.type = "button"; b.className = "tile" + (ans[q.id] === v ? " sel" : "");
        b.setAttribute("aria-label", String(v)); b.setAttribute("data-v", String(v));
        b.innerHTML = (useFace ? face(v) : "") + "<span>" + v + "</span>";
        b.addEventListener("click", function () { ans[q.id] = v; mark(g, b); advanceSoon(); });
        g.appendChild(b);
      })(v);
      w.appendChild(g);
      var e = document.createElement("div"); e.className = "ends"; e.innerHTML = "<span></span><span></span>";
      e.children[0].textContent = q.minLabel || ""; e.children[1].textContent = q.maxLabel || ""; w.appendChild(e);
    } else {
      var g2 = document.createElement("div"); g2.className = "tiles"; g2.style.setProperty("--n", q.options.length);
      q.options.forEach(function (o) {
        var b = document.createElement("button"); b.type = "button"; b.className = "tile";
        var ck = document.createElement("span"); ck.className = "ck"; ck.textContent = "✓"; ck.setAttribute("aria-hidden", "true");
        var t = document.createElement("span"); t.textContent = o;
        if (q.type === "multi") b.appendChild(ck);
        b.appendChild(t);
        var cur = ans[q.id];
        if (q.type === "multi" ? (cur || []).indexOf(o) >= 0 : cur === o) b.classList.add("sel");
        b.addEventListener("click", function () {
          if (q.type === "multi") {
            var s = (ans[q.id] || []).slice(), i = s.indexOf(o);
            if (i >= 0) s.splice(i, 1); else s.push(o);
            ans[q.id] = q.options.filter(function (x) { return s.indexOf(x) >= 0; });
            b.classList.toggle("sel", s.indexOf(o) >= 0);
            limit(g2, q); nav();
          } else { ans[q.id] = o; mark(g2, b); advanceSoon(); }
        });
        g2.appendChild(b);
      });
      w.appendChild(g2);
      if (q.type === "multi") limit(g2, q);
    }
    nav();
  }
  function mark(g, b) { Array.prototype.forEach.call(g.querySelectorAll(".tile"), function (x) { x.classList.toggle("sel", x === b); }); }
  function limit(g, q) {
    if (!q.maxSelect) return;
    var full = (ans[q.id] || []).length >= q.maxSelect;
    Array.prototype.forEach.call(g.querySelectorAll(".tile"), function (b) { b.disabled = full && !b.classList.contains("sel"); });
  }
  function nav() {
    var qs = S.questions, q = qs[idx], last = idx === qs.length - 1;
    /* 「1つ前にもどる」「さいしょにもどる」は全問に出す(答えは消さない)。必須の答えが無くても押せる。
       1問目: 「1つ前」= 待機画面へ。「さいしょ」= すでに最初なので押せない(薄く表示) */
    $("back").hidden = !S.showBack;
    $("first").hidden = !S.showBack;
    $("first").disabled = idx === 0;
    $("skip").hidden = q.required !== false;   /* 必須の設問(r2は全問)はとばせない */
    var answered = q.type === "multi" ? (ans[q.id] || []).length > 0 : ans[q.id] !== undefined;
    /* 答えるまで「えらんでね」(押せない)。選んだら自動では進まず、「つぎへ」を押して進む(kousensai_survey.html と同じ) */
    $("next").hidden = false;
    $("next").disabled = !answered && q.required !== false;
    $("next").textContent = !answered && q.required !== false ? "えらんでね" : last ? "おわる ✓" : "つぎへ →";
  }
  function advanceSoon() { clearTimeout(advTimer); nav(); }
  function next() { clearTimeout(advTimer); if (idx < S.questions.length - 1) { idx++; render(); } else finish(); }
  $("next").addEventListener("click", next);
  $("skip").addEventListener("click", function () { delete ans[S.questions[idx].id]; next(); });
  $("back").addEventListener("click", function () { clearTimeout(advTimer); if (idx > 0) { idx--; render(); } else { hook("end"); toIdle(); } });   /* 1問目から戻ると待機画面(回答は捨てる。送信しない) */
  $("first").addEventListener("click", function () { clearTimeout(advTimer); if (idx > 0) { idx = 0; render(); } });

  var doneAt = 0;
  $("done").addEventListener("click", function () { if (Date.now() - doneAt > 1500) toIdle(); });   /* 完了画面は触れば待機へ戻る(誤タップ防止に1.5秒は無視) */

  function uuid() { return (window.crypto && crypto.randomUUID) ? crypto.randomUUID() : Date.now().toString(36) + Math.random().toString(36).slice(2) + Math.random().toString(36).slice(2); }
  function finish() {
    var answers = {};
    S.questions.forEach(function (q) { if (ans[q.id] !== undefined) answers[q.id] = ans[q.id]; });
    var rec = {response_id: uuid(), submitted_at: new Date().toISOString(), survey_version: S.version, mode: "robot", condition: store.condition || "",
      location: store.location || "", duration_sec: Math.round((Date.now() - startedAt) / 1000), answers: answers, _synced: false};
    store.records.push(rec);
    if (store.records.length > 500) store.records = store.records.filter(function (r) { return !r._synced; }).concat(store.records.filter(function (r) { return r._synced; }).slice(-100));
    saveStore();
    lastFinishedId = rec.response_id;
    clearTimeout(idleTimer); clearInterval(beatTimer); beatTimer = null;
    showSv("done"); doneAt = Date.now();
    $("doneCount").textContent = "あなたは今日 " + todayCount() + "人目の回答者です";
    $("doneNote").textContent = answers.impression >= 4 ? "たのしんでくれて うれしいな！"
      : answers.impression !== undefined ? "いただいた声で もっとよくなるよ！" : "こたえてくれて ありがとう！";
    doneTimer = setTimeout(toIdle, S.doneReturnSec * 1000);
    sync();
  }
  function payload(r) { var p = {}; Object.keys(r).forEach(function (k) { if (k !== "_synced") p[k] = r[k]; }); return p; }
  var syncing = false;
  function sendOne(r) {
    var url = "/display/api/survey/answer";
    /* 今の回答者の完了(直前に終えた回答)以外の再送は ?replay=1 を付け、保存完了の通知にしない */
    if (!(r.response_id === lastFinishedId && $("ask").hidden)) url += "?replay=1";
    return fetch(url, {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(payload(r)), credentials: "same-origin"})
      .then(function (res) { if (res.ok) return true; if (res.status === 400) { r._bad = true; return true; } return false; });   /* 400=サーバが受け付けない形。何度送っても通らないので諦める */
  }
  function sync() {
    if (syncing) return Promise.resolve();
    syncing = true;
    var pend = store.records.filter(function (r) { return !r._synced; });
    var chain = Promise.resolve(true);
    pend.forEach(function (r) {
      chain = chain.then(function (okSoFar) {
        if (!okSoFar) return false;
        return sendOne(r).then(function (ok) { if (ok) { r._synced = true; saveStore(); } return ok; }).catch(function () { return false; });
      });
    });
    return chain.then(function () { syncing = false; stats(); }, function () { syncing = false; stats(); });
  }
  window.addEventListener("online", sync);
  setInterval(sync, 60000);

  /* 隠し管理 */
  var taps = [];
  $("adminHit").addEventListener("click", function (e) {
    e.stopPropagation();
    var now = Date.now(); taps = taps.filter(function (t) { return now - t < 3000; }); taps.push(now);
    if (taps.length >= 5) { taps = []; openPin(); }
  });
  var pin = "";
  ["1", "2", "3", "4", "5", "6", "7", "8", "9", "取消", "0", "OK"].forEach(function (k) {
    var b = document.createElement("button"); b.type = "button"; b.className = "btn"; b.textContent = k;
    b.addEventListener("click", function () {
      if (k === "取消") { $("pinSheet").hidden = true; return; }
      if (k === "OK") {
        if (pin === String(S.adminPin)) { $("pinSheet").hidden = true; openAdmin(); } else { $("pinMsg").textContent = "PIN が違います"; pin = ""; }
      } else if (pin.length < 8) pin += k;
      $("pinShow").textContent = new Array(pin.length + 1).join("●");
    });
    $("pinPad").appendChild(b);
  });
  function openPin() { pin = ""; $("pinShow").textContent = ""; $("pinMsg").textContent = ""; $("pinSheet").hidden = false; clearTimeout(idleTimer); clearTimeout(doneTimer); }
  S.conditions.forEach(function (c) { var o = document.createElement("option"); o.value = o.textContent = c; $("condSel").appendChild(o); });
  function stats() {
    var today = new Date().toDateString();
    $("stToday").textContent = store.records.filter(function (r) { return new Date(r.submitted_at).toDateString() === today; }).length;
    $("stPending").textContent = store.records.filter(function (r) { return !r._synced; }).length;
  }
  function openAdmin() { $("condSel").value = store.condition; $("locIn").value = store.location || ""; $("aMsg").textContent = ""; stats(); $("adminSheet").hidden = false; }
  $("condSel").addEventListener("change", function (e) { store.condition = e.target.value; saveStore(); hook("beat"); });
  $("locIn").addEventListener("input", function (e) { store.location = e.target.value.slice(0, 80); saveStore(); });
  $("aSync").addEventListener("click", function () { $("aMsg").textContent = "送信中…"; sync().then(function () { $("aMsg").textContent = "未送信 " + store.records.filter(function (r) { return !r._synced; }).length + " 件"; }); });
  $("aClose").addEventListener("click", function () { $("adminSheet").hidden = true; if (!$("done").hidden) toIdle(); poke(); });

  /* =========================== 画面を点けたままにする / 起動 =========================== */
  var lock = null;
  function wakeLock() {
    try {
      if (!("wakeLock" in navigator) || document.hidden) return;
      navigator.wakeLock.request("screen").then(function (l) { lock = l; l.addEventListener("release", function () { lock = null; }); }).catch(function () {});
    } catch (e) {}
  }
  document.addEventListener("pointerdown", function () { if (!lock) wakeLock(); }, {passive: true});
  ["gesturestart", "gesturechange"].forEach(function (ev) { document.addEventListener(ev, function (e) { e.preventDefault(); }); });
  document.addEventListener("contextmenu", function (e) { if (!/INPUT|TEXTAREA/.test((e.target || {}).tagName || "")) e.preventDefault(); });

  applySize();
  wakeLock();
  poll();
})();
</script>
</body>
</html>
"""


def _survey_json() -> str:
    # <script> の中に入れるので </ や行区切り文字を無害化する
    return (json.dumps(SURVEY_DEF, ensure_ascii=False)
            .replace("</", "<\\/").replace("\u2028", "\\u2028").replace("\u2029", "\\u2029"))


@functools.lru_cache(maxsize=1)
def build_id() -> str:
    """画面のプログラムの版。サーバのHTMLが変わるとこの値も変わり、開きっぱなしのiPadが自動で読み込み直す。"""
    return hashlib.sha1((_TEMPLATE + _survey_json() + FACE_HTML).encode("utf-8")).hexdigest()[:10]


def _face_json() -> str:
    """顔のHTMLを、インラインの `<script>` の中に置ける JS の文字列リテラルにする(`</script>` や `<!--` で親のページを壊さない)。"""
    return json.dumps(FACE_HTML, ensure_ascii=True).replace("</", "<\\/").replace("<!--", "<\\!--")


@functools.lru_cache(maxsize=1)
def build_display_html() -> bytes:
    html = (_TEMPLATE.replace("__SURVEY_JSON__", _survey_json())
            .replace("__FACE_JSON__", _face_json())
            .replace("__BUILD__", build_id())
            .replace("__LOGO__", LOGO_DATA_URI))
    return html.encode("utf-8")


# ---------------------------------------------------------------------------
# ログイン画面(最初に開いたときだけ。状態・設問・メニューなどは一切含まない。厨房サイトと同じアカウントでログイン)
# 横向き: 左=ロゴと説明、右=入力。キーボードが下半分を隠すので、入力欄とボタンは上寄せ。
# ---------------------------------------------------------------------------
_LOGIN_TEMPLATE = r"""<!doctype html>
<html lang="ja" data-orient="landscape">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,minimum-scale=1,maximum-scale=1,viewport-fit=cover,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="apple-mobile-web-app-title" content="のこさへん">
<meta name="format-detection" content="telephone=no">
<meta name="theme-color" content="#0f2e24">
<link rel="manifest" href="/display/manifest.webmanifest">
<link rel="apple-touch-icon" href="/display/icon.png">
<title>のこさへん 頭部ディスプレイ ログイン</title>
<style>
:root{
  --app-w:100vw; --app-h:100vh;
  --u:calc(min(var(--app-w),var(--app-h)) / 100);
  --sat:env(safe-area-inset-top,0px); --sar:env(safe-area-inset-right,0px); --sab:env(safe-area-inset-bottom,0px); --sal:env(safe-area-inset-left,0px);
  --f:"Hiragino Sans","Hiragino Kaku Gothic ProN","Noto Sans CJK JP","Noto Sans JP","Yu Gothic UI",system-ui,-apple-system,sans-serif;
  --g0:#0f2e24; --g1:#124739; --g2:#176b56; --ink:#16211d; --muted:#53635c; --line:#c9d3d8; --accent:#0d6b58; --warn:#b4432d; --hi:#f2b43c;
  --tap:max(56px,calc(var(--u) * 7));
}
@supports (height:100dvh){ :root{ --app-h:100dvh; } }
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
[hidden]{display:none!important}
html,body{margin:0;height:100%;overflow:hidden;overscroll-behavior:none;-webkit-text-size-adjust:100%}
body{position:fixed;inset:0;font-family:var(--f);font-weight:500;color:#fff;background:var(--g0);touch-action:manipulation}
button,input{font:inherit}
.wrap{position:absolute;inset:0;display:grid;grid-template-columns:minmax(0,5fr) minmax(0,6fr);column-gap:calc(var(--u) * 5);align-items:start;
  padding:calc(var(--u) * 6 + var(--sat)) calc(var(--u) * 5 + var(--sar)) calc(var(--u) * 3 + var(--sab)) calc(var(--u) * 5 + var(--sal))}
.brand{display:flex;flex-direction:column;gap:calc(var(--u) * 2.4);padding-top:calc(var(--u) * 4)}
.brand img{width:calc(var(--u) * 16);height:calc(var(--u) * 16);border-radius:calc(var(--u) * 3)}
.brand h1{margin:0;font-size:calc(var(--u) * 5.6);font-weight:900;line-height:1.25}
.brand p{margin:0;font-size:calc(var(--u) * 2.8);line-height:1.6;opacity:.88}
form{background:#fff;color:var(--ink);border-radius:calc(var(--u) * 2.6);padding:calc(var(--u) * 3.4);display:grid;gap:calc(var(--u) * 2);box-shadow:0 6px 24px rgba(0,0,0,.28)}
form h2{margin:0;font-size:calc(var(--u) * 3.4);font-weight:900}
label{display:grid;gap:calc(var(--u) * .7);font-weight:800;font-size:max(15px,calc(var(--u) * 2.3))}
input{font-size:max(18px,calc(var(--u) * 3.2));min-height:var(--tap);padding:calc(var(--u) * .8) calc(var(--u) * 1.8);border-radius:calc(var(--u) * 1.4);border:3px solid var(--line);
  background:#fff;color:var(--ink);width:100%;-webkit-appearance:none;appearance:none}
input:focus{border-color:var(--accent);outline:none}
button{min-height:var(--tap);border:0;border-radius:calc(var(--u) * 1.6);background:var(--accent);color:#fff;font-weight:900;font-size:calc(var(--u) * 3.2);cursor:pointer}
button:disabled{opacity:.45}
#msg{min-height:1.5em;font-size:max(14px,calc(var(--u) * 2.2));color:var(--muted)}#msg.err{color:var(--warn);font-weight:800}
/* 縦向き(フォールバック): 1列 */
html[data-orient=portrait] .wrap{grid-template-columns:minmax(0,1fr);row-gap:calc(var(--u) * 3);padding-top:calc(var(--u) * 4 + var(--sat))}
html[data-orient=portrait] .brand{flex-direction:row;align-items:center;padding-top:0}
html[data-orient=portrait] .brand img{width:calc(var(--u) * 10);height:calc(var(--u) * 10)}
html[data-orient=portrait] .brand p{display:none}
</style>
</head>
<body>
<main class="wrap">
  <section class="brand">
    <img src="__LOGO__" alt="">
    <h1>のこさへん<br>頭部ディスプレイ</h1>
    <p>最初に1回だけ、ログインしてください。<br>厨房サイトと同じアカウント名とパスワードです。<br>ログインしたあとは、ロボットの指示で表示が自動で切り替わります。</p>
  </section>
  <form id="f" autocomplete="on" method="post" action="#">
    <h2>ログイン</h2>
    <label>アカウント名<input id="u" name="username" type="text" autocomplete="username" autocapitalize="off" autocorrect="off" spellcheck="false" enterkeyhint="next"></label>
    <label>パスワード<input id="p" name="password" type="password" autocomplete="current-password" enterkeyhint="go"></label>
    <button id="go" type="submit">ログイン</button>
    <div id="msg" role="status"></div>
  </form>
</main>
<script>
(function () {
  "use strict";
  var $ = function (id) { return document.getElementById(id); }, root = document.documentElement;
  function fit() {
    var w = window.innerWidth, h = window.innerHeight;
    root.style.setProperty("--app-w", w + "px"); root.style.setProperty("--app-h", h + "px");
    root.setAttribute("data-orient", h > w ? "portrait" : "landscape");
  }
  window.addEventListener("resize", fit); window.addEventListener("orientationchange", fit); fit();
  function msg(t, err) { $("msg").textContent = t || ""; $("msg").className = err ? "err" : ""; }
  /* ログインできたのに、また ここに戻ってきたとき(Cookieを保存できない設定など)の案内 */
  try {
    if (sessionStorage.getItem("nk_login_try") === "1") {
      sessionStorage.removeItem("nk_login_try");
      msg("ログインしましたが、この画面のままです。Safariの設定でCookieをブロックしていないか確認してください(ホーム画面から開いた場合は、そこでもう一度ログインが必要です)。", true);
    }
  } catch (e) { /* 使えなくても動く */ }
  var busy = false;
  $("f").addEventListener("submit", function (ev) {
    ev.preventDefault();
    if (busy) return;
    busy = true; $("go").disabled = true; msg("確認しています…", false);
    fetch("/display/api/login", {method: "POST", headers: {"Content-Type": "application/json"}, credentials: "same-origin",
        body: JSON.stringify({username: $("u").value, password: $("p").value})})
      .then(function (r) { return r.json().catch(function () { return {}; }).then(function (d) { return [r.status, d]; }); })
      .then(function (a) {
        busy = false; $("go").disabled = false; $("p").value = "";
        if (a[0] === 200 && a[1].ok) {
          msg("ログインしました", false);
          try { sessionStorage.setItem("nk_login_try", "1"); } catch (e) {}
          location.reload();
          return;
        }
        if (a[0] === 429) msg("失敗が続いたため、しばらくログインできません(約" + Math.max(1, Math.ceil(((a[1] && a[1].retry_after_sec) || 300) / 60)) + "分あとにもう一度)", true);
        else if (a[0] === 401) msg("アカウント名またはパスワードが違います", true);
        else msg("ログインできませんでした(" + a[0] + ")", true);
      })
      .catch(function () { busy = false; $("go").disabled = false; msg("つながりません。ネットワークを確認してください", true); });
  });
  $("u").addEventListener("keydown", function (e) { if (e.key === "Enter") { e.preventDefault(); $("p").focus(); } });
  /* キーボードを閉じたあと、iOSが押し上げたままにする表示位置を戻す */
  document.addEventListener("focusout", function () { setTimeout(function () { if (!/INPUT/.test((document.activeElement || {}).tagName || "")) window.scrollTo(0, 0); }, 80); });
})();
</script>
</body>
</html>
"""


@functools.lru_cache(maxsize=1)
def build_display_login_html() -> bytes:
    """未ログインのiPadに返すログイン画面。状態・設問・メニュー・管理用PINなどは含まない(ログインフォームだけ)。"""
    return _LOGIN_TEMPLATE.replace("__LOGO__", LOGO_DATA_URI).encode("utf-8")
