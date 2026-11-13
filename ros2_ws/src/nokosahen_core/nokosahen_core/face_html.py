"""顔モード(`face`)の顔のHTML。ユーザーが用意した robot_face_cute_blush.html を**一字も変えずに**そのまま持つ。

元のファイル: docs/robot_face_cute_blush.html (同じ内容。test_face_mode_20261108.py が一致を確かめる)。
iPad表示(display_page.py)は、これを iframe の srcdoc に入れて隔離して画面いっぱいに出す。
`GET /display/face` は、このHTMLをそのまま返す。
assets/ ディレクトリやpackage_dataを使わず、Pythonの文字列として持つ(colcon の --symlink-install でも通常インストールでも同じに読める)。
"""

FACE_HTML = r'''<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover,user-scalable=no">
<meta name="theme-color" content="#ffffff">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="default">
<title>ロボットの顔</title>
<style>
  :root { color-scheme: light; --ink:#08090b; }
  * { box-sizing:border-box; -webkit-tap-highlight-color:transparent; }
  html, body { margin:0; width:100%; height:100%; overflow:hidden; background:#fff; }
  body { display:grid; place-items:center; touch-action:manipulation; }
  #face { display:block; width:100vw; height:100dvh; background:#fff; }
  .eye { fill:var(--ink); }
  .glint { fill:#fff; }
  .line-eye { stroke:var(--ink); stroke-width:19; stroke-linecap:round; fill:none; }
  #mouth { fill:var(--ink); stroke:none; }
  .blush-line { stroke:#f5a0b6; stroke-width:10; stroke-linecap:round; fill:none; }
  .hide { visibility:hidden; }
  #face { cursor:pointer; }
</style>
</head>
<body>
<!-- SVG の viewBox と preserveAspectRatio で iPad の縦横どちらも同じ顔の比率に保ちます。 -->
<svg id="face" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1000 700" preserveAspectRatio="xMidYMid meet" role="img" aria-label="表情が自動で変わるロボットの顔">
  <g id="openEyes">
    <circle class="eye" cx="315" cy="280" r="79"/>
    <circle class="eye" cx="685" cy="280" r="79"/>
    <circle class="glint" cx="288" cy="249" r="21"/>
    <circle class="glint" cx="658" cy="249" r="21"/>
  </g>
  <g id="smileEyes" class="hide">
    <path class="line-eye" d="M231 300 Q315 190 399 300"/>
    <path class="line-eye" d="M601 300 Q685 190 769 300"/>
  </g>
  <g id="blinkEyes" class="hide">
    <path class="line-eye" d="M247 284 H383"/>
    <path class="line-eye" d="M617 284 H753"/>
  </g>
  <!-- 笑顔の間だけ現れる、左右対称のピンクの斜線 -->
  <g id="smileBlush" class="hide" aria-hidden="true">
    <path class="blush-line" d="M184 406 L202 379 M211 406 L229 379 M238 406 L256 379"/>
    <path class="blush-line" d="M816 406 L798 379 M789 406 L771 379 M762 406 L744 379"/>
  </g>
  <path id="mouth" d="M440 445 Q500 443 560 445 L560 455 Q500 459 440 455 Z"/>
</svg>
<script>
(() => {
  'use strict';
  const openEyes = document.getElementById('openEyes');
  const smileEyes = document.getElementById('smileEyes');
  const smileBlush = document.getElementById('smileBlush');
  const blinkEyes = document.getElementById('blinkEyes');
  const mouth = document.getElementById('mouth');
  const face = document.getElementById('face');
  // 口はすべて横方向に左右対称。角のない丸いシルエットに統一。
  const shapes = {
    closed: 'M470 456 Q500 454 530 456 Q539 456 539 466 Q539 476 530 476 L470 476 Q461 476 461 466 Q461 456 470 456 Z',
    small: 'M500 441 C517 441 530 453 530 471 C530 489 517 502 500 502 C483 502 470 489 470 471 C470 453 483 441 500 441 Z',
    big: 'M459 448 C470 439 530 439 541 448 C550 457 548 486 532 504 C523 515 512 521 500 521 C488 521 477 515 468 504 C452 486 450 457 459 448 Z',
    smile: 'M436 444 C460 455 479 460 500 460 C521 460 540 455 564 444 C567 442 570 446 567 451 C555 481 530 504 500 504 C470 504 445 481 433 451 C430 446 433 442 436 444 Z',
    surprised: 'M500 439 C519 439 534 455 534 475 C534 495 519 511 500 511 C481 511 466 495 466 475 C466 455 481 439 500 439 Z'
  };
  let expression='talk';
  let blink=false;
  let talkIndex=0;
  let specialUntil=0;
  let nextSpecial=performance.now()+3200;
  let nextBlink=performance.now()+1900;
  let lastTalk=0;
  const talkSequence=['closed','small','big','small','closed','small','big','small'];
  function show(el, yes) { el.classList.toggle('hide',!yes); }
  function render() {
    const isSmile=expression==='smile';
    show(smileEyes,isSmile);
    show(smileBlush,isSmile);
    show(blinkEyes,!isSmile && blink);
    show(openEyes,!isSmile && !blink);
    const shape=isSmile?'smile':expression==='surprise'?'surprised':talkSequence[talkIndex];
    mouth.setAttribute('d',shapes[shape]);
  }
  function special(kind, length=1200) {
    expression=kind;
    specialUntil=performance.now()+length;
    nextSpecial=specialUntil+2400+Math.random()*2300;
    blink=false;
    render();
  }
  function frame(now) {
    if (expression!=='talk' && now>=specialUntil) expression='talk';
    if (now>=nextSpecial && expression==='talk') special(Math.random()<0.8?'smile':'surprise',1000+Math.random()*650);
    if (now-lastTalk>=210) { lastTalk=now; talkIndex=(talkIndex+1)%talkSequence.length; }
    if (now>=nextBlink) { blink=true; nextBlink=now+1800+Math.random()*2400; setTimeout(()=>{blink=false;render();},155); }
    render();
    requestAnimationFrame(frame);
  }
  // 画面タップで笑顔。自動アニメーションも続きます。
  face.addEventListener('pointerdown',()=>special('smile',1400));
  render(); requestAnimationFrame(frame);
})();
</script>
</body>
</html>
'''
