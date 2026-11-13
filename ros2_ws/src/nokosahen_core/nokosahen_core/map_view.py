"""ダッシュボード(完成版/Stage2)の「地図(クリックでそこへ自動走行)」描画用の共有JS。

【2026-10-05】地図が小さく描かれる問題の修正。従来は地図グリッド全体
(例: 238x254セル=約12x13m)を固定の360x360(Stage2は400x400)のキャンバスへ
縦横別倍率で引き伸ばして描いていたため、(1)探索済みの範囲がグリッドの一部しか
占めないと更に小さく見える、(2)縦横比が崩れる、(3)パネルが大きくても拡大
できない、という状態だった。ここでは

- 表示ビューを「世界座標の中心(cx,cy)と縮尺(px/m)」で持ち、既知セルの外接矩形
  (+ロボット・軌跡・計画経路)にぴったり合わせる(全体表示)
- キャンバスの内部解像度をCSSの表示サイズ x devicePixelRatioに追従させる
  (縦横比を保ち、伸縮しない)
- マウスホイール/ピンチで拡大縮小、ドラッグで移動、「全体表示」で元に戻す

を行う。クリック→世界座標の変換は描画と同じビュー変換の逆変換
(mapViewPixelToWorld)を使う。ここを間違えるとロボットが別の場所へ向かうため、
純粋関数部分(MAP_VIEW_MATH_JS)をtest_map_view.pyでnodeに通して往復検証している。

【2026-10-06】表示の回転。SLAMの地図はx軸=スタート時の機体の向きなので壁が斜めに
見えることがある。ビューに回転角rot[rad、画面上で反時計回り]を足し、世界<->画面の
変換(mapViewWorldToPixel / mapViewPixelToWorld)の1か所だけに回転を入れた。描画・
目的地クリック・走行範囲の指定は全てこの変換を通るので常に整合する。世界座標系
(Nav2/SLAM/保存地図)は回さない。回転角はサーバー(/api/map_view)に保存され、
地図の保存/読み込みと一緒に<name>.view.jsonへ出し入れされる。UI(スライダー・
90°回転・0°に戻す・壁に合わせて自動/2点指定・方位マーカー)と、走行可能範囲の
回転矩形(表示上は軸平行に見える矩形)の描画・コーナーハンドルによる編集もここ。

【2026-10-07】「ロボットに現在位置を教える」(AMCLの初期位置)。地図上でロボットのいる場所を
クリックし、向きの方へドラッグ(または2点目をクリック)すると、確認バー経由で
POST /api/initial_pose {x, y, yaw_deg} を送る。画面->世界座標は他と同じ逆変換
(mapViewPixelToWorld)を通すので、表示回転があっても向きが正しく出る(mapPoseFromWorldPoints)。
/api/localization(AMCLの共分散から「位置の確かさ」)を購読して、位置の確かさのチップ・警告・
「不確かな時の目的地クリックは確認バー」を出す。操作用の要素(poseToolBtn等)が無いページ
(Stage2)では何もしない。

【2026-10-07】地図ピン(名前付きの場所)・目的地の精度向上。ピンはmapフレームの{x,y,yaw_deg,color,name}で、
描画・タップ判定(mapPinHit)・ドラッグでの向き指定はどれも同じ変換(mapViewWorldToPixel /
mapViewPixelToWorld、表示回転込み)を通る。ピンの追加・更新・削除・「行く」はサーバー(/api/pins/*)が
保存・検証し、「行く」は保存した姿勢をそのまま/goal_poseへ送る(クリックし直した座標は使わない)。
目的地は/api/goalに、任意でyaw_deg(到着時の向き)を付けられる(「向き指定でクリック」ON中のドラッグ)。
目的地のマーカー(クリック位置・サーバーが寄せた位置)・残り距離・到着後の誤差[cm]・中止ボタンは
/api/goal/status と /api/goal/cancel を使う。要素(pinList / goalProgressBar)が無いページ(Stage2)では何もしない。

使い方: ページHTMLの`__MAP_VIEW_JS__`をMAP_VIEW_JSで置換する。ページ側は
`drawBoundary()`(走行可能範囲の描画。drawBoundaryShape(currentBoundary)を呼ぶ)と
`let currentBoundary`/`let boundaryPickMode`を定義しておくこと。
"""

# 純粋関数(DOM非依存)。view = {cx, cy, scale, rot}: 画面中心の世界座標(m)・縮尺(px/m)・
# 表示回転(rad、省略=0)。画面座標は左上原点・y下向き(CSS px)、世界座標はy上向き。
MAP_VIEW_MATH_JS = r"""
  const MAP_VIEW_MIN_SCALE = 2;    // px/m
  const MAP_VIEW_MAX_SCALE = 600;  // px/m
  // 世界(x,y)を中心(cx,cy)まわりに画面上で反時計回りにrotだけ回し、縮尺して画面中心へ。
  function mapViewWorldToPixel(view, w, h, x, y) {
    const r = view.rot || 0, c = Math.cos(r), s = Math.sin(r);
    const dx = x - view.cx, dy = y - view.cy;
    return [w / 2 + (dx * c - dy * s) * view.scale, h / 2 - (dx * s + dy * c) * view.scale];
  }
  function mapViewPixelToWorld(view, w, h, px, py) {
    const r = view.rot || 0, c = Math.cos(r), s = Math.sin(r);
    const rx = (px - w / 2) / view.scale, ry = -(py - h / 2) / view.scale;
    return [view.cx + rx * c + ry * s, view.cy - rx * s + ry * c];
  }
  function mapClampScale(s) {
    return Math.max(MAP_VIEW_MIN_SCALE, Math.min(MAP_VIEW_MAX_SCALE, s));
  }
  // 回転後の座標系(u,v) = 世界を原点まわりにrotだけ回した座標。
  function mapRotUV(x, y, rot) {
    const c = Math.cos(rot), s = Math.sin(rot);
    return [x * c - y * s, x * s + y * c];
  }
  function mapUVToWorld(u, v, rot) {
    const c = Math.cos(rot), s = Math.sin(rot);
    return [u * c + v * s, -u * s + v * c];
  }
  // 回転後座標系での外接矩形ext={minU,maxU,minV,maxV}が余白込みでw x h(px)に収まる最大縮尺のビュー。
  function mapFitViewRot(ext, w, h, marginM, rot) {
    const spanU = Math.max(ext.maxU - ext.minU, 0) + 2 * marginM;
    const spanV = Math.max(ext.maxV - ext.minV, 0) + 2 * marginM;
    const sx = w / Math.max(spanU, 4);
    const sy = h / Math.max(spanV, 4);
    const wc = mapUVToWorld((ext.minU + ext.maxU) / 2, (ext.minV + ext.maxV) / 2, rot);
    return { cx: wc[0], cy: wc[1], scale: mapClampScale(Math.min(sx, sy)), rot: rot };
  }
  // 外接矩形b={minX,maxX,minY,maxY}(m)が余白込みでw x h(px)に収まる最大縮尺のビュー。
  // rot(rad)を渡すと、bの4隅を回した外接で合わせる。
  function mapFitView(b, w, h, marginM, rot) {
    const r = rot || 0;
    const ext = { minU: Infinity, maxU: -Infinity, minV: Infinity, maxV: -Infinity };
    for (const [x, y] of [[b.minX, b.minY], [b.maxX, b.minY], [b.maxX, b.maxY], [b.minX, b.maxY]]) {
      const uv = mapRotUV(x, y, r);
      ext.minU = Math.min(ext.minU, uv[0]); ext.maxU = Math.max(ext.maxU, uv[0]);
      ext.minV = Math.min(ext.minV, uv[1]); ext.maxV = Math.max(ext.maxV, uv[1]);
    }
    return mapFitViewRot(ext, w, h, marginM, r);
  }
  // 画面上の点(px,py)の下にある世界座標を動かさずに縮尺をfactor倍する。
  function mapZoomAt(view, w, h, px, py, factor) {
    const wp = mapViewPixelToWorld(view, w, h, px, py);
    const s = mapClampScale(view.scale * factor);
    const z = { cx: 0, cy: 0, scale: s, rot: view.rot || 0 };
    const o = mapViewPixelToWorld(z, w, h, px, py);
    return { cx: wp[0] - o[0], cy: wp[1] - o[1], scale: s, rot: view.rot || 0 };
  }
  // 画面上で内容を(dpx,dpy)px動かしたときのビュー(回転を考慮した平行移動)。
  function mapPanBy(view, dpx, dpy) {
    const z = { cx: 0, cy: 0, scale: view.scale, rot: view.rot || 0 };
    const d = mapViewPixelToWorld(z, 0, 0, dpx, dpy);
    return { cx: view.cx - d[0], cy: view.cy - d[1], scale: view.scale, rot: view.rot || 0 };
  }
  // /api/mapのm({width,height,resolution,origin_x,origin_y})とdata(各セル=値+1のバイト列、
  // 0=未知)から、既知セル(0以外)の外接矩形(世界座標m)を返す。無ければnull。
  function mapKnownBoundsOf(m, bytes) {
    let minC = m.width, maxC = -1, minR = m.height, maxR = -1;
    for (let r = 0; r < m.height; r++) {
      const base = r * m.width;
      for (let c = 0; c < m.width; c++) {
        if (bytes[base + c] > 0) {
          if (c < minC) minC = c;
          if (c > maxC) maxC = c;
          if (r < minR) minR = r;
          if (r > maxR) maxR = r;
        }
      }
    }
    if (maxC < 0) return null;
    const res = m.resolution;
    return {
      minX: m.origin_x + minC * res, maxX: m.origin_x + (maxC + 1) * res,
      minY: m.origin_y + minR * res, maxY: m.origin_y + (maxR + 1) * res,
    };
  }
  // 既知セルを回転後座標系(u,v)で見た外接矩形(セルの大きさ分の余裕込み)。無ければnull。
  function mapKnownExtentsRot(m, bytes, rot) {
    const c = Math.cos(rot), s = Math.sin(rot), res = m.resolution;
    let minU = Infinity, maxU = -Infinity, minV = Infinity, maxV = -Infinity, any = false;
    for (let r = 0; r < m.height; r++) {
      const base = r * m.width;
      const y = m.origin_y + (r + 0.5) * res;
      for (let col = 0; col < m.width; col++) {
        if (bytes[base + col] > 0) {
          const x = m.origin_x + (col + 0.5) * res;
          const u = x * c - y * s, v = x * s + y * c;
          if (u < minU) minU = u;
          if (u > maxU) maxU = u;
          if (v < minV) minV = v;
          if (v > maxV) maxV = v;
          any = true;
        }
      }
    }
    if (!any) return null;
    const pad = res / 2 * (Math.abs(c) + Math.abs(s));
    return { minU: minU - pad, maxU: maxU + pad, minV: minV - pad, maxV: maxV + pad };
  }
  // 地図グリッド(1セル=1pxの画像、行0が最北)を画面に描くcanvas変換行列[a,b,c,d,e,f]。
  function mapImageMatrix(view, w, h, m) {
    const r = view.rot || 0, c = Math.cos(r), s = Math.sin(r);
    const k = view.scale * m.resolution;
    const tl = mapViewWorldToPixel(view, w, h, m.origin_x, m.origin_y + m.height * m.resolution);
    return [k * c, -k * s, k * s, k * c, tl[0], tl[1]];
  }
  // ---- 地図ピン ----
  // pins=[{id,x,y}]のうち、画面上の点(px,py)に最も近い(radiusPx以内の)ピンのid。無ければnull。
  function mapPinHit(view, w, h, pins, px, py, radiusPx) {
    let best = null, bestD = radiusPx;
    for (const p of pins) {
      const q = mapViewWorldToPixel(view, w, h, p.x, p.y);
      const d = Math.hypot(q[0] - px, q[1] - py);
      if (d <= bestD) { bestD = d; best = p.id; }
    }
    return best;
  }
  // 表示角度の正規化[deg] -> (-180, 180]
  function mapNormDeg(a) {
    a = a % 360;
    if (a <= -180) a += 360; else if (a > 180) a -= 360;
    return a;
  }
  // ---- 現在位置の指定(AMCLの初期位置) ----
  // 世界座標の2点(a=ロボットの位置, b=向いている方向の点)から {x, y, yaw_deg}。
  // yawは世界(map)座標系での向き(x軸=0、反時計回りが正、(-180,180])。画面のy下向き・表示回転は
  // 点を世界座標へ戻す時点(mapViewPixelToWorld)で吸収済みなので、ここは単純なatan2。
  function mapPoseFromWorldPoints(a, b) {
    const yaw = Math.atan2(b[1] - a[1], b[0] - a[0]);
    return { x: a[0], y: a[1], yaw_deg: mapNormDeg(yaw * 180 / Math.PI) };
  }
  // 画面上の2点(CSS px)から。p0=ロボットの位置、p1=向いている方向の点(ドラッグの終点)。
  function mapPoseFromScreenDrag(view, w, h, p0, p1) {
    const a = mapViewPixelToWorld(view, w, h, p0[0], p0[1]);
    const b = mapViewPixelToWorld(view, w, h, p1[0], p1[1]);
    return mapPoseFromWorldPoints(a, b);
  }
  // 世界のyaw[rad]が画面上で指す向き(canvasの角度。y下向きなので符号が逆、表示回転ぶん回る)。
  function mapPoseScreenAngle(view, yawRad) { return -(yawRad + (view.rot || 0)); }
  // ---- 走行可能範囲(回転矩形 {center_x, center_y, width, height, yaw[rad]}) ----
  // 表示ビュー(回転rot)で軸平行に見える矩形を、対角の2点(世界座標)から作る。
  // 矩形のローカルx軸(yaw)は、表示で水平に見える向き=世界の-rot方向。
  function boundaryRectFromViewCorners(p1, p2, rot) {
    const a = mapRotUV(p1[0], p1[1], rot), b = mapRotUV(p2[0], p2[1], rot);
    const uc = (a[0] + b[0]) / 2, vc = (a[1] + b[1]) / 2;
    const c = mapUVToWorld(uc, vc, rot);
    return {
      center_x: c[0], center_y: c[1],
      width: Math.abs(a[0] - b[0]), height: Math.abs(a[1] - b[1]),
      yaw: -rot,
    };
  }
  // 状態(新形式 or 旧min/max形式)から {center_x,center_y,width,height,yaw} を得る。
  function boundaryRectOf(b) {
    if (b && b.center_x !== undefined && b.width !== undefined) {
      return { center_x: b.center_x, center_y: b.center_y, width: b.width, height: b.height, yaw: b.yaw || 0 };
    }
    return {
      center_x: (b.min_x + b.max_x) / 2, center_y: (b.min_y + b.max_y) / 2,
      width: b.max_x - b.min_x, height: b.max_y - b.min_y, yaw: 0,
    };
  }
  // 世界座標の4隅(ローカル(-,-),(+,-),(+,+),(-,+)の順。サーバーのrect_cornersと同じ)。
  function boundaryCornersOf(r) {
    const c = Math.cos(r.yaw), s = Math.sin(r.yaw), hw = r.width / 2, hh = r.height / 2;
    return [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]].map(
      ([lx, ly]) => [r.center_x + lx * c - ly * s, r.center_y + lx * s + ly * c]);
  }
  // 角ハンドルidxをpWorldへドラッグ: 対角は固定、矩形の向き(yaw)は保つ。
  function boundaryResizeCorner(r, idx, pWorld) {
    const f = boundaryCornersOf(r)[(idx + 2) % 4];
    const c = Math.cos(r.yaw), s = Math.sin(r.yaw);
    const dx = pWorld[0] - f[0], dy = pWorld[1] - f[1];
    const du = dx * c + dy * s, dv = -dx * s + dy * c;
    return {
      center_x: f[0] + (du / 2) * c - (dv / 2) * s,
      center_y: f[1] + (du / 2) * s + (dv / 2) * c,
      width: Math.abs(du), height: Math.abs(dv), yaw: r.yaw,
    };
  }
"""

MAP_VIEW_JS = MAP_VIEW_MATH_JS + r"""
  const canvas = document.getElementById("mapCanvas");
  const ctx = canvas.getContext("2d");
  let mapMeta = null;   // {width, height, resolution, origin_x, origin_y}
  let robotPose = null; // {x, y, yaw}
  let routeTrail = [];  // 実際に通った軌跡 [[x,y], ...]
  let plannedPath = []; // Nav2の計画経路(/plan) [[x,y], ...]
  let mapOffscreen = null;     // 地図グリッドを1セル=1pxで描いた画像
  let mapKnownBounds = null;   // 既知セルの外接矩形(世界座標m)
  let mapBytes = null;         // 地図データ(各セル=値+1、0=未知)
  let mapExtCache = null;      // 回転ごとの既知セル外接矩形のキャッシュ {key, ext}
  let mapView = { cx: 0, cy: 0, scale: 40, rot: 0 };
  let mapRotDeg = 0;           // 表示回転[deg](サーバーに保存、-180..180)
  let mapViewUser = false;     // true: ユーザーが拡大/移動した(自動で全体表示に戻さない)
  let mapCssW = 0, mapCssH = 0, mapDpr = 1;
  let mapClickSuppressUntil = 0; // ドラッグ/ピンチ直後のclickを目的地指定として扱わない
  let mapRotLastEdit = 0;      // 最後にこの端末で角度を変えた時刻(サーバー値での巻き戻し防止)
  let mapWallPick = null;      // 壁に合わせる2点指定中: [[x,y], ...] / null
  let poseMode = false;        // 現在位置の指定モード(2026-10-07)
  let poseStart = null;        // 1点目をクリックで確定した世界座標 [x,y](2点目待ち)
  let poseCur = null;          // 向きのプレビュー用の現在点(世界座標 [x,y])
  let poseDrag = null;         // {id, startPx, startWorld, moved}
  let posePending = null;      // 確認待ちの {x, y, yaw_deg}
  let pendingGoal = null;      // 確認待ちの目的地 [x,y](位置が不確かな時)
  let locStatus = null;        // /api/localization の最新
  // 【2026-10-07】地図ピン・目的地の進捗(詳細は下の「地図ピン」ブロック)
  let pinsData = [], pinsMax = 200, pinsMode = "idle", pinSelectedId = null, pinEdit = null, pinNewColor = null;
  let pinMode = false, pinStart = null, pinCur = null, pinDrag = null, pinPending = null;
  let goalHeadMode = false, gHeadDrag = null, gHeadCur = null;
  let goalSt = null, goalDismissedAt = null, goalMsg = "", pendingGoalYaw = null, pendingPinGo = null;

  // キャンバス(CSS px)座標 <-> 世界座標。描画と目的地クリックで同じビューを使う。
  function worldToPixel(x, y) {
    if (!mapMeta) return null;
    return mapViewWorldToPixel(mapView, mapCssW, mapCssH, x, y);
  }
  function pixelToWorld(px, py) {
    return mapViewPixelToWorld(mapView, mapCssW, mapCssH, px, py);
  }
  function mapEventToPixel(ev) {
    const rect = canvas.getBoundingClientRect();
    const sx = rect.width > 0 ? mapCssW / rect.width : 1;
    const sy = rect.height > 0 ? mapCssH / rect.height : 1;
    return [(ev.clientX - rect.left) * sx, (ev.clientY - rect.top) * sy];
  }

  function sizeMapCanvas() {
    const rect = canvas.getBoundingClientRect();
    mapCssW = Math.max(1, Math.round(rect.width));
    mapCssH = Math.max(1, Math.round(rect.height));
    mapDpr = window.devicePixelRatio || 1;
    const bw = Math.round(mapCssW * mapDpr), bh = Math.round(mapCssH * mapDpr);
    if (canvas.width !== bw || canvas.height !== bh) { canvas.width = bw; canvas.height = bh; }
  }

  // 全体表示: 既知セル+ロボット+軌跡+計画経路の外接矩形(表示の向きで見た)に合わせる。
  function fitMapView() {
    if (!mapMeta) return;
    const rot = mapRotDeg * Math.PI / 180;
    let ext = null;
    if (mapBytes) {
      const key = rot + "/" + mapMeta.width + "x" + mapMeta.height;
      if (!mapExtCache || mapExtCache.key !== key || mapExtCache.bytes !== mapBytes) {
        mapExtCache = { key: key, bytes: mapBytes, ext: mapKnownExtentsRot(mapMeta, mapBytes, rot) };
      }
      ext = mapExtCache.ext;
    }
    if (!ext) {
      ext = { minU: Infinity, maxU: -Infinity, minV: Infinity, maxV: -Infinity };
      const r = mapMeta.resolution;
      for (const [x, y] of [[mapMeta.origin_x, mapMeta.origin_y],
                            [mapMeta.origin_x + mapMeta.width * r, mapMeta.origin_y],
                            [mapMeta.origin_x + mapMeta.width * r, mapMeta.origin_y + mapMeta.height * r],
                            [mapMeta.origin_x, mapMeta.origin_y + mapMeta.height * r]]) {
        const uv = mapRotUV(x, y, rot);
        ext.minU = Math.min(ext.minU, uv[0]); ext.maxU = Math.max(ext.maxU, uv[0]);
        ext.minV = Math.min(ext.minV, uv[1]); ext.maxV = Math.max(ext.maxV, uv[1]);
      }
    } else {
      ext = { minU: ext.minU, maxU: ext.maxU, minV: ext.minV, maxV: ext.maxV };
    }
    const grow = (x, y) => {
      const uv = mapRotUV(x, y, rot);
      if (uv[0] < ext.minU) ext.minU = uv[0];
      if (uv[0] > ext.maxU) ext.maxU = uv[0];
      if (uv[1] < ext.minV) ext.minV = uv[1];
      if (uv[1] > ext.maxV) ext.maxV = uv[1];
    };
    if (robotPose) grow(robotPose.x, robotPose.y);
    for (const p of routeTrail) grow(p[0], p[1]);
    for (const p of plannedPath) grow(p[0], p[1]);
    mapView = mapFitViewRot(ext, mapCssW, mapCssH, 1.0, rot);
  }

  function renderMap() {
    sizeMapCanvas();
    ctx.setTransform(mapDpr, 0, 0, mapDpr, 0, 0);
    ctx.clearRect(0, 0, mapCssW, mapCssH);
    if (!mapMeta || !mapOffscreen) return;
    if (!mapViewUser) fitMapView();
    mapView.rot = mapRotDeg * Math.PI / 180;
    const mt = mapImageMatrix(mapView, mapCssW, mapCssH, mapMeta);
    ctx.imageSmoothingEnabled = false;
    ctx.setTransform(mt[0] * mapDpr, mt[1] * mapDpr, mt[2] * mapDpr, mt[3] * mapDpr, mt[4] * mapDpr, mt[5] * mapDpr);
    ctx.drawImage(mapOffscreen, 0, 0);
    ctx.setTransform(mapDpr, 0, 0, mapDpr, 0, 0);
    drawRouteTrail();
    drawBoundary();
    drawWallPick();
    drawPins();
    drawGoalMarker();
    drawRobotMarker();
    drawPoseTool();
    drawPinTool();
    drawMapCompass();
  }

  async function fetchMap() {
    try {
      const res = await fetch("/api/map");
      if (!res.ok) {
        // 地図リセット直後等、サーバー側の地図が無くなった状態。表示側も
        // 古い地図の描画を消す(更新するまで残り続けてしまうバグの修正、
        // 2026-07-22)。
        if (mapMeta !== null) {
          mapMeta = null;
          mapOffscreen = null;
          mapKnownBounds = null;
          mapBytes = null;
          renderMap();
        }
        return;
      }
      const m = await res.json();
      const bytes = Uint8Array.from(atob(m.data_base64), c => c.charCodeAt(0));
      const img = new ImageData(m.width, m.height);
      for (let i = 0; i < m.width * m.height; i++) {
        const v = bytes[i] - 1; // -1(未知)〜100(占有)に戻す
        let shade;
        if (v < 0) shade = 60;             // 未知: 暗いグレー
        else shade = 255 - Math.round(v * 2.3); // 占有率が高いほど暗く
        const row = m.height - 1 - Math.floor(i / m.width);
        const col = i % m.width;
        const idx = (row * m.width + col) * 4;
        img.data[idx] = img.data[idx+1] = img.data[idx+2] = Math.max(0, Math.min(255, shade));
        img.data[idx+3] = 255;
      }
      const off = document.createElement("canvas");
      off.width = m.width; off.height = m.height;
      off.getContext("2d").putImageData(img, 0, 0);
      mapOffscreen = off;
      mapBytes = bytes;
      mapKnownBounds = mapKnownBoundsOf(m, bytes);
      mapMeta = m;
      renderMap();
    } catch (e) { /* 無視、次のポーリングで再試行 */ }
  }

  // 【2026-09-15追加】通った軌跡(緑の実線)とNav2の計画経路(青の破線)を地図上に
  // 重ねて描画する。renderMap()が毎回キャンバスを描き直すので重なって残らない。
  function drawRouteTrail() {
    if (!mapMeta) return;
    if (routeTrail.length > 1) {
      ctx.strokeStyle = "rgba(23, 107, 86, 0.6)";
      ctx.lineWidth = 2;
      ctx.setLineDash([]);
      ctx.beginPath();
      routeTrail.forEach(([x, y], i) => {
        const p = worldToPixel(x, y);
        if (!p) return;
        if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
      });
      ctx.stroke();
    }
    if (plannedPath.length > 1) {
      ctx.strokeStyle = "#2b6fe0";
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      plannedPath.forEach(([x, y], i) => {
        const p = worldToPixel(x, y);
        if (!p) return;
        if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]);
      });
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }
  function drawRobotMarker() {
    if (!robotPose || !mapMeta) return;
    const p = worldToPixel(robotPose.x, robotPose.y);
    if (!p) return;
    // 世界のyawは表示回転ぶん回って見える(画面はy下向きなので符号が逆)
    const ang = -(robotPose.yaw + (mapView.rot || 0));
    ctx.fillStyle = "#ff5a3c";
    ctx.beginPath(); ctx.arc(p[0], p[1], 6, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = "#ff5a3c"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(p[0], p[1]);
    ctx.lineTo(p[0] + 14 * Math.cos(ang), p[1] + 14 * Math.sin(ang));
    ctx.stroke();
  }
  // 状態ポーリング(500ms)からも呼ばれる。全体を描き直すのでマーカーが重ならない。
  function drawRobot() { renderMap(); }

  // 方位マーカー: 右上に世界座標の+X(赤)・+Y(緑)の向きを描く(回転すると一緒に回る)。
  function drawMapCompass() {
    if (!mapMeta) return;
    const cx = mapCssW - 34, cy = 34, R = 22, rot = mapView.rot || 0;
    ctx.save();
    ctx.setLineDash([]);
    ctx.fillStyle = "rgba(255,255,255,0.78)";
    ctx.beginPath(); ctx.arc(cx, cy, R + 8, 0, Math.PI * 2); ctx.fill();
    ctx.strokeStyle = "rgba(0,0,0,0.25)"; ctx.lineWidth = 1; ctx.stroke();
    const axes = [[0, "#d33a2c", "X"], [Math.PI / 2, "#1f9d55", "Y"]];
    for (const [a0, col, label] of axes) {
      const a = -(a0 + rot);
      const ex = cx + R * Math.cos(a), ey = cy + R * Math.sin(a);
      ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(ex, ey);
      ctx.lineTo(ex - 6 * Math.cos(a - 0.5), ey - 6 * Math.sin(a - 0.5));
      ctx.lineTo(ex - 6 * Math.cos(a + 0.5), ey - 6 * Math.sin(a + 0.5));
      ctx.closePath(); ctx.fill();
      ctx.font = "bold 11px sans-serif";
      ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillText(label, cx + (R + 8) * Math.cos(a) * 1.0, cy + (R + 8) * Math.sin(a) * 1.0);
    }
    ctx.fillStyle = "#222"; ctx.font = "10px sans-serif"; ctx.textAlign = "center"; ctx.textBaseline = "top";
    ctx.fillText(mapRotDeg.toFixed(1) + "°", cx, cy + R + 12);
    ctx.restore();
  }

  // 走行可能範囲(回転矩形)。表示の向きと矩形の向きが合っていれば画面上は軸平行に見える。
  // 四隅にハンドル(□)を描き、ドラッグで大きさを変えられる(対角は固定・向きは保つ)。
  const MAP_HANDLE_R = 6;
  let mapBoundaryDrag = null; // {idx, rect}
  function drawBoundaryShape(b) {
    if (!b || !b.enabled || !mapMeta) return;
    const pts = boundaryCornersOf(boundaryRectOf(b)).map(([x, y]) => worldToPixel(x, y));
    if (pts.some(p => !p)) return;
    ctx.save();
    ctx.strokeStyle = "#ffb000";
    ctx.lineWidth = 2;
    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    pts.forEach((p, i) => { if (i === 0) ctx.moveTo(p[0], p[1]); else ctx.lineTo(p[0], p[1]); });
    ctx.closePath();
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = "#fff8e0";
    for (const p of pts) {
      ctx.beginPath();
      ctx.rect(p[0] - MAP_HANDLE_R, p[1] - MAP_HANDLE_R, MAP_HANDLE_R * 2, MAP_HANDLE_R * 2);
      ctx.fill(); ctx.stroke();
    }
    ctx.restore();
  }
  function mapHandleAt(px, py) {
    if (typeof currentBoundary === "undefined" || !currentBoundary || !currentBoundary.enabled || !mapMeta) return -1;
    if (typeof boundaryPickMode !== "undefined" && boundaryPickMode) return -1;
    if (mapWallPick) return -1;  // 壁の2点指定中はハンドルを無効にする
    if (poseMode) return -1;     // 現在位置の指定中も無効(ドラッグは向きの指定に使う)
    const pts = boundaryCornersOf(boundaryRectOf(currentBoundary)).map(([x, y]) => worldToPixel(x, y));
    let best = -1, bestD = MAP_HANDLE_R + 8;
    pts.forEach((p, i) => {
      const d = Math.hypot(p[0] - px, p[1] - py);
      if (d <= bestD) { bestD = d; best = i; }
    });
    return best;
  }
  canvas.addEventListener("pointerdown", (ev) => {
    if (ev.pointerType === "mouse" && ev.button !== 0) return;
    const p = mapEventToPixel(ev);
    const idx = mapHandleAt(p[0], p[1]);
    if (idx < 0) return;
    ev.stopImmediatePropagation();
    ev.preventDefault();
    try { canvas.setPointerCapture(ev.pointerId); } catch (e) { /* 無視 */ }
    mapBoundaryDrag = { idx: idx, id: ev.pointerId, rect: boundaryRectOf(currentBoundary) };
    mapClickSuppressUntil = Date.now() + 600;
  });
  canvas.addEventListener("pointermove", (ev) => {
    if (!mapBoundaryDrag || ev.pointerId !== mapBoundaryDrag.id) return;
    const p = mapEventToPixel(ev);
    const w = pixelToWorld(p[0], p[1]);
    const r = boundaryResizeCorner(mapBoundaryDrag.rect, mapBoundaryDrag.idx, w);
    mapBoundaryDrag.preview = r;
    currentBoundary = Object.assign({}, currentBoundary, r, { enabled: true });
    mapClickSuppressUntil = Date.now() + 600;
    renderMap();
  });
  // 範囲を送った後、サーバーに保存された範囲を読み直して送った範囲と一致するか確かめる(一致しなければ画面に出す)
  async function boundaryVerify(rect, hint) {
    if (typeof fetchBoundary === "function") await fetchBoundary();
    const b = currentBoundary;
    const ok = !!(b && b.enabled && ["center_x", "center_y", "width", "height"].every((k) => Math.abs(Number(b[k]) - Number(rect[k])) < 0.02));
    if (!ok && hint) hint.textContent = "走行可能範囲が反映されていません(サーバーの範囲が送った範囲と違います)。もう一度設定してください";
    return ok;
  }
  async function mapBoundaryDragEnd(ev) {
    if (!mapBoundaryDrag || ev.pointerId !== mapBoundaryDrag.id) return;
    const drag = mapBoundaryDrag;
    mapBoundaryDrag = null;
    mapClickSuppressUntil = Date.now() + 600;
    const r = drag.preview;
    if (!r) return;
    const hint = document.getElementById("boundaryHint");
    if (r.width < 0.2 || r.height < 0.2) {
      if (hint) hint.textContent = "範囲が小さすぎるため変更しませんでした";
    } else {
      let posted = false;
      NKField.bump("boundary");
      try {
        const res = await fetch("/api/boundary/set", {
          method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(r),
        });
        let d = null;
        try { d = await res.json(); } catch (e2) { /* 無視 */ }
        posted = res.ok;
        if (hint) hint.textContent = (d && d.message) || (res.ok ? "走行可能範囲を変更しました" : "範囲の変更に失敗しました");
      } catch (e) {
        if (hint) hint.textContent = "範囲の変更に失敗しました(通信エラー)";
      }
      NKField.bump("boundary");
      if (posted) { await boundaryVerify(r, hint); return; }
    }
    if (typeof fetchBoundary === "function") fetchBoundary();
  }
  canvas.addEventListener("pointerup", mapBoundaryDragEnd);
  canvas.addEventListener("pointercancel", mapBoundaryDragEnd);

  // ===== 現在位置の指定(AMCLの初期位置)・位置の確かさ・不確かな時の目的地確認(2026-10-07) =====
  // ロボットは動かさない(送るのは/initialposeだけ)。要素が無いページ(Stage2)では何もしない。
  const poseEl = (id) => document.getElementById(id);
  const POSE_DRAG_MIN_PX = 14;
  function poseArrow(p, ang, color, dashed) {
    const L = 36, ex = p[0] + L * Math.cos(ang), ey = p[1] + L * Math.sin(ang);
    ctx.save();
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 3;
    ctx.setLineDash(dashed ? [5, 4] : []);
    ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(ex, ey); ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath();
    ctx.moveTo(ex + 6 * Math.cos(ang), ey + 6 * Math.sin(ang));
    ctx.lineTo(ex - 7 * Math.cos(ang - 0.6), ey - 7 * Math.sin(ang - 0.6));
    ctx.lineTo(ex - 7 * Math.cos(ang + 0.6), ey - 7 * Math.sin(ang + 0.6));
    ctx.closePath(); ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = "#fff";
    ctx.beginPath(); ctx.arc(p[0], p[1], 7, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill(); ctx.stroke();
    ctx.restore();
  }
  function drawPoseTool() {
    if (!mapMeta) return;
    const PURPLE = "#a21caf";
    if (poseStart) {
      const p = worldToPixel(poseStart[0], poseStart[1]);
      if (poseCur) {
        const q = worldToPixel(poseCur[0], poseCur[1]);
        poseArrow(p, Math.atan2(q[1] - p[1], q[0] - p[0]), PURPLE, true);
      } else {
        ctx.save(); ctx.fillStyle = PURPLE;
        ctx.beginPath(); ctx.arc(p[0], p[1], 7, 0, Math.PI * 2); ctx.fill(); ctx.restore();
      }
    }
    if (posePending) {
      const p = worldToPixel(posePending.x, posePending.y);
      poseArrow(p, mapPoseScreenAngle(mapView, posePending.yaw_deg * Math.PI / 180), PURPLE, false);
    }
    if (pendingGoal) {
      const p = worldToPixel(pendingGoal[0], pendingGoal[1]);
      ctx.save(); ctx.strokeStyle = "#e08a00"; ctx.lineWidth = 3; ctx.setLineDash([4, 3]);
      ctx.beginPath(); ctx.arc(p[0], p[1], 11, 0, Math.PI * 2); ctx.stroke(); ctx.restore();
    }
    // localizationモードでmap->base_linkのtfがまだ無い間は、古いマーカーを出さず明示する
    if (locStatus && locStatus.localization_active && !locStatus.tf_available) {
      ctx.save();
      ctx.font = "bold 13px sans-serif"; ctx.textAlign = "left"; ctx.textBaseline = "bottom";
      const t = "TFなし(位置合わせ前)";
      const tw = ctx.measureText(t).width;
      ctx.fillStyle = "rgba(255,243,205,0.92)"; ctx.fillRect(8, mapCssH - 30, tw + 16, 24);
      ctx.fillStyle = "#7a5b00"; ctx.fillText(t, 16, mapCssH - 11);
      ctx.restore();
    }
  }
  function poseSetHint(t) { const e = poseEl("poseHint"); if (e) e.textContent = t; }
  function poseShowBar(show) {
    const bar = poseEl("poseConfirmBar");
    if (bar) bar.hidden = !show;
    if (show && posePending) {
      poseEl("poseConfirmText").textContent = "位置 (" + posePending.x.toFixed(2) + ", " + posePending.y.toFixed(2) +
        ") m 向き " + posePending.yaw_deg.toFixed(0) + "° で位置合わせしますか";
    }
  }
  function setPoseMode(on) {
    if (on && pinMode) setPinMode(false);
    if (on && goalHeadMode) setGoalHeadMode(false);
    poseMode = !!on;
    poseStart = null; poseCur = null; poseDrag = null; posePending = null;
    poseShowBar(false);
    const btn = poseEl("poseToolBtn");
    if (btn) { btn.textContent = poseMode ? "現在位置の指定を終了" : "現在位置を指定"; btn.classList.toggle("primary", poseMode); }
    canvas.style.cursor = poseMode ? "copy" : "";
    if (poseMode) {
      mapWallPick = null;
      if (typeof boundaryPickMode !== "undefined") boundaryPickMode = false;
      clearPendingGoal();
      poseSetHint("地図上で、ロボットが実際にいる場所をクリックし、ロボットが向いている方向へドラッグしてください(ドラッグせず2点目をクリックでも可)。");
    }
    renderMap();
  }
  function poseFinalize(a, b) {
    if (Math.hypot(b[0] - a[0], b[1] - a[1]) < 0.05) {
      poseStart = null; poseCur = null;
      poseSetHint("向きの点が近すぎます。ロボットの位置から少し離れた、向いている方向の点を指定してください。");
      renderMap();
      return;
    }
    posePending = mapPoseFromWorldPoints(a, b);
    poseStart = null; poseCur = null;
    poseShowBar(true);
    poseSetHint("紫の矢印がロボットの位置と向きです。合っていれば[送信]、違えば[やり直し]。");
    renderMap();
  }
  canvas.addEventListener("pointerdown", (ev) => {
    if (!poseMode || !mapMeta) return;
    if (ev.pointerType === "mouse" && ev.button !== 0) return;
    ev.stopImmediatePropagation();
    ev.preventDefault();
    try { canvas.setPointerCapture(ev.pointerId); } catch (e) { /* 無視 */ }
    const p = mapEventToPixel(ev);
    poseDrag = { id: ev.pointerId, startPx: p, startWorld: pixelToWorld(p[0], p[1]), moved: false };
    mapClickSuppressUntil = Date.now() + 600;
  });
  canvas.addEventListener("pointermove", (ev) => {
    if (!poseMode || !mapMeta) return;
    const p = mapEventToPixel(ev);
    if (poseDrag && ev.pointerId === poseDrag.id) {
      if (!poseDrag.moved && Math.hypot(p[0] - poseDrag.startPx[0], p[1] - poseDrag.startPx[1]) >= POSE_DRAG_MIN_PX) {
        poseDrag.moved = true;
        poseStart = poseDrag.startWorld;
        posePending = null; poseShowBar(false);
      }
      if (poseDrag.moved) { poseCur = pixelToWorld(p[0], p[1]); renderMap(); }
    } else if (!poseDrag && poseStart && !posePending && ev.pointerType === "mouse") {
      poseCur = pixelToWorld(p[0], p[1]);  // 2点目クリック方式のプレビュー
      renderMap();
    }
  });
  function poseDragEnd(ev) {
    if (!poseDrag || ev.pointerId !== poseDrag.id) return;
    const d = poseDrag;
    poseDrag = null;
    mapClickSuppressUntil = Date.now() + 600;
    if (ev.type === "pointercancel") { poseStart = null; poseCur = null; renderMap(); return; }
    const p = mapEventToPixel(ev);
    if (d.moved) {
      poseFinalize(d.startWorld, pixelToWorld(p[0], p[1]));
    } else if (!poseStart || posePending) {
      posePending = null; poseShowBar(false);
      poseStart = d.startWorld; poseCur = null;
      poseSetHint("ロボットが向いている方向の点をもう一度クリックしてください(またはロボットの位置からドラッグ)。");
      renderMap();
    } else {
      poseFinalize(poseStart, d.startWorld);
    }
  }
  canvas.addEventListener("pointerup", poseDragEnd);
  canvas.addEventListener("pointercancel", poseDragEnd);
  // 指定モード中のclickは目的地・範囲指定・壁指定へ渡さない(captureで先に止める)
  canvas.addEventListener("click", (ev) => { if (poseMode) ev.stopImmediatePropagation(); }, true);

  async function poseSend() {
    if (!posePending) return;
    const sendBtn = poseEl("poseSendBtn");
    if (sendBtn) sendBtn.disabled = true;
    let d = null, ok = false;
    try {
      const res = await fetch("/api/initial_pose", {
        method: "POST", headers: {"Content-Type": "application/json"},
        body: JSON.stringify({ x: posePending.x, y: posePending.y, yaw_deg: posePending.yaw_deg }),
      });
      try { d = await res.json(); } catch (e2) { /* 無視 */ }
      ok = res.ok && !(d && d.ok === false);
      if (!ok && !d) d = { message: "送信に失敗しました(" + res.status + ")" };
    } catch (e) {
      d = { message: "通信エラー: サーバーに接続できません" };
    }
    if (sendBtn) sendBtn.disabled = false;
    poseSetHint((d && d.message) || (ok ? "送信しました" : "送信に失敗しました"));
    if (ok) {
      posePending = null; poseStart = null; poseCur = null;
      poseShowBar(false);
      setPoseMode(false);
      poseSetHint((d && d.message) || "送信しました");  // setPoseModeがヒントを消さないよう再設定
      pollLocalization();
    }
  }
  function poseRetry() {
    posePending = null; poseStart = null; poseCur = null;
    poseShowBar(false);
    poseSetHint("やり直し: ロボットのいる場所をクリックし、向いている方向へドラッグしてください。");
    renderMap();
  }
  async function globalLocalize() {
    const btn = poseEl("globalLocBtn");
    if (btn) btn.disabled = true;
    poseSetHint("AMCLへ全体位置推定を依頼しています...");
    let msg = "";
    try {
      const res = await fetch("/api/global_localization", {
        method: "POST", headers: {"Content-Type": "application/json"}, body: "{}",
      });
      let d = null;
      try { d = await res.json(); } catch (e2) { /* 無視 */ }
      msg = (d && d.message) || (res.ok ? "依頼しました" : "失敗しました(" + res.status + ")");
    } catch (e) {
      msg = "通信エラー: サーバーに接続できません";
    }
    if (btn) btn.disabled = false;
    poseSetHint(msg);
    pollLocalization();
  }

  // --- 位置の確かさ(/api/localization)と、不確かな時の目的地クリックの確認 ---
  // 【2026-11-02追加】位置が「良好」でない時の「このまま走行しますか?」の確認は、ON/OFFできる。既定はOFF
  // (確認を出さずに送る)。この端末のブラウザだけの設定(localStorage)で、再読み込み後も残る。
  function locConfirmEnabled() {
    try { return window.localStorage.getItem("nk_loc_confirm") === "1"; } catch (e) { return false; }
  }
  function locNeedsConfirm() {
    return locConfirmEnabled() && !!(locStatus && locStatus.localization_active && locStatus.quality !== "good");
  }
  function renderLocConfirmBtn() {
    const b = poseEl("locConfirmBtn");
    if (!b) return;
    const on = locConfirmEnabled();
    b.textContent = "位置が不確かなときの走行前の確認: " + (on ? "ON" : "OFF");
    b.setAttribute("aria-pressed", on ? "true" : "false");
  }
  function toggleLocConfirm() {
    try { window.localStorage.setItem("nk_loc_confirm", locConfirmEnabled() ? "0" : "1"); } catch (e) { /* 保存できない環境では切り替わらない */ }
    if (!locConfirmEnabled()) clearPendingGoal();
    renderLocConfirmBtn();
    renderMap();
  }
  async function pollLocalization() {
    if (!poseEl("locChip")) return;
    try {
      const res = await fetch("/api/localization");
      if (!res.ok) return;
      locStatus = await res.json();
    } catch (e) { return; }
    const s = locStatus, chip = poseEl("locChip");
    const cls = { good: "chip-ok", check: "chip-warn", unknown: "chip-bad", waiting: "chip-mute", inactive: "chip-mute" }[s.quality] || "chip-mute";
    chip.className = "chip " + cls;
    chip.textContent = s.localization_active ? ("位置の確かさ: " + s.label) : "位置の確かさ: -";
    const detail = poseEl("locDetail");
    if (detail) {
      let t = "";
      if (s.localization_active && s.pos_std_m !== null && s.pos_std_m !== undefined) {
        t = "ばらつき " + s.pos_std_m.toFixed(2) + " m / 向き " + (s.yaw_std_rad * 180 / Math.PI).toFixed(0) + "°";
        if (s.age_sec !== null && s.age_sec !== undefined) t += "(" + Math.round(s.age_sec) + "秒前)";
      } else if (s.localization_active) {
        t = s.quality === "waiting" ? "AMCLの推定待ち" : "";
      } else {
        t = "地図を読み込むと表示されます";
      }
      if (s.localization_active && !s.tf_available) t += (t ? " / " : "") + "TFなし(位置合わせ前)";
      detail.textContent = t;
    }
    const warn = poseEl("locWarn");
    if (warn) warn.hidden = !locNeedsConfirm();
    renderMap();
  }
  function clearPendingGoal() {
    pendingGoal = null; pendingGoalYaw = null; pendingPinGo = null;
    const bar = poseEl("goalConfirmBar");
    if (bar) bar.hidden = true;
  }
  // 目的地クリック。位置が「良好」でない(localization中)ならすぐ送らず確認バーを出す(既定は送らない)。
  // yawDegを渡すと到着時の向き(度、map座標系)も指定する(向き指定でクリック)。
  function requestGoal(x, y, yawDeg) {
    if (!poseEl("goalProgressBar")) {  // 進捗表示の無いページ(Stage2)は従来どおり
      clearPendingGoal();
      return postJson("/api/goal", { x: x, y: y });
    }
    if (!locNeedsConfirm() || !poseEl("goalConfirmBar")) {
      clearPendingGoal();
      return sendGoalNow(x, y, yawDeg);
    }
    pendingGoal = [x, y]; pendingPinGo = null;
    pendingGoalYaw = (yawDeg === undefined) ? null : yawDeg;
    const bar = poseEl("goalConfirmBar");
    poseEl("goalConfirmText").textContent = "位置が不確かです(位置の確かさ: " + (locStatus.label || "-") +
      ")。このまま走行しますか？ 目的地 (" + x.toFixed(2) + ", " + y.toFixed(2) + ") m" +
      (pendingGoalYaw === null ? "" : " 向き " + pendingGoalYaw.toFixed(0) + "°");
    bar.hidden = false;
    renderMap();
    return Promise.resolve(false);
  }
  async function goalConfirmGo() {
    const g = pendingGoal, gy = pendingGoalYaw, pid = pendingPinGo;
    clearPendingGoal();
    renderMap();
    if (pid) await pinGotoNow(pid);
    else if (g) await sendGoalNow(g[0], g[1], gy);
  }
  function goalConfirmCancel() { clearPendingGoal(); renderMap(); }
  if (poseEl("locConfirmBtn")) {
    poseEl("locConfirmBtn").addEventListener("click", toggleLocConfirm);
    renderLocConfirmBtn();
  }
  if (poseEl("poseToolBtn")) {
    poseEl("poseToolBtn").addEventListener("click", () => setPoseMode(!poseMode));
    poseEl("poseSendBtn").addEventListener("click", poseSend);
    poseEl("poseRetryBtn").addEventListener("click", poseRetry);
    poseEl("globalLocBtn").addEventListener("click", globalLocalize);
    poseEl("goalGoBtn").addEventListener("click", goalConfirmGo);
    poseEl("goalCancelBtn").addEventListener("click", goalConfirmCancel);
    pollLocalization();
    setInterval(pollLocalization, 1000);
  }

  // ===== 地図ピン(名前付きの場所)・目的地の進捗と到着誤差・向き付きの目的地(2026-10-07) =====
  // ピンはmapフレーム(世界座標)のx,y,yaw_degで持ち、描画は唯一の変換(mapViewWorldToPixel)を通す。
  // 要素(pinList / goalProgressBar)が無いページ(Stage2)では何もしない。
  const PIN_COLORS = ["#e11d48", "#2563eb", "#16a34a", "#d97706", "#7c3aed", "#0891b2", "#db2777", "#4d7c0f"];
  const PIN_HIT_PX = 18;
  const pinsOn = !!poseEl("pinList");
  const goalOn = !!poseEl("goalProgressBar");
  const GOAL_HEAD_MIN_PX = 14;

  function mkEl(tag, cls, text) {
    const e = document.createElement(tag);
    if (cls) e.className = cls;
    if (text !== undefined && text !== null) e.textContent = text;
    return e;
  }
  function fmtDist(m) {
    if (m === null || m === undefined || !isFinite(m)) return "-";
    return m < 1 ? Math.round(m * 100) + " cm" : m.toFixed(2) + " m";
  }
  function pinShowErr(msg) { if (typeof showError === "function") showError(msg || ""); }

  // --- 描画: ピン(ラベル+向きの短い線。表示回転に追従) ---
  function drawPins() {
    if (!mapMeta || !pinsData.length) return;
    ctx.save();
    ctx.setLineDash([]);
    ctx.font = "bold 12px sans-serif";
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    for (const p of pinsData) {
      const q = worldToPixel(p.x, p.y);
      if (!q) continue;
      const ang = mapPoseScreenAngle(mapView, p.yaw_deg * Math.PI / 180);
      const sel = p.id === pinSelectedId;
      const ex = q[0] + 22 * Math.cos(ang), ey = q[1] + 22 * Math.sin(ang);
      ctx.lineCap = "round";
      ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 6;
      ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(ex, ey); ctx.stroke();
      ctx.strokeStyle = p.color; ctx.lineWidth = 3;
      ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(ex, ey); ctx.stroke();
      ctx.beginPath(); ctx.arc(q[0], q[1], sel ? 10 : 8, 0, Math.PI * 2);
      ctx.fillStyle = p.color; ctx.fill();
      ctx.lineWidth = 2; ctx.strokeStyle = "#ffffff"; ctx.stroke();
      if (sel) {
        ctx.strokeStyle = "#111111"; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(q[0], q[1], 14, 0, Math.PI * 2); ctx.stroke();
      }
      ctx.lineJoin = "round"; ctx.lineWidth = 4; ctx.strokeStyle = "rgba(255,255,255,0.92)";
      ctx.strokeText(p.name, q[0], q[1] - 20);
      ctx.fillStyle = "#111111"; ctx.fillText(p.name, q[0], q[1] - 20);
    }
    ctx.restore();
  }

  // --- 描画: 目的地のマーカー(クリック位置・寄せた位置・向き・到着後の誤差) ---
  function goalVisible() {
    if (!goalSt || !goalSt.goal) return null;
    if (goalDismissedAt !== null && goalSt.sent_at === goalDismissedAt) return null;
    return goalSt;
  }
  function goalArrow(p, ang, color) {
    const L = 30, ex = p[0] + L * Math.cos(ang), ey = p[1] + L * Math.sin(ang);
    ctx.save();
    ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 6; ctx.lineCap = "round";
    ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(ex, ey); ctx.stroke();
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(ex, ey); ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(ex + 6 * Math.cos(ang), ey + 6 * Math.sin(ang));
    ctx.lineTo(ex - 6 * Math.cos(ang - 0.6), ey - 6 * Math.sin(ang - 0.6));
    ctx.lineTo(ex - 6 * Math.cos(ang + 0.6), ey - 6 * Math.sin(ang + 0.6));
    ctx.closePath(); ctx.fill();
    ctx.restore();
  }
  function drawGoalMarker() {
    if (!mapMeta) return;
    if (gHeadDrag && gHeadDrag.moved && gHeadCur) {
      const p0 = worldToPixel(gHeadDrag.startWorld[0], gHeadDrag.startWorld[1]);
      const q = worldToPixel(gHeadCur[0], gHeadCur[1]);
      goalArrow(p0, Math.atan2(q[1] - p0[1], q[0] - p0[0]), "#e08a00");
    }
    const g = goalVisible();
    if (!g) return;
    const gl = g.goal, p = worldToPixel(gl.x, gl.y);
    if (!p) return;
    const col = g.state === "succeeded" ? "#16a34a" : (g.state === "sent" ? "#e08a00" : "#dc2626");
    ctx.save();
    ctx.setLineDash([]);
    if (gl.snapped && gl.requested_x !== undefined) {
      const r = worldToPixel(gl.requested_x, gl.requested_y);
      ctx.strokeStyle = "#9a6b00"; ctx.lineWidth = 2; ctx.setLineDash([4, 3]);
      ctx.beginPath(); ctx.moveTo(r[0], r[1]); ctx.lineTo(p[0], p[1]); ctx.stroke();
      ctx.setLineDash([]);
      ctx.beginPath(); ctx.arc(r[0], r[1], 5, 0, Math.PI * 2); ctx.stroke();
      ctx.font = "11px sans-serif"; ctx.fillStyle = "#6b4a00"; ctx.textAlign = "left"; ctx.textBaseline = "top";
      ctx.fillText("クリック位置", r[0] + 7, r[1] + 3);
    }
    if (gl.yaw_specified) goalArrow(p, mapPoseScreenAngle(mapView, gl.yaw_deg * Math.PI / 180), col);
    ctx.strokeStyle = "#ffffff"; ctx.lineWidth = 5;
    ctx.beginPath(); ctx.arc(p[0], p[1], 11, 0, Math.PI * 2); ctx.stroke();
    ctx.strokeStyle = col; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(p[0], p[1], 11, 0, Math.PI * 2); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(p[0] - 16, p[1]); ctx.lineTo(p[0] + 16, p[1]);
    ctx.moveTo(p[0], p[1] - 16); ctx.lineTo(p[0], p[1] + 16); ctx.stroke();
    ctx.restore();
  }

  // --- API ---
  async function pinApi(path, body) {
    NKField.bump("pins");  // 送信前に始まった古いピン一覧の取得結果は捨てる(名前が一瞬戻るのを防ぐ)
    try {
      const res = await fetch(path, { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body || {}) });
      let d = null;
      try { d = await res.json(); } catch (e2) { /* 無視 */ }
      const ok = res.ok && !(d && d.ok === false);
      return { ok: ok, d: d || { message: "失敗しました(" + res.status + ")" }, status: res.status };
    } catch (e) {
      return { ok: false, d: { message: "通信エラー: サーバーに接続できません" }, status: 0 };
    } finally {
      NKField.bump("pins");
    }
  }
  function pinSetHint(t) { const e = poseEl("pinHint"); if (e) e.textContent = t || ""; }

  // --- ピン一覧(DOMで組む。名前はtextContentなので注入されない) ---
  function pinColorSwatches(current, onPick) {
    const wrap = mkEl("span", "pin-swatches");
    for (const c of PIN_COLORS) {
      const b = mkEl("button", "pin-swatch" + (c === current ? " on" : ""));
      b.type = "button"; b.style.background = c; b.title = c; b.setAttribute("aria-label", "色 " + c);
      b.addEventListener("click", () => onPick(c));
      wrap.appendChild(b);
    }
    return wrap;
  }
  function pinRow(p) {
    const row = mkEl("div", "pin-row" + (p.id === pinSelectedId ? " sel" : "") + (pinEdit && pinEdit.id === p.id ? " editing" : ""));
    row.dataset.id = p.id;
    const head = mkEl("div", "pin-head");
    const dot = mkEl("span", "pin-dot"); dot.style.background = p.color;
    head.appendChild(dot);
    head.appendChild(mkEl("b", "pin-name", p.name));
    const dist = mkEl("span", "pin-dist", fmtDist(p.distance_m)); dist.dataset.id = p.id;
    head.appendChild(dist);
    head.addEventListener("click", () => selectPin(p.id, false));
    row.appendChild(head);
    const body = mkEl("div", "pin-btns");
    if (pinEdit && pinEdit.id === p.id && pinEdit.kind === "name") {
      const inp = mkEl("input"); inp.type = "text"; inp.maxLength = 40; inp.value = p.name; inp.className = "pin-edit-input";
      NKField.guard(inp);
      inp.addEventListener("keydown", (ev) => { if (NKField.isEnter(ev)) pinSaveName(p.id, inp.value); });  // 日本語入力の変換確定のEnterでは保存しない
      body.appendChild(inp);
      const ok = mkEl("button", "primary", "保存"); ok.type = "button";
      ok.addEventListener("click", () => pinSaveName(p.id, inp.value));
      const no = mkEl("button", null, "やめる"); no.type = "button";
      no.addEventListener("click", () => { pinEdit = null; pinRenderList(); });
      body.appendChild(ok); body.appendChild(no);
      setTimeout(() => { try { inp.focus(); inp.select(); } catch (e) { /* 無視 */ } }, 0);
    } else if (pinEdit && pinEdit.id === p.id && pinEdit.kind === "color") {
      body.appendChild(pinColorSwatches(p.color, (c) => pinSaveColor(p.id, c)));
      const no = mkEl("button", null, "やめる"); no.type = "button";
      no.addEventListener("click", () => { pinEdit = null; pinRenderList(); });
      body.appendChild(no);
    } else if (pinEdit && pinEdit.id === p.id && (pinEdit.kind === "del" || pinEdit.kind === "upd")) {
      const t = pinEdit.kind === "del"
        ? "ピン「" + p.name + "」を削除しますか？"
        : "ピン「" + p.name + "」の位置を、ロボットの現在位置・向きに更新しますか？";
      body.appendChild(mkEl("span", "pin-confirm-text", t));
      const ok = mkEl("button", pinEdit.kind === "del" ? "danger" : "primary", pinEdit.kind === "del" ? "削除する" : "更新する");
      ok.type = "button";
      ok.addEventListener("click", () => (pinEdit.kind === "del" ? pinDelete(p.id) : pinUpdatePos(p.id)));
      const no = mkEl("button", null, "やめる"); no.type = "button";
      no.addEventListener("click", () => { pinEdit = null; pinRenderList(); });
      body.appendChild(ok); body.appendChild(no);
    } else {
      const go = mkEl("button", "primary", "行く"); go.type = "button"; go.dataset.cap = "nav.goto_pin";
      go.addEventListener("click", () => pinGoto(p.id));
      body.appendChild(go);
      const items = [["名前変更", "name"], ["色", "color"], ["現在位置に更新", "upd"], ["削除", "del"]];
      for (const [label, kind] of items) {
        const b = mkEl("button", null, label); b.type = "button";
        b.addEventListener("click", () => { pinEdit = { id: p.id, kind: kind }; pinRenderList(); });
        body.appendChild(b);
      }
    }
    row.appendChild(body);
    return row;
  }
  function pinRenderList() {
    const box = poseEl("pinList");
    if (!box) return;
    box.textContent = "";
    if (!pinsData.length) {
      box.appendChild(mkEl("p", "hint", "ピンはまだありません。ロボットのいる場所で「現在位置にピンを立てる」を押すか、「地図でピンを置く」で地図上に置けます。"));
    }
    for (const p of pinsData) box.appendChild(pinRow(p));
    const cnt = poseEl("pinCount");
    if (cnt) cnt.textContent = pinsData.length + " / " + pinsMax;
    pinRenderSelBar();
  }
  function pinRenderSelBar() {
    const bar = poseEl("pinSelBar");
    if (!bar) return;
    const p = pinsData.find((q) => q.id === pinSelectedId);
    bar.hidden = !p;
    if (!p) return;
    poseEl("pinSelText").textContent = "選択中のピン「" + p.name + "」(ロボットから " + fmtDist(p.distance_m) + ")";
  }
  function selectPin(id, scroll) {
    pinSelectedId = (pinSelectedId === id && !scroll) ? null : id;
    pinRenderList();
    if (pinSelectedId && scroll) {
      const row = poseEl("pinList").querySelector('[data-id="' + id + '"]');
      if (row && row.scrollIntoView) row.scrollIntoView({ block: "nearest" });
    }
    renderMap();
  }
  let pinsSig = "";
  async function pollPins() {
    if (!pinsOn) return;
    const tk = NKField.ticket("pins");
    try {
      const res = await fetch("/api/pins");
      if (!res.ok) return;
      const d = await res.json();
      if (!NKField.fresh(tk)) return;
      pinsData = d.pins || [];
      pinsMax = d.max || 200;
      pinsMode = d.mode || "idle";
      const note = poseEl("pinNote");
      if (note) { note.textContent = d.note || ""; note.hidden = !d.note; }
      if (pinSelectedId && !pinsData.some((p) => p.id === pinSelectedId)) pinSelectedId = null;
    } catch (e) { return; }
    // 編集中・ピンの中身が変わっていない時は一覧を作り直さない(入力が消えない/押そうとしたボタンが2秒ごとに作り直されて
    // クリックを取りこぼさないよう)。距離だけその場で更新する。
    const sig = JSON.stringify(pinsData.map((p) => [p.id, p.name, p.color, p.x, p.y, p.yaw_deg]));
    if (pinEdit || sig === pinsSig) {
      for (const p of pinsData) {
        const el = poseEl("pinList").querySelector('.pin-dist[data-id="' + p.id + '"]');
        if (el) el.textContent = fmtDist(p.distance_m);
      }
      pinRenderSelBar();
    } else {
      pinsSig = sig;
      pinRenderList();
    }
    renderMap();
  }

  // --- ピン操作 ---
  function pinNewBody() {
    const body = {};
    const nm = poseEl("pinNewName");
    if (nm && nm.value.trim()) body.name = nm.value.trim();
    if (pinNewColor) body.color = pinNewColor;
    return body;
  }
  function pinAfterAdd(r) {
    pinSetHint(r.d.message || (r.ok ? "ピンを立てました" : "ピンを立てられませんでした"));
    pinShowErr(r.ok ? "" : r.d.message);
    if (r.ok) {
      const nm = poseEl("pinNewName"); if (nm) nm.value = "";
      if (r.d.pin) pinSelectedId = r.d.pin.id;
    }
    return pollPins();
  }
  async function pinHere() {
    const btn = poseEl("pinHereBtn");
    if (btn) btn.disabled = true;
    const body = pinNewBody(); body.from = "robot";
    const r = await pinApi("/api/pins/add", body);
    if (btn) btn.disabled = false;
    await pinAfterAdd(r);
  }
  async function pinSaveName(id, name) {
    const r = await pinApi("/api/pins/update", { id: id, name: name });
    pinSetHint(r.d.message || "");
    pinShowErr(r.ok ? "" : r.d.message);
    if (r.ok) pinEdit = null;
    await pollPins();
    const saved = pinsData.find((q) => q.id === id);
    if (r.ok && saved && saved.name !== String(name).trim()) pinSetHint("名前は「" + saved.name + "」で保存されました(入力と違います)");
  }
  async function pinSaveColor(id, color) {
    const r = await pinApi("/api/pins/update", { id: id, color: color });
    pinSetHint(r.d.message || "");
    pinShowErr(r.ok ? "" : r.d.message);
    if (r.ok) pinEdit = null;
    pollPins();
  }
  async function pinUpdatePos(id) {
    const r = await pinApi("/api/pins/update", { id: id, from: "robot" });
    pinSetHint(r.d.message || "");
    pinShowErr(r.ok ? "" : r.d.message);
    if (r.ok) pinEdit = null;
    pollPins();
  }
  async function pinDelete(id) {
    const r = await pinApi("/api/pins/delete", { id: id });
    pinSetHint(r.d.message || "");
    pinShowErr(r.ok ? "" : r.d.message);
    if (r.ok) { pinEdit = null; if (pinSelectedId === id) pinSelectedId = null; }
    pollPins();
  }
  // 「行く」: 位置の確かさが「良好」でない時は、ブラウザのダイアログではなく確認バーを挟む。
  function pinGoto(id) {
    const p = pinsData.find((q) => q.id === id);
    if (!p) return Promise.resolve(false);
    if (!locNeedsConfirm() || !poseEl("goalConfirmBar")) { clearPendingGoal(); return pinGotoNow(id); }
    pendingGoal = null; pendingPinGo = id;
    poseEl("goalConfirmText").textContent = "位置が不確かです(位置の確かさ: " + (locStatus.label || "-") +
      ")。このままピン「" + p.name + "」へ走行しますか？";
    poseEl("goalConfirmBar").hidden = false;
    renderMap();
    return Promise.resolve(false);
  }
  async function pinGotoNow(id) {
    const r = await pinApi("/api/pins/goto", { id: id });
    pinSetHint(r.d.message || "");
    pinShowErr(r.ok ? "" : r.d.message);
    if (r.ok) { goalDismissedAt = null; goalMsg = r.d.message || ""; }
    pollGoalStatus();
    return r.ok;
  }

  // --- 地図でピンを置く(クリック+ドラッグで向き。「現在位置を指定」と同じ操作) ---
  function pinPlaceShowBar(show) {
    const bar = poseEl("pinPlaceBar");
    if (bar) bar.hidden = !show;
    if (show && pinPending) {
      poseEl("pinPlaceText").textContent = "ピンを (" + pinPending.x.toFixed(2) + ", " + pinPending.y.toFixed(2) +
        ") m 向き " + pinPending.yaw_deg.toFixed(0) + "° に置きますか";
    }
  }
  function setPinMode(on) {
    if (on && poseMode) setPoseMode(false);
    pinMode = !!on;
    pinStart = null; pinCur = null; pinDrag = null; pinPending = null;
    pinPlaceShowBar(false);
    const btn = poseEl("pinToolBtn");
    if (btn) { btn.textContent = pinMode ? "ピンの配置を終了" : "地図でピンを置く"; btn.classList.toggle("primary", pinMode); }
    if (pinMode) {
      goalHeadMode = false; updateGoalHeadBtn();
      mapWallPick = null;
      if (typeof boundaryPickMode !== "undefined") boundaryPickMode = false;
      clearPendingGoal();
      pinSetHint("地図上でピンを置く場所をクリックし、向きたい方向へドラッグしてください(ドラッグせず2点目をクリックでも可)。");
    }
    canvas.style.cursor = pinMode ? "copy" : "";
    renderMap();
  }
  function pinFinalize(a, b) {
    if (Math.hypot(b[0] - a[0], b[1] - a[1]) < 0.05) {
      pinStart = null; pinCur = null;
      pinSetHint("向きの点が近すぎます。ピンの位置から少し離れた点を指定してください。");
      renderMap();
      return;
    }
    pinPending = mapPoseFromWorldPoints(a, b);
    pinStart = null; pinCur = null;
    pinPlaceShowBar(true);
    pinSetHint("矢印がピンの位置と向きです。名前・色は上の欄で指定できます。合っていれば[置く]。");
    renderMap();
  }
  function drawPinTool() {
    if (!mapMeta) return;
    const col = pinNewColor || "#e11d48";
    if (pinStart) {
      const p = worldToPixel(pinStart[0], pinStart[1]);
      if (pinCur) {
        const q = worldToPixel(pinCur[0], pinCur[1]);
        poseArrow(p, Math.atan2(q[1] - p[1], q[0] - p[0]), col, true);
      } else {
        ctx.save(); ctx.fillStyle = col; ctx.beginPath(); ctx.arc(p[0], p[1], 8, 0, Math.PI * 2); ctx.fill(); ctx.restore();
      }
    }
    if (pinPending) {
      const p = worldToPixel(pinPending.x, pinPending.y);
      poseArrow(p, mapPoseScreenAngle(mapView, pinPending.yaw_deg * Math.PI / 180), col, false);
    }
  }
  canvas.addEventListener("pointerdown", (ev) => {
    if (!pinMode || !mapMeta) return;
    if (ev.pointerType === "mouse" && ev.button !== 0) return;
    ev.stopImmediatePropagation();
    ev.preventDefault();
    try { canvas.setPointerCapture(ev.pointerId); } catch (e) { /* 無視 */ }
    const p = mapEventToPixel(ev);
    pinDrag = { id: ev.pointerId, startPx: p, startWorld: pixelToWorld(p[0], p[1]), moved: false };
    mapClickSuppressUntil = Date.now() + 600;
  });
  canvas.addEventListener("pointermove", (ev) => {
    if (!pinMode || !mapMeta) return;
    const p = mapEventToPixel(ev);
    if (pinDrag && ev.pointerId === pinDrag.id) {
      if (!pinDrag.moved && Math.hypot(p[0] - pinDrag.startPx[0], p[1] - pinDrag.startPx[1]) >= POSE_DRAG_MIN_PX) {
        pinDrag.moved = true;
        pinStart = pinDrag.startWorld;
        pinPending = null; pinPlaceShowBar(false);
      }
      if (pinDrag.moved) { pinCur = pixelToWorld(p[0], p[1]); renderMap(); }
    } else if (!pinDrag && pinStart && !pinPending && ev.pointerType === "mouse") {
      pinCur = pixelToWorld(p[0], p[1]);
      renderMap();
    }
  });
  function pinDragEnd(ev) {
    if (!pinDrag || ev.pointerId !== pinDrag.id) return;
    const d = pinDrag;
    pinDrag = null;
    mapClickSuppressUntil = Date.now() + 600;
    if (ev.type === "pointercancel") { pinStart = null; pinCur = null; renderMap(); return; }
    const p = mapEventToPixel(ev);
    if (d.moved) {
      pinFinalize(d.startWorld, pixelToWorld(p[0], p[1]));
    } else if (!pinStart || pinPending) {
      pinPending = null; pinPlaceShowBar(false);
      pinStart = d.startWorld; pinCur = null;
      pinSetHint("ピンの向きの点をもう一度クリックしてください(または置く場所からドラッグ)。");
      renderMap();
    } else {
      pinFinalize(pinStart, d.startWorld);
    }
  }
  canvas.addEventListener("pointerup", pinDragEnd);
  canvas.addEventListener("pointercancel", pinDragEnd);
  canvas.addEventListener("click", (ev) => { if (pinMode) ev.stopImmediatePropagation(); }, true);
  async function pinPlaceOk() {
    if (!pinPending) return;
    const body = pinNewBody();
    body.from = { x: pinPending.x, y: pinPending.y, yaw_deg: pinPending.yaw_deg };
    const r = await pinApi("/api/pins/add", body);
    await pinAfterAdd(r);
    if (r.ok) { pinPending = null; pinStart = null; pinCur = null; pinPlaceShowBar(false); setPinMode(false); pinSetHint(r.d.message || ""); }
  }
  function pinPlaceRetry() {
    pinPending = null; pinStart = null; pinCur = null;
    pinPlaceShowBar(false);
    pinSetHint("やり直し: ピンを置く場所をクリックし、向きたい方向へドラッグしてください。");
    renderMap();
  }

  // --- 地図上のピンをタップして選択(目的地クリックより先に処理する) ---
  canvas.addEventListener("click", (ev) => {
    if (!pinsOn || poseMode || pinMode || mapWallPick || !mapMeta || !pinsData.length) return;
    if (Date.now() < mapClickSuppressUntil) return;
    if (typeof boundaryPickMode !== "undefined" && boundaryPickMode) return;
    const p = mapEventToPixel(ev);
    const id = mapPinHit(mapView, mapCssW, mapCssH, pinsData, p[0], p[1], PIN_HIT_PX);
    if (id === null) return;
    ev.stopImmediatePropagation();
    selectPin(id, true);
  }, true);

  // --- 向き付きの目的地: ONの間は、ドラッグで到着時の向きも指定する(OFFの間は従来どおりドラッグで地図移動) ---
  function updateGoalHeadBtn() {
    const b = poseEl("goalHeadBtn");
    if (!b) return;
    b.textContent = goalHeadMode ? "向き指定: ON" : "向き指定: OFF";
    b.classList.toggle("primary", goalHeadMode);
    if (!pinMode && !poseMode) canvas.style.cursor = goalHeadMode ? "alias" : "";
  }
  function setGoalHeadMode(on) {
    goalHeadMode = !!on;
    gHeadDrag = null; gHeadCur = null;
    if (goalHeadMode) {
      if (poseMode) setPoseMode(false);
      if (pinMode) setPinMode(false);
      pinSetHint("目的地の地点でドラッグすると、到着時の向きも指定できます(ドラッグしなければ向きは指定しません)。");
    }
    updateGoalHeadBtn();
    renderMap();
  }
  canvas.addEventListener("pointerdown", (ev) => {
    if (!goalHeadMode || poseMode || pinMode || mapWallPick || !mapMeta) return;
    if (typeof boundaryPickMode !== "undefined" && boundaryPickMode) return;
    if (ev.pointerType === "mouse" && ev.button !== 0) return;
    ev.stopImmediatePropagation();
    ev.preventDefault();
    try { canvas.setPointerCapture(ev.pointerId); } catch (e) { /* 無視 */ }
    const p = mapEventToPixel(ev);
    gHeadDrag = { id: ev.pointerId, startPx: p, startWorld: pixelToWorld(p[0], p[1]), moved: false };
    mapClickSuppressUntil = Date.now() + 600;
  });
  canvas.addEventListener("pointermove", (ev) => {
    if (!gHeadDrag || ev.pointerId !== gHeadDrag.id) return;
    const p = mapEventToPixel(ev);
    if (!gHeadDrag.moved && Math.hypot(p[0] - gHeadDrag.startPx[0], p[1] - gHeadDrag.startPx[1]) >= GOAL_HEAD_MIN_PX) gHeadDrag.moved = true;
    if (gHeadDrag.moved) { gHeadCur = pixelToWorld(p[0], p[1]); renderMap(); }
  });
  function goalHeadEnd(ev) {
    if (!gHeadDrag || ev.pointerId !== gHeadDrag.id) return;
    const d = gHeadDrag;
    gHeadDrag = null; gHeadCur = null;
    mapClickSuppressUntil = Date.now() + 600;
    if (ev.type === "pointercancel") { renderMap(); return; }
    const p = mapEventToPixel(ev);
    const sw = d.startWorld;
    if (d.moved) {
      const pose = mapPoseFromWorldPoints(sw, pixelToWorld(p[0], p[1]));
      requestGoal(sw[0], sw[1], pose.yaw_deg);
    } else {
      requestGoal(sw[0], sw[1]);
    }
    renderMap();
  }
  canvas.addEventListener("pointerup", goalHeadEnd);
  canvas.addEventListener("pointercancel", goalHeadEnd);

  // --- 目的地の進捗・到着誤差 ---
  function goalReasonText(g) {
    const ev = g.event;
    const m = { rejected: "Nav2が目的地を受け付けませんでした", estop_blocked: "緊急停止中のため送れませんでした",
      outside_boundary: "走行可能範囲の外のため拒否されました", server_unavailable: "Nav2が起動していません",
      send_failed: "Nav2へ送れませんでした", result_failed: "結果を取得できませんでした" };
    if (m[ev]) return m[ev];
    if (g.state === "aborted") return "経路が見つからない、または進めなくなりました";
    return "";
  }
  function goalUncertaintyText() {
    if (locStatus && locStatus.localization_active && locStatus.pos_std_m !== null && locStatus.pos_std_m !== undefined) {
      return "自己位置のばらつき ±" + Math.round(locStatus.pos_std_m * 100) + " cm(これより細かい精度は自己位置推定の限界です)";
    }
    return "自己位置推定(SLAM/AMCL)の誤差が精度の限界です";
  }
  function renderGoalBar() {
    const bar = poseEl("goalProgressBar");
    if (!bar) return;
    const g = goalVisible();
    if (!g) { bar.hidden = true; return; }
    bar.hidden = false;
    const gl = g.goal;
    const name = gl.pin_name ? "ピン「" + gl.pin_name + "」" : "目的地 (" + gl.x.toFixed(2) + ", " + gl.y.toFixed(2) + ")";
    let t = "";
    if (g.state === "sent") {
      t = name + "へ走行中: 残り " + fmtDist(g.distance_m) + "(直線距離)";
      if (gl.yaw_specified && g.yaw_err_deg !== null && g.yaw_err_deg !== undefined) t += " / 向きの差 " + Math.round(Math.abs(g.yaw_err_deg)) + "°";
      t += " / " + Math.round(g.sent_age_s || 0) + "秒経過";
    } else if (g.state === "succeeded") {
      const a = g.arrival;
      t = name + "に到着しました";
      if (a) {
        t += ": 目的地との差 " + a.dist_cm.toFixed(1) + " cm";
        if (gl.yaw_specified) t += " / 向きの差 " + Math.round(Math.abs(a.yaw_err_deg)) + "°";
      }
    } else if (g.state === "canceled") {
      t = name + "への走行を中止しました。";
    } else {
      t = name + "へ到達できませんでした";
      const why = goalReasonText(g);
      if (why) t += "(" + why + ")";
      t += "。残り " + fmtDist(g.distance_m);
    }
    if (gl.snapped) t += " / クリック位置が走行できない場所だったため " + Math.round(gl.shift_m * 100) + " cm ずらしました";
    if (g.state === "sent" || g.state === "succeeded") t += " / " + goalUncertaintyText();
    poseEl("goalProgressText").textContent = t;
    poseEl("goalCancelRunBtn").hidden = g.state !== "sent";
    poseEl("goalCloseBtn").hidden = g.state === "sent";
  }
  async function pollGoalStatus() {
    if (!goalOn) return;
    try {
      const res = await fetch("/api/goal/status");
      if (!res.ok) return;
      const d = await res.json();
      goalSt = d.goal ? d : null;
    } catch (e) { return; }
    renderGoalBar();
    renderMap();
  }
  async function goalCancelRun() {
    const r = await pinApi("/api/goal/cancel", {});
    pinShowErr(r.ok ? "" : r.d.message);
    pinSetHint(r.d.message || "");
    pollGoalStatus();
  }
  function goalClose() {
    if (goalSt) goalDismissedAt = goalSt.sent_at;
    renderGoalBar();
    renderMap();
  }
  // 目的地クリック(地図)。location確認バー経由もここへ集約する。
  async function sendGoalNow(x, y, yawDeg) {
    const body = { x: x, y: y };
    if (yawDeg !== undefined && yawDeg !== null) body.yaw_deg = yawDeg;
    const r = await pinApi("/api/goal", body);
    pinShowErr(r.ok ? "" : r.d.message);
    if (r.ok) { goalDismissedAt = null; goalMsg = r.d.message || ""; }
    else { pinSetHint(r.d.message || "目的地を送れませんでした"); }
    if (r.ok && r.d.message && r.d.snapped) pinSetHint(r.d.message);
    pollGoalStatus();
    return r.ok;
  }

  if (pinsOn) {
    const sw = poseEl("pinNewSwatches");
    if (sw) {
      const draw = () => {
        sw.textContent = "";
        const auto = mkEl("button", "pin-swatch auto" + (pinNewColor === null ? " on" : ""), "自動");
        auto.type = "button"; auto.addEventListener("click", () => { pinNewColor = null; draw(); renderMap(); });
        sw.appendChild(auto);
        sw.appendChild(pinColorSwatches(pinNewColor, (c) => { pinNewColor = c; draw(); renderMap(); }));
        // 色の選択は小さなドロップダウン(details)。いまの色を丸で表示し、選んだら閉じる。
        const dot = poseEl("pinColorDot"), txt = poseEl("pinColorTxt"), dd = poseEl("pinColorDd");
        if (dot) dot.style.background = pinNewColor || "conic-gradient(#e11d48,#2563eb,#16a34a,#d97706,#e11d48)";
        if (txt) txt.textContent = pinNewColor ? "指定" : "自動";
        if (dd) dd.open = false;
      };
      draw();
    }
    poseEl("pinHereBtn").addEventListener("click", pinHere);
    poseEl("pinToolBtn").addEventListener("click", () => setPinMode(!pinMode));
    poseEl("pinPlaceOkBtn").addEventListener("click", pinPlaceOk);
    poseEl("pinPlaceRetryBtn").addEventListener("click", pinPlaceRetry);
    poseEl("pinPlaceCancelBtn").addEventListener("click", () => setPinMode(false));
    poseEl("pinSelGoBtn").addEventListener("click", () => { if (pinSelectedId) pinGoto(pinSelectedId); });
    poseEl("pinSelCloseBtn").addEventListener("click", () => { pinSelectedId = null; pinRenderList(); renderMap(); });
    NKField.guard("pinNewName");
    pollPins();
    NKField.poll(pollPins, 2000);
  }
  if (goalOn) {
    poseEl("goalCancelRunBtn").addEventListener("click", goalCancelRun);
    poseEl("goalCloseBtn").addEventListener("click", goalClose);
    if (poseEl("goalHeadBtn")) poseEl("goalHeadBtn").addEventListener("click", () => setGoalHeadMode(!goalHeadMode));
    updateGoalHeadBtn();
    pollGoalStatus();
    NKField.poll(pollGoalStatus, 700);
  }

  // --- 表示回転のUI・サーバー同期 ---
  function drawWallPick() {
    if (!mapWallPick || !mapMeta) return;
    ctx.save();
    ctx.fillStyle = "#c026d3"; ctx.strokeStyle = "#c026d3"; ctx.lineWidth = 2; ctx.setLineDash([]);
    const ps = mapWallPick.map(([x, y]) => worldToPixel(x, y));
    for (const p of ps) { ctx.beginPath(); ctx.arc(p[0], p[1], 5, 0, Math.PI * 2); ctx.fill(); }
    if (ps.length === 2) { ctx.beginPath(); ctx.moveTo(ps[0][0], ps[0][1]); ctx.lineTo(ps[1][0], ps[1][1]); ctx.stroke(); }
    ctx.restore();
  }
  // 【2026-10-08】地図の操作は地図の上の1本のツールバー(#mapToolbar)にまとめる。回転は「回転」ボタンのポップオーバー。
  // 説明文は「?」ポップオーバー/title、メッセージは地図下端の通知ストリップ(#mapNotes)へ出す(キャンバスの高さを奪わない)。
  const mapToolbar = document.getElementById("mapToolbar");
  const mapNotesBox = document.getElementById("mapNotes");
  const MAP_ROT_HTML =
    '<span class="tb-wrap"><button id="mapRotMenuBtn" type="button" class="tb-btn" data-pop="mapRotPop" aria-expanded="false" aria-controls="mapRotPop" title="地図の表示を回します(地図・Nav2の座標は変わりません)">回転 <b id="mapRotBadge">0°</b> &#9662;</button>' +
    '<div id="mapRotPop" class="tb-pop" hidden>' +
    '<div class="tb-row"><span class="tb-lbl">地図の向き</span>' +
    '<input id="mapRotSlider" type="range" min="-180" max="180" step="0.5" value="0" style="flex:1 1 120px;min-width:100px;">' +
    '<input id="mapRotNum" type="number" min="-180" max="180" step="0.5" value="0" style="width:70px;">' +
    '<span class="tb-lbl">°</span></div>' +
    '<div class="tb-row">' +
    '<button id="mapRot90Btn" type="button">90°回転</button>' +
    '<button id="mapRot0Btn" type="button">0°に戻す</button>' +
    '<button id="mapRotAutoBtn" type="button" data-close="1">壁に合わせて自動</button>' +
    '<button id="mapRotPickBtn" type="button" data-close="1">壁を2点で指定</button></div>' +
    '<div class="tb-row"><button id="mapRotSaveBtn" type="button">この向きを地図の基準にして保存</button></div>' +
    '<p class="hint" style="margin:0;">表示だけを回します(地図・Nav2の座標は変わりません)。走行範囲は表示で水平・垂直に見える矩形になります。</p>' +
    '</div></span>';
  if (mapToolbar) mapToolbar.insertAdjacentHTML("afterbegin", MAP_ROT_HTML);
  else canvas.insertAdjacentHTML("afterend", '<div class="map-rot" style="margin-top:6px;">' + MAP_ROT_HTML + '</div>');
  const mapRotHint = document.createElement("p");
  mapRotHint.id = "mapRotHint"; mapRotHint.className = "map-note";
  if (mapNotesBox) mapNotesBox.appendChild(mapRotHint); else canvas.parentNode.appendChild(mapRotHint);
  const mapRotSlider = document.getElementById("mapRotSlider");
  const mapRotNum = document.getElementById("mapRotNum");
  const mapRotBadge = document.getElementById("mapRotBadge");
  let mapRotPostTimer = null;
  NKField.guard([mapRotSlider, mapRotNum]);
  NKField.link(mapRotSlider, mapRotNum);
  NKField.guard(mapRotNum, { rangeText: "範囲は -180〜180 です(範囲外の値は -180〜180 に折り返して反映します)" });
  function setMapRotDeg(deg, fromServer) {
    if (!isFinite(deg)) return;
    deg = Math.round(mapNormDeg(deg) * 100) / 100;
    mapRotDeg = deg;
    mapRotSlider.value = String(deg);
    if (mapRotBadge) mapRotBadge.textContent = deg + "°";
    if (document.activeElement !== mapRotNum) mapRotNum.value = String(deg);
    mapView = Object.assign({}, mapView, { rot: deg * Math.PI / 180 });
    renderMap();
    if (fromServer) {
      NKField.set(mapRotNum, String(deg), { force: true });  // サーバーの値を基準にする(未送信の印を外す)
    } else {
      mapRotLastEdit = Date.now();
      if (mapRotPostTimer) clearTimeout(mapRotPostTimer);
      mapRotPostTimer = setTimeout(() => { mapRotSend(deg); }, 150);
    }
  }
  // 向きをサーバーへ送り、サーバーが反映した角度で確かめる。失敗・不一致は画面に出す(黙って元に戻さない)
  async function mapRotSend(deg) {
    NKField.bump("mapview");
    const r = await mapRotPost("/api/map_view/set", { angle_deg: deg });
    NKField.bump("mapview");
    if (r.ok && typeof r.d.angle_deg === "number") {
      if (Math.abs(mapNormDeg(r.d.angle_deg - deg)) > 0.01) {
        mapRotHint.textContent = "地図の向きが反映されていません(サーバー値 " + r.d.angle_deg + "°)";
      } else if (mapRotDeg === deg) {
        NKField.set(mapRotNum, String(deg), { force: true });  // 送った値が反映されたので、未送信の印を外す
      }
    } else if (mapRotDeg === deg) {
      mapRotHint.textContent = (r.d.message || r.d.error || "地図の向きをサーバーへ送れませんでした") + "(この画面の向きだけ変わっています。もう一度操作してください)";
    }
  }
  async function mapRotPost(path, body) {
    try {
      const res = await fetch(path, { method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body || {}) });
      let d = null;
      try { d = await res.json(); } catch (e2) { /* 無視 */ }
      return { ok: res.ok && !(d && d.ok === false), d: d || {}, status: res.status };
    } catch (e) {
      return { ok: false, d: { message: "通信エラー" }, status: 0 };
    }
  }
  mapRotSlider.addEventListener("input", () => setMapRotDeg(parseFloat(mapRotSlider.value), false));
  mapRotNum.addEventListener("change", () => {
    const v = parseFloat(mapRotNum.value);
    if (isFinite(v)) setMapRotDeg(v, false); else mapRotNum.value = String(mapRotDeg);
  });
  document.getElementById("mapRot90Btn").addEventListener("click", () => setMapRotDeg(mapRotDeg + 90, false));
  document.getElementById("mapRot0Btn").addEventListener("click", () => setMapRotDeg(0, false));
  document.getElementById("mapRotAutoBtn").addEventListener("click", async () => {
    mapRotHint.textContent = "壁の向きを推定中...";
    const r = await mapRotPost("/api/map_view/estimate", { apply: true });
    if (r.ok && r.d.angle_deg !== undefined) {
      mapRotLastEdit = Date.now();
      setMapRotDeg(r.d.angle_deg, true);
      mapRotHint.textContent = "壁の向き " + r.d.wall_angle_deg.toFixed(1) + "° を検出し、表示を " + r.d.angle_deg.toFixed(1) +
        "° 回しました(確からしさ " + Math.round(r.d.confidence * 100) + "%)。ずれていれば数値で微調整できます。";
    } else {
      mapRotHint.textContent = r.d.message || r.d.error || "推定に失敗しました";
    }
  });
  document.getElementById("mapRotPickBtn").addEventListener("click", () => {
    mapWallPick = [];
    mapRotHint.textContent = "壁に沿った2点を地図上でクリックしてください(1点目)";
    renderMap();
  });
  document.getElementById("mapRotSaveBtn").addEventListener("click", async () => {
    const nameEl = document.getElementById("mapSaveName");
    const body = {};
    if (nameEl && nameEl.value.trim()) body.name = nameEl.value.trim();
    const r = await mapRotPost("/api/map_view/save", body);
    mapRotHint.textContent = r.d.message || r.d.error || (r.ok ? "保存しました" : "保存に失敗しました");
  });
  // 壁の2点指定(目的地クリックより先に処理する)。
  canvas.addEventListener("click", async (ev) => {
    if (!mapWallPick || !mapMeta) return;
    if (Date.now() < mapClickSuppressUntil) return;
    ev.stopImmediatePropagation();
    const p = mapEventToPixel(ev);
    mapWallPick.push(pixelToWorld(p[0], p[1]));
    if (mapWallPick.length < 2) {
      mapRotHint.textContent = "壁に沿った2点目をクリックしてください";
      renderMap();
      return;
    }
    const pts = mapWallPick;
    mapRotHint.textContent = "壁に合わせています...";
    const r = await mapRotPost("/api/map_view/set", { wall_points: [pts[0][0], pts[0][1], pts[1][0], pts[1][1]] });
    mapWallPick = null;
    if (r.ok && r.d.angle_deg !== undefined) {
      mapRotLastEdit = Date.now();
      setMapRotDeg(r.d.angle_deg, true);
      mapRotHint.textContent = "指定した壁が水平に見えるよう " + r.d.angle_deg.toFixed(1) + "° に回しました";
    } else {
      mapRotHint.textContent = r.d.message || r.d.error || "指定に失敗しました";
      renderMap();
    }
  }, true);
  async function pollMapView() {
    const tk = NKField.ticket("mapview");
    try {
      const res = await fetch("/api/map_view");
      if (!res.ok) return;
      const d = await res.json();
      if (!NKField.fresh(tk) || NKField.busy(mapRotSlider) || NKField.busy(mapRotNum)) return;  // 送信前の古い結果・操作中は反映しない
      if (typeof d.angle_deg === "number" && Date.now() - mapRotLastEdit > 2500 &&
          Math.abs(mapNormDeg(d.angle_deg - mapRotDeg)) > 1e-6) {
        setMapRotDeg(d.angle_deg, true);
      }
    } catch (e) { /* 無視 */ }
  }
  pollMapView();
  NKField.poll(pollMapView, 3000);

  // --- 拡大縮小(ホイール/ピンチ)・移動(ドラッグ)・全体表示ボタン ---
  {
    const fitHtml = '<button id="mapFitBtn" type="button" class="tb-btn" title="地図全体が見えるように戻す(ホイール/ピンチで拡大縮小、ドラッグで移動)">全体表示</button>';
    if (mapToolbar) mapToolbar.insertAdjacentHTML("afterbegin", fitHtml);
    else canvas.insertAdjacentHTML("afterend", '<div class="map-tools" style="margin-top:6px;">' + fitHtml + '</div>');
  }
  // ツールバーのポップオーバー(data-pop=ポップオーバーのid)。外側クリック/Escで閉じ、data-close付きボタンは押したら閉じる。
  function mapPopCloseAll() {
    document.querySelectorAll(".tb-pop").forEach((p) => { p.hidden = true; });
    document.querySelectorAll("[data-pop]").forEach((b) => b.setAttribute("aria-expanded", "false"));
  }
  document.addEventListener("click", (ev) => {
    const t = ev.target;
    const opener = t && t.closest ? t.closest("[data-pop]") : null;
    if (opener) {
      const pop = document.getElementById(opener.getAttribute("data-pop"));
      const willOpen = !!pop && pop.hidden;
      mapPopCloseAll();
      if (pop && willOpen) { pop.hidden = false; opener.setAttribute("aria-expanded", "true"); }
      return;
    }
    if (t && t.closest && t.closest(".tb-pop")) {
      if (t.closest("[data-close]")) mapPopCloseAll();
      return;
    }
    mapPopCloseAll();
  });
  document.addEventListener("keydown", (ev) => { if (ev.key === "Escape") mapPopCloseAll(); });
  // 通知ストリップ: 文字(.map-note)はクリックで消え、15秒で自動的に消える。
  if (mapNotesBox) {
    const noteTimers = new WeakMap();
    new MutationObserver((muts) => {
      for (const m of muts) {
        const el = m.target.nodeType === 3 ? m.target.parentElement : m.target;
        if (!el || !el.classList || !el.classList.contains("map-note")) continue;
        clearTimeout(noteTimers.get(el));
        if (el.textContent.trim()) noteTimers.set(el, setTimeout(() => { el.textContent = ""; }, 15000));
      }
    }).observe(mapNotesBox, { childList: true, characterData: true, subtree: true });
    mapNotesBox.addEventListener("click", (ev) => {
      const n = ev.target.closest ? ev.target.closest(".map-note") : null;
      if (n) n.textContent = "";
    });
  }
  document.getElementById("mapFitBtn").addEventListener("click", () => {
    mapViewUser = false;
    renderMap();
  });
  const mapPointers = new Map();
  let mapDrag = null;   // {x, y, view, moved}
  let mapPinch = null;  // {dist, mx, my}
  canvas.addEventListener("pointerdown", (ev) => {
    mapPointers.set(ev.pointerId, mapEventToPixel(ev));
    try { canvas.setPointerCapture(ev.pointerId); } catch (e) { /* 無視 */ }
    if (mapPointers.size === 1) {
      const p = mapPointers.get(ev.pointerId);
      mapDrag = { x: p[0], y: p[1], view: mapView, moved: false };
      mapPinch = null;
    } else if (mapPointers.size === 2) {
      const [a, b] = Array.from(mapPointers.values());
      mapPinch = { dist: Math.hypot(a[0] - b[0], a[1] - b[1]) || 1, mx: (a[0] + b[0]) / 2, my: (a[1] + b[1]) / 2 };
      mapDrag = null;
      mapClickSuppressUntil = Date.now() + 500;
    }
  });
  canvas.addEventListener("pointermove", (ev) => {
    if (!mapPointers.has(ev.pointerId)) return;
    mapPointers.set(ev.pointerId, mapEventToPixel(ev));
    if (!mapMeta) return;
    if (mapPointers.size === 1 && mapDrag) {
      const p = mapPointers.get(ev.pointerId);
      const dx = p[0] - mapDrag.x, dy = p[1] - mapDrag.y;
      if (!mapDrag.moved && Math.hypot(dx, dy) > 6) mapDrag.moved = true;
      if (mapDrag.moved) {
        mapViewUser = true;
        mapView = mapPanBy(mapDrag.view, dx, dy);
        mapClickSuppressUntil = Date.now() + 350;
        renderMap();
      }
    } else if (mapPointers.size === 2 && mapPinch) {
      const [a, b] = Array.from(mapPointers.values());
      const dist = Math.hypot(a[0] - b[0], a[1] - b[1]) || 1;
      const mx = (a[0] + b[0]) / 2, my = (a[1] + b[1]) / 2;
      mapViewUser = true;
      mapView = mapZoomAt(mapView, mapCssW, mapCssH, mx, my, dist / mapPinch.dist);
      // 2本指の中点の移動分だけ地図も動かす
      mapView = mapPanBy(mapView, mx - mapPinch.mx, my - mapPinch.my);
      mapPinch = { dist, mx, my };
      mapClickSuppressUntil = Date.now() + 500;
      renderMap();
    }
  });
  function mapPointerEnd(ev) {
    if (!mapPointers.has(ev.pointerId)) return;
    mapPointers.delete(ev.pointerId);
    if (mapDrag && mapDrag.moved) mapClickSuppressUntil = Date.now() + 350;
    if (mapPointers.size < 2) mapPinch = null;
    if (mapPointers.size === 0) mapDrag = null;
  }
  canvas.addEventListener("pointerup", mapPointerEnd);
  canvas.addEventListener("pointercancel", mapPointerEnd);
  canvas.addEventListener("wheel", (ev) => {
    if (!mapMeta) return;
    ev.preventDefault();
    const p = mapEventToPixel(ev);
    mapViewUser = true;
    mapView = mapZoomAt(mapView, mapCssW, mapCssH, p[0], p[1], Math.exp(-ev.deltaY * 0.0015));
    renderMap();
  }, { passive: false });
  if (window.ResizeObserver) new ResizeObserver(() => renderMap()).observe(canvas);
  window.addEventListener("resize", renderMap);
"""
