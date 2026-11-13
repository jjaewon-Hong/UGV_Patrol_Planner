// Web demo: loads the maps and the WebAssembly module, runs the tick loop and wires the controls.
// All planning and mission logic is in the module (cpp/); this file only passes input and draws state.
import { loadUgv, prepareMap, TICK_S } from './wasm.js';
import { View2D } from './view2d.js';

const $ = (id) => document.getElementById(id);
const TICKS_PER_S = { 1: 1 / TICK_S, 5: 5 / TICK_S, 20: 20 / TICK_S };  // 1x = real time
const CLASS_NAMES = ['장애물', '건물', '물', '나무'];
const MODE_HINTS = {
  flag: '빈 칸을 클릭하면 순찰 깃발을 놓고, 깃발을 클릭하면 지웁니다. 장애물 칸에 놓으면 경보가 납니다.',
  obstacle: '클릭하거나 끌어서 장애물을 놓습니다. 경로를 막으면 바로 새 경로를 계획합니다. (지우려면 ↺ 처음)',
  plan: '두 칸을 차례로 클릭하면 순찰과 별개로 A* 경로를 계산해 보여 줍니다.',
};

const app = {
  ugv: null,          // WebAssembly wrapper
  map: null,
  world: null,
  state: null,        // last state from the module
  prevPos: null,      // position before the last tick, for smooth drawing
  acc: 0,             // fraction of a tick not simulated yet
  speed: 5,
  mode: 'flag',
  view: '2d',
  view2d: new View2D($('map2d')),
  view3d: null,
  maxSlope: null,     // null = the map's own limit
  learned: [],        // obstacles found by the sensor or added by clicks: [[r, c], ...]
  trail: [],
  plan: { a: null, b: null, path: null },
  hover: null,
  energyOn: false,
  zScale: 5,
  draggingBattery: false,
  stroke: null,       // cells added in the current obstacle drag
};

// ------------------------------------------------------------------ log
function log(text, kind = 'info') {
  const list = $('log');
  list.querySelector('.empty')?.remove();
  const li = document.createElement('li');
  li.className = kind;
  const time = document.createElement('time');
  time.textContent = `${(app.state?.t ?? 0).toFixed(1)}s`;
  const span = document.createElement('span');
  span.textContent = text;
  li.append(time, span);
  list.prepend(li);
  while (list.children.length > 80) list.lastChild.remove();
}

function clearLog() {
  $('log').innerHTML = '<li class="empty">아직 기록이 없습니다.</li>';
}

// ------------------------------------------------------------------ world
/** Build the world again. keep = true keeps flags, known obstacles and battery (used by the slope slider). */
function buildWorld({ keep = false } = {}) {
  const map = app.map;
  const points = keep && app.state ? app.state.status.points.map((p) => p.cell) : map.points;
  const batteryWh = keep && app.state ? app.state.battery : map.initial_battery_wh;
  if (!keep) {
    app.learned = [];
    app.maxSlope = null;
    clearLog();
  }
  app.world = app.ugv.createWorld(map, { maxSlopeDeg: app.maxSlope ?? map.max_slope_deg, batteryWh, extraObstacles: app.learned });
  for (const [r, c] of points) app.world.command(`add ${r} ${c}`);
  app.world.command('start');
  app.trail = [];
  app.plan = { a: null, b: null, path: null };
  app.prevPos = null;
  app.acc = 0;
  app.state = null;
  consume(app.world.state());

  app.view2d.setObstacles(app.learned);
  refreshEnergy();
  app.view3d?.setMap(map, app.learned, app.zScale);
  $('plan-readout').hidden = true;

  const slope = app.maxSlope ?? map.max_slope_deg;
  $('in-slope').value = slope;
  $('out-slope').textContent = `${slope}°`;
  $('in-batt').max = map.capacity_wh;
  $('slope-note').textContent = `이 한계로 막히는 이동 ${countBlockedMoves()}곳 (지도 기본값 ${map.max_slope_deg}°). 바꾸면 기지에서 다시 출발합니다.`;
}

/** Moves between two free neighbour cells that the slope limit forbids. */
function countBlockedMoves() {
  const n = app.map.n, known = app.view2d.known;
  let blocked = 0;
  for (let r = 0; r < n; r++) {
    for (let c = 0; c < n; c++) {
      if (known[r * n + c]) continue;
      if (c + 1 < n && !known[r * n + c + 1] && !app.world.traversable([r, c], [r, c + 1])) blocked++;
      if (r + 1 < n && !known[(r + 1) * n + c] && !app.world.traversable([r, c], [r + 1, c])) blocked++;
    }
  }
  return blocked;
}

function refreshEnergy() {
  app.view2d.setEnergy(app.energyOn ? app.world.returnEnergy() : null);
}

async function loadMap(id) {
  const res = await fetch(`maps/${id}.json`);
  if (!res.ok) throw new Error(`지도 파일을 불러오지 못했습니다 (${id})`);
  app.map = prepareMap(await res.json());
  $('map-about').textContent = `${app.map.about} 칸 크기 ${app.map.cell_size_m} m, 기지 (${app.map.base.join(', ')}).`;
  app.view2d.setMap(app.map);
  buildWorld();
}

// ------------------------------------------------------------------ state from the module
/** Take one state: log what changed, remember new obstacles, extend the trail. */
function consume(s) {
  const old = app.state;
  app.state = s;
  for (const a of s.alarms) log(`경보: ${a.text}`, 'alarm');
  if (s.new_obstacles.length) {
    app.learned.push(...s.new_obstacles);
    app.view2d.setObstacles(app.learned);
    refreshEnergy();
    app.view3d?.setObstacles(app.learned);
  }
  if (old) {
    if (s.replans > old.replans) log(`재계획 ${s.replans}회째: 새 경로 계산 ${s.last_replan_ms.toFixed(2)} ms`);
    for (const [pid, n] of Object.entries(s.visits)) {
      if (n > (old.visits[pid] ?? 0)) log(`${pid} 도착 (${n}회째)`);
    }
    const state = s.status.state, was = old.status.state;
    if (state !== was && state !== '재계획 중') {
      if (state === '복귀 중') log('배터리 부족: 기지로 복귀 시작');
      else if (s.status.note === '복귀 완료') log('복귀 완료: 기지 도착. 배터리 슬라이더를 올리고 ▶ 시작을 누르면 다시 순찰합니다.');
      else if (state === '정지') log('정지');
      else if (state === '순찰 중' && (was === '대기' || was === '정지')) log('순찰 시작');
      else if (state === '대기') log('대기');
    }
  }
  const last = app.trail[app.trail.length - 1];
  if (!last || last[0] !== s.pos[0] || last[1] !== s.pos[1]) {
    app.trail.push(s.pos);
    if (app.trail.length > 3000) app.trail.shift();
  }
}

function updatePanel() {
  const s = app.state, map = app.map;
  if (!s) return;
  const badge = $('st-state');
  badge.textContent = s.status.state;
  badge.dataset.state = s.status.state;
  $('st-target').textContent = s.status.target ? `→ ${s.status.target}` : (s.status.note ?? '');
  $('st-time').textContent = `${s.t.toFixed(0)}초`;

  const frac = s.battery / s.capacity;
  const need = s.return_wh === null ? null : s.return_wh * map.return_margin;
  const fill = $('st-batt-fill');
  fill.style.width = `${(frac * 100).toFixed(1)}%`;
  fill.classList.toggle('critical', need !== null && s.battery <= need * 1.15);
  fill.classList.toggle('low', need !== null && s.battery <= need * 2 && s.battery > need * 1.15);
  $('st-batt-mark').style.left = `${Math.min(100, (need ?? 0) / s.capacity * 100).toFixed(1)}%`;
  $('st-batt-text').textContent = `배터리 ${s.battery.toFixed(1)} / ${s.capacity} Wh`;
  $('st-return').textContent = need === null ? '기지로 갈 길 없음' : `복귀 기준 ${need.toFixed(1)} Wh`;

  const visits = s.status.points.map((p) => `${p.pid} ${s.visits[p.pid] ?? 0}회`).join(' · ');
  $('st-visits').textContent = visits || '깃발 없음';
  $('st-moves').textContent = `${s.moves}칸 · ${(s.moves * map.cell_size_m / 1000).toFixed(2)} km`;
  $('st-replans').textContent = s.replans ? `${s.replans}회 · 최근 ${s.last_replan_ms.toFixed(2)} ms` : '0회';
  $('st-slope').textContent = `${s.max_slope.toFixed(1)}° / 한계 ${app.maxSlope ?? map.max_slope_deg}°`;

  if (!app.draggingBattery) $('in-batt').value = s.battery;
  $('out-batt').textContent = `${s.battery.toFixed(0)} Wh`;
}

// ------------------------------------------------------------------ frame loop
let lastTs = 0;
function frame(ts) {
  const dt = Math.min(0.1, (ts - lastTs) / 1000);  // a hidden tab does not fast-forward
  lastTs = ts;
  if (app.world) {
    app.acc += dt * TICKS_PER_S[app.speed];
    const due = Math.floor(app.acc);
    app.acc -= due;
    for (let i = 0; i < due; i++) {
      app.prevPos = app.state.pos;
      app.world.tick(1);
      consume(app.world.state());
    }
    // draw the UGV between the previous and the current cell
    const [r1, c1] = app.state.pos, [r0, c0] = app.prevPos ?? app.state.pos;
    const t = app.speed <= 5 ? app.acc : 1;
    const ugv = [r0 + (r1 - r0) * t, c0 + (c1 - c0) * t];
    const f = {
      state: app.state, ugv, trail: app.trail, hidden: app.map.hidden,
      plan: app.plan, hover: app.hover, sensorRadius: app.map.sensor_radius,
    };
    if (app.view === '3d' && app.view3d) app.view3d.draw(f);
    else app.view2d.draw(f);
    updatePanel();
  }
  requestAnimationFrame(frame);
}

// ------------------------------------------------------------------ map clicks
function sameCell(a, b) { return a && b && a[0] === b[0] && a[1] === b[1]; }

function addObstacle(cell) {
  if (sameCell(cell, app.state.pos) || sameCell(cell, app.map.base)) return;
  if (app.stroke?.has(cell.join())) return;
  app.stroke?.add(cell.join());
  app.world.reportObstacle(cell[0], cell[1]);
  consume(app.world.state());
}

function clickFlag(cell) {
  const near = app.state.status.points.find((p) => Math.abs(p.cell[0] - cell[0]) + Math.abs(p.cell[1] - cell[1]) <= 1);
  if (near) {
    app.world.command(`remove ${near.pid}`);
    log(`${near.pid} 깃발 삭제`);
  } else {
    app.world.command(`add ${cell[0]} ${cell[1]}`);
    if (!app.view2d.known[cell[0] * app.map.n + cell[1]]) log(`깃발 추가 (${cell.join(', ')})`);
  }
  consume(app.world.state());
  if (app.state.status.state === '대기' && app.state.status.points.length) {
    app.world.command('start');
    consume(app.world.state());
  }
}

function clickPlan(cell) {
  const p = app.plan;
  if (!p.a || p.b) {
    app.plan = { a: cell, b: null, path: null };
    showPlan('도착 칸을 클릭하세요.');
    return;
  }
  p.b = cell;
  const res = app.world.plan(p.a, p.b);
  p.path = res.found ? res.path : null;
  const ends = `(${p.a.join(', ')}) → (${p.b.join(', ')})`;
  showPlan(res.found
    ? `A* ${ends}: ${res.cells}칸 · ${(res.cells * app.map.cell_size_m).toFixed(0)} m · 오르막 ${res.climb_m.toFixed(1)} m · ` +
      `최대 경사 ${res.max_slope.toFixed(1)}° · 비용 ${res.cost.toFixed(1)} · 계산 ${res.plan_ms.toFixed(2)} ms (이 브라우저에서 20회 평균)`
    : `A* ${ends}: 경로 없음 (장애물이거나, 등판 한계 안에서 갈 수 없는 칸입니다) · 계산 ${res.plan_ms.toFixed(2)} ms`);
}

function showPlan(text) {
  $('plan-readout').hidden = false;
  $('plan-readout').textContent = text;
}

function showHover(cell) {
  app.hover = cell;
  const out = $('hover-readout');
  if (!cell) { out.textContent = '지도 위에 마우스를 올리면 칸 정보가 보입니다.'; out.classList.add('muted'); return; }
  out.classList.remove('muted');
  const i = cell[0] * app.map.n + cell[1];
  const known = app.view2d.known[i];
  const kind = !known ? '빈 칸' : app.map.obstacles[i] ? CLASS_NAMES[app.map.classes[i]] : '직접 놓은 장애물';
  out.textContent = `칸 (${cell.join(', ')}) · 고도 ${app.map.elevation[i].toFixed(1)} m · ${kind}`;
}

function setupMapInput() {
  const canvas = $('map2d');
  // Flags and path picks are taps, so a swipe over the map still scrolls the page on a phone.
  canvas.addEventListener('click', (ev) => {
    const cell = app.world && app.view2d.cellAt(ev);
    if (!cell) return;
    if (app.mode === 'flag') clickFlag(cell);
    else if (app.mode === 'plan') clickPlan(cell);
  });
  canvas.addEventListener('pointerdown', (ev) => {
    const cell = app.world && app.view2d.cellAt(ev);
    if (!cell || app.mode !== 'obstacle') return;
    app.stroke = new Set();
    canvas.setPointerCapture(ev.pointerId);
    addObstacle(cell);
  });
  canvas.addEventListener('pointermove', (ev) => {
    const cell = app.world && app.view2d.cellAt(ev);
    showHover(cell);
    if (cell && app.stroke) addObstacle(cell);
  });
  const end = () => { app.stroke = null; };
  canvas.addEventListener('pointerup', end);
  canvas.addEventListener('pointercancel', end);
  canvas.addEventListener('pointerleave', () => showHover(null));
}

// ------------------------------------------------------------------ controls
function segmented(id, key, onPick) {
  $(id).addEventListener('click', (ev) => {
    const b = ev.target.closest('button');
    if (!b) return;
    for (const x of $(id).children) x.classList.toggle('on', x === b);
    onPick(b.dataset[key]);
  });
}

async function setView(view) {
  app.view = view;
  $('map2d').hidden = view !== '2d';
  $('map3d').hidden = view !== '3d';
  $('zscale-row').hidden = view !== '3d';
  $('hover-readout').hidden = view !== '2d';
  $('mode-tabs').parentElement.hidden = view !== '2d';
  $('mode-hint').textContent = view === '2d' ? MODE_HINTS[app.mode] : '3D에서는 끌어서 돌리고 휠로 확대합니다. 깃발과 장애물은 2D에서 놓습니다.';
  if (view === '3d' && !app.view3d) {
    try {
      const { View3D } = await import('./view3d.js');  // three.js is loaded only when the 3D tab is opened
      app.view3d = new View3D($('map3d'));
      app.view3d.setMap(app.map, app.learned, app.zScale);
    } catch (err) {
      console.error(err);
      log(`3D 화면을 열 수 없습니다: ${err.message}`, 'alarm');
      for (const x of $('view-tabs').children) x.classList.toggle('on', x.dataset.view === '2d');
      setView('2d');
    }
  }
}

function quickObstacle() {
  const s = app.state;
  const at = s.path.findIndex((c) => sameCell(c, s.pos));
  if (at < 0 || at + 5 >= s.path.length) { log('지금은 막을 경로가 없습니다. 순찰 중에 눌러 주세요.'); return; }
  const [r, c] = s.path[at + 4], [nr, nc] = s.path[at + 5];
  const side = nr !== r ? [[0, -1], [0, 1]] : [[-1, 0], [1, 0]];  // wall across the direction of travel
  const cells = [[r, c], ...side.map(([dr, dc]) => [r + dr, c + dc])]
    .filter(([a, b]) => a >= 0 && b >= 0 && a < app.map.n && b < app.map.n);
  log(`경로 앞 (${r}, ${c}) 에 장애물 ${cells.length}칸 추가`);
  for (const cell of cells) addObstacle(cell);
}

function quickBattery() {
  const s = app.state;
  if (s.return_wh === null) { log('여기서는 기지로 갈 길이 없습니다.'); return; }
  const wh = Math.max(s.return_wh * app.map.return_margin + 0.6, 1);
  app.world.setBattery(wh);
  log(`배터리를 ${wh.toFixed(1)} Wh로 낮춤 (복귀 기준 ${(s.return_wh * app.map.return_margin).toFixed(1)} Wh)`);
}

function quickBadInput() {
  log('잘못된 입력 4개 전송: "go", "add 999 999", "remove P9", 지도 밖 장애물 (500, 500)');
  for (const text of ['go', 'add 999 999', 'remove P9']) app.world.command(text);
  app.world.reportObstacle(500, 500);
  consume(app.world.state());
}

function setupControls() {
  segmented('view-tabs', 'view', setView);
  segmented('speed-tabs', 'speed', (v) => { app.speed = Number(v); });
  segmented('mode-tabs', 'mode', (mode) => {
    app.mode = mode;
    $('map2d').classList.toggle('drawing', mode === 'obstacle');  // only drawing obstacles takes over touch drags
    $('mode-hint').textContent = MODE_HINTS[mode];
    if (mode !== 'plan') { app.plan = { a: null, b: null, path: null }; $('plan-readout').hidden = true; }
  });
  $('mode-hint').textContent = MODE_HINTS[app.mode];

  $('btn-start').addEventListener('click', () => { app.world.command('start'); consume(app.world.state()); });
  $('btn-stop').addEventListener('click', () => { app.world.command('stop'); consume(app.world.state()); });
  $('btn-reset').addEventListener('click', () => buildWorld());
  $('btn-clear-log').addEventListener('click', clearLog);
  $('q-obstacle').addEventListener('click', quickObstacle);
  $('q-battery').addEventListener('click', quickBattery);
  $('q-bad').addEventListener('click', quickBadInput);

  const batt = $('in-batt');
  batt.addEventListener('pointerdown', () => { app.draggingBattery = true; });
  batt.addEventListener('input', () => { app.world.setBattery(Number(batt.value)); });
  batt.addEventListener('change', () => {
    app.draggingBattery = false;
    log(`배터리를 ${Number(batt.value).toFixed(0)} Wh로 설정`);
  });

  const slope = $('in-slope');
  slope.addEventListener('input', () => { $('out-slope').textContent = `${slope.value}°`; });
  slope.addEventListener('change', () => {
    app.maxSlope = Number(slope.value);
    buildWorld({ keep: true });
    log(`등판 한계를 ${app.maxSlope}°로 바꿈: 지도를 다시 만들고 기지에서 출발`);
  });

  $('in-z').addEventListener('input', (ev) => {
    app.zScale = Number(ev.target.value);
    $('out-z').textContent = `${app.zScale}×`;
    app.view3d?.setZScale(app.zScale);
  });
  $('in-energy').addEventListener('change', (ev) => {
    app.energyOn = ev.target.checked;
    refreshEnergy();
  });
}

// ------------------------------------------------------------------ start
function fail(err) {
  console.error(err);
  $('loading').hidden = true;
  $('fallback').hidden = false;
  $('fallback-reason').textContent = String(err?.message ?? err);
}

async function main() {
  clearLog();
  if (typeof WebAssembly !== 'object') throw new Error('WebAssembly를 지원하지 않는 브라우저입니다.');
  app.ugv = await loadUgv();
  setupControls();
  setupMapInput();
  await loadMap('campus');  // the page shows the campus map only; tests also use the other exported maps
  $('loading').hidden = true;
  requestAnimationFrame((ts) => { lastTs = ts; frame(ts); });
}

main().catch(fail);
