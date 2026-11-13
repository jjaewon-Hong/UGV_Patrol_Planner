// WebAssembly build check, run in Node (scripts/test_web.sh): the module must give the same
// results as the native build (cpp/test/test_sim.cpp) and as the ROS 2 scenario test S5.
//   node web/test/node_test.mjs ["<line printed by native ugv_plan for 42 30 64 21 on the campus map>"]
import { readFileSync } from 'node:fs';
import { loadUgv, prepareMap } from '../js/wasm.js';

const mapFile = (id) => new URL(`../maps/${id}.json`, import.meta.url);
const loadMap = (id) => prepareMap(JSON.parse(readFileSync(mapFile(id), 'utf8')));

let failed = 0;
function check(name, ok, detail = '') {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? ': ' + detail : ''}`);
  if (!ok) failed++;
}
function runUntil(world, maxTicks, done) {
  let s = world.state();
  for (let i = 0; i < maxTicks && !done(s); i++) {
    world.tick(1);
    s = world.state();
  }
  return s;
}

const ugv = await loadUgv();

// S5 campus patrol: same numbers as ROS 2 (319 moves, steepest 13.5 deg)
{
  const map = loadMap('campus');
  const world = ugv.createWorld(map);
  for (const [r, c] of map.points) world.command(`add ${r} ${c}`);
  world.command('start');
  const s = runUntil(world, 5000, (st) => Object.keys(st.visits).length === 3);
  check('S5 캠퍼스 순찰', Object.keys(s.visits).length === 3 && s.moves === 319 &&
    Math.abs(s.max_slope - 13.5) < 0.05 && s.obstacle_entries === 0,
    `방문 ${JSON.stringify(s.visits)}, 이동 ${s.moves}회, 최대 경사 ${s.max_slope.toFixed(1)}°, 장애물 진입 ${s.obstacle_entries}`);

  // A* alone: the same path and cost as the native ugv_plan tool (which uses the default 30 deg limit)
  const native = process.argv[2];
  const plan = (native ? ugv.createWorld(map, { maxSlopeDeg: 30 }) : world).plan([42, 30], [64, 21]);
  if (native) {
    const [cost, , cells] = native.trim().split(' ');
    const mine = plan.path.map(([r, c]) => `${r},${c}`).join(';');
    check('A* 경로가 네이티브와 같음', mine === cells && Math.abs(plan.cost - Number(cost)) < 1e-6,
      `${plan.cells}칸, 비용 ${plan.cost.toFixed(3)} (네이티브 ${Number(cost).toFixed(3)}), ${plan.plan_ms.toFixed(3)} ms`);
  } else {
    check('A* 경로 탐색', plan.found && plan.cells > 0, `${plan.cells}칸, ${plan.plan_ms.toFixed(3)} ms`);
  }
  check('지도 밖 경로 탐색은 없음으로', ugv.createWorld(map).plan([42, 30], [999, 0]).found === false);
}

// S1 + S2 default map: even visits, hidden obstacles found and avoided
{
  const map = loadMap('default');
  const world = ugv.createWorld(map);
  for (const [r, c] of map.points) world.command(`add ${r} ${c}`);
  world.command('start');
  const total = (st) => Object.values(st.visits).reduce((a, b) => a + b, 0);
  const s = runUntil(world, 5000, (st) => total(st) >= 6);
  const v = Object.values(s.visits);
  check('S1 정상 순찰', v.length === 3 && Math.max(...v) - Math.min(...v) <= 1, `방문 ${JSON.stringify(s.visits)}`);
  check('S2 장애물 발견', s.replans > 0 && s.obstacle_entries === 0, `재계획 ${s.replans}회, 장애물 진입 ${s.obstacle_entries}`);
}

// S3 low battery: returns to the base with energy left
{
  const map = loadMap('default');
  const world = ugv.createWorld(map, { batteryWh: 30 });
  for (const [r, c] of map.points) world.command(`add ${r} ${c}`);
  world.command('start');
  const s = runUntil(world, 5000, (st) => st.status.note === '복귀 완료');
  check('S3 배터리 부족', s.status.note === '복귀 완료' && s.pos[0] === 0 && s.pos[1] === 0 && s.min_battery > 0,
    `기지 도착 ${s.pos}, 최저 잔량 ${s.min_battery.toFixed(2)} Wh`);
}

// S4 invalid input: alarms, still working
{
  const world = ugv.createWorld(loadMap('default'));
  for (const text of ['add 999 999', 'remove P9', 'go', 'start']) world.command(text);
  world.reportObstacle(500, 500);
  const alarms = world.state().alarms.length;
  world.command('add 15 85');
  world.command('start');
  check('S4 비정상 입력', alarms >= 5 && world.state().status.state === '순찰 중' && world.tick(1) === 1, `경보 ${alarms}개`);
}

// world options and errors
{
  const map = loadMap('campus');
  const world = ugv.createWorld(map, { maxSlopeDeg: 30, extraObstacles: [[42, 31]] });
  check('장애물 추가 옵션', world.traversable([42, 30], [42, 31]) === false && world.traversable([42, 30], [42, 29]) === true);
  const e = world.returnEnergy();
  check('복귀 에너지 지도', e.length === 10000 && e[42 * 100 + 30] === 0 && e.some((x) => x > 0));
  let message = '';
  try { ugv.createWorld({ ...map, base: [-1, 0] }); } catch (err) { message = err.message; }
  check('잘못된 설정은 오류로', message.length > 0, message);
}

console.log(failed ? `\n${failed} check(s) failed` : '\nall WebAssembly checks passed');
process.exit(failed ? 1 : 0);
