#!/usr/bin/env python3
"""Scenario tests S1~S5 on the real ROS 2 system (report 2, section 4.4).

Starts the launch file for each scenario (no GUI, faster drive), drives it through the
operator command topic, watches the topics, and prints pass/fail per scenario.
Runs on its own ROS_DOMAIN_ID so it does not disturb a demo that is already running.

    source ~/ros2_ws/install/setup.bash
    python3 scripts/scenario_test.py                 # Python controller
    python3 scripts/scenario_test.py --cpp           # C++ (rclcpp) controller
    python3 scripts/scenario_test.py --cpp s5        # only S5
"""
import json
import os
import signal
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = os.environ.get('SCENARIO_DOMAIN_ID', '77')

import rclpy  # noqa: E402  (must come after ROS_DOMAIN_ID is set)
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from geometry_msgs.msg import Point, Pose, PoseArray  # noqa: E402
from nav_msgs.msg import Path  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import Float32, String  # noqa: E402

SHARE = get_package_share_directory('ugv_patrol_planner')
POINTS = [(15, 85), (85, 85), (85, 15)]
BASE = (0, 0)
LATCHED = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
DRIVE_RATE = 20.0  # cells per second during tests
CONTROLLER = 'cpp' if '--cpp' in sys.argv else 'py'


def cell(p) -> tuple:
    return int(round(p.x)), int(round(p.y))


def stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Probe(Node):
    """Plays the operator and records everything the system publishes."""

    def __init__(self):
        super().__init__('scenario_probe')
        self.pub_cmd = self.create_publisher(String, '/operator/command', 10)
        self.pub_obs = self.create_publisher(PoseArray, '/perception/obstacles', 10)
        self.statuses, self.alarms, self.paths, self.poses, self.battery = [], [], [], [], []
        self.obstacle_msgs = []  # (time, set of cells)
        self.create_subscription(String, '/mission/status', self._status, LATCHED)
        self.create_subscription(String, '/mission/alarm', lambda m: self.alarms.append(m.data), 10)
        self.create_subscription(Path, '/controller/path', self._path, LATCHED)
        self.create_subscription(Point, '/ugv/pose', lambda m: self.poses.append(cell(m)), 10)
        self.create_subscription(Float32, '/battery/level', lambda m: self.battery.append(m.data), 10)
        self.create_subscription(PoseArray, '/perception/obstacles', self._obstacles, 10)

    def _status(self, m):
        self.statuses.append((time.monotonic(), json.loads(m.data)))

    # Paths and obstacles are compared by their send time (header stamp), not by when this
    # probe happened to process them, which can lag behind at high message rates.
    def _path(self, m):
        self.paths.append((stamp(m), [cell(p.pose.position) for p in m.poses]))

    def _obstacles(self, m):
        self.obstacle_msgs.append((stamp(m), {cell(p.position) for p in m.poses}))

    def spin_until(self, secs, done=lambda: False) -> bool:
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if done():
                return True
        return False

    def wait_ready(self) -> bool:
        """Controller is listening and the simulators already publish pose and battery.
        (A fast-starting controller must not get 'start' before it knows where the UGV is.)"""
        ok = self.spin_until(30, lambda: self.pub_cmd.get_subscription_count() > 0 and self.statuses
                             and self.poses and self.battery)
        self.spin_until(0.5)  # let the controller receive the same pose/battery messages
        return ok

    def send(self, text):
        self.pub_cmd.publish(String(data=text))
        self.spin_until(0.3)

    def state(self):
        return self.statuses[-1][1]['state'] if self.statuses else None

    def visit_counts(self) -> dict:
        counts, last = {}, {}
        for _, s in self.statuses:
            for p in s.get('points', []):
                v = p['last_visit']
                if v is not None and last.get(p['pid']) != v:
                    counts[p['pid']] = counts.get(p['pid'], 0) + 1
                last[p['pid']] = v
        return counts


class System:
    """The launched nodes for one scenario."""

    def __init__(self, scenario):
        path = os.path.join(SHARE, 'scenarios', f'{scenario}.yaml')
        self.proc = subprocess.Popen(
            ['ros2', 'launch', 'ugv_patrol_planner', 'patrol.launch.py', 'gui:=false',
             f'drive_rate:={DRIVE_RATE}', f'scenario:={path}', f'config:={config_path(scenario)}',
             f'controller:={CONTROLLER}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)

    def stop(self):
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
            self.proc.wait(timeout=10)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            os.killpg(self.proc.pid, signal.SIGKILL)


def config_path(scenario) -> str:
    """config/<scenario>.yaml when it exists (e.g. campus), otherwise config/params.yaml."""
    path = os.path.join(SHARE, 'config', f'{scenario}.yaml')
    return path if os.path.exists(path) else os.path.join(SHARE, 'config', 'params.yaml')


def run(scenario, body):
    sysm = System(scenario)
    probe = Probe()
    try:
        if not probe.wait_ready():
            return {'error': 'system did not start'}
        return body(probe)
    finally:
        probe.destroy_node()
        sysm.stop()


def start_patrol(p: Probe):
    for r, c in POINTS:
        p.send(f'add {r} {c}')
    p.send('start')


def s1_s2(p: Probe):
    """Default scenario: normal patrol (S1) with hidden obstacles found on the way (S2)."""
    start_patrol(p)
    p.spin_until(90, lambda: sum(p.visit_counts().values()) >= 6)
    counts = p.visit_counts()
    s1_ok = len(counts) == 3 and max(counts.values()) - min(counts.values()) <= 1
    s1 = (s1_ok, f'방문 횟수 {dict(sorted(counts.items()))}')

    # S2: re-planning happened, was fast, and no path or position used a known obstacle.
    replans = [t for t, s in p.statuses if s['state'] == '재계획 중']
    first_seen = {}  # obstacle cell -> send time of the first message that reported it
    for to, cells in sorted(p.obstacle_msgs):
        for c in cells:
            first_seen.setdefault(c, to)
    latencies, prev_tp = [], 0.0
    for tp, _ in sorted(p.paths):
        # Latency of a re-plan: new path time minus the newest obstacle report it reacted to.
        triggers = [t for t in first_seen.values() if prev_tp < t <= tp]
        if triggers and tp - max(triggers) < 2.0:
            latencies.append(tp - max(triggers))
        prev_tp = tp
    # A path may still contain an obstacle reported while the controller was busy with the previous
    # report, but never one reported more than 0.5 s earlier (report 2: avoid it within 0.5 s).
    worst_stale, example = 0.0, ''
    for tp, path in p.paths:
        for c in path:
            if c in first_seen and first_seen[c] < tp:
                age = tp - first_seen[c]
                if age > worst_stale:
                    worst_stale, example = age, f' (경로 {path[0]}->{path[-1]}, 칸 {c})'
    seen = set().union(*[cells for _, cells in p.obstacle_msgs] or [set()])
    on_obstacle = any(pos in seen for pos in p.poses)
    worst = max(latencies) if latencies else float('nan')
    s2_ok = bool(replans) and bool(latencies) and worst < 0.5 and worst_stale < 0.5 and not on_obstacle
    s2 = (s2_ok, f'재계획 {len(replans)}회, 최대 재계획 지연 {worst * 1000:.0f} ms, '
                 f'알려진 장애물이 경로에 남은 최대 시간 {worst_stale * 1000:.0f} ms{example}, '
                 f'장애물 칸 진입 {"있음" if on_obstacle else "없음"}')
    return {'S1 정상 순찰': s1, 'S2 장애물 발견': s2}


def s3(p: Probe):
    """Low battery: must return to base before running out (NFR-03)."""
    start_patrol(p)
    done = p.spin_until(90, lambda: any(s.get('note') == '복귀 완료' for _, s in p.statuses))
    went_home = any(s['state'] == '복귀 중' for _, s in p.statuses)
    low = min(p.battery) if p.battery else float('nan')
    at_base = bool(p.poses) and p.poses[-1] == BASE
    ok = done and went_home and at_base and low > 0
    return {'S3 배터리 부족': (ok, f'복귀 완료 {"예" if done else "아니오"}, 기지 도착 {"예" if at_base else "아니오"}, '
                               f'최저 잔량 {low:.2f} Wh')}


def s4(p: Probe):
    """Invalid input: alarms, no crash, and the controller still works afterwards (NFR-04)."""
    for text in ['add 999 999', 'remove P9', 'go', 'start']:
        p.send(text)
    bad = PoseArray()
    bad.poses = [Pose(position=Point(x=500.0, y=500.0))]
    p.pub_obs.publish(bad)
    p.spin_until(2)
    alarms = len(p.alarms)
    p.send('add 15 85')
    p.send('start')
    alive = p.spin_until(10, lambda: p.state() == '순찰 중')
    ok = alarms >= 5 and alive
    return {'S4 비정상 입력': (ok, f'경보 {alarms}개 (기대 5개 이상), 이후 정상 순찰 시작 {"예" if alive else "아니오"}')}


def s5(p: Probe):
    """Campus map: random flags are reproducible, all visited, no obstacle cell entered (FR-20~22),
    and no move the UGV made was steeper than the campus climbing limit (FR-03, 15 deg in v1.3)."""
    import yaml
    from ugv_patrol_planner.common import load_config
    from ugv_patrol_planner.mission.flags import scenario_points
    from ugv_patrol_planner.terrain.terrain_map import TerrainMap

    cfg = load_config(config_path('campus'))
    terrain = TerrainMap.from_file(os.path.join(SHARE, cfg.dem_file), cfg, os.path.join(SHARE, cfg.obstacle_file))
    with open(os.path.join(SHARE, 'scenarios', 'campus.yaml'), encoding='utf-8') as f:
        scenario = yaml.safe_load(f)
    flags = scenario_points(terrain, cfg.base_cell, scenario)
    same = flags == scenario_points(terrain, cfg.base_cell, scenario)
    for r, c in flags:
        p.send(f'add {r} {c}')
    p.send('start')
    p.spin_until(120, lambda: len(p.visit_counts()) == len(flags))
    counts = p.visit_counts()
    entered = sorted({c for c in p.poses if terrain.is_obstacle(c)})
    # one-cell moves only (a pose message the probe missed would join two cells that are not neighbours)
    moves = [(a, b) for a, b in zip(p.poses, p.poses[1:]) if abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1]
    steepest = max((terrain.slope_deg(a, b) for a, b in moves), default=0.0)
    ok = same and len(counts) == len(flags) and not entered and steepest <= cfg.max_slope_deg + 1e-9
    return {'S5 캠퍼스 순찰': (ok, f'깃발 {flags} (같은 시드 재현 {"예" if same else "아니오"}), '
                               f'방문 {dict(sorted(counts.items()))}, 장애물 칸 진입 {len(entered)}개, '
                               f'이동 {len(moves)}회 중 최대 경사 {steepest:.1f}° (한계 {cfg.max_slope_deg:g}°)')}


def main():
    rclpy.init()
    results = {}
    try:
        plan = [('default', s1_s2), ('low_battery', s3), ('default', s4), ('campus', s5)]
        only = [a for a in sys.argv[1:] if not a.startswith('--')]  # e.g. `scenario_test.py s1_s2`
        print(f'controller: {CONTROLLER}', flush=True)
        for scenario, body in [x for x in plan if not only or x[1].__name__ in only]:
            print(f'running {body.__doc__.splitlines()[0]}', flush=True)
            out = run(scenario, body)
            if 'error' in out:
                results[f'{body.__name__}'] = (False, out['error'])
            else:
                results.update(out)
    finally:
        rclpy.shutdown()
    print()
    for name, (ok, detail) in results.items():
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    passed = sum(ok for ok, _ in results.values())
    print(f'\n{passed}/{len(results)} scenarios passed')
    sys.exit(0 if passed == len(results) else 1)


if __name__ == '__main__':
    main()
