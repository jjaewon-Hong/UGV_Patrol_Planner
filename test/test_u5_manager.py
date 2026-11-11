"""U5 MissionManager unit tests (UT-U5-01 ~ 08)."""
import numpy as np
import pytest

from ugv_patrol_planner.common import Config, MissionState as S
from ugv_patrol_planner.mission.energy import EnergyMonitor
from ugv_patrol_planner.mission.manager import (Alarm, Command, MissionManager, NextCell, PathUpdate,
                                                Status, parse_command)
from ugv_patrol_planner.mission.scheduler import PatrolScheduler
from ugv_patrol_planner.planning.astar import AStarPlanner
from ugv_patrol_planner.terrain.terrain_map import TerrainMap

CFG = Config(base_cell=(0, 0))


def world(n=10):
    t = TerrainMap(np.zeros((n, n)), CFG)
    e = EnergyMonitor(t, CFG)
    return MissionManager(t, AStarPlanner(t), PatrolScheduler(), e)


def ready(mm, points, pos=(0, 0), battery=500.0):
    for c in points:
        mm.on_command(Command('add', cell=c), 0)
    mm.on_pose(pos, 0)
    mm.on_battery(battery)


def of(outs, kind):
    return [o for o in outs if isinstance(o, kind)]


def states(outs):
    """State sequence from Status outputs, with repeats (e.g. new target, same state) collapsed."""
    seq = []
    for o in of(outs, Status):
        if not seq or seq[-1] != o.state:
            seq.append(o.state)
    return seq


def drive(mm, outs, now=1.0, battery=None, on_step=None, max_steps=500):
    """Simulate drive_sim (and battery_sim if battery is given): follow NextCell until none."""
    log = list(outs)
    for k in range(max_steps):
        nxt = of(outs, NextCell)
        if not nxt:
            return log, battery
        prev, cell = mm.pos, nxt[-1].cell
        outs = []
        if battery is not None:
            battery -= mm.energy.energy_step(prev, cell)
            outs += mm.on_battery(battery)
        outs += mm.on_pose(cell, now + k)
        if on_step:
            outs += on_step(k, mm)
        log += outs
    raise AssertionError('did not stop')


def test_ut_u5_01_start():
    mm = world()
    ready(mm, [(0, 5), (5, 5)])
    outs = mm.on_command(Command('start'), 1)
    assert mm.state == S.PATROLLING
    assert states(outs) == [S.PATROLLING]
    assert of(outs, PathUpdate)[0].path[-1] == (0, 5)
    assert of(outs, NextCell)[0].cell == (0, 1)


def test_ut_u5_02_stop():
    mm = world()
    ready(mm, [(0, 5)])
    mm.on_command(Command('start'), 1)
    outs = mm.on_command(Command('stop'), 2)
    assert mm.state == S.STOPPED and states(outs) == [S.STOPPED]
    assert mm.on_pose((0, 1), 3) == []


def test_ut_u5_03_arrival_updates_visit():
    mm = world()
    ready(mm, [(0, 3), (3, 3)])
    outs = mm.on_command(Command('start'), 1)
    for k, c in enumerate([(0, 1), (0, 2)]):
        outs = mm.on_pose(c, 2 + k)
    outs = mm.on_pose((0, 3), 10)
    assert mm.scheduler.get('P1').last_visit == 10
    assert mm.target.pid == 'P2'
    assert of(outs, PathUpdate)[0].path[-1] == (3, 3)
    status = of(outs, Status)[0]                      # new target reported while still patrolling
    assert status.info['target'] == 'P2'
    assert status.info['points'][0]['last_visit'] == 10


def test_ut_u5_04_obstacle_on_and_off_path():
    mm = world()
    ready(mm, [(0, 6)])
    mm.on_command(Command('start'), 1)
    mm.on_pose((0, 1), 2)
    assert mm.on_obstacles([(5, 5)], 3) == []            # not on the path: nothing happens
    outs = mm.on_obstacles([(0, 3)], 4)                   # on the path
    assert states(outs) == [S.REPLANNING, S.PATROLLING]
    new_path = of(outs, PathUpdate)[0].path
    assert (0, 3) not in new_path and new_path[0] == (0, 1) and new_path[-1] == (0, 6)


def test_ut_u5_05_unreachable_target():
    mm = world()
    ready(mm, [(5, 5), (8, 8)])
    mm.terrain.add_obstacles([(4, 5), (6, 5), (5, 4), (5, 6)])  # box in P1
    outs = mm.on_command(Command('start'), 1)
    assert [a.text for a in of(outs, Alarm)] == ['경로 없음: P1']
    assert mm.target.pid == 'P2' and mm.state == S.PATROLLING

    mm2 = world()
    ready(mm2, [(5, 5)])
    mm2.terrain.add_obstacles([(4, 5), (6, 5), (5, 4), (5, 6)])
    outs = mm2.on_command(Command('start'), 1)
    assert [a.text for a in of(outs, Alarm)] == ['경로 없음: P1', '경로 없음']
    assert mm2.state == S.IDLE


def test_ut_u5_06_low_battery_returns_home():
    mm = world()
    ready(mm, [(9, 9)], pos=(3, 3))
    mm.on_battery(CFG.return_margin * mm.energy.return_energy((3, 3)))
    outs = mm.on_command(Command('start'), 1)
    assert states(outs) == [S.PATROLLING, S.RETURNING]
    assert of(outs, PathUpdate)[-1].path[-1] == CFG.base_cell


def test_ut_u5_07_full_scenario_status_sequence():
    """Start -> obstacle on path -> low battery -> back at base (FR-15, NFR-03)."""
    mm = world()
    ready(mm, [(0, 9), (9, 9)], battery=6.0)
    outs = mm.on_command(Command('start'), 1)

    def obstacle_once(k, m):
        return m.on_obstacles([(0, 4)], 100 + k) if k == 1 else []

    log, battery = drive(mm, outs, battery=6.0, on_step=obstacle_once)
    assert states(log) == [S.PATROLLING, S.REPLANNING, S.PATROLLING, S.RETURNING, S.IDLE]
    assert mm.pos == CFG.base_cell
    assert battery > 0                                   # never ran out outside the base
    assert of(log, Status)[-1].info.get('note') == '복귀 완료'


def test_ut_u5_08_invalid_inputs():
    mm = world()
    assert of(mm.on_command(Command('add', cell=(50, 50)), 0), Alarm)
    assert of(mm.on_command(Command('remove', pid='P9'), 0), Alarm)
    assert of(mm.on_command(Command('start'), 0), Alarm)   # no points
    assert mm.state == S.IDLE
    assert parse_command('add 3 4') == Command('add', cell=(3, 4))
    assert parse_command(' start ') == Command('start')
    for bad in ['add 3 x', 'go', 'remove', '']:
        with pytest.raises(ValueError):
            parse_command(bad)
