"""U4 EnergyMonitor unit tests (UT-U4-01 ~ 05)."""
import math

import numpy as np
import pytest

from ugv_patrol_planner.common import Config
from ugv_patrol_planner.mission.energy import EnergyMonitor
from ugv_patrol_planner.terrain.terrain_map import TerrainMap

CFG = Config(base_cell=(0, 0))
STEP = CFG.wh_per_m * CFG.cell_size_m  # flat one-cell move


def monitor(elev) -> EnergyMonitor:
    return EnergyMonitor(TerrainMap(np.asarray(elev, dtype=float), CFG), CFG)


def test_ut_u4_01_flat_return_map():
    m = monitor(np.zeros((6, 6)))
    assert m.return_energy((0, 0)) == 0
    for c in [(0, 5), (3, 2), (5, 5)]:
        assert m.return_energy(c) == pytest.approx((c[0] + c[1]) * STEP)


def test_ut_u4_02_slope_return_path_matches_map():
    rng = np.random.default_rng(4)
    m = monitor(rng.uniform(0, 12, (12, 12)))
    for pos in [(11, 11), (5, 9), (8, 2)]:
        path = m.return_path(pos)
        assert path[0] == pos and path[-1] == (0, 0)
        used = sum(m.energy_step(a, b) for a, b in zip(path, path[1:]))
        assert used == pytest.approx(m.return_energy(pos))


def test_ut_u4_03_threshold_boundary():
    m = monitor(np.zeros((6, 6)))
    pos = (3, 3)
    limit = CFG.return_margin * m.return_energy(pos)
    assert m.should_return(pos, limit)
    assert not m.should_return(pos, limit + 0.01)


def test_ut_u4_04_look_ahead_before_moving_away():
    elev = np.zeros((3, 3))
    elev[0, 2] = 10.0  # moving (0,1) -> (0,2) climbs 10 m, away from base
    m = monitor(elev)
    pos, nxt = (0, 1), (0, 2)
    battery = CFG.return_margin * m.return_energy(pos) + 0.5
    assert not m.should_return(pos, battery)        # fine if it stays here
    assert m.should_return(pos, battery, nxt)       # but not after this move


def test_ut_u4_05_obstacles_update_return_map():
    t = TerrainMap(np.zeros((5, 5)), CFG)
    m = EnergyMonitor(t, CFG)
    assert m.return_energy((0, 2)) == pytest.approx(2 * STEP)
    t.add_obstacles([(0, 1), (3, 4), (4, 3)])
    m.update_return_map()
    assert m.return_energy((0, 2)) == pytest.approx(4 * STEP)  # detour around (0,1)
    assert m.return_energy((4, 4)) == math.inf                 # boxed in
    assert m.return_path((4, 4)) is None
    assert m.should_return((4, 4), 1000.0)
