"""U1 TerrainMap unit tests (UT-U1-01 ~ 04)."""
import math

import numpy as np
import pytest

from ugv_patrol_planner.common import Config
from ugv_patrol_planner.terrain.terrain_map import TerrainMap

CFG = Config(map_size=100)


def slope_map(deg: float) -> TerrainMap:
    """3 x 3 flat map where moving (0,0) -> (0,1) climbs at `deg` degrees."""
    elev = np.zeros((3, 3))
    elev[0, 1] = CFG.cell_size_m * math.tan(math.radians(deg))
    return TerrainMap(elev, CFG)


def test_ut_u1_01_load_map(tmp_path):
    elev = np.random.default_rng(0).uniform(0, 50, (100, 100))
    path = tmp_path / 'dem.npy'
    np.save(path, elev)
    t = TerrainMap.from_file(str(path), CFG)
    assert t.n == 100
    assert t.elevation((12, 34)) == pytest.approx(elev[12, 34])

    bad = tmp_path / 'bad.npy'
    np.save(bad, np.zeros((99, 100)))
    with pytest.raises(ValueError):
        TerrainMap.from_file(str(bad), CFG)


@pytest.mark.parametrize('deg, ok', [(29.9, True), (30.0, True), (30.1, False)])
def test_ut_u1_02_slope_boundary(deg, ok):
    assert slope_map(deg).is_traversable((0, 0), (0, 1)) is ok


def test_ut_u1_03_cost_grows_with_slope():
    c0 = slope_map(0).move_cost((0, 0), (0, 1))
    c10 = slope_map(10).move_cost((0, 0), (0, 1))
    c20 = slope_map(20).move_cost((0, 0), (0, 1))
    assert c0 == pytest.approx(CFG.cell_size_m)
    assert c0 < c10 < c20
    assert slope_map(30.1).move_cost((0, 0), (0, 1)) == math.inf


def test_ut_u1_04_obstacles_and_out_of_range():
    cfg = Config(map_size=5, static_obstacles=((2, 2),))
    t = TerrainMap(np.zeros((5, 5)), cfg, cfg.static_obstacles)
    assert not t.is_traversable((2, 1), (2, 2))            # static obstacle (FR-02)
    assert t.add_obstacles([(1, 1), (1, 1)]) == [(1, 1)]   # added once (FR-11)
    assert t.move_cost((0, 1), (1, 1)) == math.inf
    assert t.add_obstacles([(-1, 0), (100, 5)]) == []      # ignored, no exception (NFR-04)
    assert {nb for nb, _ in t.neighbors((1, 2))} == {(0, 2), (1, 3)}
