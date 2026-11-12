"""FR-22 random flag placement and FR-20 obstacle grid loading."""
import numpy as np
import pytest

from ugv_patrol_planner.common import Config
from ugv_patrol_planner.mission.flags import place_flags, reachable_cells, scenario_points
from ugv_patrol_planner.terrain.terrain_map import TerrainMap

CFG = Config(map_size=20, cell_size_m=10.0)
BASE = (0, 0)


def walled_map():
    """Flat 20x20 map; a closed wall cuts off the bottom-right 5x5 corner."""
    wall = [(14, c) for c in range(14, 20)] + [(r, 14) for r in range(14, 20)]
    return TerrainMap(np.zeros((20, 20)), CFG, wall), wall


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def test_same_seed_same_flags_different_seed_different_flags():
    t, _ = walled_map()
    assert place_flags(t, BASE, 3, seed=1, min_spacing=5) == place_flags(t, BASE, 3, seed=1, min_spacing=5)
    assert place_flags(t, BASE, 3, seed=1, min_spacing=5) != place_flags(t, BASE, 3, seed=2, min_spacing=5)


@pytest.mark.parametrize('seed', range(20))
def test_flags_are_reachable_free_and_spaced(seed):
    t, wall = walled_map()
    flags = place_flags(t, BASE, 3, seed=seed, min_spacing=5)
    reach = set(reachable_cells(t, BASE))
    assert len(flags) == 3
    for f in flags:
        assert f in reach and not t.is_obstacle(f)
        assert not (f[0] > 14 and f[1] > 14)                 # never inside the walled-off corner
        assert manhattan(f, BASE) >= 5
    for i, a in enumerate(flags):
        for b in flags[i + 1:]:
            assert manhattan(a, b) >= 5


def test_too_many_flags_or_bad_base_is_rejected():
    t, wall = walled_map()
    with pytest.raises(ValueError):
        place_flags(t, BASE, 50, seed=0, min_spacing=10)
    with pytest.raises(ValueError):
        place_flags(t, wall[0], 3)


def test_scenario_points_fixed_or_random():
    t, _ = walled_map()
    assert scenario_points(t, BASE, {'patrol_points': [[3, 4], [5, 6]]}) == [(3, 4), (5, 6)]
    rf = {'random_flags': {'count': 2, 'seed': 3, 'min_spacing': 4}}
    assert scenario_points(t, BASE, rf) == place_flags(t, BASE, 2, 3, 4)


def test_obstacle_grid_file_is_loaded(tmp_path):
    cfg = Config(map_size=10, cell_size_m=10.0, static_obstacles=((0, 9),))
    np.save(tmp_path / 'dem.npy', np.zeros((10, 10)))
    grid = np.zeros((10, 10), bool)
    grid[4, 2:6] = True
    np.save(tmp_path / 'obs.npy', grid)
    t = TerrainMap.from_file(str(tmp_path / 'dem.npy'), cfg, str(tmp_path / 'obs.npy'))
    assert all(t.is_obstacle((4, c)) for c in range(2, 6))
    assert t.is_obstacle((0, 9)) and not t.is_obstacle((5, 5))
    np.save(tmp_path / 'bad.npy', np.zeros((9, 9), bool))
    with pytest.raises(ValueError):
        TerrainMap.from_file(str(tmp_path / 'dem.npy'), cfg, str(tmp_path / 'bad.npy'))
