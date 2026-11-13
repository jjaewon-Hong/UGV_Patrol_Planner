"""Campus climbing limit (FR-03, requirements v1.3): 15 deg, and it actually blocks moves on the campus map."""
import os

import pytest

from ugv_patrol_planner.common import load_config
from ugv_patrol_planner.planning.astar import AStarPlanner
from ugv_patrol_planner.terrain.terrain_map import TerrainMap

ROOT = os.path.join(os.path.dirname(__file__), '..')


def campus(name):
    cfg = load_config(os.path.join(ROOT, 'config', f'{name}.yaml'))
    return cfg, TerrainMap.from_file(os.path.join(ROOT, cfg.dem_file), cfg, os.path.join(ROOT, cfg.obstacle_file))


def free_moves(t):
    for r in range(t.n):
        for c in range(t.n):
            for b in ((r + 1, c), (r, c + 1)):
                if t.in_bounds(b) and not t.is_obstacle((r, c)) and not t.is_obstacle(b):
                    yield (r, c), b


@pytest.mark.parametrize('name', ['campus', 'campus_osm'])
def test_campus_limit_is_15_and_blocks_steep_moves(name):
    cfg, t = campus(name)
    assert cfg.max_slope_deg == 15.0
    steep = [(a, b) for a, b in free_moves(t) if t.slope_deg(a, b) > 15.0]
    assert steep                                            # with 30 deg nothing on campus would be blocked
    assert not any(t.is_traversable(a, b) or t.is_traversable(b, a) for a, b in steep)


def test_campus_paths_stay_under_the_limit():
    cfg, t = campus('campus_osm')
    path = AStarPlanner(t).plan((75, 3), (18, 95))              # a long leg that has to go round a slope
    assert path is not None
    assert max(t.slope_deg(a, b) for a, b in zip(path, path[1:])) <= 15.0


def test_default_map_keeps_30_degrees():
    assert load_config(os.path.join(ROOT, 'config', 'params.yaml')).max_slope_deg == 30.0
