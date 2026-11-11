"""U2 AStarPlanner unit tests (UT-U2-01 ~ 05)."""
import heapq
import math

import numpy as np
import pytest

from ugv_patrol_planner.common import Config
from ugv_patrol_planner.planning.astar import AStarPlanner
from ugv_patrol_planner.terrain.terrain_map import TerrainMap

CFG = Config()


def random_map(rng, n=20, height=25.0, obstacle_ratio=0.15) -> TerrainMap:
    elev = rng.uniform(0, height, (n, n))
    t = TerrainMap(elev, CFG)
    cells = [tuple(c) for c in np.argwhere(rng.random((n, n)) < obstacle_ratio)]
    t.add_obstacles([c for c in cells if c not in ((0, 0), (n - 1, n - 1))])
    return t


def dijkstra_cost(t: TerrainMap, start, goal) -> float:
    dist = {start: 0.0}
    heap = [(0.0, start)]
    while heap:
        d, u = heapq.heappop(heap)
        if u == goal:
            return d
        if d > dist[u]:
            continue
        for v, c in t.neighbors(u):
            if d + c < dist.get(v, math.inf):
                dist[v] = d + c
                heapq.heappush(heap, (d + c, v))
    return math.inf


def test_ut_u2_01_flat_path():
    p = AStarPlanner(TerrainMap(np.zeros((10, 10)), CFG))
    path = p.plan((0, 0), (9, 9))
    assert len(path) == 19
    assert p.path_cost(path) == pytest.approx(18 * 30)
    assert p.plan((3, 3), (3, 3)) == [(3, 3)]


def test_ut_u2_02_matches_dijkstra():
    rng = np.random.default_rng(1)
    for _ in range(50):
        t = random_map(rng)
        p = AStarPlanner(t)
        path = p.plan((0, 0), (19, 19))
        expected = dijkstra_cost(t, (0, 0), (19, 19))
        if path is None:
            assert expected == math.inf
        else:
            assert p.path_cost(path) == pytest.approx(expected)


def test_ut_u2_03_unreachable_goal():
    t = TerrainMap(np.zeros((10, 10)), CFG)
    t.add_obstacles([(4, 5), (6, 5), (5, 4), (5, 6)])  # wall around (5, 5)
    p = AStarPlanner(t)
    assert p.plan((0, 0), (5, 5)) is None
    assert p.plan((0, 0), (4, 5)) is None      # goal is an obstacle
    assert p.plan((0, 0), (10, 0)) is None     # goal off the map


def test_ut_u2_04_paths_are_safe_and_validity_check():
    rng = np.random.default_rng(2)
    found = 0
    for _ in range(100):
        t = random_map(rng)
        p = AStarPlanner(t)
        path = p.plan((0, 0), (19, 19))
        if path is None:
            continue
        found += 1
        for a, b in zip(path, path[1:]):
            assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1
            assert not t.is_obstacle(b)
            assert t.slope_deg(a, b) <= CFG.max_slope_deg + 1e-9
        assert p.is_path_valid(path)
        if len(path) > 2:
            t.add_obstacles([path[len(path) // 2]])
            assert not p.is_path_valid(path)
    assert found > 20  # enough reachable maps to be meaningful


def test_ut_u2_05_replan_time():
    rng = np.random.default_rng(3)
    t = TerrainMap(rng.uniform(0, 10, (100, 100)), CFG)
    t.add_obstacles([(r, 50) for r in range(10, 100)])  # long wall forces a detour
    p = AStarPlanner(t)
    worst = 0.0
    for _ in range(20):
        assert p.plan((0, 0), (99, 99)) is not None
        worst = max(worst, p.last_plan_ms)
    assert worst < CFG.replan_time_limit_s * 1000
