"""Check that the C++ A* (cpp/) returns the same paths as the Python one, and compare speed.

    python3 tools/compare_cpp.py <path/to/ugv_plan> [pairs=200]

Runs both planners on the same random start/goal pairs for each map in data/ and prints
how many paths match exactly (same cells), the largest cost difference and the planning time.
"""
import os
import random
import subprocess
import sys

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, ROOT)
from ugv_patrol_planner.common import Config  # noqa: E402
from ugv_patrol_planner.planning.astar import AStarPlanner  # noqa: E402
from ugv_patrol_planner.terrain.terrain_map import TerrainMap  # noqa: E402

MAPS = [  # name, dem, obstacles, cell size
    ('synthetic 100x100', 'data/dem_100x100.npy', '-', 30.0),
    ('campus (model)', 'data/campus_pred/dem.npy', 'data/campus_pred/obstacles.npy', 10.0),
    ('campus (OSM)', 'data/campus_osm/dem.npy', 'data/campus_osm/obstacles.npy', 10.0),
]


def main(binary: str, pairs: int) -> int:
    failures = 0
    print(f'{"map":20s} {"pairs":>5s} {"same path":>9s} {"max |dcost|":>11s} {"py ms (mean/max)":>17s} {"C++ ms (mean/max)":>18s}')
    for name, dem, obs, cs in MAPS:
        dem_p = os.path.join(ROOT, dem)
        obs_p = '-' if obs == '-' else os.path.join(ROOT, obs)
        elev = np.load(dem_p)
        cfg = Config(map_size=elev.shape[0], cell_size_m=cs)
        terrain = TerrainMap.from_file(dem_p, cfg, '' if obs == '-' else obs_p)
        free = [(r, c) for r in range(terrain.n) for c in range(terrain.n) if not terrain.is_obstacle((r, c))]
        rng = random.Random(0)
        queries = [(rng.choice(free), rng.choice(free)) for _ in range(pairs)]

        planner = AStarPlanner(terrain)
        py = []
        for s, g in queries:
            path = planner.plan(s, g)
            py.append((path, planner.path_cost(path) if path else None, planner.last_plan_ms))

        text = ''.join(f'{s[0]} {s[1]} {g[0]} {g[1]}\n' for s, g in queries)
        out = subprocess.run([binary, dem_p, obs_p, str(cs)], input=text, capture_output=True, text=True, check=True)
        same, max_diff, cpp_ms = 0, 0.0, []
        for (path, cost, _), line in zip(py, out.stdout.splitlines()):
            parts = line.split()
            cpp_ms.append(float(parts[1]))
            if parts[0] == 'none':
                same += path is None
                continue
            cells = [tuple(int(v) for v in p.split(',')) for p in parts[2].split(';')]
            same += path == cells
            if cost is not None:
                max_diff = max(max_diff, abs(cost - float(parts[0])))
        py_ms = [m for _, _, m in py]
        print(f'{name:20s} {pairs:5d} {same:5d}/{pairs:<3d} {max_diff:11.2e} '
              f'{np.mean(py_ms):8.2f}/{max(py_ms):7.2f} {np.mean(cpp_ms):9.3f}/{max(cpp_ms):7.3f}')
        failures += (same != pairs) + (max_diff > 1e-6)
    return failures


if __name__ == '__main__':
    sys.exit(1 if main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 200) else 0)
