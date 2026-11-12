"""Random patrol flag placement (FR-22).

Flags go on free cells that the UGV can actually reach from the base, at least `min_spacing` cells
(Manhattan) from the base and from each other. The same seed always gives the same flags.

    python3 -m ugv_patrol_planner.mission.flags <config.yaml> <scenario.yaml>   # prints "row col" lines
"""
import random
import sys
from collections import deque

from ugv_patrol_planner.common import Cell
from ugv_patrol_planner.terrain.terrain_map import TerrainMap


def reachable_cells(terrain: TerrainMap, base: Cell) -> list[Cell]:
    """Cells reachable from base by traversable moves, in row-major order (independent of search order)."""
    seen = {tuple(base)}
    queue = deque([tuple(base)])
    while queue:
        c = queue.popleft()
        for nb, _ in terrain.neighbors(c):
            if nb not in seen:
                seen.add(nb)
                queue.append(nb)
    return sorted(seen)


def place_flags(terrain: TerrainMap, base: Cell, count: int = 3, seed: int = 0,
                min_spacing: int = 10) -> list[Cell]:
    base = tuple(base)
    if not terrain.in_bounds(base) or terrain.is_obstacle(base):
        raise ValueError(f'base {base} is off the map or on an obstacle')
    candidates = [c for c in reachable_cells(terrain, base) if c != base]
    rng = random.Random(seed)
    rng.shuffle(candidates)
    flags: list[Cell] = []
    for c in candidates:
        if all(abs(c[0] - o[0]) + abs(c[1] - o[1]) >= min_spacing for o in [base] + flags):
            flags.append(c)
            if len(flags) == count:
                return flags
    raise ValueError(f'only {len(flags)} of {count} flags fit with spacing {min_spacing}')


def scenario_points(terrain: TerrainMap, base: Cell, scenario: dict) -> list[Cell]:
    """Patrol points of a scenario: fixed `patrol_points`, or `random_flags: {count, seed, min_spacing}`."""
    rf = scenario.get('random_flags')
    if rf:
        return place_flags(terrain, base, int(rf.get('count', 3)), int(rf.get('seed', 0)),
                           int(rf.get('min_spacing', 10)))
    return [tuple(p) for p in scenario.get('patrol_points', [])]


def main(argv: list[str]) -> None:
    import os

    import yaml
    from ament_index_python.packages import get_package_share_directory

    from ugv_patrol_planner.common import load_config

    share = get_package_share_directory('ugv_patrol_planner')

    def resolve(p):
        return p if os.path.isabs(p) else os.path.join(share, p)

    cfg = load_config(resolve(argv[0]))
    terrain = TerrainMap.from_file(resolve(cfg.dem_file), cfg, resolve(cfg.obstacle_file) if cfg.obstacle_file else '')
    with open(resolve(argv[1]), encoding='utf-8') as f:
        scenario = yaml.safe_load(f) or {}
    for r, c in scenario_points(terrain, cfg.base_cell, scenario):
        print(r, c)


if __name__ == '__main__':
    main(sys.argv[1:])
