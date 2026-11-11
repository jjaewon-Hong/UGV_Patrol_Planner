"""U2 AStarPlanner: minimum-cost path on TerrainMap with A* (L2)."""
import heapq
import itertools
import math
import time
from typing import Optional

from ugv_patrol_planner.common import Cell, Path
from ugv_patrol_planner.terrain.terrain_map import TerrainMap


class AStarPlanner:
    def __init__(self, terrain: TerrainMap):
        self._terrain = terrain
        self.last_plan_ms = 0.0

    def _h(self, c: Cell, goal: Cell) -> float:
        # Manhattan distance * cell_size. Every move costs at least cell_size,
        # so h never overestimates (admissible) and A* returns an optimal path.
        return (abs(c[0] - goal[0]) + abs(c[1] - goal[1])) * self._terrain.cell_size

    def plan(self, start: Cell, goal: Cell) -> Optional[Path]:
        """Minimum-cost path from start to goal, or None if unreachable (FR-05, FR-06)."""
        t0 = time.perf_counter()
        try:
            return self._search(start, goal)
        finally:
            self.last_plan_ms = (time.perf_counter() - t0) * 1000

    def _search(self, start: Cell, goal: Cell) -> Optional[Path]:
        t = self._terrain
        if not t.in_bounds(start) or not t.in_bounds(goal) or t.is_obstacle(goal):
            return None
        seq = itertools.count()
        g = {start: 0.0}
        came_from: dict[Cell, Cell] = {}
        closed: set[Cell] = set()
        h0 = self._h(start, goal)
        open_heap = [(h0, h0, next(seq), start)]
        while open_heap:
            _, _, _, cur = heapq.heappop(open_heap)
            if cur in closed:
                continue  # stale duplicate entry
            if cur == goal:
                return self._reconstruct(came_from, goal)
            closed.add(cur)
            for nb, cost in t.neighbors(cur):  # only traversable moves (NFR-02)
                if nb in closed:
                    continue
                ng = g[cur] + cost
                if ng < g.get(nb, math.inf):
                    g[nb] = ng
                    came_from[nb] = cur
                    h = self._h(nb, goal)
                    heapq.heappush(open_heap, (ng + h, h, next(seq), nb))
        return None

    @staticmethod
    def _reconstruct(came_from: dict[Cell, Cell], goal: Cell) -> Path:
        path = [goal]
        while path[-1] in came_from:
            path.append(came_from[path[-1]])
        path.reverse()
        return path

    def is_path_valid(self, path: Path, from_idx: int = 0) -> bool:
        """True if every move after from_idx is still traversable (FR-12)."""
        return all(self._terrain.is_traversable(path[i], path[i + 1])
                   for i in range(max(from_idx, 0), len(path) - 1))

    def path_cost(self, path: Path) -> float:
        return sum(self._terrain.move_cost(path[i], path[i + 1]) for i in range(len(path) - 1))
