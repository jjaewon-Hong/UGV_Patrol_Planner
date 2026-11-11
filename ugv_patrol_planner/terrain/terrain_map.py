"""U1 TerrainMap: grid elevation/obstacle map, traversability and move cost (L1).

Move costs for the 4 directions are precomputed once (and updated when obstacles are added),
so path planning only looks them up. This keeps re-planning well under 0.5 s (NFR-01).
"""
import math
from typing import Iterable

import numpy as np

from ugv_patrol_planner.common import Cell, Config

MOVES = ((-1, 0), (1, 0), (0, -1), (0, 1))  # 4-connected: up, down, left, right
_EPS = 1e-9  # so that exactly max_slope_deg counts as passable despite float rounding


class TerrainMap:
    def __init__(self, elev, cfg: Config, obstacles: Iterable[Cell] = ()):
        elev = np.asarray(elev, dtype=np.float64)
        if elev.ndim != 2 or elev.shape[0] != elev.shape[1]:
            raise ValueError(f'elevation must be N x N, got {elev.shape}')
        if not np.isfinite(elev).all():
            raise ValueError('elevation contains NaN or inf')
        self._elev = elev
        self.n = elev.shape[0]
        self._obstacle = np.zeros(elev.shape, dtype=bool)
        self.cell_size = cfg.cell_size_m
        self.max_slope_deg = cfg.max_slope_deg
        self.slope_weight = cfg.slope_weight
        self._cost = self._build_costs()  # _cost[d][row][col]: cost of moving from (row, col) in direction d
        self.add_obstacles(obstacles)

    @classmethod
    def from_file(cls, path: str, cfg: Config) -> 'TerrainMap':
        """Load an N x N elevation array (.npy) and register static obstacles (FR-01, FR-02)."""
        elev = np.load(path)
        if elev.shape != (cfg.map_size, cfg.map_size):
            raise ValueError(f'expected {cfg.map_size} x {cfg.map_size} map, got {elev.shape}')
        return cls(elev, cfg, cfg.static_obstacles)

    def _build_costs(self) -> list:
        n, e = self.n, self._elev
        costs = []
        for dr, dc in MOVES:
            cost = np.full((n, n), math.inf)
            # Source cells whose neighbour in this direction is on the map.
            src = (slice(max(-dr, 0), n - max(dr, 0)), slice(max(-dc, 0), n - max(dc, 0)))
            dst = (slice(max(dr, 0), n - max(-dr, 0)), slice(max(dc, 0), n - max(-dc, 0)))
            slope = np.degrees(np.arctan(np.abs(e[dst] - e[src]) / self.cell_size))
            ok = slope <= self.max_slope_deg + _EPS
            cost[src] = np.where(ok, self.cell_size * (1 + self.slope_weight * slope / self.max_slope_deg), math.inf)
            costs.append(cost.tolist())  # nested lists: fast scalar lookups in the search loop
        return costs

    def in_bounds(self, c: Cell) -> bool:
        return 0 <= c[0] < self.n and 0 <= c[1] < self.n

    def elevation(self, c: Cell) -> float:
        return float(self._elev[c])

    def is_obstacle(self, c: Cell) -> bool:
        return bool(self._obstacle[c])

    def slope_deg(self, a: Cell, b: Cell) -> float:
        return math.degrees(math.atan(abs(self._elev[b] - self._elev[a]) / self.cell_size))

    def move_cost(self, a: Cell, b: Cell) -> float:
        """cell_size * (1 + w * slope / max_slope); inf if not traversable (FR-04)."""
        d = MOVES.index((b[0] - a[0], b[1] - a[1])) if self.in_bounds(a) else None
        if d is None or not self.in_bounds(b):
            return math.inf
        return self._cost[d][a[0]][a[1]]

    def is_traversable(self, a: Cell, b: Cell) -> bool:
        """True if b is on the map, not an obstacle, and the slope is within the limit (FR-03)."""
        if (b[0] - a[0], b[1] - a[1]) not in MOVES:
            return False
        return self.move_cost(a, b) != math.inf

    def neighbors(self, c: Cell) -> list[tuple[Cell, float]]:
        """Traversable 4-neighbours of c with their move costs."""
        r, col = c
        out = []
        for d, (dr, dc) in enumerate(MOVES):
            cost = self._cost[d][r][col]
            if cost != math.inf:
                out.append(((r + dr, col + dc), cost))
        return out

    def add_obstacles(self, cells: Iterable[Cell]) -> list[Cell]:
        """Register new in-bounds obstacles and return the newly added ones (FR-11).
        Out-of-range cells are ignored (NFR-04)."""
        added = []
        for c in cells:
            c = (int(c[0]), int(c[1]))
            if self.in_bounds(c) and not self._obstacle[c]:
                self._obstacle[c] = True
                added.append(c)
                for d, (dr, dc) in enumerate(MOVES):  # block every move into c
                    a = (c[0] - dr, c[1] - dc)
                    if self.in_bounds(a):
                        self._cost[d][a[0]][a[1]] = math.inf
        return added
