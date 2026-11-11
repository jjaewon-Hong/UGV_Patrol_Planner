"""U4 EnergyMonitor: return-to-base energy map and return decision (L3)."""
import heapq
import math
from typing import Optional

import numpy as np

from ugv_patrol_planner.common import Cell, Config, Path
from ugv_patrol_planner.terrain.terrain_map import MOVES, TerrainMap


def energy_step(terrain: TerrainMap, a: Cell, b: Cell, wh_per_m: float, wh_per_climb_m: float) -> float:
    """Energy [Wh] to move one cell: distance part + climbing part. Shared with battery_sim (FR-19)."""
    climb = max(0.0, terrain.elevation(b) - terrain.elevation(a))
    return wh_per_m * terrain.cell_size + wh_per_climb_m * climb


class EnergyMonitor:
    def __init__(self, terrain: TerrainMap, cfg: Config):
        if not terrain.in_bounds(cfg.base_cell):
            raise ValueError(f'base cell {cfg.base_cell} is off the map')
        self._terrain = terrain
        self.base: Cell = tuple(cfg.base_cell)
        self.wh_per_m = cfg.wh_per_m
        self.wh_per_climb_m = cfg.wh_per_climb_m
        self.margin = cfg.return_margin
        self._ret = np.full((terrain.n, terrain.n), math.inf)
        self.update_return_map()

    def energy_step(self, a: Cell, b: Cell) -> float:
        return energy_step(self._terrain, a, b, self.wh_per_m, self.wh_per_climb_m)

    def update_return_map(self) -> None:
        """R[c] = minimum energy from c back to base. Reverse Dijkstra from the base, so the
        uphill/downhill asymmetry is handled. Call at start and when obstacles change (FR-13)."""
        t = self._terrain
        R = np.full((t.n, t.n), math.inf)
        R[self.base] = 0.0
        heap = [(0.0, self.base)]
        while heap:
            d, u = heapq.heappop(heap)
            if d > R[u]:
                continue
            for dr, dc in MOVES:
                v = (u[0] + dr, u[1] + dc)
                if not t.in_bounds(v) or not t.is_traversable(v, u):  # need a v -> u move
                    continue
                nd = d + self.energy_step(v, u)
                if nd < R[v]:
                    R[v] = nd
                    heapq.heappush(heap, (nd, v))
        self._ret = R

    def return_energy(self, c: Cell) -> float:
        return float(self._ret[c])

    def should_return(self, pos: Cell, battery_wh: float, next_cell: Optional[Cell] = None) -> bool:
        """True if the UGV must head back now (FR-14). With next_cell, also checks the state
        after that move, so the margin still holds after moving (NFR-03)."""
        if battery_wh <= self.margin * self._ret[pos]:
            return True
        if next_cell is not None:
            predicted = battery_wh - self.energy_step(pos, next_cell)
            if predicted <= self.margin * self._ret[next_cell]:
                return True
        return False

    def return_path(self, pos: Cell) -> Optional[Path]:
        """Minimum-energy path from pos to base, following the return energy map."""
        if self._ret[pos] == math.inf:
            return None
        path = [pos]
        c = pos
        for _ in range(self._terrain.n * self._terrain.n):
            if c == self.base:
                return path
            best, best_e = None, math.inf
            for nb, _ in self._terrain.neighbors(c):
                e = self.energy_step(c, nb) + self._ret[nb]
                if e < best_e:
                    best, best_e = nb, e
            if best is None:
                return None
            c = best
            path.append(c)
        return None
