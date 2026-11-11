"""U3 PatrolScheduler: patrol point table and longest-wait-first target selection (L3)."""
import math
from typing import Iterable, Optional

from ugv_patrol_planner.common import Cell, PatrolPoint


class PatrolScheduler:
    def __init__(self):
        self._points: dict[str, PatrolPoint] = {}  # keeps registration order
        self._next_id = 1

    def __len__(self) -> int:
        return len(self._points)

    def add_point(self, cell: Cell) -> str:
        """Register a point and return its ID. The same cell returns the existing ID (FR-07)."""
        for p in self._points.values():
            if p.cell == cell:
                return p.pid
        pid = f'P{self._next_id}'
        self._next_id += 1
        self._points[pid] = PatrolPoint(pid, cell)
        return pid

    def remove_point(self, pid: str) -> bool:
        return self._points.pop(pid, None) is not None

    def get(self, pid: str) -> Optional[PatrolPoint]:
        return self._points.get(pid)

    def select_next(self, now: float, exclude: Iterable[str] = ()) -> Optional[PatrolPoint]:
        """Point with the longest time since its last visit; never-visited points first,
        ties go to the earlier registered point (FR-09)."""
        exclude = set(exclude)
        best, best_wait = None, -math.inf
        for p in self._points.values():
            if p.pid in exclude:
                continue
            wait = math.inf if p.last_visit is None else now - p.last_visit
            if wait > best_wait:
                best, best_wait = p, wait
        return best

    def mark_visited(self, pid: str, now: float) -> None:
        """Update the visit time when the UGV reaches the point (FR-10)."""
        if pid in self._points:
            self._points[pid].last_visit = now

    def snapshot(self) -> list[PatrolPoint]:
        return [PatrolPoint(p.pid, p.cell, p.last_visit) for p in self._points.values()]
