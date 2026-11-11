"""U5 MissionManager: mission state machine (L3).

It never calls ROS 2 directly. Each event handler returns a list of outputs
(NextCell, PathUpdate, Status, Alarm) that the interface layer publishes as topics.
"""
from dataclasses import dataclass, field
from typing import Iterable, Optional, Union

from ugv_patrol_planner.common import Cell, MissionState, Path, PatrolPoint
from ugv_patrol_planner.mission.energy import EnergyMonitor
from ugv_patrol_planner.mission.scheduler import PatrolScheduler
from ugv_patrol_planner.planning.astar import AStarPlanner
from ugv_patrol_planner.terrain.terrain_map import TerrainMap

S = MissionState
RUNNING = (S.PATROLLING, S.REPLANNING, S.RETURNING)


@dataclass
class NextCell:
    cell: Cell


@dataclass
class PathUpdate:
    path: Path


@dataclass
class Status:
    state: MissionState
    info: dict = field(default_factory=dict)


@dataclass
class Alarm:
    text: str


Output = Union[NextCell, PathUpdate, Status, Alarm]


@dataclass
class Command:
    kind: str                      # 'start' | 'stop' | 'add' | 'remove'
    cell: Optional[Cell] = None    # for 'add'
    pid: Optional[str] = None      # for 'remove'


def parse_command(text: str) -> Command:
    """Parse '/operator/command' text: start | stop | add <row> <col> | remove <id>.
    Raises ValueError on a malformed command (the interface layer turns it into an alarm)."""
    parts = text.strip().split()
    if parts in (['start'], ['stop']):
        return Command(parts[0])
    if len(parts) == 3 and parts[0] == 'add':
        try:
            return Command('add', cell=(int(parts[1]), int(parts[2])))
        except ValueError:
            pass
    if len(parts) == 2 and parts[0] == 'remove':
        return Command('remove', pid=parts[1])
    raise ValueError(f'잘못된 명령: {text!r}')


class MissionManager:
    def __init__(self, terrain: TerrainMap, planner: AStarPlanner,
                 scheduler: PatrolScheduler, energy: EnergyMonitor):
        self.terrain = terrain
        self.planner = planner
        self.scheduler = scheduler
        self.energy = energy
        self.state = S.IDLE
        self.resume_state = S.PATROLLING
        self.target: Optional[PatrolPoint] = None
        self.path: Path = []
        self.idx = 0
        self.pos: Optional[Cell] = None
        self.battery_wh: Optional[float] = None

    def status(self) -> Status:
        """Current state as a Status output (e.g. to publish once at start-up)."""
        return self._status()

    # ---------------------------------------------------------------- events
    def on_command(self, cmd: Command, now: float) -> list[Output]:
        """Operator commands (FR-07, FR-08). Invalid input gives an alarm, never an exception (NFR-04)."""
        if cmd.kind == 'add':
            c = cmd.cell
            if c is not None and self.terrain.in_bounds(c) and not self.terrain.is_obstacle(c):
                self.scheduler.add_point(c)
                return [self._status()]
            return [Alarm(f'잘못된 순찰 지점: {c}')]
        if cmd.kind == 'remove':
            if self.scheduler.remove_point(cmd.pid):
                return [self._status()]
            return [Alarm(f'없는 지점 ID: {cmd.pid}')]
        if cmd.kind == 'start':
            if self.state not in (S.IDLE, S.STOPPED):
                return []
            if len(self.scheduler) == 0 or self.pos is None or self.battery_wh is None:
                return [Alarm('시작 조건 미충족 (순찰 지점, 현재 위치, 배터리 잔량 필요)')]
            return self._start_next_target(now)
        if cmd.kind == 'stop':
            if self.state not in RUNNING:
                return []
            self.path, self.idx = [], 0
            return self._set_state(S.STOPPED)
        return [Alarm(f'알 수 없는 명령: {cmd.kind}')]

    def on_battery(self, wh: float) -> list[Output]:
        self.battery_wh = wh
        return []

    def on_pose(self, cell: Cell, now: float) -> list[Output]:
        """Position update: progress, arrival, return check, next cell (FR-10, FR-13, FR-14, FR-16)."""
        self.pos = cell
        if self.state not in (S.PATROLLING, S.RETURNING) or not self.path:
            return []
        if cell in self.path[self.idx:]:
            self.idx = self.path.index(cell, self.idx)
        if cell == self.path[-1]:
            if self.state == S.RETURNING:
                return self._set_state(S.IDLE, note='복귀 완료')
            self.scheduler.mark_visited(self.target.pid, now)
            return self._start_next_target(now)
        return self._step()

    def on_obstacles(self, cells: Iterable[Cell], now: float) -> list[Output]:
        """New obstacles: update the map and re-plan if the current path is blocked (FR-11, FR-12)."""
        if not self.terrain.add_obstacles(cells):
            return []
        self.energy.update_return_map()
        if self.state not in (S.PATROLLING, S.RETURNING) or self.planner.is_path_valid(self.path, self.idx):
            return []
        self.resume_state = self.state
        out = self._set_state(S.REPLANNING)
        if self.resume_state == S.RETURNING:
            return out + self._go_home()
        new_path = self.planner.plan(self.pos, self.target.cell)
        if new_path is None:
            out.append(Alarm(f'경로 없음: {self.target.pid}'))
            return out + self._start_next_target(now, exclude={self.target.pid})
        self.path, self.idx = new_path, 0
        return out + self._set_state(S.PATROLLING) + [PathUpdate(self.path)] + self._step()

    # ---------------------------------------------------------------- helpers
    def _start_next_target(self, now: float, exclude: Iterable[str] = ()) -> list[Output]:
        """Pick the next point and plan to it; skip unreachable points (FR-05, FR-06, FR-09)."""
        out: list[Output] = []
        failed = set(exclude)
        while True:
            p = self.scheduler.select_next(now, exclude=failed)
            if p is None:
                self.path, self.idx, self.target = [], 0, None
                return out + [Alarm('경로 없음')] + self._set_state(S.IDLE)
            new_path = self.planner.plan(self.pos, p.cell)
            if new_path is None:
                failed.add(p.pid)
                out.append(Alarm(f'경로 없음: {p.pid}'))
                continue
            self.target, self.path, self.idx = p, new_path, 0
            # Report the new target and visit times even when the state stays "patrolling".
            status = self._set_state(S.PATROLLING) or [self._status()]
            return out + status + [PathUpdate(self.path)] + self._step()

    def _step(self) -> list[Output]:
        """Send the next cell, unless the battery says it is time to go home."""
        if self.idx + 1 >= len(self.path):
            return []
        nxt = self.path[self.idx + 1]
        if self.state == S.PATROLLING and self.energy.should_return(self.pos, self.battery_wh, nxt):
            return self._go_home()
        return [NextCell(nxt)]

    def _go_home(self) -> list[Output]:
        home = self.energy.return_path(self.pos)
        if home is None:
            self.path, self.idx = [], 0
            return [Alarm('기지 복귀 경로 없음')] + self._set_state(S.STOPPED)
        self.target = None
        if len(home) == 1:
            self.path, self.idx = [], 0
            return self._set_state(S.IDLE, note='복귀 완료')
        self.path, self.idx = home, 0
        return self._set_state(S.RETURNING) + [PathUpdate(self.path), NextCell(self.path[1])]

    def _set_state(self, s: MissionState, note: str = '') -> list[Output]:
        """Change state; every change is reported (FR-15)."""
        if s == self.state:
            return []
        self.state = s
        return [self._status(note)]

    def _status(self, note: str = '') -> Status:
        info = {
            'state': self.state.value,
            'target': self.target.pid if self.target else None,
            'points': [{'pid': p.pid, 'cell': list(p.cell), 'last_visit': p.last_visit}
                       for p in self.scheduler.snapshot()],
        }
        if note:
            info['note'] = note
        return Status(self.state, info)
