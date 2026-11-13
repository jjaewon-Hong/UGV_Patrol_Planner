"""Shared data types: Cell, Path, PatrolPoint, MissionState, Config."""
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

Cell = tuple[int, int]  # (row, col)
Path = list[Cell]       # consecutive 4-connected cells, start to goal


@dataclass
class PatrolPoint:
    pid: str
    cell: Cell
    last_visit: Optional[float] = None  # None = never visited


class MissionState(Enum):
    IDLE = '대기'
    PATROLLING = '순찰 중'
    REPLANNING = '재계획 중'
    RETURNING = '복귀 중'
    STOPPED = '정지'


@dataclass(frozen=True)
class Config:
    """Read-only settings (config/params.yaml). Passed to each unit at construction."""
    dem_file: str = 'data/dem_100x100.npy'
    obstacle_file: str = ''          # optional N x N bool grid of known obstacles (FR-20)
    obstacle_class_file: str = ''    # optional N x N uint8 grid, 1 building / 2 water / 3 tree (display only)
    landmarks: tuple[tuple[str, Cell], ...] = field(default_factory=tuple)  # (label, cell), display only, e.g. gates
    map_size: int = 100
    cell_size_m: float = 30.0
    static_obstacles: tuple[Cell, ...] = field(default_factory=tuple)
    max_slope_deg: float = 30.0
    slope_weight: float = 2.0
    sensor_radius: int = 3
    capacity_wh: float = 500.0
    wh_per_m: float = 0.01
    wh_per_climb_m: float = 0.5
    return_margin: float = 1.2
    base_cell: Cell = (0, 0)
    replan_time_limit_s: float = 0.5


def load_config(path: str) -> Config:
    """Load config/params.yaml into a Config."""
    import yaml  # available in ROS 2 environments (python3-yaml)

    with open(path, encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    m, t = raw.get('map', {}), raw.get('terrain', {})
    s, e, mi = raw.get('sensor', {}), raw.get('energy', {}), raw.get('mission', {})
    d = Config()
    return Config(
        dem_file=m.get('dem_file', d.dem_file),
        obstacle_file=m.get('obstacle_file', d.obstacle_file) or '',
        obstacle_class_file=m.get('obstacle_class_file', d.obstacle_class_file) or '',
        map_size=int(m.get('size', d.map_size)),
        cell_size_m=float(m.get('cell_size_m', d.cell_size_m)),
        static_obstacles=tuple(tuple(c) for c in m.get('static_obstacles', [])),
        landmarks=tuple((str(lm['name']), tuple(lm['cell'])) for lm in m.get('landmarks', [])),
        max_slope_deg=float(t.get('max_slope_deg', d.max_slope_deg)),
        slope_weight=float(t.get('slope_weight', d.slope_weight)),
        sensor_radius=int(s.get('radius_cells', d.sensor_radius)),
        capacity_wh=float(e.get('capacity_wh', d.capacity_wh)),
        wh_per_m=float(e.get('wh_per_m', d.wh_per_m)),
        wh_per_climb_m=float(e.get('wh_per_climb_m', d.wh_per_climb_m)),
        return_margin=float(e.get('return_margin', d.return_margin)),
        base_cell=tuple(mi.get('base_cell', d.base_cell)),
        replan_time_limit_s=float(mi.get('replan_time_limit_s', d.replan_time_limit_s)),
    )
