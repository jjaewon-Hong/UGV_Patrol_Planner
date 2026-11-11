"""Loading shared by all nodes: config, terrain map and scenario file."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node

from ugv_patrol_planner.common import Cell, Config, load_config
from ugv_patrol_planner.terrain.terrain_map import TerrainMap

PKG = 'ugv_patrol_planner'


def share_path(*parts: str) -> str:
    return os.path.join(get_package_share_directory(PKG), *parts)


def _resolve(path: str) -> str:
    return path if os.path.isabs(path) else share_path(path)


def load_world(node: Node) -> tuple[Config, TerrainMap, dict]:
    """Read the `config` and `scenario` parameters and build the terrain map."""
    node.declare_parameter('config', share_path('config', 'params.yaml'))
    node.declare_parameter('scenario', share_path('scenarios', 'default.yaml'))
    cfg = load_config(_resolve(node.get_parameter('config').value))
    terrain = TerrainMap.from_file(_resolve(cfg.dem_file), cfg)
    with open(_resolve(node.get_parameter('scenario').value), encoding='utf-8') as f:
        scenario = yaml.safe_load(f) or {}
    return cfg, terrain, scenario


def to_point(c: Cell) -> Point:
    """Grid cell as geometry_msgs/Point (x = row, y = col)."""
    return Point(x=float(c[0]), y=float(c[1]), z=0.0)


def to_cell(p) -> Cell:
    return int(round(p.x)), int(round(p.y))
