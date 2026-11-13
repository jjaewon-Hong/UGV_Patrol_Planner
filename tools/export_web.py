#!/usr/bin/env python3
"""Turn the maps, configs and scenarios into one JSON file per map for the web demo (web/maps/).

The browser then needs no .npy or YAML reader. Arrays are base64 text: elevation as little-endian
float64 (unchanged, so slopes are exactly the ones the ROS 2 run sees), obstacles and obstacle
classes as one byte per cell. Random flags are placed here, with the same function the ROS 2 demo uses.

    python3 tools/export_web.py web/maps
"""
import base64
import json
import os
import sys

import numpy as np
import yaml

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, ROOT)
from ugv_patrol_planner.common import load_config  # noqa: E402
from ugv_patrol_planner.mission.flags import scenario_points  # noqa: E402
from ugv_patrol_planner.terrain.terrain_map import TerrainMap  # noqa: E402

# id, config, scenario, title, one-line description (shown on the page)
MAPS = [
    ('campus', 'campus.yaml', 'campus.yaml', '캠퍼스 · 모델 인식 지도',
     '영남대 캠퍼스 1 km × 1 km. 장애물은 정사영상을 분할 모델이 인식한 결과입니다.'),
    ('campus_osm', 'campus_osm.yaml', 'campus_osm.yaml', '캠퍼스 · OSM 지도',
     '같은 지역, 장애물은 OpenStreetMap의 건물·물·숲입니다. 모델 지도와 비교용입니다.'),
    ('default', 'params.yaml', 'default.yaml', '시험용 합성 지형',
     '3 km × 3 km 합성 지형. 제어기가 모르는 숨은 장애물 16칸이 있어 가는 길에 발견합니다.'),
]


def b64(array: np.ndarray, dtype: str) -> str:
    return base64.b64encode(np.ascontiguousarray(array, dtype=dtype).tobytes()).decode('ascii')


def path(*parts: str) -> str:
    return os.path.join(ROOT, *parts)


def export_map(map_id: str, config_file: str, scenario_file: str, title: str, about: str) -> dict:
    cfg = load_config(path('config', config_file))
    with open(path('scenarios', scenario_file), encoding='utf-8') as f:
        scenario = yaml.safe_load(f) or {}
    elev = np.load(path(cfg.dem_file))
    terrain = TerrainMap.from_file(path(cfg.dem_file), cfg, path(cfg.obstacle_file) if cfg.obstacle_file else '')
    n = cfg.map_size
    obstacles = np.array([[terrain.is_obstacle((r, c)) for c in range(n)] for r in range(n)], dtype=np.uint8)
    classes = (np.load(path(cfg.obstacle_class_file)) if cfg.obstacle_class_file
               else np.zeros((n, n), np.uint8))
    return {
        'id': map_id,
        'title': title,
        'about': about,
        'n': n,
        'cell_size_m': cfg.cell_size_m,
        'max_slope_deg': cfg.max_slope_deg,
        'slope_weight': cfg.slope_weight,
        'sensor_radius': cfg.sensor_radius,
        'capacity_wh': cfg.capacity_wh,
        'wh_per_m': cfg.wh_per_m,
        'wh_per_climb_m': cfg.wh_per_climb_m,
        'return_margin': cfg.return_margin,
        'base': list(cfg.base_cell),
        'start': list(scenario.get('start_cell', cfg.base_cell)),
        'initial_battery_wh': float(scenario.get('initial_battery_wh', cfg.capacity_wh)),
        'landmarks': [{'name': name, 'cell': list(cell)} for name, cell in cfg.landmarks],
        'hidden': [list(c) for c in scenario.get('hidden_obstacles', [])],
        'points': [list(p) for p in scenario_points(terrain, cfg.base_cell, scenario)],
        'elevation_b64': b64(elev, '<f8'),        # n*n float64, row-major
        'obstacles_b64': b64(obstacles, 'u1'),    # n*n bytes, 1 = obstacle (static ones included)
        'classes_b64': b64(classes, 'u1'),        # n*n bytes, 1 building / 2 water / 3 tree / 0 unknown
    }


def main(out_dir: str) -> None:
    os.makedirs(out_dir, exist_ok=True)
    index = []
    for map_id, config_file, scenario_file, title, about in MAPS:
        data = export_map(map_id, config_file, scenario_file, title, about)
        with open(os.path.join(out_dir, f'{map_id}.json'), 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, separators=(',', ':'))
        index.append({'id': map_id, 'title': title, 'about': about})
        print(f"{map_id}: {data['n']}x{data['n']}, {int(np.frombuffer(base64.b64decode(data['obstacles_b64']), np.uint8).sum())} "
              f"obstacle cells, points {data['points']}")
    with open(os.path.join(out_dir, 'index.json'), 'w', encoding='utf-8') as f:
        json.dump(index, f, ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else path('web', 'maps'))
