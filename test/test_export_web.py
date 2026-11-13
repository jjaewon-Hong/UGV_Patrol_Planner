"""Web demo map export (tools/export_web.py): the JSON holds the same map, settings and flags as the ROS 2 run."""
import base64
import json
import os
import sys

import numpy as np
import pytest

ROOT = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import export_web  # noqa: E402


@pytest.fixture(scope='module')
def maps(tmp_path_factory):
    out = tmp_path_factory.mktemp('maps')
    export_web.main(str(out))
    loaded = {}
    for name in ['campus', 'campus_osm', 'default']:
        with open(out / f'{name}.json', encoding='utf-8') as f:
            loaded[name] = json.load(f)
    with open(out / 'index.json', encoding='utf-8') as f:
        loaded['index'] = json.load(f)
    return loaded


def grid(m, key, dtype):
    return np.frombuffer(base64.b64decode(m[key]), dtype).reshape(m['n'], m['n'])


def test_index_lists_the_three_maps(maps):
    assert [e['id'] for e in maps['index']] == ['campus', 'campus_osm', 'default']


def test_campus_settings_and_flags_match_the_ros2_run(maps):
    m = maps['campus']
    assert (m['n'], m['cell_size_m'], m['max_slope_deg']) == (100, 10.0, 15.0)
    assert m['base'] == [42, 30] and m['start'] == [42, 30]
    # scripts/scenario_test.py S5 prints these flags; cpp/test/test_sim.cpp uses them too
    assert m['points'] == [[64, 21], [3, 1], [35, 48]]
    assert m['hidden'] == []
    assert [lm['cell'] for lm in m['landmarks']] == [[0, 14], [13, 99]]


def test_elevation_is_bit_identical(maps):
    for name, dem in [('campus', 'share/data/campus_pred/dem.npy'), ('default', 'share/data/dem_100x100.npy')]:
        original = np.load(os.path.join(ROOT, dem))
        assert np.array_equal(grid(maps[name], 'elevation_b64', '<f8'), original)


def test_obstacles_and_classes(maps):
    m = maps['campus']
    obstacles = grid(m, 'obstacles_b64', np.uint8)
    assert np.array_equal(obstacles, np.load(os.path.join(ROOT, 'share/data/campus_pred/obstacles.npy')))
    classes = grid(m, 'classes_b64', np.uint8)
    assert set(np.unique(classes)) <= {0, 1, 2, 3}
    assert not obstacles[tuple(m['base'])]
    for r, c in m['points']:
        assert not obstacles[r, c]


def test_default_map_has_static_and_hidden_obstacles(maps):
    m = maps['default']
    obstacles = grid(m, 'obstacles_b64', np.uint8)
    assert obstacles.sum() == 2 and obstacles[40, 41] and obstacles[40, 42]   # config static_obstacles
    assert len(m['hidden']) == 16 and not any(obstacles[r, c] for r, c in m['hidden'])
    assert m['points'] == [[15, 85], [85, 85], [85, 15]]
    assert (m['cell_size_m'], m['max_slope_deg'], m['base']) == (30.0, 30.0, [0, 0])
