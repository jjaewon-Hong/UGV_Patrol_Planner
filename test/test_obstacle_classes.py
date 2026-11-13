"""Campus obstacle class grids (building / water / tree colours) agree with the obstacle grids the planner uses."""
import os

import numpy as np
import pytest

from ugv_patrol_planner.common import load_config

ROOT = os.path.join(os.path.dirname(__file__), '..', 'share')  # config/, data/ as installed


@pytest.mark.parametrize('name', ['campus', 'campus_osm'])
def test_every_obstacle_has_a_class_and_free_cells_have_none(name):
    cfg = load_config(os.path.join(ROOT, 'config', f'{name}.yaml'))
    assert cfg.obstacle_class_file
    obs = np.load(os.path.join(ROOT, cfg.obstacle_file))
    cls = np.load(os.path.join(ROOT, cfg.obstacle_class_file))
    assert cls.shape == obs.shape == (cfg.map_size, cfg.map_size)
    assert set(np.unique(cls[obs])) <= {1, 2, 3}      # building, water, tree
    assert not cls[~obs].any()
    assert (cls == 1).any() and (cls == 3).any()       # campus has both buildings and trees


@pytest.mark.parametrize('name', ['campus', 'campus_osm'])
def test_campus_gates_are_on_the_map(name):
    cfg = load_config(os.path.join(ROOT, 'config', f'{name}.yaml'))
    labels = [label for label, _ in cfg.landmarks]
    assert labels == ['Main gate (300 m N)', 'East gate']
    assert all(0 <= r < cfg.map_size and 0 <= c < cfg.map_size for _, (r, c) in cfg.landmarks)


def test_maps_without_classes_default_to_empty():
    cfg = load_config(os.path.join(ROOT, 'config', 'params.yaml'))
    assert cfg.obstacle_class_file == ''
    assert cfg.landmarks == ()
