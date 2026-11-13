"""RViz2 view is fitted to the map size (default 3 km map, 1 km campus map)."""
import os

import pytest
import yaml

from ugv_patrol_planner.rviz_view import fit_view, write_fitted

TEMPLATE = os.path.join(os.path.dirname(__file__), '..', 'rviz', 'patrol.rviz')


def template():
    with open(TEMPLATE, encoding='utf-8') as f:
        return yaml.safe_load(f)


def grid_of(rviz):
    return next(d for d in rviz['Visualization Manager']['Displays'] if d['Class'] == 'rviz_default_plugins/Grid')


@pytest.mark.parametrize('n, cell', [(100, 30.0), (100, 10.0), (50, 4.0)])
def test_camera_and_grid_cover_the_map(n, cell):
    rviz = fit_view(template(), n, cell, ground_z=50.0, focus_z=70.0)
    side, centre = n * cell, (n - 1) * cell / 2
    view = rviz['Visualization Manager']['Views']['Current']
    assert view['Focal Point'] == {'X': centre, 'Y': centre, 'Z': 70.0}
    assert view['Distance'] == pytest.approx(1.45 * side)
    grid = grid_of(rviz)
    assert grid['Cell Size'] * grid['Plane Cell Count'] == pytest.approx(side)
    assert grid['Offset'] == {'X': centre, 'Y': centre, 'Z': 50.0}


def test_campus_map_is_three_times_closer_than_default():
    default = fit_view(template(), 100, 30.0)['Visualization Manager']['Views']['Current']['Distance']
    campus = fit_view(template(), 100, 10.0)['Visualization Manager']['Views']['Current']['Distance']
    assert campus == pytest.approx(default / 3)


def markers_of(rviz):
    return next(d for d in rviz['Visualization Manager']['Displays']
                if d['Class'] == 'rviz_default_plugins/MarkerArray')


def test_stretched_heights_are_labelled_and_true_scale_is_not():
    assert markers_of(fit_view(template(), 100, 10.0, z_scale=5.0))['Name'] == 'Patrol map (height x5)'
    assert markers_of(fit_view(template(), 100, 10.0))['Name'] == markers_of(template())['Name']


def test_written_file_keeps_the_rest_of_the_template(tmp_path):
    out = tmp_path / 'fitted.rviz'
    write_fitted(TEMPLATE, str(out), 100, 10.0)
    with open(out, encoding='utf-8') as f:
        rviz = yaml.safe_load(f)
    assert grid_of(rviz)['Name'] == 'Grid (100 m)'
    original = template()
    assert rviz['Panels'] == original['Panels']
    assert rviz['Visualization Manager']['Global Options'] == original['Visualization Manager']['Global Options']
    assert rviz['Visualization Manager']['Displays'][1] == original['Visualization Manager']['Displays'][1]
