"""Fit the RViz2 camera and grid to the map size (maps differ: 3 km default, 1 km campus).

Used by share/launch/patrol.launch.py to write a per-run copy of share/rviz/patrol.rviz.
"""
import yaml

DISTANCE_PER_SIDE = 1.45  # camera distance / map side; the map fills the 3D view at the default pitch
GRID_CELLS = 10


def fit_view(rviz: dict, map_size: int, cell_size_m: float, ground_z: float = 0.0, focus_z: float = 0.0,
             z_scale: float = 1.0) -> dict:
    """Centre the orbit camera on the map and size the grid to it. Changes and returns `rviz`.

    With z_scale != 1 the map display is named e.g. 'height x5' so a stretched view is not read as true slope.
    """
    side = map_size * cell_size_m
    centre = (map_size - 1) * cell_size_m / 2  # markers sit at cell centres 0 .. (N-1)*cell
    grid_cell = side / GRID_CELLS
    manager = rviz['Visualization Manager']
    for d in manager['Displays']:
        if d.get('Class') == 'rviz_default_plugins/Grid':
            d['Name'] = f'Grid ({grid_cell:g} m)'
            d['Cell Size'] = grid_cell
            d['Plane Cell Count'] = GRID_CELLS
            d['Offset'] = {'X': centre, 'Y': centre, 'Z': ground_z}
        elif d.get('Class') == 'rviz_default_plugins/MarkerArray' and z_scale != 1.0:
            d['Name'] = f"Patrol map (height x{z_scale:g})"
    view = manager['Views']['Current']
    view['Distance'] = DISTANCE_PER_SIDE * side
    view['Focal Point'] = {'X': centre, 'Y': centre, 'Z': focus_z}
    return rviz


def write_fitted(template: str, out: str, map_size: int, cell_size_m: float,
                 ground_z: float = 0.0, focus_z: float = 0.0, z_scale: float = 1.0) -> None:
    with open(template, encoding='utf-8') as f:
        rviz = yaml.safe_load(f)
    fit_view(rviz, map_size, cell_size_m, ground_z, focus_z, z_scale)
    with open(out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(rviz, f, sort_keys=False, allow_unicode=True)
