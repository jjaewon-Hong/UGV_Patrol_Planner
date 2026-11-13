"""Build the campus world (elevation + obstacle grids) used by the patrol controller (FR-20, FR-21).

    python tools/campus_world.py dem   <processed_dir> <copernicus_dem.tif> [cell_m=10]
    python tools/campus_world.py build <processed_dir> <osm|pred.png> <out_dir> <row0> <col0> [n=100] [cell_m=10]

`dem` samples the elevation at the centre of every cell of the whole campus crop (same grid as the
obstacle grid, so both line up by construction). `build` cuts an n x n window starting at cell (row0, col0)
and writes dem.npy (float, metres) and obstacles.npy (bool) for the controller, plus obstacle_classes.npy
(uint8: 0 free, 1 building, 2 water, 3 tree) so the views can colour each obstacle by what it is.

Elevation: Copernicus GLO-30 DEM (c) DLR/Airbus, provided under COPERNICUS by the European Union and ESA.
It is a surface model (includes roofs and tree tops); buildings and trees are obstacles anyway.
"""
import os
import sys

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(__file__))
from campus_map import EVAL_GSD, load_pred  # noqa: E402
from georef import CampusGeoref, from_tm  # noqa: E402
from osm_mask import _size, rasterize  # noqa: E402

RATIO = 0.3   # a cell is an obstacle when this share of its area is building / water / forest


def _geo(d):
    return CampusGeoref.load(os.path.join(d, 'campus_georef.json'))


def sample_dem(d: str, tif: str, cell_m: float = 10.0) -> np.ndarray:
    im = Image.open(tif)
    (sx, sy, _), (_, _, _, lon0, lat0, _) = im.tag_v2[33550], im.tag_v2[33922]
    dem = np.asarray(im, dtype=np.float32)
    geo = _geo(d)
    w, h = _size(d, cell_m)
    out = np.zeros((h, w), np.float32)
    for r in range(h):
        for c in range(w):
            e, n = geo.to_tm((c + 0.5) * cell_m / geo.gsd, (r + 0.5) * cell_m / geo.gsd)
            lat, lon = from_tm(e, n)
            x, y = (lon - lon0) / sx, (lat0 - lat) / sy      # Copernicus grid points sit on whole arcseconds
            x0, y0 = int(x), int(y)
            fx, fy = x - x0, y - y0
            q = dem[y0:y0 + 2, x0:x0 + 2]
            out[r, c] = (q[0, 0] * (1 - fx) * (1 - fy) + q[0, 1] * fx * (1 - fy)
                         + q[1, 0] * (1 - fx) * fy + q[1, 1] * fx * fy)
    path = os.path.join(d, f'campus_dem_{cell_m:g}m.npy')
    np.save(path, out)
    print(path, out.shape, f'elevation {out.min():.1f}..{out.max():.1f} m')
    return out


def obstacle_grid(d: str, source: str, cell_m: float = 10.0) -> tuple[np.ndarray, np.ndarray]:
    """Obstacle cells (bool) and what they are (uint8: 0 free, 1 building, 2 water, 3 forest = osm_mask.CLASSES).

    A cell is an obstacle when building + water + forest cover at least RATIO of it; its class is the
    largest of the three, so the obstacle grid itself does not depend on the classes.
    """
    size = _size(d, EVAL_GSD)
    labels = rasterize(d, EVAL_GSD, size) if source == 'osm' else load_pred(source, size)
    k = int(round(cell_m / EVAL_GSD))
    rows, cols = labels.shape[0] // k, labels.shape[1] // k
    cells = labels[:rows * k, :cols * k].reshape(rows, k, cols, k)
    frac = np.stack([(cells == c).mean(axis=(1, 3)) for c in (1, 2, 3)])
    obs = frac.sum(axis=0) >= RATIO
    return obs, np.where(obs, frac.argmax(axis=0) + 1, 0).astype(np.uint8)


def build(d: str, source: str, out_dir: str, row0: int, col0: int, n: int = 100, cell_m: float = 10.0) -> None:
    dem = np.load(os.path.join(d, f'campus_dem_{cell_m:g}m.npy'))
    obs, cls = obstacle_grid(d, source, cell_m)
    rows, cols = min(dem.shape[0], obs.shape[0]), min(dem.shape[1], obs.shape[1])
    if row0 + n > rows or col0 + n > cols:
        raise SystemExit(f'window {row0}+{n}, {col0}+{n} exceeds grid {rows}x{cols}')
    win = (slice(row0, row0 + n), slice(col0, col0 + n))
    os.makedirs(out_dir, exist_ok=True)
    np.save(os.path.join(out_dir, 'dem.npy'), dem[win].astype(np.float64))
    np.save(os.path.join(out_dir, 'obstacles.npy'), obs[win])
    np.save(os.path.join(out_dir, 'obstacle_classes.npy'), cls[win])  # display only (building / water / tree colours)
    # preview: photo with obstacle cells tinted red
    img = cv2.imread(os.path.join(d, 'campus_1m.jpg'))
    px = int(cell_m)                                  # campus_1m.jpg is 1 m/px
    crop = img[row0 * px:(row0 + n) * px, col0 * px:(col0 + n) * px].copy()
    red = cv2.resize(obs[win].astype(np.uint8), (crop.shape[1], crop.shape[0]), interpolation=cv2.INTER_NEAREST)
    crop[red > 0] = (0.5 * crop[red > 0] + (0, 0, 127)).astype(np.uint8)
    cv2.imwrite(os.path.join(d, 'campus_world_preview.jpg'), crop, [cv2.IMWRITE_JPEG_QUALITY, 85])
    e = dem[win]
    slope = np.degrees(np.arctan(np.maximum(np.abs(np.diff(e, axis=0))[:, :-1], np.abs(np.diff(e, axis=1))[:-1]) / cell_m))
    free = ~obs[win][:-1, :-1]
    print(f'{out_dir}: {n}x{n} cells of {cell_m:g} m, obstacles {obs[win].mean():.2f}, '
          f'elevation {e.min():.1f}..{e.max():.1f} m, free cells steeper than 30 deg: {(slope[free] > 30).mean():.3f}')


if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'dem':
        sample_dem(sys.argv[2], sys.argv[3], *(float(v) for v in sys.argv[4:5]))
    elif cmd == 'build':
        build(sys.argv[2], sys.argv[3], sys.argv[4], *(int(v) for v in sys.argv[5:8]),
              *(float(v) for v in sys.argv[8:9]))
    else:
        raise SystemExit(__doc__)
