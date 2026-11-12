"""OpenStreetMap reference masks for the campus image (ground truth for NFR-08).

    python tools/osm_mask.py fetch   <processed_dir>          # Overpass -> osm_campus.json
    python tools/osm_mask.py overlay <processed_dir> [dx dy]  # draw OSM outlines on the 1 m image
    python tools/osm_mask.py align   <processed_dir>          # shift OSM onto image edges -> campus_georef.json
    python tools/osm_mask.py mask    <processed_dir> <gsd_m>  # osm_mask_<gsd>.png, 0 free / 1 building / 2 water / 3 forest

OSM data (c) OpenStreetMap contributors, ODbL. Not stored in this repo; fetched on demand.
"""
import json
import os
import sys

import cv2
import numpy as np
import requests

sys.path.insert(0, os.path.dirname(__file__))
from georef import CampusGeoref, to_tm  # noqa: E402

OVERPASS = 'https://overpass-api.de/api/interpreter'
MARGIN_DEG = 0.003
CLASSES = {'building': 1, 'water': 2, 'forest': 3}
COLORS = {1: (0, 0, 255), 2: (255, 128, 0), 3: (0, 200, 0)}   # BGR


def _paths(d):
    return os.path.join(d, 'campus_georef.json'), os.path.join(d, 'osm_campus.json')


def fetch(d: str) -> None:
    with open(_paths(d)[0]) as f:
        (n, w), (s, e) = json.load(f)['latlon_nw_se']
    bbox = f'{s - MARGIN_DEG},{w - MARGIN_DEG},{n + MARGIN_DEG},{e + MARGIN_DEG}'
    q = f"""[out:json][timeout:60];
(
  way["building"]({bbox}); relation["building"]({bbox});
  way["natural"="water"]({bbox}); relation["natural"="water"]({bbox});
  way["landuse"~"reservoir|basin"]({bbox}); way["waterway"="riverbank"]({bbox});
  way["natural"="wood"]({bbox}); way["landuse"="forest"]({bbox}); relation["landuse"="forest"]({bbox});
);
out geom;"""
    r = requests.post(OVERPASS, data={'data': q}, headers={'User-Agent': 'ugv-patrol-planner/0.1'}, timeout=120)
    r.raise_for_status()
    data = r.json()
    with open(_paths(d)[1], 'w', encoding='utf-8') as f:
        json.dump(data, f)
    print(f"{len(data['elements'])} elements saved")


def _class_of(tags: dict) -> int | None:
    if 'building' in tags:
        return CLASSES['building']
    if tags.get('natural') == 'water' or tags.get('landuse') in ('reservoir', 'basin') \
            or tags.get('waterway') == 'riverbank':
        return CLASSES['water']
    if tags.get('natural') == 'wood' or tags.get('landuse') == 'forest':
        return CLASSES['forest']
    return None


def polygons(d: str):
    """Yield (class, outer rings as lists of (E, N)). Holes of multipolygons are ignored."""
    with open(_paths(d)[1], encoding='utf-8') as f:
        data = json.load(f)
    for el in data['elements']:
        cls = _class_of(el.get('tags', {}))
        if cls is None:
            continue
        if el['type'] == 'way' and 'geometry' in el:
            rings = [el['geometry']]
        elif el['type'] == 'relation':
            rings = [m['geometry'] for m in el.get('members', []) if m.get('role') == 'outer' and 'geometry' in m]
        else:
            continue
        yield cls, [[to_tm(p['lat'], p['lon']) for p in ring] for ring in rings]


def rasterize(d: str, gsd: float, size_px: tuple[int, int], shift=(0.0, 0.0)) -> np.ndarray:
    geo = CampusGeoref.load(_paths(d)[0], gsd)
    geo.left += shift[0]
    geo.top += shift[1]
    mask = np.zeros((size_px[1], size_px[0]), np.uint8)
    # forest first, then water, then buildings on top
    for cls_order in (3, 2, 1):
        for cls, rings in polygons(d):
            if cls != cls_order:
                continue
            for ring in rings:
                pts = np.array([geo.to_px(e, n) for e, n in ring], np.int32)
                cv2.fillPoly(mask, [pts], cls)
    return mask


def _size(d, gsd):
    with open(_paths(d)[0]) as f:
        w, h = json.load(f)['size_m']
    return int(round(w / gsd)), int(round(h / gsd))


def overlay(d: str, dx: float = 0.0, dy: float = 0.0) -> None:
    img = cv2.imread(os.path.join(d, 'campus_1m.jpg'))
    mask = rasterize(d, 1.0, (img.shape[1], img.shape[0]), (dx, dy))
    out = img.copy()
    for cls, color in COLORS.items():
        cnts, _ = cv2.findContours((mask == cls).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, cnts, -1, color, 2)
    path = os.path.join(d, f'osm_overlay_{dx:+.0f}_{dy:+.0f}.jpg')
    cv2.imwrite(path, out, [cv2.IMWRITE_JPEG_QUALITY, 85])
    print(path)


def align(d: str, gsd: float = 0.5, search_m: float = 40.0) -> None:
    """Shift (dx, dy) in metres that puts OSM building outlines on the strongest image edges.

    Uses only the photo and OSM, not the model's prediction, so the later IoU is not biased.
    The result is written to shift_m in campus_georef.json.
    """
    img = cv2.imread(os.path.join(d, 'campus_50cm.png'), cv2.IMREAD_GRAYSCALE).astype(np.float32)
    img = cv2.GaussianBlur(img, (0, 0), 1.0)
    grad = cv2.magnitude(cv2.Sobel(img, cv2.CV_32F, 1, 0), cv2.Sobel(img, cv2.CV_32F, 0, 1))
    size = (img.shape[1], img.shape[0])
    geo_path = _paths(d)[0]
    with open(geo_path) as f:
        info = json.load(f)
    info['shift_m'] = [0.0, 0.0]
    with open(geo_path, 'w') as f:
        json.dump(info, f, indent=2)
    outline = np.zeros((size[1], size[0]), np.uint8)
    mask = rasterize(d, gsd, size) == CLASSES['building']
    cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cv2.drawContours(outline, cnts, -1, 1, 1)
    ys, xs = np.nonzero(outline)
    r = int(search_m / gsd)
    scores = np.zeros((2 * r + 1, 2 * r + 1), np.float32)
    for iy, sy in enumerate(range(-r, r + 1)):          # shift in pixels, +row = south
        for ix, sx in enumerate(range(-r, r + 1)):
            x, y = xs + sx, ys + sy
            ok = (x >= 0) & (x < size[0]) & (y >= 0) & (y < size[1])
            scores[iy, ix] = grad[y[ok], x[ok]].mean()
    iy, ix = np.unravel_index(np.argmax(scores), scores.shape)
    # Outlines fit best when moved (ix-r, iy-r) px right/down, i.e. the image origin moves the other way.
    dx, dy = -(ix - r) * gsd, (iy - r) * gsd               # origin correction: metres east, metres north
    base = float(np.median(scores))
    print(f'image origin shift dx={dx:+.1f} m east, dy={dy:+.1f} m north; '
          f'edge score {scores[iy, ix]:.1f} vs median {base:.1f} ({len(xs)} outline px)')
    info['shift_m'] = [dx, dy]
    with open(geo_path, 'w') as f:
        json.dump(info, f, indent=2)


def save_mask(d: str, gsd: float) -> None:
    with open(_paths(d)[0]) as f:
        shift = json.load(f).get('shift_m', [0, 0])
    del shift  # already applied by CampusGeoref
    mask = rasterize(d, gsd, _size(d, gsd))
    path = os.path.join(d, f'osm_mask_{gsd:g}m.png')
    cv2.imwrite(path, mask)
    counts = {name: int((mask == v).sum()) for name, v in CLASSES.items()}
    print(path, mask.shape, counts)


if __name__ == '__main__':
    cmd, root = sys.argv[1], sys.argv[2]
    if cmd == 'fetch':
        fetch(root)
    elif cmd == 'overlay':
        overlay(root, *(float(v) for v in sys.argv[3:5]))
    elif cmd == 'align':
        align(root)
    elif cmd == 'mask':
        save_mask(root, float(sys.argv[3]))
    else:
        raise SystemExit(__doc__)
