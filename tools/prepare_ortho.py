"""Join two vertically adjacent NGII orthophoto tiles and crop the study area.

The tiles come without georeferencing, so the overlap between the north tile's bottom and the
south tile's top is found by template matching. Step 1 (`mosaic`) writes a 1 m/px preview mosaic
and the offset; step 2 (`crop`) cuts a box (given in preview pixels) out of the full 0.25 m tiles.

    python tools/prepare_ortho.py mosaic <north.tif> <south.tif> <out_dir>
    python tools/prepare_ortho.py crop <north.tif> <south.tif> <out_dir> <x0> <y0> <x1> <y1>

Source data (not in this repo; large files with their own terms of use):
    NGII 국토정보플랫폼 정사영상, 2025, 경북동부권역 25cm, 도엽 35804061 (north) and 35804071 (south).
Yeungnam University campus crop used here:
    python tools/prepare_ortho.py crop north.tif south.tif processed 400 1450 1420 2980
    -> campus_25cm.png, 4080 x 6120 px = 1020 m x 1530 m (also campus_50cm.png, campus_preview_2m.jpg)
"""
import json
import os
import sys

import cv2
import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = None
GSD = 0.25        # metres per pixel in the source tiles
PREVIEW = 4       # preview is downsampled by this factor (1 m/px)


def load(path: str, factor: int = 1) -> np.ndarray:
    with Image.open(path) as im:
        im = im.convert('RGB')
        if factor > 1:
            im = im.reduce(factor)
        return np.asarray(im)


def find_offset(north: np.ndarray, south: np.ndarray) -> tuple[int, int]:
    """(dx, dy) of the south tile's top-left corner in north-tile pixels."""
    h, w = north.shape[:2]
    gn = cv2.cvtColor(north, cv2.COLOR_RGB2GRAY)
    gs = cv2.cvtColor(south, cv2.COLOR_RGB2GRAY)
    margin = w // 10
    # Tiles overlap by only ~5 % of their height, so match a thin strip at the very bottom.
    top = h - h // 25
    templ = gn[top: h - h // 200, margin: w - margin]
    search = gs[: h // 4, :]                                     # the south tile's top quarter
    res = cv2.matchTemplate(search, templ, cv2.TM_CCOEFF_NORMED)
    _, score, _, (mx, my) = cv2.minMaxLoc(res)
    dx = margin - mx
    dy = top - my
    print(f'match score {score:.3f}, offset dx={dx}, dy={dy} (preview px)')
    if score < 0.5:
        raise SystemExit('overlap not found reliably; check that the tiles are north/south neighbours')
    return dx, dy


def mosaic(north_path, south_path, out_dir):
    north, south = load(north_path, PREVIEW), load(south_path, PREVIEW)
    dx, dy = find_offset(north, south)
    h, w = north.shape[:2]
    x0 = min(0, dx)
    canvas = np.zeros((dy + south.shape[0], max(w, dx + south.shape[1]) - x0, 3), dtype=np.uint8)
    canvas[dy:, dx - x0: dx - x0 + south.shape[1]] = south
    canvas[:h, -x0: -x0 + w] = north          # north tile on top where they overlap
    os.makedirs(out_dir, exist_ok=True)
    Image.fromarray(canvas).save(os.path.join(out_dir, 'mosaic_1m.jpg'), quality=88)
    info = {'preview_factor': PREVIEW, 'gsd_m': GSD, 'south_offset_preview_px': [dx, dy],
            'north_origin_in_mosaic_preview_px': [-x0, 0], 'mosaic_preview_size': [canvas.shape[1], canvas.shape[0]]}
    with open(os.path.join(out_dir, 'mosaic.json'), 'w') as f:
        json.dump(info, f, indent=2)
    print(f'mosaic {canvas.shape[1]}x{canvas.shape[0]} px at 1 m/px saved in {out_dir}')


def crop(north_path, south_path, out_dir, box):
    with open(os.path.join(out_dir, 'mosaic.json')) as f:
        info = json.load(f)
    k = info['preview_factor']
    nox, _ = info['north_origin_in_mosaic_preview_px']
    sdx, sdy = info['south_offset_preview_px']
    x0, y0, x1, y1 = (v * k for v in box)              # full-resolution mosaic pixels
    out = np.zeros((y1 - y0, x1 - x0, 3), dtype=np.uint8)
    # Paste south first, then north on top (same order as the preview).
    for path, ox, oy in [(south_path, (nox + sdx) * k, sdy * k), (north_path, nox * k, 0)]:
        tile = load(path)
        th, tw = tile.shape[:2]
        ax0, ay0 = max(x0, ox), max(y0, oy)
        ax1, ay1 = min(x1, ox + tw), min(y1, oy + th)
        if ax0 < ax1 and ay0 < ay1:
            out[ay0 - y0: ay1 - y0, ax0 - x0: ax1 - x0] = tile[ay0 - oy: ay1 - oy, ax0 - ox: ax1 - ox]
        del tile
    Image.fromarray(out).save(os.path.join(out_dir, 'campus_25cm.png'), optimize=True)
    small = Image.fromarray(out).reduce(2)
    small.save(os.path.join(out_dir, 'campus_50cm.png'), optimize=True)
    Image.fromarray(out).reduce(8).save(os.path.join(out_dir, 'campus_preview_2m.jpg'), quality=88)
    info['crop_box_preview_px'] = list(box)
    info['crop_size_m'] = [(x1 - x0) * GSD, (y1 - y0) * GSD]
    with open(os.path.join(out_dir, 'mosaic.json'), 'w') as f:
        json.dump(info, f, indent=2)
    print(f'crop {out.shape[1]}x{out.shape[0]} px = {(x1 - x0) * GSD:.0f} m x {(y1 - y0) * GSD:.0f} m')


if __name__ == '__main__':
    cmd, north_p, south_p, out = sys.argv[1:5]
    if cmd == 'mosaic':
        mosaic(north_p, south_p, out)
    elif cmd == 'crop':
        crop(north_p, south_p, out, tuple(int(v) for v in sys.argv[5:9]))
    else:
        raise SystemExit(__doc__)
