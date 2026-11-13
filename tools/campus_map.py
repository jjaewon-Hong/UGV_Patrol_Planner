"""Turn the model's campus prediction into an obstacle grid (FR-20) and score it against OSM (NFR-08).

    python tools/campus_map.py eval <processed_dir> <campus_pred_0.3m.png>
    python tools/campus_map.py eval-label <processed_dir> <campus_pred_0.3m.png> <labels/val_x_y_size.png>
    python tools/campus_map.py grid <processed_dir> <campus_pred_0.3m.png> [cell_m=10] [ratio=0.3]

The prediction holds LoveDA labels: 1 background, 2 building, 3 road, 4 water, 5 barren, 6 forest, 7 agriculture.
Buildings, water and forest (trees) are obstacles; the rest is drivable.
"""
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from osm_mask import CLASSES, _size, rasterize  # noqa: E402

LOVEDA_TO_OBS = {2: CLASSES['building'], 4: CLASSES['water'], 6: CLASSES['forest']}
EVAL_GSD = 0.5


def load_pred(path: str, size: tuple[int, int]) -> np.ndarray:
    """Prediction resized to `size` and mapped to 0 free / 1 building / 2 water / 3 forest."""
    raw = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    raw = cv2.resize(raw, size, interpolation=cv2.INTER_NEAREST)
    out = np.zeros_like(raw)
    for src, dst in LOVEDA_TO_OBS.items():
        out[raw == src] = dst
    return out


def iou(a: np.ndarray, b: np.ndarray) -> float:
    return float((a & b).sum() / max(1, (a | b).sum()))


def evaluate(d: str, pred_path: str) -> dict:
    size = _size(d, EVAL_GSD)
    gt = rasterize(d, EVAL_GSD, size)
    pred = load_pred(pred_path, size)
    b, w, f = (CLASSES[k] for k in ('building', 'water', 'forest'))
    res = {
        'building': iou(pred == b, gt == b),
        'water': iou(pred == w, gt == w),
        'forest': iou(pred == f, gt == f),
        'building+forest (NFR-08)': iou(np.isin(pred, [b, f]), np.isin(gt, [b, f])),
        'obstacle (all)': iou(pred > 0, gt > 0),
        'pixel accuracy obstacle/free': float(((pred > 0) == (gt > 0)).mean()),
    }
    res = {k: round(v, 3) for k, v in res.items()}
    for k, v in res.items():
        print(f'{k:30s} {v:.3f}')
    # TP green, FP red (model says obstacle, OSM free), FN blue (OSM obstacle, model free)
    img = cv2.imread(os.path.join(d, 'campus_1m.jpg'))
    p1 = cv2.resize((pred > 0).astype(np.uint8), (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
    g1 = cv2.resize((gt > 0).astype(np.uint8), (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
    color = np.zeros_like(img)
    color[p1 & g1] = (0, 200, 0)
    color[p1 & ~g1] = (0, 0, 255)
    color[~p1 & g1] = (255, 0, 0)
    vis = cv2.addWeighted(img, 0.6, color, 0.4, 0)
    cv2.imwrite(os.path.join(d, 'eval_overlay.jpg'), vis, [cv2.IMWRITE_JPEG_QUALITY, 85])
    with open(os.path.join(d, 'eval_iou.json'), 'w') as fp:
        json.dump(res, fp, indent=2)
    return res


def eval_label(d: str, pred_path: str, label_path: str) -> dict:
    """Score the model AND OpenStreetMap against a hand-labelled area (tools/label_tool.py).

    Label file name: val_<x0>_<y0>_<size>.png (metres in the campus image), 0.25 m/px,
    0 none / 1 building / 2 tree / 3 water.
    """
    x0, y0, size = (int(v) for v in os.path.splitext(os.path.basename(label_path))[0].split('_')[1:4])
    label = cv2.imread(label_path, cv2.IMREAD_GRAYSCALE)
    h, w = label.shape
    gsd = size / w
    # model: LoveDA labels on the 0.3 m grid -> crop the same metres -> label grid
    raw = cv2.imread(pred_path, cv2.IMREAD_GRAYSCALE)
    k = raw.shape[1] / _size(d, 1.0)[0]                       # pred px per metre
    crop = raw[int(round(y0 * k)):int(round((y0 + size) * k)), int(round(x0 * k)):int(round((x0 + size) * k))]
    crop = cv2.resize(crop, (w, h), interpolation=cv2.INTER_NEAREST)
    model = np.zeros_like(label)
    model[crop == 2], model[crop == 6], model[crop == 4] = 1, 2, 3
    # OSM: 1 building / 2 water / 3 forest -> 1 building / 2 tree / 3 water
    full = rasterize(d, gsd, _size(d, gsd))
    o = full[int(round(y0 / gsd)):int(round(y0 / gsd)) + h, int(round(x0 / gsd)):int(round(x0 / gsd)) + w]
    osm = np.zeros_like(label)
    osm[o == CLASSES['building']], osm[o == CLASSES['forest']], osm[o == CLASSES['water']] = 1, 2, 3

    def scores(p):
        r = {'building': iou(p == 1, label == 1), 'tree': iou(p == 2, label == 2)}
        if (label == 3).any() or (p == 3).any():
            r['water'] = iou(p == 3, label == 3)
        r['building+tree (NFR-08)'] = iou(np.isin(p, [1, 2]), np.isin(label, [1, 2]))
        r['obstacle (all)'] = iou(p > 0, label > 0)
        return {key: round(v, 3) for key, v in r.items()}

    res = {'area_m': [x0, y0, size], 'label_share': {n: round(float((label == c).mean()), 3)
                                                     for n, c in (('building', 1), ('tree', 2), ('water', 3))},
           'model': scores(model), 'osm': scores(osm)}
    print(f"area x{x0} y{y0} {size} m, label share {res['label_share']}")
    print(f"{'metric':26s} {'model':>7s} {'OSM':>7s}")
    for key in res['model']:
        print(f"{key:26s} {res['model'][key]:7.3f} {res['osm'].get(key, float('nan')):7.3f}")
    with open(os.path.join(d, 'eval_label.json'), 'w') as fp:
        json.dump(res, fp, indent=2)
    return res


def grid(d: str, pred_path: str, cell_m: float = 10.0, ratio: float = 0.3) -> np.ndarray:
    """Cell is an obstacle when at least `ratio` of its area is building, water or forest."""
    size = _size(d, EVAL_GSD)
    obs = (load_pred(pred_path, size) > 0).astype(np.float32)
    k = int(round(cell_m / EVAL_GSD))
    rows, cols = obs.shape[0] // k, obs.shape[1] // k
    frac = obs[:rows * k, :cols * k].reshape(rows, k, cols, k).mean(axis=(1, 3))
    occ = frac >= ratio
    np.save(os.path.join(d, f'campus_obstacles_{cell_m:g}m.npy'), occ)
    preview = cv2.resize(np.where(occ, 0, 255).astype(np.uint8), (cols * 4, rows * 4), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(os.path.join(d, f'campus_obstacles_{cell_m:g}m.png'), preview)
    print(f'grid {rows}x{cols} cells of {cell_m:g} m, obstacle share {occ.mean():.2f}')
    return occ


if __name__ == '__main__':
    cmd, root, pred_file = sys.argv[1:4]
    if cmd == 'eval':
        evaluate(root, pred_file)
    elif cmd == 'eval-label':
        eval_label(root, pred_file, sys.argv[4])
    elif cmd == 'grid':
        grid(root, pred_file, *(float(v) for v in sys.argv[4:6]))
    else:
        raise SystemExit(__doc__)
