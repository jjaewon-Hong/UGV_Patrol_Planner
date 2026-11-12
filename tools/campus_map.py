"""Turn the model's campus prediction into an obstacle grid (FR-20) and score it against OSM (NFR-08).

    python tools/campus_map.py eval <processed_dir> <campus_pred_0.3m.png>
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
    elif cmd == 'grid':
        grid(root, pred_file, *(float(v) for v in sys.argv[4:6]))
    else:
        raise SystemExit(__doc__)
