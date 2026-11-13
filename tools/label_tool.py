"""Hand-label a validation area of the campus photo: buildings, trees, water (ground truth for NFR-08).

    python tools/label_tool.py <processed_dir> [x0_m=250 y0_m=780 size_m=200]

Shows only the photo (no model output, no OSM) so the labels are independent of both.
Saves labels/val_<x0>_<y0>_<size>.png at 0.25 m/px: 0 none, 1 building, 2 tree, 3 water.

Mouse   left drag: paint     right drag: erase
Keys    1 building  2 tree  3 water  0 eraser
        p  polygon mode on/off (click corners, Enter = fill, Backspace = undo corner, Esc = cancel)
        [ ]  brush smaller / larger     t  hide/show labels     z  undo
        s  save                          q  save and quit
"""
import os
import sys

import cv2
import numpy as np

NONE, BUILDING, TREE, WATER = 0, 1, 2, 3
NAMES = {BUILDING: 'building', TREE: 'tree', WATER: 'water', NONE: 'eraser'}
COLORS = {BUILDING: (40, 40, 230), TREE: (40, 200, 40), WATER: (230, 120, 20)}  # BGR
GSD = 0.25
BAR = 30  # status bar height in window pixels


class Labeler:
    """Label editing without any GUI, so it can be tested."""

    def __init__(self, image: np.ndarray, label: np.ndarray | None = None):
        self.image = image
        self.label = np.zeros(image.shape[:2], np.uint8) if label is None else label.copy()
        self.cls = BUILDING
        self.brush = 8                # px radius (2 m)
        self.polygon: list[tuple[int, int]] = []
        self.history: list[np.ndarray] = []

    def checkpoint(self):
        self.history.append(self.label.copy())
        del self.history[:-30]

    def undo(self):
        if self.history:
            self.label = self.history.pop()

    def paint(self, x: int, y: int, cls: int | None = None):
        cv2.circle(self.label, (x, y), self.brush, int(self.cls if cls is None else cls), -1)

    def stroke(self, a: tuple[int, int], b: tuple[int, int], cls: int | None = None):
        cv2.line(self.label, a, b, int(self.cls if cls is None else cls), self.brush * 2)
        self.paint(*b, cls=cls)

    def fill_polygon(self) -> bool:
        """Fill the polygon with the current class, but only where nothing is labelled yet,
        so a rough tree outline drawn over a building keeps the building.
        With the eraser (0) it clears everything inside."""
        if len(self.polygon) < 3:
            return False
        self.checkpoint()
        inside = np.zeros_like(self.label)
        cv2.fillPoly(inside, [np.array(self.polygon, np.int32)], 1)
        target = inside.astype(bool) if self.cls == NONE else inside.astype(bool) & (self.label == NONE)
        self.label[target] = self.cls
        self.polygon = []
        return True

    def render(self, show_labels: bool = True, alpha: float = 0.45) -> np.ndarray:
        out = self.image.copy()
        if show_labels:
            color = np.zeros_like(out)
            for cls, bgr in COLORS.items():
                color[self.label == cls] = bgr
            m = self.label > 0
            out[m] = (out[m] * (1 - alpha) + color[m] * alpha).astype(np.uint8)
        if self.polygon:
            pts = np.array(self.polygon, np.int32)
            cv2.polylines(out, [pts], False, (0, 255, 255), 1)
            for p in self.polygon:
                cv2.circle(out, p, 3, (0, 255, 255), -1)
        return out

    def counts(self) -> dict:
        return {NAMES[c]: round(float((self.label == c).mean()), 3) for c in (BUILDING, TREE, WATER)}


def run(d: str, x0: int, y0: int, size: int) -> None:
    img = cv2.imread(os.path.join(d, 'campus_25cm.png'))
    px = int(round(1 / GSD))
    crop = img[y0 * px:(y0 + size) * px, x0 * px:(x0 + size) * px].copy()
    del img
    out_dir = os.path.join(d, 'labels')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f'val_{x0}_{y0}_{size}.png')
    lab = Labeler(crop, cv2.imread(path, cv2.IMREAD_GRAYSCALE) if os.path.exists(path) else None)

    state = {'down': None, 'last': None, 'show': True, 'poly': False}
    win = 'label tool  (1 building, 2 tree, 3 water, 0 eraser, p polygon, z undo, s save, q quit)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, min(crop.shape[1], 1000), min(crop.shape[0], 1000) + BAR)

    def on_mouse(event, x, y, flags, _):
        y -= BAR  # the status bar sits above the image in the window
        if y < 0 and event in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN):
            return
        y = max(y, 0)
        if state['poly']:
            if event == cv2.EVENT_LBUTTONDOWN:
                lab.polygon.append((x, y))
            return
        if event in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN):
            lab.checkpoint()
            state['down'] = NONE if event == cv2.EVENT_RBUTTONDOWN else lab.cls
            state['last'] = (x, y)
            lab.paint(x, y, state['down'])
        elif event == cv2.EVENT_MOUSEMOVE and state['down'] is not None:
            lab.stroke(state['last'], (x, y), state['down'])
            state['last'] = (x, y)
        elif event in (cv2.EVENT_LBUTTONUP, cv2.EVENT_RBUTTONUP):
            state['down'] = None

    cv2.setMouseCallback(win, on_mouse)

    def save():
        cv2.imwrite(path, lab.label)
        print('saved', path, lab.counts(), flush=True)

    while True:
        view = lab.render(state['show'])
        bar = np.full((BAR, view.shape[1], 3), 30, np.uint8)
        mode = 'POLYGON (click corners, Enter fill)' if state['poly'] else f'brush {lab.brush * GSD:.1f} m'
        cv2.putText(bar, f'class: {NAMES[lab.cls]}   {mode}   labels {"on" if state["show"] else "off"}',
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow(win, np.vstack([bar, view]))
        k = cv2.waitKey(20) & 0xFF
        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:   # window closed with the X button
            save()
            break
        if k == 255:
            continue
        ch = chr(k)
        if ch in '0123':
            lab.cls = int(ch)
        elif ch == 'p':
            state['poly'] = not state['poly']
            lab.polygon = []
        elif k in (13, 32) and state['poly']:
            lab.fill_polygon()
        elif k == 8 and lab.polygon:
            lab.polygon.pop()
        elif k == 27:
            lab.polygon = []
        elif ch == '[':
            lab.brush = max(1, lab.brush - 2)
        elif ch == ']':
            lab.brush = min(80, lab.brush + 2)
        elif ch == 't':
            state['show'] = not state['show']
        elif ch == 'z':
            lab.undo()
        elif ch == 's':
            save()
        elif ch == 'q':
            save()
            break
    cv2.destroyAllWindows()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    args = [int(v) for v in sys.argv[2:5]]
    run(sys.argv[1], *(args + [250, 780, 200][len(args):]))
