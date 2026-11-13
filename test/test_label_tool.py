"""Label editing of tools/label_tool.py (no GUI)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
from label_tool import BUILDING, NONE, TREE, Labeler  # noqa: E402


def make():
    return Labeler(np.zeros((100, 100, 3), np.uint8))


def test_paint_erase_and_undo():
    lab = make()
    lab.cls, lab.brush = TREE, 3
    lab.checkpoint()
    lab.stroke((10, 10), (40, 10))
    assert lab.label[10, 25] == TREE and lab.label[50, 50] == NONE
    lab.checkpoint()
    lab.paint(25, 10, cls=NONE)                      # right-drag erase
    assert lab.label[10, 25] == NONE
    lab.undo()
    assert lab.label[10, 25] == TREE
    lab.undo()
    assert not lab.label.any()


def test_polygon_fill_needs_three_corners():
    lab = make()
    lab.cls = BUILDING
    lab.polygon = [(10, 10), (60, 10)]
    assert not lab.fill_polygon()
    lab.polygon.append((60, 60))
    lab.polygon.append((10, 60))
    assert lab.fill_polygon()
    assert lab.label[30, 30] == BUILDING and lab.label[80, 80] == NONE
    assert lab.polygon == []
    assert abs(lab.counts()['building'] - 0.25) < 0.02


def test_polygon_fill_keeps_existing_labels():
    lab = make()
    lab.cls = BUILDING
    lab.polygon = [(40, 40), (60, 40), (60, 60), (40, 60)]
    lab.fill_polygon()
    lab.cls = TREE                                    # rough tree outline drawn over the building
    lab.polygon = [(0, 0), (99, 0), (99, 99), (0, 99)]
    lab.fill_polygon()
    assert lab.label[50, 50] == BUILDING and lab.label[10, 10] == TREE
    lab.cls = NONE                                    # eraser polygon clears everything inside
    lab.polygon = [(30, 30), (70, 30), (70, 70), (30, 70)]
    lab.fill_polygon()
    assert lab.label[50, 50] == NONE and lab.label[10, 10] == TREE
