"""Georeferencing helpers used to place the campus orthophoto (tools/georef.py)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import georef  # noqa: E402


def test_origin_of_east_belt_maps_to_false_origin():
    assert georef.to_tm(38.0, 129.0) == pytest.approx((200000.0, 600000.0), abs=1e-6)


@pytest.mark.parametrize('lat,lon', [(35.83, 128.76), (36.5, 129.4), (34.9, 128.1)])
def test_tm_round_trip_within_a_millimetre(lat, lon):
    e, n = georef.to_tm(lat, lon)
    back = georef.from_tm(e, n)
    assert back == pytest.approx((lat, lon), abs=1e-8)


def test_sheet_bounds_and_neighbour():
    s, n, w, e = georef.sheet_bounds(35804061)
    assert (s, n, w, e) == pytest.approx((35.825, 35.85, 128.75, 128.775))
    s2, n2, w2, e2 = georef.sheet_bounds(35804071)        # the sheet directly south
    assert n2 == pytest.approx(s) and w2 == pytest.approx(w)


def test_campus_pixel_round_trip():
    geo = georef.CampusGeoref({'top_left_tm': [177737.76, 360048.17], 'shift_m': [-13.0, 4.5]})
    e, n = geo.to_tm(1000, 2000)
    assert (e, n) == pytest.approx((177737.76 - 13 + 250, 360048.17 + 4.5 - 500))
    assert geo.to_px(e, n) == pytest.approx((1000, 2000))
