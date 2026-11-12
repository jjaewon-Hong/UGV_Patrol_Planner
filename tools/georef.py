"""Georeferencing for the campus orthophoto crop.

The NGII tiles carry no georeference tags. Their position is estimated from the map sheet number
(1:5000 도엽) and then corrected by a shift found against OpenStreetMap buildings (osm_mask.py align).

Coordinate system: Korea 2000 / East Belt 2010 (EPSG:5187), TM on GRS80,
lat0 38N, lon0 129E, scale 1, false easting 200 000 m, false northing 600 000 m.

    python tools/georef.py init <processed_dir>          # write campus_georef.json from sheet numbers
    python tools/osm_mask.py fetch <processed_dir>       # OSM buildings / water / forest
    python tools/osm_mask.py align <processed_dir>       # correct the position against OSM building outlines

Result for the Yeungnam campus crop (1020 m x 1530 m): top-left from sheet centre (177737.76, 360048.17),
OSM correction -13.0 m east, +4.5 m north.
"""
import json
import math
import os
import sys

A = 6378137.0
F = 1 / 298.257222101
E2 = F * (2 - F)
EP2 = E2 / (1 - E2)
LAT0, LON0, FE, FN, K0 = 38.0, 129.0, 200000.0, 600000.0, 1.0

NORTH_SHEET = 35804061
TILE_PX = (9636, 11664)   # width, height of every tile at 0.25 m


def _meridian(phi: float) -> float:
    e4, e6 = E2 * E2, E2 ** 3
    return A * ((1 - E2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * phi
                - (3 * E2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * math.sin(2 * phi)
                + (15 * e4 / 256 + 45 * e6 / 1024) * math.sin(4 * phi)
                - (35 * e6 / 3072) * math.sin(6 * phi))


def to_tm(lat: float, lon: float) -> tuple[float, float]:
    """WGS84/GRS80 degrees -> (easting, northing) in metres."""
    phi, lam = math.radians(lat), math.radians(lon - LON0)
    n = A / math.sqrt(1 - E2 * math.sin(phi) ** 2)
    t = math.tan(phi) ** 2
    c = EP2 * math.cos(phi) ** 2
    a = lam * math.cos(phi)
    x = K0 * n * (a + (1 - t + c) * a ** 3 / 6 + (5 - 18 * t + t * t + 72 * c - 58 * EP2) * a ** 5 / 120)
    y = K0 * (_meridian(phi) - _meridian(math.radians(LAT0)) + n * math.tan(phi) * (
        a * a / 2 + (5 - t + 9 * c + 4 * c * c) * a ** 4 / 24
        + (61 - 58 * t + t * t + 600 * c - 330 * EP2) * a ** 6 / 720))
    return FE + x, FN + y


def from_tm(e: float, n: float) -> tuple[float, float]:
    """(easting, northing) -> (lat, lon) in degrees."""
    m = _meridian(math.radians(LAT0)) + (n - FN) / K0
    mu = m / (A * (1 - E2 / 4 - 3 * E2 ** 2 / 64 - 5 * E2 ** 3 / 256))
    e1 = (1 - math.sqrt(1 - E2)) / (1 + math.sqrt(1 - E2))
    phi1 = (mu + (3 * e1 / 2 - 27 * e1 ** 3 / 32) * math.sin(2 * mu)
            + (21 * e1 ** 2 / 16 - 55 * e1 ** 4 / 32) * math.sin(4 * mu)
            + (151 * e1 ** 3 / 96) * math.sin(6 * mu) + (1097 * e1 ** 4 / 512) * math.sin(8 * mu))
    n1 = A / math.sqrt(1 - E2 * math.sin(phi1) ** 2)
    t1 = math.tan(phi1) ** 2
    c1 = EP2 * math.cos(phi1) ** 2
    r1 = A * (1 - E2) / (1 - E2 * math.sin(phi1) ** 2) ** 1.5
    d = (e - FE) / (n1 * K0)
    lat = phi1 - (n1 * math.tan(phi1) / r1) * (
        d * d / 2 - (5 + 3 * t1 + 10 * c1 - 4 * c1 * c1 - 9 * EP2) * d ** 4 / 24
        + (61 + 90 * t1 + 298 * c1 + 45 * t1 * t1 - 252 * EP2 - 3 * c1 * c1) * d ** 6 / 720)
    lon = (d - (1 + 2 * t1 + c1) * d ** 3 / 6
           + (5 - 2 * c1 + 28 * t1 - 3 * c1 * c1 + 8 * EP2 + 24 * t1 * t1) * d ** 5 / 120) / math.cos(phi1)
    return math.degrees(lat), LON0 + math.degrees(lon)


def sheet_bounds(num: int) -> tuple[float, float, float, float]:
    """(south, north, west, east) in degrees of a 1:5000 sheet such as 35804061.

    Digits: lat (2) | last digit of lon (1) | 1:50000 sheet in the 1x1 degree block, 4x4 from NW (2)
            | 1:5000 sheet in that 15' block, 10x10 from NW (3).
    """
    s = str(num)
    lat, lon = int(s[:2]), 120 + int(s[2])
    k50, k5 = int(s[3:5]) - 1, int(s[5:8]) - 1
    top = lat + 1 - (k50 // 4) * 0.25 - (k5 // 10) * 0.025
    left = lon + (k50 % 4) * 0.25 + (k5 % 10) * 0.025
    return top - 0.025, top, left, left + 0.025


def sheet_center_tm(num: int) -> tuple[float, float]:
    s, n, w, e = sheet_bounds(num)
    pts = [to_tm(la, lo) for la, lo in ((n, w), (n, e), (s, w), (s, e))]
    return sum(p[0] for p in pts) / 4, sum(p[1] for p in pts) / 4


class CampusGeoref:
    """Pixel (col, row) of a campus image <-> TM metres. Rows grow southwards."""

    def __init__(self, info: dict, gsd: float = 0.25):
        self.left = info['top_left_tm'][0] + info.get('shift_m', [0, 0])[0]
        self.top = info['top_left_tm'][1] + info.get('shift_m', [0, 0])[1]
        self.gsd = gsd

    @classmethod
    def load(cls, path: str, gsd: float = 0.25) -> 'CampusGeoref':
        with open(path) as f:
            return cls(json.load(f), gsd)

    def to_px(self, e: float, n: float) -> tuple[float, float]:
        return (e - self.left) / self.gsd, (self.top - n) / self.gsd

    def to_tm(self, col: float, row: float) -> tuple[float, float]:
        return self.left + col * self.gsd, self.top - row * self.gsd


def init(processed_dir: str) -> None:
    """Assume the north tile is centred on its sheet, then place the crop inside it."""
    with open(os.path.join(processed_dir, 'mosaic.json')) as f:
        mos = json.load(f)
    k, gsd = mos['preview_factor'], mos['gsd_m']
    cx, cy = sheet_center_tm(NORTH_SHEET)
    tile_left = cx - TILE_PX[0] * gsd / 2
    tile_top = cy + TILE_PX[1] * gsd / 2
    nox, _ = mos['north_origin_in_mosaic_preview_px']
    x0, y0, x1, y1 = mos['crop_box_preview_px']
    left = tile_left + (x0 - nox) * k * gsd
    top = tile_top - y0 * k * gsd
    w, h = (x1 - x0) * k * gsd, (y1 - y0) * k * gsd
    corners = [from_tm(left, top), from_tm(left + w, top - h)]
    info = {'crs': 'EPSG:5187', 'top_left_tm': [round(left, 2), round(top, 2)], 'shift_m': [0.0, 0.0],
            'size_m': [w, h], 'note': 'top_left from sheet centre; shift_m from OSM alignment',
            'latlon_nw_se': [[round(v, 6) for v in c] for c in corners]}
    out = os.path.join(processed_dir, 'campus_georef.json')
    with open(out, 'w') as f:
        json.dump(info, f, indent=2)
    print(json.dumps(info, indent=2))


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[1] == 'init':
        init(sys.argv[2])
    else:
        raise SystemExit(__doc__)
