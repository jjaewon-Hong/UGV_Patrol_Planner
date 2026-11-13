"""Make a synthetic N x N elevation map for testing (stand-in until real SRTM data is prepared).

Hills give gentle slopes; a cliff band in the middle is steeper than 30 deg, with two gaps,
so the planner has to detour through a gap.

Usage: python tools/make_synthetic_dem.py [out.npy] [N]
"""
import sys

import numpy as np


def make_dem(n: int = 100, seed: int = 7) -> np.ndarray:
    rng = np.random.default_rng(seed)
    r, c = np.mgrid[0:n, 0:n].astype(float)
    z = np.zeros((n, n))
    for _ in range(8):  # gentle hills: 30~90 m high, ~10 cells wide
        cr, cc = rng.uniform(0, n, 2)
        h, s = rng.uniform(30, 90), rng.uniform(8, 14)
        z += h * np.exp(-((r - cr) ** 2 + (c - cc) ** 2) / (2 * s ** 2))
    # Cliff: 60 m step across columns 49~50 for rows 20~79, except two gaps.
    step = 60.0 * (c >= 50 * n / 100)
    band = (r >= 0.2 * n) & (r < 0.8 * n)
    gaps = ((r >= 0.35 * n) & (r < 0.40 * n)) | ((r >= 0.65 * n) & (r < 0.70 * n))
    z += np.where(band & ~gaps, step, 0.0)
    # Outside the band (and in the gaps) the extra height ramps up gently instead.
    ramp = 60.0 * np.clip((c - 0.3 * n) / (0.4 * n), 0, 1)
    z += np.where(band & ~gaps, 0.0, ramp)
    return z.round(2)


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else 'share/data/dem_100x100.npy'
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 100
    dem = make_dem(n)
    np.save(out, dem)
    print(f'saved {out}: {dem.shape}, elevation {dem.min():.1f} ~ {dem.max():.1f} m')
