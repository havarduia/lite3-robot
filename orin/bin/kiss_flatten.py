#!/usr/bin/env python3
"""Undo SLAM height drift in a saved map of ONE level floor.

KISS-SLAM's pitch can wander several degrees with no IMU in the loop, and a
pitch error turns forward walking into climbing. live_20261007_134448 (12 min,
flat ground) gained up to 16 m of fake height: the floor fitted under the path
was level (+1 deg) where the path itself "climbed" at 17 deg. map-walls then
split the one floor into four storeys and the Walls layer fell apart.

When the ground really is one level, every scan was taken from the same
height, so the path's height IS the drift. Each map point is moved down by the
drift of the pass that most plausibly saw it (nearest path point in 3D, with
the point compared at the sensor's typical viewing height, so stacked passes
over the same spot keep their own corrections).

    kiss-flatten ~/maps/live_20261007_134448      then map-walls / map-web

Keeps the original as gravity_aligned.unflattened.ply. Do NOT use on maps
with stairs or ramps: real height changes would be flattened too. Horizontal
drift is untouched - that needs loop closures.
"""
import argparse
import glob
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from maplib import read_ply, write_ply  # noqa: E402

VIEW_DZ = 1.0     # m: points sit ~0.5 m below to ~2.5 m above the sensor


def floor_share(z):
    c = np.bincount(((z - z.min()) / 0.05).astype(np.int64))
    return c.max() / len(z)


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__.split("\n\n")[0])
    ap.add_argument("map_dir")
    ap.add_argument("--smooth", type=float, default=2.0,
                    help="metres of path over which drift is averaged")
    a = ap.parse_args()

    d = glob.glob(os.path.join(os.path.expanduser(a.map_dir), "**", "latest"),
                  recursive=True)
    if not d:
        sys.exit("error: no slam_output/latest")
    d = d[0]
    ply = os.path.join(d, "gravity_aligned.ply")
    orig = os.path.join(d, "gravity_aligned.unflattened.ply")
    pf = os.path.join(d, "gravity_aligned_path.npy")
    if not os.path.exists(pf):
        sys.exit("error: no gravity_aligned_path.npy (a kiss-live or kiss-gravity map)")
    if not os.path.exists(orig):
        os.rename(ply, orig)            # first run: keep the original
    pts, inten = read_ply(orig)
    pts = pts[:, :3].astype(np.float64)
    path = np.load(pf).astype(np.float64)
    if not os.path.exists(pf + ".unflattened.npy"):
        np.save(pf + ".unflattened.npy", path)
    else:
        path = np.load(pf + ".unflattened.npy")

    # drift along the path, smoothed over a few metres of walking
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    grid = np.arange(0, s[-1] + a.smooth, a.smooth / 4)
    zg = np.interp(grid, s, path[:, 2])
    k = np.ones(4) / 4
    zg = np.convolve(np.pad(zg, 2, mode="edge"), k, "same")[2:-2]
    drift = np.interp(s, grid, zg) - path[0, 2]

    tree = cKDTree(np.c_[path[:, :2], path[:, 2] + VIEW_DZ])
    _, idx = tree.query(pts, k=1, workers=-1)
    before = floor_share(pts[:, 2])
    pts[:, 2] -= drift[idx]
    after = floor_share(pts[:, 2])
    flat = path.copy()
    flat[:, 2] -= drift

    write_ply(ply, np.column_stack([pts, inten if inten is not None
                                    else np.zeros(len(pts))]))
    np.save(pf, flat.astype(np.float32))
    print(f"  {len(pts):,} points; path drift removed: "
          f"{drift.min():+.1f} .. {drift.max():+.1f} m")
    print(f"  vertical extent {np.ptp(np.load(pf + '.unflattened.npy')[:, 2]):.1f} m "
          f"of path -> {np.ptp(flat[:, 2]):.2f} m")
    print(f"  densest 5 cm floor slab: {before*100:.1f}% -> {after*100:.1f}% of points")
    print(f"  wrote {ply} (original kept as {os.path.basename(orig)})")


if __name__ == "__main__":
    main()
