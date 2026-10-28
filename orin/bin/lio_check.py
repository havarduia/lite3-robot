#!/usr/bin/env python3
"""Score saved maps for drift and divergence (kiss-live or lio-live layout).

    lio-check ~/maps/<run> [...]

jumps      odometry steps over 0.5 m between consecutive updates: FAST-LIO
           "shooting away" (walking is ~5-15 cm per update)
tilt       SLAM attitude vs the accelerometer's gravity (5 s average), deg
floor      p90 of |path climb angle - slope of the floor fitted under it|:
           real slopes and stairs agree, SLAM pitch error climbs over flat floor
gap        end minus start - the drift, if the walk was a closed loop
"""
import glob, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from maplib import read_ply


def floor_p90(pts, path):
    c = np.floor(pts[:, :2]).astype(np.int64); key = c[:, 0] * 100000 + c[:, 1]
    o = np.argsort(key); key = key[o]; pts = pts[o]
    def near(x, y, r=1.5):
        out = []
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                k = (int(np.floor(x)) + dx) * 100000 + int(np.floor(y)) + dy
                a, b = np.searchsorted(key, k), np.searchsorted(key, k, "right"); out.append(pts[a:b])
        q = np.vstack(out); return q[np.hypot(q[:, 0] - x, q[:, 1] - y) < r]
    E = []
    for i in range(0, len(path) - 30, 10):
        x, y, z = path[i]; q = near(x, y); q = q[(q[:, 2] < z - 0.15) & (q[:, 2] > z - 1.0)]
        if len(q) < 30: continue
        h = np.histogram(q[:, 2], np.arange(z - 1.0, z - 0.1, 0.1)); zc = h[1][np.argmax(h[0])] + 0.05
        q = q[np.abs(q[:, 2] - zc) < 0.2]
        if len(q) < 30: continue
        for _ in range(3):
            A = np.c_[q[:, 0] - x, q[:, 1] - y, np.ones(len(q))]
            w, *_ = np.linalg.lstsq(A, q[:, 2], rcond=None); res = q[:, 2] - A @ w
            q = q[np.abs(res) < max(0.05, 2 * np.std(res))]
            if len(q) < 30: break
        if len(q) < 30 or np.std(res) > 0.08: continue
        j = min(len(path) - 1, i + 20); d = path[j] - path[i]; L = np.hypot(d[0], d[1])
        if L < 1.0: continue
        u = d[:2] / L
        E.append(abs(np.degrees(np.arctan2(d[2], L)) - np.degrees(np.arctan(w[0] * u[0] + w[1] * u[1]))))
    return (np.percentile(E, 90), len(E)) if E else (float("nan"), 0)


for arg in sys.argv[1:]:
    for L in glob.glob(os.path.join(os.path.expanduser(arg), "**", "slam_output", "latest"), recursive=True):
        ply = os.path.join(L, "gravity_aligned.unflattened.ply")
        ply = ply if os.path.exists(ply) else os.path.join(L, "gravity_aligned.ply")
        pf = os.path.join(L, "gravity_aligned_path.npy")
        pf = pf + ".unflattened.npy" if os.path.exists(pf + ".unflattened.npy") else pf
        p = np.load(pf).astype(np.float64)
        d = np.linalg.norm(np.diff(p, axis=0), axis=1)
        jumps = np.flatnonzero(d > 0.5)
        t = os.path.join(L, "imu_tilt.npy")
        e = np.load(t)[:, 1] if os.path.exists(t) else np.array([np.nan])
        pts, _ = read_ply(ply)
        f90, nf = floor_p90(pts[:, :3].astype(np.float64), p)
        name = os.path.relpath(L, os.path.expanduser(arg)).split(os.sep)[0]
        print(f"{os.path.basename(os.path.normpath(arg))}{'/' + name if name != 'slam_output' else ''}")
        print(f"  {len(p)} poses (~{len(p)/10:.0f} s), path {d.sum():.0f} m | jumps >0.5 m: {len(jumps)}"
              + (f" (first at {jumps[0]/10:.0f} s, biggest {d.max():.1f} m)" if len(jumps) else ""))
        print(f"  tilt vs gravity p50 {np.nanmedian(e):.1f}, p95 {np.nanpercentile(e, 95):.1f}, max {np.nanmax(e):.1f} deg"
              f" | floor p90 {f90:.1f} deg ({nf} patches)")
        print(f"  end - start: xy {np.hypot(*(p[-1, :2] - p[0, :2])):.2f} m, z {p[-1, 2] - p[0, 2]:+.2f} m"
              f" ({100 * np.hypot(*(p[-1, :2] - p[0, :2])) / max(d.sum(), 1):.1f}% of path)")
