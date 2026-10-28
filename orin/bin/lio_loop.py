#!/usr/bin/env python3
"""Measure a closed loop's real error: match the last scans against the first.

    lio-loop ~/maps/<run> [--start-s 20 --end-s 20]

If the robot ended where it started, the end scans show the same place as the
start scans; the rigid transform that lines them up IS the accumulated drift
(position and heading). Prints the match quality (share of end points within
0.2 m of the start cloud: ~0.9 = same place, < 0.5 = not a closed loop) and
the error. Pure numpy/scipy ICP: open3d segfaults on this Jetson.
"""
import argparse, glob, os, sys
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from maplib import centres, read_scans  # noqa: E402


def kabsch(A, B):
    ca, cb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


def icp(E, tree, S, R, t):
    for thr in (2.0, 1.0, 0.5, 0.3, 0.2):
        for _ in range(15):
            Q = E @ R.T + t
            d, i = tree.query(Q)
            m = d < thr
            if m.sum() < 100:
                return R, t, 0.0
            dR, dt = kabsch(Q[m], S[i[m]])
            R, t = dR @ R, dR @ t + dt
    d, _ = tree.query(E @ R.T + t)
    return R, t, float((d < 0.2).mean())


def match(scans, a0, a1, b0, b1, r=12.0):
    """Transform taking scans[b0:b1] onto scans[a0:a1]; returns R, t, fit, oa, ob."""
    O = np.array([o for o, _ in scans])
    def cloud(a, b):
        P = centres(np.unique(np.concatenate([scans[i][1] for i in range(a, b)])))
        o = O[a:b].mean(0)
        return P[np.linalg.norm(P - o, axis=1) < r], o
    S, oa = cloud(a0, a1)
    E, ob = cloud(b0, b1)
    tree = cKDTree(S)
    best = None
    for yaw in np.radians(np.arange(-40, 41, 5)):
        R0 = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
        R, t, fit = icp(E, tree, S, R0, oa - R0 @ ob)
        if best is None or fit > best[2]:
            best = (R, t, fit)
    return best[0], best[1], best[2], oa, ob


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map_dir")
    ap.add_argument("--start-s", type=float, default=20)
    ap.add_argument("--end-s", type=float, default=20)
    a = ap.parse_args()
    L = glob.glob(os.path.join(os.path.expanduser(a.map_dir), "**", "latest"), recursive=True)[0]
    scans = read_scans(os.path.join(L, "scans.bin"))
    n = len(scans)
    R, t, fit, oa, ob = match(scans, 0, int(a.start_s * 10), n - int(a.end_s * 10), n)
    err = ob - (R @ ob + t)
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    print(f"match {100 * fit:.0f}% ({'same place' if fit > 0.7 else 'NOT a clear match - was it a closed loop?'})")
    print(f"accumulated error: dx {err[0]:+.2f} dy {err[1]:+.2f} dz {err[2]:+.2f} m "
          f"(xy {np.hypot(*err[:2]):.2f}), heading {-yaw:+.1f} deg")


if __name__ == "__main__":
    main()
