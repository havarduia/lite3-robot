#!/usr/bin/env python3
"""Remove "ghosts" - people who walked past - from a saved lio-live map.

A wall stops the laser every time the LiDAR looks at it. A person only blocks
it while they are there; after they have moved on, later scans see straight
THROUGH the cells they occupied, to whatever is behind. So every logged scan
(scans.bin: sensor position + the 10 cm cells it hit) is replayed, and for
each map cell this counts

    hits    scans that hit it
    free    scans with a beam that passed THROUGH it, stopping MARGIN
            (0.3 m + 3% of range) short of where that beam hit

A cell is a ghost when free >= FREE_MIN and, summed over its 30 cm
neighbourhood, free > FREE_RATIO * hits. Guards, each from a failure seen
while building this (walk_20260924_140629 / _165520):
  - real beams, not a direction grid: in 3-degree cells a sloping floor or an
    oblique wall spans several metres of range, and 34% of a map "was seen
    through"
  - a cell next to anything the same scan hit is not seen through by it:
    beams skimming along a wall
  - neighbourhoods, not cells: the MID-360 is sparse, a 10 cm cell of wall
    may be hit once in a whole run; its neighbourhood is hit many times
  - only 0.15-2.1 m above the floor under the nearest pass: where people
    are; floors and ceilings (skimmed at grazing angles) are never touched
Removes people, things moved during the walk, doors opened then shut. Keeps
anything solid for the whole run - including, honestly, someone who walked
beside the robot the whole way, since nothing ever saw past them.
Measured: 0.69% of a 3.7-min robot walk removed, floor-map walls unchanged;
about 45 s per 4 min of walk on the Jetson.

    lio-clean ~/maps/<run>                    clean, then rebuild walls + viewer
    lio-clean ~/maps/<run> --no-rebuild       just the cloud
    lio-clean ~/maps/<run> --dry-run          count, change nothing

Writes slam_output/latest/gravity_aligned.ply (cleaned), keeps the original
as gravity_aligned.uncleaned.ply, and puts what was removed in ghosts.ply.
Needs scans.bin + relevel.npy, which lio_live.py saves since 2026-10-12;
earlier live maps cannot be cleaned (they kept only the merged cells).
"""

import argparse
import glob
import os
import shutil
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from maplib import centres, read_scans, vkeys, write_ply  # noqa: E402

STEP = 0.05           # m between samples along a ray (half a cell)
RAY_EVERY = 2         # follow every 2nd ray of a scan
EVERY = 2             # use every 2nd scan (5 Hz): plenty of views, half the time
MAX_R = 10.0          # m: rays are followed this far
MIN_R = 0.6           # m: the robot's own body
MARGIN = 0.30         # m, + MARGIN_REL * range: "clearly past" where a beam hit
MARGIN_REL = 0.03
FREE_MIN = 3          # a ghost was seen through at least this often...
FREE_RATIO = 2.0      # ...and its neighbourhood more than this many times as often as hit
BAND = (0.15, 2.1)    # m above the floor: where people are
SENSOR_H = 0.46       # m, LiDAR above the floor while walking

# Key offsets of a cell's 26 neighbours (and itself).
_NB = np.array([(dx << 42) + (dy << 21) + dz for dx in (-1, 0, 1)
                for dy in (-1, 0, 1) for dz in (-1, 0, 1)], np.int64)


def members(sorted_keys, k):
    """Bool per k: is it in sorted_keys?"""
    j = np.searchsorted(sorted_keys, k)
    ok = j < len(sorted_keys)
    ok[ok] = sorted_keys[j[ok]] == k[ok]
    return ok


def people_band(keys, scans):
    """Cells BAND m above the floor under the nearest robot pass. Floors and
    ceilings are never candidates: beams skimming along them would otherwise
    "see through" them."""
    from scipy.spatial import cKDTree
    C = centres(keys)
    O = np.array([o for o, _ in scans])
    _, i = cKDTree(np.c_[O[:, :2], O[:, 2] + 1.0]).query(C, k=1, workers=-1)
    rel = C[:, 2] - (O[i, 2] - SENSOR_H)
    return (rel >= BAND[0]) & (rel <= BAND[1])


def seen_through(o, k, cand_keys, steps):
    """Candidate cells that scan (o, k) saw through: on a beam's way to what
    it hit, stopping MARGIN short, and not next to anything this scan hit
    (beams skimming along a wall)."""
    d = centres(k) - o
    r = np.linalg.norm(d, axis=1)
    m = (r > MIN_R + MARGIN) & (np.arange(len(r)) % RAY_EVERY == 0)
    d, r = d[m] / r[m, None], r[m]
    stop = np.minimum(r - MARGIN - MARGIN_REL * r, MAX_R)
    ri, si = np.nonzero(steps[None, :] < stop[:, None])
    q = vkeys(o + d[ri] * steps[si, None])
    # Narrow to candidate cells first (a few thousand of ~300 000 samples),
    # and only then do the costlier neighbour test.
    q = np.unique(q[members(cand_keys, q)])
    near_hit = members(k, (q[:, None] + _NB[None, :]).ravel()).reshape(len(q), -1).any(1)
    return q[~near_hit]


def find_ghosts(scans, keys, cand):
    """Per map cell (keys sorted): how many scans hit it, and how many saw
    through it (counted only where cand)."""
    hits = np.zeros(len(keys), np.int32)
    free = np.zeros(len(keys), np.int32)
    cand_keys = keys[cand]
    steps = np.arange(MIN_R, MAX_R, STEP)
    t0 = time.time()
    views = range(0, len(scans), EVERY)
    for n, s in enumerate(views, 1):
        o, k = scans[s]
        hits[np.searchsorted(keys, k[members(keys, k)])] += 1
        free[np.searchsorted(keys, seen_through(o, k, cand_keys, steps))] += 1
        if n % 100 == 0:
            print(f"\r  {s + 1}/{len(scans)} scans  ({time.time() - t0:.0f} s)", end="", flush=True)
    print(f"\r  {len(views)} views of {len(scans)} scans in {time.time() - t0:.0f} s" + " " * 10)
    return hits, free


def neighbourhood_sum(keys, v):
    """Sum of v over each cell and its 26 neighbours (missing cells = 0)."""
    out = np.zeros(len(keys), np.int64)
    for off in _NB:
        nb = keys + off
        ok = members(keys, nb)
        out[ok] += v[np.searchsorted(keys, nb[ok])]
    return out


def ghosts(scans, keys):
    """Bool per map cell: a ghost? Judged on 30 cm neighbourhoods, not single
    cells: the MID-360's pattern is sparse, so a 10 cm cell of wall may be hit
    only once in the whole run while beams skimming along the wall pass
    through it many times. Summed over its neighbours, a wall has many hits;
    a passer-by's neighbourhood was hit only while they were there."""
    cand = people_band(keys, scans)
    hits, free = find_ghosts(scans, keys, cand)
    hn, fn = neighbourhood_sum(keys, hits), neighbourhood_sum(keys, free)
    return cand & (free >= FREE_MIN) & (fn > FREE_RATIO * hn)


def rebuild(latest):
    """Floor maps and the 3D viewer again, from the cleaned cloud."""
    top = os.path.dirname(os.path.dirname(latest))
    name = os.path.basename(top)
    here = os.path.dirname(os.path.realpath(__file__))
    shutil.rmtree(os.path.join(latest, "walls"), ignore_errors=True)
    subprocess.run(["/usr/bin/python3", os.path.join(here, "map_walls.py"), top])
    subprocess.run(["/usr/bin/python3", os.path.join(here, "map_web.py"), top,
                    "-o", os.path.join(os.path.dirname(top), name + ".html"), "--title", name])


def main():
    sys.stdout.reconfigure(line_buffering=True)   # in order with the rebuild's output
    ap = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                 description=__doc__.split("\n\n")[0])
    ap.add_argument("map_dir")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-rebuild", action="store_true",
                    help="don't re-run map-walls / map-web afterwards")
    a = ap.parse_args()
    found = glob.glob(os.path.join(os.path.expanduser(a.map_dir), "**", "latest"), recursive=True)
    if not found:
        sys.exit("error: no slam_output/latest")
    L = found[0]
    sf, rf = os.path.join(L, "scans.bin"), os.path.join(L, "relevel.npy")
    if not os.path.exists(sf):
        sys.exit("error: no scans.bin - only lio-live runs saved since 2026-10-12 can be cleaned")
    scans = read_scans(sf)
    relevel = np.load(rf) if os.path.exists(rf) else np.eye(3)
    keys = np.unique(np.concatenate([k for _, k in scans]))
    print(f"  {len(scans)} scans, {len(keys):,} map cells")

    ghost = ghosts(scans, keys)
    print(f"  ghosts: {ghost.sum():,} cells ({100 * ghost.mean():.2f}% of the map)")
    if a.dry_run:
        return

    # into the saved map's frame (re-levelled at save), one point per cell
    keep = centres(np.unique(vkeys(centres(keys[~ghost]) @ relevel.T)))
    gone = centres(np.unique(vkeys(centres(keys[ghost]) @ relevel.T)))
    ply = os.path.join(L, "gravity_aligned.ply")
    orig = os.path.join(L, "gravity_aligned.uncleaned.ply")
    if os.path.exists(ply) and not os.path.exists(orig):
        os.rename(ply, orig)
    write_ply(ply, np.column_stack([keep, np.zeros(len(keep))]))
    write_ply(os.path.join(L, "ghosts.ply"), np.column_stack([gone, np.zeros(len(gone))]))
    print(f"  wrote {ply} ({len(keep):,} points); removed points in ghosts.ply; "
          f"original kept as {os.path.basename(orig)}")
    if not a.no_rebuild:
        rebuild(L)


if __name__ == "__main__":
    main()
