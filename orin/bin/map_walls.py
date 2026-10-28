#!/usr/bin/env python3
"""Pull the walls out of a finished map and write a clean 2D floor plan.

The kiss-slam occupancy grid slices 0.1-0.5 m above the floor, which is where
chairs, table legs and clutter live, so walls drown in furniture. This tool
looks at how TALL the structure in each 5 cm cell is instead: a wall fills a
column from floor to ceiling, a table fills one thin slab. No floor height is
needed, which matters because the floor in these maps is not flat - residual
drift spreads it over ~1 m of z across a walk, so any fixed z band cuts
through furniture in one room and misses walls in the next.

    map-walls ~/maps/walk_20260921_140438
    map-walls ~/maps/walk_x --min-height 1.2     stricter: fewer cabinets
    map-walls --selftest

Multi-storey maps are split into storeys first (see floors()), one plan each.

Writes <slam_output>/latest/walls/, per storey N counted from the bottom:
    floorN.pgm + floorN.yaml  Nav2 map_server format (black wall, white free,
                              grey unknown); in the gravity_aligned.ply frame,
                              NOT the frame of occupancy_grid/map2d
    floorN.png                same image, for a quick look
"""
import argparse, glob, os, sys
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from maplib import read_ply

WALL, FREE, UNKNOWN = 0, 254, 205     # map_server trinary values


def wall_grid(pts, res=0.05, slab=0.10, min_height=1.0, min_len=0.5, max_gap=None):
    """Return (image uint8, origin_xy) for points Nx3.

    min_height: metres of occupied column that make a cell a wall. Furniture
    tops out ~0.8-1.0 m; tall cabinets and shelves will pass, and should - they
    block the robot just the same.
    min_len: wall pieces whose longest side is shorter than this many metres
    are dropped as noise (people walking past, reflections). Length, not area:
    a single static scan sees walls as 1-cell-thin lines, and an area limit
    threw most of them away.
    max_gap: a column may miss up to this much (m) and still count as one.
    The MID-360's pattern is sparse, so a real wall's column often lacks a
    slab or two; demanding an unbroken 1 m left corridor walls dashed on
    long-walk-recovered_20261012_150849 (+23% wall cells at 0.2 m, with no
    thickening). Floor and ceiling (~2.3 m apart) or floor and a table top
    (~0.75 m) are still far more than this apart. Default: 0.2 for the
    10 cm clouds FAST-LIO maps are (sparse), 0 for dense 5 cm clouds - old
    KISS maps stack drifted copies of a floor a few cm apart, and bridging
    those nearly tripled their "walls".
    """
    if max_gap is None:
        max_gap = 0.2 if res >= 0.1 - 1e-9 else 0.0
    xy0 = pts[:, :2].min(0)
    # +1e-3 cell: kiss-live saves points exactly on a grid, where float32
    # rounding would otherwise split one column across two cells or slabs.
    ij = np.floor((pts[:, :2] - xy0) / res + 1e-3).astype(np.int64)
    k = np.floor((pts[:, 2] - pts[:, 2].min()) / slab + 1e-3).astype(np.int64)
    W, H = ij.max(0) + 1
    cell = ij[:, 1] * W + ij[:, 0]
    # Distinct (cell, slab) pairs, sorted by cell then slab. Height is the
    # longest run of slabs with no gap over max_gap: a floor plus a ceiling,
    # or a floor plus a table top, are separate runs and must not add up to
    # a wall.
    K = k.max() + 1
    key = np.unique(cell * K + k)
    c, s = key // K, key % K
    gapk = int(round(max_gap / slab))
    new = np.r_[True, (c[1:] != c[:-1]) | (s[1:] - s[:-1] > gapk + 1)]
    start = np.maximum.accumulate(np.where(new, np.arange(len(key)), 0))
    run = s - s[start] + 1
    height = np.zeros(W * H)
    np.maximum.at(height, c, run * slab)
    height = height.reshape(H, W)

    wall = (height >= min_height).astype(np.uint8)
    # Close 1-cell gaps from sparse scans, then drop specks.
    wall = cv2.morphologyEx(wall, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(wall, connectivity=8)
    side = np.maximum(stats[:, cv2.CC_STAT_WIDTH], stats[:, cv2.CC_STAT_HEIGHT])
    keep = side * res >= min_len
    keep[0] = False
    wall = keep[lab]

    img = np.full((H, W), UNKNOWN, np.uint8)
    img[height > 0] = FREE
    img[wall] = WALL
    return img[::-1], xy0             # row 0 is +y in map_server images


def wall_res(pts):
    """Cell size for wall_grid that suits this cloud's point spacing.

    5 cm for the dense clouds kiss-gravity and lidar-scan write. kiss-live
    saves one point per 10 cm voxel, and on 5 cm cells such a cloud leaves
    every other cell empty, so no column is ever unbroken and no wall passes.
    Tell them apart by occupancy: a dense surface fills ~2-4 distinct 5 cm
    voxels per 10 cm voxel, a 10 cm grid exactly one.
    """
    # All points: thinning by stride would make a dense cloud look sparse.
    def n(v):
        k = np.floor(pts[:, :3] / v + 1e-3).astype(np.int64) + (1 << 20)
        return len(np.unique((k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]))
    return 0.10 if n(0.05) < 1.5 * n(0.10) else 0.05


SENSOR_H = 0.47      # m, LiDAR above the floor while walking (measured 0.46)
VALLEY_S = 2.0       # s of level walking per 0.2 m below which heights split storeys


def path_for(ply):
    """The sensor path saved beside a map cloud, or None (older maps)."""
    f = os.path.join(os.path.dirname(ply), "gravity_aligned_path.npy")
    return np.load(f).astype(np.float64)[:, :3] if os.path.exists(f) else None


def sensor_height(samples, pts):
    """Median height of the sensor above the floor under it: for each sample
    the densest 10 cm slab within 1 m, 0.15-1.0 m below the sensor."""
    from scipy.spatial import cKDTree
    low = pts[pts[:, 2] < samples[:, 2].max()]
    tree = cKDTree(low[:, :2])
    hs = []
    for x, y, z in samples:
        q = low[tree.query_ball_point((x, y), 1.0)]
        q = q[(q[:, 2] < z - 0.15) & (q[:, 2] > z - 1.0), 2]
        if len(q) < 30:
            continue
        h, e = np.histogram(q, np.arange(z - 1.0, z - 0.1, 0.1))
        hs.append(z - (e[np.argmax(h)] + 0.05))
    return float(np.median(hs)) if len(hs) >= 5 else SENSOR_H


def storeys_from_path(path, pts, hz=10.0, gap=0.4, min_dwell=10.0, hs=None,
                      below=0.3, above=2.4):
    """Storeys where the robot actually walked: [(z_lo, z_hi, floor_z)].

    The robot walks ON the floors, so a floor is where the path dwelt while
    moving level. The cloud's height histogram (floors()) cannot tell a floor
    from a ceiling or a roof, needs storeys 2.2 m apart, and on
    fast-lio-2-multiple-floors-test_20261007_171517 cut the last floor across
    two bands so it got no plan at all.
      - level walking: >= 0.1 m/s over 1 s, climbing < 5 deg (no stairs)
      - floor = sensor height - sensor's height above the floor
      - sorted floor heights split where they jump > `gap`: slow height drift
        within a storey stays one storey, split levels 1.4 m apart do not
      - fewer than `min_dwell` s of walking (a stair landing): not a storey
    Each storey's band runs from `below` under its lowest floor reading to
    `above` over its highest - floor to ceiling, so its walls are whole.
    Returns None when the path cannot say (then use floors()).
    """
    if path is None or len(path) < 3 * hz:
        return None
    k = int(hz)
    d = path[k:] - path[:-k]
    L = np.hypot(d[:, 0], d[:, 1])
    walk = (L >= 0.1) & (np.abs(d[:, 2]) <= np.tan(np.radians(5)) * L)
    if walk.sum() < min_dwell * hz:
        return None
    mid = (path[k:] + path[:-k])[walk] / 2
    if hs is None:                  # the sensor's height above the floor, if not given
        hs = sensor_height(mid[:: max(1, len(mid) // 80)], pts)
    f = np.sort(mid[:, 2] - hs)
    # Split where little level walking happened over a stretch of height:
    # < VALLEY_S seconds per 0.2 m for at least `gap` m. A real floor or ramp
    # collects steady walking time (a 1.2 deg ramp: >= 4.1 s per 0.2 m); the
    # way between two floors only a few seconds (1.1-1.9 s on
    # long-walk-recovered_20261012_150849, whose two lower floors a plain
    # "empty gap" rule merged). An empty gap is the extreme case of this.
    e = np.arange(f[0], f[-1] + 0.4, 0.2)
    h = np.histogram(f, e)[0] / hz
    low = h < VALLEY_S
    cuts, run = [], 0
    for i, lw in enumerate(low):
        run = run + 1 if lw else 0
        if run * 0.2 >= gap - 1e-9 and (i + 1 == len(low) or not low[i + 1]):
            cuts.append(e[i + 1 - run // 2])      # middle of the low stretch
    groups = np.split(f, np.searchsorted(f, cuts))
    st = [(g.min() - below, g.max() + above, float(np.median(g)))
          for g in groups if len(g) >= min_dwell * hz]
    return st or None


def split_by_path(pts, path, below=0.3, above=2.4, view_dz=1.0, **kw):
    """[(floor_z, mask)] per storey, or None: storeys_from_path(), then every
    point goes to the storey of the path point that most plausibly saw it
    (nearest in 3D, compared at the sensor's typical viewing height, as in
    kiss-flatten), and is kept if it lies `below`..`above` the floor THERE.
    A height band per storey cannot do this: on 171517 the ground storey
    rises 2.3 m along a 1.2 deg ramp plus drift, so one band either cut its
    walls or reached into the storey above.
    """
    if path is None:
        return None
    hs = sensor_height(path[:: max(1, len(path) // 80)], pts)   # once, for both steps
    st = storeys_from_path(path, pts, below=0, above=0, hs=hs, **kw)
    if not st:
        return None
    from scipy.spatial import cKDTree
    f = path[:, 2] - hs                                  # floor under each pose
    fz = np.array([s[2] for s in st])
    lab = np.argmin(np.abs(f[:, None] - fz[None, :]), axis=1)
    _, idx = cKDTree(np.c_[path[:, :2], path[:, 2] + view_dz]).query(
        pts[:, :3], k=1, workers=-1)
    rel = pts[:, 2] - f[idx]
    keep = (rel >= -below) & (rel <= above)
    return [(float(z), keep & (lab[idx] == k)) for k, z in enumerate(fz)]


def floors(z, min_storey=2.2, peak=1.5):
    """Split heights into storeys: [(z_lo, z_hi, floor_z)], bottom up.

    Floors (and ceilings) are the dense horizontal slabs, so they are the peaks
    of a smoothed height histogram; walls spread evenly over z and set the
    baseline. Peaks closer than min_storey merge, which folds a ceiling into
    the floor above it. Each storey runs from 0.3 m below its floor peak to
    0.3 m below the next one. A band holding only a ceiling finds no walls and
    is dropped by the caller.
    ponytail: one global floor height per storey; drift that tilts a storey by
    more than ~0.3 m end to end slices its floor. Use per-area floors if seen.
    """
    b = 0.1
    e = np.arange(z.min(), z.max() + 2 * b, b)
    raw = np.histogram(z, e)[0]
    h = np.convolve(raw, np.ones(5) / 5, "same")
    peaks = []
    for i in np.argsort(h)[::-1]:
        if h[i] < peak * np.median(h):
            break
        # Smoothing finds the slab, the raw bins place it: the smoothed peak
        # of a thin floor sits up to 0.2 m off.
        j = max(0, i - 2) + int(np.argmax(raw[max(0, i - 2):i + 3]))
        zc = e[j] + b / 2
        if all(abs(zc - p) >= min_storey for p in peaks):
            peaks.append(zc)
    peaks = sorted(peaks) or [float(z.min())]
    cuts = [-np.inf] + [p - 0.3 for p in peaks[1:]] + [np.inf]
    return [(cuts[i], cuts[i + 1], peaks[i]) for i in range(len(peaks))]


def write_map(out_dir, name, img, origin, res):
    os.makedirs(out_dir, exist_ok=True)
    cv2.imwrite(os.path.join(out_dir, name + ".pgm"), img)
    cv2.imwrite(os.path.join(out_dir, name + ".png"), img)
    with open(os.path.join(out_dir, name + ".yaml"), "w") as f:
        f.write(f"image: {name}.pgm\nresolution: {res}\n"
                f"origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.15\n")
    # free_thresh 0.15, not the usual 0.25: map_server reads a pixel as
    # p = (255 - v) / 255, so our unknown grey 205 is p = 0.196, which 0.25
    # would call FREE and let the planner route through unscanned space.


def selftest():
    """4 x 3 m room, 2.5 m walls, a 0.75 m table in the middle."""
    g = np.arange(0, 4, 0.02)
    h = np.arange(0, 2.5, 0.05)
    walls = [np.c_[x, y, 0 * x + z] for z in h for x, y in
             [(g, 0 * g), (g, 0 * g + 3), (0 * g[:150], g[:150]), (0 * g[:150] + 4, g[:150])]]
    floor = np.c_[np.random.rand(20000, 2) * [4, 3], np.zeros(20000)]
    table = np.c_[np.random.rand(3000, 2) * 0.8 + [1.6, 1.1], np.full(3000, 0.75)]
    img, o = wall_grid(np.vstack(walls + [floor, table]))
    img = img[::-1]                    # back to row 0 = min y for indexing
    at = lambda x, y: img[int((y - o[1]) / 0.05), int((x - o[0]) / 0.05)]
    assert at(2.0, 0.0) == WALL and at(0.0, 1.5) == WALL, "wall missed"
    assert at(2.0, 1.5) == FREE, "table marked as wall"
    assert at(1.0, 1.0) == FREE, "floor not free"
    two = np.vstack(walls + [floor, table])
    two = np.vstack([two, two + [0, 0, 3.0]])     # same room one storey up
    st = floors(two[:, 2])
    assert len(st) == 2 and abs(st[1][2] - 3.0) < 0.2, f"storeys: {st}"
    print("selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map_dir", nargs="?")
    ap.add_argument("--res", type=float, default=None,
                    help="cell size in m (default: 5 cm, or 10 cm for a "
                         "kiss-live cloud - see wall_res)")
    ap.add_argument("--min-height", type=float, default=1.0)
    ap.add_argument("--min-len", type=float, default=0.5)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--one-floor", action="store_true",
                    help="one storey, no splitting: for a map known to be a "
                         "single level, where a ceiling ~2.3 m up would "
                         "otherwise count as a second floor")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if not a.map_dir:
        ap.error("map_dir required")

    hits = glob.glob(os.path.join(os.path.expanduser(a.map_dir), "**",
                                  "latest", "gravity_aligned.ply"), recursive=True)
    if not hits:
        sys.exit("error: no gravity_aligned.ply - run kiss-run (with kiss-gravity) first")
    pts, _ = read_ply(hits[0])
    out = os.path.join(os.path.dirname(hits[0]), "walls")
    if a.res is None:
        a.res = wall_res(pts)
    n = 0
    if a.one_floor:
        parts = [(floors(pts[:, 2])[0][2], np.ones(len(pts), bool))]
    else:
        parts = split_by_path(pts, path_for(hits[0]))
        print(f"  storeys from the path walked: {len(parts)}" if parts
              else "  no usable path: storeys from the height histogram")
        parts = parts or [(fz, (pts[:, 2] >= lo) & (pts[:, 2] < hi))
                          for lo, hi, fz in floors(pts[:, 2])]
    for fz, m in parts:
        sel = pts[m]
        if len(sel) < 100:
            continue
        img, origin = wall_grid(sel, a.res, min_height=a.min_height, min_len=a.min_len)
        if not (img == WALL).any():
            continue
        write_map(out, f"floor{n}", img, origin, a.res)
        print(f"  floor{n} at z {fz:+.2f} m: {len(sel):,} points, "
              f"{(img == WALL).mean() * 100:.1f}% wall -> {out}/floor{n}.pgm")
        n += 1


if __name__ == "__main__":
    main()
