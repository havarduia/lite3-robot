#!/usr/bin/env python3
"""Turn a finished map into a self-contained interactive HTML page.

Answers "how do I see the map without plugging a monitor into the Jetson":
writes ONE .html file with the cloud embedded, viewable in any browser on any
device. No ROS, no RViz, no X server.

    map-web ~/maps/kitchen_20260921_1530     a kiss-slam result
    map-web ~/scans/scan_20260921_123533.ply a static scan
    map-web <input> --budget 400000 -o /tmp/m.html

The MID360 is mounted nose-down, so raw points arrive in a tilted frame: floors
climb, walls lean, and "height" colouring is meaningless. Every cloud is
therefore levelled before it is written. By default the tilt is measured from
the cloud itself, by searching for the rotation that packs the most points into
one 5 cm horizontal slab; --pitch pins it to a known number instead, and
--no-level writes the raw sensor frame. Angles follow lidar-angle: +pitch = nose down, +roll =
right side down. After levelling, z=0 is the floor, so the viewer's metre grid
sits where the floor is.

Points are thinned to a budget so the file stays openable on a phone - one
point per voxel, so a big map goes coarse rather than patchy - and embedded as
base64 Float32Array rather than text (4x smaller).
"""
import argparse, base64, datetime, glob, json, math, os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from maplib import read_ply  # noqa: E402

MOUNT_PITCH = 20.0      # deg nose-down, the angle the mount is built to


# ---------------------------------------------------------------- reading ---

def gather(inp, tiles=False):
    """Pick the point cloud to render out of a kiss-slam output directory.

    occupancy_grid/occupancy_pcd.ply is ONE cloud, fused from every scan at its
    optimised pose, ground-aligned and thresholded. local_maps/plys/*.ply is a
    per-node dump, and each node keeps everything within max_range of itself -
    40 m by default, which indoors is the whole building. Unioning the tiles
    therefore stacks the same walls once per node: measured on a stairwell
    walk, 38 tiles produced 59x the points of one tile and 40x the distinct
    5 cm voxels, i.e. dozens of mutually offset copies of one staircase. Prefer
    the fused cloud; --tiles forces the old union for comparison.

    gravity_aligned.ply, when kiss-gravity has written one, beats both: the same
    fusion, but with each scan's tilt pinned to measured gravity instead of to a
    SLAM attitude that drifts tens of degrees over a walk.
    """
    if os.path.isfile(inp):
        return [inp]
    # Re-running kiss-run adds a new slam_output/<timestamp> run beside the old
    # ones; globbing them all unions every run, failed ones included. Only
    # read the run 'latest' points at.
    latest = glob.glob(os.path.join(inp, "**", "slam_output", "latest"), recursive=True)
    if latest:
        inp = latest[0]
    pats = [
        os.path.join(inp, "**", "local_maps", "plys", "*.ply") if tiles else
        os.path.join(inp, "**", "gravity_aligned.ply"),
        os.path.join(inp, "**", "occupancy_grid", "occupancy_pcd.ply"),
        os.path.join(inp, "**", "local_maps", "plys", "*.ply"),
        os.path.join(inp, "**", "*.ply"),
    ]
    for p in pats:
        hits = sorted(glob.glob(p, recursive=True))
        if hits:
            # slam_output/latest is a symlink to slam_output/<timestamp>, so a
            # recursive glob finds every tile under both names. Reading each
            # cloud twice doubles the point count and hides real duplication.
            seen, out = set(), []
            for h in hits:
                r = os.path.realpath(h)
                if r not in seen:
                    seen.add(r)
                    out.append(h)
            return out
    return []


# --------------------------------------------------------------- levelling ---

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def level_matrix(pitch_deg, roll_deg):
    """Rotation taking the tilted sensor frame back to a level world frame.

    pitch is nose-down-positive and roll is right-side-down-positive (the
    lidar-angle convention), so undoing them means +pitch about y and -roll
    about x, roll first.
    """
    return rot_y(math.radians(pitch_deg)) @ rot_x(math.radians(-roll_deg))


def fit_level(pts, bin_m=0.05, seed=0, pitch_range=(-10.0, 40.0),
              roll_range=(-20.0, 20.0)):
    """Find the (pitch, roll) that makes the dominant horizontal surface flat.

    Returns (pitch_deg, roll_deg, share_of_points_in_the_densest_slab).

    Levelling needs exactly one thing: the rotation that turns the floor
    horizontal. So score that directly. Only the z row of the rotation matters,
    so each candidate costs one dot product, and the score is the population of
    the densest bin_m slab - flat floor, one fat slab; any residual tilt smears
    it across many.

    This replaced a 3-point RANSAC plane fit, which sampled 60k points out of a
    1.6 M cloud and could not find a real floor in a building-sized map: it
    returned a 5%-inlier "ground plane" and levelled an already-straight map
    into an 11 deg slope. Measured on the same clouds, this search puts 12.8%
    of a one-floor map and 7.7% of a two-floor map into a single 10 cm slab.

    A wall cannot win: making one horizontal needs ~90 deg, far outside the
    search range. A ceiling can, and that is fine - it is parallel to the floor,
    so it yields the same rotation.
    """
    rng = np.random.default_rng(seed)
    sub = pts if len(pts) <= 60000 else pts[rng.choice(len(pts), 60000, replace=False)]
    sub = sub.astype(np.float32)

    def score(pitch, roll):
        zrow = level_matrix(pitch, roll)[2].astype(np.float32)
        z = sub @ zrow
        q = ((z - z.min()) / bin_m).astype(np.int32)
        return int(np.bincount(q).max())

    best, best_score = (MOUNT_PITCH, 0.0), -1
    for step, span in ((2.0, None), (0.5, 3.0), (0.1, 0.7)):
        if span is None:
            ps = np.arange(pitch_range[0], pitch_range[1] + step, step)
            rs = np.arange(roll_range[0], roll_range[1] + step, step)
        else:
            ps = np.arange(best[0] - span, best[0] + span + step, step)
            rs = np.arange(best[1] - span, best[1] + span + step, step)
        best_score = -1
        for pitch in ps:
            for roll in rs:
                sc = score(pitch, roll)
                if sc > best_score:
                    best_score, best = sc, (float(pitch), float(roll))
    return best[0], best[1], best_score / len(sub)


def floor_height(z, bin_m=0.05, cap=0.0):
    """z of the densest horizontal slab below the sensor.

    The sensor sits at z=0 and cannot be under the floor, so the search stops
    there. Without that cap the densest low slab indoors is usually a desk or
    a bench - measured here, a 5 cm slab of desktops beat the real floor 5:1
    and put the sensor 46 cm underground.
    """
    lo = float(z.min())
    hi = float(np.median(z) if cap is None else min(np.median(z), cap))
    if not np.isfinite(lo) or hi - lo < bin_m:
        return float(np.percentile(z, 1.0))
    bins = np.arange(lo, hi + bin_m, bin_m)
    h, e = np.histogram(z, bins=bins)
    if not len(h):
        return float(lo)
    return float(e[int(np.argmax(h))] + bin_m / 2)


# ------------------------------------------------------------- thinning ---

def voxel_thin(pts, inten, budget):
    """Keep one point per voxel, sized so the result lands near budget.

    Random sampling was keeping 9.9% of a 4 M point walk: every surface became
    scattered dust, which reads as "the map is missing points" because at any
    one place it is. One point per voxel spends the same budget on even
    coverage instead - the map gets coarser, never patchier.

    Distinct voxels on a surface scale as s^-2, so one probe predicts the size
    for the next: three or four passes land inside 15% of the budget.
    """
    if len(pts) <= budget:
        return pts, inten, 0.0
    mn = pts.min(axis=0)

    def keys(s):
        k = np.floor((pts - mn) / s).astype(np.int64)
        # 1-D hash: np.unique on 4 M rows of 3 is far slower than on 4 M ints,
        # and a collision merges two voxels, which is invisible.
        return (k[:, 0] * 73856093) ^ (k[:, 1] * 19349663) ^ (k[:, 2] * 83492791)

    size = float((pts.max(axis=0) - mn).max()) / 512.0
    for _ in range(5):
        n = len(np.unique(keys(size)))
        if 0.85 * budget <= n <= 1.15 * budget:
            break
        size *= math.sqrt(max(n, 1) / budget)
    _, idx = np.unique(keys(size), return_index=True)
    if len(idx) > budget:
        idx = np.random.default_rng(0).choice(idx, budget, replace=False)
    idx = np.sort(idx)
    return pts[idx], (inten[idx] if inten is not None else None), size


# -------------------------------------------------------------------- main ---

def main():
    # The web panel runs inside ~/kiss_env, whose numpy 2 breaks the system
    # OpenCV that the wall plans need. Hop to the system interpreter then.
    try:
        import cv2  # noqa: F401
    except ImportError:
        if sys.prefix != sys.base_prefix:          # in a venv: leave it
            os.execv("/usr/bin/python3", ["/usr/bin/python3"] + sys.argv)
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__.split("\n\n")[0])
    ap.add_argument("input")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--budget", type=int, default=900000,
                    help="points to keep (page weight is ~17 MB per million); "
                         "lower it for phones over slow links")
    ap.add_argument("--pitch", type=float, default=None,
                    help=f"nose-down mount angle in deg (default: measure it, "
                         f"falling back to {MOUNT_PITCH:g})")
    ap.add_argument("--roll", type=float, default=None,
                    help="right-side-down roll in deg (default: measure it)")
    ap.add_argument("--no-level", action="store_true",
                    help="write the raw tilted sensor frame")
    ap.add_argument("--tiles", action="store_true",
                    help="render the per-node local map tiles instead of the "
                         "fused global cloud (they overlap heavily)")
    ap.add_argument("--sensor-height", type=float, default=None,
                    help="metres from the floor to the sensor (default: "
                         "measure it from the cloud)")
    ap.add_argument("--title", default=None)
    ap.add_argument("--one-floor", action="store_true",
                    help="one wall plan for the whole map (see map-walls)")
    a = ap.parse_args()

    files = gather(a.input, a.tiles)
    if not files:
        sys.exit(f"error: no .ply found under {a.input}")
    print(f"  {len(files)} ply file(s)")

    chunks, itens = [], []
    for f in files:
        try:
            v, inten = read_ply(f)
            if v is not None and len(v):
                chunks.append(v)
                itens.append(inten if inten is not None else np.full(len(v), np.nan, np.float32))
        except Exception as e:
            print(f"  skip {os.path.basename(f)}: {e}")
    if not chunks:
        sys.exit("error: no readable points")
    pts = np.vstack(chunks)
    inten = np.concatenate(itens)
    good = np.isfinite(pts).all(axis=1)
    pts, inten = pts[good], inten[good]
    if not np.isfinite(inten).any():
        inten = None
    print(f"  {len(pts):,} points")

    # ---- levelling -------------------------------------------------------
    # kiss-slam's fused cloud goes through align_map_to_local_ground, but that
    # leaves 12-13 deg on these maps, so it gets measured like everything else.
    pitch = roll = 0.0
    source = "off (raw sensor frame)"
    if not a.no_level:
        if a.pitch is None or a.roll is None:
            fp, fr, frac = fit_level(pts)
            pitch = a.pitch if a.pitch is not None else fp
            roll = a.roll if a.roll is not None else fr
            source = f"measured ({frac*100:.0f}% of points in one 5 cm slab)"
            print(f"  levelling fit: pitch {fp:+.2f} deg, roll {fr:+.2f} deg "
                  f"({frac*100:.0f}% of points in the densest 5 cm slab)")
        else:
            pitch, roll = a.pitch, a.roll
            source = "given on the command line"
        R = level_matrix(pitch, roll)
        pts = (pts @ R.T).astype(np.float32)
        print(f"  levelled: pitch {pitch:+.2f} deg nose-down, roll {roll:+.2f} deg")
    else:
        R = np.eye(3)
        print("  levelling disabled")

    # ---- wall plans ------------------------------------------------------
    # From the full levelled cloud, before thinning, so thin walls survive.
    # One plan per storey, drawn on that storey's floor as a texture: walls
    # opaque, seen floor faint, unknown clear. Needs OpenCV; without it the
    # page just has no Walls layer.
    walls = []                      # [png b64, x, y, w, h, floor z] per storey
    try:
        import cv2
        from map_walls import (wall_grid, wall_res, floors, path_for,
                               split_by_path, WALL, FREE)
        wres = wall_res(pts)          # 10 cm for a kiss-live cloud, else 5
        # Storeys from the path the robot walked (see map_walls.split_by_path),
        # in this page's levelled frame; the height histogram only as a
        # fallback - it mistakes ceilings for floors and merges split levels.
        raw = path_for(files[0]) if len(files) == 1 else None
        parts = None
        if a.one_floor:
            parts = [(None, np.ones(len(pts), bool))]
        elif raw is not None:
            parts = split_by_path(pts, raw @ R.T)
        if parts is None:
            parts = [(None, (pts[:, 2] >= lo) & (pts[:, 2] < hi))
                     for lo, hi, _ in floors(pts[:, 2])]
        for fz, m in parts:
            band = pts[m]
            if len(band) < 100:
                continue
            if fz is None:
                # The storey's peak can be its ceiling (a single scan often
                # sees more ceiling than floor); the plan belongs on the floor.
                fz = floor_height(band[:, 2], cap=None)
            wimg, wxy = wall_grid(band, wres)
            if not (wimg == WALL).any():
                continue
            rgba = np.zeros(wimg.shape + (4,), np.uint8)   # BGRA for cv2
            rgba[wimg == FREE] = (255, 255, 255, 30)
            rgba[wimg == WALL] = (80, 210, 255, 255)       # amber #ffd250
            h, w = wimg.shape
            walls.append([base64.b64encode(cv2.imencode(".png", rgba)[1]).decode(),
                          float(wxy[0]), float(wxy[1]), w * wres, h * wres, fz])
            print(f"  walls: storey at z {fz:+.2f}, "
                  f"{(wimg == WALL).mean()*100:.1f}% of {w}x{h} cells")
    except Exception as e:
        print(f"  walls skipped: {e}")

    # ---- downsample, centre ---------------------------------------------
    if len(pts) > a.budget:
        before = len(pts)
        pts, inten, vsize = voxel_thin(pts, inten, a.budget)
        print(f"  thinned {before:,} -> {len(pts):,} points "
              f"(one per {vsize*100:.0f} cm voxel)")

    # The sensor sits at the origin of the frame the points arrive in - the
    # LiDAR itself for a static scan, the first scan pose for a SLAM map - and
    # levelling only rotates about that origin. So keeping x=y=0 puts the map
    # where it belongs relative to the sensor: the viewer's grid centre, its
    # axes and its orbit pivot all land on the real sensor position, instead of
    # drifting with the point centroid (which moves whenever one wall happens
    # to return more points than another).
    if a.sensor_height is not None:
        z0 = -a.sensor_height
    elif a.no_level:
        z0 = float(pts[:, 2].min())
    else:
        z0 = floor_height(pts[:, 2])
    centre = np.array([0.0, 0.0, z0], dtype=np.float32)
    pts = pts - centre
    sensor_z = float(-z0)          # sensor height above the floor
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    span = float(np.abs(pts).max())
    print(f"  extent: {hi[0]-lo[0]:.1f} x {hi[1]-lo[1]:.1f} x {hi[2]-lo[2]:.1f} m"
          f"   floor at z=0")
    print(f"  sensor at x=0 y=0, {sensor_z:.2f} m above the floor")

    # ---- path ---------------------------------------------------------------
    # The sensor's path, in the cloud's frame, for the Path layer: kiss-gravity
    # and kiss-live write gravity_aligned_path.npy beside gravity_aligned.ply;
    # a plain kiss-run has only the SLAM poses (the frame of its own clouds).
    path = np.empty((0, 3))
    d = os.path.dirname(files[0]) if len(files) == 1 else None
    if d and os.path.basename(files[0]) == "gravity_aligned.ply":
        f = os.path.join(d, "gravity_aligned_path.npy")
        if os.path.isfile(f):
            path = np.load(f)[:, :3]
    elif d:
        hits = glob.glob(os.path.join(d, "..", "..", "*_poses.npy")) + \
            glob.glob(os.path.join(d, "*_poses.npy"))
        if hits:
            path = np.load(hits[0])[:, :3, 3]
    if len(path):
        path = path @ R.T - centre
        keep, last = [], None             # one point per 5 cm: a light page
        for i, q in enumerate(path):
            if last is None or np.linalg.norm(q - last) >= 0.05:
                keep.append(i)
                last = q
        keep.append(len(path) - 1)
        path = path[keep]
        print(f"  path: {len(path):,} points, "
              f"{np.linalg.norm(np.diff(path, axis=0), axis=1).sum():.1f} m")

    # intensity normalised to 0..1 on a robust range, as uint8
    if inten is not None:
        finite = np.isfinite(inten)
        inten = np.where(finite, inten, 0.0)
        ilo, ihi = np.percentile(inten[finite], [2, 98]) if finite.any() else (0, 1)
        if ihi - ilo < 1e-6:
            ihi = ilo + 1.0
        ib = np.clip((inten - ilo) / (ihi - ilo), 0, 1)
        i_b64 = base64.b64encode((ib * 255).astype(np.uint8).tobytes()).decode()
    else:
        i_b64 = ""

    b64 = base64.b64encode(np.ascontiguousarray(pts, dtype="<f4").tobytes()).decode()
    # inverse rotation, so the viewer can swing back to the raw tilted frame
    inv = np.linalg.inv(R)

    name = a.title or os.path.basename(os.path.normpath(a.input))
    out = a.out or os.path.join(os.path.expanduser("~/maps"),
                                os.path.splitext(os.path.basename(
                                    os.path.normpath(a.input)))[0] + ".html")
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

    subst = {
        "__DATA__": b64,
        "__INTEN__": i_b64,
        "__N__": str(len(pts)),
        "__SPAN__": f"{span:.3f}",
        "__NAME__": name,
        "__PITCH__": f"{pitch:.2f}",
        "__ROLL__": f"{roll:.2f}",
        "__LEVELSRC__": source,
        "__LEVELLED__": "false" if a.no_level else "true",
        "__INV__": "[" + ",".join(f"{x:.6f}" for x in inv.T.flatten()) + "]",
        "__DIMS__": f"{hi[0]-lo[0]:.1f} x {hi[1]-lo[1]:.1f} x {hi[2]-lo[2]:.1f}",
        "__SENSORZ__": f"{sensor_z:.2f}",
        "__ZLO__": f"{lo[2]:.2f}",
        "__ZHI__": f"{hi[2]:.2f}",
        "__WHEN__": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
        "__SRC__": os.path.basename(os.path.normpath(a.input)),
        "__NFILES__": str(len(files)),
        "__PATH__": json.dumps([round(float(v), 3) for v in path.ravel()]),
        "__WALLS__": json.dumps([[b, round(x, 3), round(y, 3), round(w, 3),
                                   round(h, 3), round(fz - z0, 3)]
                                  for b, x, y, w, h, fz in walls]),
    }
    html = TEMPLATE
    for k, v in subst.items():
        html = html.replace(k, v)
    with open(out, "w") as f:
        f.write(html)
    print(f"  wrote {out}  ({os.path.getsize(out)/1e6:.1f} MB)")
    print("  open it in any browser - no ROS, no display needed")


# The page itself: map_viewer.html next to this script, with __NAME__-style
# placeholders that main() fills in.
TEMPLATE = open(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                             "map_viewer.html"), encoding="utf-8").read()

if __name__ == "__main__":
    main()
