#!/usr/bin/env python3
"""Measure what is bolted to the rig, so the pipeline can be told to ignore it.

A cage, a guard or a mount is rigid with respect to the sensor: its returns
keep the same direction AND the same range no matter where the rig goes. The
room does not. So the test needs motion - a stationary sensor cannot tell a
cage bar from a wall.

    lidar-cage                 move the rig around for 25 s, then read the answer
    lidar-cage ~/bags/walk_x   measure from a recording instead
    lidar-cage --seconds 40

Prints the radius to reject, which goes in two places:

    lidar-scan --min-range <r>
    ~/kiss_indoor.yaml -> odometry.preprocessing.min_range

A radial cut is enough whenever the cage is nearer than everything real you
care about. It is the wrong tool only if real geometry shares the cage's radius,
and the printout says so if that is the case.
"""
import argparse
import os
import sys
from collections import defaultdict

import numpy as np

DT = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
    'formats': ['<f4', '<f4', '<f4', '<f4', 'u1', 'u1', '<f8'],
    'offsets': [0, 4, 8, 12, 16, 17, 18], 'itemsize': 26,
})
CELL = 2.0          # deg, the direction bin: finer than the beam spacing is noise
REF_BAND = (2.0, 8.0)   # m, the control band: only a still sensor holds this fixed
REF_MAX_RIGID = 0.10    # above this the rig was not moving and the run is void
RIGID_STD = 0.05    # m, range spread a rigid object must stay inside
RIGID_HITS = 0.50   # fraction of frames a rigid direction must appear in


def frames_from_bag(path, step):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""))
    rd.set_filter(rosbag2_py.StorageFilter(topics=["/livox/lidar"]))
    i = 0
    while rd.has_next():
        _, data, _ = rd.read_next()
        i += 1
        if i % step:
            continue
        m = deserialize_message(data, PointCloud2)
        yield np.frombuffer(m.data, dtype=DT, count=m.width * m.height)


def frames_live(seconds):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    got = []

    class Grab(Node):
        def __init__(self):
            super().__init__('lidar_cage')
            self.create_subscription(PointCloud2, '/livox/lidar',
                                     lambda m: got.append(np.frombuffer(
                                         m.data, dtype=DT,
                                         count=m.width * m.height)), 50)

    rclpy.init()
    node = Grab()
    print(f"\n  MOVE THE RIG for {seconds:.0f}s - walk it around, turn it, tilt it.")
    print("  Standing still makes this measurement impossible: a still sensor")
    print("  cannot tell a cage bar from a wall.\n")
    import time
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.5)
        print(f"\r  {time.time()-t0:4.0f}s  {len(got):4d} frames", end="", flush=True)
    print()
    node.destroy_node()
    rclpy.shutdown()
    if len(got) < 30:
        sys.exit("error: too few frames - is the driver running? (systemctl status livox-driver)")
    return got


def main():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__.split("\n\n")[0])
    ap.add_argument("bag", nargs="?", default=None)
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--step", type=int, default=8, help="use every Nth bag frame")
    ap.add_argument("--max-range", type=float, default=1.5,
                    help="only look this close to the sensor")
    a = ap.parse_args()

    src = (frames_from_bag(os.path.expanduser(a.bag), a.step) if a.bag
           else frames_live(a.seconds))

    def accumulate(arr, lo, hi, out):
        p = np.stack([arr['x'], arr['y'], arr['z']], 1).astype(np.float32)
        r = np.linalg.norm(p, axis=1)
        # r == 0 is the driver's "no return", not a point 0 m away
        k = (r > max(lo, 0.05)) & (r < hi)
        if not k.any():
            return
        p, r = p[k], r[k]
        az = np.round(np.degrees(np.arctan2(p[:, 1], p[:, 0])) / CELL).astype(int)
        el = np.round(np.degrees(np.arcsin(np.clip(p[:, 2] / r, -1, 1))) / CELL).astype(int)
        nearest = {}
        for A, E, R in zip(az.tolist(), el.tolist(), r.tolist()):
            c = (A, E)
            if c not in nearest or R < nearest[c]:
                nearest[c] = R
        for c, R in nearest.items():
            out[c].append(R)

    def rigid_cells(out, nframe):
        hits = []
        for c, v in out.items():
            v = np.array(v)
            if len(v) / nframe >= RIGID_HITS and v.std() < RIGID_STD:
                hits.append((float(np.median(v)), c, len(v) / nframe))
        return hits

    cells, ref = defaultdict(list), defaultdict(list)
    nframe = 0
    for arr in src:
        nframe += 1
        accumulate(arr, 0.05, a.max_range, cells)
        accumulate(arr, *REF_BAND, ref)

    if not nframe:
        sys.exit("error: no /livox/lidar frames found")
    print(f"\n  {nframe} frames, {len(cells)} directions with a return inside "
          f"{a.max_range:g} m")

    # Self-check. To a rig that never moved, the whole room looks bolted on, and
    # this tool would happily tell you to throw away everything inside 1.5 m.
    # Real geometry in the far band can only hold a fixed range if the sensor
    # stayed put, so that band is the control.
    ref_rigid = rigid_cells(ref, nframe)
    ref_frac = len(ref_rigid) / max(1, len(ref))
    print(f"  motion check: {ref_frac*100:.0f}% of the {REF_BAND[0]:g}-{REF_BAND[1]:g} m "
          f"band held a fixed range")
    if ref_frac > REF_MAX_RIGID:
        print("\n  THE RIG DID NOT MOVE ENOUGH. Measurement discarded.")
        print(f"  {ref_frac*100:.0f}% of the far field looks bolted to the sensor, which")
        print("  only happens when the sensor is stationary. A still sensor cannot")
        print("  tell a cage bar from a wall, so there is no answer to give.")
        print("\n  Redo it while carrying the rig around: turn it, tilt it, walk it.")
        sys.exit(2)

    rigid = rigid_cells(cells, nframe)
    if not rigid:
        best = sorted(((len(v) / nframe, np.std(v), np.median(v), c)
                       for c, v in cells.items()), reverse=True)[:3]
        print("\n  NOTHING RIGID FOUND.")
        print(f"  No direction held a return in >={RIGID_HITS*100:.0f}% of frames with less")
        print(f"  than {RIGID_STD*100:.0f} cm of range spread, so nothing inside {a.max_range:g} m is")
        print("  attached to the rig.")
        for frac, sd, med, c in best:
            print("    closest candidate: az %+4.0f el %+3.0f, %.2f m, in %.0f%% of frames, "
                  "spread %.2f m" % (c[0]*CELL, c[1]*CELL, med, 100*frac, sd))
        print("\n  If the rig did not MOVE during this measurement, the result means")
        print("  nothing - redo it while walking the rig around.")
        print("  Leave lidar-scan and kiss_indoor.yaml as they are.")
        return

    radii = np.array([r for r, _, _ in rigid])
    lo, hi = radii.min(), radii.max()
    print(f"\n  RIGID STRUCTURE: {len(rigid)} directions, "
          f"{100.0*len(rigid)/len(cells):.1f}% of the near field")
    print(f"  radius {lo:.3f} .. {hi:.3f} m  (median {np.median(radii):.3f})")

    # is there anything real at that radius that a radial cut would also eat?
    cut = hi + 0.05
    loose = [(np.median(v), c) for c, v in cells.items()
             if np.median(v) < cut and (len(v) / nframe < RIGID_HITS
                                        or np.std(v) >= RIGID_STD)]
    print(f"\n  recommended cut: {cut:.2f} m")
    print(f"    lidar-scan --min-range {cut:.2f}")
    print(f"    ~/kiss_indoor.yaml: odometry.preprocessing.min_range: {cut:.2f}")
    if loose:
        print(f"\n  WARNING: {len(loose)} non-rigid directions also sit inside {cut:.2f} m.")
        print("  A radial cut throws those real returns away too. Acceptable when they")
        print("  are the operator's own body; not acceptable if you need the floor")
        print("  directly under the sensor. In that case mask by direction instead.")


if __name__ == "__main__":
    main()
