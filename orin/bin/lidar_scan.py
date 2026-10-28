#!/usr/bin/env python3
"""Dense static scan from a stationary MID360.

Not SLAM - there is no trajectory and no loop closure. But the MID360's scan
pattern is non-repetitive, so a stationary sensor keeps revealing new geometry:
measured on this unit, 30 s yields ~24x the distinct surface detail of a single
sweep. Because the sensor does not move, no registration is needed - scans are
already in a common frame and simply accumulate.

    lidar-scan                 30 s, 2 cm voxels
    lidar-scan 60              60 s
    lidar-scan 60 --voxel 0.01 finer

Writes a .ply and publishes it (latched) on /scan/cloud for RViz.
Pure numpy + rclpy, so a plain ROS terminal is enough - no venv.
"""
import argparse
import os
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

DT = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
    'formats': ['<f4', '<f4', '<f4', '<f4', 'u1', 'u1', '<f8'],
    'offsets': [0, 4, 8, 12, 16, 17, 18], 'itemsize': 26,
})
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)


class Scanner(Node):
    def __init__(self, seconds, voxel, min_range=0.25):
        super().__init__('lidar_scan')
        self.voxel = voxel
        self.seconds = seconds
        self.min_range = min_range
        self.acc = {}          # voxel key -> (x, y, z, intensity)
        self.frames = 0
        self.t0 = None
        self.create_subscription(PointCloud2, '/livox/lidar', self.cb, 50)

    def cb(self, msg):
        a = np.frombuffer(msg.data, dtype=DT, count=msg.width * msg.height)
        xyz = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float32)
        inten = a['intensity'].astype(np.float32)
        # r == 0 is the driver's "no return" marker, not a point at the origin,
        # and measured on this unit the 0.05-0.20 m shell is ~8 noise returns
        # per frame at intensity 0. A stationary scan accumulates those into a
        # solid blob sitting on the sensor. Nothing real was measured between
        # 0.20 and 0.38 m, so the default 0.25 m cut is free.
        good = np.linalg.norm(xyz, axis=1) > self.min_range
        xyz, inten = xyz[good], inten[good]
        if self.t0 is None:
            self.t0 = time.time()
        keys = np.round(xyz / self.voxel).astype(np.int32)
        for k, p, i in zip(map(tuple, keys), xyz, inten):
            if k not in self.acc:
                self.acc[k] = (p[0], p[1], p[2], i)
        self.frames += 1

    @property
    def elapsed(self):
        return 0.0 if self.t0 is None else time.time() - self.t0

    def cloud(self):
        v = np.array([(x, y, z, i) for x, y, z, i in self.acc.values()],
                     dtype=np.float32)
        return v


def write_ply(path, v):
    with open(path, 'wb') as f:
        f.write(b"ply\nformat binary_little_endian 1.0\n")
        f.write(f"element vertex {len(v)}\n".encode())
        f.write(b"property float x\nproperty float y\nproperty float z\n")
        f.write(b"property float intensity\nend_header\n")
        f.write(np.ascontiguousarray(v, dtype='<f4').tobytes())


def cloud_msg(v, stamp, frame):
    m = PointCloud2()
    m.header = Header(frame_id=frame)
    m.header.stamp = stamp
    m.height, m.width = 1, len(v)
    m.fields = [PointField(name=n, offset=i * 4,
                           datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate(("x", "y", "z", "intensity"))]
    m.is_bigendian = False
    m.point_step = 16
    m.row_step = 16 * len(v)
    m.is_dense = True
    m.data = np.ascontiguousarray(v, dtype='<f4').tobytes()
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seconds", nargs="?", type=float, default=30.0)
    ap.add_argument("--voxel", type=float, default=0.02)
    ap.add_argument("--out", default=None)
    ap.add_argument("--min-range", type=float, default=0.25,
                    help="drop returns closer than this: the driver's near "
                         "field is noise, and a cage or mount lives here too "
                         "(measure yours with lidar-cage)")
    ap.add_argument("--wait", type=float, default=15.0,
                    help="seconds to wait for the first scan before giving up")
    ap.add_argument("--no-publish", action="store_true",
                    help="exit once the .ply is written instead of staying up "
                         "to publish it (used by the web panel)")
    args = ap.parse_args()

    rclpy.init()
    node = Scanner(args.seconds, args.voxel, args.min_range)
    print(f"\n  accumulating {args.seconds:.0f}s at {args.voxel*100:.0f} cm voxels, "
          f"ignoring returns inside {args.min_range*100:.0f} cm")
    print("  KEEP THE SENSOR PERFECTLY STILL - any motion smears the result\n")

    # Without a deadline this loop is unbounded: frames==0 keeps it spinning
    # for ever, which is exactly what happens when the driver is up but its SDK
    # failed to bind. Fail loudly instead of hanging.
    t_start = time.time()
    while node.elapsed < args.seconds or node.frames == 0:
        rclpy.spin_once(node, timeout_sec=1.0)
        if node.frames == 0 and time.time() - t_start > args.wait:
            print(f"\n  error: no scans on /livox/lidar after {args.wait:g}s")
            print("  the driver may be up but unable to reach the sensor:")
            print("    systemctl status livox-driver")
            print("    journalctl -u livox-driver -b | tail")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        if node.frames % 10 == 0 and node.t0:
            print(f"\r  {node.elapsed:5.1f}s  {node.frames:4d} scans  "
                  f"{len(node.acc):,} points", end="", flush=True)

    v = node.cloud()
    print(f"\r  {node.elapsed:5.1f}s  {node.frames:4d} scans  {len(v):,} points\n")

    out = args.out or os.path.join(
        os.path.expanduser("~/scans"),
        time.strftime("scan_%Y%m%d_%H%M%S.ply"))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    write_ply(out, v)
    print(f"  saved {out}  ({os.path.getsize(out)/1e6:.1f} MB)")

    if args.no_publish:
        node.destroy_node()
        rclpy.shutdown()
        return

    pub = node.create_publisher(PointCloud2, "/scan/cloud", LATCHED)
    pub.publish(cloud_msg(v, node.get_clock().now().to_msg(), "livox_frame"))
    print("\n  publishing on /scan/cloud (latched)")
    print("  RViz: Fixed Frame = livox_frame, Add > By topic > /scan/cloud")
    print("  Ctrl-C to stop\n")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("  stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
