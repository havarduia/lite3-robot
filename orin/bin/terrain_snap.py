#!/usr/bin/env python3
"""terrain_snap - keep CMU terrainAnalysis's /terrain_map, a few seconds apart, to plot elsewhere.

The Orin has no matplotlib: this saves what to plot, and the laptop draws it.
Each snapshot is the map's points (x, y, z, height above ground) and the robot's
levelled pose (/state_estimation) at that moment, all in terrain_level's 'level' frame.

    python3 terrain_snap.py OUT.npz      ROS sourced; Ctrl-C (SIGINT) writes OUT.npz
"""
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

SNAP_S = 2.0            # seconds between snapshots


def xyzi(m):
    off = {f.name: f.offset for f in m.fields}
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(-1, m.point_step)
    return np.stack([a[:, off[k]:off[k] + 4].copy().view(np.float32)[:, 0]
                     for k in ('x', 'y', 'z', 'intensity')], axis=1)


def main(out):
    rclpy.init()
    node = rclpy.create_node('terrain_snap')
    snaps, pose, last = [], {}, [0.0]

    def odom(m):
        p = m.pose.pose.position
        pose['p'] = (p.x, p.y, p.z)

    def terrain(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - last[0] < SNAP_S or 'p' not in pose:
            return
        last[0] = t
        snaps.append((t, xyzi(m), pose['p']))

    node.create_subscription(Odometry, '/state_estimation', odom, 20)
    node.create_subscription(PointCloud2, '/terrain_map', terrain, 2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    np.savez_compressed(out, t=np.array([s[0] for s in snaps]), pose=np.array([s[2] for s in snaps]),
                        **{'map%03d' % i: s[1] for i, s in enumerate(snaps)})
    print('%d snapshots -> %s' % (len(snaps), out))


if __name__ == '__main__':
    main(sys.argv[1])
