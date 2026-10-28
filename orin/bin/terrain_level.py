#!/usr/bin/env python3
"""terrain_level - FAST-LIO2's cloud and pose, levelled, for CMU terrainAnalysis.

FAST-LIO2's world frame (camera_init) is the IMU frame at start-up, tilted ~20 deg
nose down on this mount; terrainAnalysis takes z as up. This turns both by the
rotation that takes FAST-LIO2's own "up" (minus /lio/gravity) to z. It is fixed once,
from the first GRAVITY_N estimates: terrainAnalysis keeps a map in this frame, so the
frame must not move under it.

    /cloud_registered, /Odometry  ->  /registered_scan, /state_estimation   (frame 'level')

    python3 terrain_level.py          system python, ROS sourced (nav_stack_migration.md, step 2)
    python3 terrain_level.py check    self-check of the maths, no ROS
"""
import sys

import numpy as np

GRAVITY_N = 20          # /lio/gravity estimates averaged before the frame is fixed (2 s at 10 Hz)
FRAME = 'level'


def align(a, b):
    """The rotation matrix that turns unit vector a onto unit vector b (not opposite)."""
    v, c = np.cross(a, b), float(np.dot(a, b))
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1.0 + c)


def quat(R):
    """(x, y, z, w) of a rotation matrix."""
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def level_points(data, step, R):
    """The cloud's bytes with x, y, z (float32 at 0, 4, 8 of each point) turned by R."""
    a = np.frombuffer(data, np.float32).reshape(-1, step // 4).copy()
    a[:, :3] = a[:, :3] @ R.T.astype(np.float32)
    return a.tobytes()


def main():
    import rclpy
    from geometry_msgs.msg import Vector3Stamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = rclpy.create_node('terrain_level')
    cloud_pub = node.create_publisher(PointCloud2, '/registered_scan', 5)
    odom_pub = node.create_publisher(Odometry, '/state_estimation', 5)
    ups, frame = [], {}

    def gravity(m):
        if 'R' in frame:
            return
        ups.append(-np.array([m.vector.x, m.vector.y, m.vector.z]))
        if len(ups) >= GRAVITY_N:
            up = np.mean(ups, axis=0)
            frame['R'] = align(up / np.linalg.norm(up), np.array([0.0, 0.0, 1.0]))
            frame['q'] = quat(frame['R'])
            node.get_logger().info('levelled by %.1f deg' % np.degrees(np.arccos(up[2] / np.linalg.norm(up))))

    def cloud(m):
        if 'R' not in frame:
            return
        assert [(f.name, f.offset) for f in m.fields[:3]] == [('x', 0), ('y', 4), ('z', 8)], m.fields
        m.data = level_points(bytes(m.data), m.point_step, frame['R'])
        m.header.frame_id = FRAME
        cloud_pub.publish(m)

    def odom(m):
        if 'R' not in frame:
            return
        p, o = m.pose.pose.position, m.pose.pose.orientation
        p.x, p.y, p.z = frame['R'] @ np.array([p.x, p.y, p.z])
        o.x, o.y, o.z, o.w = qmul(frame['q'], (o.x, o.y, o.z, o.w))
        m.header.frame_id = FRAME
        odom_pub.publish(m)

    node.create_subscription(Vector3Stamped, '/lio/gravity', gravity, 20)
    node.create_subscription(PointCloud2, '/cloud_registered', cloud, 5)
    node.create_subscription(Odometry, '/Odometry', odom, 20)
    rclpy.spin(node)


def check():
    tilt = np.radians(20.4)
    up = np.array([np.sin(tilt), 0.0, np.cos(tilt)])          # FAST-LIO2's up, as on this mount
    R = align(up, np.array([0.0, 0.0, 1.0]))
    assert np.allclose(R @ up, [0, 0, 1]) and np.allclose(R @ R.T, np.eye(3)) and np.isclose(np.linalg.det(R), 1)
    # the quaternion is the same rotation
    q = quat(R)
    v = np.array([0.3, -1.2, 0.5])
    qv = qmul(qmul(q, np.r_[v, 0.0]), q * [-1, -1, -1, 1])[:3]
    assert np.allclose(qv, R @ v), (qv, R @ v)
    # a floor point 0.46 m below the lidar, seen in FAST-LIO2's tilted frame, comes out 0.46 m below
    floor = -0.46 * up
    pts = np.zeros((2, 12), np.float32)
    pts[0, :3] = floor
    pts[1, :3], pts[1, 8] = [1.0, 2.0, 3.0], 7.0                 # and the other fields are left alone
    out = np.frombuffer(level_points(pts.tobytes(), 48, R), np.float32).reshape(2, 12)
    assert np.allclose(out[0, :3], [0, 0, -0.46], atol=1e-6) and out[1, 8] == 7.0, out
    print('terrain_level ok')


if __name__ == '__main__':
    check() if sys.argv[1:] == ['check'] else main()
