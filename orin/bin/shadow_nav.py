#!/usr/bin/env python3
"""shadow_nav - MPPI follows the Jetson's route in shadow: it steers nothing (nav_stack_migration.md, step 3).

The Jetson's robot/shadow_link.py sends Nav2's route (in base_link, the moment it was planned)
and the command RPP gives. This puts the route in front of the robot as the Orin sees him and
hands it to the Orin's controller_server (MPPI on the terrain map); MPPI's command, on
/shadow/cmd_vel, goes nowhere but the log next to RPP's.

Also the 2D frame Nav2 needs: odom -> base_link (planar: x, y, yaw), /odom with speeds, and
/terrain_h, the terrain map with z set to its height above the ground (what the costmap marks).

    python3 shadow_nav.py LOG.csv        system python, ROS sourced; started by shadow.sh
    python3 shadow_nav.py check          self-check, no ROS
"""
import csv
import math
import socket
import struct
import sys
import threading
import time

import numpy as np

from lio_relay import MOUNT_XYZ, Q_MOUNT, join, qconj, qmul, rot, twist, wrap, yaw_of

PORT = 8044                 # robot/shadow_link.py sends here
JETSON = '192.168.1.103'
ROUTE = struct.Struct('<c')         # T, then float32 (x, y, yaw) triplets in base_link
CMD = struct.Struct('<c3d')         # V, RPP's vx, vy, wz
ROUTE_STALE_S = 5.0         # no route for this long: the Jetson's goal is over, MPPI lets go too
CMD_STALE_S = 0.5           # an RPP command older than this is logged as none


def base_of(p, q):
    """The levelled FAST-LIO2 body pose -> base_link as (x, y, yaw); the mount is base_link -> body."""
    q_base = qmul(q, qconj(Q_MOUNT))
    lever = rot(q_base, MOUNT_XYZ)
    return p[0] - lever[0], p[1] - lever[1], yaw_of(q_base)


def route_in_odom(base, route):
    """Route points (x, y, yaw in base_link) -> odom, with base_link at `base` (x, y, yaw)."""
    return [join(base, r) for r in route]


def unpack_route(data):
    a = np.frombuffer(data, np.float32, offset=ROUTE.size).reshape(-1, 3)
    return [tuple(map(float, r)) for r in a]


def main(log_path):
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from nav2_msgs.action import FollowPath
    from nav_msgs.msg import Odometry, Path
    from rclpy.action import ActionClient
    from sensor_msgs.msg import PointCloud2
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    node = rclpy.create_node('shadow_nav')
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    terrain_pub = node.create_publisher(PointCloud2, '/terrain_h', 2)
    path_pub = node.create_publisher(Path, '/shadow/route', 2)
    tf_pub = TransformBroadcaster(node)
    follow = ActionClient(node, FollowPath, 'follow_path')
    s = {'base': None, 'prev': None, 'rpp': None, 'route_at': None, 'goal': None, 'sent': 0}
    lock = threading.Lock()
    out = open(log_path, 'w', newline='')
    log = csv.writer(out)
    log.writerow(['t', 'x', 'y', 'yaw', 'rpp_vx', 'rpp_vy', 'rpp_wz', 'mppi_vx', 'mppi_vy', 'mppi_wz', 'routes'])

    def odom(m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x, y, yaw = base_of((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        now = (t, x, y, yaw)
        vx, vy, wz = twist(s['prev'], now) if s['prev'] and 0 < t - s['prev'][0] < 1.0 else (0.0, 0.0, 0.0)
        s['prev'], s['base'] = now, (x, y, yaw)
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = m.header.stamp, 'odom', 'base_link'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        tf_pub.sendTransform(tf)
        o2 = Odometry()
        o2.header.stamp, o2.header.frame_id, o2.child_frame_id = m.header.stamp, 'odom', 'base_link'
        o2.pose.pose.position.x, o2.pose.pose.position.y = x, y
        o2.pose.pose.orientation = tf.transform.rotation
        o2.twist.twist.linear.x, o2.twist.twist.linear.y, o2.twist.twist.angular.z = vx, vy, wz
        odom_pub.publish(o2)

    def terrain(m):
        # x, y as they are (level and odom share them); z := height above the local ground
        off = {f.name: f.offset for f in m.fields}
        a = np.frombuffer(bytes(m.data), np.float32).reshape(-1, m.point_step // 4).copy()
        a[:, off['z'] // 4] = a[:, off['intensity'] // 4]
        m.data = a.tobytes()
        m.header.frame_id = 'odom'
        terrain_pub.publish(m)

    def mppi(c):
        with lock:
            rpp = s['rpp'] if s['rpp'] and time.monotonic() - s['rpp'][0] < CMD_STALE_S else None
        b = s['base'] or (math.nan,) * 3
        log.writerow(['%.3f' % time.time()] + ['%.3f' % v for v in b]
                     + (['%.3f' % v for v in rpp[1:]] if rpp else ['', '', ''])
                     + ['%.3f' % c.linear.x, '%.3f' % c.linear.y, '%.3f' % c.angular.z, s['sent']])

    def give(route):
        if s['base'] is None or len(route) < 2:
            return
        p = Path()
        p.header.stamp, p.header.frame_id = node.get_clock().now().to_msg(), 'odom'
        for x, y, yaw in route_in_odom(s['base'], route):
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            p.poses.append(ps)
        path_pub.publish(p)
        g = FollowPath.Goal()
        g.path, g.controller_id, g.goal_checker_id = p, 'FollowPath', 'general_goal_checker'
        s['goal'] = follow.send_goal_async(g)       # a new goal replaces the running one
        s['sent'] += 1

    def watch():
        # the Jetson's route has stopped coming: stop following it
        if s['route_at'] and time.monotonic() - s['route_at'] > ROUTE_STALE_S and s['goal']:
            gh = s['goal'].result() if s['goal'].done() else None
            if gh is not None and gh.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                gh.cancel_goal_async()
            s['goal'] = None
            node.get_logger().info('route stopped coming; MPPI let go')

    def read():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', PORT))
        while True:
            data, (host, _) = sock.recvfrom(65535)
            if host != JETSON or not data:
                continue
            if data[:1] == b'V' and len(data) == CMD.size:
                with lock:
                    s['rpp'] = (time.monotonic(),) + CMD.unpack(data)[1:]
            elif data[:1] == b'T' and (len(data) - ROUTE.size) % 12 == 0:
                s['route_at'] = time.monotonic()
                s['route'] = unpack_route(data)

    def take_route():
        r = s.pop('route', None)
        if r is not None:
            give(r)

    node.create_subscription(Odometry, '/state_estimation', odom, 20)
    node.create_subscription(PointCloud2, '/terrain_map', terrain, 2)
    node.create_subscription(Twist, '/shadow/cmd_vel', mppi, 10)
    node.create_timer(0.1, take_route)      # the ROS calls stay on the spinning thread
    node.create_timer(1.0, watch)
    threading.Thread(target=read, daemon=True).start()
    try:
        rclpy.spin(node)
    except Exception:       # KeyboardInterrupt, or rclpy's ExternalShutdownException on SIGTERM
        pass
    finally:
        out.close()


def check():
    # standing level: the body is pitched by the mount, base_link sits MOUNT_XYZ behind/below it
    body = (1.0 + MOUNT_XYZ[0], 2.0, 0.5 + MOUNT_XYZ[2])
    x, y, yaw = base_of(body, Q_MOUNT)
    assert abs(x - 1.0) < 1e-9 and abs(y - 2.0) < 1e-9 and abs(yaw) < 1e-9, (x, y, yaw)
    # turned 90 deg left: the lever arm turns with him
    qz = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    x, y, yaw = base_of((1.0, 2.0 + MOUNT_XYZ[0], 0.0), qmul(qz, Q_MOUNT))
    assert abs(x - 1.0) < 1e-9 and abs(y - 2.0) < 1e-9 and abs(yaw - math.pi / 2) < 1e-9, (x, y, yaw)
    # a route 1 m ahead of a robot facing +y at (1, 2) is at (1, 3) in odom
    (rx, ry, ryaw), = route_in_odom((1.0, 2.0, math.pi / 2), [(1.0, 0.0, 0.0)])
    assert abs(rx - 1.0) < 1e-9 and abs(ry - 3.0) < 1e-9 and abs(wrap(ryaw - math.pi / 2)) < 1e-9
    # the datagram round trip
    pts = [(0.0, 0.0, 0.0), (0.5, -0.25, 1.5)]
    data = ROUTE.pack(b'T') + np.array(pts, np.float32).tobytes()
    assert unpack_route(data) == pts, unpack_route(data)
    print('shadow_nav ok')


if __name__ == '__main__':
    check() if sys.argv[1:] == ['check'] else main(sys.argv[1])
