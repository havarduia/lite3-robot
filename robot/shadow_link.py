"""shadow_link - Nav2's route and RPP's commands, to the Orin's MPPI running in shadow.

Step 3 of ~/nav_stack_migration.md: the Orin's MPPI (orin/bin/shadow_nav.py) follows the
same route as today's stack, and its commands are logged next to RPP's, never sent. Each
/plan goes over in base_link at the moment it arrives (the two computers share no frame);
each /cmd_vel goes over as it is. Nothing here changes what drives the robot.

    python3 -m robot.shadow_link          while Nav2 runs (source env/lite3_env.sh first)
    python3 -m robot.shadow_link check    self-check, no ROS
"""
import math
import socket
import struct
import sys

from .lio_relay import ORIN, yaw_of

ORIN_PORT = 8044
STEP_M = 0.1            # route points kept this far apart (NavFn gives one per 5 cm cell)
MAX_POINTS = 1000       # 100 m of route, 12 kB: one datagram
CMD = struct.Struct('<c3d')


def thin(points):
    """Every point at least STEP_M from the last kept one, and always the last."""
    kept = [points[0]]
    for p in points[1:-1]:
        if math.hypot(p[0] - kept[-1][0], p[1] - kept[-1][1]) >= STEP_M - 1e-6:
            kept.append(p)
    return (kept + [points[-1]] if len(points) > 1 else kept)[:MAX_POINTS]


def in_base(points, t, yaw):
    """(x, y, yaw) points in the route's frame -> base_link, by the transform base_link <- that frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return [(t[0] + c * x - s * y, t[1] + s * x + c * y, a + yaw) for x, y, a in points]


def route_packet(points):
    return b'T' + struct.pack('<%df' % (3 * len(points)), *[v for p in points for v in p])


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Path
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

    rclpy.init()
    node = rclpy.create_node('shadow_link')
    tf = Buffer()
    TransformListener(tf, node)
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def plan(m):
        if not m.poses:
            return
        try:
            tr = tf.lookup_transform('base_link', m.header.frame_id, Time()).transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            node.get_logger().warn('no transform for the route: %s' % e)
            return
        q = tr.rotation
        # NavFn sets no heading on its points but the last: give each the way to the next
        xy = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        o = m.poses[-1].pose.orientation
        pts = [(x, y, math.atan2(n[1] - y, n[0] - x)) for (x, y), n in zip(xy, xy[1:])]
        pts.append(xy[-1] + (yaw_of((o.x, o.y, o.z, o.w)),))
        out.sendto(route_packet(in_base(thin(pts), (tr.translation.x, tr.translation.y),
                                        yaw_of((q.x, q.y, q.z, q.w)))), (ORIN, ORIN_PORT))

    node.create_subscription(Path, '/plan', plan, 2)
    node.create_subscription(Twist, '/cmd_vel', lambda c: out.sendto(
        CMD.pack(b'V', c.linear.x, c.linear.y, c.angular.z), (ORIN, ORIN_PORT)), 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def check():
    line = [(i * 0.05, 0.0, 0.0) for i in range(41)]          # 2 m at NavFn's 5 cm
    t = thin(line)
    assert len(t) == 21 and t[-1] == line[-1], len(t)
    assert thin(line[:1]) == line[:1]
    # robot at (1, 2) facing +y on the map: base_link <- map is yaw -90 deg, then t = -R(-90)(1, 2) = (-2, 1)
    (x, y, a), = in_base([(1.0, 3.0, math.pi / 2)], (-2.0, 1.0), -math.pi / 2)
    assert abs(x - 1.0) < 1e-9 and abs(y) < 1e-9 and abs(a) < 1e-9, (x, y, a)    # 1 m straight ahead
    pk = route_packet([(1.0, 2.0, 3.0)])
    assert pk[:1] == b'T' and struct.unpack('<3f', pk[1:]) == (1.0, 2.0, 3.0)
    print('shadow_link ok')


if __name__ == '__main__':
    check() if sys.argv[1:] == ['check'] else main()
