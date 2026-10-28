"""shadow_link - Nav2's route and RPP's commands, to the Orin's MPPI running in shadow.

Step 3 of ~/nav_stack_migration.md: the Orin's MPPI (orin/bin/shadow_nav.py) follows the
same route as today's stack, and its commands are logged next to RPP's, never sent. Each
/plan goes over in map coordinates with where robot/locate.py last put him on the map and
when; the Orin turns it into its own frame with its odometry of that moment. Each /cmd_vel
goes over as it is. Nothing here changes what drives the robot.

Kept cheap: the Jetson is ~75% busy without it. A tf listener (every /tf, 50+ a second, in
Python) cost ~0.4 core and starved the panel into "lost on the way" stops (big_walk, 2026-10-28).

    python3 -m robot.shadow_link          while Nav2 runs (source env/lite3_env.sh first)
    python3 -m robot.shadow_link check    self-check, no ROS
"""
import math
import socket
import struct
import sys
import time

from .lio_relay import ORIN, yaw_of
from .locate import LOCALIZED, TOPIC as LOCALIZED_TOPIC

ORIN_PORT = 8044
STEP_M = 0.1            # route points kept this far apart (NavFn gives one per 5 cm cell)
MAX_POINTS = 1000       # 100 m of route, 12 kB: one datagram
POSE_STALE_S = 1.5      # locate.py speaks twice a second; an older pose places no route
ROUTE = struct.Struct('<c4d')       # T, when (wall clock), base_link x y yaw on the map; float32 (x, y, yaw) follow
CMD = struct.Struct('<c3d')


def thin(points):
    """Every point at least STEP_M from the last kept one, and always the last."""
    kept = [points[0]]
    for p in points[1:-1]:
        if math.hypot(p[0] - kept[-1][0], p[1] - kept[-1][1]) >= STEP_M - 1e-6:
            kept.append(p)
    return (kept + [points[-1]] if len(points) > 1 else kept)[:MAX_POINTS]


def route_packet(when, pose, points):
    return ROUTE.pack(b'T', when, *pose) + struct.pack('<%df' % (3 * len(points)), *[v for p in points for v in p])


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Path
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64MultiArray

    rclpy.init()
    node = rclpy.create_node('shadow_link')
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    here = {}

    def localized(m):
        if len(m.data) >= 4 and int(m.data[0]) == LOCALIZED:
            here['pose'], here['at'] = tuple(m.data[1:4]), time.time()

    def plan(m):
        if not m.poses or 'pose' not in here or time.time() - here['at'] > POSE_STALE_S:
            return
        # NavFn sets no heading on its points but the last: give each the way to the next
        xy = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        o = m.poses[-1].pose.orientation
        pts = [(x, y, math.atan2(n[1] - y, n[0] - x)) for (x, y), n in zip(xy, xy[1:])]
        pts.append(xy[-1] + (yaw_of((o.x, o.y, o.z, o.w)),))
        out.sendto(route_packet(here['at'], here['pose'], thin(pts)), (ORIN, ORIN_PORT))

    # best effort takes either kind of publisher
    node.create_subscription(Float64MultiArray, LOCALIZED_TOPIC, localized, 2)
    node.create_subscription(Path, '/plan', plan, qos_profile_sensor_data)
    node.create_subscription(Twist, '/cmd_vel', lambda c: out.sendto(
        CMD.pack(b'V', c.linear.x, c.linear.y, c.angular.z), (ORIN, ORIN_PORT)), qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def check():
    line = [(i * 0.05, 0.0, 0.0) for i in range(41)]          # 2 m at NavFn's 5 cm
    t = thin(line)
    assert len(t) == 21 and t[-1] == line[-1], len(t)
    assert thin(line[:1]) == line[:1]
    pk = route_packet(12.5, (1.0, 2.0, 0.5), [(1.0, 2.0, 3.0)])
    assert ROUTE.unpack(pk[:ROUTE.size]) == (b'T', 12.5, 1.0, 2.0, 0.5)
    assert struct.unpack('<3f', pk[ROUTE.size:]) == (1.0, 2.0, 3.0)
    print('shadow_link ok')


if __name__ == '__main__':
    check() if sys.argv[1:] == ['check'] else main()
