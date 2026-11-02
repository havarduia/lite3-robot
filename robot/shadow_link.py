"""shadow_link - Nav2's route and RPP's commands, to the Orin's MPPI running in shadow.

Step 3 of ~/nav_stack_migration.md: the Orin's MPPI (orin/bin/shadow_nav.py) follows the
same route as today's stack, and its commands are logged next to RPP's, never sent. Each
/plan goes over in map coordinates with where robot/locate.py last put him on the map and
when; the Orin turns it into its own frame with its odometry of that moment. Each /cmd_vel
goes over as it is. Nothing here changes what drives the robot.

Step 4, `drive`: started by env/start_nav2_map.sh when the switch says orin (NAV_STACK, or the
file ~/nav_stack). Nav2 here still plans and decides when a goal is over, but RPP and the
recoveries publish on /rpp/cmd_vel, which only goes to the Orin's log. MPPI's command comes back
from the Orin and is published on /cmd_vel, clamped to DRIVE_VX/DRIVE_WZ, and only while RPP is
following a route here (a cancel, estop or finished goal ends that within RPP_FRESH_S).
No MPPI datagram for MPPI_STALE_S: a stop.

Kept cheap: the Jetson is ~75% busy without it. A tf listener (every /tf, 50+ a second, in
Python) cost ~0.4 core and starved the panel into "lost on the way" stops (big_walk, 2026-10-28).

    python3 -m robot.shadow_link          while Nav2 runs (source env/lite3_env.sh first)
    python3 -m robot.shadow_link drive    MPPI drives (env/start_nav2_map.sh starts it)
    python3 -m robot.shadow_link check    self-check, no ROS
"""
import math
import signal
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
DRIVE_PORT = 8045       # MPPI's commands from the Orin (orin/bin/shadow_nav.py)
DRIVE_VX = (-0.2, 0.3)  # m/s: step 4 stays slow until the camera and sonars reach the Orin (step 5)
DRIVE_WZ = 0.8          # rad/s, MPPI's own wz_max
RPP_FRESH_S = 0.6       # RPP speaks at 5 Hz (controller_frequency) while it follows a route, late under load
MPPI_STALE_S = 0.3      # MPPI speaks at 10 Hz too
DRIVE_HZ = 20
STOP_TICKS = 10         # zeros sent after driving ends (best effort drops some), then quiet


def thin(points):
    """Every point at least STEP_M from the last kept one, and always the last."""
    kept = [points[0]]
    for p in points[1:-1]:
        if math.hypot(p[0] - kept[-1][0], p[1] - kept[-1][1]) >= STEP_M - 1e-6:
            kept.append(p)
    return (kept + [points[-1]] if len(points) > 1 else kept)[:MAX_POINTS]


def route_packet(when, pose, points):
    return ROUTE.pack(b'T', when, *pose) + struct.pack('<%df' % (3 * len(points)), *[v for p in points for v in p])


def _interrupt(*_):
    raise KeyboardInterrupt


def drive_cmd(mppi, mppi_age, rpp_age):
    """What to publish: MPPI's (vx, vy, wz) clamped, or zeros when RPP is not following a
    route here or MPPI has gone quiet."""
    if mppi is None or mppi_age > MPPI_STALE_S or rpp_age > RPP_FRESH_S:
        return 0.0, 0.0, 0.0
    vx, _, wz = mppi
    return min(max(vx, DRIVE_VX[0]), DRIVE_VX[1]), 0.0, min(max(wz, -DRIVE_WZ), DRIVE_WZ)


def main(drive=False):
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
    def rpp(c):
        here['rpp_at'] = time.monotonic()
        out.sendto(CMD.pack(b'V', c.linear.x, c.linear.y, c.angular.z), (ORIN, ORIN_PORT))

    node.create_subscription(Twist, '/rpp/cmd_vel' if drive else '/cmd_vel', rpp, qos_profile_sensor_data)
    if drive:
        pub = node.create_publisher(Twist, '/cmd_vel', qos_profile_sensor_data)
        back = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        back.bind(('', DRIVE_PORT))
        back.setblocking(False)
        d = {'mppi': None, 'at': 0.0, 'zeros': 0}

        def send(vx, vy, wz):
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
            pub.publish(t)

        def tick():
            while True:
                try:
                    data, (host, _) = back.recvfrom(64)
                except BlockingIOError:
                    break
                if host == ORIN and len(data) == CMD.size and data[:1] == b'V':
                    d['mppi'], d['at'] = CMD.unpack(data)[1:], time.monotonic()
            now = time.monotonic()
            if now - here.get('rpp_at', 0.0) <= RPP_FRESH_S:
                send(*drive_cmd(d['mppi'], now - d['at'], now - here['rpp_at']))
                d['zeros'] = STOP_TICKS
            elif d['zeros'] > 0:        # quiet afterwards, so the panel's own turns and walks get through
                send(0.0, 0.0, 0.0)
                d['zeros'] -= 1

        node.create_timer(1.0 / DRIVE_HZ, tick)
    # nav_stop() ends the group with INT, TERM, KILL; a bash script's background job ignores INT
    signal.signal(signal.SIGTERM, _interrupt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if drive:
            for _ in range(STOP_TICKS):
                send(0.0, 0.0, 0.0)
                time.sleep(0.02)


def check():
    line = [(i * 0.05, 0.0, 0.0) for i in range(41)]          # 2 m at NavFn's 5 cm
    t = thin(line)
    assert len(t) == 21 and t[-1] == line[-1], len(t)
    assert thin(line[:1]) == line[:1]
    pk = route_packet(12.5, (1.0, 2.0, 0.5), [(1.0, 2.0, 3.0)])
    assert ROUTE.unpack(pk[:ROUTE.size]) == (b'T', 12.5, 1.0, 2.0, 0.5)
    assert struct.unpack('<3f', pk[ROUTE.size:]) == (1.0, 2.0, 3.0)
    assert drive_cmd((0.5, 0.1, -2.0), 0.1, 0.1) == (0.3, 0.0, -0.8)         # clamped, no strafe
    assert drive_cmd((-0.5, 0.0, 0.2), 0.1, 0.1) == (-0.2, 0.0, 0.2)
    assert drive_cmd((0.2, 0.0, 0.1), 0.1, 0.1) == (0.2, 0.0, 0.1)
    assert drive_cmd((0.2, 0.0, 0.1), 0.5, 0.1) == (0.0, 0.0, 0.0)          # MPPI quiet
    assert drive_cmd((0.2, 0.0, 0.1), 0.1, 0.7) == (0.0, 0.0, 0.0)          # no route here
    assert drive_cmd(None, 0.0, 0.0) == (0.0, 0.0, 0.0)
    print('shadow_link ok')


if __name__ == '__main__':
    check() if sys.argv[1:] == ['check'] else main(drive=sys.argv[1:] == ['drive'])
