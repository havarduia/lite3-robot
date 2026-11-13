#!/usr/bin/env python3
"""orin_nav - the Orin's Nav2 (Humble) and perception, joined over UDP (nav_stack_migration.md, step 5b).

Perception's robot/orin_nav.py serves the navigate_to_pose action there and passes each goal here;
this hands it to bt_navigator and sends back how it went, the route (/plan) and the command
(/cmd_vel: MPPI's, through the collision monitor). Perception publishes the command while its goal runs.

Also, for Nav2 here:
  odom -> base_link and /odom from FAST-LIO2 (/state_estimation, levelled by terrain_level);
  map -> odom from cloud_locate.py's fixes (sent to FIX_LOCAL_PORT too), each matched to the own pose
    at its FAST-LIO2 stamp: both come from the same /Odometry;
  perception's depth camera as /depth_grid (a StaticLayer in the local costmap) and /depth_points
    (the collision monitor), its sonars as /sonar/front and /sonar/rear (shadow_nav.py's way).
No command goes back while the camera, the sonars or /terrain_h are stale, and a goal is cancelled when
perception goes quiet (PERCEPTION_STALE_S): its sonars alone come 20 times a second.

    python3 orin_nav.py          system python, ROS sourced; started by nav.sh
    python3 orin_nav.py check    self-check, no ROS
"""
import collections
import math
import socket
import struct
import sys
import threading
import time

import numpy as np

from lio_relay import FIX, FIX_LOCAL_PORT, FIX_NO_MAP, FIX_OK, inv, join, twist, unpack_fix
from shadow_nav import (CMD, DEPTH, DRIVE_PORT, GRID_RES, JETSON, MATCH_S, PORT, POSES_KEPT, SENSORS_STALE_S,
                        SONAR, SONARS, base_of, depth_cells)

GOAL = struct.Struct('<cI3d')       # G, goal number, x y yaw on the map
CANCEL = struct.Struct('<cI')       # C, goal number
STATUS = struct.Struct('<cIB')      # A, goal number, action_msgs GoalStatus (0: not taken yet)
PATH = struct.Struct('<c')          # P; float32 (x, y) on the map follow
STEP_M = 0.1                        # route points kept this far apart
MAX_POINTS = 1000
PERCEPTION_STALE_S = 1.0
STATUS_S = 0.5                      # how often the running goal's status is told
MAP_TF_S = 0.1
MAP_TF_AHEAD_S = 0.3                # map -> odom stamped this far past the newest odom, as AMCL postdates
FIX_MATCH_S = 0.05                  # the own pose this near the fix's stamp (FAST-LIO2 is 10 Hz)
DEPTH_Z = 0.3                       # m: the camera's voxels as points this high, for the collision monitor
# m: only camera voxels ahead of his nose (0.3 m, plus half of STVL's 0.1 m voxel) count. The camera is
# for the lidar's blind spot in front of him; beside and under him is the lidar's job, and the camera can't
# see under him anyway: glare on the bridge floor (bridge_nov_4_2) showed up 0.5 m ahead, he stepped onto
# it and stood stuck ~20 s while it stayed in the camera's memory.
NOSE_X = 0.35


def map_odom(fix, poses):
    """map -> odom from a fix (base_link on the map at a FAST-LIO2 stamp) and the own
    (stamp, x, y, yaw) poses in odom; None if none is near enough that stamp."""
    then = min(poses, key=lambda p: abs(p[0] - fix['stamp']), default=None)
    if then is None or abs(then[0] - fix['stamp']) > FIX_MATCH_S:
        return None
    x, y, _, yaw = fix['pose']
    return join((x, y, yaw), inv(then[1:]))


def ahead(rel):
    """Camera voxel centres around base_link (n x 2), only those ahead of his nose."""
    rel = np.asarray(rel, np.float32).reshape(-1, 2)
    return rel[rel[:, 0] > NOSE_X]


def thin(xy):
    kept = xy[:1]
    for p in xy[1:-1]:
        if math.hypot(p[0] - kept[-1][0], p[1] - kept[-1][1]) >= STEP_M - 1e-6:
            kept.append(p)
    return (kept + xy[-1:] if len(xy) > 1 else kept)[:MAX_POINTS]


def main():
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from rclpy.time import Time
    from rclpy.duration import Duration
    from sensor_msgs.msg import PointCloud2, PointField, Range
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    rclpy.init()
    node = rclpy.create_node('orin_nav')
    log = node.get_logger()
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    tf_pub = TransformBroadcaster(node)
    latched = QoSProfile(depth=1)
    latched.durability = DurabilityPolicy.TRANSIENT_LOCAL      # the StaticLayer subscribes so
    grid_pub = node.create_publisher(OccupancyGrid, '/depth_grid', latched)
    points_pub = node.create_publisher(PointCloud2, '/depth_points', 2)
    sonar_pubs = [node.create_publisher(Range, '/sonar/' + n, 10) for n, _, _ in SONARS]
    static_tf = StaticTransformBroadcaster(node)
    tfs = []
    for name, x, yaw in SONARS:
        t = TransformStamped()
        t.header.frame_id, t.child_frame_id = 'base_link', 'sonar_' + name
        t.transform.translation.x = x
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        tfs.append(t)
    static_tf.sendTransform(tfs)
    nav = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    s = {'prev': None, 'stamp': None, 'map_odom': None, 'heard': -1e9, 'depth_at': -1e9, 'terrain_at': -1e9,
         'sonar_at': [-1e9, -1e9], 'blind': None,
         # the goal: its number, the handle once taken, its status
         'seq': None, 'handle': None, 'status': 0}
    by_stamp = collections.deque(maxlen=POSES_KEPT)     # (FAST-LIO2 stamp, x, y, yaw): for the fixes
    by_clock = collections.deque(maxlen=POSES_KEPT)     # (wall clock, ...): for perception's camera
    back = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tell = lambda data: back.sendto(data, (JETSON, DRIVE_PORT))     # noqa: E731

    def odom(m):
        p, o = m.pose.pose.position, m.pose.pose.orientation
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        x, y, yaw = base_of((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        now = (t, x, y, yaw)
        vx, vy, wz = twist(s['prev'], now) if s['prev'] and 0 < t - s['prev'][0] < 1.0 else (0.0, 0.0, 0.0)
        s['prev'], s['stamp'] = now, m.header.stamp
        by_stamp.append(now)
        by_clock.append((time.time(), x, y, yaw))
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

    def take_fix(f):
        if f['state'] == FIX_NO_MAP:
            if s['map_odom'] is not None:
                log.warn('cloud_locate has no map: no map -> odom')
            s['map_odom'] = None
        elif f['state'] == FIX_OK:
            m = map_odom(f, by_stamp)
            if m is not None:
                if s['map_odom'] is None:
                    log.info('map -> odom from cloud_locate')
                s['map_odom'] = m

    def map_tf():
        if s['map_odom'] is None or s['stamp'] is None:
            return
        x, y, yaw = s['map_odom']
        t = TransformStamped()
        t.header.stamp = (Time.from_msg(s['stamp']) + Duration(seconds=MAP_TF_AHEAD_S)).to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        tf_pub.sendTransform(t)

    # --- goals -----------------------------------------------------------------

    def goal(seq, x, y, yaw):
        if seq == s['seq']:
            return                      # sent again until told it was taken
        if not nav.server_is_ready():
            return                      # Nav2 still starting: the next one sent tries again
        s['seq'], s['handle'], s['status'] = seq, None, 0
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id, g.pose.header.stamp = 'map', node.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        log.info('goal %d: (%.2f, %.2f, %.0f deg)' % (seq, x, y, math.degrees(yaw)))
        nav.send_goal_async(g).add_done_callback(lambda fut: taken(seq, fut.result()))

    def taken(seq, handle):
        if seq != s['seq']:
            return
        if handle is None or not handle.accepted:
            # bt_navigator turns goals down until it is active; perception gives up in its own time
            log.warn('goal %d turned down; taken again when sent again' % seq)
            s['seq'] = None
            return
        s['handle'], s['status'] = handle, GoalStatus.STATUS_EXECUTING
        handle.get_result_async().add_done_callback(lambda fut: ended(seq, fut.result().status))

    def ended(seq, status):
        if seq == s['seq']:
            s['status'] = status
            log.info('goal %d: status %d' % (seq, status))
            status_tick()

    def cancel(seq=None):
        if (seq is None or seq == s['seq']) and s['handle'] is not None and s['status'] == GoalStatus.STATUS_EXECUTING:
            s['handle'].cancel_goal_async()
            s['status'] = GoalStatus.STATUS_CANCELING

    def status_tick():
        if s['seq'] is not None:
            tell(STATUS.pack(b'A', s['seq'], s['status']))

    def plan(m):
        if s['status'] not in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING) or not m.poses:
            return
        xy = thin([(p.pose.position.x, p.pose.position.y) for p in m.poses])
        tell(PATH.pack(b'P') + np.array(xy, np.float32).tobytes())

    def command(c):
        now = time.monotonic()
        blind = now - min(s['depth_at'], s['terrain_at'], *s['sonar_at']) > SENSORS_STALE_S
        if blind != s['blind']:
            s['blind'] = blind
            # two calls: rclpy raises if one call site logs at two severities, and that ended this node
            # the first time the gate tripped
            if blind:
                log.warn('camera, sonars or /terrain_h stopped coming: no commands go back')
            else:
                log.info('camera, sonars and /terrain_h coming')
        if not blind:
            tell(CMD.pack(b'V', c.linear.x, c.linear.y, c.angular.z))

    def watch():
        if time.monotonic() - s['heard'] > PERCEPTION_STALE_S and s['status'] == GoalStatus.STATUS_EXECUTING:
            log.warn('perception went quiet: goal %d cancelled' % s['seq'])
            cancel()

    # --- perception's camera and sonars (shadow_nav.py's) --------------------

    def depth(when, rel):
        then = min(by_clock, key=lambda p: abs(p[0] - when), default=None)
        if then is None or abs(then[0] - when) > MATCH_S:
            return
        pts = [join(then[1:], (float(x), float(y), 0.0))[:2] for x, y in ahead(rel)]
        ox, oy, n, g = depth_cells(then[1:], pts)
        stamp = node.get_clock().now().to_msg()
        m = OccupancyGrid()
        m.header.stamp, m.header.frame_id = stamp, 'odom'
        m.info.resolution, m.info.width, m.info.height = GRID_RES, n, n
        m.info.origin.position.x, m.info.origin.position.y = ox, oy
        m.info.origin.orientation.w = 1.0
        m.data = g.ravel().tolist()
        grid_pub.publish(m)
        c = PointCloud2()
        # the odom stamp: the monitor looks odom -> base_link up at it (a later "now" would wait)
        c.header.stamp, c.header.frame_id = s['stamp'] or stamp, 'odom'
        c.height, c.width = 1, len(pts)
        c.fields = [PointField(name=a, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, a in enumerate('xyz')]
        c.point_step, c.row_step, c.is_dense = 12, 12 * len(pts), True
        xyz = np.zeros((len(pts), 3), np.float32)
        if pts:
            xyz[:, :2], xyz[:, 2] = pts, DEPTH_Z
        c.data = xyz.tobytes()
        points_pub.publish(c)
        s['depth_at'] = time.monotonic()

    def sonar(i, rng, lo, hi, fov):
        if s['stamp'] is None:
            return
        r = Range()
        # the newest odom stamp, not now(): see shadow_nav.py (mppi_drive3)
        r.header.stamp, r.header.frame_id = s['stamp'], 'sonar_' + SONARS[i][0]
        r.radiation_type, r.field_of_view = Range.ULTRASOUND, fov
        r.min_range, r.max_range, r.range = lo, hi, rng
        sonar_pubs[i].publish(r)
        s['sonar_at'][i] = time.monotonic()

    inbox = collections.deque(maxlen=200)       # datagrams for the ROS thread

    def read(port, host_ok):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', port))
        while True:
            data, (host, _) = sock.recvfrom(65535)
            if host_ok(host) and data:
                inbox.append(data)

    def take():
        latest = {}         # of the camera and each sonar only the newest counts
        while inbox:
            data = inbox.popleft()
            k = data[:1]
            if k == b'L' and len(data) == FIX.size:
                take_fix(unpack_fix(data))
                continue
            s['heard'] = time.monotonic()
            if k == b'G' and len(data) == GOAL.size:
                goal(*GOAL.unpack(data)[1:])
            elif k == b'C' and len(data) == CANCEL.size:
                cancel(CANCEL.unpack(data)[1])
            elif k == b'D' and len(data) >= DEPTH.size and (len(data) - DEPTH.size) % 8 == 0:
                latest['D'] = (DEPTH.unpack(data[:DEPTH.size])[1],
                               np.frombuffer(data, np.float32, offset=DEPTH.size).reshape(-1, 2))
            elif k == b'S' and len(data) == SONAR.size:
                _, i, *r = SONAR.unpack(data)
                if i in (0, 1):
                    latest[i] = r
        if 'D' in latest:
            depth(*latest.pop('D'))
        for i, r in latest.items():
            sonar(i, *r)

    node.create_subscription(Odometry, '/state_estimation', odom, 20)
    node.create_subscription(Twist, '/cmd_vel', command, 10)
    node.create_subscription(Path, '/plan', plan, 2)
    # the lidar's obstacles (terrainAnalysis -> terrain_level), not decoded: the costmap keeps its old
    # marks when they stop, and odometry goes on
    node.create_subscription(PointCloud2, '/terrain_h', lambda _: s.__setitem__('terrain_at', time.monotonic()), 2,
                             raw=True)
    node.create_timer(0.05, take)               # the ROS calls stay on the spinning thread
    node.create_timer(MAP_TF_S, map_tf)
    node.create_timer(STATUS_S, status_tick)
    node.create_timer(0.2, watch)
    threading.Thread(target=read, args=(PORT, lambda h: h == JETSON), daemon=True).start()
    threading.Thread(target=read, args=(FIX_LOCAL_PORT, lambda h: h == '127.0.0.1'), daemon=True).start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):      # SIGTERM; anything else shows its traceback
        pass
    finally:
        cancel()


def check():
    from lio_relay import pack_fix
    # he is at (1, 2) facing +y in odom at stamp 10.0, and the fix puts him at (5, 5) facing +x
    # on the map: map -> odom takes odom (1, 2) to (5, 5), and a point 1 m ahead, (1, 3), to (6, 5)
    poses = [(9.9, 0.0, 0.0, 0.0), (10.0, 1.0, 2.0, math.pi / 2)]
    f = unpack_fix(pack_fix(11.0, 10.0, (5.0, 5.0, 0.2, 0.0), 0.9, 0.8, 0.05, (), (0, 0, 0), FIX_OK))
    m = map_odom(f, poses)
    for odom_pt, map_pt in (((1.0, 2.0), (5.0, 5.0)), ((1.0, 3.0), (6.0, 5.0))):
        x, y, _ = join(m, odom_pt + (0.0,))
        assert abs(x - map_pt[0]) < 1e-9 and abs(y - map_pt[1]) < 1e-9, (x, y)
    f['stamp'] = 10.5
    assert map_odom(f, poses) is None            # no own pose of that moment
    assert len(thin([(i * 0.05, 0.0) for i in range(41)])) == 21
    # under him, at his nose, beside and behind him: dropped; ahead of the nose: kept
    kept = ahead([(0.0, 0.0), (0.33, 0.1), (0.0, 0.6), (-1.0, 0.0), (0.4, 0.0), (1.0, 1.0)])
    assert np.allclose(kept, [[0.4, 0.0], [1.0, 1.0]]), kept.tolist()
    assert ahead(np.zeros((0, 2), np.float32)).shape == (0, 2)
    print('orin_nav ok')


if __name__ == '__main__':
    check() if sys.argv[1:] == ['check'] else main()
