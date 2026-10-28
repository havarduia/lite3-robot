"""lio_relay - FAST-LIO2's pose from the Orin, onto the robot's ROS graph.

    python3 -m robot.lio_relay          the relay (source env/lite3_env.sh first)
    python3 -m robot.lio_relay check    self-check of the maths, no ROS
    python3 -m robot.lio_relay start    start FAST-LIO2 on the Orin (also: stop, state)
    python3 lio_relay.py send           on the ORIN: its end (env/lio-send.service)

Publishes /lio_odom (FAST-LIO2's pose as it comes), /odom_fused
(/leg_odom2 with FAST-LIO2's correction; plain /leg_odom2 whenever the Orin
is silent or its pose jumps), /scan and /scan_walls (the lidar flattened
to 2D by the Orin's lidar-scan.service: low for Nav2's costmaps, high for
finding itself on a floor map) and the odom -> base_link transform for Nav2.

The Orin is Humble with Fast DDS on domain 42; the robot's nodes are
CycloneDDS on domain 0 and crash on meeting it. So this file runs on the
Orin too, as `send`, and passes each pose and scan over as a UDP datagram.
README.md section 10.7.
"""
import array
import collections
import json
import math
import socket
import struct
import sys
import threading
import time
import urllib.error
import urllib.request

LIO_TOPIC = '/Odometry'     # FAST-LIO2: pose of its `body` (the lidar's IMU) in `camera_init`
OUT_TOPIC = '/lio_odom'     # the same motion as odom -> base_link, shaped like /leg_odom2
ORIN = '192.168.1.5'
ROBOT = ('192.168.1.103', 8042)     # the perception computer and the port this relay listens on
POSE = struct.Struct('<c9d')        # P, when sent (the Orin's clock), stamp, position xyz, quaternion xyzw
SCAN = struct.Struct('<c4d')        # S or W, angle_min, angle_increment, range_min, range_max; the ranges follow
# base_link -> lidar, as the Orin's lidar-scan.service has it. The pitch is
# measured (20.4 deg nose-down); x, y, z are good to about 2 cm (2026-10-09).
MOUNT_XYZ = (0.20, 0.0, 0.14)
MOUNT_PITCH = 0.356
RETRY_S = 2.0
ORIN_PANEL = 'http://192.168.1.5:8000'      # the Orin's web panel; FAST-LIO2 is its Live run
LEG_TOPIC = 'leg_odom2'
FUSED_TOPIC = '/odom_fused'
LEGS_KEPT = 100         # leg poses remembered, 2 s of them: a pose older than that is not used
CLOCK_DRIFT = 2e-4      # s per pose the two computers' clocks may drift apart
# Same names both sides; in base_link, by the Orin's mount transform. S is cut
# low (0.15 m under to 1.0 m over base_link): what he could walk into. W is cut
# above the furniture (0.9 to 1.7 m over): the walls a floor map shows.
SCANS = {b'S': '/scan', b'W': '/scan_walls'}
# The 3D localizer on the Orin (robot/cloud_locate.py): its fixes come here as 'L' (base_link's pose on
# the map at a FAST-LIO2 stamp, how well it fits, a state); "I am here" and the floor's cloud go to it.
FIX = struct.Struct('<cdd4d3dB3dB')     # L, sent (Orin clock), stamp, x y z yaw, fitness, walls, rmse, weak bits, var x y yaw, state
PLACE = struct.Struct('<c3d')           # I, base_link x, y, yaw on the map
CLOUD_PORT = 8043                       # where cloud_locate.py listens on the Orin, for 'I' and 'M' (the cloud's path)
FIX_AXES = ('x', 'y', 'z', 'yaw')
CLOUD_FIX_TOPIC = '/cloud_fix'       # the fixes as map -> odom, for robot/locate.py (CloudFix.take's row)
MAP_TF_EVERY = 5                    # map -> odom with every 5th leg message: 10 Hz
FIX_OK, FIX_NO_MAP, FIX_NO_LIO, FIX_TILTED, FIX_SEARCHING, FIX_REFUSED = range(6)
FIX_STATES = ('ok', 'no map', 'no FAST-LIO2', 'tilted', 'searching', 'refused')
SCAN_DELAY_S = 0.1      # a scan gathers 0.1 s of points before it is sent; not measured
GAP_S = 1.0             # longer than this between poses: a new run, or it stalled. A few go missing on the way
STEP_TOL = (0.15, 0.2)  # m, rad: a step differing more than this from the legs' is not believed
# FAST-LIO2 is started from here, not from the Orin: it has to start with the
# robot standing still, and only this side knows whether he is.
STANDING = 6            # basic state, first number of /robot_state_debug
STILL = (0.02, 0.02)    # m, rad over the two seconds remembered
CHECK_S = 5.0
START_RETRY_S = 30.0    # it needs a few seconds to come up; do not ask again before this


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def qconj(q):
    return (-q[0], -q[1], -q[2], q[3])


def rot(q, v):
    return qmul(qmul(q, (v[0], v[1], v[2], 0.0)), qconj(q))[:3]


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


Q_MOUNT = (0.0, math.sin(MOUNT_PITCH / 2), 0.0, math.cos(MOUNT_PITCH / 2))


def to_base(p, q):
    """The lidar's pose since FAST-LIO started -> base_link's pose since then.
    ponytail: takes the body as level at that start (the Live tab asks for
    standing still); level it with /lio/gravity if runs start otherwise."""
    q_base = qmul(qmul(Q_MOUNT, q), qconj(Q_MOUNT))
    moved = rot(Q_MOUNT, p)
    lever = rot(q_base, MOUNT_XYZ)
    return tuple(MOUNT_XYZ[i] + moved[i] - lever[i] for i in range(3)), q_base


def twist(prev, now):
    """(vx, vy, wz) in base_link from two (t, x, y, yaw) samples."""
    dt = now[0] - prev[0]
    if dt <= 0:
        return 0.0, 0.0, 0.0
    dx, dy = now[1] - prev[1], now[2] - prev[2]
    c, s = math.cos(now[3]), math.sin(now[3])
    turn = math.atan2(math.sin(now[3] - prev[3]), math.cos(now[3] - prev[3]))
    return (c * dx + s * dy) / dt, (-s * dx + c * dy) / dt, turn / dt


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def join(a, b):
    """2D poses (x, y, yaw): b, given in frame a, in a's parent."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]


def inv(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2]


class Fuse:
    """Leg odometry corrected by FAST-LIO2. `fix` maps a leg pose to the fused
    one and only changes while FAST-LIO2's steps agree with the legs', so the
    fused pose never jumps: not on a divergence, a stall or a new run.
    ponytail: a slow drift in FAST-LIO2 passes; compare over seconds if it shows."""

    def __init__(self):
        self.fix = (0.0, 0.0, 0.0)
        self.anchor = None      # maps a FAST-LIO2 pose to the fused one
        self.prev = None        # (t, lio, leg) last taken
        self.using = False

    def lio(self, t, lio, leg):
        """A FAST-LIO2 pose and the leg pose of the same moment."""
        ok = False
        if self.prev and t - self.prev[0] < GAP_S:
            a, b = join(inv(self.prev[1]), lio), join(inv(self.prev[2]), leg)
            ok = (math.hypot(a[0] - b[0], a[1] - b[1]) < STEP_TOL[0]
                  and abs(wrap(a[2] - b[2])) < STEP_TOL[1])
        if not ok:              # the legs carried that stretch: go on from where they put us
            self.anchor = join(join(self.fix, leg), inv(lio))
        self.fix = join(join(self.anchor, lio), inv(leg))
        self.prev, self.using = (t, lio, leg), ok

    def pose(self, leg):
        x, y, yaw = join(self.fix, leg)
        return x, y, wrap(yaw)


class CloudFix:
    """The Orin's 3D fixes (robot/cloud_locate.py) as map -> odom. take() each one with
    /odom_fused's pose at its moment; map_odom is the newest accepted, None while
    the Orin has no map (Nav2 on AMCL, which owns map -> odom then)."""

    def __init__(self):
        self.map_odom = None

    def take(self, fix, odom_then):
        """The row for /cloud_fix: state, map -> odom (x y yaw), walls, rmse, weak bits,
        spread (m, rad), fitness."""
        state = fix['state']
        if state == FIX_NO_MAP:
            self.map_odom = None
        elif state == FIX_OK and odom_then is not None:
            x, y, _, yaw = fix['pose']
            self.map_odom = join((x, y, yaw), inv(odom_then))
        bits = sum(1 << i for i, a in enumerate(FIX_AXES) if a in fix['weak'])
        vx, vy, vyaw = fix['var']
        return ([float(state)] + list(self.map_odom or (math.nan,) * 3)
                + [fix['walls'], fix['rmse'], float(bits), math.sqrt(max(vx, vy, 0.0)), math.sqrt(max(vyaw, 0.0)),
                   fix['fitness']])


def place(x, y, yaw):
    """Tell the Orin's localizer where base_link is on the map ("I am here")."""
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(PLACE.pack(b'I', x, y, yaw), (ORIN, CLOUD_PORT))


def cloud(path):
    """Tell the Orin's localizer which cloud to load (a .ply or a run folder on the Orin); '' unloads."""
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(b'M' + path.encode(), (ORIN, CLOUD_PORT))


def live(what, **options):
    """The Orin's FAST-LIO2 run: live('start'), live('stop', name='hall'),
    live('state'). Returns the panel's answer; start it with the robot
    standing still. A stop saves the run's map on the Orin."""
    data = None if what == 'state' else json.dumps(options).encode()
    req = urllib.request.Request('%s/api/live/%s' % (ORIN_PANEL, what), data,
                                 {'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:     # 409: it is already in that state
        return json.load(e)


def leg_at(legs, when):
    """The leg pose remembered nearest to `when` (this computer's clock), or
    None if `when` is older than anything remembered."""
    if not legs or when < legs[0][0] - 0.05:
        return None
    return min(legs, key=lambda g: abs(g[0] - when))[1]


def should_start(basic, legs, lio_age):
    """Start FAST-LIO2 now? Standing, still for the two seconds in `legs`, and
    no pose from it lately (lio_age: seconds, None = never)."""
    if basic != STANDING or len(legs) < legs.maxlen:
        return False
    a, b = legs[0][1], legs[-1][1]
    still = math.hypot(b[0] - a[0], b[1] - a[1]) < STILL[0] and abs(wrap(b[2] - a[2])) < STILL[1]
    return still and (lio_age is None or lio_age > GAP_S)


def scan_packet(kind, angle_min, angle_increment, range_min, range_max, ranges):
    """A scan as one datagram; scan_fields() reads it back. The ranges go as
    the float32s they already are: no work per beam at either end. Both
    computers are little-endian."""
    return SCAN.pack(kind, angle_min, angle_increment, range_min, range_max) + array.array('f', ranges).tobytes()


def scan_fields(data):
    """A scan datagram -> (angle_min, angle_increment, range_min, range_max, ranges)."""
    return SCAN.unpack_from(data)[1:] + (array.array('f', data[SCAN.size:]),)


def pack_fix(sent, stamp, pose, fitness, walls, rmse, weak, var, state):
    bits = sum(1 << i for i, a in enumerate(FIX_AXES) if a in weak)
    return FIX.pack(b'L', sent, stamp, *pose, fitness, walls, min(rmse, 1e3), bits, *var, state)


def unpack_fix(data):
    v = FIX.unpack(data)
    return {'sent': v[1], 'stamp': v[2], 'pose': v[3:7], 'fitness': v[7], 'walls': v[8], 'rmse': v[9],
            'weak': [a for i, a in enumerate(FIX_AXES) if v[10] >> i & 1], 'var': v[11:14], 'state': v[14]}


def well_formed(data):
    """Is this datagram a pose, a scan or a 3D fix, whole?"""
    if data[:1] == b'P':
        return len(data) == POSE.size
    if data[:1] == b'L':
        return len(data) == FIX.size
    return data[:1] in SCANS and len(data) > SCAN.size and (len(data) - SCAN.size) % 4 == 0


def send():
    """On the Orin, on its own DDS: each FAST-LIO2 pose and each scan to the relay."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def pose(m):
        p, q, s = m.pose.pose.position, m.pose.pose.orientation, m.header.stamp
        out.sendto(POSE.pack(b'P', time.time(), s.sec + s.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w), ROBOT)

    def scan(kind):
        return lambda m: out.sendto(
            scan_packet(kind, m.angle_min, m.angle_increment, m.range_min, m.range_max, m.ranges), ROBOT)

    rclpy.init()
    node = rclpy.create_node('lio_send')
    node.create_subscription(Odometry, LIO_TOPIC, pose, qos_profile_sensor_data)
    for kind, topic in SCANS.items():
        node.create_subscription(LaserScan, topic, scan(kind), qos_profile_sensor_data)
    rclpy.spin(node)


def main():
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.clock import Clock, ClockType
    from rclpy.duration import Duration
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Float64MultiArray, Int32MultiArray
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    node = rclpy.create_node('lio_relay')
    lio_pub = node.create_publisher(Odometry, OUT_TOPIC, 10)
    fused_pub = node.create_publisher(Odometry, FUSED_TOPIC, 10)
    scan_pubs = {kind: node.create_publisher(LaserScan, topic, 5) for kind, topic in SCANS.items()}
    # odom -> base_link for Nav2, from here: the vendor's node for it cost a fifth of a core
    tf_pub = TransformBroadcaster(node)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
    # map -> odom from the Orin's 3D fixes, while it has a map (Nav2 with LOCALIZER=cloud)
    fix_pub = node.create_publisher(Float64MultiArray, CLOUD_FIX_TOPIC, 5)
    map_tf = TransformStamped()
    map_tf.header.frame_id, map_tf.child_frame_id = 'map', 'odom'
    cloud_fix = CloudFix()
    steady = Clock(clock_type=ClockType.STEADY_TIME)    # what /leg_odom2 is stamped with
    box = {}                # 'lio': the newest (arrival, sent, stamp, position, quaternion)

    def send_scan(data):
        if state['basic'] != STANDING:      # lying down, the floor is inside the scan's height band
            return
        m = LaserScan()
        # the Orin's clock is not this graph's: stamp it as it was a moment ago here
        m.header.stamp = (steady.now() - Duration(seconds=SCAN_DELAY_S)).to_msg()
        m.header.frame_id = 'base_link'
        m.angle_min, m.angle_increment, m.range_min, m.range_max, m.ranges = scan_fields(data)
        m.angle_max = m.angle_min + m.angle_increment * (len(m.ranges) - 1)
        m.scan_time = 0.1
        scan_pubs[data[:1]].publish(m)

    def read():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while True:
            try:
                sock.bind(('', ROBOT[1]))
                break
            except OSError as e:        # another relay has the port
                print('lio_relay: cannot listen for the Orin:', e, file=sys.stderr)
                time.sleep(RETRY_S)
        while True:
            data, (host, _) = sock.recvfrom(65535)
            if host != ORIN or not well_formed(data):
                continue
            if data[:1] in SCANS:
                send_scan(data)
            elif data[:1] == b'L':
                box['fix'] = unpack_fix(data)
            else:
                v = POSE.unpack(data)[1:]
                box['lio'] = (time.monotonic(), v[0], v[1]) + to_base(v[2:5], v[5:])

    fuse = Fuse()
    legs = collections.deque(maxlen=LEGS_KEPT)      # (arrival, leg pose)
    state = {'seen': None, 'prev': None, 'using': False, 'basic': None, 'asked': -START_RETRY_S,
             'ahead': None,     # this clock minus the Orin's, from the quickest datagram so far
             'behind': 0.0,     # how old the newest pose was when the Orin sent it
             'heard': None,     # when the last usable pose arrived
             'ticks': 0}

    def take_lio(s):
        arrived, sent, t, p, q = s
        now = (t, p[0], p[1], yaw_of(q))
        prev, state['prev'] = state['prev'], now
        # a gap is a new FAST-LIO run, with a new origin: no speed across it
        vx, vy, wz = twist(prev, now) if prev and t - prev[0] < 1.0 else (0.0, 0.0, 0.0)
        m = Odometry()
        m.header.stamp = steady.now().to_msg()
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        pos, ori = m.pose.pose.position, m.pose.pose.orientation
        pos.x, pos.y, pos.z = p
        ori.x, ori.y, ori.z, ori.w = q
        m.twist.twist.linear.x, m.twist.twist.linear.y = vx, vy
        m.twist.twist.angular.z = wz
        lio_pub.publish(m)
        # Each pose is matched by its own stamp to where the legs were THEN, not to
        # where they are now. The clocks are compared by when the Orin SENT it, not
        # by the stamp: FAST-LIO2 can fall minutes behind (2026-10-09, 35 min), and
        # a relay started then would take that lag for a clock difference. A pose
        # older than the legs remembered finds no match and is not used.
        ahead = arrived - sent if state['ahead'] is None else min(state['ahead'] + CLOCK_DRIFT, arrived - sent)
        state['ahead'], state['behind'] = ahead, sent - t
        then = leg_at(legs, t + ahead)
        if then is not None:
            state['heard'] = arrived
            fuse.lio(t, now[1:], then)

    def leg(m):
        pos, ori = m.pose.pose.position, m.pose.pose.orientation
        q = (ori.x, ori.y, ori.z, ori.w)
        pose = (pos.x, pos.y, yaw_of(q))
        arrived = time.monotonic()
        legs.append((arrived, pose))
        s = box.get('lio')
        if s is not None and s is not state['seen']:
            state['seen'] = s
            take_lio(s)
        using = fuse.using and arrived - state['heard'] < GAP_S
        if using != state['using']:
            state['using'] = using
            print('lio_relay: /odom_fused is', 'corrected by FAST-LIO2' if using
                  else 'leg odometry alone', file=sys.stderr)
        # the leg message itself, moved: its stamp, height, tilt and speeds stay
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        pos.x, pos.y, _ = fuse.pose(pose)
        half = fuse.fix[2] / 2
        ori.x, ori.y, ori.z, ori.w = qmul((0.0, 0.0, math.sin(half), math.cos(half)), q)
        fused_pub.publish(m)
        tf.header.stamp = m.header.stamp
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = pos.x, pos.y, pos.z
        tf.transform.rotation = ori
        tf_pub.sendTransform(tf)
        f = box.pop('fix', None)
        if f is not None:
            # base_link on the map at the fix's moment, against /odom_fused at that moment
            then = leg_at(legs, f['stamp'] + state['ahead']) if state['ahead'] is not None else None
            fix_pub.publish(Float64MultiArray(data=cloud_fix.take(f, fuse.pose(then) if then else None)))
        state['ticks'] += 1
        if cloud_fix.map_odom is not None and state['ticks'] % MAP_TF_EVERY == 0:
            x, y, yaw = cloud_fix.map_odom
            map_tf.header.stamp = m.header.stamp
            map_tf.transform.translation.x, map_tf.transform.translation.y = x, y
            map_tf.transform.rotation.z, map_tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            tf_pub.sendTransform(map_tf)

    def start():
        try:
            print('lio_relay: asked the Orin to start FAST-LIO2:', live('start'), file=sys.stderr)
        except (OSError, ValueError) as e:      # Orin off, panel down, or not its answer
            print('lio_relay: could not reach the Orin panel:', e, file=sys.stderr)

    def check():
        now = time.monotonic()
        age = now - state['heard'] if state['heard'] else None
        if state['behind'] > GAP_S and now - box['lio'][0] < CHECK_S:     # and still sending them
            print('lio_relay: FAST-LIO2 is %.0f s behind; its poses are not used' % state['behind'], file=sys.stderr)
        if now - state['asked'] > START_RETRY_S and should_start(state['basic'], legs, age):
            state['asked'] = now
            threading.Thread(target=start, daemon=True).start()     # not in the 50 Hz path

    node.create_subscription(Odometry, LEG_TOPIC, leg, 1)
    # "I am here" goes to AMCL as ever, and to the Orin's localizer (ignored there without a map)
    node.create_subscription(PoseWithCovarianceStamped, '/initialpose', lambda m: place(
        m.pose.pose.position.x, m.pose.pose.position.y, yaw_of((m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                                                                 m.pose.pose.orientation.z, m.pose.pose.orientation.w))), 5)
    node.create_subscription(Int32MultiArray, '/robot_state_debug',
                             lambda m: state.__setitem__('basic', m.data[0]), 1)
    node.create_timer(CHECK_S, check)
    threading.Thread(target=read, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def demo():
    """Self-check of the pose maths: no ROS, no robot."""
    def near(a, b):
        return all(abs(x - y) < 1e-9 for x, y in zip(a, b))

    ident = (0.0, 0.0, 0.0, 1.0)
    assert near(to_base((0, 0, 0), ident)[0], (0, 0, 0))
    # the lidar going 1 m along its own nose-down x is the body going forward and down
    p, q = to_base((1, 0, 0), ident)
    assert near(p, (math.cos(MOUNT_PITCH), 0, -math.sin(MOUNT_PITCH))) and near(q, ident)
    # a body pose turned into what FAST-LIO would report for it, and back
    q_body = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))      # yaw 90 deg
    p_body = (1.0, 2.0, 0.0)
    lever = rot(q_body, MOUNT_XYZ)
    p_lidar = rot(qconj(Q_MOUNT), [p_body[i] + lever[i] - MOUNT_XYZ[i] for i in range(3)])
    q_lidar = qmul(qmul(qconj(Q_MOUNT), q_body), Q_MOUNT)
    p, q = to_base(p_lidar, q_lidar)
    assert near(p, p_body) and near(q, q_body) and abs(yaw_of(q) - math.pi / 2) < 1e-9
    # facing +y, 0.05 m further along +y and 0.1 rad of turn in 0.1 s
    assert near(twist((0.0, 0, 0, math.pi / 2), (0.1, 0, 0.05, math.pi / 2 + 0.1)),
                (0.5 * math.cos(0.1), -0.5 * math.sin(0.1), 1.0))
    # fusing. The two odometries count from different places and directions.
    def run(fuse, steps, lio, leg, lio_step, leg_step, t=0.0):
        for _ in range(steps):
            lio, leg, t = join(lio, lio_step), join(leg, leg_step), t + 0.1
            fuse.lio(t, lio, leg)
        return lio, leg, t

    f = Fuse()
    lio, leg = (5.0, 5.0, math.pi / 2), (0.0, 0.0, 0.0)
    f.lio(0.0, lio, leg)
    # they agree: the fused pose is the leg pose
    lio, leg, t = run(f, 10, lio, leg, (0.1, 0, 0.02), (0.1, 0, 0.02))
    assert near(f.pose(leg), leg) and f.using
    # the legs under-count by a tenth: the fused pose goes FAST-LIO2's distance
    before = f.pose(leg)
    lio, leg, t = run(f, 10, lio, leg, (0.1, 0, 0), (0.09, 0, 0), t)
    after = f.pose(leg)
    assert abs(math.hypot(after[0] - before[0], after[1] - before[1]) - 1.0) < 1e-9
    # FAST-LIO2 shoots away: the fused pose does not, and follows the legs
    held = f.fix
    lio, leg, t = run(f, 3, lio, leg, (5.0, 0, 0), (0.1, 0, 0), t)
    assert near(f.fix, held) and not f.using
    # it comes back, and so does a new run from a new origin after a silence
    lio, leg, t = run(f, 2, lio, leg, (0.1, 0, 0), (0.1, 0, 0), t)
    assert f.using and near(f.fix, held)
    f.lio(t + 10, (0.0, 0.0, 0.0), leg)
    assert near(f.fix, held) and not f.using
    # a pose is matched to where the legs were at its own time, however late it comes
    walk = collections.deque([(10.0 + i * 0.02, (i * 0.02, 0.0, 0.0)) for i in range(100)], maxlen=100)   # 1 m/s
    assert near(leg_at(walk, 10.5), (0.5, 0.0, 0.0)) and leg_at(walk, 9.0) is None
    # a scan survives the trip, its "nothing there" readings included
    sent = scan_packet(b'S', -3.14159, 0.00873, 0.4, 30.0, [1.25, float('inf'), 0.5])
    back = scan_fields(sent)
    assert back[:4] == (-3.14159, 0.00873, 0.4, 30.0) and list(back[4]) == [1.25, float('inf'), 0.5]
    # only whole poses and scans are taken
    assert well_formed(sent) and well_formed(b'W' + sent[1:]) and well_formed(POSE.pack(b'P', *[0.0] * 9))
    assert not well_formed(sent[:-1]) and not well_formed(b'P' + sent[1:]) and not well_formed(b'') \
        and not well_formed(SCAN.pack(b'S', 0, 0, 0, 0)) and not well_formed(b'X' * POSE.size)
    # starting FAST-LIO2: only standing, still, and with nothing coming from it
    still = collections.deque([(i * 0.02, (1.0, 2.0, 0.5)) for i in range(LEGS_KEPT)], maxlen=LEGS_KEPT)
    moving = collections.deque([(i * 0.02, (1.0 + i * 0.01, 2.0, 0.5)) for i in range(LEGS_KEPT)], maxlen=LEGS_KEPT)
    assert should_start(STANDING, still, None) and should_start(STANDING, still, 3.0)
    assert not should_start(STANDING, still, 0.1)       # it is running
    assert not should_start(STANDING, moving, None)     # walking
    assert not should_start(1, still, None)             # lying down
    assert not should_start(STANDING, collections.deque(list(still)[:10], maxlen=LEGS_KEPT), None)
    # 3D fixes: whole ones only, and turned into map -> odom against the odometry of their moment
    d = pack_fix(5.0, 4.9, (5.0, 5.0, 0.4, math.pi / 2), 0.95, 0.9, 0.05, ['x'], (0.01, 0.0004, 0.0001), FIX_OK)
    assert well_formed(d) and not well_formed(d[:-1]) and unpack_fix(d)['weak'] == ['x']
    c = CloudFix()
    row = c.take(unpack_fix(d), (1.0, 0.0, 0.0))          # odometry read (1, 0, 0) then
    x, y, yaw = join(c.map_odom, (3.0, 0.0, 0.0))         # 2 m on by odometry: 2 m on, on the map
    assert near((x, y, yaw), (5.0, 7.0, math.pi / 2)), (x, y, yaw)
    assert row[0] == FIX_OK and row[4] == 0.9 and row[6] == 1.0 and abs(row[7] - 0.1) < 1e-12 and abs(row[8] - 0.01) < 1e-12, row
    held = c.map_odom
    c.take(unpack_fix(pack_fix(6, 6, (9, 9, 0, 0), 0.4, 0.2, 0.1, [], (0, 0, 0), FIX_REFUSED)), (1.0, 0.0, 0.0))
    assert c.map_odom == held                              # refused: the last one stands
    c.take(unpack_fix(pack_fix(7, 7, (9, 9, 0, 0), 0.9, 0.9, 0.05, [], (0, 0, 0), FIX_OK)), None)
    assert c.map_odom == held                              # its moment not in the odometry kept: not used
    row = c.take(unpack_fix(pack_fix(8, 8, (0, 0, 0, 0), 0, 0, 0, FIX_AXES, (0, 0, 0), FIX_NO_MAP)), (1.0, 0.0, 0.0))
    assert c.map_odom is None and math.isnan(row[1])      # unloaded: no map -> odom from here (AMCL's job)
    print('lio_relay ok')


if __name__ == '__main__':
    word = ''.join(sys.argv[1:])
    if sys.argv[1:2] == ['map']:
        cloud(sys.argv[2] if len(sys.argv) > 2 else '')
    elif word in ('start', 'stop', 'state'):
        print(json.dumps(live(word), indent=1))
    else:
        {'send': send, 'check': demo}.get(word, main)()
