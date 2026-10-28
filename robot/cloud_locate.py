"""cloud_locate - where he is on a floor's 3D map, from FAST-LIO2's scans. Runs on the ORIN.

    python3 cloud_locate.py              the node (env/cloud-locate.service; copied to ~/bin with cloudreg.py)
    python3 -m robot.cloud_locate check  its own check, no ROS

FAST-LIO2 already knows his motion and which way is down; this places its
frame on the floor's map (the mapping walk's cloud) by matching its
registered scans about once a second (cloudreg.Tracker), and sends the
result to the relay on the perception computer as 'L' datagrams: base_link's
pose on the map at a FAST-LIO2 stamp, how well it fits, and a state. The
relay turns that into map -> odom. It listens on UDP COMMAND_PORT for
'M' (a cloud's path on this computer: load it) and 'I' (base_link's map
pose, from "I am here"). .planning/plans/cloud-localization.md, phase 1.
"""
import math
import os
import socket
import sys
import threading
import time

import numpy as np

try:
    from . import cloudreg
    from .lio_relay import (CLOUD_PORT as COMMAND_PORT, FIX, FIX_NO_LIO as NO_LIO, FIX_NO_MAP as NO_MAP,
                            FIX_OK as OK, FIX_REFUSED as REFUSED, FIX_SEARCHING as SEARCHING, FIX_STATES as STATES,
                            FIX_TILTED as TILTED, MOUNT_XYZ, PLACE, ROBOT, pack_fix, unpack_fix)
except ImportError:                             # on the Orin: flat in ~/bin
    import cloudreg
    from lio_relay import (CLOUD_PORT as COMMAND_PORT, FIX, FIX_NO_LIO as NO_LIO, FIX_NO_MAP as NO_MAP,
                           FIX_OK as OK, FIX_REFUSED as REFUSED, FIX_SEARCHING as SEARCHING, FIX_STATES as STATES,
                           FIX_TILTED as TILTED, MOUNT_XYZ, PLACE, ROBOT, pack_fix, unpack_fix)

FIX_S = 1.0             # a fix this often; FAST-LIO2 carries the pose in between
SMOOTH = 0.3            # a tracked fix moves the sent correction this far towards it: no 5 cm shake a second
SCAN_VOXEL_M = 0.1      # scans thinned to this on arrival: a 3 s stack stays small
MIN_SCANS = 10          # no fix from less than 1 s of scans
RESTART_S = 1.0         # the live run's age fell by more than this: FAST-LIO2 restarted, a new frame.
                        # Not a gap in /Odometry: a long search holds the callbacks up and its queue
                        # overflows (2026-10-15: a 10 s false restart after "I am here")
LIDAR_ABOVE_FLOOR = 0.46   # m: a guess's height is the map's floor under it plus this
FLOOR_NEAR_M = 1.5
TILT_MAX_DEG = 1.5      # its frame off level more than this (the Orin's live state says): no fixes
CLOUD_FILES = ('gravity_aligned.unflattened.ply', 'gravity_aligned.ply')   # in a run folder: slopes kept if flattened


def align_to(a, b):
    """The rotation taking direction a onto direction b."""
    a, b = np.asarray(a, float) / np.linalg.norm(a), np.asarray(b, float) / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


def quat_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def smooth(sent, new):
    """The sent correction moved SMOOTH of the way to a newly tracked one."""
    dyaw = math.atan2(math.sin(new[3] - sent[3]), math.cos(new[3] - sent[3]))
    return tuple(o + SMOOTH * (n - o) for o, n in zip(sent[:3], new[:3])) + (sent[3] + SMOOTH * dyaw,)


def cloud_file(path):
    """The cloud to load for 'M': a .ply as given, or a run's slam_output/latest folder."""
    if path.endswith('.ply'):
        return path
    for name in CLOUD_FILES:
        if os.path.exists(os.path.join(path, name)):
            return os.path.join(path, name)
    raise FileNotFoundError('no cloud in %s' % path)


class Locator:
    """The node's logic, without ROS. Fed FAST-LIO2's gravity, odometry and clouds
    (its raw frame), and commands; fix() gives the 'L' datagram to send."""

    def __init__(self, load=cloudreg.Map.from_ply):
        self.load, self.map, self.track = load, None, None
        self.A = None                   # levels FAST-LIO2's frame (its own gravity estimate)
        self.lidar = None               # (stamp, x, y, z, yaw): the lidar in the levelled frame, newest
        self.run_t = None               # the live run's age, as last seen
        self.sent_corr = None           # the correction as sent, smoothed
        self.tilt = 0.0
        self.diverged = False
        self.pending_place = None       # an 'I' that came before the map or FAST-LIO2

    # --- inputs ---------------------------------------------------------------
    def command(self, data):
        if data[:1] == b'M':
            path = data[1:].decode()
            # empty: unload (Nav2 on AMCL). Either way an old "I am here" is not this map's
            self.map = self.load(cloud_file(path)) if path else None
            self.track, self.pending_place, self.sent_corr = None, None, None
        elif data[:1] == b'I' and len(data) == PLACE.size:
            self.pending_place = PLACE.unpack(data)[1:]
        self._start()

    def on_gravity(self, g):
        """FAST-LIO2's gravity in its frame (pointing down)."""
        if self.A is None:
            self.A = align_to(-np.asarray(g, float), (0.0, 0.0, 1.0))

    def on_odom(self, stamp, pos, quat):
        if self.A is None:
            return
        p, R = self.A @ np.asarray(pos, float), self.A @ quat_R(*quat)
        self.lidar = (stamp, p[0], p[1], p[2], math.atan2(R[1, 0], R[0, 0]))
        self._start()

    def on_cloud(self, pts):
        if self.A is not None and self.track is not None:
            self.track.add(cloudreg.voxel(np.asarray(pts, float) @ self.A.T, SCAN_VOXEL_M))

    def live_state(self, tilt, diverged, run_t=None):
        """From the Orin's live-run state (lidar_web's /api/live/state): tilt, diverged
        or not running, and the run's age, whose fall is a new run."""
        self.tilt, self.diverged = tilt, diverged
        if run_t is not None:
            if self.run_t is not None and run_t < self.run_t - RESTART_S:
                self._restarted()
            self.run_t = run_t

    # --- the work ---------------------------------------------------------------
    def _restarted(self):
        """A new FAST-LIO2 run: a new frame. Start again from where he was on the map."""
        if self.track is not None and self.lidar is not None and self.sent_corr is not None:
            x, y, yaw = self.base_on_map(self.sent_corr)
            self.pending_place = (x, y, yaw)
        self.A, self.lidar, self.track, self.sent_corr = None, None, None, None

    def _start(self):
        """With a map, FAST-LIO2 and a place to start from: a tracker round that guess."""
        if self.map is None or self.lidar is None or self.pending_place is None:
            return
        x, y, yaw = self.pending_place
        mx, my, mz = MOUNT_XYZ
        lx, ly = x + mx * math.cos(yaw) - my * math.sin(yaw), y + mx * math.sin(yaw) + my * math.cos(yaw)
        _, ax, ay, az, ayaw = self.lidar
        dyaw = yaw - ayaw
        c, s = math.cos(dyaw), math.sin(dyaw)
        pts = self.map.all.data
        near = pts[np.hypot(pts[:, 0] - lx, pts[:, 1] - ly) < FLOOR_NEAR_M]
        lz = (np.percentile(near[:, 2], 5) if len(near) else 0.0) + LIDAR_ABOVE_FLOOR     # the floor here, any level
        guess = (lx - (c * ax - s * ay), ly - (s * ax + c * ay), lz - az, dyaw)
        self.track = cloudreg.Tracker(self.map, guess)
        self.sent_corr, self.pending_place = None, None

    def base_on_map(self, corr):
        """base_link's (x, y, yaw) on the map, by correction corr, at the newest odometry."""
        _, ax, ay, az, ayaw = self.lidar
        lx, ly, _, lyaw = cloudreg.compose(corr, (ax, ay, az, ayaw))
        mx, my, _ = MOUNT_XYZ
        return (lx - mx * math.cos(lyaw) + my * math.sin(lyaw), ly - mx * math.sin(lyaw) - my * math.cos(lyaw), lyaw)

    def fix(self, now):
        """One fix if one can be made; the 'L' datagram either way (state says why not)."""
        stamp = self.lidar[0] if self.lidar else 0.0
        if self.map is None:
            return pack_fix(now, stamp, (0, 0, 0, 0), 0, 0, 0, cloudreg.AXES, (0, 0, 0), NO_MAP)
        if self.lidar is None or self.diverged or self.track is None:
            return pack_fix(now, stamp, (0, 0, 0, 0), 0, 0, 0, cloudreg.AXES, (0, 0, 0), NO_LIO)
        # a run started tilted stays tilted; checked before he is placed only, since once he walks
        # the reading follows his own accelerations (23 of 208 fixes lost to it, 2026-10-15)
        if self.tilt > TILT_MAX_DEG and not self.track.placed:
            return pack_fix(now, stamp, (0, 0, 0, 0), 0, 0, 0, cloudreg.AXES, (0, 0, 0), TILTED)
        if len(self.track.scans) < MIN_SCANS:
            return pack_fix(now, stamp, (0, 0, 0, 0), 0, 0, 0, cloudreg.AXES, (0, 0, 0), SEARCHING)
        f = self.track.fix()
        if not self.track.placed:
            return pack_fix(now, stamp, (0, 0, 0, 0), f.fitness, f.wall_fitness, f.rmse, f.weak, (0, 0, 0), SEARCHING)
        c = self.track.corr
        if self.sent_corr is None or f.why.startswith('search'):
            self.sent_corr = c
        elif f.taken:
            self.sent_corr = smooth(self.sent_corr, c)
        x, y, yaw = self.base_on_map(self.sent_corr)
        z = cloudreg.compose(self.sent_corr, self.lidar[1:])[2]
        var = (float(f.cov[0, 0]), float(f.cov[1, 1]), float(f.cov[3, 3]))
        return pack_fix(now, stamp, (x, y, z, yaw), f.fitness, f.wall_fitness, f.rmse, f.weak, var,
                        OK if f.taken else REFUSED)


# --- the node ---------------------------------------------------------------------

def cloud_xyz(m):
    """x, y, z of a PointCloud2 as an Nx3 float array (any point layout)."""
    off = {f.name: f.offset for f in m.fields}
    raw = np.frombuffer(m.data, np.uint8).reshape(-1, m.point_step)
    return np.column_stack([raw[:, off[a]:off[a] + 4].copy().view('<f4')[:, 0] for a in 'xyz']).astype(float)


def main():
    import json
    import urllib.request
    import rclpy
    from geometry_msgs.msg import Vector3Stamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    loc = Locator()
    lock = threading.Lock()
    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd.bind(('0.0.0.0', COMMAND_PORT))
    cmd.settimeout(0.5)

    def commands():                                 # a map loads for seconds: not on the ROS thread
        while True:
            try:
                data, _ = cmd.recvfrom(4096)
            except socket.timeout:
                continue
            t = time.time()
            try:                                    # nothing a command does may stop this thread: it is the only ear
                with lock:
                    loc.command(data)
                if data[:1] == b'M':
                    print('cloud_locate: ' + ('map %s, %d points, %.1f s' % (data[1:].decode(), len(loc.map.pts), time.time() - t)
                                              if loc.map is not None else 'map unloaded'), flush=True)
            except Exception as e:                  # a bad path or file: say so, keep listening
                print('cloud_locate: %r failed: %s' % (data[:60], e), flush=True)
    threading.Thread(target=commands, daemon=True).start()

    rclpy.init()
    node = rclpy.create_node('cloud_locate')
    stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9                                  # noqa: E731

    def odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        with lock:
            loc.on_odom(stamp(m.header), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def cloud(m):
        if loc.track is None or loc.A is None:      # nothing to track: do not even decode it (10 a second)
            return
        pts = cloud_xyz(m)
        with lock:
            loc.on_cloud(pts)

    def gravity(m):
        with lock:
            loc.on_gravity((m.vector.x, m.vector.y, m.vector.z))

    node.create_subscription(Odometry, '/Odometry', odom, 100)
    node.create_subscription(PointCloud2, '/cloud_registered', cloud, qos_profile_sensor_data)
    node.create_subscription(Vector3Stamped, '/lio/gravity', gravity, 20)

    def tick():
        try:
            s = json.loads(urllib.request.urlopen('http://localhost:8000/api/live/state', timeout=0.5).read())
            live = (s.get('tilt') or 0.0, bool(s.get('diverged')) or not s.get('running'), s.get('t'))
        except Exception:
            live = (0.0, False, None)
        with lock:
            loc.live_state(*live)
            t = time.time()
            data = loc.fix(t)
        took = time.time() - t
        if took > 2.0:                                  # a search: worth a line
            print('cloud_locate: a fix took %.1f s (%s)' % (took, STATES[unpack_fix(data)['state']]), flush=True)
        out.sendto(data, ROBOT)
    node.create_timer(FIX_S, tick)
    rclpy.spin(node)


# --- check ---------------------------------------------------------------------------

def check():
    room = cloudreg._room()
    m = cloudreg.Map(room)
    # FAST-LIO2's raw frame: the lidar's start, pitched 20 deg (the mount), at 1.2 m over the floor
    pitch = math.radians(20)
    Rm = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    g_raw = Rm.T @ np.array([0.0, 0.0, -9.81])                        # gravity as FAST-LIO2 sees it
    start = np.array([3.0, 2.0, LIDAR_ABOVE_FLOOR])                    # where the run started, on the map

    def raw_of(pts_map):                                               # a map point in FAST-LIO2's raw frame
        return (np.asarray(pts_map) - start) @ Rm

    loc = Locator(load=lambda path: m)
    assert unpack_fix(loc.fix(1.0))['state'] == NO_MAP
    loc.command(b'M/maps/room.ply')
    assert unpack_fix(loc.fix(1.0))['state'] == NO_LIO
    loc.on_gravity(g_raw)
    assert np.allclose(loc.A @ (Rm.T @ [0, 0, 1.0]), [0, 0, 1], atol=1e-9)     # levelled
    # standing at the run's start, facing +x on the map: base_link is MOUNT_XYZ behind the lidar
    loc.on_odom(10.0, (0, 0, 0), (0, 0, 0, 1))                       # FAST-LIO2's start: its raw pose is identity
    truth_base = (start[0] - MOUNT_XYZ[0], start[1], 0.0)
    loc.command(PLACE.pack(b'I', truth_base[0] + 0.4, truth_base[1] - 0.3, math.radians(12)))   # a tap, off
    seen = room[np.linalg.norm(room - start, axis=1) < 30]
    for k in range(12):
        loc.on_cloud(raw_of(seen))
    r = unpack_fix(loc.fix(11.0))
    assert r['state'] == OK, r
    assert abs(r['pose'][0] - truth_base[0]) < 0.03 and abs(r['pose'][1] - truth_base[1]) < 0.03, r
    assert abs(r['pose'][3]) < math.radians(0.5) and r['walls'] > 0.9, r
    # tracked fixes are smoothed: 30% of the way, the heading the short way round
    sm = smooth((0.0, 0.0, 0.0, math.radians(179)), (0.1, 0.0, 0.0, math.radians(-179)))
    assert abs(sm[0] - 0.03) < 1e-12 and abs(math.degrees(sm[3]) - 179.6) < 1e-9, sm
    # a tilt reading over the limit once placed is his walking (the reading follows the accelerometer:
    # 23 of 208 fixes lost to it on 2026-10-15): fixes go on. Before he is placed it still stops them.
    loc.live_state(2.0, False, 50.0)
    assert unpack_fix(loc.fix(12.0))['state'] == OK
    placed, loc.track.placed = loc.track.placed, False
    assert unpack_fix(loc.fix(12.5))['state'] == TILTED
    loc.track.placed = placed
    loc.live_state(0.3, False, 51.0)
    # a long search held the odometry up (2026-10-15: 10 s, its queue overflowed): a gap is NOT a restart
    loc.on_odom(14.0, (0, 0, 0), (0, 0, 0, 1))
    assert loc.track is not None and loc.A is not None
    # FAST-LIO2 restarts: its run's age (the Orin's live state) goes down. A new frame, from where he was
    loc.live_state(0.3, False, 2.0)
    assert loc.track is None and loc.A is None and loc.pending_place is not None
    p = loc.pending_place
    assert abs(p[0] - truth_base[0]) < 0.03 and abs(p[1] - truth_base[1]) < 0.03, p
    loc.on_gravity(g_raw)
    loc.on_odom(14.1, (0, 0, 0), (0, 0, 0, 1))
    assert loc.track is not None
    for k in range(12):
        loc.on_cloud(raw_of(seen))
    r = unpack_fix(loc.fix(15.0))
    assert r['state'] == OK and abs(r['pose'][0] - truth_base[0]) < 0.03, r
    # an empty 'M' unloads (Nav2 back on AMCL), and a place given before a map does not carry over
    loc.command(b'M')
    assert loc.map is None and loc.track is None and unpack_fix(loc.fix(16.0))['state'] == NO_MAP
    loc.command(PLACE.pack(b'I', 1.0, 1.0, 0.0))
    loc.command(b'M/maps/room.ply')
    assert loc.pending_place is None and loc.track is None
    with __import__('tempfile').TemporaryDirectory() as d:
        open(os.path.join(d, 'gravity_aligned.ply'), 'w').close()
        assert cloud_file(d) == os.path.join(d, 'gravity_aligned.ply')
        open(os.path.join(d, 'gravity_aligned.unflattened.ply'), 'w').close()
        assert cloud_file(d) == os.path.join(d, 'gravity_aligned.unflattened.ply') and cloud_file('/a/b.ply') == '/a/b.ply'
    # the datagram round-trips
    d = pack_fix(1.5, 2.5, (1, 2, 3, 0.5), 0.9, 0.8, 0.05, ['x', 'yaw'], (0.01, 0.02, 0.003), REFUSED)
    u = unpack_fix(d)
    assert len(d) == FIX.size and u['weak'] == ['x', 'yaw'] and u['state'] == REFUSED and u['pose'] == (1, 2, 3, 0.5), u
    # a PointCloud2 laid out like FAST-LIO2's (x y z intensity + more) decodes
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], '<f4')
    rec = np.zeros(2, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4'), ('pad', '<f4', 4)])
    rec['x'], rec['y'], rec['z'] = pts.T
    fields = [type('F', (), {'name': n, 'offset': o}) for n, o in (('x', 0), ('y', 4), ('z', 8), ('intensity', 12))]
    msg = type('M', (), {'fields': fields, 'data': rec.tobytes(), 'point_step': rec.itemsize})
    assert np.allclose(cloud_xyz(msg), pts)
    print('cloud_locate: check passed')


if __name__ == '__main__':
    if sys.argv[1:] == ['check']:
        check()
    else:
        main()
