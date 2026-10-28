#!/usr/bin/env python3
"""FAST-LIO2 live: map and localise while the robot walks, for the web panel.

Replaced kiss_live.py (KISS-SLAM) on 2026-10-07. KISS-ICP is LiDAR-only: on
this robot its pitch wandered up to 12 deg (fake climbing - 16 m on a flat
floor), and on a fast turn it lost track outright (79 m return error on a
129 m loop). FAST-LIO2 fuses the MID-360's IMU into every registration: tilt
stays pinned to gravity and the gyro carries it through turns.

This process starts fastlio_mapping (config: fastlio_mid360s.yaml), reads its
/Odometry and /cloud_registered, and writes into --out (default ~/maps/live):

    state.json   every 0.2 s: pose, rate, tile versions
    tiles/*.png  every 0.2 s, only the ones that changed: top-down wall map
                 in 12.8 m tiles, brighter = taller structure
    path.json    every 1 s: the path walked
    scans.bin    every scan: sensor position + the cells it hit (lio_clean.py)

On SIGINT/SIGTERM it stops FAST-LIO and saves where map-web and map-walls
look for a result (same layout kiss_live used, so the panel is unchanged):

    ~/maps/live_<stamp>/slam_output/latest/gravity_aligned.ply   10 cm voxels
                                          /gravity_aligned_path.npy
                                          /live_<stamp>_poses.npy
                                          /imu_tilt.npy

    lio-live                    system python, ROS sourced
    lio-live --no-spawn         FAST-LIO already running (e.g. under gdb)

FAST-LIO's world frame is the IMU frame at start-up, which on this mount is
tilted 20 deg; everything here is levelled by the first second of gravity.
There are no loop closures: horizontal drift accumulates (slowly).
"""
import argparse
import collections
import json
import os
import signal
import struct
import shutil
import subprocess
import sys
import threading
import time
import zlib

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from maplib import VOX, align_to, centres, cells, cloud_xyz, pack_scan, vkeys, write_ply  # noqa: E402

HOME = os.path.expanduser("~")
BIN = os.path.dirname(os.path.realpath(__file__))
SLAB = 0.15           # m, vertical bin of the display column masks
ZLO = -2.4            # m: the 64 display slabs span -2.4 .. +7.2 m (2-3 storeys)
TRAJ_STEP = 0.10      # m between path points sent to the browser
PUBLISH_S = 0.2
PATH_EVERY = 5        # ticks: path.json once a second
TILE = 128            # cells per tile side (12.8 m)
RUNAWAY_MS = 4.0      # m moved within 1 s that only divergence produces


# ---- PNG with a palette, no image libraries ------------------------------
def _chunk(tag, data):
    return (struct.pack(">I", len(data)) + tag + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))


def _palette():
    # 0 transparent (never seen); 1..255 seen floor (dim slate) up to a
    # full-height wall (near white), for the panel's dark theme.
    lo, hi = np.array([38, 46, 60]), np.array([235, 240, 250])
    t = np.linspace(0, 1, 255)[:, None] ** 0.8
    pal = np.vstack([[0, 0, 0], lo + (hi - lo) * t]).astype(np.uint8)
    return pal.tobytes(), bytes([0] + [255] * 255)


PLTE, TRNS = _palette()


def png8(img):
    h, w = img.shape
    raw = np.hstack([np.zeros((h, 1), np.uint8), img]).tobytes()
    return (b"\x89PNG\r\n\x1a\n"
            + _chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 3, 0, 0, 0))
            + _chunk(b"PLTE", PLTE) + _chunk(b"tRNS", TRNS)
            + _chunk(b"IDAT", zlib.compress(raw, 6)) + _chunk(b"IEND", b""))


def atomic(path, data):
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def merge_sorted(a, b):
    """Union of two sorted unique int arrays, sorted. A stable sort of two
    sorted runs is a merge (linear), unlike np.unique on the whole lot - which
    re-sorted millions of map cells every 5 s, while holding the lock."""
    m = np.concatenate([a, b])
    m.sort(kind="stable")
    keep = np.ones(len(m), bool)
    keep[1:] = m[1:] != m[:-1]
    return m[keep]


def longest_run(x):
    """Longest run of set bits in each uint64: AND with itself shifted."""
    x = x.copy()
    run = np.zeros(len(x), np.float32)
    one = np.uint64(1)
    while True:
        nz = x != 0
        if not nz.any():
            return run
        run += nz
        x &= x >> one


class Live:
    def __init__(self, a):
        self.a = a
        self.lock = threading.Lock()
        self.A = None                         # levels FAST-LIO's world frame
        self.up = None                        # first-second accel mean
        self.imu_t0 = None
        self.imu_sum = None
        self.imu_n = 0
        self.up_ema = None                    # for the tilt diagnostic
        self.up_t = None
        self.pend_keys = []                   # voxel keys not yet merged
        self.keys = np.empty(0, np.int64)
        self.fresh = []                       # keys since the last tick
        self.poses = []                       # levelled 4x4 per odometry msg
        self.tilt_log = []
        self.tilt = 0.0
        self.frames = 0
        self.odo_t = collections.deque(maxlen=60)
        self.t0 = time.time()
        self.status = "starting FAST-LIO"
        self.masks = {}                       # tile -> uint64 [TILE, TILE]
        self.heights = {}                     # tile -> float32 [TILE, TILE]
        self.shown = {}
        self.tile_v = {}
        self.map_v = 0
        self.bounds = None
        self.path_v = 0
        self.dist = 0.0
        self.path = []                        # decimated [x, y] for the page
        self.path_n = 0                       # poses already folded into it
        self.path_last = None
        self.pub_ms = collections.deque(maxlen=25)
        self.pose_now = None
        self.scan_log = open(os.path.join(a.out, "scans.bin"), "wb")
        self.R_fl = None                      # FAST-LIO's raw attitude (unlevelled)
        self.f_sum = np.zeros(3)              # sum of accel, in FAST-LIO's frame
        self.f_n = 0
        self.g_w = None                       # FAST-LIO's gravity estimate, its frame
        self.jumps = 0                        # odometry steps > 1 m: corrections
        self.recent = collections.deque(maxlen=11)   # last ~1 s of positions
        self.diverged = False
        self.log_sync = time.monotonic()
        os.makedirs(os.path.join(a.out, "tiles"), exist_ok=True)

    # ---- ROS callbacks ------------------------------------------------------
    def on_imu(self, m):
        a = np.array([m.linear_acceleration.x, m.linear_acceleration.y,
                      m.linear_acceleration.z])
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.imu_t0 is None:
            self.imu_t0, self.imu_sum, self.imu_n = t, a.copy(), 1
        elif self.A is None:
            self.imu_sum += a
            self.imu_n += 1
            if t - self.imu_t0 >= 1.0:
                # Level by the first second of gravity: FAST-LIO initialises
                # from the same (standing) samples.
                self.up = self.imu_sum / self.imu_n
                self.A = align_to(self.up, np.array([0.0, 0.0, 1.0]))
        # Gravity in FAST-LIO's world frame, averaged over the whole walk: the
        # robot's own accelerations cancel out, leaving "down" to a fraction
        # of a degree. save() re-levels with it - the first second is wrong
        # whenever the robot was already moving at start-up.
        if self.R_fl is not None:
            self.f_sum += self.R_fl @ a
            self.f_n += 1
        # 5 s average for the tilt diagnostic (the trot bounces ~3 g)
        if self.up_ema is None:
            self.up_ema = a
        else:
            k = min(1.0, max(0.0, t - self.up_t) / 5.0)
            self.up_ema = self.up_ema + k * (a - self.up_ema)
        self.up_t = t

    def on_grav(self, m):
        self.g_w = np.array([m.vector.x, m.vector.y, m.vector.z])

    def on_odom(self, m):
        if self.A is None:
            return
        p = m.pose.pose
        self.R_fl = quat_R(p.orientation)
        T = np.eye(4)
        T[:3, :3] = self.A @ self.R_fl
        T[:3, 3] = self.A @ np.array([p.position.x, p.position.y, p.position.z])
        err = np.nan
        if self.up_ema is not None:
            u = T[:3, :3] @ (self.up_ema / np.linalg.norm(self.up_ema))
            err = float(np.degrees(np.arccos(np.clip(u[2], -1, 1))))
            self.tilt = err
        # Divergence ("shooting away") is a runaway: real ones went 11 m/s and
        # fell 155 m. A single 1-3 m step is FAST-LIO snapping back into line
        # (21 of them on a 13-min, 1.5 m/s walk whose map was fine), so count
        # those as corrections and call it diverged only on > RUNAWAY_MS
        # sustained over a second - faster than this robot can walk.
        if self.pose_now is not None:
            step = float(np.linalg.norm(T[:3, 3] - self.pose_now[:3, 3]))
            if step > 1.0:
                self.jumps += 1
        self.recent.append(T[:3, 3].copy())
        if (len(self.recent) == self.recent.maxlen and not self.diverged and
                np.linalg.norm(self.recent[-1] - self.recent[0]) > RUNAWAY_MS):
            self.diverged = True
            print(f"FAST-LIO ran away (> {RUNAWAY_MS:g} m/s) at {self.frames / 10:.0f} s - "
                  f"diverged; the map from here on is not trustworthy", flush=True)
        with self.lock:
            self.poses.append(T)
            self.tilt_log.append((self.frames, err, 0.0))
            self.pose_now = T
            self.frames += 1
        self.odo_t.append(time.monotonic())
        self.status = ("DIVERGED - stop and restart" if self.diverged else "mapping")

    def on_cloud(self, m):
        if self.A is None:
            return
        p = cloud_xyz(m) @ self.A.T
        k = np.unique(vkeys(p))
        self.log_scan(k)
        with self.lock:
            self.pend_keys.append(k)
            self.fresh.append(k)
            if len(self.fresh) > 100:          # nobody publishing: don't hoard
                del self.fresh[:-100]
            if len(self.pend_keys) >= 50:
                self.keys = merge_sorted(self.keys, np.unique(np.concatenate(self.pend_keys)))
                self.pend_keys = []

    def log_scan(self, k):
        """Append this scan to scans.bin: where the sensor was and which 10 cm
        cells it hit. lio_clean.py replays them to find "ghosts" - people who
        walked past - as cells that later scans saw straight through. Record
        format: maplib.pack_scan / read_scans."""
        T = self.pose_now
        if T is None:
            return
        try:
            self.scan_log.write(pack_scan(self.frames, T[:3, 3], k))
            # To disk every 2 s: a battery swap cuts power with no warning,
            # and lio-recover can only rebuild what reached the disk (one
            # power cut lost the last ~19 s, still in a write buffer).
            if time.monotonic() - self.log_sync > 2.0:
                self.scan_log.flush()
                os.fsync(self.scan_log.fileno())
                self.log_sync = time.monotonic()
        except (OSError, ValueError):
            pass                                  # never let logging stop mapping

    # ---- every tick: changed tiles + state ----------------------------------
    def update_tiles(self, fresh):
        if not fresh:
            return []
        ijk = cells(np.unique(np.concatenate(fresh)))
        z = (ijk[:, 2] + 0.5) * VOX
        s = np.floor((z - ZLO) / SLAB).astype(np.int64)
        ok = (s >= 0) & (s < 64)
        ix, iy, s = ijk[ok, 0], ijk[ok, 1], s[ok]
        if not len(ix):
            return []
        b = [int(ix.min()), int(iy.min()), int(ix.max()), int(iy.max())]
        o = self.bounds
        self.bounds = b if o is None else [min(o[0], b[0]), min(o[1], b[1]),
                                           max(o[2], b[2]), max(o[3], b[3])]
        tx, ty = ix // TILE, iy // TILE
        tk = tx * (1 << 32) + ty
        order = np.argsort(tk, kind="stable")
        tk, ix, iy, s = tk[order], ix[order], iy[order], s[order]
        cuts = np.flatnonzero(np.r_[True, tk[1:] != tk[:-1], True])
        dirty = []
        for a0, a1 in zip(cuts[:-1], cuts[1:]):
            t = (int(ix[a0] // TILE), int(iy[a0] // TILE))
            m = self.masks.get(t)
            if m is None:
                m = self.masks[t] = np.zeros((TILE, TILE), np.uint64)
                self.heights[t] = np.zeros((TILE, TILE), np.float32)
            ly, lx = iy[a0:a1] - t[1] * TILE, ix[a0:a1] - t[0] * TILE
            flat = m.ravel()
            idx = ly * TILE + lx
            u = np.unique(idx)
            before = flat[u].copy()
            np.bitwise_or.at(flat, idx, np.left_shift(np.uint64(1), s[a0:a1].astype(np.uint64)))
            after = flat[u]
            ch = after != before
            if ch.any():
                self.heights[t].ravel()[u[ch]] = longest_run(after[ch]) * SLAB
                dirty.append(t)
        return dirty

    def write_tiles(self, dirty):
        d = os.path.join(self.a.out, "tiles")
        for t in dirty:
            h = self.heights[t]
            # 0 = unseen; 1..255 by column height, saturating at 1.5 m: past
            # where furniture stops and walls carry on
            q = np.where(h > 0, 1 + np.clip(h / 1.5, 0, 1) * 254, 0).astype(np.uint8)
            if t in self.shown and np.array_equal(self.shown[t], q):
                continue
            self.shown[t] = q
            atomic(os.path.join(d, f"t_{t[0]}_{t[1]}.png"), png8(q[::-1]))
            self.map_v += 1
            self.tile_v[f"{t[0]},{t[1]}"] = self.map_v

    def publish(self, full):
        t0 = time.perf_counter()
        with self.lock:
            fresh, self.fresh = self.fresh, []
            T = self.pose_now
            new = self.poses[self.path_n:] if full else []
        self.write_tiles(self.update_tiles(fresh))
        if full and (new or not self.path_v):
            self.extend_path(new)
            self.path_v += 1
            atomic(os.path.join(self.a.out, "path.json"), json.dumps(self.path).encode())
        pose = None
        if T is not None:
            pose = ([round(float(v), 3) for v in T[:3, 3]]
                    + [round(float(np.arctan2(T[1, 0], T[0, 0])), 3)])
        self.pub_ms.append((time.perf_counter() - t0) * 1000)
        self.write_state(pose)

    def extend_path(self, poses):
        """Add new poses to the page's path: a point every TRAJ_STEP metres.
        Only the poses since the last call - with no loop closure, earlier
        poses never move - so the cost stays flat however long the walk."""
        for q in poses:
            p = q[:2, 3]
            if self.path_last is None or np.hypot(*(p - self.path_last)) >= TRAJ_STEP:
                if self.path_last is not None:
                    self.dist += float(np.hypot(*(p - self.path_last)))
                self.path.append([round(float(p[0]), 2), round(float(p[1]), 2)])
                self.path_last = p
        self.path_n += len(poses)

    def write_state(self, pose):
        now = time.monotonic()
        recent = [t for t in self.odo_t if now - t < 3.0]
        st = {
            "engine": "FAST-LIO2",
            "status": self.status, "t": round(time.time() - self.t0, 1),
            "frames": self.frames, "dropped": 0,
            "hz": round(len(recent) / 3.0, 1), "ms": None,
            "pub_ms": round(float(np.median(self.pub_ms)), 1) if self.pub_ms else None,
            "lag": 0, "tilt": round(self.tilt, 1),
            "pose": pose, "path_v": self.path_v, "dist": round(self.dist, 1),
            "closures": None, "nodes": None,
            "map": {"v": self.map_v, "res": VOX, "tile": TILE,
                    "bounds": self.bounds, "tiles": self.tile_v}
                   if self.tile_v else None,
            "imu": self.imu_t0 is not None, "jumps": self.jumps,
            "diverged": self.diverged,
            "updated": time.time(),
        }
        atomic(os.path.join(self.a.out, "state.json"), json.dumps(st).encode())

    # ---- shutdown -----------------------------------------------------------
    def save(self):
        with self.lock:
            poses = list(self.poses)
            keys = np.unique(np.concatenate([self.keys] + self.pend_keys))
            tilt = np.array(self.tilt_log, np.float32).reshape(-1, 3)
        if len(poses) < 20 or not len(keys):
            print("too little data to save", flush=True)
            return None
        self.status = "saving"
        self.write_state(None)
        name = time.strftime("live_%Y%m%d_%H%M%S")
        out = os.path.join(self.a.save_dir, name, "slam_output", "latest")
        if self.a.save_path:                  # lio_run.sh: an exact place
            out = self.a.save_path
            name = os.path.basename(os.path.dirname(os.path.dirname(out))) or name
        os.makedirs(out, exist_ok=True)
        pts = centres(keys)
        P = np.array(poses)
        Z = np.array([0.0, 0.0, 1.0])
        deg = lambda R: np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
        C = None
        if self.f_n > 1000:
            Cf = align_to(self.A @ (self.f_sum / self.f_n), Z)
            print(f"whole-walk accelerometer says level is {deg(Cf):.2f} deg "
                  f"from the first second's", flush=True)
        if self.g_w is not None:
            # Re-level with FAST-LIO's final gravity estimate: the "down" its
            # trajectory was integrated against, with the accelerometer bias
            # it estimated already removed. A raw accelerometer average was
            # 3-5 deg off the plane a single-level walk's path lay in.
            C = align_to(self.A @ (-self.g_w), Z)
            print(f"re-levelled by {deg(C):.2f} deg (FAST-LIO's gravity vs "
                  f"the first second's)", flush=True)
        if C is not None:
            pts = centres(np.unique(vkeys(pts @ C.T)))
            P = np.einsum("ij,njk->nik", np.pad(C, ((0, 1), (0, 1))) + np.diag([0, 0, 0, 1.0]), P)
        write_ply(os.path.join(out, "gravity_aligned.ply"),
                  np.column_stack([pts, np.zeros(len(pts))]))
        np.save(os.path.join(out, f"{name}_poses.npy"), P)
        np.save(os.path.join(out, "gravity_aligned_path.npy"),
                P[:, :3, 3].astype(np.float32))
        np.save(os.path.join(out, "imu_tilt.npy"), tilt)
        # Keep FAST-LIO's log with the run (its [lite3] lines show processing
        # time and IMU gaps): ~/maps/live/ is overwritten by the next start.
        log = os.path.join(self.a.out, "fastlio.log")
        if os.path.exists(log):
            shutil.copy(log, os.path.join(out, "fastlio.log"))
        # Per-scan log + the re-levelling, for lio_clean.py (ghost removal).
        # The log is in the first-second frame; C takes it to the saved one.
        try:
            self.scan_log.close()
            shutil.move(os.path.join(self.a.out, "scans.bin"), os.path.join(out, "scans.bin"))
            np.save(os.path.join(out, "relevel.npy"), C if C is not None else np.eye(3))
        except OSError as e:
            print(f"scan log not saved: {e}", flush=True)
        top = os.path.dirname(os.path.dirname(out))
        print(f"saved {len(pts):,} points, {len(P)} poses -> {top}", flush=True)
        print(f"SAVED {top}", flush=True)
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", default=os.path.join(HOME, "maps", "live"))
    ap.add_argument("--save-dir", default=os.path.join(HOME, "maps"))
    ap.add_argument("--config", default=os.path.join(BIN, "fastlio_mid360s.yaml"))
    ap.add_argument("--no-spawn", action="store_true",
                    help="FAST-LIO is already running; just read its topics")
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--save-path", default=None,
                    help="save into exactly this slam_output/latest directory")
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out, "tiles"), exist_ok=True)
    for f in ["state.json", "path.json", "map.png", "scans.bin"] + [
            os.path.join("tiles", t) for t in os.listdir(os.path.join(a.out, "tiles"))]:
        if os.path.exists(os.path.join(a.out, f)):
            os.remove(os.path.join(a.out, f))

    lio = None
    if not a.no_spawn:
        # Own process group, so stopping it takes its threads down too; its
        # log goes next to the live state for when something looks wrong.
        lio = subprocess.Popen(
            ["ros2", "run", "fast_lio", "fastlio_mapping", "--ros-args",
             "--params-file", a.config],
            stdout=open(os.path.join(a.out, "fastlio.log"), "w"),
            stderr=subprocess.STDOUT, start_new_session=True)

    live = Live(a)
    rclpy.init(args=None)
    node = rclpy.create_node("lio_live")
    deep = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(Imu, "/livox/imu", live.on_imu, deep)
    node.create_subscription(Odometry, "/Odometry", live.on_odom, 100)
    node.create_subscription(PointCloud2, "/cloud_registered", live.on_cloud, 20)
    node.create_subscription(Vector3Stamped, "/lio/gravity", live.on_grav, 20)
    ex = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    print(f"FAST-LIO2 live, writing {a.out}", flush=True)
    tick = 0
    while not stop.wait(PUBLISH_S):
        if lio is not None and lio.poll() is not None and live.status != "saving":
            live.status = f"FAST-LIO exited ({lio.returncode}) - see fastlio.log"
        try:
            live.publish(full=tick % PATH_EVERY == 0)
            tick += 1
        except Exception as e:                    # never let the view kill us
            print(f"publish failed: {e!r}", flush=True)
    if lio is not None and lio.poll() is None:
        os.killpg(lio.pid, signal.SIGINT)
        try:
            lio.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(lio.pid, signal.SIGKILL)
    time.sleep(0.3)                               # last messages in flight
    if not a.no_save:
        live.save()
    live.status = "stopped"
    live.write_state(None)
    ex.shutdown()
    spin.join(timeout=5)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
