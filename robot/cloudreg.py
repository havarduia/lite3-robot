"""cloudreg - a lidar scan placed on a 3D map: point-to-plane ICP in x, y, z, yaw.

    python3 -m robot.cloudreg check     its own check, no robot, no ROS

numpy and scipy only: it runs on the Orin as it is (no open3d there). The map
is a mapping walk's cloud (gravity_aligned.ply); a scan is FAST-LIO2's
registered points, already in its own gravity-aligned frame, so only x, y,
z and yaw are left to find (.planning/plans/cloud-localization.md).

Directions the scene does not pin down (along a plain corridor) are found
from the problem's Hessian and left at the guess, so the odometry carries
him there instead of the fix sliding: a "weak" direction is reported.
"""
import math
import os
import re
import struct
import zlib

import numpy as np
from scipy.spatial import cKDTree

VOXEL_M = 0.2           # map and scan are thinned to one point per cell this size
NORMAL_K = 10           # neighbours a map normal is fitted to
FLAT = 0.2              # smallest/middle eigenvalue under this: a surface, usable for point-to-plane.
                        # 0.05 kept 16% of a real map and ICP slid on the few walls left (lab replay
                        # 2026-10-15: 0.22 m / 2.8 deg wander, 28% refused); 0.2: 0.04-0.07 m, 0.8 deg, none
MATCH_M = (1.0, 0.5, 0.3)   # a scan point further than this from the map is left out of a step:
                            # wide while the guess may be off, then tight, so clutter stops pulling
MATCH_STEPS = 3             # iterations at each width before the next
INLIER_M = 0.3          # ... and one closer than this counts towards fitness
HUBER_M = 0.1           # residuals past this weigh less: people, moved chairs, a door
ITERATIONS = 30
MAX_POINTS = 6000        # a stack thinned to at most this many (by chance): the bridge's open view took 0.53 s a fix on the Orin
DONE_M = 0.001          # a step smaller than this (and DONE_RAD) ends it
DONE_RAD = math.radians(0.05)
LEVER_M = 5.0           # yaw is scaled by this so all four directions weigh alike in the Hessian
WEAK = 0.02             # an eigenvalue under this share of the largest: a direction not pinned down.
                        # x, y and yaw are judged against each other and z on its own: floors and
                        # ceilings fill a real scan, and against them every sideways direction looks weak
FLAT_AXES = [0, 1, 3]   # x, y, yaw
AXES = ('x', 'y', 'z', 'yaw')
# search(): round a pose given by hand (a tap on the map, a guessed heading)
SEARCH_M, SEARCH_STEP_M = 1.0, 0.25
SEARCH_RAD, SEARCH_STEP_RAD = math.radians(30), math.radians(5)
SEARCH_POINTS = 1500    # a thinned scan scores the candidates
SEARCH_BEST = 3         # this many of the best go on to ICP
# Tracker: what a fix must pass before it moves the correction (lab, two walks a day apart:
# a 3 s stack, this gate and walls-only fitness took the wander from 5.4 m / 100 deg to 0.22 m / 2.8 deg)
STACK = 30              # scans registered together: 3 s (one is ~2000 cells, mostly floor and ceiling)
ACCEPT = 0.5            # wall_fitness under this: refused
GATE_M = 0.3            # FAST-LIO2 drifts far less than this between fixes: a bigger jump is a wrong fit
GATE_RAD = math.radians(3)
REFUSED_MAX = 10        # refused this many in a row: search round the correction afresh
WALL_NZ = 0.5           # a map surface whose normal is this far from vertical is a wall

# scans.bin (the Orin's lio_live.py): 10 cm cells as packed keys
CELL_M = 0.10
_B = 1 << 20
_M21 = (1 << 21) - 1


# --- reading -----------------------------------------------------------------

def read_ply(path):
    """Nx3 points (float64) from a binary little-endian ply, as map_web/map_walls write it."""
    data = open(path, 'rb').read()
    end = data.index(b'end_header\n') + len(b'end_header\n')
    head = data[:end].decode('ascii', 'replace')
    n = int(re.search(r'element vertex (\d+)', head).group(1))
    types = {'float': '<f4', 'float32': '<f4', 'double': '<f8', 'float64': '<f8',
             'uchar': 'u1', 'uint8': 'u1', 'int': '<i4', 'int32': '<i4', 'uint': '<u4'}
    props = re.findall(r'property (\w+) (\w+)', head)
    dt = np.dtype([(name, types[t]) for t, name in props])
    v = np.frombuffer(data, dt, count=n, offset=end)
    return np.column_stack([v['x'], v['y'], v['z']]).astype(np.float64)


def write_ply(path, pts):
    """For checks: x y z as float, the way read_ply expects it."""
    pts = np.asarray(pts, '<f4')
    with open(path, 'wb') as f:
        f.write(b'ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\n'
                b'property float y\nproperty float z\nend_header\n' % len(pts))
        f.write(pts.tobytes())


def cells_to_points(keys):
    ijk = np.stack([(keys >> 42) & _M21, (keys >> 21) & _M21, keys & _M21], 1) - _B
    return (ijk + 0.5) * CELL_M


def read_scans(path, relevel=None):
    """[(sensor xyz, Nx3 points)] from a run's scans.bin, in its saved frame
    (relevel: relevel.npy's 3x3, which takes the first-second frame there).
    A record cut off by a power cut ends the list."""
    data = open(path, 'rb').read()
    C = np.eye(3) if relevel is None else np.asarray(relevel, float)
    out, i = [], 0
    while i + 20 <= len(data):
        _, x, y, z, nb = struct.unpack_from('<I3fI', data, i)
        i += 20
        if i + nb > len(data):
            break
        try:
            d = np.frombuffer(zlib.decompress(data[i:i + nb]), '<i8')
        except zlib.error:
            break
        out.append((C @ np.array([x, y, z]), cells_to_points(np.cumsum(d)) @ C.T))
        i += nb
    return out


def write_scans(path, scans):
    """For checks: the lio_live.py format, from [(sensor xyz, points)]."""
    with open(path, 'wb') as f:
        for n, (o, pts) in enumerate(scans):
            k = np.floor(np.asarray(pts) / CELL_M).astype(np.int64) + _B
            keys = np.unique((k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2])
            z = zlib.compress(np.diff(keys, prepend=np.int64(0)).astype('<i8').tobytes(), 1)
            f.write(struct.pack('<I3fI', n, o[0], o[1], o[2], len(z)) + z)


def voxel(pts, size=VOXEL_M):
    """One point per cell: the mean of what fell in it."""
    pts = np.asarray(pts, float)
    if not len(pts):
        return pts.reshape(0, 3)
    # One int64 key per cell (21 bits an axis, offset to stay >= 0): sorts like
    # the (x, y, z) rows did, and np.unique on it is ~11x faster than on rows
    # (2.7 M points: 4.1 s -> 0.4 s, same result) - a map load waits on this.
    k = np.floor(pts / size).astype(np.int64) + _B
    key = (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]
    _, inv, count = np.unique(key, return_inverse=True, return_counts=True)
    out = np.zeros((len(count), 3))
    for d in range(3):
        out[:, d] = np.bincount(inv, pts[:, d]) / count
    return out


# --- the map -------------------------------------------------------------------

class Map:
    """A cloud thinned to VOXEL_M, with a normal per point, ready to match against.
    Points not on a surface (no clear normal) are left out of the matching, not
    out of fitness: corners, edges and small things still explain a scan."""

    def __init__(self, pts, size=VOXEL_M):
        p = voxel(pts, size)
        tree = cKDTree(p)
        _, nb = tree.query(p, k=min(NORMAL_K, len(p)))
        nbp = p[nb] - p[nb].mean(axis=1, keepdims=True)
        cov = np.einsum('nki,nkj->nij', nbp, nbp)
        w, v = np.linalg.eigh(cov)                       # ascending
        flat = w[:, 0] < FLAT * np.maximum(w[:, 1], 1e-12)
        self.pts, self.normals = p[flat], v[flat, :, 0]
        self.tree = cKDTree(self.pts)
        self.all = tree
        self.wall = np.abs(self.normals[:, 2]) < WALL_NZ

    @classmethod
    def from_ply(cls, path, size=VOXEL_M):
        return cls(read_ply(path), size)


# --- matching ------------------------------------------------------------------

def transform(pose, pts):
    """pose (x, y, z, yaw) applied to Nx3 points: turned about z, then moved."""
    x, y, z, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.asarray(pts, float)
    return np.column_stack([c * p[:, 0] - s * p[:, 1] + x, s * p[:, 0] + c * p[:, 1] + y, p[:, 2] + z])


def compose(a, b):
    """a after b, both (x, y, z, yaw): the pose that maps by b, then by a."""
    c, s = math.cos(a[3]), math.sin(a[3])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2],
            math.atan2(math.sin(a[3] + b[3]), math.cos(a[3] + b[3])))


class Fix:
    """ICP's answer: pose (x, y, z, yaw), fitness (share of the scan within
    INLIER_M of the map), rmse (m, of those), weak (names of the directions
    left at the guess), cov (4x4, yaw in rad; huge along weak directions),
    iterations, n (scan points used)."""

    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __repr__(self):
        return 'Fix(%s)' % ', '.join('%s=%s' % (k, v if not isinstance(v, float) else round(v, 4))
                                     for k, v in self.__dict__.items() if k != 'cov')


def icp(m, scan, guess, iterations=ITERATIONS):
    """Place scan (Nx3, in its own frame) on map m, starting at guess (x, y, z, yaw).
    Returns a Fix. Directions weak in the scene stay at the guess."""
    src = voxel(scan)
    if len(src) > MAX_POINTS:
        src = src[np.random.RandomState(0).choice(len(src), MAX_POINTS, replace=False)]
    pose = np.array(guess, float)
    weak_mask = np.zeros(4, bool)
    H = np.eye(4)
    for it in range(1, iterations + 1):
        p = transform(pose, src)
        d, j = m.tree.query(p, distance_upper_bound=MATCH_M[min((it - 1) // MATCH_STEPS, len(MATCH_M) - 1)])
        ok = np.isfinite(d)
        if ok.sum() < 6:
            return Fix(pose=tuple(pose), fitness=0.0, rmse=float('inf'), weak=list(AXES),
                       cov=np.eye(4) * 1e6, iterations=it, n=len(src))
        pp, q, n = p[ok], m.pts[j[ok]], m.normals[j[ok]]
        r = np.einsum('ij,ij->i', n, pp - q)
        w = np.where(np.abs(r) <= HUBER_M, 1.0, HUBER_M / np.maximum(np.abs(r), 1e-12))
        # d residual / d (x, y, z, yaw * LEVER), the turn about the scan's own centre:
        # about its frame's origin (FAST-LIO2's start, maybe 100 m off) a turn looks like a slide
        centre = pp.mean(axis=0)
        rel = pp - centre
        J = np.column_stack([n[:, 0], n[:, 1], n[:, 2], (n[:, 1] * rel[:, 0] - n[:, 0] * rel[:, 1]) / LEVER_M])
        H = (J * w[:, None]).T @ J
        g = (J * w[:, None]).T @ r
        step, weak_mask = _solve(H, g)
        step[3] /= LEVER_M
        c, s_ = math.cos(step[3]), math.sin(step[3])        # turn about the centre, then move: as a pose
        inc = (step[0] + centre[0] - (c * centre[0] - s_ * centre[1]),
               step[1] + centre[1] - (s_ * centre[0] + c * centre[1]), step[2], step[3])
        pose = np.array(compose(inc, pose))
        if (it > MATCH_STEPS * (len(MATCH_M) - 1) and np.hypot(np.hypot(step[0], step[1]), step[2]) < DONE_M
                and abs(step[3]) < DONE_RAD):
            break
    p = transform(pose, src)
    d, j = m.tree.query(p, distance_upper_bound=INLIER_M)
    on = d < INLIER_M
    rr = np.einsum('ij,ij->i', m.normals[j[on]], p[on] - m.pts[j[on]]) if on.any() else np.zeros(1)
    sigma2 = float(np.mean(rr ** 2)) if on.any() else 1.0
    inl = m.all.query(p, distance_upper_bound=INLIER_M)[0] < INLIER_M
    cov = np.zeros((4, 4))
    hb = H[np.ix_(FLAT_AXES, FLAT_AXES)]
    lam, vec = np.linalg.eigh(hb)
    inv = np.where(lam < WEAK * lam[-1], 1e6, 1.0 / np.maximum(lam, 1e-12))
    cov[np.ix_(FLAT_AXES, FLAT_AXES)] = sigma2 * (vec * inv) @ vec.T
    cov[2, 2] = sigma2 / H[2, 2] if H[2, 2] > 1e-9 else 1e6
    cov[3, :] /= LEVER_M
    cov[:, 3] /= LEVER_M
    return Fix(pose=tuple(float(v) for v in pose), fitness=float(inl.mean()), rmse=math.sqrt(sigma2),
               weak=[a for a, w_ in zip(AXES, weak_mask) if w_], cov=cov, iterations=it, n=len(src))


def wall_fitness(m, pose, scan):
    """Of the scan's points not on a floor or ceiling, the share on a wall: floors and
    ceilings fit whatever the sideways error, walls do not. 0 with nothing to judge."""
    p = transform(pose, voxel(scan))
    miss = m.all.query(p, distance_upper_bound=INLIER_M)[0] >= INLIER_M
    d, j = m.tree.query(p)
    on_wall = m.wall[j] & (d < INLIER_M)
    n = int(on_wall.sum() + miss.sum())
    return float(on_wall.sum()) / n if n else 0.0


class Tracker:
    """The correction from a scan frame (FAST-LIO2's) onto the map, kept up to date.
    add() each scan; fix() now and then (once a second): ICP on the last `stack`
    scans from the current correction, taken only if its walls fit and it moves
    the correction no more than GATE_M / GATE_RAD. The first fix searches round
    the guess; REFUSED_MAX refusals in a row search again round the correction."""

    def __init__(self, m, guess, stack=STACK):
        self.m, self.corr, self.stack = m, tuple(guess), stack
        self.scans, self.refused, self.placed = [], 0, False

    def add(self, pts):
        self.scans.append(np.asarray(pts, float))
        del self.scans[:-self.stack]

    def _judge(self, f, pts, searched):
        f.wall_fitness = wall_fitness(self.m, f.pose, pts)
        jump = math.hypot(f.pose[0] - self.corr[0], f.pose[1] - self.corr[1])
        turn = abs(math.atan2(math.sin(f.pose[3] - self.corr[3]), math.cos(f.pose[3] - self.corr[3])))
        if f.wall_fitness < ACCEPT:
            f.taken, f.why = False, 'walls fit %.2f' % f.wall_fitness
        elif not searched and (jump > GATE_M or turn > GATE_RAD):
            f.taken, f.why = False, 'jump %.2f m %.1f deg' % (jump, math.degrees(turn))
        else:
            f.taken, f.why = True, 'search' if searched else 'tracked'
        return f

    def fix(self):
        """A Fix with .taken, .why and .wall_fitness; self.corr moved if taken."""
        pts = np.vstack(self.scans)
        if not self.placed:
            f = self._judge(search(self.m, pts, self.corr), pts, True)
        else:
            f = self._judge(icp(self.m, pts, self.corr), pts, False)
            if not f.taken:
                self.refused += 1
                if self.refused >= REFUSED_MAX:
                    f = self._judge(search(self.m, pts, self.corr), pts, True)
                    f.why = 'search after %d refused' % self.refused if f.taken else f.why
        if f.taken:
            self.corr, self.placed, self.refused = f.pose, True, 0
        return f


def _solve(H, g):
    """The Gauss-Newton step, and which directions were too weak to move: x, y
    and yaw against each other (their own eigenvalues), z on its own."""
    step, weak = np.zeros(4), np.zeros(4, bool)
    hb, gb = H[np.ix_(FLAT_AXES, FLAT_AXES)], g[FLAT_AXES]
    lam, vec = np.linalg.eigh(hb)
    weak_e = lam < WEAK * lam[-1]
    sb = np.zeros(3)
    for k in np.nonzero(~weak_e)[0]:                     # solve only where the scene says something
        sb -= vec[:, k] * (vec[:, k] @ gb) / lam[k]
    step[FLAT_AXES] = sb
    if weak_e.any():
        weak[FLAT_AXES] = np.abs(vec[:, weak_e]).max(axis=1) > 0.5
    # z: weak if hardly anything faces up or down, compared with the sideways surfaces
    if H[2, 2] < WEAK * lam[-1]:
        weak[2] = True
    else:
        step[2] = -g[2] / H[2, 2]
    return step, weak


def search(m, scan, guess):
    """For a pose given by hand: score a grid round guess (SEARCH_M, SEARCH_RAD) by
    the share of the scan landing on the map, ICP from the best few, keep the
    fittest. A Fix, as icp() gives."""
    src = voxel(scan)
    if len(src) > SEARCH_POINTS:
        src = src[np.random.RandomState(0).choice(len(src), SEARCH_POINTS, replace=False)]
    offsets = np.arange(-SEARCH_M, SEARCH_M + 1e-9, SEARCH_STEP_M)
    turns = np.arange(-SEARCH_RAD, SEARCH_RAD + 1e-9, SEARCH_STEP_RAD)
    scored = []
    for dyaw in turns:
        base = transform((0.0, 0.0, guess[2], guess[3] + dyaw), src)
        for dx in offsets:
            for dy in offsets:
                p = base + (guess[0] + dx, guess[1] + dy, 0.0)
                hit = (m.all.query(p, distance_upper_bound=INLIER_M)[0] < INLIER_M).mean()
                scored.append((hit, (guess[0] + dx, guess[1] + dy, guess[2], guess[3] + dyaw)))
    scored.sort(key=lambda t: -t[0])
    fixes = [icp(m, scan, pose) for _, pose in scored[:SEARCH_BEST]]
    return max(fixes, key=lambda f: f.fitness)


# --- check -------------------------------------------------------------------

def _plane(origin, u, v, nu, nv, step=0.1):
    a, b = np.meshgrid(np.arange(nu) * step, np.arange(nv) * step)
    return np.asarray(origin) + a.reshape(-1, 1) * np.asarray(u) + b.reshape(-1, 1) * np.asarray(v)


def _room():
    """8 x 6 x 3 m: floor, ceiling, four walls, a pillar and a cabinet for x and y."""
    parts = [_plane((0, 0, 0), (1, 0, 0), (0, 1, 0), 80, 60), _plane((0, 0, 3), (1, 0, 0), (0, 1, 0), 80, 60),
             _plane((0, 0, 0), (1, 0, 0), (0, 0, 1), 80, 30), _plane((0, 6, 0), (1, 0, 0), (0, 0, 1), 80, 30),
             _plane((0, 0, 0), (0, 1, 0), (0, 0, 1), 60, 30), _plane((8, 0, 0), (0, 1, 0), (0, 0, 1), 60, 30),
             _plane((5, 2, 0), (1, 0, 0), (0, 0, 1), 6, 30), _plane((5, 2, 0), (0, 1, 0), (0, 0, 1), 6, 30),
             _plane((1.5, 4.5, 0), (1, 0, 0), (0, 0, 1), 10, 20)]
    return np.vstack(parts)


def _corridor():
    """60 m along x, 1.4 m wide, 2.6 m high: nothing along it to hold on to."""
    return np.vstack([_plane((0, 0, 0), (1, 0, 0), (0, 1, 0), 600, 14), _plane((0, 0, 2.6), (1, 0, 0), (0, 1, 0), 600, 14),
                      _plane((0, 0, 0), (1, 0, 0), (0, 0, 1), 600, 26), _plane((0, 1.4, 0), (1, 0, 0), (0, 0, 1), 600, 26)])


def _seen_from(world, at, rng, pose_inv):
    """What a scan at `at` sees of world within rng, in a frame offset by pose_inv."""
    near = world[np.linalg.norm(world - np.asarray(at), axis=1) < rng]
    return transform(pose_inv, near)


def _inverse(pose):
    x, y, z, yaw = pose
    c, s = math.cos(-yaw), math.sin(-yaw)
    return (-(c * x - s * y), -(s * x + c * y), -z, -yaw)


def check():
    import tempfile
    rnd = np.random.RandomState(1)

    # reading and writing what the Orin writes
    with tempfile.TemporaryDirectory() as d:
        room = _room()
        write_ply(os.path.join(d, 'm.ply'), room)
        assert np.allclose(read_ply(os.path.join(d, 'm.ply')), room, atol=1e-5)
        C = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])     # a quarter turn, as relevel.npy may be
        scans = [(np.array([1.0, 2.0, 0.5]), room[:500]), (np.array([3.0, 1.0, 0.5]), room[500:900])]
        write_scans(os.path.join(d, 's.bin'), scans)
        back = read_scans(os.path.join(d, 's.bin'), C)
        assert len(back) == 2 and np.allclose(back[0][0], C @ scans[0][0], atol=1e-5)
        raw = read_scans(os.path.join(d, 's.bin'))
        assert len(back[1][1]) == len(raw[1][1])
        assert np.allclose(back[1][1], raw[1][1] @ C.T)
        # cells come back as their centres: within half a cell of what was written
        orig = room[:500]
        gap, _ = cKDTree(raw[0][1]).query(orig)
        assert gap.max() <= CELL_M * math.sqrt(3) / 2 + 1e-9, gap.max()
        data = open(os.path.join(d, 's.bin'), 'rb').read()
        open(os.path.join(d, 'cut.bin'), 'wb').write(data[:-5])             # a power cut mid-record
        assert len(read_scans(os.path.join(d, 'cut.bin'))) == 1
    v = voxel(np.array([[0.01, 0.01, 0.01], [0.03, 0.05, 0.07], [0.5, 0.5, 0.5]]), 0.2)
    assert len(v) == 2 and np.allclose(sorted(v[:, 0]), [0.02, 0.5])
    assert np.allclose(compose((1, 0, 0, math.pi / 2), (1, 0, 0, 0)), (1, 1, 0, math.pi / 2))
    assert np.allclose(compose((0.5, -0.3, 0.05, 0.2), _inverse((0.5, -0.3, 0.05, 0.2))), 0, atol=1e-12)

    # a room: a known offset found again
    m = Map(_room())
    truth = (0.5, -0.3, 0.05, math.radians(10))
    scan = _seen_from(_room(), (4, 3, 1.2), 30, _inverse(truth)) + rnd.normal(0, 0.01, (1, 3))
    f = icp(m, scan, (0, 0, 0, 0))
    err = np.subtract(f.pose[:3], truth[:3])
    assert np.abs(err).max() < 0.02 and abs(f.pose[3] - truth[3]) < math.radians(0.5), f
    assert not f.weak and f.fitness > 0.95 and f.rmse < 0.03, f

    # clutter not in the map (people, a cart): 30% more points, still found
    junk = rnd.uniform([1, 1, 0], [7, 5, 1.8], (len(scan) * 3 // 10, 3))
    f = icp(m, np.vstack([scan, junk]), (0, 0, 0, 0))
    assert np.abs(np.subtract(f.pose[:3], truth[:3])).max() < 0.03 and abs(f.pose[3] - truth[3]) < math.radians(0.7), f
    assert 0.6 < f.fitness < 0.9, f

    # a plain corridor, the guess as far off as odometry leaves it: along it is weak and stays
    # at the guess; across it and the heading are found
    cor = _corridor()
    m = Map(cor)
    guess = (0.0, 0.0, 0.0, 0.0)
    for truth in ((0.6, 0.2, 0.0, math.radians(1)), (0.6, 0.4, 0.0, math.radians(1)), (-0.4, -0.1, 0.0, math.radians(-0.5))):
        scan = _seen_from(cor, (30, 0.7, 1.2), 12, _inverse(truth))
        f = icp(m, scan, guess)
        assert f.weak == ['x'] and f.fitness > 0.95, (truth, f)
        assert abs(f.pose[0] - guess[0]) < 0.05, (truth, f)                    # held at the guess, not slid
        assert abs(f.pose[1] - truth[1]) < 0.01 and abs(f.pose[3] - truth[3]) < math.radians(0.1), (truth, f)
        assert f.cov[0, 0] > 1e3 * f.cov[1, 1], f.cov
    # a guess 3 deg off in a 1.4 m corridor ends on the wrong walls (as any ICP would): it must say
    # so with a low fitness, not pass as a fix. Big errors are cloud_locate's search to cure.
    truth = (0.6, 0.2, 0.0, math.radians(3))
    f = icp(m, _seen_from(cor, (30, 0.7, 1.2), 12, _inverse(truth)), guess)
    assert abs(f.pose[1] - truth[1]) > 0.5 and f.fitness < 0.5, f

    # "I am here" from a tap: 0.7 m and 25 deg off. The search finds the neighbourhood, ICP the rest
    m = Map(_room())
    truth = (0.9, 0.2, 0.0, math.radians(25))
    scan = _seen_from(_room(), (4, 3, 1.2), 30, _inverse(truth))
    f = search(m, scan, (0.2, -0.1, 0.0, 0.0))
    assert np.abs(np.subtract(f.pose[:3], truth[:3])).max() < 0.03 and abs(f.pose[3] - truth[3]) < math.radians(0.5), f
    assert f.fitness > 0.9, f
    m = Map(_corridor())

    # a stack thinned to MAX_POINTS gives the same answer
    real_max = MAX_POINTS
    globals()['MAX_POINTS'] = 1500
    try:
        f = icp(Map(_room()), _seen_from(_room(), (4, 3, 1.2), 30, _inverse((0.5, -0.3, 0.05, math.radians(10)))), (0, 0, 0, 0))
    finally:
        globals()['MAX_POINTS'] = real_max
    assert f.n == 1500 and abs(f.pose[0] - 0.5) < 0.02 and abs(f.pose[3] - math.radians(10)) < math.radians(0.5), f

    # walls only: shifted 0.5 m sideways, the floor and ceiling still fit, the walls do not
    m = Map(_room())
    scan = _seen_from(_room(), (4, 3, 1.2), 30, (0, 0, 0, 0))
    good, off = wall_fitness(m, (0, 0, 0, 0), scan), wall_fitness(m, (0.5, 0, 0, 0), scan)
    shifted_all = (m.all.query(transform((0.5, 0, 0, 0), voxel(scan)), distance_upper_bound=INLIER_M)[0] < INLIER_M).mean()
    assert good > 0.9 and off < 0.6 and shifted_all > off, (good, off, shifted_all)

    # the tracker: right from the start, a bad jump refused, a real change found again by searching
    room = _room()
    truth = (0.3, -0.2, 0.0, math.radians(4))
    t = Tracker(m, (0.0, 0.0, 0.0, 0.0), stack=3)
    for k in range(3):
        t.add(_seen_from(room, (4, 3, 1.2), 30, _inverse(truth)))
    f = t.fix()
    assert f.taken and np.abs(np.subtract(t.corr[:2], truth[:2])).max() < 0.03, f
    bad = _seen_from(room, (4, 3, 1.2), 30, _inverse((1.3, -0.2, 0.0, math.radians(4))))   # 1 m off at once
    for k in range(3):
        t.add(bad)
    f = t.fix()
    assert not f.taken and 'jump' in f.why and abs(t.corr[0] - truth[0]) < 0.03, f
    for k in range(REFUSED_MAX - 2):                                     # it stays that way: a real change
        t.fix()
    f = t.fix()
    assert f.taken and 'search' in f.why and abs(t.corr[0] - 1.3) < 0.05, (f, t.corr)
    m = Map(_corridor())
    scan = _seen_from(_corridor(), (30, 0.7, 1.2), 12, (0, 0, 0, 0))

    # nothing near: no fix, said plainly
    f = icp(m, scan + (500.0, 0, 0), guess)
    assert f.fitness == 0.0 and f.weak == list(AXES), f
    print('cloudreg: check passed')


if __name__ == '__main__':
    import sys
    if sys.argv[1:] == ['check']:
        check()
    else:
        sys.exit(__doc__)
