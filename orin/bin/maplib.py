"""Shared pieces of the mapping tools: map files, 10 cm cells, the scan log.

One place for what several scripts need, so none of them has to import
another script just to borrow a function:

    read_ply / write_ply     map clouds (gravity_aligned.ply and friends)
    vkeys / centres          points <-> 10 cm cell keys (lio_live's map cells)
    read_scans               a run's scans.bin (lio_live writes it)
    align_to                 the tilt-only rotation that levels a frame
    cloud_xyz                x, y, z out of a sensor_msgs/PointCloud2

numpy only (no ROS, no OpenCV), so both pythons can import it: the system
one (ROS nodes, map-walls) and ~/kiss_env (the web panel).
"""
import struct
import zlib

import numpy as np

# ---- map clouds ---------------------------------------------------------------
PLY_TYPES = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
}


def read_ply(path):
    """Return (xyz float32 Nx3, intensity float32 N or None).

    Widths come from the header, never assumed: Open3D writes the SLAM map
    tiles as float32 but the occupancy cloud as float64, so reading everything
    as float32 turns the latter into 1e38-sized garbage instead of failing.
    """
    with open(path, "rb") as f:
        hdr = b""
        while b"end_header" not in hdr:
            line = f.readline()
            if not line:
                return None, None
            hdr += line
        txt = hdr.decode("ascii", "replace")
        n = int([l for l in txt.split("\n") if l.startswith("element vertex")][0].split()[-1])
        fields = []
        for l in txt.split("\n"):
            w = l.split()
            if len(w) >= 3 and w[0] == "property" and w[1] != "list":
                fields.append((w[-1], PLY_TYPES.get(w[1], "f4")))
        names = [nm for nm, _ in fields]
        if not {"x", "y", "z"} <= set(names):
            return None, None
        iname = "intensity" if "intensity" in names else None
        if "binary_little_endian" not in txt:
            arr = np.loadtxt(path, skiprows=len(txt.strip().split("\n")), max_rows=n)
            cols = [names.index(c) for c in ("x", "y", "z")]
            inten = (arr[:, names.index(iname)].astype(np.float32)
                     if iname is not None else None)
            return arr[:, cols].astype(np.float32), inten
        dt = np.dtype([(nm, "<" + t) for nm, t in fields])
        buf = f.read(n * dt.itemsize)
        raw = np.frombuffer(buf[: (len(buf) // dt.itemsize) * dt.itemsize], dtype=dt)
        xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float32)
        inten = raw[iname].astype(np.float32) if iname is not None else None
        return xyz, inten


def write_ply(path, v):
    """Write Nx4 (x, y, z, intensity) as a binary float32 PLY."""
    with open(path, "wb") as f:
        f.write(b"ply\nformat binary_little_endian 1.0\n")
        f.write(f"element vertex {len(v)}\n".encode())
        f.write(b"property float x\nproperty float y\nproperty float z\n")
        f.write(b"property float intensity\nend_header\n")
        f.write(np.ascontiguousarray(v, dtype='<f4').tobytes())


# ---- 10 cm cells: one int64 key each, 21 bits per axis (+-100 km) -------------
VOX = 0.10
_B = 1 << 20
_M21 = (1 << 21) - 1


def vkeys(p):
    """Cell key of each point (Nx3, metres)."""
    k = np.floor(p / VOX).astype(np.int64) + _B
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def cells(k):
    """Integer cell indices (Nx3) of keys."""
    return np.stack([(k >> 42) & _M21, (k >> 21) & _M21, k & _M21], 1) - _B


def centres(k):
    """Centre (metres) of each cell key."""
    return (cells(k) + 0.5) * VOX


# ---- the scan log (scans.bin) ----------------------------------------------------
SCAN_HEAD = struct.Struct("<I3fI")      # scan number, sensor x y z, payload bytes


def pack_scan(n, o, keys):
    """One scans.bin record: header + zlib of the sorted keys' differences
    (small numbers: ~5x smaller than the raw keys)."""
    z = zlib.compress(np.diff(keys, prepend=np.int64(0)).astype("<i8").tobytes(), 1)
    return SCAN_HEAD.pack(n, o[0], o[1], o[2], len(z)) + z


def read_scans(path):
    """[(sensor position, sorted cell keys)] for every scan in a scans.bin."""
    with open(path, "rb") as f:
        data = f.read()
    i, out = 0, []
    while i + SCAN_HEAD.size <= len(data):
        _, x, y, z, nb = SCAN_HEAD.unpack_from(data, i)
        i += SCAN_HEAD.size
        if i + nb > len(data):
            break                                 # truncated last record
        try:
            d = np.frombuffer(zlib.decompress(data[i:i + nb]), "<i8")
        except zlib.error:
            # A run cut off by power loss (battery) leaves a half-written
            # record: everything before it is intact, so stop there.
            print(f"  scan log ends in a damaged record after {len(out)} scans "
                  f"(power cut?) - using those", flush=True)
            break
        out.append((np.array([x, y, z]), np.cumsum(d)))
        i += nb
    return out


# ---- geometry ----------------------------------------------------------------------
UP = np.array([0.0, 0.0, 1.0])


def align_to(v, target=UP):
    """Minimal rotation taking unit v onto unit target.

    Its axis is perpendicular to both, so when target is world up the axis is
    horizontal: pure tilt, zero yaw. That is what keeps the heading SLAM
    estimated while replacing only the part gravity can see.
    """
    v = v / np.linalg.norm(v)
    ax = np.cross(v, target)
    s = np.linalg.norm(ax)
    c = float(v @ target)
    if s < 1e-9:
        return np.eye(3) if c > 0 else -np.eye(3)
    ax = ax / s
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    th = np.arctan2(s, c)
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


_FIELD_TYPES = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}


def cloud_xyz(m):
    """Finite x, y, z (float64 Nx3) of a sensor_msgs/PointCloud2."""
    off = {f.name: (f.offset, _FIELD_TYPES[f.datatype]) for f in m.fields}
    dt = np.dtype({"names": ["x", "y", "z"],
                   "formats": [off[c][1] for c in "xyz"],
                   "offsets": [off[c][0] for c in "xyz"],
                   "itemsize": m.point_step})
    a = np.frombuffer(m.data, dtype=dt, count=m.width * m.height)
    p = np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)
    return p[np.isfinite(p).all(1)]
