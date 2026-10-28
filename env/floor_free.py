#!/usr/bin/env python3
"""Where the lidar saw floor is floor. Run by get_map.sh on a fetched map, before walked_free.py:

    floor_free.py <map.pgm> <map.yaml> <gravity_aligned.ply> <gravity_aligned_path.npy>   rewrites the .pgm
    floor_free.py check                                                                    its own check

The Orin's slicer (map_walls.py) makes a cell free if its column holds any
point, and leaves it unknown if none. Live FAST-LIO2 clouds are sparse, so
open floor comes out speckled with unknown cells, and unknown is a wall to
the planner: with his half-width round each one, a room or a doorway
closes (docs/history/LIDAR_ORIN_LOG.md section 23). Here an unknown cell becomes free
where the cloud has floor in it, and then where it is a pinhole in free
space. Walls stay walls; low furniture is left to the costmap, as before.
(Thinning walls where floor shows in a wall cell was tried and dropped:
every wall stands on the floor, so nearly every wall cell has floor-height
points, docs/history/LIDAR_ORIN_LOG.md section 25.)
"""
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.cloudreg import read_ply  # noqa: E402

LIDAR_ABOVE_FLOOR = 0.46    # m, the path is the lidar's; map_walls.py uses the same
FLOOR_BAND = 0.12           # m either side of the local floor: still floor (FAST-LIO's cloud is 10 cm)
PINHOLE = 5                 # cells: unknown inside free space that a square this wide closes over
WALL, FREE, UNKNOWN = 0, 254, 205


def floor_seen(shape, x0, y0, res, pts, path):
    """Cells (rows top first) holding a point within FLOOR_BAND of the floor under the
    nearest pose of the path: the floor follows the path, so a slow drift in height
    or a gentle ramp is still floor."""
    from scipy.spatial import cKDTree
    _, near = cKDTree(path[:, :2]).query(pts[:, :2])
    on = np.abs(pts[:, 2] - (path[near, 2] - LIDAR_ABOVE_FLOOR)) < FLOOR_BAND
    h, w = shape
    cx = np.floor((pts[on, 0] - x0) / res + 1e-3).astype(int)     # +1e-3 as map_walls.py
    cy = np.floor((pts[on, 1] - y0) / res + 1e-3).astype(int)
    ok = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    seen = np.zeros(shape, bool)
    seen[h - 1 - cy[ok], cx[ok]] = True
    return seen


def floor_free(grid, seen):
    """grid with unknown cells made free where floor was seen, then where they are
    pinholes in free space (a PINHOLE-wide closing). Walls are never touched."""
    import cv2
    out = grid.copy()
    out[seen & (grid == UNKNOWN)] = FREE
    free = (out == FREE).astype(np.uint8)
    closed = cv2.morphologyEx(free, cv2.MORPH_CLOSE, np.ones((PINHOLE, PINHOLE), np.uint8)).astype(bool)
    out[closed & (out == UNKNOWN)] = FREE
    return out


def main(pgm, yaml_path, ply, path_npy):
    data = open(pgm, 'rb').read()
    m = re.match(rb'P5\s+(\d+)\s+(\d+)\s+255\s', data)
    w, h = int(m.group(1)), int(m.group(2))
    grid = np.frombuffer(data[m.end():], dtype=np.uint8).reshape(h, w)
    text = open(yaml_path).read()
    x0, y0 = map(float, re.search(r'origin: \[([-0-9.e]+), ([-0-9.e]+)', text).groups())
    res = float(re.search(r'resolution: ([0-9.e-]+)', text).group(1))
    out = floor_free(grid, floor_seen(grid.shape, x0, y0, res, read_ply(ply), np.load(path_npy)))
    with open(pgm, 'wb') as f:
        f.write(data[:m.end()] + out.tobytes())
    print('floor_free: %d cells made floor (free %d -> %d)' % (
        int((out != grid).sum()), int((grid == FREE).sum()), int((out == FREE).sum())))


def check():
    # 3 x 3 m at 0.1 m, origin (0, 0): walked along y = 1.5 with the lidar 0.46 m over a floor at z = 0
    path = np.array([(x, 1.5, 0.46) for x in np.arange(0.2, 2.8, 0.1)])
    floor = [(x, y, 0.03) for x in np.arange(0.05, 3.0, 0.1) for y in (0.55, 0.65, 2.35)]
    pts = np.array(floor + [(1.05, 2.05, 0.8),        # a table top: not floor
                            (2.05, 0.25, -0.5)])      # below the floor (a reflection): not floor
    seen = floor_seen((30, 30), 0.0, 0.0, 0.1, pts, path)
    row = lambda y: 29 - int(y / 0.1)                 # noqa: E731  (rows top first)
    assert seen[row(0.55), 3] and seen[row(2.35), 20], 'floor not seen'
    assert not seen[row(2.05), 10] and not seen[row(0.25), 20], 'a table top or a reflection taken for floor'
    grid = np.full((30, 30), UNKNOWN, np.uint8)
    grid[row(1.0):row(0.4) + 1, :] = FREE             # a free band, y 0.4-1.0 ...
    grid[row(0.7), 15] = UNKNOWN                      # ... with a pinhole in it
    grid[row(0.65), 5] = WALL                         # and a wall cell where floor was also seen
    out = floor_free(grid, seen)
    assert out[row(2.35), 20] == FREE                 # floor seen: free
    assert out[row(0.7), 15] == FREE                  # pinhole: closed
    assert out[row(0.65), 5] == WALL                  # walls stay walls
    assert out[row(1.8), 10] == UNKNOWN               # never seen and no pinhole: stays unknown
    assert grid[row(0.7), 15] == UNKNOWN              # the input is untouched

    print('floor_free ok')


if __name__ == '__main__':
    check() if sys.argv[1:] == ['check'] else main(*sys.argv[1:5])
