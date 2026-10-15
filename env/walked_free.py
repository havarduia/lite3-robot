#!/usr/bin/env python3
"""Where he walked is floor. Run by get_map.sh on a fetched map:

    walked_free.py <map.pgm> <map.yaml> <gravity_aligned_path.npy>     rewrites the .pgm
    walked_free.py check                                               its own check

The Orin's slicer leaves cells unseen round the run's start and in clutter,
and unseen is a wall to the planner (README 7.5): on lab3 and lab6 he
localized on the start spot and had no route anywhere. Walls stay walls,
except where the path crosses one (a door he walked through).
"""
import re
import sys

import numpy as np

# This far either side of the lidar's path. His body reaches 0.25 m; the rest is taken on trust,
# because the costmap makes everything within about 0.3 m of a wall or an unseen cell impassable
# and a narrower strip leaves no lane (tried at 0.3 on lab6: still no route from the start).
WALKED_M = 0.6
# Where the path crosses a wall cell he walked through it: an open door or a door leaf sliced
# as wall. A door-wide gap round the crossing; walls beside the path stay. Without it
# big_walk_to_arcade had no route past its first door (2026-10-15, 16 crossings).
THROUGH_M = 0.4
WALL_UNDER = 90     # the map.yaml thresholds as grey levels: under this a wall,
FREE = 254          # this is free floor


def walked_free(grid, x0, y0, res, path):
    """grid (rows top first) with the cells within WALKED_M of path made free, and walls
    within THROUGH_M of where the path crosses one."""
    out, h = grid.copy(), grid.shape[0]
    disc = lambda m: [(dr, dc) for n in [int(round(m / res))] for dr in range(-n, n + 1)    # noqa: E731
                      for dc in range(-n, n + 1) if dr * dr + dc * dc <= n * n]
    cells = {(h - 1 - int(round((y - y0) / res)), int(round((x - x0) / res))) for x, y in path}
    inside = lambda r, c: 0 <= r < h and 0 <= c < grid.shape[1]                              # noqa: E731
    for r, c in cells:
        through = inside(r, c) and grid[r, c] < WALL_UNDER
        for dr, dc in disc(THROUGH_M) if through else []:
            if inside(r + dr, c + dc):
                out[r + dr, c + dc] = FREE
        for dr, dc in disc(WALKED_M):
            if inside(r + dr, c + dc) and out[r + dr, c + dc] >= WALL_UNDER:
                out[r + dr, c + dc] = FREE
    return out


def main(pgm, yaml_path, path_npy):
    data = open(pgm, 'rb').read()
    m = re.match(rb'P5\s+(\d+)\s+(\d+)\s+255\s', data)
    w, h = int(m.group(1)), int(m.group(2))
    grid = np.frombuffer(data[m.end():], dtype=np.uint8).reshape(h, w)
    text = open(yaml_path).read()
    x0, y0 = map(float, re.search(r'origin: \[([-0-9.e]+), ([-0-9.e]+)', text).groups())
    res = float(re.search(r'resolution: ([0-9.e-]+)', text).group(1))
    out = walked_free(grid, x0, y0, res, np.load(path_npy)[:, :2])
    with open(pgm, 'wb') as f:
        f.write(data[:m.end()] + out.tobytes())
    print('walked_free: %d cells made floor' % int((out != grid).sum()))


def check():
    grid = np.full((20, 20), 205, dtype=np.uint8)
    grid[10, 12] = 0                                    # a wall beside the path
    out = walked_free(grid, -1.0, -1.0, 0.1, [(0.0, 0.0), (0.5, 0.0)])
    row = 19 - 10                                       # y = 0 is 10 cells up from the bottom row
    assert out[row, 10] == FREE and out[row, 15] == FREE and out[row - 6, 10] == FREE     # on it, its end, 0.6 m aside
    assert out[row - 7, 10] == 205 and out[row, 3] == 205                                 # beyond: untouched
    assert out[10, 12] == 0 and grid[row, 10] == 205                                      # the wall stays; the input too
    # a wall straight across the path (a door he walked through): a door-wide gap, the rest stays
    grid = np.full((40, 40), 205, dtype=np.uint8)
    grid[:, 20] = 0                                     # x = 1.0, all the way down
    out = walked_free(grid, -1.0, -1.0, 0.1, [(x / 10, 1.0) for x in range(0, 21)])
    row = 39 - 20                                       # y = 1.0
    gap = [r for r in range(40) if out[r, 20] != 0]
    assert gap == list(range(row - 4, row + 5)), gap    # THROUGH_M 0.4 m either side of where he crossed
    assert out[row - 5, 20] == 0 and out[row + 5, 20] == 0
    print('walked_free ok')


if __name__ == '__main__':
    check() if sys.argv[1:] == ['check'] else main(*sys.argv[1:4])
