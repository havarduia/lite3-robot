#!/usr/bin/env python3
"""Localize one recorded walk on another walk's 3D map, offline. Laptop or Orin.

    bin/locate_replay.py <map.ply> <walk dir> [--every 10] [--start x y z yaw_deg] [--out fixes.jsonl]

<map.ply>: the map walk's gravity_aligned.ply (unflattened). <walk dir>: the
other walk's slam_output/latest, with scans.bin and relevel.npy. Its scans are
already in its own gravity-aligned frame, as FAST-LIO2's /cloud_registered
is live, so what is found is one (x, y, z, yaw) taking that frame onto the
map: searched for at the first scan (--start, round it), then refined at
every --every-th scan (10: once a second) from the one before, as
robot/cloud_locate.py does live. Per stretch of the route: fitness, error,
weak directions, how far the correction jumps between fixes, time per fix.
.planning/plans/cloud-localization.md, phase 0.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot import cloudreg  # noqa: E402

# Stretches of big_walk_to_arcade, on its map (LIDAR_ORIN_LOG section 34): x, y boxes, first match wins
STRETCHES = [('bridge', (-80, -76), (-45, -32)), ('slope corridor', (-80, -76), (-31, -14)),
             ('arcade', (-120, -40), (-75, -52)), ('lab', (-5, 10), (-10, 10)),
             ('long corridor', (-80, -5), (-15, -9))]


def stretch_of(x, y):
    for name, (x0, x1), (y0, y1) in STRETCHES:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return name
    return 'elsewhere'


def main(argv):
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('map_ply')
    p.add_argument('walk')
    p.add_argument('--every', type=int, default=10)
    p.add_argument('--stack', type=int, default=cloudreg.STACK, help='scans registered together (10: 1 s); one scan is ~2000 cells')
    p.add_argument('--start', type=float, nargs=4, default=[0.0, 0.0, 0.0, 0.0], metavar=('X', 'Y', 'Z', 'YAW_DEG'))
    p.add_argument('--out')
    a = p.parse_args(argv)

    t = time.time()
    m = cloudreg.Map.from_ply(a.map_ply)
    print('map: %d points kept for matching, %.1f s to load' % (len(m.pts), time.time() - t), flush=True)
    rl = os.path.join(a.walk, 'relevel.npy')
    scans = cloudreg.read_scans(os.path.join(a.walk, 'scans.bin'), np.load(rl) if os.path.exists(rl) else None)
    print('walk: %d scans' % len(scans), flush=True)

    out = open(a.out, 'w') if a.out else None
    track = cloudreg.Tracker(m, (a.start[0], a.start[1], a.start[2], math.radians(a.start[3])), stack=a.stack)
    rows, prev = [], None
    for k, (sensor, pts) in enumerate(scans):
        track.add(pts)                                  # every scan, as cloud_locate.py gets them
        if k % a.every:
            continue
        before = track.corr
        t = time.time()
        f = track.fix()
        dt = time.time() - t
        corr = track.corr
        jump = math.hypot(corr[0] - before[0], corr[1] - before[1]) if prev is not None else 0.0
        at = cloudreg.transform(corr, sensor[None, :])[0]
        row = {'scan': k, 'x': round(float(at[0]), 2), 'y': round(float(at[1]), 2), 'z': round(float(at[2]), 2),
               'fitness': round(f.fitness, 3), 'walls': round(f.wall_fitness, 3), 'rmse': round(f.rmse, 3),
               'weak': f.weak, 'taken': f.taken, 'why': f.why, 'jump': round(jump, 3),
               'corr': [round(v, 4) for v in corr], 's': round(dt, 3), 'stretch': stretch_of(at[0], at[1])}
        rows.append(row)
        prev = f
        if out:
            out.write(json.dumps(row) + '\n')
        if k % (a.every * 30) == 0:
            print('  scan %5d at (%7.1f, %6.1f) %-14s walls %.2f rmse %.3f weak %-8s %-22s %.2f s' % (
                k, at[0], at[1], row['stretch'], f.wall_fitness, f.rmse, ','.join(f.weak) or '-', f.why, dt), flush=True)

    print('\n%-15s %5s %16s %9s %8s %11s %11s %7s' % ('stretch', 'fixes', 'walls med / min', 'rmse med', 'refused',
                                                       'weak (any)', 'jump max m', 's/fix'))
    for name in [s[0] for s in STRETCHES] + ['elsewhere']:
        r = [q for q in rows if q['stretch'] == name]
        if not r:
            continue
        wf = np.array([q['walls'] for q in r])
        print('%-15s %5d %9.2f / %4.2f %9.3f %7.0f%% %10.0f%% %11.3f %7.2f' % (
            name, len(r), np.median(wf), wf.min(), np.median([q['rmse'] for q in r]),
            100 * np.mean([not q['taken'] for q in r]), 100 * np.mean([bool(q['weak']) for q in r]),
            max(q['jump'] for q in r), np.median([q['s'] for q in r])))
    c = np.array([q['corr'] for q in rows])
    print('\ncorrection over the walk (the truth is near constant): x %.2f..%.2f, y %.2f..%.2f, z %.2f..%.2f m, '
          'yaw %.1f..%.1f deg; searches %d' % (
              c[:, 0].min(), c[:, 0].max(), c[:, 1].min(), c[:, 1].max(), c[:, 2].min(), c[:, 2].max(),
              math.degrees(c[:, 3].min()), math.degrees(c[:, 3].max()), sum('search' in q['why'] for q in rows)))

if __name__ == '__main__':
    main(sys.argv[1:])
