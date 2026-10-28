#!/usr/bin/env python3
"""What happened in a recorded run, as numbers and one picture. Laptop only.

    bin/report.py logs/<run>                 writes <run>/report.txt and <run>/report.png
    bin/report.py --compare logs/<a> logs/<b>   the numbers side by side
    ... --tags env/tags/<floor>.json         also AMCL against the surveyed tags (bin/tags.py survey)
    bin/report.py check                      self-check on a made-up run

Reads what bin/run.py fetched: the bag (with Jazzy's sqlite3 plugin), the
load of all three computers, nav2.log, the relay's journal and the notes.
Times are the laptop's clock, in seconds from the run's start. Write
"arrived 0.07 m" as a note after a goal and it becomes that goal's arrival
error. docs/tools.md, "Measuring a run".
"""
import glob
import json
import math
import os
import re
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.lio_relay import wrap  # noqa: E402
from robot.locate import SPREAD_XY_M  # noqa: E402

ODOMS = ('/odom_fused', '/leg_odom2', '/lio_odom')
DONE = {4: 'succeeded', 5: 'canceled', 6: 'aborted'}     # action_msgs/GoalStatus
ARRIVED = re.compile(r'arriv\w*\D*?([\d.]+)\s*m\b')
LOG_TIME = re.compile(r'\[(\d{10}\.\d+)\]')
NUMBER = re.compile(r'-?\d+(\.\d+)?')
TOP_PROCS = 5
STEP_M = 0.05       # path_length: smaller moves are jitter
MOVING_VX = 0.02    # m/s: a forward command at least this is walking, not standing or a turn on the spot
FAST_VX = 0.45      # m/s: near the slow gait's top of 0.5


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(path, offset):
    """{topic: [(t, msg)]}, t = the recorder's receive time moved onto the laptop's clock."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    out, unread = {}, set()
    while reader.has_next():
        topic, data, ns = reader.read_next()
        try:
            out.setdefault(topic, []).append((ns * 1e-9 - offset, deserialize_message(data, types[topic])))
        except Exception:       # a type that changed after Foxy (sensor_msgs/Range gained `variance`)
            unread.add(topic)
    if unread:
        print('report: could not read %s (Foxy message, Jazzy definition)' % ', '.join(sorted(unread)), file=sys.stderr)
    return out


def series(bag):
    """The bag boiled down to numpy arrays and event lists the rest works on."""
    run = {}
    for topic in ODOMS:
        rows = [(t, m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))
                for t, m in bag.get(topic, [])]
        run[topic] = np.array(rows).reshape(-1, 4)
    run['localized'] = np.array([[t] + list(m.data) for t, m in bag.get('/localized', [])
                                 if len(m.data) == 7]).reshape(-1, 8)
    run['cmd_vel'] = np.array([(t, m.linear.x, m.angular.z) for t, m in bag.get('/cmd_vel', [])]).reshape(-1, 3)
    # the 3D localizer (LOCALIZER=cloud): t, state, map->odom x y yaw, walls, rmse, weak bits, spread m rad, fitness
    run['cloud_fix'] = np.array([[t] + list(m.data) for t, m in bag.get('/cloud_fix', []) if len(m.data) == 10]).reshape(-1, 11)
    run['plans'] = [(t, path_length([(p.pose.position.x, p.pose.position.y) for p in m.poses]))
                    for t, m in bag.get('/plan', [])]
    run['battery'] = [(t, m.data[7]) for t, m in bag.get('/robot_state_debug', []) if len(m.data) > 7]
    run['goals'] = goals((t, [(bytes(s.goal_info.goal_id.uuid), s.status) for s in m.status_list])
                         for t, m in bag.get('/navigate_to_pose/_action/status', []))
    return run


def goals(status_msgs):
    """[(t, [(goal id, status)])] -> [{'start', 'end', 'result'}] in order of start."""
    seen, stale = {}, set()
    for t, statuses in status_msgs:
        for gid, status in statuses:
            if gid not in seen and status in DONE:
                stale.add(gid)              # already over when first seen: a Nav2 before this one
            g = seen.setdefault(gid, {'start': t, 'end': None, 'result': None})
            if status in DONE and g['end'] is None:
                g['end'], g['result'] = t, DONE[status]
    return sorted((g for gid, g in seen.items() if gid not in stale), key=lambda g: g['start'])


def path_length(xy, step=STEP_M):
    """Metres along a track, counting a step only once he has moved `step` from the last
    point counted: at 50 Hz, standing still jitters a few mm per pose and that adds up."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if len(xy) < 2:
        return 0.0
    total, last = 0.0, xy[0]
    for p in xy[1:]:
        d = math.hypot(p[0] - last[0], p[1] - last[1])
        if d >= step:
            total, last = total + d, p
    return total


def between(a, t0, t1):
    return a[(a[:, 0] >= t0) & (a[:, 0] <= t1)] if len(a) else a


def relative(a):
    """An odometry track as seen from its own first pose, so tracks with different origins compare."""
    if not len(a):
        return a
    x0, y0, th = a[0, 1:]
    c, s = math.cos(-th), math.sin(-th)
    dx, dy = a[:, 1] - x0, a[:, 2] - y0
    return np.column_stack([a[:, 0], c * dx - s * dy, s * dx + c * dy, a[:, 3] - th])


def apart(a, b):
    """Distance over time between two tracks, both from their first pose, at a's times."""
    a, b = relative(a), relative(b)
    if len(a) < 2 or len(b) < 2:
        return np.empty((0, 2))
    keep = (a[:, 0] >= b[0, 0]) & (a[:, 0] <= b[-1, 0])
    t = a[keep, 0]
    return np.column_stack([t, np.hypot(a[keep, 1] - np.interp(t, b[:, 0], b[:, 1]),
                                        a[keep, 2] - np.interp(t, b[:, 0], b[:, 2]))])


def read_notes(folder):
    path = os.path.join(folder, 'notes.txt')
    if not os.path.exists(path):
        return []
    return [(float(t), text) for t, text in (line.rstrip('\n').split(' ', 1) for line in open(path) if ' ' in line)]


def read_load(folder, offsets):
    """{host: [sample]} with t on the laptop's clock."""
    out = {}
    for path in sorted(glob.glob(os.path.join(folder, '*', 'load.jsonl'))):
        host = os.path.basename(os.path.dirname(path))
        rows = [json.loads(line) for line in open(path) if line.strip()]
        for r in rows:
            r['t'] -= offsets.get(host, 0.0)
        out[host] = rows
    return out


def timed_lines(path):
    """[(t or None, line)]: a ROS log line carries its time; one without takes the last seen."""
    if not os.path.exists(path):
        return []
    out, last = [], None
    for line in open(path, errors='replace'):
        m = LOG_TIME.search(line)
        last = float(m.group(1)) if m else last
        out.append((last, line.rstrip('\n')))
    return out


def journal(path):
    """journalctl -o short-unix lines -> [(t, text)]."""
    out = []
    if os.path.exists(path):
        for line in open(path, errors='replace'):
            head, _, rest = line.partition(' ')
            try:
                out.append((float(head), rest.rstrip('\n')))
            except ValueError:
                pass
    return out


def load_run(folder):
    meta = json.load(open(os.path.join(folder, 'meta.json')))
    offsets = meta.get('clock_offset_s', {})
    bags = glob.glob(os.path.join(folder, 'jetson', 'bag'))
    run = series(read_bag(bags[0], offsets.get('jetson', 0.0)) if bags else {})
    j = offsets.get('jetson', 0.0)
    run.update(meta=meta, notes=read_notes(folder), load=read_load(folder, offsets),
               nav_log=[(None if t is None else t - j, s)
                        for t, s in timed_lines(os.path.join(folder, 'jetson', 'nav2.log'))],
               relay=[(t - j, s) for t, s in journal(os.path.join(folder, 'jetson', 'lio_relay.journal.txt'))])
    return run


def problems(lines, t0, t1):
    """WARN/ERROR lines of nav2.log in [t0, t1], grouped with the numbers taken out: [(count, message)]."""
    counts = {}
    for t, line in lines:
        if t is not None and t0 <= t <= t1 and ('[WARN]' in line or '[ERROR]' in line):
            key = NUMBER.sub('#', line.split(']: ', 1)[-1])[:100]
            counts[key] = counts.get(key, 0) + 1
    return sorted(((n, k) for k, n in counts.items()), reverse=True)


def who(proc):
    """A process's name for the report: python3 alone says nothing, so its module or script."""
    args = proc.get('args', '').split()
    if proc['comm'].startswith('python') and len(args) > 1:
        if '-m' in args[:-1]:
            return args[args.index('-m') + 1]
        script = next((a for a in args[1:] if not a.startswith('-')), args[0])
        return os.path.basename(script)
    return proc['comm']


def summarize(run):
    """(numbers for --compare, the lines of report.txt)."""
    meta, t0 = run['meta'], run['meta']['start']
    t_end = meta.get('stop', t0 + meta.get('cap_s', 0))
    num, out = {}, []
    out.append('Run %s, %s, %.0f s. Laptop %s; robot %s.' % (
        meta['name'], meta['via'], t_end - t0, meta.get('laptop_commit', '?'), meta.get('robot_commit', '?')))
    out.append('')
    out.append('Goals (s from start):')
    fused = run['/odom_fused']
    later = [g['start'] for g in run['goals']][1:] + [t_end]
    for i, (g, nxt) in enumerate(zip(run['goals'], later), 1):
        end = g['end'] if g['end'] is not None else t_end
        walked = path_length(between(fused, g['start'], end)[:, 1:3])
        plan = next((length for t, length in run['plans'] if g['start'] <= t <= end), None)
        arrived = next((float(m.group(1)) for t, text in run['notes']
                        if end <= t <= nxt for m in [ARRIVED.search(text)] if m), None)
        troubles = problems(run['nav_log'], g['start'], end)
        tries = sum(1 for t, line in run['nav_log']
                    if t is not None and g['start'] <= t <= end and 'Attempting' in line)
        out.append('  %d. %6.1f-%6.1f  %-9s %5.1f s  walked %5.2f m  plan %s  arrival error %s  recoveries %d' % (
            i, g['start'] - t0, end - t0, g['result'] or 'unfinished', end - g['start'], walked,
            '-' if plan is None else '%.2f m' % plan, '-' if arrived is None else '%.2f m' % arrived, tries))
        for n, k in troubles[:3]:
            out.append('        %3d x %s' % (n, k))
        num.update({'goal%d_s' % i: end - g['start'], 'goal%d_walked_m' % i: walked})
        if arrived is not None:
            num['goal%d_arrival_m' % i] = arrived
    if not run['goals']:
        out.append('  none')
    done = [g for g in run['goals'] if g['result']]
    num['goals_succeeded'] = sum(g['result'] == 'succeeded' for g in done)
    num['goals_failed'] = sum(g['result'] == 'aborted' for g in done)

    cv = run['cmd_vel']
    fwd = cv[cv[:, 1] >= MOVING_VX, 1] if len(cv) else cv
    if len(fwd):
        num.update(vx_p50=float(np.median(fwd)), vx_p90=float(np.percentile(fwd, 90)), vx_max=float(fwd.max()),
                   vx_fast_share=float(np.mean(fwd >= FAST_VX)), wz_max=float(np.abs(cv[:, 2]).max()))
        out += ['', 'Speed commanded, walking: median %.2f m/s, p90 %.2f, max %.2f; %.2f m/s or more %.0f%% '
                'of those commands; turn rate at most %.2f rad/s' % (
                    num['vx_p50'], num['vx_p90'], num['vx_max'], FAST_VX, 100 * num['vx_fast_share'], num['wz_max'])]

    out += ['', 'Odometry (each track from its own first pose):']
    leg = run['/leg_odom2']
    for topic in ODOMS:
        a = run[topic]
        if len(a):
            gap = float(np.hypot(*(a[-1, 1:3] - a[0, 1:3])))
            num['gap_m ' + topic] = gap
            out.append('  %-12s %5d poses, walked %6.2f m, start-to-end gap %.2f m' % (
                topic, len(a), path_length(a[:, 1:3]), gap))
    for topic in ('/odom_fused', '/lio_odom'):
        d = apart(run[topic], leg)
        if len(d):
            num['apart_max_m ' + topic] = float(d[:, 1].max())
            out.append('  %s vs /leg_odom2: at most %.2f m apart, %.2f m at the end' % (topic, d[:, 1].max(), d[-1, 1]))

    loc = run['localized']
    if len(loc):
        spread = loc[:, 5]
        num['spread_max_m'] = float(np.nanmax(spread))
        out += ['', 'Localization: spread at most %.2f m (limit %.2f), above the limit %.0f%% of the time; '
                'fit %.2f-%.2f; localized %.0f%% of the time' % (
                    np.nanmax(spread), SPREAD_XY_M, 100 * np.mean(spread > SPREAD_XY_M),
                    np.nanmin(loc[:, 7]), np.nanmax(loc[:, 7]), 100 * np.mean(loc[:, 1] == 1))]

    cf = run.get('cloud_fix', np.zeros((0, 11)))
    if len(cf):
        ok = cf[:, 1] == 0                                  # lio_relay.FIX_OK
        walls = cf[ok, 5]
        num.update(cloud_accepted=float(ok.mean()), cloud_walls_median=float(np.median(walls)) if ok.any() else 0.0)
        out += ['', '3D localizer: %d fixes, accepted %.0f%%, walls median %.2f (min %.2f), weak in %.0f%% of the accepted' % (
            len(cf), 100 * ok.mean(), np.median(walls) if ok.any() else 0, walls.min() if ok.any() else 0,
            100 * np.mean(cf[ok, 7] > 0) if ok.any() else 0)]

    relay = run['relay']
    lags = [float(m.group(1)) for _, s in relay for m in [re.search(r'is ([\d.]+) s behind', s)] if m]
    drops = sum('leg odometry alone' in s for _, s in relay)
    num['relay_drops'] = drops
    out += ['', 'Relay: fell back to leg odometry %d times; FAST-LIO2 lag at most %s' % (
        drops, '%.1f s' % max(lags) if lags else 'not reported')]
    camera = sum('dropping message' in s for t, s in run['nav_log'] if t is not None and t0 <= t <= t_end)
    num['camera_dropped'] = camera
    out.append('Camera messages dropped (nav2.log): %d' % camera)
    if run['battery']:
        out.append('Battery: %d%% -> %d%%' % (run['battery'][0][1], run['battery'][-1][1]))

    out += ['', 'Load (busy = mean over cores):']
    for host, rows in run['load'].items():
        if not rows:
            continue
        busy = [np.mean(list(r['cores'].values())) for r in rows]
        num['busy_mean ' + host] = float(np.mean(busy))
        procs = {}
        for r in rows:
            for p in r['top']:
                procs[who(p)] = procs.get(who(p), 0.0) + p['cpu'] / len(rows)
        out.append('  %-7s busy %4.1f%% mean, %4.1f%% max; load avg max %.1f; memory free min %d MB' % (
            host, np.mean(busy), np.max(busy), max(r['load'][0] for r in rows), min(r['avail_mb'] for r in rows)))
        out.append('          ' + ', '.join('%s %.0f%%' % (c, u) for u, c in
                                           sorted(((u, c) for c, u in procs.items()), reverse=True)[:TOP_PROCS]))
    if run['notes']:
        out += ['', 'Notes:'] + ['  %6.1f  %s' % (t - t0, s) for t, s in run['notes']]
    return num, out


def plot(run, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t0 = run['meta']['start']
    t_end = run['meta'].get('stop', t0 + run['meta'].get('cap_s', 0))
    fig = plt.figure(figsize=(11, 15))
    grid = fig.add_gridspec(5, 1, height_ratios=[3, 1, 1, 1, 1.3])
    ax = fig.add_subplot(grid[0])
    for topic in ODOMS:
        a = relative(run[topic])
        if len(a):
            ax.plot(a[:, 1], a[:, 2], label=topic, lw=1)
    ax.set_aspect('equal', 'datalim')
    ax.set_title('Tracks, each from its own first pose (m)')
    ax.legend(loc='best', fontsize=8)
    axes = [fig.add_subplot(grid[i]) for i in range(1, 5)]
    for a in axes[1:]:
        a.sharex(axes[0])
    for topic in ('/odom_fused', '/lio_odom'):
        d = apart(run[topic], run['/leg_odom2'])
        if len(d):
            axes[0].plot(d[:, 0] - t0, d[:, 1], label=topic + ' vs legs')
    axes[0].set_ylabel('apart (m)')
    loc = run['localized']
    if len(loc):
        axes[1].plot(loc[:, 0] - t0, loc[:, 5], label='spread')
        axes[1].plot(loc[:, 0] - t0, loc[:, 7], label='fit')
        axes[1].axhline(SPREAD_XY_M, color='grey', ls=':', lw=1)
    axes[1].set_ylabel('AMCL')
    cv = run['cmd_vel']
    if len(cv):
        axes[2].plot(cv[:, 0] - t0, cv[:, 1], label='vx m/s')
        axes[2].plot(cv[:, 0] - t0, cv[:, 2], label='wz rad/s')
    axes[2].set_ylabel('cmd_vel')
    for host, rows in run['load'].items():
        if rows:
            axes[3].plot([r['t'] - t0 for r in rows], [np.mean(list(r['cores'].values())) for r in rows], label=host)
    axes[3].set_ylabel('busy %')
    axes[3].set_xlabel('s from start')
    for a in axes:
        for g in run['goals']:
            a.axvspan(g['start'] - t0, (g['end'] or t_end) - t0, color='tab:green', alpha=0.08)
        for t, _ in run['notes']:
            a.axvline(t - t0, color='tab:red', lw=0.6, alpha=0.6)
        if a.get_legend_handles_labels()[0]:
            a.legend(loc='upper right', fontsize=8)
    fig.suptitle('%s  (green: goals, red: notes)' % run['meta']['name'])
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    fig.savefig(path, dpi=90)
    plt.close(fig)


def tag_check(folder, run, tags_file):
    """AMCL against the tags: at each good sighting of a surveyed tag, where the tag says he is
    against where AMCL had him. (numbers, lines)."""
    import tags
    surveyed = {int(k): v for k, v in json.load(open(tags_file))['tags'].items()}
    t0, rows = run['meta']['start'], []
    for t, tid, T, pose in tags.good_sightings(folder, run):
        if tid not in surveyed:
            continue
        v = surveyed[tid]
        # On the floor only: where the tag is and which way its face points, seen from him and on
        # the map. Never its turn within its own plane: tags hang any way up (1, 3 and 4 do).
        bx, by, b_yaw = tags.on_floor(T)
        yaw = math.radians(v['yaw_deg']) - b_yaw
        x = v['x'] - (math.cos(yaw) * bx - math.sin(yaw) * by)
        y = v['y'] - (math.sin(yaw) * bx + math.cos(yaw) * by)
        rows.append((t - t0, tid, math.hypot(x - pose[0], y - pose[1]), math.degrees(wrap(yaw - pose[2]))))
    if not rows:
        return {}, ['', 'Tags: no good sighting of a surveyed tag']
    e = np.array([r[2] for r in rows])
    a = np.abs([r[3] for r in rows])
    out = ['', 'AMCL against the tags (%s): %d sightings of %s' % (
        os.path.basename(tags_file), len(rows), sorted({r[1] for r in rows})),
        '  position error: median %.3f m, p90 %.3f m, max %.3f m; heading: median %.1f deg, max %.1f deg' % (
            np.median(e), np.percentile(e, 90), e.max(), np.median(a), a.max())]
    for tid in sorted({r[1] for r in rows}):
        et = np.array([r[2] for r in rows if r[1] == tid])
        out.append('    tag %d: %3d sightings, median %.3f m' % (tid, len(et), np.median(et)))
    return {'tag_err_median_m': float(np.median(e)), 'tag_err_p90_m': float(np.percentile(e, 90))}, out


def compare(a, b, tags_file=None):
    na, _ = summarize(load_run(a))
    nb, _ = summarize(load_run(b))
    if tags_file:
        na.update(tag_check(a, load_run(a), tags_file)[0])
        nb.update(tag_check(b, load_run(b), tags_file)[0])
    name = lambda f: re.sub(r'^\d{4}-\d\d-\d\d_', '', os.path.basename(f.rstrip('/')))[:10]   # noqa: E731
    print('%-28s %10s %10s %10s' % ('', name(a), name(b), 'b - a'))
    for k in sorted(set(na) | set(nb)):
        va, vb = na.get(k), nb.get(k)
        diff = '' if va is None or vb is None else '%10.2f' % (vb - va)
        print('%-28s %10s %10s %s' % (k, '-' if va is None else '%.2f' % va, '-' if vb is None else '%.2f' % vb, diff))


def made_up_run():
    """A run with known answers: a 2 m square walked in 40 s, the lidar drifting 1 cm/s sideways."""
    t0 = 1000.0
    t = np.arange(0, 40.0, 0.1)
    side = np.clip(t / 10, 0, 4)
    x = np.interp(side, [0, 1, 2, 3, 4], [0, 2, 2, 0, 0])
    y = np.interp(side, [0, 1, 2, 3, 4], [0, 0, 2, 2, 0])
    track = lambda dy: np.column_stack([t0 + t, x + 5, y + dy, np.zeros_like(t)])   # noqa: E731
    loc = np.column_stack([t0 + t, np.ones_like(t), x, y, t * 0, np.where(t > 30, 0.6, 0.1), t * 0, t * 0 + 0.9])
    return {
        'meta': {'name': 'made-up', 'via': 'none', 'start': t0, 'stop': t0 + 45},
        '/odom_fused': track(0), '/leg_odom2': track(0), '/lio_odom': track(0.01 * t),
        'localized': loc,
        'cloud_fix': np.array([[t0 + i, 0.0 if i < 8 else 5.0, 0, 0, 0, 0.9 if i < 8 else 0.3, 0.05, 1.0 if i < 2 else 0.0,
                               0.05, 0.01, 0.9] for i in range(10)]),
        'cmd_vel': np.column_stack([t0 + t, np.where(t < 10, 0.5, 0.2), np.where((t >= 20) & (t < 30), -0.3, 0.0)]),
        'plans': [(t0 + 1, 7.5)], 'battery': [(t0, 80), (t0 + 40, 78)],
        'goals': goals([(t0 + 0.5, [(b'g1', 2)]), (t0 + 40.0, [(b'g1', 4)])]),
        'notes': [(t0 + 41, 'arrived 0.07 m')],
        'load': {'jetson': [{'t': t0 + 2, 'cores': {'cpu0': 50.0, 'cpu1': 30.0}, 'load': [3.0, 2, 1],
                             'avail_mb': 4000, 'top': [{'comm': 'realsense', 'cpu': 50.0}]}]},
        'nav_log': [(t0 + 10, '[controller_server-6] [WARN] [1010.0] [controller_server]: Control loop missed '
                     'its desired rate of 5.0000Hz'),
                    (t0 + 20, '[recoveries_server-4] [INFO] [1020.0] [recoveries_server]: Attempting Spin'),
                    (t0 + 21, '[INFO] [x]: dropping message')],
        'relay': [(t0 + 5, 'lio_relay: /odom_fused is leg odometry alone'),
                  (t0 + 6, 'lio_relay: FAST-LIO2 is 1.5 s behind')],
    }


def check():
    run = made_up_run()
    num, lines = summarize(run)
    text = '\n'.join(lines)
    # samples 0.5-39.9 s: sides 0.05-3.99 of 2 m = 7.88; counting in STEP_M steps cuts ~3 cm per corner
    assert abs(num['goal1_walked_m'] - 7.88) < 0.12, num
    assert num['goal1_arrival_m'] == 0.07 and abs(num['goal1_s'] - 39.5) < 1e-9, num
    assert num['goals_succeeded'] == 1 and num['goals_failed'] == 0, num
    assert goals([(5.0, [(b'old', 4), (b'new', 2)]), (9.0, [(b'old', 4), (b'new', 4)])]) == [
        {'start': 5.0, 'end': 9.0, 'result': 'succeeded'}]      # a goal already over when first seen is left out
    assert abs(num['apart_max_m /lio_odom'] - 0.399) < 0.002, num     # 1 cm/s for 39.9 s
    assert num['apart_max_m /odom_fused'] == 0.0 and abs(num['gap_m /leg_odom2'] - 0.02) < 1e-6, num   # last sample 0.1 s short of closing
    assert num['spread_max_m'] == 0.6 and num['relay_drops'] == 1 and num['camera_dropped'] == 1, num
    assert num['busy_mean jetson'] == 40.0, num
    assert 'lag at most 1.5 s' in text and 'plan 7.50 m' in text and 'Battery: 80% -> 78%' in text, text
    assert 'Control loop missed its desired rate of #Hz' in text, text
    # only WARN/ERROR lines are listed; recoveries (INFO) are counted on their own above
    assert problems(run['nav_log'], 1000, 1045) == [(1, 'Control loop missed its desired rate of #Hz')]
    # the track drawn from its own first pose: start at 0,0 whatever the odometry origin
    assert np.allclose(relative(run['/odom_fused'])[0, 1:], 0)
    assert 'recoveries 1' in text, text
    assert 'accepted 80%' in text and 'walls median 0.90' in text and 'weak in 25%' in text, text    # 2 of the 8 accepted
    # 100 commands at 0.5 m/s, 300 at 0.2; a 0.3 rad/s turn the other way
    assert (num['vx_p50'], num['vx_max'], num['vx_fast_share'], num['wz_max']) == (0.2, 0.5, 0.25, 0.3), num
    assert abs(num['vx_p90'] - 0.5) < 1e-9, num
    assert 'median 0.20 m/s, p90 0.50, max 0.50; 0.45 m/s or more 25%' in text, text
    assert who({'comm': 'python3', 'args': 'python3 -m robot.lio_relay'}) == 'robot.lio_relay'
    assert who({'comm': 'python3', 'args': 'python3 /tmp/runlog/x/jetson/loadwatch.py a 9 2'}) == 'loadwatch.py'
    assert who({'comm': 'python3', 'args': 'python3 -u /home/lite3/web/app.py'}) == 'app.py'
    jitter = np.column_stack([np.zeros(500), 0.003 * (np.arange(500) % 2)])
    assert path_length(jitter) == 0.0 and path_length([(0, 0), (1, 0)]) == 1.0
    with tempfile.TemporaryDirectory() as d:
        plot(run, os.path.join(d, 'r.png'))
        assert os.path.getsize(os.path.join(d, 'r.png')) > 10000
    print('report: check passed')


def main(argv):
    if argv == ['check']:
        return check()
    tags_file = None
    if '--tags' in argv:
        i = argv.index('--tags')
        tags_file, argv = argv[i + 1], argv[:i] + argv[i + 2:]
    if len(argv) == 3 and argv[0] == '--compare':
        return compare(argv[1], argv[2], tags_file)
    if len(argv) != 1:
        sys.exit(__doc__)
    folder = argv[0]
    run = load_run(folder)
    _, lines = summarize(run)
    if tags_file:
        lines += tag_check(folder, run, tags_file)[1]
    open(os.path.join(folder, 'report.txt'), 'w').write('\n'.join(lines) + '\n')
    plot(run, os.path.join(folder, 'report.png'))
    print('\n'.join(lines))
    print('\nwrote %s and report.png' % os.path.join(folder, 'report.txt'))


if __name__ == '__main__':
    main(sys.argv[1:])
