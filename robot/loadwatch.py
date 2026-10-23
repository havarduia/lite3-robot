#!/usr/bin/env python3
"""loadwatch - CPU, load and memory of one computer, as JSON lines.

    python3 loadwatch.py <out.jsonl> <seconds> [interval_s]
    python3 loadwatch.py check        self-check, anywhere with a /proc

Stdlib only, Python 3.8: bin/run copies this one file to all three robot
computers (the motion computer and the Orin have no repo). One line per
sample: wall time, host, per-core busy %, load average, memory, and the
TOP processes by CPU (% of one core, like top). Each line is flushed, so a
run cut short still leaves everything up to the cut. docs/tools.md, "Measuring a run".
"""
import json
import os
import signal
import socket
import sys
import time

TOP = 15
TICK = os.sysconf('SC_CLK_TCK')


def cores(stat_text):
    """/proc/stat -> {cpuN: (busy, total)} in ticks; iowait counts as idle."""
    out = {}
    for line in stat_text.splitlines():
        name, *f = line.split()
        if name.startswith('cpu') and name != 'cpu':
            f = [int(x) for x in f[:8]]
            idle = f[3] + f[4]
            out[name] = (sum(f) - idle, sum(f))
    return out


def busy(before, after):
    """Per-core busy % between two cores() readings."""
    return {c: round(100.0 * (after[c][0] - before[c][0]) / max(1, after[c][1] - before[c][1]), 1)
            for c in after if c in before}


def proc_ticks(stat_text):
    """/proc/<pid>/stat -> (comm, utime + stime). comm may hold spaces and brackets."""
    comm = stat_text[stat_text.index('(') + 1:stat_text.rindex(')')]
    f = stat_text[stat_text.rindex(')') + 2:].split()
    return comm, int(f[11]) + int(f[12])


def procs():
    """{pid: (comm, ticks)} for every process still there when read."""
    out = {}
    for pid in os.listdir('/proc'):
        if pid.isdigit():
            try:
                with open('/proc/%s/stat' % pid) as f:
                    out[int(pid)] = proc_ticks(f.read())
            except (OSError, ValueError):
                pass        # gone between listdir and open
    return out


def cmdline(pid):
    try:
        with open('/proc/%d/cmdline' % pid, 'rb') as f:
            return f.read().replace(b'\0', b' ').decode(errors='replace').strip()[:160]
    except OSError:
        return ''


def top(before, after, seconds):
    """The TOP processes by % of one core between two procs() readings."""
    use = [(100.0 * (t - before[p][1]) / TICK / seconds, p, c)
           for p, (c, t) in after.items() if p in before and t > before[p][1]]
    return [{'pid': p, 'comm': c, 'cpu': round(u, 1), 'args': cmdline(p)}
            for u, p, c in sorted(use, reverse=True)[:TOP]]


def memory(meminfo_text):
    """/proc/meminfo -> (total, available) in MB."""
    kb = {}
    for line in meminfo_text.splitlines():
        k, v = line.split(':', 1)
        kb[k] = int(v.split()[0])
    return kb['MemTotal'] // 1024, kb['MemAvailable'] // 1024


def read(path):
    with open(path) as f:
        return f.read()


def sample(prev, now, host):
    """One JSON-ready line from two (time, cores, procs) readings."""
    (t0, c0, p0), (t1, c1, p1) = prev, now
    total, avail = memory(read('/proc/meminfo'))
    return {'t': round(t1, 3), 'host': host, 'cores': busy(c0, c1),
            'load': [float(x) for x in read('/proc/loadavg').split()[:3]],
            'mem_mb': total, 'avail_mb': avail, 'top': top(p0, p1, t1 - t0)}


def snap():
    return time.time(), cores(read('/proc/stat')), procs()


def main(out_path, seconds, interval=2.0):
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    host, end = socket.gethostname(), time.time() + seconds
    with open(out_path, 'a') as out:
        prev = snap()
        while time.time() < end:
            time.sleep(interval)
            now = snap()
            out.write(json.dumps(sample(prev, now, host)) + '\n')
            out.flush()
            prev = now


def check():
    stat = 'cpu  9 9 9 9 9 0 0 0 0 0\ncpu0 100 0 50 800 50 0 0 0 0 0\ncpu1 10 0 0 90 0 0 0 0 0 0\nintr 5\n'
    later = 'cpu  9 9 9 9 9 0 0 0 0 0\ncpu0 150 0 100 850 50 0 0 0 0 0\ncpu1 10 0 0 190 0 0 0 0 0 0\n'
    assert cores(stat) == {'cpu0': (150, 1000), 'cpu1': (10, 100)}, cores(stat)
    assert busy(cores(stat), cores(later)) == {'cpu0': 66.7, 'cpu1': 0.0}, busy(cores(stat), cores(later))
    # a comm with a space and a bracket in it; utime 40 and stime 2 are fields 14 and 15
    line = '123 (a b) c) S 1 123 123 0 -1 4194560 100 0 0 0 40 2 0 0 20 0 1 0 5 1000 50'
    assert proc_ticks(line) == ('a b) c', 42), proc_ticks(line)
    assert memory('MemTotal:  4096 kB\nMemFree: 1 kB\nMemAvailable:  2048 kB\n') == (4, 2)
    before = {1: ('x', 0), 2: ('y', 0), 3: ('z', 5)}
    after = {1: ('x', TICK), 2: ('y', TICK // 2), 3: ('z', 5), 4: ('new', 9)}
    got = [(r['pid'], r['cpu']) for r in top(before, after, 1.0)]
    assert got == [(1, 100.0), (2, 50.0)], got        # idle and newborn processes left out
    # the real /proc: two samples, a line that round-trips
    prev = snap()
    time.sleep(0.3)
    s = sample(prev, snap(), 'here')
    assert s['cores'] and all(0 <= v <= 100 for v in s['cores'].values()), s['cores']
    assert len(s['load']) == 3 and 0 < s['avail_mb'] <= s['mem_mb'], s
    assert json.loads(json.dumps(s)) == s
    print('loadwatch: check passed (%d cores, %d busy processes)' % (len(s['cores']), len(s['top'])))


if __name__ == '__main__':
    if sys.argv[1:] == ['check']:
        check()
    elif len(sys.argv) in (3, 4):
        main(sys.argv[1], float(sys.argv[2]), *[float(x) for x in sys.argv[3:]])
    else:
        sys.exit(__doc__)
