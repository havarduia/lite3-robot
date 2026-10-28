#!/usr/bin/env python3
"""Record a run on all three robot computers. Run on the laptop.

    bin/run.py start <name> [--minutes 40]    load on all three, a bag on the perception computer
    bin/run.py start <name> --frames          also the front camera, 2 frames a second, on the Orin
    bin/run.py start <name> --depth --minutes 2   also the depth camera's cloud (what the costmap's camera
                                              layer reads, ~10 MB/s): short runs only
    bin/run.py start <name> --orin-bag        also FAST-LIO2's cloud, odometry, gravity and the IMU, bagged
                                              on the Orin (~6 MB/s) into ~/bags/<date>_<name>; left there
    bin/run.py note "arrived 0.07 m"          a remark, stamped with the laptop's clock
    bin/run.py stop                           stop, fetch into logs/<date>_<name>/, clean up
    bin/run.py purge                          delete camera frames over an hour old (cron, every 5 min)
    bin/run.py ... --local                    the same with the laptop as all three (a dry run)

Everything on the robot runs detached and under `timeout`, so a dropped
ssh, or a laptop that goes to sleep, ends nothing early and leaves nothing
running past --minutes. Stopped by pid, never by name. The topic list is
env/runlog_topics.txt, sent along: no Nav2 word on any command line.
Reached through the robot's hotspot when the laptop is on it, else over
Tailscale. bin/report.py reads the folder. docs/tools.md, "Measuring a run".

Camera frames show people, so they are personal data (PRIVACY.md): only with
--frames, only where everyone in view knows, and deleted FRAMES_KEEP_S after
the run stops, by `purge` (which every other command also does first). Read
the tags out of them (bin/tags.py) within that hour. Install the purge once:
    (crontab -l; echo '*/5 * * * * cd <repo> && bin/run.py purge') | crontab -
"""
import argparse
import datetime
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOGS = os.path.join(REPO, 'logs')
CURRENT = os.path.join(LOGS, '.current')
SSH = ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10']
HOSTS = ('motion', 'jetson', 'orin')
ROBOT_ENV = 'source ~/robot/env/lite3_env.sh >/dev/null 2>&1'
ORIN_ENV = 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=42'
# For terrain analysis offline (nav_stack_migration.md, step 2). Too big to fetch over the
# hotspot, and the replay runs on the Orin, so the bag stays there.
ORIN_TOPICS = '/cloud_registered /Odometry /lio/gravity /livox/imu'
# The laptop's own ROS, walled off from any robot (memory: laptop testing).
LOCAL_ENV = 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1; unset CYCLONEDDS_URI'
STOP_WAIT_S = 15        # a bag closes its file on SIGINT; give it this long
CLOCK_SAMPLES = 20
FRONT_PER_S = 2         # front camera frames kept a second: enough for tags, ~70 kB each
FRAMES_KEEP_S = 3600    # PRIVACY.md: front camera frames are deleted this long after their run stops


def routes(via):
    """host -> ssh arguments; None = run here (--local)."""
    if via == 'local':
        return dict.fromkeys(HOSTS)
    if via == 'hotspot':        # the motion computer is the hotspot
        hop = ['-J', 'ysc@192.168.2.1']
        return {'motion': ['ysc@192.168.2.1'], 'jetson': hop + ['ysc@192.168.1.103'],
                'orin': hop + ['lite3@192.168.1.5']}
    hop = ['-J', 'ysc@lite3-perception']
    return {'jetson': ['ysc@lite3-perception'], 'motion': hop + ['ysc@192.168.1.120'],
            'orin': hop + ['lite3@192.168.1.5']}


def on_hotspot():
    out = subprocess.run(['ip', '-4', '-o', 'addr'], capture_output=True, text=True).stdout
    return ' 192.168.2.' in out


def sh(route, script, stdin=None, check=True):
    """Run a bash script on one computer; its stdout."""
    cmd = ['bash', '-c', script] if route is None else SSH + route + [script]
    r = subprocess.run(cmd, input=stdin, capture_output=True)
    if check and r.returncode:
        raise SystemExit('failed on %s: %s' % (route or 'laptop', r.stderr.decode(errors='replace')[-500:]))
    return r.stdout


def detached(path, name, command):
    """Shell that starts `command` cut off from this ssh session, its pid in <path>/<name>.pid."""
    return ('setsid bash -c %s </dev/null >%s/%s.out 2>&1 &' %
            (quote('echo $$ > %s/%s.pid; exec %s' % (path, name, command)), path, name))


def quote(s):
    return "'" + s.replace("'", "'\\''") + "'"


def remote_dir(name, host):
    return '/tmp/runlog/%s/%s' % (name, host)


def clock_offset(route, samples=CLOCK_SAMPLES):
    """(seconds that computer's clock is ahead of the laptop's, how sure: half the round trip).
    Over one open connection, the quickest of `samples` asks: a fresh ssh each time spends most
    of its round trip (0.2-1 s through the hotspot hop) setting up, which made the Jetson look
    0.5-1 s ahead when NTP had it within 3 ms (docs/history/LIDAR_ORIN_LOG.md section 29)."""
    loop = 'while read x; do date +%s.%N; done'
    cmd = ['bash', '-c', loop] if route is None else SSH + route + [loop]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                         text=True, bufsize=1)
    best = None
    try:
        for _ in range(samples):
            a = time.time()
            p.stdin.write('x\n')
            p.stdin.flush()
            there = float(p.stdout.readline())
            b = time.time()
            if best is None or b - a < best[0]:
                best = (b - a, there - (a + b) / 2)
    finally:
        p.stdin.close()
        p.terminate()
    return round(best[1], 4), round(best[0] / 2, 4)


def purge_frames(now=None):
    """Delete the front camera frames of every run that stopped more than
    FRAMES_KEEP_S ago (started, if it never stopped); the rest of the run
    (bag, load, report) stays. Times are the laptop's: the Orin's clock can be off."""
    now = now or time.time()
    for front in glob.glob(os.path.join(LOGS, '*', 'orin', 'front')):
        meta = os.path.join(os.path.dirname(os.path.dirname(front)), 'meta.json')
        m = json.load(open(meta)) if os.path.exists(meta) else {}
        ended = m.get('stop') or m.get('start') or os.path.getmtime(front)
        if now - ended > FRAMES_KEEP_S:
            shutil.rmtree(front)
            print('deleted the camera frames of %s (over %d min since the run)'
                  % (os.path.relpath(os.path.dirname(os.path.dirname(front)), REPO), FRAMES_KEEP_S // 60))


def start(name, minutes, via, frames=False, depth=False, orin_bag=False):
    if os.path.exists(CURRENT):
        raise SystemExit('a run is going: %s (bin/run.py stop first)' % open(CURRENT).read().strip())
    if not re.fullmatch(r'[A-Za-z0-9._-]+', name):
        raise SystemExit('name: letters, digits, . _ - only')
    folder = os.path.join(LOGS, '%s_%s' % (datetime.date.today().isoformat(), name))
    os.makedirs(folder)
    cap = int(minutes * 60)
    meta = {'name': name, 'via': via, 'start': time.time(), 'cap_s': cap, 'clock_offset_s': {}, 'clock_error_s': {},
            'laptop_commit': subprocess.run(['git', '-C', REPO, 'log', '-1', '--oneline'],
                                            capture_output=True, text=True).stdout.strip()}
    loadwatch = open(os.path.join(REPO, 'robot', 'loadwatch.py'), 'rb').read()
    frontcam = open(os.path.join(REPO, 'robot', 'frontcam.py'), 'rb').read()
    topics = open(os.path.join(REPO, 'env', 'runlog_topics.txt'), 'rb').read()
    if depth:
        topics += b'\n/camera/depth/color/points\n'
    for host, route in routes(via).items():
        d = remote_dir(name, host)
        sh(route, 'mkdir -p %s && cat > %s/loadwatch.py' % (d, d), stdin=loadwatch)
        meta['clock_offset_s'][host], meta['clock_error_s'][host] = clock_offset(route)
        sh(route, detached(d, 'loadwatch', 'timeout %d python3 %s/loadwatch.py %s/load.jsonl %d 2'
                           % (cap, d, d, cap)))
        if frames and host == 'orin' and route is not None:    # the front camera, decoded where there is room
            sh(route, 'cat > %s/frontcam.py' % d, stdin=frontcam)
            sh(route, detached(d, 'frontcam', 'timeout %d python3 %s/frontcam.py %s/front %d %s'
                               % (cap, d, d, cap, FRONT_PER_S)))
        if orin_bag and host == 'orin':
            env, home = (LOCAL_ENV, '/tmp') if route is None else (ORIN_ENV, '/home/lite3')
            meta['orin_bag'] = '%s/bags/%s' % (home, os.path.basename(folder))
            sh(route, 'mkdir -p %s/bags; %s; %s' % (home, env, detached(
                d, 'bag', 'timeout -s INT %d ros2 bag record -s sqlite3 -o %s %s' % (cap, meta['orin_bag'], ORIN_TOPICS))))
        if host == 'jetson':
            sh(route, 'cat > %s/topics.txt' % d, stdin=topics)
            env = LOCAL_ENV if route is None else ROBOT_ENV
            meta['robot_commit'] = sh(route, 'git -C ~/robot log -1 --oneline 2>/dev/null || true').decode().strip()
            sh(route, '%s; %s' % (env, detached(
                d, 'bag', 'timeout -s INT %d ros2 bag record -s sqlite3 --include-hidden-topics -o %s/bag '
                '$(grep -v "^#" %s/topics.txt)' % (cap, d, d))))
    json.dump(meta, open(os.path.join(folder, 'meta.json'), 'w'), indent=1)
    open(CURRENT, 'w').write(folder + '\n')
    print('recording into %s, at most %d min; bin/run.py stop when done' % (folder, minutes))


def note(text):
    folder = current()
    with open(os.path.join(folder, 'notes.txt'), 'a') as f:
        f.write('%.3f %s\n' % (time.time(), text))
    print('noted in %s' % os.path.basename(folder))


def current():
    if not os.path.exists(CURRENT):
        raise SystemExit('no run going (bin/run.py start <name>)')
    return open(CURRENT).read().strip()


STOP = '''d={d}
for p in bag:INT loadwatch:TERM frontcam:TERM; do f=$d/${{p%:*}}.pid; [ -f $f ] && kill -${{p#*:}} $(cat $f) 2>/dev/null; done
for i in $(seq {wait}); do alive=0
  for f in $d/*.pid; do [ -f $f ] && kill -0 $(cat $f) 2>/dev/null && alive=1; done
  [ $alive = 0 ] && break; sleep 1; done
for f in $d/*.pid; do [ -f $f ] && kill -0 $(cat $f) 2>/dev/null && echo "LEFT RUNNING: $f"; done; true'''

# What each computer adds besides its own folder; t = the run's start, wall clock.
EXTRA = {
    'jetson': 'journalctl -u lio_relay --since @{t} -o short-unix --no-pager > {d}/lio_relay.journal.txt 2>&1; '
              'cp /tmp/nav2.log {d}/nav2.log 2>/dev/null; true',
    'orin': 'journalctl -u lio-send -u livox-driver -u lidar-scan --since @{t} -o short-unix --no-pager '
            '> {d}/orin.journal.txt 2>&1; true',
}


def stop():
    folder = current()
    meta = json.load(open(os.path.join(folder, 'meta.json')))
    for host, route in routes(meta['via']).items():
        d = remote_dir(meta['name'], host)
        print(host + ': ' + (sh(route, STOP.format(d=d, wait=STOP_WAIT_S)).decode().strip() or 'stopped'))
        if route is not None and host in EXTRA:
            sh(route, EXTRA[host].format(d=d, t=int(meta['start'])), check=False)
        os.makedirs(os.path.join(folder, host), exist_ok=True)
        data = sh(route, 'tar -C %s -cf - --exclude=loadwatch.py --exclude=frontcam.py --exclude=topics.txt .' % d)
        subprocess.run(['tar', '-C', os.path.join(folder, host), '-xf', '-'], input=data, check=True)
        sh(route, 'rm -rf %s; rmdir /tmp/runlog/%s /tmp/runlog 2>/dev/null; true' % (d, meta['name']))
    meta['stop'] = time.time()
    json.dump(meta, open(os.path.join(folder, 'meta.json'), 'w'), indent=1)
    os.remove(CURRENT)
    print('fetched into %s; bin/report.py %s' % (folder, os.path.relpath(folder)))
    if meta.get('orin_bag'):
        print('the Orin bag stays on the Orin: %s' % meta['orin_bag'])


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('what', choices=['start', 'note', 'stop', 'purge'])
    p.add_argument('arg', nargs='?')
    p.add_argument('--minutes', type=float, default=40)
    p.add_argument('--local', action='store_true')
    p.add_argument('--frames', action='store_true', help='record front camera frames (personal data: PRIVACY.md)')
    p.add_argument('--depth', action='store_true', help="also record the depth camera's point cloud (heavy)")
    p.add_argument('--orin-bag', action='store_true', help="also bag FAST-LIO2's output on the Orin (kept there)")
    a = p.parse_args()
    purge_frames()
    if a.what == 'start':
        if not a.arg:
            p.error('start needs a name')
        start(a.arg, a.minutes, 'local' if a.local else 'hotspot' if on_hotspot() else 'tailscale', a.frames, a.depth, a.orin_bag)
    elif a.what == 'note':
        if not a.arg:
            p.error('note needs the text')
        note(a.arg)
    elif a.what == 'stop':
        stop()


if __name__ == '__main__':
    sys.exit(main())
