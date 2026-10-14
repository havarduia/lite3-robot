"""mapmaker - a floor map from one walk, for the panel's Record map and Save.

    start(bot)          Nav2 off, a fresh FAST-LIO2 run from where he stands (checked level)
    recording()         {'dist': m, 't': s} while the run goes, else None
    save(name)          the run saved on the Orin as one floor, its floor map fetched, made
                        walkable (env/floor_free.py, env/walked_free.py) and installed as
                        ~/lite3_maps/<name>/, with the place 'start' where the recording began
    cancel()            the run stopped (the Orin keeps it as 'cancelled-map')

    python3 -m robot.mapmaker check     self-check against a pretend Orin, no robot

All on the perception computer, over the Orin panel's HTTP API (no ssh, no
laptop): it is what env/get_map.sh and LIDAR_ORIN_LOG.md sections 19 and 27
did by hand. README.md section 7.5.
"""
import importlib.util
import json
import os
import re
import shutil
import tempfile
import time
import urllib.error
import urllib.request

from . import places
from .lio_relay import MOUNT_XYZ, ORIN_PANEL, live
from .protocol import Lite3Error

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAME = re.compile(r'^[A-Za-z0-9_-]{1,40}$')     # the Orin keeps only these in a run's name
TILT_MAX_DEG = 1.5      # a run started while he still settled draws the floor tilted (5.7 deg once, section 27)
SETTLE_S = 3.0          # how long a fresh run goes before its tilt is believed
WAIT_S = 30.0           # for a run to stop, or a new one to start
PROCESS_S = 600.0       # for the Orin to clean, flatten and slice a run
POLL_S = 1.0
START = 'start'


def check_name(name):
    if not isinstance(name, str) or not NAME.match(name):
        raise Lite3Error('a map name is 1-40 letters, digits, - or _ (no spaces), not %r' % (name,))
    return name


def _get(path, timeout=10.0):
    with urllib.request.urlopen(ORIN_PANEL + path, timeout=timeout) as r:
        return r.read()


def _wait(test, seconds, what):
    end = time.time() + seconds
    while time.time() < end:
        v = test()
        if v:
            return v
        time.sleep(POLL_S)
    raise Lite3Error('the Orin did not %s within %.0f s' % (what, seconds))


def _stop(name):
    if live('state').get('running'):
        live('stop', name=name)
        _wait(lambda: not live('state').get('running'), WAIT_S, 'stop its run')


def start(bot):
    """A fresh FAST-LIO2 run from where he stands now: the floor's origin. He must be
    standing and still. Nav2 goes off: a new run moves the odometry it relies on."""
    if not bot.standing:
        raise Lite3Error('stand him up where the map should start, then press Record map')
    if bot.nav_running:
        bot.nav_stop()
    if not bot.pose_settled():
        raise Lite3Error('he is still moving: let him stand still where the map should start')
    for attempt in (1, 2):
        _stop('before-map' if attempt == 1 else 'tilted-start')
        r = live('start')
        if not r.get('ok') and 'already running' not in str(r.get('error')):
            raise Lite3Error('the Orin would not start FAST-LIO2: %s' % r)
        # the relay starts one itself when he stands still: that one counts if it is fresh
        s = _wait(lambda: (lambda s: s if s.get('running') and s.get('t', 0) >= SETTLE_S else None)(live('state')),
                  WAIT_S, 'start FAST-LIO2')
        if abs(s.get('tilt') or 0.0) <= TILT_MAX_DEG and s.get('dist', 0.0) < 0.5:
            return 'recording: drive him round with the handheld, then press Save'
    raise Lite3Error('FAST-LIO2 started tilted (%.1f deg) twice: let him stand still and try again' % s.get('tilt'))


def recording():
    try:
        s = live('state')
    except (OSError, ValueError):
        return None
    return {'dist': round(s.get('dist') or 0.0, 1), 't': round(s.get('t') or 0.0)} if s.get('running') else None


def cancel():
    _stop('cancelled-map')
    return 'recording cancelled'


def _module(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(REPO, 'env', name + '.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def save(name, root=None):
    """Stop the run as floor `name`, wait for the Orin's floor map, install it. Returns a
    line for the log."""
    name = check_name(name)
    if not live('state').get('running'):
        raise Lite3Error('nothing is recording: press Record map first')
    asked = time.time()
    r = live('stop', name=name, one_floor=True)
    if not r.get('ok'):
        raise Lite3Error('the Orin would not stop the run: %s' % r)
    base = _wait(lambda: _finished(name, asked), PROCESS_S, 'make the floor map')
    files = {}
    src = '/maps/%s/slam_output/latest/' % base
    for key, path in (('pgm', 'walls/floor0.pgm'), ('yaml', 'walls/floor0.yaml'),
                      ('ply', 'gravity_aligned.ply'), ('path', 'gravity_aligned_path.npy')):
        files[key] = _get(src + path, timeout=60.0)
    work = tempfile.mkdtemp(prefix='mapmaker-')
    try:
        p = {k: os.path.join(work, k) for k in ('pgm', 'yaml', 'ply', 'path')}
        p['path'] += '.npy'
        for k, data in files.items():
            if k == 'yaml':
                data = re.sub(rb'(?m)^image:.*$', b'image: map.pgm', data)
            open(p[k], 'wb').write(data)
        _module('floor_free').main(p['pgm'], p['yaml'], p['ply'], p['path'])
        _module('walked_free').main(p['pgm'], p['yaml'], p['path'])
        install(name, p['pgm'], p['yaml'], root)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    return 'floor %s saved (run %s); load it, then I am here on the start mark' % (name, base)


def _finished(name, asked):
    """The run's folder once the Orin has made its floor map, else None. Raises if it failed."""
    s = json.loads(_get('/api/status'))
    if s.get('state') == 'error':
        raise Lite3Error('the Orin failed: %s' % ((s.get('log') or ['?'])[-1]))
    result = s.get('result') or ''
    base = os.path.basename(result)[:-len('.html')] if result.endswith('.html') else ''
    if s.get('state') in ('idle', 'done') and base.startswith(name + '_'):
        return base
    if s.get('state') == 'idle' and time.time() - asked > WAIT_S and not base.startswith(name + '_'):
        raise Lite3Error('the Orin saved the run but made no floor map (busy with another job?)')
    return None


def install(name, pgm, yaml_path, root=None):
    """The map into ~/lite3_maps/<name>/ (the old one kept as .prev), with START where the
    recording began: base_link was MOUNT_XYZ behind the lidar, which is the map's origin."""
    root = root or places.ROOT
    d = os.path.join(root, name)
    os.makedirs(d, exist_ok=True)
    for f in ('map.pgm', 'map.yaml'):
        if os.path.exists(os.path.join(d, f)):
            shutil.copy2(os.path.join(d, f), os.path.join(d, f + '.prev'))
    shutil.copyfile(pgm, os.path.join(d, 'map.pgm'))
    shutil.copyfile(yaml_path, os.path.join(d, 'map.yaml'))
    if any(places._key(q['name']) == START for q in places.load(name, root)):
        places.delete(name, START, root)
    places.save(name, START, -MOUNT_XYZ[0], 0.0, 0.0, root=root)


def check():
    """Against a pretend Orin panel on localhost: a run is started, recorded, saved and installed."""
    import http.server
    import threading
    import numpy as np
    from . import lio_relay

    work = tempfile.mkdtemp(prefix='mapmaker-check-')
    try:
        run = 'hall_20261014_120000'
        latest = os.path.join(work, 'maps', run, 'slam_output', 'latest')
        os.makedirs(os.path.join(latest, 'walls'))
        g = np.full((30, 30), 205, np.uint8)
        g[:, 0] = g[:, -1] = 0                        # two walls, nothing seen between
        open(os.path.join(latest, 'walls', 'floor0.pgm'), 'wb').write(b'P5\n30 30\n255\n' + g.tobytes())
        open(os.path.join(latest, 'walls', 'floor0.yaml'), 'w').write(
            'image: floor0.pgm\nresolution: 0.1\norigin: [-1.5, -1.5, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.15\n')
        path = np.array([(x, 0.0, 0.46) for x in np.arange(-1.0, 1.0, 0.1)])
        np.save(os.path.join(latest, 'gravity_aligned_path.npy'), path)
        floor = np.array([(x, y, 0.0, 1.0) for x in np.arange(-1.3, 1.3, 0.1) for y in np.arange(-1.3, 1.3, 0.1)], '<f4')
        open(os.path.join(latest, 'gravity_aligned.ply'), 'wb').write(
            b'ply\nformat binary_little_endian 1.0\nelement vertex %d\nproperty float x\nproperty float y\n'
            b'property float z\nproperty float intensity\nend_header\n' % len(floor) + floor.tobytes())
        orin = {'running': True, 't': 1.0, 'tilt': 0.2, 'dist': 0.0, 'state': 'idle', 'result': None, 'stops': []}

        class Orin(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def reply(self, code, obj=None, raw=None):
                body = raw if raw is not None else json.dumps(obj).encode()
                self.send_response(code)
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == '/api/live/state':
                    orin['t'] += 2.0                   # time passes for each look
                    return self.reply(200, {k: orin[k] for k in ('running', 't', 'tilt', 'dist')})
                if self.path == '/api/status':
                    return self.reply(200, {'state': orin['state'], 'result': orin['result'], 'log': []})
                f = os.path.join(work, self.path.lstrip('/'))
                return self.reply(200, raw=open(f, 'rb').read()) if os.path.isfile(f) else self.reply(404, {})

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers.get('Content-Length') or 0)) or b'{}')
                if self.path == '/api/live/stop':
                    orin['stops'].append(body)
                    orin['running'] = False
                    if body.get('name') == 'hall':
                        orin['result'] = '/maps/%s.html' % run
                    return self.reply(200, {'ok': True})
                if self.path == '/api/live/start':
                    orin.update(running=True, t=0.0, dist=0.0)
                    return self.reply(200, {'ok': True})
                return self.reply(404, {})

        server = http.server.ThreadingHTTPServer(('127.0.0.1', 0), Orin)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        global ORIN_PANEL, POLL_S
        real = ORIN_PANEL, lio_relay.ORIN_PANEL, POLL_S
        ORIN_PANEL = lio_relay.ORIN_PANEL = 'http://127.0.0.1:%d' % server.server_address[1]
        POLL_S = 0.01

        class Bot:
            standing, nav_running, stopped = True, True, []

            def nav_stop(self):
                self.stopped.append(True)
                self.nav_running = False

            def pose_settled(self):
                return True

        try:
            bot = Bot()
            assert 'recording' in start(bot)
            assert bot.stopped and orin['stops'][0]['name'] == 'before-map', orin['stops']  # Nav2 off, old run put away
            assert recording()['dist'] == 0.0
            for bad in ('two words', '', 'a/b', '../x'):
                try:
                    save(bad, root=os.path.join(work, 'floors'))
                    raise AssertionError('took the name %r' % bad)
                except Lite3Error:
                    pass
            root = os.path.join(work, 'floors')
            msg = save('hall', root=root)
            assert 'floor hall saved' in msg and orin['stops'][-1] == {'name': 'hall', 'one_floor': True}, (msg, orin['stops'])
            d = os.path.join(root, 'hall')
            m = open(os.path.join(d, 'map.pgm'), 'rb').read()
            assert m.startswith(b'P5') and m.count(bytes([254])) > 500, 'the floor seen was not made free'
            assert 'image: map.pgm' in open(os.path.join(d, 'map.yaml')).read()
            s = places.load('hall', root)
            assert [(q['name'], q['x'], q['y']) for q in s] == [('start', -MOUNT_XYZ[0], 0.0)], s
            # again under the same name: replaced, the old one kept, still one start
            orin.update(running=True, result=None)
            save('hall', root=root)
            assert os.path.exists(os.path.join(d, 'map.pgm.prev')) and len(places.load('hall', root)) == 1
            # a run that starts tilted is started again, then given up on
            orin.update(running=True, tilt=4.0)
            try:
                start(bot)
                raise AssertionError('a tilted start was taken')
            except Lite3Error as e:
                assert 'tilted' in str(e), e
        finally:
            ORIN_PANEL, lio_relay.ORIN_PANEL, POLL_S = real
            server.shutdown()
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print('mapmaker: check passed')


if __name__ == '__main__':
    import sys
    if sys.argv[1:] == ['check']:
        check()
    else:
        sys.exit(__doc__)
