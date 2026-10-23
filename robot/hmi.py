"""hmi - a web page to watch and drive the robot, on the tailnet only.

    python3 -m robot.hmi            # then open http://lite3-perception:8080

It holds one Lite3 with the heartbeat running, so stop it before running any
other script that drives the robot. A browser button is not a hardware stop:
keep the handheld at hand. The safety model (hold-to-move, the depth and
sonar stops, E-STOP, what happens when the page goes away) and the camera
plumbing are in docs/panel.md.
"""
import asyncio
import hmac
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import yaml
from aiohttp import ClientSession, web, WSMsgType

from . import protocol as P
from .depth import sampled
from . import mapmaker
from . import person
from . import places
from .lite3 import BATTERY_REFUSE, BATTERY_WARN, HARD_TIMEOUT, LYING, Lite3
from .nav import CANCELED, REPO, SUCCEEDED, clamp, status_text
from .protocol import Lite3Error
from .rviz import height
from .sonar_range import FOV, SONARS

PORT = 8080
STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'hmi_static')

# What he says on a map goal (nav.go_to_point's say()). Fixed lines: quick and the same every time.
SAY = {'start': 'On my way.', 'arrived': "I'm here.", 'failed': "I couldn't get there."}
LAN_DEV = 'eth0'        # the perception computer's wire to the motion computer
DEADMAN_S = 0.3        # drive halts this long after the last stick message
FORWARD_STOP_M = 0.6   # camera clearance (from body centre), as walk()
REAR_STOP_M = 0.5      # rear sonar reading
DRIVE_MAX_VX = 0.5     # m/s, the page's speed slider tops out here
DRIVE_MAX_WZ = 0.8     # rad/s
RS_LINGER_S = 10.0     # rs_stream.py keeps running this long after the last viewer
GONE_S = 3.0           # no page for this long: a Go is cancelled. The page reconnects in 1.5 s
LET_GO_S = 2.0         # Nav2 gets this long to act on that cancel before it is an E-STOP
FOLLOW_S = HARD_TIMEOUT  # one press of Follow lasts this long: the ceiling on any one move
PING_S = 5.0           # a page that has gone silent is noticed within about 1.5 x this
MODEL_HZ = 10.0        # joint angles to pages showing the 3D model; /joint_states comes at 10
LEGS = ('LF', 'RF', 'LB', 'RB')    # the order the page gets the joints in, three per leg
PLAN_POINTS = 40       # Nav2's path goes to the page thinned to about this many points
COOKIE = 'lite3'       # a logged-in page's cookie: P.panel_key()
LOGIN_DAYS = 30        # a login lasts this long on one browser

# What the page needs to know to draw and warn with the same numbers as here.
LIMITS = {'vx': DRIVE_MAX_VX, 'wz': DRIVE_MAX_WZ, 'stop': FORWARD_STOP_M,
          'rear': REAR_STOP_M, 'refuse': BATTERY_REFUSE, 'warn': BATTERY_WARN,
          'lying': LYING, 'gone': GONE_S, 'follow': FOLLOW_S,
          # for the 3D model: each sonar's x on the body, heading, longest reading; beam width
          'sonars': {name: [x, yaw, top] for name, (_, x, yaw, top) in SONARS.items()},
          'fov': FOV}

# basic_state values seen on this robot (see project notes)
BASIC = {1: 'lying, ready', 6: 'standing', 8: 'not armed', 9: 'arming',
         98: 'fresh boot'}


def ip_of(dev):
    out = subprocess.run(['ip', '-4', '-o', 'addr', 'show', dev], capture_output=True, text=True).stdout
    m = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', out)
    return m.group(1) if m else None


def video_candidate(host_header, local_ip, tailnet_ip):
    """Where a page's browser sends its camera traffic (udp_relay): the address
    it reached the panel at. A phone on his own wifi comes through the motion
    computer (env/robot-nat.sh); HTTPS comes through `tailscale serve` on localhost."""
    if host_header.split(':')[0] == P.AP_IP:
        return P.AP_IP, P.AP_RTC_PORT
    return (tailnet_ip if local_ip.startswith('127.') else local_ip), P.CAMERA_RTC_PORT


class Frames:
    """Latest JPEG from one source, plus how many pages are watching it."""

    def __init__(self):
        self.jpeg, self.seq, self.viewers = None, 0, 0
        self.cond = threading.Condition()

    def put(self, jpeg):
        with self.cond:
            self.jpeg, self.seq = jpeg, self.seq + 1
            self.cond.notify_all()


class FrontCamera(threading.Thread):
    """The wide-angle front camera (RTSP on the motion computer) as JPEGs,
    via ffmpeg. Runs only while someone watches."""

    def __init__(self, frames):
        super().__init__(daemon=True)
        self.frames = frames

    def run(self):
        while True:
            if self.frames.viewers == 0:
                time.sleep(0.5)
                continue
            proc = subprocess.Popen(
                P.camera_mjpeg('-an', '-vf', 'scale=640:-2,fps=8', '-q:v', '7'),
                stdout=subprocess.PIPE, stdin=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL)
            buf = b''
            try:
                while self.frames.viewers > 0:
                    chunk = proc.stdout.read(65536)
                    if not chunk:
                        break
                    buf += chunk
                    while True:
                        a = buf.find(b'\xff\xd8')
                        b = buf.find(b'\xff\xd9', a + 2)
                        if a < 0 or b < 0:
                            break
                        self.frames.put(buf[a:b + 2])
                        buf = buf[b + 2:]
                    if len(buf) > 4_000_000:
                        buf = b''
            finally:
                proc.kill()
                proc.wait()
                # Forget the last frame: the next viewer would get it first, however old.
                self.frames.put(None)
            time.sleep(1.0)


LOGIN_PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Lite3 login</title>
<style>
:root { color-scheme: light dark; --bg: #eef1f5; --card: #fff; --ink: #0f172a; --ink-2: #475569; --line: #94a3b8; --accent: #0369a1; --on: #fff; --bad: #c81e1e; }
@media (prefers-color-scheme: dark) { :root { --bg: #0f172a; --card: #172033; --ink: #f1f5f9; --ink-2: #a3b1c6; --line: #475569; --accent: #38bdf8; --on: #0b1220; --bad: #fca5a5; } }
body { margin: 0; min-height: 100vh; display: grid; place-items: center; background: var(--bg); color: var(--ink); font: 16px/1.5 system-ui, sans-serif; }
form { width: min(340px, calc(100vw - 32px)); padding: 24px; background: var(--card); border: 1px solid var(--line); border-radius: 12px; display: grid; gap: 12px; }
h1 { margin: 0; font-size: 22px; }
p { margin: 0; color: var(--ink-2); font-size: 14px; }
label { font-weight: 700; }
input, button { font: inherit; min-height: 44px; border-radius: 8px; }
input { padding: 0 12px; border: 1px solid var(--line); background: var(--bg); color: var(--ink); }
button { border: 0; background: var(--accent); color: var(--on); font-weight: 700; cursor: pointer; }
.bad { color: var(--bad); font-weight: 700; }
</style></head>
<body><form method="post" action="/login">
<h1>Lite3</h1>
<p>This panel shows live camera video and drives the robot. Authorised operators only.</p>
%s
<label for="pw">Password</label>
<input id="pw" name="password" type="password" autocomplete="current-password" required autofocus>
<button>Log in</button>
</form></body></html>
"""


class Robot:
    """Everything the page can ask of the robot. Blocking; called from threads."""

    def __init__(self, emit):
        self.emit = emit                        # emit(dict) -> all pages
        self.bot = Lite3(estop_on_sigint=False)
        self.bot.heartbeat_start()
        self.work = ThreadPoolExecutor(1)       # motion: one thing at a time
        self.talk_pool = ThreadPoolExecutor(1)  # talking runs beside motion
        self.busy = None                        # name of the running command
        self.estopped = False
        self.drive_cmd = (0.0, 0.0, 0.0)        # vx, wz, time received
        self.driving = False
        self.blocked = None                     # why driving is limited
        self.services = {'camera': None, 'voa': None}
        self.scan = None
        self._warned = {}                       # msg -> time, to stop log spam
        self.talker = None
        self.persona = None
        self.det = None                         # PersonDetector while detection is on
        self.stop_asked = False                 # Cancel, for a walk to or after a person
        self.joints = None                      # name -> angle, for the 3D model
        self.floors = []                        # the floors that have a map
        self.mapping = None                     # while Record map runs: {'dist', 't'} of the Orin's run
        self.floor = None                       # the loaded one, for the page: name, origin, cell size, places
        self.target = None                      # the map point he was last sent to
        from geometry_msgs.msg import PointStamped, PoseStamped
        from nav_msgs.msg import Path
        from sensor_msgs.msg import JointState
        self.bot._node.create_subscription(
            JointState, '/joint_states',
            lambda m: setattr(self, 'joints', dict(zip(m.name, m.position))), 1)
        # What he intends, for the 3D views: Nav2's path in, his goal and where he
        # takes the person to be out (rviz gets those two through robot/rviz.py).
        self.plan = None
        self.bot._node.create_subscription(Path, '/plan', self._plan_cb, 1)
        self._goal_pub = self.bot._node.create_publisher(PoseStamped, '/intent/goal', 1)
        self._them_pub = self.bot._node.create_publisher(PointStamped, '/intent/person', 1)
        self.talk_pool.submit(self._warm_talk)

    # --- commands --------------------------------------------------------
    def log(self, msg, level=None):
        self.emit({'t': 'log', 'level': level, 'msg': msg})
        # also to stdout (/tmp/hmi.log when started with nohup): the page's log is gone with the page,
        # and a goal that failed far from the laptop left no reason behind (2026-10-15)
        print('%s %s%s' % (time.strftime('%H:%M:%S'), '' if level is None else level + ': ', msg), flush=True)

    def warn(self, msg, every=3.0):
        """A warning, but at most once per `every` s: the drive pad sends 10
        messages a second, and each refusal used to log a line."""
        if time.time() - self._warned.get(msg, 0) >= every:
            self._warned[msg] = time.time()
            self.log(msg, 'warn')

    def submit(self, name, fn, *args):
        if self.busy:
            self.warn('busy with %s - %s ignored' % (self.busy, name))
            return
        self.estopped = False
        self.busy = name
        self.work.submit(self._run, name, fn, *args)

    def _run(self, name, fn, *args):
        self.log(name + ' ...')
        try:
            r = fn(*args)
            self.log('%s: %s' % (name, self._fmt(r)))
        except Exception as e:                   # Lite3Error and the rest
            self.log('%s failed: %s' % (name, e), 'error')
        finally:
            self.busy = None

    @staticmethod
    def _fmt(r):
        if isinstance(r, dict) and 'reason' in r:
            return r['reason']
        if r is True or r is None:
            return 'done'
        return str(r)

    def estop(self):
        self.estopped = True
        self.drive_cmd = (0.0, 0.0, 0.0)
        try:
            self.bot.estop()
            self.log('E-STOP: halted, Nav2 killed', 'error')
        except Exception as e:
            self.log('E-STOP error: %s' % e, 'error')

    def cancel(self, why='from the panel'):
        """End a Go, an approach or a follow that is under way. Nav2 stays up,
        unlike E-STOP."""
        going = self.bot.goto_cancel()
        if self.busy in ('approach', 'follow'):
            self.stop_asked = going = True
        if going:
            self.log('cancelled ' + why, 'warn')
        return going

    def abandoned(self):
        """No page is open, so nobody can press E-STOP: a Go, an approach or a
        follow must not carry on alone. (Driving stops by itself, DEADMAN_S.)"""
        if self.cancel('- no panel connected'):
            time.sleep(LET_GO_S)
            if self.bot.goto_cancel():          # still there: Nav2 has not let go
                self.estop()

    def goto(self, metres, degrees):
        if not self.bot.nav_running:
            self.log('starting Nav2 first ...')
            self.bot.nav_start()
            self.bot.cost_ahead(out_to=3.0, step=0.1, settle=10)
        return status_text(self.bot.goto(metres, heading_deg=degrees or None, timeout=90))

    def map_record(self):
        r = mapmaker.start(self.bot)
        self.mapping = mapmaker.recording() or {'dist': 0.0, 't': 0}
        return r

    def map_save(self, name):
        try:
            return mapmaker.save(name)
        finally:
            self.mapping = None
            self.floors = self.bot.floors()

    def map_cancel(self):
        try:
            return mapmaker.cancel()
        finally:
            self.mapping = None

    def go_point(self, x, y, deg=None):
        self.target = [round(x, 2), round(y, 2)]
        try:
            status = self.bot.go_to_point(x, y, yaw_deg=deg, say=self.say)
        except Lite3Error:
            self.say('failed')
            raise
        self.say('arrived' if status == SUCCEEDED else None if status == CANCELED else 'failed')
        return status_text(status)

    def say(self, what):
        """One of SAY aloud, in the persona's voice, beside whatever he is doing."""
        if what in SAY:
            self.talk('say', SAY[what], self.persona)

    def service(self, which, on):
        self.services[which] = getattr(P, which)(on)
        return '%s %s' % (which, 'up' if self.services[which] else 'down')

    def detect(self, on):
        """Person detection on or off. On its own it only looks."""
        if self.det:
            self.det.stop()
            self.det = None
        if on:
            self.det = person.PersonDetector()
            self.det.start()
        self.log('person detection ' + ('on' if on else 'off'))

    def to_person(self, move, **kw):
        """person.approach or person.follow, ended by Cancel and E-STOP as
        well. Switches detection on if it is off, and leaves it on."""
        if not self.det:
            self.detect(True)
        det = self.det
        det.ready.wait(6)
        if det.error:
            raise Lite3Error('detector failed: %s' % det.error)
        self.stop_asked = False
        return move(self.bot, det, stop_distance=FORWARD_STOP_M, abort=lambda: (
            'E-STOP' if self.estopped else 'cancelled' if self.stop_asked else None), **kw)

    # --- hold-to-drive ---------------------------------------------------
    def drive(self, vx, wz):
        vx, wz = clamp(float(vx), DRIVE_MAX_VX), clamp(float(wz), DRIVE_MAX_WZ)
        self.drive_cmd = (vx, wz, time.time())
        if self.driving or (vx == 0 and wz == 0):
            return
        if self.busy:
            self.warn('busy with %s - not driving' % self.busy)
            return
        if self.bot.nav_running:
            self.warn('Nav2 is running - stop it before driving by hand')
            return
        self.driving = True
        self.submit('drive', self._drive_loop)

    def _drive_loop(self):
        clearance = sampled(self.bot.clearance)

        def control():
            if self.estopped:
                return 'E-STOP'
            vx, wz, t = self.drive_cmd
            if time.time() - t > DEADMAN_S:
                return 'released'
            self.blocked = None
            if vx > 0:
                try:
                    clear = clearance()
                except Lite3Error:
                    return 'lost the depth stream'
                if clear <= FORWARD_STOP_M:
                    self.blocked = 'obstacle %.2f m ahead' % clear
                    vx = 0.0
            elif vx < 0:
                rear = self.bot.ultrasound[1]
                if rear is not None and rear < REAR_STOP_M:
                    self.blocked = 'rear sonar %.2f m' % rear
                    vx = 0.0
            return vx, wz

        try:
            return self.bot.steer(control)
        finally:
            self.driving = False
            self.blocked = None

    # --- talking ---------------------------------------------------------
    def _warm_talk(self):
        """Load the voice and open the chat session now. Left to the first
        question they cost it about 6 s (Piper 3.9 s, Gemini Live 1.9 s, 2026-10-07).
        On the talk thread, so a question asked meanwhile waits for it. Also how a
        new Gemini voice takes effect: a session keeps the voice it was opened with."""
        try:
            from .talk import Talker, PERSONA
            if self.talker:
                self.talker.close()
                self.talker = None
            self.persona = self.persona or PERSONA
            self.talker = Talker(persona=self.persona)
            self.talker.live.warm()
        except Exception as e:                  # talking unavailable, driving is not
            self.log('talk is not ready: %s' % e, 'warn')

    def voice(self, name):
        from . import talk
        if name in talk.GEMINI_VOICES and name != talk.GEMINI_VOICE:
            talk.GEMINI_VOICE = name
            self.talk_pool.submit(self._warm_talk)

    def talk(self, kind, text, persona):
        self.talk_pool.submit(self._talk, kind, text, persona)

    def _talk(self, kind, text, persona):
        try:
            from .talk import Talker, PERSONA
            persona = persona or PERSONA
            if self.talker is None or self.persona != persona:
                if self.talker:
                    self.talker.close()
                self.talker = Talker(persona=persona)
                self.persona = persona
            said = lambda s: self.emit({'t': 'said', 'who': 'robot', 'text': s})
            if kind == 'say':               # in the persona's voice, like its replies
                self.talker.say(text, on_sentence=said)
            elif kind == 'ask':
                self.emit({'t': 'said', 'who': 'you', 'text': text})
                self.talker.ask(text, on_sentence=said)
            else:
                self.emit({'t': 'said', 'who': 'you', 'text': '(look) ' + (text or 'what do you see?')})
                self.talker.look(text or None, on_sentence=said)
        except Exception as e:
            self.log('%s failed: %s' % (kind, e), 'error')

    # --- status ----------------------------------------------------------
    def status(self):
        b = self.bot
        s = b.state
        p = b.pose
        f, r = b.ultrasound
        det = self.det
        if det and det.error:
            self.log('person detection failed: %s' % det.error, 'error')
            self.det = det = None
        who = det and det.person()
        goal, plan, them = self.intent(who)
        return {
            't': 'status',
            'basic': s.get('basic'), 'basic_text': BASIC.get(s.get('basic'), '?'),
            'battery': s.get('battery'), 'standing': b.standing,
            'pose': [round(p[0], 2), round(p[1], 2), round(math.degrees(p[2]))] if p else None,
            'sonar': [f, r], 'tilt': b.attitude,
            'nav': b.nav_running, 'heartbeat': b.heartbeat_running,
            'camera': self.services['camera'], 'voa': self.services['voa'],
            'busy': self.busy, 'driving': self.driving, 'blocked': self.blocked,
            'estopped': self.estopped, 'scan': self.scan,
            # the box is left, top, right, bottom as fractions of the front camera's picture
            'detect': det is not None,
            'person': [round(who[k], 3) for k in ('left', 'top', 'right', 'bottom')] if who else None,
            # for the 3D view, in odom: Nav2's goal and path, and where the person is
            'goal': goal, 'plan': plan, 'them': them,
            'floors': self.floors, 'map': self.on_map(), 'mapping': self.mapping,
            'public': P.public_mode(),
            'limits': LIMITS,
        }

    def on_map(self):
        """The floor map view's part of the status, in the map's frame: None with
        no floor loaded. `why` is None while he knows where he is."""
        if self.floor is None:
            return None
        b = self.bot
        p = b.map_pose
        going = b.goal_active()
        return dict(self.floor, why=b.lost_why(),
                    pose=[round(v, 2) for v in p] if p else None,
                    target=self.target if (self.busy or '').startswith('goto map') else None,
                    route=self.plan if going else None)       # Nav2 plans in the map frame on a floor

    def _plan_cb(self, m):
        step = max(1, len(m.poses) // PLAN_POINTS)
        pts = m.poses[::step] + m.poses[-1:]
        self.plan = [[round(q.pose.position.x, 2), round(q.pose.position.y, 2)] for q in pts]

    def intent(self, who):
        """His goal (x, y, yaw), Nav2's path to it and where the person `who` is, each
        None when there is none. Also puts goal and person on /intent/... for rviz."""
        from geometry_msgs.msg import PointStamped, PoseStamped
        b = self.bot
        goal = b.goal if b.goal_active() else None
        if goal is None:
            self.plan = None                    # the path of a goal that is over
        them = person.where(b.pose, who, self.scan or []) if who and b.pose else None
        # Stamp left at zero: 'wherever odom is now' to rviz.
        if goal:
            m = PoseStamped()
            m.header.frame_id = 'odom'
            m.pose.position.x, m.pose.position.y = goal[0], goal[1]
            m.pose.orientation.z, m.pose.orientation.w = math.sin(goal[2] / 2), math.cos(goal[2] / 2)
            self._goal_pub.publish(m)
        if them:
            m = PointStamped()
            m.header.frame_id = 'odom'
            m.point.x, m.point.y = them
            self._them_pub.publish(m)
        return ([round(v, 2) for v in goal] if goal else None, self.plan if goal else None,
                [round(v, 2) for v in them] if them else None)

    def model(self):
        """Where the 3D model's joints and body are, or None before the first joint states."""
        j, p = self.joints, self.bot.pose
        if j is None:
            return None
        return {'t': 'model',
                'j': [round(j[leg + '_Joint' + n], 3) for leg in LEGS for n in ('', '_1', '_2')],
                'p': [round(v, 3) for v in p] if p else None, 'z': round(height(j), 3)}

    def poll_slow(self):
        """systemd states and the depth scan, ~1 Hz from a thread."""
        self.services = {'camera': P.camera(), 'voa': P.voa()}
        self.floors = self.bot.floors()
        if self.mapping is not None and not (self.busy or '').startswith('map save'):
            self.mapping = mapmaker.recording() or self.mapping    # the Orin's distance and time, ~1 Hz
        name = self.bot.floor
        if name is None:
            self.floor = None
        else:
            try:
                if self.floor is None or self.floor['floor'] != name:
                    with open(os.path.join(places.floor_dir(name), 'map.yaml')) as f:
                        info = yaml.safe_load(f)
                    self.floor = {'floor': name, 'res': info['resolution'], 'origin': info['origin'][:2]}
                self.floor['places'] = [[q['name'], q['x'], q['y']] for q in places.load(name)]
            except Exception as e:               # a floor without its files: no map view, the rest goes on
                self.floor = None
                self.warn('map of %s: %s' % (name, e), every=30.0)
        try:
            self.scan = [[b, None if rr is None else round(rr, 2)]
                         for b, rr in self.bot.scan()] \
                if self.services['camera'] and self.bot._node.cloud is not None else None
        except Exception:
            self.scan = None

    def shutdown(self):
        try:
            self.estopped = True
            if self.bot.standing:
                self.bot.estop()
                self.bot.sit()
        finally:
            if self.det:
                self.det.stop()
                self.det.join(2)                # it switches the tracker's detection off on the way out
            if self.talker:
                self.talker.close()
            self.bot.close()


# --- web -------------------------------------------------------------------
class Server:
    def __init__(self):
        self.pages = set()
        self.loop = None
        self.front = Frames()
        self.robot = None
        self.log = []
        self.rs_pages = set()       # pages showing the RealSense view
        self.rs_proc = None         # rs_stream.py, running only while watched
        self.rs_stop = None         # pending stop (timer handle)
        self.model_pages = set()    # pages showing the 3D model

    def emit(self, msg):
        """Thread-safe broadcast to every open page."""
        if msg.get('t') in ('log', 'said'):
            msg['time'] = time.strftime('%H:%M:%S')
            self.log = (self.log + [msg])[-80:]
        if self.loop:
            self.loop.call_soon_threadsafe(asyncio.ensure_future, self._send_all(msg))

    async def _send_all(self, msg):
        data = json.dumps(msg)
        for ws in list(self.pages):
            try:
                await ws.send_str(data)
            except Exception:
                self.pages.discard(ws)

    # --- login (PRIVACY.md): one password, in P.PANEL_PASSWORD_FILE ----------
    @web.middleware
    async def auth(self, request, handler):
        if request.path == '/login':
            return await handler(request)
        given = request.cookies.get(COOKIE) or request.headers.get('X-Panel-Key') or ''
        if hmac.compare_digest(given, self.key):
            return await handler(request)
        if request.path == '/':
            raise web.HTTPFound('/login')
        return web.Response(status=401, text='log in first')

    async def login_page(self, request, wrong=False):
        return web.Response(text=LOGIN_PAGE % ('<p class="bad">Wrong password.</p>' if wrong else ''),
                            content_type='text/html', headers={'Cache-Control': 'no-store'})

    async def login(self, request):
        form = await request.post()
        if not P.panel_password_ok(str(form.get('password', ''))):
            await asyncio.sleep(1.0)            # slows guessing to one try a second a page
            return await self.login_page(request, wrong=True)
        r = web.HTTPFound('/')
        r.set_cookie(COOKIE, self.key, max_age=LOGIN_DAYS * 86400, httponly=True, samesite='Strict')
        raise r

    async def index(self, request):
        # no-cache = ask before reusing a kept copy. Without it a browser may show
        # yesterday's page for hours after a pull (a phone did, 2026-10-07).
        return web.FileResponse(os.path.join(STATIC, 'index.html'),
                                headers={'Cache-Control': 'no-cache'})

    async def ws(self, request):
        ws = web.WebSocketResponse(heartbeat=PING_S)
        await ws.prepare(request)
        self.pages.add(ws)
        try:
            from . import talk
            from .talk import PERSONAS, PERSONA
            personas, persona = sorted(PERSONAS), PERSONA
        except Exception as e:                  # talking unavailable, driving is not
            personas, persona = [], None
            self.robot.log('talk unavailable: %s' % e, 'warn')
        await ws.send_str(json.dumps({'t': 'hello', 'personas': personas,
                                      'persona': persona, 'log': self.log,
                                      'voices': talk.GEMINI_VOICES if personas else [],
                                      'voice': talk.GEMINI_VOICE if personas else None,
                                      'volume': talk.VOLUME if personas else 1.0}))
        try:
            async for m in ws:
                if m.type != WSMsgType.TEXT:
                    continue
                try:
                    await self.command(ws, json.loads(m.data))
                except (ValueError, TypeError, KeyError, AttributeError) as e:
                    # e.g. an emptied number box. Not worth the connection:
                    # losing it stops the drive and costs the page 1.5 s.
                    self.robot.warn('ignored a malformed message: %r' % e)
        finally:
            self.pages.discard(ws)
            self.rs_pages.discard(ws)
            self.model_pages.discard(ws)
            self.rs_update()
            self.robot.drive_cmd = (0.0, 0.0, 0.0)  # a closed page never drives
            if not self.pages:
                self.loop.call_later(GONE_S, self.gone)
        return ws

    async def command(self, ws, c):
        r, k = self.robot, c.get('cmd')
        if k == 'estop':
            await self.loop.run_in_executor(None, r.estop)
        elif k == 'cancel':
            await self.loop.run_in_executor(None, r.cancel)
        elif k == 'drive':
            r.drive(c.get('vx', 0), c.get('wz', 0))
        elif k == 'stand':
            r.submit('stand', r.bot.stand)
        elif k == 'sit':
            r.submit('sit', r.bot.sit)
        elif k == 'nav':
            r.submit('nav ' + ('start' if c.get('on') else 'stop'),
                     r.bot.nav_start if c.get('on') else r.bot.nav_stop)
        elif k == 'goto':
            r.submit('goto %.1f m' % float(c['m']), r.goto,
                     float(c['m']), float(c.get('deg') or 0))
        elif k == 'floor':
            r.submit('load ' + str(c['floor']), r.bot.load_floor, str(c['floor']))
        elif k == 'map_record':
            r.submit('map record', r.map_record)
        elif k == 'map_save':
            r.submit('map save ' + str(c.get('name')), r.map_save, str(c.get('name') or ''))
        elif k == 'map_cancel':
            r.submit('map cancel', r.map_cancel)
        elif k == 'here':
            r.submit('here', r.bot.set_pose, float(c['x']), float(c['y']), float(c.get('deg') or 0))
        elif k == 'go_point':
            # 'goto ...' so the page shows Cancel, as for a Go ahead
            deg = None if c.get('deg') is None else float(c['deg'])    # none: he ends facing the way he came
            r.submit('goto map %.1f, %.1f%s' % (float(c['x']), float(c['y']), '' if deg is None else ' facing %.0f°' % deg),
                     r.go_point, float(c['x']), float(c['y']), deg)
        elif k in ('camera', 'voa'):
            r.submit('%s %s' % (k, 'on' if c.get('on') else 'off'),
                     r.service, k, bool(c.get('on')))
        elif k in ('approach', 'follow'):
            # the page's speed slider; without it, person.py's own default
            kw = {'speed': clamp(abs(float(c['speed'])), DRIVE_MAX_VX)} if c.get('speed') else {}
            if k == 'follow':
                kw['seconds'] = FOLLOW_S
            # With Nav2 up, a follow goes round what is in the way.
            via_nav = k == 'follow' and r.bot.nav_running
            r.submit(k, lambda: r.to_person(person.follow_nav if via_nav else getattr(person, k), **kw))
        elif k == 'person':
            r.detect(bool(c.get('on')))
        elif k == 'watch' and c.get('cam') == 'realsense':
            (self.rs_pages.add if c.get('on') else self.rs_pages.discard)(ws)
            self.rs_update()
        elif k == 'watch' and c.get('cam') == 'model':
            (self.model_pages.add if c.get('on') else self.model_pages.discard)(ws)
        elif k == 'volume':
            from . import talk
            talk.VOLUME = min(1.0, max(0.0, float(c['v'])))
        elif k == 'voice':
            r.voice(c.get('v'))
        elif k == 'public':
            P.set_public_mode(bool(c.get('on')))
            r.log('public mode ' + ('on: looking is off' if c.get('on') else 'OFF: camera frames may go to Gemini'),
                  None if c.get('on') else 'warn')
        elif k in ('say', 'ask', 'look'):
            r.talk(k, (c.get('text') or '').strip(), c.get('persona'))

    async def floor_map(self, request):
        """A floor's map as the map server reads it (PGM); the page paints it."""
        try:
            return web.FileResponse(os.path.join(places.floor_dir(request.match_info['floor']), 'map.pgm'),
                                    headers={'Cache-Control': 'no-cache'})
        except Lite3Error:
            return web.Response(status=404)

    def gone(self):
        if not self.pages:                      # nobody came back
            self.loop.run_in_executor(None, self.robot.abandoned)

    async def stream_front(self, request):
        """The front camera as MJPEG, for a page whose WebRTC did not connect."""
        frames = self.front
        resp = web.StreamResponse(headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
            'Cache-Control': 'no-cache'})
        await resp.prepare(request)
        frames.viewers += 1
        seq = -1
        try:
            while True:
                jpeg, seq = await self.loop.run_in_executor(None, self._next, frames, seq)
                if jpeg is None:
                    continue
                await resp.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                 b'Content-Length: %d\r\n\r\n' % len(jpeg) + jpeg + b'\r\n')
        except (ConnectionResetError, RuntimeError, asyncio.CancelledError):
            pass                             # the page went away
        finally:
            frames.viewers -= 1
        return resp

    def rs_update(self):
        """Run rs_stream.py while some page shows the RealSense view (it costs
        about half a core). Stopping waits RS_LINGER_S, so a page that drops
        its websocket and reconnects keeps its video."""
        if self.rs_pages:
            if self.rs_stop:
                self.rs_stop.cancel()
                self.rs_stop = None
            if self.rs_proc is None or self.rs_proc.poll() is not None:
                self.rs_proc = subprocess.Popen(
                    [sys.executable, '-m', 'robot.rs_stream'], cwd=REPO)
        elif self.rs_proc and not self.rs_stop:
            self.rs_stop = self.loop.call_later(RS_LINGER_S, self.rs_kill)

    def rs_kill(self):
        self.rs_stop = None
        if self.rs_proc:
            self.rs_proc.terminate()
            self.rs_proc = None

    async def whep(self, request):
        """WebRTC signalling for a camera: pass the page's offer to mediamtx,
        and point the answer's video address at udp_relay."""
        url = {'front': P.CAMERA_WHEP, 'realsense': P.RS_WHEP}.get(request.match_info['cam'])
        if not url:
            return web.Response(status=404)
        offer, cam = await request.read(), request.match_info['cam']
        try:
            async with ClientSession() as s:
                # rs_stream.py has only just been started by the page's
                # 'watch': 404 until its first frames reach mediamtx (~3 s).
                for attempt in range(20 if cam == 'realsense' else 1):
                    async with s.post(url, data=offer,
                                      headers={'Content-Type': 'application/sdp'},
                                      timeout=5) as r:
                        status, sdp = r.status, await r.text()
                    if status != 404:
                        break
                    await asyncio.sleep(0.5)
                if status != 201:
                    return web.Response(status=502, text='mediamtx said %d' % status)
        except Exception as e:
            return web.Response(status=502, text='mediamtx unreachable: %s' % e)
        lines = [l for l in sdp.splitlines() if not l.startswith('a=candidate:')]
        at = lines.index('a=end-of-candidates') if 'a=end-of-candidates' in lines else len(lines)
        local = request.transport.get_extra_info('sockname')[0] if request.transport else self.host
        lines.insert(at, 'a=candidate:1 1 udp 2130706431 %s %d typ host'
                     % video_candidate(request.host, local, self.host))
        return web.Response(status=201, text='\r\n'.join(lines) + '\r\n',
                            content_type='application/sdp')

    @staticmethod
    def _next(frames, seq):
        with frames.cond:
            frames.cond.wait_for(lambda: frames.seq != seq, timeout=2.0)
            return frames.jpeg, frames.seq

    async def ticker(self):
        n = 0
        while True:
            try:
                if n % 2 == 0:
                    await self.loop.run_in_executor(None, self.robot.poll_slow)
                await self._send_all(self.robot.status())
            except Exception as e:               # keep the status flowing
                self.robot.log('status: %s' % e, 'error')
            n += 1
            await asyncio.sleep(0.5)

    async def model_ticker(self):
        while True:
            try:
                m = self.model_pages and self.robot.model()
            except Exception as e:              # keep the ticker alive
                self.robot.warn('model: %r' % e)
                m = None
            if m:
                data = json.dumps(m)
                for ws in list(self.model_pages):
                    try:
                        await ws.send_str(data)
                    except Exception:
                        self.model_pages.discard(ws)
            await asyncio.sleep(1.0 / MODEL_HZ)

    def main(self):
        # Tailscale, and his own network for a phone on his wifi. In handheld mode (no internet)
        # Tailscale may have no address: then his own network alone.
        lan = ip_of(LAN_DEV)
        host = self.host = ip_of('tailscale0') or lan
        if not host:
            raise SystemExit('neither tailscale0 nor %s has an address' % LAN_DEV)
        self.loop = asyncio.get_event_loop()
        self.robot = Robot(self.emit)
        FrontCamera(self.front).start()
        self.key = P.panel_key()               # makes the password file on the first start
        app = web.Application(middlewares=[self.auth])
        app.router.add_get('/login', self.login_page)
        app.router.add_post('/login', self.login)
        app.router.add_get('/', self.index)
        app.router.add_get('/ws', self.ws)
        app.router.add_get('/stream/front', self.stream_front)
        app.router.add_post('/whep/{cam}', self.whep)
        app.router.add_get('/map/{floor}', self.floor_map)
        app.router.add_static('/lib', os.path.join(STATIC, 'lib'))      # three.js
        app.router.add_static('/model', os.path.join(REPO, 'urdf'))     # lite3.urdf, meshes
        runner = web.AppRunner(app)
        self.loop.run_until_complete(runner.setup())
        self.loop.run_until_complete(web.TCPSite(runner, host, PORT).start())
        # localhost too, so `tailscale serve` can put HTTPS in front of it
        self.loop.run_until_complete(web.TCPSite(runner, '127.0.0.1', PORT).start())
        # and the robot's own network, for a phone on his wifi with no Tailscale (handheld
        # mode): the motion computer forwards http://192.168.2.1:8080 here (env/robot-nat.sh)
        if lan and lan != host:
            self.loop.run_until_complete(web.TCPSite(runner, lan, PORT).start())
        relay = subprocess.Popen(
            [sys.executable, '-m', 'robot.udp_relay', '0.0.0.0', str(P.CAMERA_RTC_PORT),
             P.MOTION_IP, str(P.CAMERA_RTC_PORT)],
            cwd=REPO)
        self.loop.create_task(self.ticker())
        self.loop.create_task(self.model_ticker())
        for sig in (signal.SIGINT, signal.SIGTERM):
            self.loop.add_signal_handler(sig, self.loop.stop)
        print('HMI on %shttp://%s:%d on his own wifi - Ctrl-C sits him and stops'
              % ('http://%s:%d (tailnet) and ' % (host, PORT) if host != lan else '', P.AP_IP, PORT), flush=True)
        try:
            self.loop.run_forever()
        finally:
            relay.kill()
            self.rs_kill()
            print('shutting down: halting and sitting ...', flush=True)
            self.robot.shutdown()


if __name__ == '__main__':
    Server().main()
