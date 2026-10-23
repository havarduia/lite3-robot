#!/usr/bin/env python3
"""The robot walks a route, and says a line at the end (--finale).

    python3 ~/robot/bin/tour.py                          # walk 2 m forward
    python3 ~/robot/bin/tour.py "walk 1.5, turn 90, walk 1, turn -90"
    python3 ~/robot/bin/tour.py "goto 2.5, goto 1.5 90"  # Nav2: routes around obstacles
    python3 ~/robot/bin/tour.py --interactive            # type steps one at a time
    python3 ~/robot/bin/tour.py --speed 0.2 "walk 3"
    python3 ~/robot/bin/tour.py --finale "Hello, welcome to the lab." "goto 2"

Steps (comma separated, or one per line with --interactive):
    walk <m>            straight line, stops for obstacles within --stop
    turn <deg>          in place, +ve = left
    goto <m> [<deg>]    Nav2 goal <m> ahead, then turn <deg>. Routes around
                        obstacles; if the spot itself is blocked, goes to the
                        closest free distance on that line
    approach <m>        Nav2 toward <m> ahead, stopping short of the FIRST
                        obstacle on that line

He stands first and sits at the end, and skips a step that fails. At the end
he says --finale, tilted up (--look-up), or how far he walked. The speech is Piper, on this computer: nothing goes online.
Without --handheld it sends its own heartbeat, so Ctrl-C (an e-stop) and the
power switch are the only stops. More in docs/tools.md.
"""
import argparse
import contextlib
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robot.nav import LETHAL, dist, goal_ahead, status_text
from robot.protocol import Lite3Error
from robot.talk import Voice

DONE = 'That was my walk: %.1f metres.'
APPROACH_MARGIN = 0.2  # stop this far short of the last free cell
MIN_GOAL = 0.5         # set when xy_goal_tolerance was 0.25 (0.15 now): a nearer goal "arrives" at once and he only turns
STEPS = 'walk <m> | turn <deg> | goto <m> [<deg>] | approach <m>'


def settled_profile(bot, out_to):
    """cost_ahead() once it stops changing, printed every 0.3 m."""
    prof = bot.cost_ahead(out_to=out_to, step=0.1, settle=10)
    print('\n[costmap ahead: %s]' % ' '.join(
        '%.1f:%s' % (d, '?' if c is None else c) for d, c in prof[::3]))
    return prof


def parse_step(step):
    """'walk 1.5' -> ('walk', [1.5]); 'goto 2 90' -> ('goto', [2.0, 90.0])"""
    verb, *args = step.split()
    nargs = {'walk': (1,), 'turn': (1,), 'goto': (1, 2), 'approach': (1,)}
    if verb not in nargs or len(args) not in nargs[verb]:
        raise ValueError('bad step %r: use  %s' % (step, STEPS))
    return verb, [float(x) for x in args]


def parse_route(text):
    try:
        return [parse_step(s) for s in filter(None, (s.strip() for s in text.split(',')))]
    except ValueError as e:
        raise SystemExit(e)


class Odometer(threading.Thread):
    """Path length walked, sampled at 2 Hz, so the closing line is true.
    Steps under 2 cm are ignored: standing-still odometry noise adds up.
    """

    def __init__(self, bot):
        super().__init__(daemon=True)
        self.bot, self.total, self.last = bot, 0.0, None

    def reset(self):
        self.last = self.bot.wait_pose()[:2]
        if not self.is_alive():
            self.start()

    def run(self):
        while True:
            time.sleep(0.5)
            self.sample()

    def sample(self):
        if self.last is None:
            return
        p = self.bot.pose
        if p is None:
            return
        d = dist(self.last, p)
        if d >= 0.02:
            self.total += d
            self.last = p[:2]


def goto_step(bot, want, *heading):
    """goto <m> [<deg>]: a Nav2 goal <m> ahead, or at the free distance closest
    to that if the spot itself is inside an obstacle."""
    prof = settled_profile(bot, want + 1.5)
    if not any(abs(d - want) < 0.05 and c is not None and c < LETHAL
               for d, c in prof):
        # Nearest free distance on the line, ties to the shorter one.
        # Beyond an obstacle is allowed: Nav2 routes around it.
        free = [d for d, c in prof
                if d >= MIN_GOAL and c is not None and c < LETHAL]
        if not free:
            raise Lite3Error('nothing free between %.1f and %.1f m ahead'
                             % (MIN_GOAL, want + 1.5))
        closest = min(free, key=lambda d: (abs(d - want), d))
        print('\n[goto %.1f m is inside an obstacle; using %.1f m, the '
              'closest free distance]' % (want, closest))
        want = closest
    start = bot.wait_pose()
    status = bot.goto(want, *heading)
    there = bot.wait_pose()
    time.sleep(2.0)
    print('\n[goto %.1f m: %s - moved %.2f m, then drifted %.2f m in 2 s]' % (
        want, status_text(status), dist(start, there), dist(there, bot.wait_pose())))


def run_step(bot, a, verb, args):
    try:
        if verb == 'walk':
            # walk() samples the depth camera at 4 Hz and has no slow-down
            # ramp, so give it half a second of travel as extra margin.
            r = bot.walk(args[0], speed=a.speed,
                         stop_distance=None if a.blind else a.stop + 0.5 * a.speed)
            print('\n[walked %.2f m, %s]' % (r['moved'], r['reason']))
        elif verb == 'turn':
            bot.turn_deg(args[0])
            print('\n[turned %+.0f deg]' % args[0])
        elif verb == 'approach':
            prof = settled_profile(bot, args[0])
            # Short of the FIRST obstacle on the line: a free cell BEHIND it
            # would be routed to around it, i.e. past the thing being
            # approached.
            goal = goal_ahead(prof, APPROACH_MARGIN, MIN_GOAL)
            if goal is None:
                raise Lite3Error('obstacle within 0.5 m ahead, nowhere to approach to')
            print('[approach: goal %.1f m ahead]' % goal)
            print('\n[approach %.1f m: %s]' % (goal, status_text(bot.goto(goal))))
        else:
            goto_step(bot, *args)
    except Lite3Error as e:
        print('\n[%s %s skipped: %s]' % (verb, ' '.join('%g' % x for x in args), e))


def steps_from_stdin():
    print('Steps: %s. Ctrl-D to finish.' % STEPS)
    while True:
        try:
            line = input('step> ').strip()
        except EOFError:
            return
        if line:
            try:
                yield parse_step(line)
            except ValueError as e:
                print(e)


def start_nav(bot):
    # Standing makes leg odometry jump; nav_start() refuses until the pose has
    # settled, so give it a moment first.
    bot._wait(bot.pose_settled, 15, 0.5)
    print('[starting Nav2...]', flush=True)
    bot.nav_start()
    print('[Nav2 up]', flush=True)


def run_route(bot, a, route, track):
    """Every step of the route (typed ones if it is None)."""
    for verb, args in steps_from_stdin() if route is None else route:
        run_step(bot, a, verb, args)
        track.sample()


def finale(bot, a, voice):
    """Say --finale, tilted nose-up (--look-up) to face whoever is in front of him."""
    with (bot.tilt(a.look_up, settle=0.3) if a.look_up
          else contextlib.nullcontext()):
        print('\n%s' % a.finale, flush=True)
        voice.say(a.finale, voice=a.voice)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('route', nargs='?', default='walk 2')
    ap.add_argument('--interactive', action='store_true',
                    help='read steps from the keyboard instead of the route')
    ap.add_argument('--speed', type=float, default=0.3,
                    help='m/s, max 0.6. Below ~0.2 the gait shuffles and drifts '
                         'sideways. walk steps add speed * 0.5 s to --stop as '
                         'reaction margin')
    ap.add_argument('--stop', type=float, default=1.0,
                    help='obstacle stop distance in metres, measured from the '
                         'BODY CENTRE: the nose is ~0.33 m ahead of it, so '
                         '0.45 stops ~12 cm short of the nose')
    ap.add_argument('--blind', action='store_true',
                    help='walk without the depth-camera obstacle stop')
    ap.add_argument('--handheld', dest='heartbeat', action='store_false',
                    help='use the powered-on handheld instead of our own heartbeat')
    ap.add_argument('--voice', default=None, help='a Piper voice (python3 -m robot.talk voices)')
    ap.add_argument('--finale', default=None,
                    help='at the end, still standing, say this instead of how far he walked')
    ap.add_argument('--look-up', type=float, default=12.0, metavar='DEG',
                    help='tilt the body nose-up this much for the finale, so he '
                         'faces the person, not their knees (max ~14, 0 = off)')
    a = ap.parse_args()
    route = None if a.interactive else parse_route(a.route)
    use_nav = a.interactive or any(v in ('goto', 'approach') for v, _ in route)
    if not a.blind and a.stop < 0.4:
        raise SystemExit('--stop %.2f is inside or at the robot\'s nose (~0.33 m from '
                         'the body centre the distance is measured from); the '
                         'camera cannot see that close either. Use >= 0.4.' % a.stop)

    voice = Voice()
    from robot.lite3 import Lite3
    # upright(): arm, stand, and on the way out sit FIRST; close() then
    # disarms SECOND.
    with Lite3() as bot, bot.upright(heartbeat=a.heartbeat):
        track = Odometer(bot)
        try:
            if use_nav:
                start_nav(bot)
            track.reset()
            run_route(bot, a, route, track)
            if a.finale:
                finale(bot, a, voice)
        finally:
            bot.halt()
            if use_nav:
                bot.nav_stop()
    if not a.finale:
        print('\n' + DONE % track.total, flush=True)
        voice.say(DONE % track.total, voice=a.voice)
    return 0


if __name__ == '__main__':
    sys.exit(main())
