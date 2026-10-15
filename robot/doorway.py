"""doorway - what holds him up on a route: a closed door, or something else.

    python3 -m robot.doorway check      its own check, no robot

nav.go_to_point() asks Watch.check() about four times a second while Nav2
drives. A spot on the route ahead that the costmap now calls blocked, with
the floor map's walls close on both sides of it, is a closed door: he stops
there and asks for it to be opened (Nav2 would otherwise go another way at
once). Anything else blocking him is asked to let him pass, once he has
stood held up for a moment. README 7.6.
"""
import math
import re
import time

LOOK_M = 4.0        # along the route: the lidar marks the costmap out to 3 m
BLOCKED = 99        # costmap value Nav2 will not put his centre on (inscribed); 100 is lethal
DOOR_HALF_M = 0.8   # a doorway: the map has wall this close on both sides of the route
DOOR_ALONG_M = 0.5  # looked for this far either side of the blocked point: a shut door's inflation blocks the route first
SIDE_STEP_M = 0.05
ASK_NEAR_M = 1.5    # something else in the way this close ...
STILL_S, STILL_M = 2.0, 0.2     # ... while he has moved less than this for this long: ask once
SAME_DOOR_M = 1.0   # a door gone round counts as this wide
CHECK_S = 0.25
FREE_OVER = 216     # map.yaml's free_thresh 0.15 as a grey level


def blocked_ahead(route, pose, cost, look=LOOK_M):
    """(point, index, metres ahead) of the first route point within `look` of
    the nearest one to pose whose cost is BLOCKED or more; None if clear."""
    if not route:
        return None
    i0 = min(range(len(route)), key=lambda i: math.hypot(route[i][0] - pose[0], route[i][1] - pose[1]))
    ahead = 0.0
    for i in range(i0, len(route)):
        if i > i0:
            ahead += math.hypot(route[i][0] - route[i - 1][0], route[i][1] - route[i - 1][1])
        if ahead > look:
            return None
        c = cost(*route[i])
        if c is not None and c >= BLOCKED:
            return route[i], i, ahead
    return None


def is_door(route, i, free):
    """Is the route at point i, or within DOOR_ALONG_M of it, a doorway: the
    floor map's wall (or unseen) within DOOR_HALF_M on both sides of it?
    free(x, y): the map has floor there."""
    steps = [SIDE_STEP_M * k for k in range(1, int(DOOR_HALF_M / SIDE_STEP_M) + 1)]
    for j in range(len(route)):
        x, y = route[j]
        if math.hypot(x - route[i][0], y - route[i][1]) > DOOR_ALONG_M:
            continue
        a, b = route[max(j - 1, 0)], route[min(j + 1, len(route) - 1)]
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d == 0:
            continue
        nx, ny = -(b[1] - a[1]) / d, (b[0] - a[0]) / d          # across the route
        if all(any(not free(x + s * nx * side, y + s * ny * side) for s in steps) for side in (1, -1)):
            return True
    return False


class StaticMap:
    """A floor's map.pgm and map.yaml: free(x, y) in the map frame. A missing
    map makes everything free, so nothing is ever taken for a door."""

    def __init__(self, pgm, yaml_path):
        try:
            data = open(pgm, 'rb').read()
            text = open(yaml_path).read()
        except OSError:
            self.grid = None
            return
        m = re.match(rb'P5\s+(\d+)\s+(\d+)\s+255\s', data)
        self.w, self.h = int(m.group(1)), int(m.group(2))
        self.grid = data[m.end():]
        self.x0, self.y0 = map(float, re.search(r'origin: \[([-0-9.e]+), ([-0-9.e]+)', text).groups())
        self.res = float(re.search(r'resolution: ([0-9.e-]+)', text).group(1))

    def free(self, x, y):
        if self.grid is None:
            return True
        c, r = int((x - self.x0) / self.res), self.h - 1 - int((y - self.y0) / self.res)
        return 0 <= c < self.w and 0 <= r < self.h and self.grid[r * self.w + c] > FREE_OVER


class Watch:
    """One goal's route, watched. check(pose): the point of a closed door ahead
    to stop at, or None; says 'other' once while something else holds him up."""

    def __init__(self, route, cost, free, say):
        self.route, self.cost, self.free, self.say = route, cost, free, say
        self.door = None        # the closed door check() last stopped him at
        self.skip = []          # doors he is going round: not stopped at again
        self.asked = False
        self.seen = []          # (t, x, y): where he has been lately
        self.next = 0.0

    def go_round(self, door, route):
        """Taking another way: forget this door, watch the new route."""
        self.skip.append(door)
        self.route, self.seen, self.asked = route, [], False

    def check(self, pose):
        now = time.time()
        if now < self.next:
            return None
        self.next = now + CHECK_S
        self.seen = [s for s in self.seen if now - s[0] <= STILL_S] + [(now, pose[0], pose[1])]
        hit = blocked_ahead(self.route, pose, self.cost)
        if hit is None:
            self.asked = False
            return None
        point, i, ahead = hit
        if any(math.hypot(point[0] - d[0], point[1] - d[1]) < SAME_DOOR_M for d in self.skip):
            return None
        if is_door(self.route, i, self.free):
            self.door = point
            return point
        held = now - self.seen[0][0] >= STILL_S - CHECK_S and all(
            math.hypot(x - pose[0], y - pose[1]) < STILL_M for _, x, y in self.seen)
        if ahead <= ASK_NEAR_M and held and not self.asked:
            self.asked = True
            self.say('other')
        return None


def check():
    # a corridor along y = 0, a wall across it at x = 2 with a 0.8 m gap: the door
    def free(x, y):
        return not (abs(x - 2.0) < 0.06 and abs(y) > 0.4)
    route = [(0.1 * k, 0.0) for k in range(51)]                # (0, 0) to (5, 0)
    shut = {(2.0, 0.0)}
    cost = lambda x, y: 100 if (round(x, 1), round(y, 1)) in shut else 0      # noqa: E731
    hit = blocked_ahead(route, (0.0, 0.0, 0.0), cost)
    assert hit[:2] == ((2.0, 0.0), 20) and abs(hit[2] - 2.0) < 1e-6, hit
    assert blocked_ahead(route, (0.0, 0.0, 0.0), cost, look=1.5) is None        # too far ahead to matter yet
    assert blocked_ahead(route, (3.0, 0.0, 0.0), cost) is None                  # behind him
    assert is_door(route, 20, free) and not is_door(route, 30, free)            # the gap; open corridor
    assert is_door(route, 17, free) and not is_door(route, 14, free)            # 0.3 m before it (the shut door's inflation
                                                                                # blocks the route first); 0.6 m before: no
    assert not is_door(route, 20, lambda x, y: y > -0.6)                         # wall on one side only: not a door

    said, t = [], [100.0]
    real = time.time
    time.time = lambda: t[0]
    try:
        w = Watch(route, cost, free, said.append)
        assert w.check((0.5, 0.0, 0.0)) == (2.0, 0.0) and not said               # the door, at once
        w.go_round((2.0, 0.0), route)
        t[0] += 1
        assert w.check((0.5, 0.0, 0.0)) is None                                  # gone round: not again
        # something in the open, 1 m ahead: asked to move once he has been held up 2 s, and only once
        shut.clear()
        shut.add((4.0, 0.0))
        w = Watch(route, cost, free, said.append)
        for k in range(12):                                                      # 3 s standing at x = 3
            t[0] += CHECK_S
            assert w.check((3.0, 0.0, 0.0)) is None
        assert said == ['other'], said
        shut.clear()
        t[0] += CHECK_S
        w.check((3.0, 0.0, 0.0))                                                 # it moved: asked again next time
        shut.add((4.0, 0.0))
        for k in range(12):
            t[0] += CHECK_S
            w.check((3.0, 0.0, 0.0))
        assert said == ['other', 'other'], said
        # walking up to it is not being held up
        said.clear()
        w = Watch(route, cost, free, said.append)
        for k in range(12):
            t[0] += CHECK_S
            w.check((2.5 + 0.04 * k, 0.0, 0.0))
        assert not said, said
    finally:
        time.time = real
    m = StaticMap('/nonexistent.pgm', '/nonexistent.yaml')
    assert m.free(1.0, 1.0)
    print('doorway: check passed')


if __name__ == '__main__':
    import sys
    if sys.argv[1:] == ['check']:
        check()
    else:
        sys.exit(__doc__)
