# Lidar, switch and Orin Nano: what we did on 2026-10-08

A step-by-step record of one working session: a switch, a lidar and an Orin
Nano were added to the Lite3, and by the end FAST-LIO2's pose reaches the
robot's own ROS graph. Written in the order things happened, including the
mistakes. How the code works is in `README.md` §10.7; this file is the
story. Logins are left out on purpose.

## Where we ended up

| Thing | State |
|---|---|
| Network | All four devices reach each other, no configuration needed |
| Lidar driver on the Orin | Running, 10 Hz cloud and 200 Hz IMU |
| FAST-LIO2 on the Orin | Installed; runs only during a Live run from its web panel |
| `robot/lio_relay.py` | Publishes `/lio_odom` and `/odom_fused` on the robot's graph |
| Nav2 and `Lite3` | Switched to `/odom_fused` and deployed on the robot (sections 10, 11). Walked by handheld; `walk()`, `turn()` and one Nav2 goal run on it |

## 1. The hardware

A TP-Link TL-SG105E switch now rides on the robot.

| Switch port | Device | Address |
|---|---|---|
| 1 | Perception computer (Jetson Xavier NX) | 192.168.1.103 |
| 4 | Livox MID-360 lidar | 192.168.1.102 |
| 5 | Orin Nano, on top of the robot | 192.168.1.5 |

The motion computer (192.168.1.120) and the perception computer are joined
by a cable inside the robot. That cable is always there; the switch does
not replace it.

## 2. Can everything talk to each other?

Goal: every device reaches every other one.

1. ssh to the perception computer over Tailscale. It stopped on a Tailscale
   check that had to be approved in a browser.
2. From the perception computer: pinged the motion computer, the lidar and
   the Orin. All answered.
3. From the motion computer (hopping through the perception computer):
   pinged the perception computer, the lidar and the Orin. All answered.
4. From the Orin, once its login was known: pinged the other three. All
   answered.

Result: nothing needed setting up. Everything is on 192.168.1.x and the
switch just passes traffic. The laptop's ssh key was added to the Orin, so
this now works without a password:

```
ssh -J ysc@lite3-perception lite3@192.168.1.5
```

Also seen: the Orin has its own campus Wi-Fi and its own Tailscale address,
which is not in the laptop's tailnet list.

## 3. Is the lidar publishing on the Orin?

Rule set here: nothing gets changed on the Orin without asking first.

What was found, read-only:

- ROS 2 **Humble** (not Foxy), Fast DDS, `ROS_DOMAIN_ID=42`.
- JetPack 6 line: L4T 36.5.2, Ubuntu 22.04.5, Orin Nano Super developer
  kit. JetPack 6 is Ubuntu 22.04, which is the Humble release; Foxy would
  need JetPack 5.
- `livox-driver.service` running, publishing `/livox/lidar` (PointCloud2)
  and `/livox/imu`. `lidar-web.service` is a web panel on port 8000.
- The sensor was sending about 3 MB/s to the Orin.

### The problem I caused

A new subscriber on the Orin got no messages, although the topics were
listed and the driver was busy.

- Cause: the driver runs as user `lite3`. When the last `lite3` login
  session ends, Ubuntu deletes that user's shared-memory files, and ROS
  uses those files to pass messages between programs on the same machine.
- My first ssh login ended at 15:43:11. After that the driver was holding
  deleted files (`/dev/shm/fastrtps_port17915_el (deleted)`).
- Any ssh logout as `lite3` would have done the same.
- Only programs on the Orin itself were affected. Over the network the
  data kept flowing.

### The fix

Two commands on the Orin, run by the user (Claude Code's permission check
blocked me from running the first, because it changes an account setting):

```
sudo loginctl enable-linger lite3
sudo systemctl restart livox-driver.service
```

Verified afterwards: `Linger=yes`, 200 Hz and 10 Hz for a local subscriber,
and still arriving in a second login after the first had logged out.

## 4. Can Foxy and Humble talk?

Tested from the perception computer (Foxy), listening on the Orin's
domain 42:

| DDS on the perception computer | Result |
|---|---|
| Fast DDS | Works: `/livox/imu` 200 Hz, `/livox/lidar` 10.0 Hz |
| CycloneDDS (what the robot's nodes use) | Every `ros2` command segfaults on meeting the Orin's node |

Control: the same CycloneDDS command on an empty domain (43) did not crash.

What follows from it:

- Nothing is at risk as things are: the robot's nodes are on domain 0, the
  Orin on 42, and they do not see each other.
- The Orin must not be moved to domain 0. The robot's CycloneDDS nodes
  could crash the same way, including the bridge to the motion computer.

## 5. FAST-LIO2 on the Orin

- It is installed (`~/livox_ws/src/FAST_LIO`, patched) but is not a
  service. It starts when Live is pressed in the web panel, or after a
  recording.
- The Orin's own notes: `~/bin/README.md` (mapping) and
  `~/robot_pc/README.md` (a navigation plan that expects the perception
  computer on domain 42 with Fast DDS). That plan was never copied to the
  perception computer, and it does not match this repo's setup (CycloneDDS,
  domain 0).
- It publishes the lidar's pose at 10 Hz on `/Odometry`. It has no loop
  closure and cannot find itself in a saved map; every run starts at its
  own origin.

## 6. Getting the pose to Nav2: the choice

Two uses were on the table:

1. Better odometry: replace the leg odometry Nav2 uses. Nav2 stays mapless.
2. Navigating in a saved map: needs localization, which FAST-LIO2 does not
   do.

Chosen: 1.

How to carry it across: a relay on the perception computer. Two programs
joined by a pipe, the idea `robot/rviz.py` already uses. One listens on
Fast DDS, domain 42; the other publishes on the robot's own graph. Nothing
changes on the Orin or in the robot's services. The alternative, moving
the robot's whole ROS stack to Fast DDS, would touch the vendor services.

## 7. The relay, first version: `/lio_odom`

`robot/lio_relay.py` turns FAST-LIO2's lidar pose into the body's pose
(`odom -> base_link`), stamps it like `/leg_odom2`, and fills in speed
from consecutive poses.

Tests, in order:

1. The maths self-check (`python3 -m robot.lio_relay check`).
2. On the robot from a temporary folder, with a made-up pose published on
   domain 42 (lidar 1 m along its own x): `/lio_odom` showed x 0.937,
   z -0.349 at 10.0 Hz. That is the right answer for the 20.4° mount
   pitch.
3. Committed (`ea54663`) and pulled on the robot.
4. With a real Live run, robot still on a table: 10 Hz, position steady
   within a few millimetres, speed noise about ±0.01 m/s.

Should FAST-LIO2 always run? Not yet. It must start with the robot standing
still, it has diverged once on the current build for an unknown reason, and
today only the mapping panel starts it.

## 8. The walked loops

A logger recorded `/lio_odom` and `/leg_odom2` side by side while the user
drove a closed loop with the handheld. On a closed loop, the distance
between end and start is the drift.

**First loop: not usable.** The robot was already walking when logging
started, and the loop was not finished when the three minutes ran out.
What it did show: no dropouts or jumps, and the two agreed closely on
heading and distance.

**Second loop: clean.** Standing still on a floor mark at the start,
parked on the same mark at the end.

| | FAST-LIO2 (`/lio_odom`) | Leg odometry (`/leg_odom2`) |
|---|---|---|
| Distance walked | 10.40 m | 10.19 m |
| Gap between end and start | 0.05 m | 0.21 m |
| Heading compared with start | -5.5° | -5.2° |

FAST-LIO2 was clearly better on this loop. Both gave the same heading, so
the 5° is how the robot was parked. A 5 cm gap also leaves little room for
a wrong lidar mount offset. It is one 10 m loop: it says nothing about a
long walk, or about standing up from a sit.

## 9. The relay, second version: `/odom_fused`

Goal: an odometry Nav2 can drive on, which falls back to the legs if
FAST-LIO2 jumps or goes silent, without the pose ever jumping.

How it works:

- The fused pose is the leg pose moved by a correction. FAST-LIO2 updates
  the correction ten times a second; the legs supply the 50 Hz in between.
- A FAST-LIO2 step is believed only if it agrees with the legs' step over
  the same stretch (within 0.15 m and 0.2 rad). Otherwise the correction
  is kept and FAST-LIO2 is re-anchored to where the legs put the robot.
- So with the Orin silent, `/odom_fused` is `/leg_odom2` exactly.
- Not caught: FAST-LIO2 drifting slowly.

Measured for it: FAST-LIO2's pose arrives roughly 0.05 s late (from the
second loop's log), so each pose is matched with the leg pose from that
long ago.

Tests:

1. Self-check: agreement, legs under-counting, FAST-LIO2 shooting away, a
   silence followed by a new run.
2. On the robot, standing still with the Live run on: `/odom_fused` at
   50.0 Hz, within 3 mm of `/leg_odom2`, and the relay reported "corrected
   by FAST-LIO2".
3. Cost: about 25% of a core for the relay plus 8% for its listening
   child, averaged over the first 28 s.

## 10. Switching everything to `/odom_fused`

Nav2, `Lite3`, the sonar node and the rviz relay pass goals and obstacle
positions to each other in one odom frame, so they had to switch together.
Two ways were on the table: the relay always running as a service, or only
while Nav2 runs. Chosen: always, as a service. With FAST-LIO2 off,
`/odom_fused` is leg odometry exactly, so nothing changes until a Live run
is on.

What changed in the repo:

- `robot/lite3.py`: reads `/odom_fused`. New stop: if the newest pose is
  older than 0.5 s, a motion under way stops ("odometry stopped
  arriving") and a new one refuses to start. Needed because the relay is
  now a process that can die while the bridge still takes velocities.
- `robot/sonar_range.py`, `robot/rviz.py`: read `/odom_fused`. Old rviz
  recordings still play.
- `env/nav2-mapless-lite3.patch`: `odom_to_tf.py` makes
  `odom -> base_link` from `/odom_fused`.
- `env/lio_relay.service`: the systemd unit.
- `demos/check_motion.py`: two new checks on the fake robot (the relay
  dying mid-walk; setting off without odometry). Passes.

**Starting FAST-LIO2 from code.** The Orin's panel turned out to have an
HTTP API (`/api/live/start`, `/stop`, `/state`), reachable from the
perception computer. So nothing on the Orin changed:
`python3 -m robot.lio_relay start|stop|state`, or `lio_relay.live(...)`.
Only `state` was tried, because a Live run was on. Nothing calls it
automatically yet.

## 11. Deploying it on the robot

Order matters: once the new `Lite3` is pulled, it will not move without
the service.

1. Commit and push; pull in `~/robot`.
2. Install and start `lio_relay.service`.
3. Apply the `odom_to_tf.py` change in the vendor tree (source and
   installed copy).
4. Restart the web panel.
5. Walk a loop on `/odom_fused`, then try Nav2 on it.

Done 2026-10-08, with the robot standing and a Live run on:

1. Pushed (`70682f8`) and pulled.
2. Service installed, enabled and started. Its log said "/odom_fused is
   corrected by FAST-LIO2"; `/odom_fused` at 49.9 Hz.
3. `odom_to_tf.py` changed in both places, backups beside them as
   `odom_to_tf.py.pre-fused`. The repo's patch hunk reverses cleanly
   against the result, so the two match.
4. The panel had been restarted after the pull, before the service
   existed. It needed no second restart: it only waits for the topic.

5. Walked loop on `/odom_fused` (handheld, mark to mark, 96 s):

| | Fused | FAST-LIO2 | Leg odometry |
|---|---|---|---|
| Distance walked | 9.50 m | 9.44 m | 9.05 m |
| Gap between end and start | 0.033 m | 0.032 m | 0.073 m |
| Heading compared with start | -3.3° | -3.2° | -1.9° |

   The fused pose stayed within 0.029 m of FAST-LIO2 all the way and
   ended 0.001 m from it. The relay never left "corrected by FAST-LIO2",
   and no step between samples was larger than walking speed.

6. The robot driving itself on it, from the command line, user watching:

| Command | Result | Measured by the fused odometry |
|---|---|---|
| `walk 0.5` | target reached | moved 0.52 m |
| `turn 90` | target reached | turned 90.0° |
| `turn -90` | target reached | turned -89.2° |

   The relay stayed on "corrected by FAST-LIO2" throughout.

7. Nav2 on it (the user started Nav2 from the panel; I sent the goal):

- `odom -> base_link` came from the fused odometry: `tf2_echo` showed
  the same position as `/odom_fused`.
- `goto 1.5`: Nav2 reported the goal reached. The fused pose moved
  1.32 m.
- The relay stayed on "corrected by FAST-LIO2" and did not restart.
- In the Nav2 log: 38 dropped camera frames inside one second, and a few
  "Range sensor layer can't transform from odom to sonar_front/rear".
  The dropped frames came while I was starting several check commands on
  the computer; it was quiet before and after. The load average was
  about 9 on 6 cores.

So it drives. The open worry is the computer's load, not the odometry:
the odom transform now passes through two Python programs (the relay,
then `odom_to_tf.py`), and the relay itself takes about a third of a
core.

## 12. Nav2's margins

The user watched the 1.5 m goal: it went forward but curved, ended angled
to the right, and looked nervous on an easy path.

- The odometry itself recorded the turn (heading -179° to 142°, 39° to
  the right), so the robot turned on purpose; it was not the fused
  odometry misreading the heading.
- The costmap showed why: a solid obstacle from 0.5 m to the left of the
  straight line for the whole first metre (a table or its chairs), free
  floor on the right. The space straight ahead was clear, as the user said.
- Nav2 keeps a 0.45 m zone of raised cost around obstacles. With
  `cost_scaling_factor` at 3.0 that cost hardly fades inside the zone
  (54 or more out of 99, then zero), so the middle of an aisle looks
  almost as costly as the edge. Nav2 also accepts any final heading, so
  it stayed angled.

Changed, with the user's go-ahead: `cost_scaling_factor` 3.0 to 8.0 in
both costmaps (vendor config on the robot, source and installed copy,
backups `*.pre-csf8`; same change in `env/nav2-mapless-lite3.patch`).
The 0.45 m radius stays: it was raised from 0.30 because the planner
failed next to box corners. Nav2 restarted; both costmaps report 8.0.

### Driving with the new value

The user picked the robot up and put it in place (it detects the lift and
tucks its legs; it is made for that).

1. First goal: **aborted, the robot did not move.** The planner could not
   make a path: the robot's own cell and the cells just ahead of it were
   marked as obstacles, left from the carry. They had not faded after two
   minutes. The relay stayed on "corrected by FAST-LIO2" through the
   carry.
2. Nav2 restarted for a fresh costmap: own cell free, cost along the line
   0 to 29.
3. Second goal, 1.5 m ahead: reached. 1.28 m forward, 0.14 m to the left,
   **heading 40° to the right again.**

So the cost scaling change did not stop the turning: same goal, same 40°
as before it. The cause is still unknown. Next step is to record what the
controller commands (`/cmd_vel`) next to the pose during one drive, to
see whether Nav2 asks for the turn or the robot yaws on its own. Not done:
the battery was at 20%.

Also seen in this drive: 87 camera frames dropped for lack of a transform,
with the load average between 8.5 and 11.

## 13. After the battery change: FAST-LIO2 starting by itself

The battery went to 20% and was changed. After the reboot, as expected:
the relay service was up by itself, FAST-LIO2 was not running, and
`/odom_fused` was plain leg odometry.

The user offered to let me change code on the Orin so FAST-LIO2 would not
need starting by hand. It turned out not to need any change there:

- FAST-LIO2 must start with the robot standing still. The Orin does not
  know the robot's posture; the perception computer does.
- So the relay does it: every 5 s, if the robot is standing, has been
  still for a second and no FAST-LIO2 pose is arriving, it calls the
  Orin panel's start (the API from section 10), then waits 30 s before
  asking again.
- It never stops a run and does not restart a diverged one.

Self-check added for the rule (standing and still: yes; walking, lying,
already running: no).

Run on the robot (commit `40d3897`, robot standing still, FAST-LIO2 off):
the service was restarted at 23:27:37, asked the Orin to start at
23:27:50 (answer `ok`), and reported "/odom_fused is corrected by
FAST-LIO2" at 23:27:52. `/odom_fused` at 50.0 Hz.

## 14. Why Nav2 turns near the goal

One more 1.5 m goal, with a recorder on `/cmd_vel` (what Nav2's controller
commands), the fused pose and the leg odometry, 20 times a second. This
one was in an open spot: zero cost along the whole line.

Result: 1.24 m forward, 0.03 m to the right, heading -5°. Nearly straight
this time. The recording shows what happens:

| Phase | Controller commands | Heading |
|---|---|---|
| First 2.5 s of driving | 0.30 to 0.36 m/s forward, turn rate about 0 | within 1.3° |
| From 0.65 m before the goal | slows to 0.09 to 0.19 m/s, turn rate jumps between -0.31 and +0.31 rad/s | swings: -6°, -10°, -1°, +3°, -3° |

- The turn command changed sign 6 times in the last 3.5 seconds.
- Adding up the commanded turn gives -5.4°; the measured change was -4.9°
  (fused) and -5.1° (legs). So the robot does what it is told. It is
  Nav2's controller asking for the turns, not the odometry and not the
  robot yawing by itself.
- In the two earlier runs the same wiggle happened to hold one direction:
  0.31 rad/s for a bit over two seconds is the 40°.

So the "nervous" look and the angled finish are one thing: the controller
(DWB) has no steady preference for a heading once it is close to the goal.
In the config its scoring list is
`PreferForward, GoalDist, PathDist, RotateToGoal, BaseObstacle`. The two
entries that reward pointing along the path (`PathAlign`, `GoalAlign`)
have their weights set in the file but are not in the list, and the entry
made to stop left-right flipping (`Oscillation`) is absent. It also
simulates 3 s ahead (`sim_time`), which at 0.65 m from the goal means
crawling. This list is the vendor's; our patch never touched it.

### First try: adding PathAlign and GoalAlign. No effect.

With the user's go-ahead: the two alignment entries added to the list
(vendor config on the robot, backups `*.pre-align`; same in the repo
patch), Nav2 restarted, and the controller confirmed to have loaded the
new list. The robot backed up 1.23 m by itself (`walk(-1.2)`, rear sonar
3.93 m) and got the same 1.5 m goal with the recorder on.

| | Before | With PathAlign + GoalAlign |
|---|---|---|
| Turn-command sign flips | 6 | 7 |
| Samples with a hard turn command | 66 of 113 | 59 of 110 |
| Heading swing during the drive | -12° to +5° | -15° to +1° |
| Final heading | -5° | -12° |

Same behaviour: straight until about 0.65 m from the goal, then the
forward speed drops and the turn rate jumps between -0.31 and +0.31.

### What the numbers point to instead: PreferForward

The hard turning starts at the exact moment the commanded forward speed
drops below 0.30 m/s (0.30 to 0.25), and the turn rate is always the
first sample above 0.30 rad/s (0.3077). The config has
`PreferForward.strafe_x: 0.3` and `PreferForward.strafe_theta: 0.3`. As
I understand that scoring rule, any candidate that moves forward slower
than `strafe_x` while turning slower than `strafe_theta` gets the full
penalty, as if it were strafing. So as soon as the controller has to
slow down for the goal, going straight is penalised and turning at just
over 0.3 rad/s is the cheapest thing left, in either direction. That
reading is from memory of how the rule works, not from its source on the
robot; the two thresholds matching the recording is the evidence.

### Second try: `PreferForward.strafe_x` 0.3 to 0.05. Fixed.

With the user's go-ahead: PathAlign and GoalAlign taken out again (they
changed nothing), `PreferForward.strafe_x` lowered to 0.05 (vendor config
on the robot, both copies; same in the repo patch, which reverses cleanly
against the robot's files). Nav2 restarted and the controller reported
0.05.

The user asked for the robot to be turned before backing up, so it was
turned back to its starting heading first (+12°), then backed. The rear
sonar read 1.29 m, so by the script's rule (keep 0.6 m) it backed only
0.72 m, not 1.2 m. Same 1.5 m goal, recorder on.

| | Original | + PathAlign, GoalAlign | `strafe_x` 0.05 |
|---|---|---|---|
| Turn-command sign flips | 6 | 7 | 0 |
| Samples with a hard turn command | 66 of 113 | 59 of 110 | 0 of 99 |
| Largest turn command (rad/s) | 0.31 | 0.35 | 0.06 |
| Heading swing during the drive | -12° to +5° | -15° to +1° | 0° to +3° |
| Final heading | -5° | -12° | +2° |
| Drive time | 5.9 s | 5.7 s | 5.2 s |

It now slows down in a straight line (0.30, 0.19, 0.14, 0.09 m/s) and
stops 0.25 m from the goal, which is the tolerance. One run in an open
spot; the aisle between the tables, where it ended 40° off, has not been
repeated.

A slip of mine on the way: the first attempt at this run failed before
the robot moved, with "2 nav2 process groups are running". My ssh command
line contained the name of one of Nav2's programs (for a parameter
check), and `nav_start()` counts any process whose command line contains
such a name as a Nav2 stack. One real stack was running; I reran without
the word.

### The aisle between the tables

The user carried the robot to the start of the aisle. Nav2 restarted for
a clean costmap (the carry left marks last time). The costmap before the
goal: obstacles from 0.5 m to the left and to the right of the line from
0.75 m on, a free lane about half a metre wide down the middle, cost 0
to 13 on the line itself.

Same 1.5 m goal: reached, 1.24 m forward, 0.07 m to the right, heading
-1°. Before today's two config changes the same aisle gave 39° and 40°.
In the recording: no turn-command sign flips, largest turn command
0.10 rad/s, heading within 2° of the start all the way, at most 0.10 m
off the line sideways.

Also seen in the relay's log around the carries and restarts: it dropped
to "leg odometry alone" five times and was back on FAST-LIO2 within a
second each time.

### Goal tolerance 0.25 m back to 0.15 m

Every run stopped 0.25 to 0.27 m short of the goal, because Nav2 counted
the goal as reached within 0.25 m (our patch had raised it from the
vendor's 0.15). The user asked whether a smaller margin would hurt longer
routes. It does not affect the route, only the last stretch: a few more
seconds at crawling speed, and a risk of a legged robot stepping around a
goal it cannot settle into. With the wiggle gone, 0.15 was worth trying.

Changed with the user's go-ahead in both places it is set (goal checker
and controller), on the robot (backups `*.pre-tol15`) and in the repo
patch. Nav2 restarted; same 1.5 m goal from where the robot stood.

| | Tolerance 0.25 | Tolerance 0.15 |
|---|---|---|
| Moved forward | 1.24 m | 1.36 m |
| Stopped this far from the goal | 0.27 m | 0.15 m |
| Drive time | 5.7 s | 6.7 s |
| Final heading | -1° | +1° |
| Backward commands | none | none |

No stepping around the goal: it crept the last 15 cm at 0.09 m/s and
stopped. One run.

Three places in the code were tuned around the old 0.25 (`tour.py`
`MIN_GOAL`, `nav.goal_ahead`, `person.NAV_NEAR`). Their values still
hold as they are (they are lower limits, and stay on the safe side); only
their comments were updated. One behaviour does shift: when following a
person with Nav2, the robot now ends about 0.1 m closer to them. Not
re-tested.

## 15. Lidar into Nav2's costmap, step 1 (started)

The plan agreed for "navigate a whole building", in three steps: (1) the
lidar as an obstacle source in today's mapless Nav2, (2) a building map
and localization in it, (3) named places as goals. Step 1 first; the
user allowed a new service on the Orin for it.

Done:

- **On the Orin:** `pointcloud_to_laserscan` was already installed and
  `~/bin/lidar_nav.launch.py` already existed. Run by hand it gave a
  720-beam scan at 10 Hz in `base_link`. Installed as
  `lidar-scan.service`, enabled at boot (copy in `env/lidar-scan.service`).
- **The relay** now also carries `/scan` across and stamps it with the
  perception computer's clock. Tested on a separate ROS domain so the
  running odometry service was not disturbed: 9.6 Hz, 720 beams.

A false alarm worth recording: the scan showed returns 0.64 m straight
ahead while the sonar read 2 m clear. Looking at the raw cloud, they were
a level surface 0.21 m below the lidar. That was the floor: the robot had
been laid down in the meantime (state 8), and the scan's height band
assumes it is standing. It also says the lidar sits about 0.21 m above
the floor lying down, which fits the Orin's own figure of 0.46 m
standing.

The user confirmed it: the lidar is angled down, the robot was lying,
it was seeing the floor. Measured in that state: with the mount
transform the floor comes out as a level plane (0.7° and 0.6° off, fore
and sideways), 0.10 m below `base_link`. So the 20.4° pitch is right to
within a degree. Standing lifts the body about 0.24 m, which puts the
floor about 0.19 m below the scan's lower edge.

Because of this the relay now only passes scans on while the robot is
standing, so a lying robot does not fill the costmap with floor.

### With the robot standing (commit `ece671e` pulled, relay restarted)

- The relay restarted and was back on FAST-LIO2 in 9 s.
- `/scan` on the robot's side at 10 Hz. Straight ahead it read 2.27 to
  2.31 m; the depth camera said 2.29 m and the front sonar 2.09 m. So no
  floor in it while standing. To the sides and diagonally behind it saw
  things 0.71 to 0.87 m away that neither camera nor costmap knew about.
- Behind, only a handful of returns: the lidar is tilted nose-down, so
  backwards it looks upward and only catches tall things.

### In the costmaps

Added a layer on `/scan` to both costmaps (marking and clearing, used out
to 3 m), on the robot (backups `*.pre-scan`) and in the repo patch. The
patch was regenerated on the robot this time: our patch undone on a copy
to get the vendor original back, then a fresh diff against the edited
file. It reverses cleanly.

First start: Nav2 came up, both costmaps subscribed to the scan, no
errors, and the layer marked nothing. Cause: this layer's per-source
height limit (`max_obstacle_height`) defaults to 0.0, so every point was
discarded silently. I had it in my first draft of the block and dropped
it. Set to 2.0.

Second start: the costmap shows the room all around. The tables to the
left from 0.75 m, something to the right behind, the wall 2 m ahead, an
obstacle 1 m behind. Check: of 403 scan returns within 2.5 m, 402 sat on
an obstacle cell.

The user confirmed the picture: a table on the left, the wall ahead, a
Husky robot under a table on the right behind, a chair behind on the left.

### First goal with the lidar layer, and a bug in the fused odometry

A 1.0 m goal with the recorder on: straight, no turn-command flips,
heading within 1°. Nav2 reported the goal reached.

But the recording shows the fused odometry was wrong on this drive:

| Source | Says the robot moved |
|---|---|
| Fused odometry | 0.86 m |
| Leg odometry | 0.62 m |
| Lidar: how much closer the wall got | 0.63 m |

The fused pose jumped forward 0.24 m within 0.4 s in the middle of the
drive, while the legs moved 0.08 m. So the robot stopped about a quarter
of a metre short of where it believed it was. The relay's log also showed
it flipping between "leg odometry alone" and "corrected by FAST-LIO2"
every few seconds since its last restart.

Measured, robot standing still, 25 s:

- FAST-LIO2's poses reach the perception computer at 7.9 a second, not 10.
- One in twenty arrives more than 0.35 s late; the worst was 0.78 s.
- By their own stamps the poses are up to 0.5 s apart, so some never
  arrive. FAST-LIO2's own log shows it getting 8.3 to 10 scans a second.
- The scan itself arrives steadily at 10 a second.

The mistake is in my fusion, and it was there from the start; the loops
earlier in the day just did not hit it hard. It paired each pose with
where the legs were when the pose ARRIVED, assuming a fixed 0.05 s delay.
A pose arriving half a second late was therefore compared with legs that
had walked on, and the 0.5 s gap limit treated every missing pose or two
as a new run.

Changed in `robot/lio_relay.py` (not deployed yet):

- Each pose is matched by its own time stamp to where the legs were then.
  The offset between the two computers' clocks is taken from the
  quickest pose seen so far.
- Leg poses are remembered for 2 s instead of 1 s.
- A gap has to be 1.0 s before it counts as a new run.

Self-check added.

### Side-by-side test on the robot

The new code was run beside the service, publishing under test names, and
the robot backed 0.6 m and walked 0.6 m forward (0.3 m/s) while old
fused, new fused, legs and the lidar's distance to the wall were recorded.

| | Old fused | New fused | Legs | Wall (lidar) |
|---|---|---|---|---|
| Backward leg | 0.65 m | 0.65 m | 0.67 m | 0.67 m |
| Forward leg | 0.67 m | 0.67 m | 0.71 m | 0.69 m |
| Net, start to end | +0.02 m | +0.02 m | +0.05 m | +0.02 m |
| Largest change in 0.2 s | 0.11 m | 0.11 m | 0.11 m | |

What it shows and does not show:

- The new code is right on this walk: it ends where the wall says, and
  never moves faster than the legs.
- The old code was also right on this walk. Its jump did not come back,
  so this test does not prove the fix; it only shows no harm.
- Where the two did differ: standing for 90 s, the old relay dropped to
  leg odometry 15 times, the new one once.

Two mistakes of mine in running it:

- The test script failed twice before the robot moved, because Python
  found the test copy of the `robot` package before the real one.
- My clean-up command after the first failure matched the real service's
  process as well and killed it. systemd restarted it within seconds
  (`NRestarts=1`), nothing was moving, but the odometry was away for
  about ten seconds and the fused pose lost the correction it had built
  up, so the odom frame shifted under Nav2's costmap.

Note for later: the Orin panel's Navigate tab starts the same launch
file itself and kills any copy it finds first. Using that tab would
fight with the new service.

### Deployed (commit `9c0814a`)

Relay restarted with the new matching, Nav2 restarted, the robot backed
0.37 m (rear sonar 0.96 m, so the script allowed no more) and got a
1.0 m goal with the recorder on.

| Source | Says the robot moved |
|---|---|
| Fused odometry | 0.83 m |
| Leg odometry | 0.89 m |
| Lidar: how much closer the wall got | 0.81 m |

No jump: the fused pose never changed more than 0.11 m in 0.2 s (legs
0.12 m). No turn-command flips, heading +2°. The relay dropped to leg
odometry once in the three minutes after the restart (while standing,
before the drive), against 15 times in 90 s before.

One drive. The same goal before the fix read fused 0.86, legs 0.62,
wall 0.63.

## 16. Spec for the rest

Steps 2 and 3 were specified with the spec command, question by question.
What the user decided:

- Mainly "send it to a place"; talking while it walks would be nice.
- From code, from the panel and by voice, all three.
- Anyone with access to the panel may use it.
- Waypoints as pins on the map, and also as physical tags in the
  building ("A for convenience, B for fun"); tags as a later phase.
- One floor at a time. Stairs and lifts are for later.
- Localization on the perception computer, with the Orin as the fallback
  if the load does not fit.
- The building (UiA): some glass, all doors opaque and sometimes closed,
  long featureless corridors.
- It is a project of a month or more.

Saved as `.planning/specs/building-navigation.md`, five phases: map and
localization, places from code, the panel, voice, tags.

## 17. Building navigation: plan and first measurements (2026-10-09)

Plan saved as `.planning/plans/building-navigation.md` (25 tasks). What
the robot said before anything was built:

- `nav2_amcl` and `nav2_map_server` are installed. The vendor's map
  config (`dr_nav2`) has no AMCL in it, and the robot's
  `nav2_bringup/localization_launch.py` starts only the map server: the
  vendor localizes with `hdl_localization`. AMCL is ours to launch.
- The Orin's `~/robot_pc/nav2_params.yaml` has an AMCL block written for
  this robot; it is the starting point.

### Lidar position on the body (plan task 2)

From four photos, the torso mesh (`urdf/meshes/torso.STL`: nose at
x 0.279 m, top of the shell at z 0.06 m where the mount sits) and one
measurement by hand (7.5 cm from the top of the shell to the middle of
the grey ring under the dome):

| | was (guess) | now | from |
|---|---|---|---|
| x forward | 0.20 | 0.20 | ruler in the photo: dome centre 8-9 cm behind the nose, so 0.19 +-0.02 |
| y left | 0 | 0 | photo from above: dome in the middle of the mount |
| z up | 0.10 | 0.14 | 0.06 + 0.075 = 0.135 to the ring |

Set on the Orin as `lidar_z:=0.14` in `lidar-scan.service` (backup
`lidar-scan.service.pre-z14` beside it; our copy in `env/`). After the
restart: `base_link -> livox_frame` reads (0.200, 0.000, 0.140) and
`/scan` is back at 10.0 Hz.

### Load baseline, mapless Nav2 (plan task 1)

One process, one `Lite3`; no panel running; battery 69%. 60 s standing
with Nav2 up, then one 3.0 m goal (arrived in 13 s), 20 s window.

| | standing, no goal | during the goal |
|---|---|---|
| idle, of 6 cores | 41.9% | 39.3% |
| load average | 5.9 | 5.8 to 7.0 |
| camera driver | 57% of a core | 54% |
| `lio_relay` + its listening child | 38% + 27% | 39% + 29% |
| controller server | 20% | 35% |
| planner server | 18% | 18% |
| `odom_to_tf.py` | 21% | 21% |
| `jetson2motion` | 21% | 22% |
| `sonar_range` | 18% | 21% |
| `sensor_checker` | 17% | 17% |

Camera messages dropped ("dropping message" in `/tmp/nav2.log`): 10 in
the whole run, all between 10 and 20 s after Nav2 started. None in the
60 s standing, none during the goal. Control loop misses: 1. So the load
gate's bar is: no dropped camera messages once Nav2 has been up 20 s.

The relay is now the second largest consumer, at 65% of a core for its
two processes (it was about a third before it carried the scan).

### What `/tf` costs to listen to (plan task 4)

With Nav2 up `/tf` carries one transform, `odom -> base_link`, at 50 Hz.
A Python node that only counts those messages: 17.3% of a core. A Python
tf2 listener: 18.5%. (With Nav2 down the topic is silent and the same
listener costs 2.8%.) So the node that reports "localized" will not
listen to `/tf`; it reads odometry at a low fixed rate instead.

### The relay made cheaper (same day)

The baseline showed the relay as the second largest consumer. Two changes:

1. The crossing from the Orin is UDP, not a child process on the Orin's
   DDS printing text. `robot/lio_relay.py send` runs on the Orin and sends
   each pose (73 bytes) and each scan (2913 bytes, the ranges as the
   float32s they are) to port 8042 on the perception computer.
2. The relay publishes `odom -> base_link` itself; the launch no longer
   starts the vendor's `odom_to_tf.py`.

Same measurement as the baseline, robot standing, Nav2 up, no panel:

| | before | after |
|---|---|---|
| idle, of 6 cores, standing | 41.9% | 53.1% |
| idle during a goal | 39.3% | 52.9% |
| relay | 38% + 27% (child) | 33% |
| `odom_to_tf.py` | 21% | gone |
| controller / planner server | 20% / 18% | 15% / 14% |
| camera messages dropped | 10, all at start-up | 0 |

A 0.5 m goal arrived (4 s). `/tf` still 50.1 Hz, one transform.

### FAST-LIO2 was 35 minutes behind

Found while testing: the Live run that had been going since the Orin
booted was sending poses stamped 2075 s in the past, 88 a second, with
`fastlio_mapping` holding 5.1 GB and the Orin down to 117 MB free. The
relay, restarted during such a run, took the lag for a clock difference
(it compared its clock with the pose stamps), used the poses for about
3 s and then fell back to leg odometry: that is the "corrected" then
"leg odometry alone" pair in its journal.

- Stopped that run (`lio_relay stop`; saved on the Orin as
  `live_20261009_173432`). Memory back to 5.2 GB free.
- Each pose now carries the Orin's send time; the clocks are compared by
  that, so an old pose is seen as old and not used. The relay logs
  "FAST-LIO2 is N s behind".
- A fresh run is 0.04 s behind. Standing 60 s, turning +90 and -90, then
  20 s more: corrected the whole 110 s, not one fall back to the legs;
  the correction moved by about 4 cm over the two turns.
- Not known: why that run fell behind, or when. Yesterday's "poses at
  about 8 a second, up to 0.78 s late" may have been the start of it.

### Map mode, the laptop pieces (plan tasks 5, 7, 11)

Three files, each with its own self-check (`... check`), all passing on
the laptop and on the robot's Python 3.8:

- `env/nav2_map_params.py`: map mode's Nav2 parameters, generated from
  the mapless file. On the robot's installed file it changes nine paths
  and nothing else.
- `robot/places.py`: the places file per floor.
- `robot/locate.py`: "localized or not" from AMCL's spread and from how
  much of the scan lands on the map's walls. The node has not run yet:
  there is no map and no AMCL to run it against. One scan fit takes
  0.54 ms on the Jetson.

What the paper room in the check says about the scan fit, before any
real floor: a pose 0.2 m or 3 degrees off still passes the 0.7 limit; a
different room that shares a corner with the map scores 0.62; sliding
0.5 m along one pair of walls scores 0.58, because the other pair still
fits. The limit is a guess until it is tuned on the floor.

### First floor map, and the scan does not see what the map shows

The panel's "Record & map" gave a map that flew off (`lab_20261009_181337`,
9.5 km of path for a walk round one room). The bag is missing 22% of
`/livox/imu` (22,691 of about 29,150 samples, gaps up to 1.1 s); the
lidar frames are all there. The Live run of the same walk had no gaps.
Why the recorder drops them was not found.

The Live run was saved instead (`lio_relay.live('stop', name='lab-live',
one_floor=True)`): 29.5 m, ended 2 cm from its start, re-levelled by
1.1 degrees. Copied with `env/get_map.sh` to `~/lite3_maps/lab/`. One
room, about 6 x 10 m, 0.1 m cells. The map's frame is the lidar's pose at
the start, so `base_link` on the start mark is at about (-0.20, 0).

`demos/check_map.py lab -0.20 0 0`, robot standing near the mark: 0.40 of
the scan on mapped walls, 0.52 at the best pose within 0.5 m and 15
degrees. `locate.py` asks for 0.70. The map tool keeps only what runs
from floor to ceiling, so tables and chairs are not on it; the scan is
cut from 0.15 m below to 1.0 m above `base_link` and is mostly furniture.

One second of raw lidar points, cut into other height bands and scored
the same way (single 0.1 s frame; band given above `base_link`, which
stands 0.32 m over the floor):

| band | beams with a return | fit at (-0.20, 0, 0) | best fit nearby |
|---|---|---|---|
| -0.15 .. 1.0 m (today's `/scan`) | 569 | 0.45 | 0.55 |
| 0.7 .. 1.5 m | 487 | 0.66 | 0.79 |
| 0.9 .. 1.7 m | 382 | 0.87 | 0.99 at (-0.20, 0.00, 4 deg) |
| 1.1 .. 1.9 m | 453 | 0.62 | 0.70 |
| 1.3 .. 2.1 m | 470 | 0.64 | 0.71 |
| 0.9 .. 2.3 m | 512 | 0.65 | 0.74 |

So a scan cut 0.9 to 1.7 m above `base_link` (1.2 to 2.0 m over the
floor) sees the walls and little else, and its best pose is the start
mark. Above 1.7 m something else comes in (a fifth of all points are the
ceiling, 2.4 m above `base_link`). One room, one robot position: the band
may want moving elsewhere.

### `/scan_walls` built (same day)

The Orin's `lidar_nav.launch.py` got a second `pointcloud_to_laserscan`
node, 0.9 to 1.7 m above `base_link`, on `/scan_walls` (backup there
`lidar_nav.launch.py.pre-walls2`; our copy is now in `env/`). The sender
and the relay carry it as datagram kind `W`; an older relay ignores it.

Live, robot standing by the start mark, `demos/check_map.py lab -0.20 0 4`:

| scan | beams with a return | fit at that pose | best nearby |
|---|---|---|---|
| `/scan_walls` | 379 | 1.00 | 1.00 |
| `/scan` | 569 | 0.37 | 0.52 |

The fit is at its ceiling over a patch of poses (0.15 m and a few degrees
apart score the same), so it tells "about right" from "wrong", not more.
Relay with both scans passing: 30% of a core. Both scans 10.0 Hz on the
Orin; its two scan nodes cost 8% of a core each.

### Map mode runs (plan task 6), started by hand

`env/start_nav2_map.sh lab`: the generated parameters, the sonar node,
`robot/locate.py`, then `env/nav2_map.launch.py` (the vendor's mapless
launch + map server + AMCL + a lifecycle manager for those two). Robot
standing near the start mark; the pose to give AMCL was found with
`demos/check_map.py` first.

What it took, in the order it went wrong:

1. **Nav2 does not finish starting until AMCL has a pose.** The global
   costmap is in `map` now and waits for `map -> base_link`, which only
   exists once AMCL has been told where it is. Map server and AMCL are
   active after 3 s; the planner and the rest 0.5 s after the pose is
   given. So loading a floor and giving the pose are two steps, and
   "Nav2 is up" cannot be waited for in between.
2. **The costmap never got the map.** It logged "Subscribing to the map
   topic (/map) with volatile durability", subscribed 1.4 s after the map
   server had sent its one copy, and stayed an empty 30 m square with its
   corner at the origin ("Robot is out of bounds of the costmap!", and
   the planner server at 109% of a core). Cause: `nav2_bringup`'s
   `navigation_launch.py` rewrites `map_subscribe_transient_local` in
   the parameter file it is given, to `false` by default. Fixed by
   passing it as a launch argument where the mapless launch is included.
   Then: "transient local durability", "Resizing costmap to 122 X 133 at
   0.100000 m/pix", planner server 15%.
3. **Grey (never seen) cells were free.** `track_unknown_space: true`
   changed nothing in the published costmap (0 unknown cells): the other
   layers write free over it. `unknown_cost_value: 50` under the static
   layer changed nothing either: this Nav2 reads it from the costmap
   node. Set there, the map's real unknown (-1) is no longer "the
   unknown value" and becomes a wall: 11,952 lethal cells against 2,448
   before, of 16,226 (the picture has 791 black, 9,846 grey, 5,589
   white; the rest of the difference is inflation, 0.45 m).

Result, standing:

| | |
|---|---|
| AMCL after the pose | (-0.21, -0.13), variance 0.16 / 0.21 m2, 0.06 rad2 (it had been given 0.25 / 0.25 / 0.07) |
| after turning +45 and -45 degrees | variance 0.023 / 0.16 m2, 0.021 rad2 |
| `/localized` | localized, fit 0.97 to 1.00, spread 0.40 to 0.49 m (limit 0.5) |
| AMCL / map server / `locate.py` | 8% / 5% / 9% of a core |
| idle, of 6 cores | 40.9% (mapless, same day: 53.1%) |
| camera messages dropped, control loop misses | 0, 0 in 30 s |

The camera driver read 80 to 92% of a core in the three map-mode runs
against 57 to 70% in the mapless ones; not explained. No goal has been
sent in map mode yet: `goal_send()` still speaks `odom`.

The spread sits just under its limit right after a pose is given,
because the pose was given with 0.5 m of doubt. It narrows with
movement, slowly sideways (only turned on the spot so far).

### The `Lite3` calls for map mode (plan tasks 8, 12, 13, 14)

`floors`, `load_floor`, `set_pose`, `localized`, `map_pose`, `go_to`,
`go_to_point`, `places`, `save_place`, `rename_place`, `delete_place`,
and the command-line words for them. Written against checks added to
`demos/check_motion.py` first (fake robot, pretend Nav2): goals go out in
the map frame, the refusals, lost on the way, cancel, the final heading,
places by name.

On the robot, standing still: `load_floor('lab')` 17 s, `set_pose` 2 s
to localized with Nav2 ready, a place saved where he stood, and "not
going: it has not been told where it is" before the pose was given.
No route walked yet.

### First routes on the map

`go_to_point(1.9, -0.3)` from the start mark, then `go_to('start')`,
twice. Where he believed he was at each end was checked against the
best-fitting pose from `demos/check_map.py`.

| | run 1 | run 2 |
|---|---|---|
| out, 2.1 m | 11 s, stopped 0.18 m from the point | 10 s, 0.17 m |
| his pose against the scan's best fit there | 0.07 m, 1 degree | 0.07 m, 1 degree |
| back | 57 s, 0.12 m from the place | 26 s (his own turn, then 9 s of Nav2), 0.11 m |
| his pose against the scan's best fit there | 0.15 m, 2 degrees | 0.11 m, 0 degrees |
| idle while walking, of 6 cores | 29% / 38% | 34% / 38% |

- **Run 1's way back took 57 s**: Nav2 stood for 20 s ("Failed to make
  progress" twice), found "No valid trajectories out of 5780", span as a
  recovery and then walked it. The controller's critics are PreferForward,
  GoalDist, PathDist, RotateToGoal and BaseObstacle; none rewards turning
  towards a path that starts behind him, and turning costs. Mapless goals
  are always ahead, so it never showed. Now `go_to_point()` asks the
  planner for the route first (`route_to()`, which also refuses a point
  there is no way to) and, if the route starts more than 60 degrees off
  his heading, turns to face it himself with `turn()`. Run 2 is with
  that. A sharp corner in the middle of a route is still Nav2's alone.
- **He did not turn to the saved heading at the end of run 2** (place
  2 degrees, ended at 81). `turn()` returned without finishing, probably
  its side check (front sonar 0.59 m, rear 0.46 m where he stood), and
  `go_to_point()` did not look at what it returned. It now prints a NOTE
  with the reason. Not reproduced.
- **Camera messages dropped**: about 130 while Nav2 starts (before the
  pose is given there is no `map` frame to put a cloud in), about 50 in
  the few seconds after the first goal starts, none after that, in both
  runs, the second leg included. A guess, not checked: AMCL's first real
  update, with its particles still at the maximum, takes longer than the
  0.5 s its transform is good for. The local costmap is in `odom` and
  does not depend on it.

## 18. Still open

As of the end of 2026-10-09. `SESSION_2026-10-09.md` has the summary of
that day and where to pick up.

Map mode:
- About 50 camera messages are dropped in the seconds after the first
  goal in map mode. Guess, not checked: AMCL's first update outlasts its
  0.5 s transform tolerance.
- One final turn to a place's saved heading did not happen; `turn()`'s
  side check, probably. Not reproduced.
- Nothing is tuned on the floor: `locate.py`'s limits, AMCL's noise, the
  height band of `/scan_walls` in other rooms.
- Through the door and a closed door are done (section 19). Not tried:
  carrying him mid-route, the wrong floor, a long corridor, glass.
- The load with the panel open, and why the camera driver reads 80 to
  92% of a core in map mode against 57 to 70% mapless.
- A sharp corner in the middle of a route is left to Nav2, whose
  controller does not turn towards a path by itself.
- Phases 3 to 5 of the plan (panel map view, voice, tags): not started.

The Orin:
- Why a long FAST-LIO2 run falls behind real time and fills the Orin's
  memory. The relay now ignores it; nothing restarts it.
- Why the panel's "Record & map" recorder drops IMU samples. The Live
  run is used for maps instead.

Older:
- The wiggle fix (section 14) has two runs behind it: one in the open,
  one in the aisle.
- Whether the floor leaks into the low scan while trotting.
- Marks left in the costmap by carrying the robot block planning until
  Nav2 is restarted.
- The relay assumes the robot stands level when the Live run starts.
- `person.follow_nav` stops about 0.1 m closer since the goal tolerance
  went back to 0.15 m; not re-tested.

## 19. Through the door, map `lab2` (2026-10-12)

The `lab` map was the room alone, and unmapped cells are walls, so there
was no route out. A new Live run (230 s, by handheld: round the room, out
of the door, along the corridor, back; it ended 0.04 m from where it
started) was saved on the Orin as `lab2_20261012_132548` and copied to
the robot as floor `lab2`: room, door, about 20 m of corridor, 0.1 m
cells. The start mark is new (tape), `here -0.2 0 0` as before.

- People walked in the corridor during the run. Two blobs just north of
  the door are left on the map and the planner finds no route north past
  them (`route_to(3.4, 2.4)`: none). South of the door is clean.
- Mark to the corridor, `go 3.4 0.5`: route 6.4 m through the door at
  about (2.3, 2.3), arrived in about 90 s, own estimate (3.43, 0.62),
  0.12 m from the target.
- Back, `go start`: 70 s, own estimate 0.14 m from the saved place, the
  final turn to the saved heading done. By eye he stood about 0.15 m to
  the right of the tape.
- Door closed, `go 3.4 0.5` from the mark: he set off, walked 0.8 m,
  the planner then found no path; Nav2 waited and asked again three
  times and gave up. `go` returned "aborted - no path" 30 s after the
  start, with him standing still and still localized. No spinning.
- Battery 85% to 76% over the three runs and the standing between them.

- **A door seen shut stayed shut.** Back on the mark with the door open
  again, `go 3.4 0.5` answered "no route". The closed door had been
  marked on the global costmap from 0.8 m ahead of the mark (within the
  3.0 m `obstacle_range`); from the mark it is about 3.4 m away, at the
  edge of the 3.5 m `raytrace_range`, so nothing cleared it. Clearing
  the global costmap by hand brought the route back at once. Fix:
  `route_to()` now forgets the old marks once and asks again before it
  refuses (`_forget_marks()`, `CLEAR_SETTLE_S` 2.5 s). Checked on the
  laptop (`check_motion.py`). On the robot the new code clears in 2.7 s,
  finds routes afterwards, and a point that really is cut off costs one
  clear and is then refused. The stale door itself did not come back
  when the same steps were repeated: that time the planner answered
  with a 314-point route (about 16 m; the usual one is 124) before
  anything was cleared, and he then walked the usual way in 75 s.
  Someone was walking in front of him on purpose during that run (the
  user, afterwards), which would explain a route planned round them; it
  was not checked. The map itself has no way round the door.
- **Carried mid-route.** Lifted 8 s into `go 3.4 0.5`, after about 1.2 m,
  and set down about 1.2 m away 3 s later (basic state 22 while in the
  air, 6 again by himself). `/odom_fused` followed the carry. `go`
  raised "lost on the way, stopped: no usable lidar scan" and he stood
  still. The map estimate, (0.37, -1.00, -5 deg), then stayed "too
  wide": AMCL does not narrow while he stands, so he needs `here` again.
- **`here` did nothing after the carry.** `set_pose()` held the pose back
  when the old estimate lay within 1 m of the new one, and the wide one
  left by the carry did (he was put back on the mark). AMCL's spread was
  0.97 m and 0.82 rad and it never got an `/initialpose`. Now the pose is
  always sent once. With that he localized on the mark at once.
- Arrived twice more at (3.4, 0.5): the last time at (3.44, 0.35) facing
  180 deg, where 8 deg was asked for and the first arrival faced -22.
  That was the run with the person in front of him. Not looked into.
- **Lost in the doorway on the way back.** After standing some minutes
  in the corridor, `go start` stopped 35 s in with "lost on the way,
  stopped: its position estimate is too wide", at (2.62, 1.98, 111 deg),
  in the door. AMCL's spread there: 0.19 m in x, 0.51 m in y (the limit,
  `SPREAD_XY_M`, is 0.5), 0.17 rad. y is along the corridor, whose walls
  say nothing about it. The estimate itself was right as far as can be
  told: FAST-LIO2, whose run started on the mark, had him at
  (2.75, 1.92). Load average 9.3 on six cores, the panel open. This is
  task 10's corridor slide, met for the first time.
- **Getting him out of the doorway.** `here 2.62 1.97 111` (his own
  estimate back, with the `set_pose` fix now pulled) localized him. The
  next `go start` stopped after 16 s and 0.4 m with "what the lidar sees
  does not fit the map here"; a moment later he read localized again.
  Not explained; people were standing near him in the door.
- **Nav2 said "succeeded" 40 ms after taking a goal**, 3.3 m from it,
  on the `go start` that followed those two cancelled goals; `go` said
  "arrived". `go_to_point()` now sends the goal once more when
  "succeeded" comes further than `SHORT_M` (0.6 m) from the point, and
  raises if it happens twice. Checked on the laptop only.
- The session ended there: he was carried to the mark and laid down,
  Nav2 stopped with `estop`, battery 47%.

The distances are his own position estimate, apart from the one by eye.
Not done yet: three places, the wrong floor.

## 20. A long walk across campus (2026-10-12)

Recorded by the user and a friend from the Orin's panel, saved there as
`long-walk-recovered_20261012_150849`: 18 minutes (1086 s), 961 m, 7925
scans at 7.7 Hz (processing p50 up to 76 ms, max 103 ms), one IMU gap.
The robot was lifted and carried down a staircase on the way. They
started and ended at the lab.

- The panel's saved state: "DIVERGED - jumped 21 times; stop and
  restart". Steps between poses are 0.13 m at the median, up to 2.8 m.
- The path drops 8 m over 95 m of walking early on (the stairs), goes to
  -13.4 m, and ends at (-1.6, 9.9, -8.9): 10 m beside the start and
  8.9 m under it, standing within 1 m of that point for its last 208 s.
  So it does not close on the lab. The user's reading: it drifted.
- The floor slicer made four maps by height: floor0 182 x 140 m (36% of
  the poses on its free cells, around -12 m), floor1 193 x 210 m (53%,
  around -9 m), floor2 42 x 80 m (7%), floor3 103 x 40 m (11%, the
  level of the start). The start itself is free floor on none of them.
  Outdoors the floor is speckle, not solid free cells.
- Copied to the robot as floors `campus0` to `campus3`. Not loaded, not
  tried; floor1 is about 4 million cells.

For the next one: a run per level, stopped before he is lifted and
started again standing still; slower than 0.9 m/s.

## 21. The panel's map view on the robot, and map `lab3` (2026-10-12)

- The Map section of the panel (README 10, the feature list) ran on the
  robot the evening it was written: `load lab2` 16 s, "I am here"
  localized him in 2 s, and a press on the map sent `goto map 3.3, 1.2`.
  He walked from the mark to (1.5, 1.6) and Nav2 gave up with "aborted -
  no path" 30 s after the start, as at a closed door. Whether the door
  was shut was not confirmed. Battery 30%.
- `lab3_20261012_160546`, copied to the robot as floor `lab3`: 148 m,
  2897 poses, biggest step 0.12 m, 47 x 29 m, a 42 m corridor and two
  rooms. 215 m2 of floor connected at 0.2 m clearance, 97% of the walked
  path inside it. The cells within 1.3 m of the run's start are unseen.
- The run ended (-5.3, 0.2) from its start although the user walked back
  to it: drift on the way back (the user), all of it along the corridor's
  direction. What was drawn after the slip is then 5.3 m off from what
  was drawn before, so the lab end of the map may hold two copies of the
  same walls. Not checked against a live scan.
- `lab6_20261012_161859`, floor `lab6`: 73 m one way (start in the lab
  area, end in the hallway 20.7 m off), biggest step 0.11 m, 42 x 28 m.
  On its start spot the wall scan fits 0.99 at (-0.2, 0, 0), but a press
  on the panel's map had given (0.51, 1.00), 1.2 m off, and that does
  not fit: the map view needs zoom.
- **Localized, and no route anywhere.** The global costmap read 100
  under him: the slicer had left the cells round the run's start unseen
  (9 of 81 free within 0.5 m), and unseen is a wall. `get_map.sh` now
  makes the walked path floor (`env/walked_free.py`). 0.3 m either side
  was not enough: with the 0.45 m inflation only a one-cell lane was
  left and he stood at its end (cost 99). At 0.6 m the cost under him is
  29 and the planner finds routes from the start to the passage
  (16.3 m), both rooms (27.0 and 30.9 m) and the hallway end (32.8 m).
  None of them walked yet.

## 22. Load on all three computers, and a lidar flood (2026-10-14)

Read-only checks for `.planning/plans/compute-distribution.md` task 1, all
three computers reached through the robot's hotspot (`ssh -J
ysc@192.168.2.1`). Robot lying down, about 5 min after boot, no Nav2, no
panel.

### The lidar driver was publishing every network packet

| | flooding | after `systemctl restart livox-driver` |
|---|---|---|
| `/livox/lidar` | 2,070 msg/s, 96 points each | 10.0 Hz, 20,160 points |
| `/scan`, `/scan_walls` | ~2,080 msg/s each | 10.0 Hz |
| `lio_relay.py send` (Orin) | ~100% of a core | 2.6% |
| `livox_ros_driver2` (Orin) | 50% | 18-25% |
| two `pointcloud_to_laserscan` (Orin) | 31% each | 7% each |
| `lio_relay` (Jetson) | 41% | 30% |

Cause, as far as it is shown: the Orin booted with its clock 1.9 days
behind (its journal for this boot starts "Oct 12 16:38"). The driver
started at 24.8 s, and `systemd-timesyncd` stepped the clock forward
about 12 s later ("Initial synchronization ... 1.pool.ntp.org"). After
that step the driver's 10 Hz frame grouping broke and every packet went
out as its own message. A restart, with the clock already right, fixed
it. On 10-08 and 10-12 the same setup gave 10 Hz, so it depends on
whether NTP arrives before or after the driver starts. Without internet
(handheld mode) the clock is wrong for the whole run instead; whether that
floods too is not known.

Not fixed for good yet: the driver should start only after the clock is
synced, with a timeout so it still starts with no internet. That needs
the user's OK (Orin).

How to spot it: `ros2 topic hz /livox/lidar` on the Orin (domain 42)
shows ~2,000 instead of 10, or `lio_relay.py send` near a full core in
`top`.

### Motion computer (RK3588)

- Cores 1-4 are Cortex-A76 at 2.2 GHz; 0 and 5-7 are A55 at 1.8 GHz.
  Governor `performance`. 3.9 GB memory. Python 3.8.10.
- `jy_exe` runs real-time threads (SCHED_FIFO) pinned to all four A76
  cores: priority 98 on core 2, 95 and 45 on core 4, several on core 1,
  one on core 3. No `isolcpus`. `sched_rt_runtime_us` 950000.
- Lying down: 82% idle. `track` (person detector) 75% of a core at nice
  -19, the camera's GStreamer pipeline 37%, `mediamtx` 12%, `jy_exe` 12%.
- So the free capacity is the four A55 cores. Nav2 does not go here.

### Jetson

- `/camera/imu`: 1 publisher, 0 subscribers. VOA's launch and yaml files
  mention no IMU (only a fake one in `debug_launch.py`). The camera IMU
  can be switched off.
- A full GNOME desktop runs (gdm auto-login, `gnome-shell` 9-36%, Xorg,
  pulseaudio); default target `graphical.target`. Nothing of ours needs it.
- Lying down after the restart: camera driver 50%, `lio_relay` 30%,
  `sensor_checker` 18%, `jetson2motion` 14%; load average 1.6.

### Orin

- 6 x Cortex-A78AE at 1.7 GHz, 7.6 GB memory, Python 3.10.12.
- FAST-LIO2: `config/mid360.yaml` has `pcd_save_en: true` and
  `path_en: true`, but `~/bin/README.md` says `pcd_save_en` and `map_en`
  are off for runs, so the override lives elsewhere (not found yet). The
  guess that saving the map causes the 35 min lag (section 17) is
  therefore probably wrong; still unexplained.

### What recording a run costs (plan task 6)

`bin/run.py` through the hotspot, robot lying down (state 98), no Nav2,
no panel, 2 min each (`logs/2026-10-14_record-cost*`):

| | with `/tf` in the bag | without |
|---|---|---|
| recorder (`ros2 bag record`, the `ros2` process) | 13% of a core | 11% |
| `loadwatch.py` | about 2% on each computer | same |
| Jetson busy, mean over cores | 29.8% | 31.3% |

`/tf` left out: 50 Hz of mostly `odom -> base_link`, which `/odom_fused`
already is. Lying down without Nav2 the Jetson's biggest are the camera
driver 47%, `robot.lio_relay` 30%, `sensor_checker` 18%, `jetson2motion`
15%. The motion computer is 18% busy (`track` 76%, camera 37%); the Orin
9-12% (lidar driver 24%, the two scan nodes 14% together).

## 23. First baseline runs, and why routes close (2026-10-14)

The standard run of `.planning/plans/compute-distribution.md` (task 7),
recorded with `bin/run.py` and read with `bin/report.py`. Runs in
`logs/2026-10-14_baseline` (floor `lab2`) and `logs/2026-10-14_baseline-lab6`.
User stood him up and said "go" to each walk; the panel was open.

### `lab2`: out through the door and back

| | |
|---|---|
| walk 1, `go 3.4 0.5` | arrived 0.15 m from the goal (the tolerance), 70 s; ended facing 160 deg (no heading asked) |
| walk 2, `go start` | stopped before moving: the turn to face the route took AMCL's spread past 0.5 m in the corridor ("lost on the way"). Fit there 0.96, so the pose was right, only the spread had grown. `here` at the same pose, then it arrived |
| arrival at the mark | tape: 15.5 cm left, 13.5 cm forward = **0.21 m**; he believed about 11 cm |
| walk 3 | refused: "point (3.40, 0.50) is in a wall" with the corridor empty. `go x y` checks the goal cell before it starts and, unlike `route_to`, does not clear old marks and look again |
| then | Nav2 killed at 20:58:02 by the panel's E-STOP, pressed by accident (the panel's terminal says so) |
| walk 3 again | walked 3 m, then "no path" from just inside the door, 9 planner failures; clearing the global costmap did not help |

Start pose: the lidar's best fit put him 15 cm behind the mark and 6 deg
turned ((-0.35, -0.05, 6)); `here -0.2 0 0` pushed AMCL off (fit 0.67).
Set the start from `demos/check_map.py`'s best fit, not from the mark's
nominal coordinates. Later at the mark the tape said 13.5 cm forward where
the lidar said 3 cm (sideways both about 16 cm): the saved mark or the body
point measured from is 10 cm off. Measure from one agreed body point.

### `lab6`: to the hallway (-9.86, 8.30)

The mark is at (0, 0, 5 deg) on `lab6` (fit 0.99), not (-0.2, 0, 0); saved
as place `start`. One walk: aborted "no path" 4 m out, at (0.30, -2.85).

The room has two doors to the corridor. The left one was open, the right
one closed (both are open on the map). The planner first took the left
door (24 m), lost that route within 4 s, then took the right door (32 m)
and lost it when the low lidar saw the closed door at (2.3, -4.5).

### Why the left door closed while it was open

Drawn from the recorded global costmap, plan and low `/scan`:

- The map's free floor is little more than the strip he walked when it was
  recorded (`walked_free.py`, 0.6 m either side); unseen is a wall. The
  left doorway is a channel about 0.3 m wide between lethal cells, much
  narrower than the door.
- The low lidar marks the real wall beside it (x 2.55 to 2.7), and every
  mark makes his half-width (0.25 m with padding) lethal. With AMCL 10 to
  15 cm off (as measured at the mark), the marks land on the channel. Route
  cells turned lethal at 162 s and 173 s with the door open.
- In map mode the GLOBAL costmap also holds the depth camera (STVL, 15 s
  memory) and the sonars: static, stvl, range, scan, inflation. The cells
  at (1.71, 0.66) turned lethal with no low-lidar return within 0.45 m, so
  probably the depth camera. Not proven: depth is not recorded.

The same mechanism fits `lab2`'s walk 3. Not phantom marks: a map that
leaves no room, plus real marks and a small localization error.

### Load and other numbers

| | `lab2` run (29 min) | `lab6` run (5 min) |
|---|---|---|
| Jetson busy, mean / max | 67% / 98% | 78% / 97% |
| Jetson load average, max | 16.6 | 15.2 |
| camera messages dropped | 2,671 | 961 |
| camera driver | 63% of a core | 74% |
| `robot.lio_relay` / `robot.hmi` | 36% / 33% | 37% / 37% |
| AMCL spread max | 0.52 m | 2.06 m (during the handheld walk back) |
| Orin: `lio_live.py` / FAST-LIO2 | 63% / 44% | 72% / 49% |

- The Orin panel's `lio_live.py` uses more CPU than FAST-LIO2 itself.
- Leg odometry missed the handheld walk back on `lab6`: at the end
  `/odom_fused` and `/lio_odom` were 0.03 m from the start (true: he was
  back on the mark), `/leg_odom2` 3.09 m. Not looked into.
- The bag's first version had no low `/scan` (added for `lab6`) and
  `/cmd_vel` from `Lite3` is not recorded: the recorder asks for reliable,
  `Lite3` publishes best effort (QoS warning on start).
- `bin/report.py` fixes made from these runs: walked length counts 5 cm
  steps (50 Hz standing jitter had doubled it), and `python3 -u x.py` is
  named `x.py`.

## 24. Task 13a on the robot: routes on the map and the lidar only (2026-10-14)

Commit `d4dd7df`: in map mode the global costmap is `static_layer,
scan_layer, inflation_layer` (checked in `/tmp/nav2_map.yaml`); the local
one keeps depth and sonars. Same standard route as the morning's `lab2`
run (section 23), left door open, right closed, fresh boot, 95% battery.
Run `logs/2026-10-14_13a`; start pose from `check_map.py` (fit 1.00 at
(-0.20, 0.00, 4 deg), on the mark this time).

| | morning (section 23) | 13a |
|---|---|---|
| walks that arrived | 2 of 3 | 4 of 4 |
| walk 1, out through the door | 70 s | 23 s |
| walk 3, out again | "goal in a wall", then "no path" | arrived, 47 s |
| "lost" stop in the corridor turn | once | once (walk 4); `here` from the lidar fit, then arrived |
| arrival at the mark (tape) | 0.21 m | 0.11 m and 0.21 m |
| camera messages dropped | 2,671 | **1** |
| Jetson busy, mean | 67% | 73% |
| AMCL spread, max | 0.52 m | 0.53 m |

- The dropped camera messages were the global costmap's depth layer
  waiting for transforms: with depth out of it they are gone.
- Jetson busy did not fall. The two runs are not alike (the morning one
  had minutes lying down and standing idle, which pull the mean down), and
  the camera driver read 78% of a core against 63%. Load is for phase 2.
- Arrival is measured from the front of the body from now on. The second
  return was 21 cm to the left, dead on forward, a little turned; he
  believed about 14 cm.
- The corridor turn still loses him: AMCL's spread reaches the 0.5 m limit
  and the estimate drifts (22 cm from the lidar's best fit at the door).
  Not something 13a touches: building-navigation plan task 10.

### No sideways movement for Nav2 (same day)

He crab-walked a lot and slid left into a table leg on a return to the
start. Recorded `/cmd_vel` of the 13a run: commands more sideways than
forward in 24%, 40%, 12% and 51% (the return with the leg) of each walk,
up to 0.20 m/s. The controller allowed `max_vel_y` 0.2 but `min_vel_y`
-0.02: left only. With `PreferForward.strafe_x` at 0.05 (section 14) its
strafe penalty almost never applies. The leg (front-left, at the table's
corner, visible in the front camera) is where nothing looks: too thin and
too close for `/scan` (range_min 0.4 m), outside the depth camera's ±43°
and the front sonar's ±33°.

Changed with the user's yes: `min_vel_y` 0.0, `max_vel_y` 0.0,
`vy_samples` 1 in both copies of `lite_nav2_mapless.yaml` (src and
install, backups `*.pre-novy`), and in `env/nav2-mapless-lite3.patch`
(regenerated: the vendor file recovered by reversing the old patch, which
round-trips; the new patch reproduces the robot's file). Takes effect at
the next floor load. Not run yet.

### The no-sideways run (`logs/2026-10-14_novy`)

`lab2` reloaded with the new limits (`/tmp/nav2_map.yaml`: `max_vel_y`
0.0, `min_vel_y` 0.0, `vy_samples` 1), pose from the lidar fit ((-0.20,
-0.05, 6 deg), fit 1.00). Two walks from the CLI, then the user sent three
more goals from the panel; five Nav2 goals in the bag.

- **Sideways: 0.000 m/s in every command of all five goals.** No crab
  walking, and he cleared the table leg he had slid into before.
- Both CLI walks arrived; no "lost" stop in the corridor turn this time.
- After the last panel goal (ending at the mark): 6 cm left of the mark,
  a little turned (tape, front of the body).
- **He touched another robot parked to his right, on its big rubber
  tyres, at the end of walk 2** (`go start` from the CLI; first written
  here as a panel goal, wrongly). Nav2's part ended at 284.5 s (its last
  `/cmd_vel`), at about 138 deg. Then `go_to`'s final turn to the place's
  heading turned him on the spot from 134 to 4 deg, about 130 deg to the
  right, in 6 s (285.6-291.3 s). That turn is `Lite3.turn()`, not Nav2:
  it never looks at the costmap, and its only check is the depth camera's
  ±43 deg ahead. During and after it the low `/scan` had something
  0.45-0.56 m away on his right and right-rear (the parked robot); its
  tyres were lower than anything the lidar sees there (lowest side ray
  -7 deg). Plan task 13g.
- The CLI had printed walk 2 as the last goal: goals sent from the panel
  during a recording are easy to miss. Note them as they are sent.

### Turns on the spot look at the costmap first (task 13g, same day)

`Lite3.turn()` now sweeps his body (0.7 x 0.5 m: the Nav2 footprint and
its padding) through the turn in 5 deg steps over Nav2's local costmap
(depth, sonars, lidar; the global one if there is none) and refuses the
turn if the body would cover an obstacle cell it does not cover at the
start (`nav.sweep_hit`, `Nav.turn_blocked`). `Lite3` subscribes to
`/local_costmap/costmap` for it. Every turn goes through it: `go_to`'s
face-the-route and final-heading turns, `turn_deg`, the panel.

Turning the other way round instead was planned and dropped: with a
rectangular body, any point near enough to be touched one way is swept the
long way round too (the long way covers more than 180 deg).

Laptop checks (`demos/check_motion.py`): a cell 0.35 m to his right refuses
a 130 deg turn either way with nothing sent; a cell 1.4 m away does not; a
cell he already stands on does not; in map mode the final turn to a
place's heading is refused and he arrives facing the way he came.

On the robot (commit `d591f82`), standing on the `lab2` mark with the
parked robot still beside him: the local costmap (odom, 8 x 8 m, 0.1 m
cells) marks it as a block of obstacle cells at his right-rear, 0.33 to
0.85 m to his right and 0 to 0.5 m behind his centre: about 8 cm from his
side (4 to 14 cm with 0.1 m cells). So the local costmap does see it
(lidar or rear sonar; which one not checked). Asked without moving: right
turns are clear to about 45 deg and touch from 50 deg; left turns touch
within 10 deg (his rear swings right). Walk 2's -130 deg final turn would
have been refused about 50 deg in.

Live, user's go, handheld ready: `turn -130` refused ("would touch
something 65 deg in"), pose unchanged at (0.04, 0.01, 3 deg); `turn -30`
turned, 3 to -28 deg. Not yet seen: whether depth or sonar marks refuse
turns in the open that are fine.

## 25. Floor maps with the real floor (task 13b, 2026-10-14)

How the Orin's `map_walls.py` labels a cell: wall where its column of
points is 1 m or taller, free where the column holds any point, unknown
where it holds none. Live FAST-LIO2 clouds are sparse (10 cm), so open
floor came out speckled with unknown cells (`lab6` from the Orin: 24,870
free, 90,488 unknown, 3,922 wall). The robot reads unknown as a wall, and
his half-width round each speckle closed rooms and doorways.

`env/floor_free.py` (run by `env/get_map.sh` before `walked_free.py`, for
one-level runs): an unknown cell becomes free where the run's own cloud
(`gravity_aligned.ply`, same frame as the map) has a point within 0.12 m
of the floor under the nearest path pose (path height minus the lidar's
0.46 m), then where it is a pinhole in free space (5 x 5 closing). Walls
are never touched. The Orin is unchanged. `get_map.sh` now keeps the
replaced map as `map.pgm.prev`/`map.yaml.prev` and takes `JUMP=` to work
over the robot's own Wi-Fi.

Both floors re-made on the robot from their 10-12 runs:

| | free cells before | after | where his centre can be (0.25 m from walls and unknown) |
|---|---|---|---|
| `lab6` | 24,870 (+ walked strip) | 29,012 | 13,585 -> 20,420 cells (+50%) |
| `lab2` | 8,505 (+ walked strip) | 9,661 | |

The narrowest point of a route (widest route, its least clearance):

| route | old robot map | new map |
|---|---|---|
| `lab2` mark -> (3.4, 0.5), the left door | 0.35 m | 0.30 m |
| `lab6` mark -> hallway (-9.86, 8.30) | 0.38 m | 0.38 m |
| `lab6` mark -> (3.26, -4.36) | 0.38 m | 0.38 m |

The `lab2` door got narrower, not wider: 594 cells that were free on the
robot's old `lab2` are walls on the Orin's map, and the new pipeline keeps
walls. 594 is exactly what `walked_free.py` added on 10-12, so it looks as
if the version used then freed wall cells within 0.6 m of his path (the
door frame); the present one does not. The door on the map: at (2.8, 1.9)
about 0.8 m from wall to wall, between a long wall and a large shape that
is very likely the open door leaf (2 m tall, so a wall). 0.30 m of
clearance against his 0.25 m half-width with padding: 5 cm per side,
against AMCL errors of 10-15 cm. Maps do not fix that; the footprint's
0.05 m padding (if his body allows) and localization at doors do.

Not yet driven on the new maps.

### The `lab2` door: 92 cm real, and what narrows the way (same day)

The user measured the left door: about 92 cm. Margin for his 0.5 m (with
padding): 21 cm per side, not the map's 5 cm.

Tried and dropped: thinning walls by a cell where floor shows inside a wall
cell (`thin_walls` in `floor_free.py`). Drawn over the Orin's `lab2`,
nearly every wall cell has floor-height points in it: walls stand on the
floor, so the test cannot tell a wall that clips its cell from a real one,
and it shaved every thick wall's edge. Gain at the narrow point was 0.29 to
0.31 m. Removed before use; the robot's maps never had it.

What the picture shows instead: his path leaves the room at about
(2.7, 1.9) and runs up beside a large wall mass at x 3.3 to 4.3, just
outside the door: where two people stood while `lab2` was recorded
(section 19 notes, the "two blobs"). The route's narrowest point is
between the door and them, not in the door frame. The fix is the one
already planned: record `lab2` again with the corridor empty (it then goes
through `floor_free.py` and `lio_clean.py`'s ghost removal on the Orin).
