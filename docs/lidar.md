# Lidar and FAST-LIO2 odometry (`robot/lio_relay.py`)

Since 2026-10-08 the robot carries an Ethernet switch (TP-Link TL-SG105E)
with a Livox MID-360 lidar (192.168.1.102) and an Orin Nano (192.168.1.5,
user `lite3`) on it, on the same 192.168.1.x network as the two computers.
The Orin runs the lidar driver and, when a Live run is started from its
web panel (port 8000), FAST-LIO2. Its own notes are in `~/bin/README.md`
there.

```bash
sudo systemctl status lio_relay     # it runs as a service on the perception computer
python3 -m robot.lio_relay check    # the maths only, anywhere
python3 -m robot.lio_relay start    # start FAST-LIO2 on the Orin; also: stop, state
```

It publishes two topics on the robot's graph:

- `/lio_odom`: FAST-LIO2's pose as it comes (`odom -> base_link`, 10 Hz).
- `/odom_fused`: `/leg_odom2` with FAST-LIO2's correction, at `/leg_odom2`'s
  50 Hz and with its stamps. With the Orin silent it is `/leg_odom2`
  exactly.

- `/scan`: the lidar flattened to a 2D scan (720 beams, 10 Hz, in
  `base_link`), stamped with this computer's steady clock. The Orin makes
  it (`lidar-scan.service` there runs its `~/bin/lidar_nav.launch.py`;
  our copy of the unit is `env/lidar-scan.service`): everything between
  0.15 m below and 1.0 m above `base_link`, nearer than 0.4 m dropped as
  the robot itself. It feeds the scan layer of Nav2's costmaps ([What "mapless" means](navigation.md#what-mapless-means)).
  Lying down, the floor is inside that height band and shows as a ring
  of returns from 0.6 m out, so the relay only passes scans on while the
  robot is standing (basic state 6). Measured lying down, 2026-10-08:
  with the mount transform the floor comes out level within 1° both
  ways, so the 20.4° pitch holds; it lay 0.10 m below `base_link`.
- `/scan_walls` (2026-10-09): the same, cut 0.9 to 1.7 m above `base_link`,
  which is above tables and chairs. It is for finding the robot on a floor
  map (AMCL and `robot/locate.py`), because the floor maps show only what
  runs from floor to ceiling and the low scan is mostly furniture. In the
  lab, standing on the map's start mark: all of the high scan's returns
  land on mapped walls, 37% of the low scan's (`demos/check_map.py`).
  About half its beams have a return, since the lidar is tilted nose-down.

`/odom_fused` is the robot's odometry: `Lite3.pose`, the sonar node, the
rviz relay and Nav2 all read it, because goals and obstacle positions pass between them in that
one frame. Nav2 still takes speed from `/leg_odom2`.

The relay also publishes the `odom -> base_link` transform, from the same
numbers and with the same stamps as `/odom_fused` (2026-10-09). The
vendor's `odom_to_tf.py`, a node that did only that, is no longer
launched: it cost a fifth of a core.

**The service.** `env/lio_relay.service` (`lio_relay` on the perception
computer) keeps the relay up from boot and restarts it after 2 s. If it
is down there is no odometry: `wait_pose()` refuses to start a motion,
and a motion under way stops with "odometry stopped arriving" once the
pose is `ODOM_STALE_S` (0.5 s) old. That check exists because the relay
can die on its own while the bridge still takes velocities.

**Starting FAST-LIO2 from code.** FAST-LIO2 belongs to the Orin's web
panel (its Live run), and the panel has an HTTP API, so nothing on the
Orin had to change: `lio_relay.live('start')`, `live('stop', name='hall')`
and `live('state')`, or the same three words on the command line. A stop
saves that run's map on the Orin.

**It starts by itself.** Every 5 s the relay asks: is the robot standing
(basic state 6), has leg odometry been still for the last second, and has
no FAST-LIO2 pose arrived lately? If so it calls `live('start')`, and
does not ask again for 30 s while FAST-LIO2 comes up. This lives on the
perception computer, not the Orin, because FAST-LIO2 has to start with
the robot standing still and only this side knows his posture; started
at the Orin's boot it would initialise lying down. So after a boot or a
battery change it comes on the first time he stands still. If you stop
the Live run from the Orin's panel, the relay starts a new one the next
time he stands still. It never stops a run, and it does not restart one
that has diverged: `/odom_fused` is then leg odometry until someone
stops and starts it.

**Why a relay.** The Orin is ROS 2 Humble with Fast DDS on domain 42. The
robot's nodes are Foxy with CycloneDDS on domain 0. Measured 2026-10-08
from the perception computer on domain 42: with Fast DDS the Orin's topics
arrive (`/livox/imu` 200 Hz, `/livox/lidar` 10 Hz); with CycloneDDS every
`ros2` command segfaults on meeting the Orin's node (an empty domain does
not). So nothing crosses by ROS. The same file runs on the Orin as
`python3 lio_relay.py send` (`lio-send.service` there; our copy of the
unit is `env/lio-send.service`) and sends each pose and each scan as one
UDP datagram to port 8042 here; the relay publishes them in the robot's
normal environment. Neither side's ROS settings change, and the Orin must
stay off domain 0. The file on the Orin has to be the same version as
the one here: the datagram layout is defined in it (`POSE`, `SCAN`).

Until 2026-10-09 the crossing was a child process of the relay on Fast
DDS, domain 42, printing each message as a line of text. Measured that
day, robot standing with Nav2 up: relay 38% of a core + child 27% +
`odom_to_tf.py` 21% before; relay 33% and nothing else after. The whole
computer went from 42% idle to 53%.

**A FAST-LIO2 that falls behind.** Each pose carries the time the Orin
sent it as well as its own stamp. The two clocks are compared by the send
time, and a pose is matched to the leg pose of its own stamp, so one that
is older than the 2 s of leg poses remembered finds no match and is not
used. This matters: on 2026-10-09 a Live run that had been going since
the Orin booted was 35 minutes behind and held 5 GB of memory. The relay
logs "FAST-LIO2 is N s behind" and carries on with leg odometry; it does
not restart the run.

**What it does to the pose.**
- FAST-LIO2 reports the lidar's IMU (`body`) relative to where it was when
  the run started (`camera_init`). The relay turns that into the body's
  motion with the mount transform (`MOUNT_XYZ`, `MOUNT_PITCH`): pitch is
  measured, 20.4° nose-down; x, y, z are (0.20, 0, 0.14) m, good to about
  2 cm (photos, the torso mesh and one measurement, 2026-10-09).
- It takes the body as level when the run started. A run started while
  lying or sitting tilts `/lio_odom`; FAST-LIO2 publishes its gravity
  estimate on `/lio/gravity` if that needs fixing.
- The origin is where the Live run started, so it is not `/leg_odom2`'s
  origin, and it jumps to zero when a new run starts.
- Stamps are the perception computer's steady clock on arrival, like
  `/leg_odom2`, so they match the TF the rest of the graph uses.
- FAST-LIO2 sends no speed, so `twist` (vx, vy, yaw rate, in `base_link`)
  is the difference of the last two poses.

**`/odom_fused`: how the two are joined** (`Fuse`).
- The fused pose is the leg pose moved by a correction. The correction is
  updated from each FAST-LIO2 pose and held between them, so the legs give
  the 50 Hz and FAST-LIO2 removes their drift.
- A FAST-LIO2 step is only believed if it agrees with the legs' step over
  the same stretch (`STEP_TOL`: 0.15 m and 0.2 rad per pose). If it does
  not, or more than `GAP_S` (1.0 s) passed since the last pose, the
  correction is kept as it was and FAST-LIO2 is re-anchored to where the
  legs put the robot. So a divergence, a stall, the Orin going away or a
  new Live run never makes the fused pose jump; it just goes on as leg
  odometry.
- Not caught: FAST-LIO2 drifting slowly, a little per pose.
- Poses do not arrive evenly. Measured 2026-10-08, robot still: 7.9 a
  second reach this computer, not 10; one in twenty is more than 0.35 s
  late, the worst 0.78 s; some never arrive. So each pose is matched by
  its OWN stamp to where the legs were then (`leg_at`, 2 s of leg poses
  kept). The Orin's clock is turned into this one's with the offset of
  the quickest pose seen so far, less `LIO_DELAY_S` (0.05 s, rough: in
  one loop log the two matched best 0.07-0.09 s apart, of which about
  0.05 s was the logger's own sampling).
- The first version matched each pose to the legs' pose at ARRIVAL. On a
  1 m Nav2 goal that put a false 0.24 m into the fused pose within 0.4 s
  (fused 0.86 m, legs 0.62 m, lidar-to-wall 0.63 m), and with the gap
  limit at 0.5 s it flipped between the two modes every few seconds.
- The relay prints a line when it changes between "corrected by
  FAST-LIO2" and "leg odometry alone".

**Tested** 2026-10-08:
- A made-up pose on domain 42 (lidar 1 m along its own x) gave
  `/lio_odom` x 0.937, z -0.349 at 10.0 Hz.
- Against a real Live run, robot still: 10 Hz, steady within a few mm.
- A walked closed loop (handheld, parked on a floor mark, 10.4 m, 92 s):
  end 0.05 m from start by `/lio_odom`, 0.21 m by `/leg_odom2`; both
  said the heading was 5° off, no jumps.
- `/odom_fused`, robot still, Live run on: 50.0 Hz, within 3 mm of
  `/leg_odom2`. Not yet walked, and nothing has driven on it.
- The stale-odometry stop: `demos/check_motion.py`, on the fake robot.
- Cost on the perception computer: 33% of a core, robot standing, Nav2
  up, scans passing (2026-10-09).
