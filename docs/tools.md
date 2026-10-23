# Programs and tools

## `bin/tour.py`: walk a route and comment on it

```
python3 bin/tour.py "walk 1.5, turn 90, goto 2 -90, person, follow 60"
```

Verbs: `walk <m>`, `turn <deg>`, `goto <m> [<deg>]` (Nav2), `approach <m>`,
`person` (walk up to the nearest person), `follow <s>`. Flow: heartbeat →
wait for the interlock → stand → (if needed) wait for the pose to settle and
start Nav2 → for each step: move, stop, **look and talk** (it only talks while
standing still; the motors are too loud while walking) → stop Nav2 → sit →
drop heartbeat. A failed step is reported and skipped. `--finale "..."` ends
with a custom line, optionally tilting the body up (`--look-up 12`) so the
camera sees a face. An `Odometer` thread sums the actual path length for the
closing sentence (summing start-to-end per step counted a 14 m follow as
7.2 m).

`walk` steps add half a second of travel to `--stop`, because `walk()` samples
the depth camera at 4 Hz and has no slow-down ramp. When the last step is
`person`, the finale line is written during the last metre so he can speak on
arrival (frame grab plus Gemini take 3–4 s), but only without `--look-up`:
that frame is taken before the tilt and shows knees and shoes, so with
`--look-up` the finale is done live after tilting, at the cost of ~3 s of
silence.

## `bin/teleop.py`: keyboard driving over SSH

`w/s/a/d`, space to stop, `+/-` speed. Keys are momentary: it stops 0.5 s
after the last keypress, so a dropped SSH connection stops the robot.
**No obstacle guard**. `--selftest` checks the key mapping without a robot.

## `demos/`

- `demo.py`: ten lessons from "connect" to "run with no handheld". Lessons
  that move the robot need `--move`. Lesson 4 is worth showing: it tries unsafe
  things and prints the API's refusals.
- `shake.py`: end-to-end regression: stand, walk, turn ±90, Nav2 goal, sit,
  summary table.
- `estop_nav.py`: sends a real SIGINT 6 s into a Nav2 goal and measures the
  coast.
- `check_motion.py`: what the motion code promises (a halt on every way out,
  clamping, the time ceiling, handheld takeover, the ways a Go ends, a quick
  `nav_stop()`) against a fake robot and a pretend Nav2. Sends nothing, so it
  runs anywhere; run it after changing the motion code.

## `robot/protocol.py` as a CLI

`python3 -m robot.protocol stand_toggle`, `... dance`, `... 0x21010300`,
`... camera on`, `... voa off`. Sends a raw code with **no checks**; for
debugging and capturing unknown codes only.

## `robot/rviz.py`: the live robot in rviz2 on the laptop

    python3 -m robot.rviz                # on the laptop, in ~/lite3_api: the model only
    python3 -m robot.rviz sensors        # plus depth cloud, sonars, costmaps, Nav2 path, goal, person
    python3 -m robot.rviz nav            # the floor map view (env/lite3_nav.rviz), goals from rviz
    python3 -m robot.rviz /some/topic    # plus any topics you name
    python3 -m robot.rviz --record walk.rec sensors    # the same, and kept in walk.rec
    python3 -m robot.rviz --play walk.rec              # walk.rec again, with no robot
    python3 -m robot.rviz --play walk.rec 0.25         # at quarter speed (2 = twice as fast)

Opens rviz2 with the Lite3 model moving its legs as the robot does. Close
rviz (or Ctrl-C) to stop everything, including the robot end.
Run it on the laptop, not on the robot: it reaches the robot with
`ssh ysc@lite3-perception` (Tailscale). With no Tailscale (handheld mode,
the laptop on his wifi) give it the hop through the motion computer:
`LITE3_ROBOT='-J ysc@192.168.2.1 ysc@192.168.1.103' python3 -m robot.rviz nav`.

**How.** The laptop starts `python3 -m robot.rviz --send ...` on the robot
over ssh. That end subscribes to `/joint_states`, `/tf`, `/tf_static`, `/odom_fused` and
the topics you name, and writes the messages, undecoded, down the ssh pipe;
the laptop end republishes them unchanged. `robot_state_publisher` on the
laptop turns `/joint_states` plus `urdf/lite3.urdf` into the leg frames.

**The floor map view (`nav`).** Opens `env/lite3_nav.rviz` instead: fixed
frame `map`, seen from above and following him, with the floor map, the
walls scan, the local costmap, Nav2's global and local paths, RPP's
lookahead point and collision arc, his footprint, AMCL's particles (with
`LOCALIZER=amcl`), the sonars and the panel's goal. It needs Nav2 up with a
floor loaded; without the `map` frame rviz shows nothing but a frame
warning. The global costmap is in the preset but off and not relayed: on a
big floor (the arcade map is 200 x 125 m) it is megabytes a second over
ssh. Add `/global_costmap/costmap` on the command line to have it anyway.

rviz's **2D Goal Pose** and **2D Pose Estimate** go back up the same ssh
link: the robot end hands them to the panel (`robot/hmi.py`, at
127.0.0.1:8080 there) as its Go here (`go_point`, with the arrow's heading)
and I am here (`here`). So the panel's checks apply, the panel shows the
goal and its Cancel, and the panel's log comes back to the laptop's
terminal (`panel: ...`), where a refused goal says why. The panel must be
running. While rviz is connected the panel counts it as a page: closing
rviz with nobody else on the panel cancels a goal, as closing the page
does. Only poses in the `map` frame are sent; in the plain view (fixed
frame `odom`) they are refused with a note. There is no Cancel in rviz:
use the panel, or its E-STOP.

**Record and replay.** `--record FILE` writes every message that comes down
the pipe to FILE, each with the time it arrived, while showing it as usual.
`--play FILE` feeds rviz from FILE with the same timing and no connection to
the robot, so a walk or a failed follow can be looked at again afterwards,
slowed down if need be. What is in the file is what was relayed: name the
topics you will want when recording (`sensors`, a costmap, ...).
- Size: the model alone is about 10 kB/s; with `sensors` the depth cloud
  dominates, 3.5 MB for 23 s on a relayed link (2026-10-07), more on a
  direct one. `*.rec` is in `.gitignore`.
- A replay runs once and then leaves rviz open. The messages keep the
  robot's own time stamps, and rviz ignores frames older than ones it
  already has, so looping in the same rviz would show a frozen robot. Run
  it again to see it again.
- There is no pause or step. Slow it down with the speed number.
- It replays into rviz exactly what the live view does, the laptop-made
  `odom -> base_link` and sonar cones included, because those are made
  from the recorded messages on the way through.

**Why not plain DDS.** The laptop only reaches the robot over Tailscale,
which carries no multicast, and the robot's Foxy CycloneDDS talks on one
interface only (`eth0`, which the bridge to the motion computer needs).
Moving it to Tailscale means reconfiguring every node on the robot. The
pipe needs nothing changed on the robot and no open ports.

**The model.** `urdf/lite3.urdf` with the small meshes in `urdf/meshes`
(3.4 MB, in git) always works. On its first run the laptop end also
downloads the vendor's dense meshes (about ten times the triangles, 57 MB)
from `DeepRoboticsLab/Lite3_rl_training` and Intel's D435 mesh (16 MB, in
place of the grey box where the camera sits) into `urdf/meshes_hd/`, which
git ignores, and uses those from then on. The vendor publishes no Lite3 Pro
model, so the computer box on the back is not shown. No network on that first run: it says
so and shows the small ones. Delete the folder to go back to them.

**What to expect.**
- `sensors` also shows **what he intends**: Nav2's path (`/plan`, and the
  controller's `/local_plan`), its goal as an arrow on the floor and the
  person he sees as a column. Goal and person come from the panel
  (`/intent/goal`, `/intent/person`), so the panel has to be running; the
  laptop end draws them as markers that fade 1.5 s after the last one, so
  a reached goal or a lost person disappears. (Markers are made on the
  laptop because the Marker message changed after Foxy and would not
  survive the relay.)
- `sensors` shows what fits the 3D view: the depth cloud; the two sonars
  as cones (made on the laptop from
  the bare readings, the way `robot/sonar_range.py` does on the robot);
  the global and local costmaps once Nav2 is up. The lidar is on the
  Orin's ROS graph, not this one ([Lidar and FAST-LIO2 odometry (robot/lio_relay.py)](lidar.md)), and the IMU has no display.
- Each topic is capped at 20 Hz (`MAX_HZ`), messages over 20 kB (the
  cloud) at 5 Hz; latched topics (`/tf_static`, costmaps) are passed
  whole. A topic that appears later (Nav2 started after rviz) is picked up
  within 2 s.
- The link decides the rest. When Tailscale cannot connect the laptop and
  robot directly it relays through a server (`tailscale status` says
  `relay`), and everything shares roughly 300 kB/s. Each topic then waits
  four times as long as its last message took to send, so a big topic
  cannot starve the model. Measured 2026-10-07 on a relayed link, with the
  160 kB camera picture also going (since removed): cloud 2 Hz, sonars
  11 Hz.
- Every extra topic is one more subscriber on a loaded Jetson ([Measured numbers and calibration](environment.md#measured-numbers-and-calibration)).
  The depth cloud is the heavy one: name it when you want it, not always.
- A message type the laptop does not have (`transfer_interfaces`, the
  RealSense extras) is skipped with a line saying so.
- Fixed frame is `odom`, so the grid is the floor and the robot walks and
  turns across it. The laptop end makes `odom -> base_link` itself: x, y
  and heading from `/odom_fused`, height from the joint angles (the lowest
  foot, knee or the belly rests on the floor, body taken as level). The
  odometry's own height stays at standing height (0.32 m) when the robot
  lies down, which left the model floating. The robot's own
  `odom -> base_link` (from `lio_relay.py`, [Lidar and FAST-LIO2 odometry (robot/lio_relay.py)](lidar.md)) is dropped for that reason.
  `python3 -m robot.rviz --check` tests the height sum.
- Time stamps are the robot's steady clock, untouched, so the cloud and the
  frames agree with each other but not with the laptop's clock.
- The laptop end runs with `ROS_LOCALHOST_ONLY=1` and without the laptop's
  own `CYCLONEDDS_URI`, so nothing leaves the laptop. To use `ros2 topic`
  beside it, set the same two things in that shell.
- After a cold boot (basic state 98) the thigh angles the robot reports are
  not yet meaningful: lying in the ready pose it reports about ±0.3 rad and
  the model's thighs hang straight down. Once it has stood up the angles
  are right (standing: thigh 0.67, knee -1.35 on `/joint_states`).
- If the link drops (the laptop changes network), the laptop end notices
  within about 6 s and reconnects every 2 s until the robot answers.

## Measuring a run: `bin/run.py`, `bin/report.py`, `bin/tags.py`

To judge a Nav2 or FAST-LIO2 test by numbers instead of by eye. All three
run on the laptop. Plan: `.planning/plans/compute-distribution.md`.

```
bin/run.py start door-test          # start recording on all three computers
bin/run.py start door-test --frames # also front camera frames (people: PRIVACY.md; deleted an hour after stop)
bin/run.py note "arrived 0.07 m"    # remarks, stamped with the laptop's clock
bin/run.py stop                     # stop, fetch into logs/<date>_door-test/, clean up
bin/report.py logs/2026-10-14_door-test
bin/report.py --compare logs/<before> logs/<after>
```

What `start` does, on each computer, detached and under `timeout`
(`--minutes`, default 40), so a dropped ssh ends nothing early and leaves
nothing running past the limit:

- **all three**: `robot/loadwatch.py` (copied over; stdlib only, so it runs
  on the motion computer and the Orin, which have no repo) writes CPU per
  core, load average, free memory and the top 15 processes every 2 s;
- **perception computer**: `ros2 bag record` of the topics in
  `env/runlog_topics.txt` (odometry from all three sources, AMCL,
  `/localized`, `/cmd_vel`, `/plan`, goal status, `/scan_walls`, state).
  Light topics only: no clouds, no images. A Python logger would cost about
  17% of a core per 50 Hz topic here; the recorder is C++.

`stop` stops them by pid (never by name), adds `nav2.log` and the relay's
journal (perception computer) and the lidar services' journal (Orin), and
copies everything into `logs/<date>_<name>/<computer>/`. It reaches the
robot through the hotspot when the laptop is on it, else over Tailscale.
`--local` does the whole thing with the laptop as all three computers
(the bag on DDS domain 77), as a dry run.

Clocks: `start` records each computer's offset from the laptop, and the
report puts everything on the laptop's clock. The bag is read by the time
each message arrived, not by its stamp (stamps use the legs' steady clock).

`report.py` writes `report.txt` and `report.png` into the run's folder:

- **per goal**: result, time, distance walked against the planned length,
  recoveries, the most frequent Nav2 warnings, and the arrival error if a
  note after the goal says "arrived 0.07 m";
- **odometry**: each source's track from its own first pose, the
  start-to-end gap (the loop closure error when the run ends where it
  began), and how far `/odom_fused` and `/lio_odom` drift from the legs;
- **localization**: AMCL spread against `locate.SPREAD_XY_M`, scan fit,
  share of the time localized;
- **relay**: falls back to leg odometry, FAST-LIO2's worst lag;
- camera messages dropped, battery, and the **load** of all three computers
  with their busiest processes.

`--compare` puts the numbers of two runs side by side: the same walk
before and after a change.

**Tags in `lab2b`** (`env/tags/lab2b.json`): 9 in front of the start mark,
1 and 3 either side of the left door (room side), and 4, **temporary**: put
it up only for test sessions, in the corridor 23 cm to the right of the door
with a red sign on it, flat on the wall, and take it down after.

**AprilTags** (`bin/tags.py`) are the ground truth: `make` writes ten
AprilTag 36h11 sheets (14 cm black square) and `board` a ChArUco board
for calibrating the front camera. Print at 100% and measure the 10 cm
scale bar. Do not trim the sheets: the detector needs two cells of white
round the tag and the paper is that margin (with one cell it misses the
tag on anything darker than paper). In the synthetic check a tag is found
to 1 mm at 0.6 m, 4 mm at 1.5 m and 2.5 cm at 3 m, 45° off. Detecting
tags in a run's video and the camera calibration are tasks 8c-8g of the
plan, not built yet.
