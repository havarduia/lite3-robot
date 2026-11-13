# Review report: Lite3 robot, code + hardware stack

Written 2026-11-13 from `docs/review-brief.md`. Read-only review: nothing was
edited and nothing was run on the robot. The repo's offline self-checks all
pass (`python3 -m robot.{lio_relay,locate,shadow_link,orin_nav,cloud_locate,mapmaker} check`,
`orin/bin/orin_nav.py check`, `orin/bin/shadow_nav.py check`,
`demos/check_motion.py`).

Short version: nine findings, none of which need a rework to fix. Keep the
architecture, fix these, finish the MPPI migration, then strip Nav2 off
perception.

Not reviewed in depth: `robot/cloudreg.py`, `orin/bin/lio_live.py`, the Orin
mapping scripts, `robot/hmi_static/index.html`, `robot/talk.py`, `bin/tour.py`,
`env/nav2_map_params.py`, the body of `orin/lite3_terrain/src/terrain_level.cpp`.

## Findings, most important first

### 1. Cancel and E-STOP don't stop the turn before a Go

- **Where:** `robot/nav.py:617`, `robot/nav.py:396`, `robot/hmi.py:253-266`,
  `robot/lite3.py:596`
- **Why it matters on the robot:** `go_to_point()` turns him towards the route
  before Nav2 gets the goal. A Cancel in that phase finds no goal handle and
  does nothing, so he finishes the turn and walks off to the goal. E-STOP only
  interleaves zeros with the turn commands; he keeps turning, up to about 11 s
  (180 degrees at the 0.28 rad/s he delivers). The "no panel connected" check
  (`abandoned()`) has the same hole: a page that drops during route planning or
  the turn leaves the Go running with nobody watching.
- **Fix:** have `goto_cancel()` and `estop()` set a stop time, let `_safety()`
  return a reason when it is newer than the motion's start (as `stick_time`
  does), and check it in `go_to_point()` after `route_to` and the turn, before
  the goal is sent. `_stop_asked` is already set at `nav.py:400` but never
  read. `abandoned()` should act on `busy` starting with "goto", not on a goal
  handle existing.
- **Confidence:** verified in code.

### 2. The depth guard never notices a dead camera

- **Where:** `robot/lite3.py:160`, `robot/depth.py:40-47`
- **Why it matters on the robot:** the last cloud is kept forever, with no
  arrival time. If the RealSense stops, `walk()` and the panel's drive pad keep
  reading the frozen picture as "clear"; the "lost the depth stream" stop can
  only fire before the first cloud.
- **Fix:** store `time.monotonic()` with the cloud and raise `Lite3Error` in
  `wait_cloud()` / `scan` when it is older than about 1 s.
- **Confidence:** verified in code.

### 3. "Localized" stays true on fixes that were never applied

- **Where:** `robot/lio_relay.py:171-184`, `robot/locate.py:185`
- **Why it matters on the robot:** when a fix's stamp matches no remembered leg
  pose (FAST-LIO2 more than 2 s behind, which the docs say happens),
  map -> odom is not updated, but the row still goes out as OK. `locate.py`
  refreshes its timer on it, so he navigates on a frozen correction plus leg
  drift and never reports lost.
- **Fix:** send the row as REFUSED when `odom_then` is None.
- **Confidence:** verified in code. How often it happens needs a runtime
  check, on perception after a run:
  `journalctl -u lio_relay | grep behind`

### 4. The Orin's "sensors alive" gate has two holes

- **Where:** `orin/bin/orin_nav.py:211-213`, `robot/orin_nav.py:86`
- **Why it matters on the robot:**
  - The lidar obstacle cloud `/terrain_h` isn't checked at all. `nav.sh`
    starts terrainAnalysis unsupervised; if it dies, odometry and TF carry on
    and MPPI drives on frozen lidar obstacles. I believe Humble's collision
    monitor ignores a stale source rather than stopping, but I couldn't verify
    that for the installed Nav2 version.
  - The camera check times perception's voxel grid
    (`/local_costmap/voxel_grid`), which the costmap probably keeps publishing,
    emptying, with the camera dead.
- **Fix:** add a `/terrain_h` age to the `blind` test, and on perception send
  depth only while the camera cloud itself is fresh.
- **Confidence:** the missing check is verified in code; the consequences are
  likely. Bench check with `ORIN_NAV_DRY=1`: kill terrainAnalysis, then stop
  `realsense_ros2`, and see each time whether the Orin log says
  "camera or sonars stopped coming".

### 5. Depth voxels are matched across two computers' wall clocks

- **Where:** `orin/bin/orin_nav.py:127` and `:228`, `MATCH_S = 0.3` in
  `orin/bin/shadow_nav.py`
- **Why it matters on the robot:** in the recorded runs the two clocks agree
  within 30 ms, so it works with internet. The Orin has no clock battery: it
  boots with its clock behind and NTP steps it. After a boot without internet
  (handheld mode) no depth packet would match, the Orin would count as blind,
  and he would never drive on the Orin stack, with a log line that points at
  the camera.
- **Fix:** make the Orin the time server for perception, or estimate the
  offset from arrival times as `lio_relay` already does (`ahead`).
- **Confidence:** verified in code; the impact needs a runtime check. On both
  computers, with internet and after an offline boot:
  `timedatectl timesync-status; date +%s.%N`

### 6. Safety timers use the wall clock

- **Where:** `robot/lite3.py:185` and `:376`, `robot/hmi.py:319` and `:338`,
  and about 30 other `time.time()` intervals in lite3/hmi/nav/depth/orin_nav
- **Why it matters on the robot:** a backward clock step makes stale odometry
  look fresh and holds the drive deadman open for the size of the step.
- **Fix:** `time.monotonic()` for every interval.
- **Confidence:** verified in code; low probability.

### 7. The Orin panel's Navigate tab breaks the live scan

- **Where:** `orin/bin/lidar_web.py:256-270` and `:354`
- **Why it matters on the robot:** opening the tab polls `/api/nav/state`,
  which kills `lidar-scan.service`'s launch and starts its own copy with
  `lidar_z` 0.10 instead of 0.14. systemd then restarts the service, so two
  publishers feed `/scan` at different heights. The tab belongs to the old
  `~/robot_pc` setup.
- **Fix:** delete the tab and its endpoints along with `nav_link.py`,
  `robot_send.sh` and `robot_pc_check.sh`.
- **Confidence:** verified in code.

### 8. Control ports may be open on campus Wi-Fi

- **Where:** `orin/bin/lidar_web.py:593`, `robot/cloud_locate.py:212-218`
- **Why it matters on the robot:** the Orin panel binds every interface with
  no login (it can stop FAST-LIO2 and shows the thermal stream), and the
  localizer takes "I am here" and "unload map" datagrams from any sender.
  `tts_server.py` already binds the cable only.
- **Fix:** drop TCP 8000 and UDP 8042-8046 arriving on `wlP1p1s0`, or check
  the sender in `commands()`.
- **Confidence:** the binding is verified in code; the exposure needs a
  runtime check on the Orin: `sudo iptables -S INPUT; ss -lntu`

### 9. Duplication and drift

- The brief's lead was wrong on one point: `robot/orin_nav.py` and
  `orin/bin/orin_nav.py` are the two ends of one link, not copies. Their
  GOAL/CANCEL/STATUS/PATH layouts are duplicated but cross-checked by
  `check()`. CMD/DEPTH/SONAR and the ports are duplicated between
  `robot/shadow_link.py` and `orin/bin/shadow_nav.py` with no check.
- The real copies are `lio_relay.py`, `cloud_locate.py`, `cloudreg.py` (robot/
  and orin/bin) and `lidar_nav.launch.py` (env/ and orin/bin). They differ only
  in docstrings and one unused import, so they are in sync today, but there
  are three of each (repo, snapshot in `orin/bin`, the live one in the Orin's
  `~/bin` outside git), synced by hand.
- `odom()`, `depth()` and `sonar()` are duplicated between
  `orin/bin/shadow_nav.py` and `orin/bin/orin_nav.py`.
- `orin/bin/lio_relay.py.pre-cloud` (442 lines) is a leftover.
- **Fix:** one small deploy script (rsync `orin/bin/` plus the three `robot/`
  files to the Orin) instead of tracked copies, and delete the leftover.
- **Confidence:** verified in code (the Orin's `~/bin` not compared).

### Smaller notes

- The Orin's Nav2 children in `nav.sh` are unsupervised: a dead `orin_nav.py`
  only shows as "the Orin went quiet" / "couldn't get there".
- A goal accepted late on the Orin after perception gave up runs with nobody
  driving. Harmless, since perception gates `/cmd_vel`.

### Motion command paths

Three paths can send velocity: `Lite3._drive` (walk, strafe, turn, steer,
halt), `robot/orin_nav.py` (only while its goal is executing and the MPPI
command is under 0.3 s old), and `shadow_link drive`; the old stack adds Nav2's
controller. Each ends in zeros and has a staleness stop. The vendor bridge has
no watchdog of its own, but `jy_exe` stops about 1.5 s after commands stop
(`docs/environment.md:79`), which covers a killed process. The holes are
findings 1, 2, 4 and 6.

## Restructuring ideas

1. **Take Nav2 off perception in Orin mode.** It still runs a planner, both
   costmaps and a map server only for the panel's pre-checks (`cost_at`,
   `route_to`, `turn_blocked`) and as camera memory. Send the camera points
   straight to the Orin instead. Removes: perception's planner_server,
   controller_server, map_server and `sonar_range` in Orin mode. Cost: 3-4
   days (steps 2 and 3 below). Should be the largest CPU saving on perception.
   Wait until MPPI has passed the bridge walk.
2. **One file for the wire formats.** The datagram layouts and ports are
   spread over four files, and only some are cross-checked. Removes: the
   unchecked duplicates. Cost: a couple of hours. Can be done now.
3. **Retire the scan path after the migration.** `lidar-scan`, the scan
   relaying, AMCL, RPP/DWB and `shadow_*` only serve the old stack. Cost: half
   a day. Wait until the fallback is no longer wanted.

## Part 2: would I rebuild it?

No. Starting today on this hardware I would aim for the same split the
migration is heading towards, with perception thinner:

```
 Orin (Humble)                          Perception (Foxy)
 lidar, FAST-LIO2, localizer            vendor bridge, RealSense
 Nav2: Smac + MPPI + monitor   <-UDP->  one relay: odom, sonar, camera out; cmd in
 TTS                                    panel + Lite3 (owns stop and /cmd_vel)
                                              |
                                              | UDP 43893
                                        Motion (jy_exe)
```

- **One ROS distro:** as far as I know the Xavier NX is not supported by the
  JetPack release that brings Ubuntu 22.04, and the vendor packages are Foxy
  builds, so Humble on perception means a source build or Docker. Not worth
  it; the UDP link stays.
- **Retiring perception:** would need the camera cable moved, the vendor
  telemetry bridge rewritten for the Orin, `jy_exe`'s telemetry repointed and
  the panel moved. The Orin Nano's CPU budget and the vendor config are
  unknowns. Too risky for what it saves.
- **Process layout:** `nav.sh` running binaries directly instead of launch
  files is a measured workaround; leave it. The number of relays drops by
  itself with the steps below.
- **Navigation target:** MPPI + terrainAnalysis on the Orin is the right
  target; the drives so far show it.

Steps, each leaving a robot that walks. The day counts are rough guesses.

1. **Fix findings 1-4 and 7:** about 1 day. Check with
   `demos/check_motion.py`, the offline checks, and the bench run from
   finding 4.
2. **Finish the migration,** including the bridge walk, then drop perception's
   planner, global costmap and map server in Orin mode; the panel's pre-checks
   become the Orin's answer (its goal checker already has
   `yaw_goal_tolerance: 0.25`). About 1 day. Measure perception's CPU before
   and after with `bin/run.py` or `top`.
3. **Camera points direct to the Orin** (voxel layer and a collision-monitor
   source there), perception's controller_server and `sonar_range` off in Orin
   mode: 2-3 days. Carry over the 2.5 s camera memory and the "ahead of the
   nose only" (x > 0.35 m) filter that fixed the bridge glare. Measure
   perception's CPU again, and repeat the doorway and bridge walks.
4. **Delete the old stack** once the fallback is no longer needed (RPP/DWB,
   the AMCL path, `shadow_*`, `lidar-scan` and the scan relay): half a day.

**Verdict:** don't rework. Do step 1 now, the rest after the MPPI migration.
