# Plan: Compute distribution and run evaluation

**Spec**: none. Planned from the 2026-10-14 conversation (no spec written)
**Epic**: none
**Created**: 2026-10-14
**Status**: in progress (2026-10-14: phase 1 tasks 1-6, 8, 8a, 8b done; a first baseline (7) without tags; phase 2b added from it)

Stack: Python on ROS 2 Foxy (perception computer), stdlib Python on the
Orin and the motion computer, laptop tools on Jazzy. Same patterns as
`building-navigation.md`: self-checks instead of a test framework, scripts
in `bin/`, long explanations in the README, measurements in
`LIDAR_ORIN_LOG.md`.

Two questions, one plan, because the answer to the second comes from the
tool built for the first:

1. **How do we judge a Nav2 / FAST-LIO2 run properly?** Today a run gives
   us the user's account, a few log lines and "ended N m from the tape
   mark". That says pass or fail, but not why, and it can't show whether a
   change made things better.
2. **Is the work split well across the three computers?** The Jetson is
   saturated in map mode. The Orin and the motion computer have never been
   measured.

Phases 1 to 3 are planned to the task. Phase 4 (moving AMCL / Nav2 to the
Orin) is planned loosely on purpose: it only happens if gate A fails, and
gate A's numbers will shape it.

---

## What we know before starting

| Question | Answer |
|---|---|
| Jetson load, map mode | Load average 9.3 on 6 cores with the panel open (2026-10-12). About 50 camera messages dropped after the first goal. Camera driver 80-92% of a core, against 57-70% mapless; nobody knows why. |
| Jetson load, mapless, no panel | 53% idle after the relay rework (`LIDAR_ORIN_LOG.md` section 17). |
| Biggest Jetson users | Camera driver ~50-90%, Nav2 ~80% in total, `lio_relay` 33%, panel ~30%, `jetson2motion` ~20%, `sonar_range` ~18%, `sensor_checker` 17% (pure overhead). |
| What a Python subscriber costs here | ~17% of a core per 50 Hz topic (`/tf` check, section 17). A Python logger on the Jetson is NOT cheap. |
| Orin load | Never measured. One long FAST-LIO2 run fell 35 min behind and held 5.1 GB (section 17). |
| Motion computer load | Never measured. Vendor territory. |
| Camera settings now | Depth 424x240 at 30 fps; camera IMU on (gyro 200 Hz, accel 100 Hz, joined into `/camera/imu`); no consumer of `/camera/imu` in this repo. |
| Laptop tools | Jazzy has `rosbag2_storage_sqlite3` and matplotlib 3.6, so it can read a Foxy sqlite bag and draw plots. `nav2_msgs` is not installed, so Nav2 action feedback can't be decoded there. |
| Existing run folder pattern | `logs/2026-10-12_lab6/` holds `lio_relay.journal.txt`, `nav2.log`, `summary.txt`. Keep that layout. |
| Cross-computer link | Foxy/CycloneDDS and Humble/Fast DDS must not share a domain (segfaults). Anything that crosses goes over UDP, like `lio_relay`. |

---

## Architecture

```
laptop: bin/run.py start|note|stop <name>     bin/report.py logs/<run>
          |  ssh, detached, capped by timeout          ^
          v                                            |  fetched files
  Jetson:  ros2 bag record (topic list in env/runlog_topics.txt)
           robot/loadwatch.py  -> load.jsonl
           /tmp/nav2.log, journalctl lio_relay          -+
  Orin:    loadwatch.py        -> load.jsonl              +-> logs/<date>_<name>/
           journalctl lio-send, FAST-LIO2 state           |
  Motion:  loadwatch.py        -> load.jsonl            -+
           notes.txt (bin/run.py note, laptop clock)
```

Decisions:

1. **`ros2 bag record`, not a Python logger.** It's already installed,
   it's C++, and a Python subscriber costs ~17% of a core per 50 Hz topic
   here. Light topics only, no point clouds or images. Task 5 measures what
   the bag costs; if it's over 10% of a core we cut topics, and only if
   that fails do we write a fixed-rate rclpy logger.
2. **One stdlib load sampler for all three computers.** `robot/loadwatch.py`
   reads `/proc` every 2 s: CPU per core, load average, memory, and the top
   15 processes by CPU. One JSON line per sample with wall time and the
   host name. It must run on Python 3.8 (Jetson) and whatever the motion
   computer has (task 1 checks). No ROS, no pip.
3. **The laptop drives a run.** `bin/run.py` starts everything detached with
   `nohup` + `timeout` (cap 40 min), keeps pids in `/tmp/runlog/`, and
   stops by pid. Never `pkill` by name (the `lio_relay` trap), and no Nav2
   program name on any command line (the marker-word trap). ssh dropped
   twice on 10-12, so nothing depends on the ssh session staying up.
4. **Notes use the laptop clock**, and `bin/run.py` records each computer's
   clock offset at start (`date +%s.%N` on each one). The bag uses the
   legs' steady clock for stamps, but rosbag2 also stores receive time in
   wall clock, and the report uses that.
5. **The report works on the laptop only.** It reads the bag with Jazzy's
   sqlite plugin and `rclpy.serialization` for message types that are the
   same in Foxy and Jazzy (nav_msgs, geometry_msgs, sensor_msgs,
   action_msgs status). Nav2 recoveries and abort reasons come from
   `nav2.log`, not from action feedback.
6. **Ground truth: AprilTags, with tape marks as backup.** Family
   AprilTag 36h11 (OpenCV calls it `DICT_APRILTAG_36h11`, and it reads it
   with no extra package). This settles the family that
   `building-navigation` phase 5 left open, so the same tags serve both
   plans. Details:
   - **Camera**: the robot's front wide-angle camera (H.264, 1280x720,
     30 fps, RTSP from the motion computer; the person detector already
     uses it). It needs a one-off calibration (ChArUco board) and a
     measured mounting position. Recording copies the H.264 stream without
     decoding it, so it costs the Jetson almost nothing, and the RealSense
     colour stream stays off. The RealSense colour stream (factory
     intrinsics) is the fallback if the front camera's delay turns out to
     be too large or too uneven.
   - **Detection on the laptop, after the run**, from the recorded frames.
     It costs the Jetson nothing beyond recording the frames.
   - **Size**: 14 cm, not 16: the detector needs two cells of white
     round the tag (with one it misses the tag on anything darker than
     paper), and on A4 only 14 cm leaves that.
   - **Placement**: on walls at the front camera's height or a little
     above (the exact band is settled once the camera's pitch is measured
     in 8d). Print 14 cm tags on matte paper on stiff
     board; the paper round the tag is its white margin, so don't trim it.
   - **Main measure: revisits, which need no survey.** When the robot sees
     the same tag twice, the change in what odometry, LIO and AMCL believe
     between the two sightings is the drift. The tag pins the true pose
     both times, so camera mounting errors cancel out.
   - **Absolute check: tag positions on the map.** Each tag's position is
     surveyed once from the start mark, where the map frame is exact (it's
     the lidar's start pose). That gives AMCL a true error against the map.
   - **Backup**: the user still types the tape-measured arrival error with
     `bin/run.py note "arrived 0.07 m"` where no tag is in view.
7. **One standard run**, repeated before and after every change in phases
   2 to 4, so the numbers can be compared:
   - lying 60 s, then standing 60 s with mapless Nav2 up and no panel
   - map mode: `load lab2`, `here`, panel open, `go 3.4 0.5` and back to
     start, twice
   - 30 s standing afterwards

Not doing: an MCP server or SDK (talked about, not needed), tag detection
on the robot (it belongs to `building-navigation` phase 5), Python logging on the Jetson, recording point clouds, fixing the Orin
recorder's dropped IMU samples (separate problem).

### Components

| Component | Type | Purpose |
|---|---|---|
| `loadwatch.py` | stdlib script, all three computers | CPU, load, memory and top processes every 2 s, as JSON lines |
| `runlog_topics.txt` | topic list | What the bag records. It lives in a file so no Nav2 word appears on a command line |
| `bin/run.py` | laptop script | start / note / stop a run on all three computers; fetches files into `logs/` |
| `bin/report.py` | laptop script | Numbers (`report.txt`) and one plot (`report.png`) for a run folder; `--compare` for two runs |
| Standard run | procedure (README) | The fixed walk every measurement uses |
| `tags.py` | laptop script | `make` prints tag sheets at a known size; `survey` and the detection functions are used by the report |
| `tags.json` | per floor, next to the map | Tag id → pose on the map (from the survey) |

### Topics recorded

| Topic | Why |
|---|---|
| `/odom_fused`, `/leg_odom2`, `/lio_odom` | Where it thought it was, by each source; where they split |
| `/amcl_pose`, `/localized` | AMCL pose and spread; `locate.py`'s fit score |
| `/cmd_vel` | What Nav2 told the legs (the DWB wiggle was found this way) |
| `/plan` | The global plan each time it changes |
| `/navigate_to_pose/_action/status` | Goal start and end, and the result |
| `/scan_walls` | What AMCL saw (about 10 Hz, 3 kB per message) |
| `/global_costmap/costmap` | Latched; changes rarely in map mode |
| Robot state topic (`/robot_state_debug`, capped to 50 Hz) | Gait, battery, lying or standing |

### New files

| File | Location | Purpose |
|---|---|---|
| `loadwatch.py` | `robot/` | Load sampler; `check` argument runs its self-check |
| `runlog_topics.txt` | `env/` | Bag topic list |
| `run.py` | `bin/` | Run control from the laptop |
| `report.py` | `bin/` | Run report and comparison |
| `tags.py` | `bin/` | Tag sheets, detection + pose (OpenCV `aruco` + `solvePnP` with `SOLVEPNP_IPPE_SQUARE`), survey |
| `tags.json` | `~/lite3_maps/<floor>/` (laptop copy in `logs/`) | Surveyed tag poses |
| `front_camera.yaml` | `env/` | Front camera intrinsics, distortion and mounting (from 8d) |

### Files to change

| File | What changes | Why |
|---|---|---|
| `README.md` | New section "Measuring a run": the standard run, `bin/run.py`, `bin/report.py`, what each number means | Long explanations live in the README |
| `LIDAR_ORIN_LOG.md` | New section per measurement day | Same as before |
| `env/camera.launch.py` | Phase 2: camera IMU off, depth 15 fps (each only if its task says so) | Jetson load |
| `env/nav2-mapless-lite3.patch` / `env/nav2_map_params.py` | Only if 15 fps depth needs a longer observation persistence | Keep Nav2 marking obstacles |
| Orin `~/livox_ws/.../FAST_LIO` config | Phase 3, only with the user's OK | Memory growth |

---

## Phase 1: Measure (laptop first, then one robot run)

| # | Task | Files | Check |
|---|---|---|---|
| 1 | Read-only, on the motion computer and the Orin: Python version, `/proc` layout, CPU model per core. On the motion computer also: which cores the vendor locomotion controller `jy_exe` runs on (`taskset -p`), its scheduling priority (`chrt -p`), any isolated cores (`/proc/cmdline`), and its CPU use standing and walking | none (notes in log) | Written in `LIDAR_ORIN_LOG.md`; tells us which motion-computer cores, if any, are safe to use |
| 2 | `robot/loadwatch.py`: sample `/proc`, write JSON lines, stop at a time limit or SIGTERM; `check` runs a self-check against the laptop's own `/proc` | `robot/loadwatch.py` | `python3 robot/loadwatch.py check`; `ast.parse(..., feature_version=(3, 8))` |
| 3 | `env/runlog_topics.txt` plus a laptop check: fake publishers on domain 77 (localhost only) for the light topics, `ros2 bag record` a short bag, read it back with `rclpy.serialization` | `env/runlog_topics.txt` | Every topic in the list read back with the right type |
| 4 | `bin/run.py start|note|stop <name>`: clock offsets, detached starts with pid files, stop by pid, fetch the bag, load files, `nav2.log` window, relay and sender journals, notes into `logs/<date>_<name>/` | `bin/run.py` | Dry run on the laptop against `localhost` (ssh to self, or a `--local` switch) |
| 5 | `bin/report.py`: arrival error (from notes), time to goal, walked length vs plan length, the three odometry sources overlaid, AMCL spread vs the 0.5 m limit, relay lag (from its journal), CPU and load for all three computers on one time axis, recoveries and aborts with reasons (from `nav2.log`), loop closure error when the run ends at its start, camera messages dropped. Writes `report.txt` + `report.png` | `bin/report.py` | Run on the task 3/4 laptop output; `--compare` on two copies of it gives zero difference |
| 6 | Robot, **lying down**: what the bag and the samplers cost (60 s with them, 60 s without) | none (log) | Bag ≤ 10% of a core, sampler ≤ 2%; otherwise cut topics first |
| 7 | Robot: the standard run as the **baseline** (the user stands him up and says go) | log entry, `logs/<date>_baseline/` | Report readable; numbers into `LIDAR_ORIN_LOG.md` |
| 8 | README section "Measuring a run" (including tags) | `README.md` | The user can run `bin/run.py` + `bin/report.py` from it alone |

### Phase 1b: AprilTags (8a-8b on the laptop now; 8c onwards needs the robot)

| # | Task | Files | Check |
|---|---|---|---|
| 8a | `bin/tags.py make`: 36h11 ids 0-9 as a print-ready A4 PDF at 14 cm with a scale bar to check the print; detection + pose from a frame and `camera_info`; self-check on a synthetic rendered tag at a known pose | `bin/tags.py` | Recovered pose within 1 cm / 1° of the rendered one; the printed scale bar measures 10.0 cm |
| 8b | DONE 2026-10-14 (`tags.front_camera()`, `calibrated()`). Front camera → `base_link` transform read from `env/front_camera.yaml` (filled in by 8d), used by the report | `bin/tags.py` | Self-check with a made-up mounting |
| 8c | DONE 2026-10-14 (log section 29): 0.29 s median, spread 0.036 s; the front camera is the tag camera. Front camera delay, robot **lying down**: point it at the laptop screen showing a millisecond clock and record the stream with each frame's arrival wall time (`ffmpeg -c copy`, no decoding). Frame time against the clock in the picture gives the delay and how much it varies. Also record the motion computer's clock offset | none (log) | Delay known; its spread ≤ 50 ms, otherwise use the RealSense colour fallback |
| 8d | Lens DONE 2026-10-14 (env/front_camera.yaml, 0.20 px held out, valid area x 310-1000 y 150-525; log section 30); mounting: position from the mesh (x 0.271, z -0.006), pitch measured with a tag: 20 deg UP; a tag at a taped 1.00 m reads 1.007 m. DONE. Calibrate the front camera: `bin/tags.py board` prints a ChArUco board; record 30-60 s of the stream while the user moves the board around the view; calibrate on the laptop (fisheye model if the plain one leaves the edges bent). Measure the camera's mounting position with a tape; get its pitch from a tag lying on the floor at a measured spot | `bin/tags.py`, `env/front_camera.yaml` | Reprojection error < 0.5 px; a tag at a taped 1.0 m reads 1.00 ± 0.02 m |
| 8e | DONE 2026-10-14 (robot/frontcam.py on the Orin, 2 frames/s; log section 31). `bin/run.py` also records the front camera stream (copy, with arrival times) for the length of the run | `bin/run` | Recording runs while the panel is open, with no effect on the panel's video |
| 8f | The user places 4-6 tags in `lab2`: start mark, both sides of the door, corridor end. Survey: stand on the start mark, `bin/tags.py survey lab2`, which records the tags in view and walks the rest in once slowly right after start (LIO drift is lowest then) | `tags.json` | Re-surveying the start-mark tags gives the same pose within 3 cm |
| 8g | Report: for each sighting, the error of `/odom_fused`, `/lio_odom` and AMCL against the tag; drift per revisit; arrival error from the tag nearest the goal | `bin/report.py` | Synthetic run with a known drift gives that drift back |

The baseline (task 7) runs after 8g, so it has tag numbers from the start.

## Phase 2: Cheap Jetson cuts (depends on task 7)

Each task: one change, standard run, compare with the baseline, keep or
revert. Robot lying down while the change is made.

| # | Task | Files | Check |
|---|---|---|---|
| 9 | `sensor_checker`: find which service starts it and whether anything reads its output; stop it, then disable it (user's OK, vendor service) | none, or a note in README §11.4 | Nothing else changes in the standard run; ~17% of a core back |
| 10 | Camera IMU off. VOA reads the robot's own odometry (our patch tunes its odometry buffer), and our `camera.launch.py` added the camera IMU on top of the vendor launch. Confirm with `ros2 topic info /camera/imu` (0 subscribers), then switch gyro and accel off | `env/camera.launch.py` | Subscriber count 0; driver CPU before and after; VOA still starts |
| 11a | Blind-zone test, robot standing (ask first): a shoe, a 15 cm box, a chair and a wheel/tyre (he touched a parked robot's tyres beside him, log section 24), each at 0.4 / 0.7 / 1.0 m in front, beside and behind. Record what `/scan`, the depth cloud and the sonars each report | none (log, with a table) | Each sensor's blind zone measured, not worked out from spec sheets |
| 11b | Depth becomes a near-field sensor only: 30 → 6 fps (the D435 offers 6/15/30 at 424x240), VOA off unless something needs it. Check `depth.py`'s 4 Hz guards, `voxel_decay` and Nav2's depth layer still catch the 11a objects at full walking speed | `env/camera.launch.py`, maybe the Nav2 patch | Driver CPU before and after; the 11a objects still stop him |
| 11c | `person.py` takes range and free path from the lidar `/scan` (360°, sees the person beside him too) instead of depth; depth stays as the near-field stop | `robot/person.py` | Follow test; the `DEPTH_VIEW_DEG` limit goes away |
| 11d | Lidar `range_min` 0.4 m: check whether anything closer is only the robot's own body; if so, lower it to bring the near zone in | Orin `lidar-scan.service` args (user's OK) | 11a objects at 0.4 m appear in `/scan` |
| 12 | Why the camera driver costs more in map mode: compare its threads (`top -H`) mapless vs map mode within one run | none (log) | A named cause, or "not found" written down |
| 12a | DONE 2026-10-14 (drop-in installed, log section 26; first real test at the next boot). Lidar driver waits for the clock: start `livox-driver` after `time-sync.target` with a timeout, so a late NTP step can't make it publish every packet (log section 22). User's OK (Orin) | Orin unit drop-in | Reboot with Wi-Fi: 10 Hz; reboot in handheld mode: driver still starts |
| 12b | Jetson boots without the desktop (`multi-user.target`); GNOME was 9-36% of a core (log section 22). User's OK | none (log) | Panel, camera, relay and Nav2 still start after a reboot |
| 13 | Panel: CPU with no viewer, one viewer, video open; anything that runs with nobody watching becomes on-demand | `robot/hmi.py` | Panel with no viewer ≤ 5% of a core |

### Phase 2b: What the first baseline found (2026-10-14, log section 23)

Routes in map mode close in doorways: the map's free floor is only the
walked strip, and real lidar marks plus AMCL's 10-15 cm error eat the
channel. These come before the phase 2 load cuts are judged, since a run
that can't finish its route can't be compared.

| # | Task | Files | Check |
|---|---|---|---|
| 13a | DONE 2026-10-14 (commit d4dd7df; robot run log section 24: 4 of 4 walks arrived, camera drops 2,671 -> 1). Global costmap in map mode on the map and the lidar only: drop `stvl_obstacle_layer` (depth) and `range_layer` (sonars) from the GLOBAL costmap, keep both in the local one | `env/nav2_map_params.py` (its `demo()` checks the plugin list) | Laptop self-check; standard run on `lab6` with the left door open reaches the hallway; Jetson load before and after |
| 13b | DONE 2026-10-14 for the speckle (env/floor_free.py; lab2/lab6 re-made from their runs; log section 25). lab2's narrow point was not the door (92 cm) but people baked into the map outside it: re-recorded as floor lab2b from a new mark in the open (log section 27): door 0.40 m clearance on the map, about 0.9 m wide. Not driven yet. Wall thinning tried and dropped (log section 25). Floor maps with real floor: free space = what the lidar saw as open floor at walking height, not only the walked strip, so doorways get their real width. Then re-make `lab2` (corridor empty, no people in it) and `lab6` | `env/walked_free.py`, `env/get_map.sh`, the Orin's slicer (user's OK) | On the new map the left doorway is as wide as measured with a tape; the planner finds routes with the doors as they are |
| 13c | `go x y`: a goal cell blocked only by old marks gets the same "forget the marks and look again" as `route_to` | `robot/nav.py`, `demos/check_motion.py` | Laptop check; on the robot a stale mark on the goal no longer refuses it |
| 13d | Bag: `/cmd_vel` recorded best effort (a QoS override file sent by `bin/run.py`) | `env/runlog_qos.yaml`, `bin/run.py` | No QoS warning when `Lite3` starts during a recording |
| 13e | Leg odometry missed a 3 m handheld walk (`/leg_odom2` 3.09 m off, fused 0.03 m): find out when the legs' odometry stops counting | none (log) | Cause written down |
| 13g | DONE 2026-10-14 (commit d591f82; refuses, does not turn the other way; on the robot -130 deg next to the parked robot refused unmoved, -30 deg turned: log section 24). The final turn on the spot (`go_to` turning to a place's heading, and the turn to face a route) checks the local costmap for his footprint at each heading it sweeps through, and stops or turns the other way when it would touch something; today only the depth camera's ±43 deg ahead is checked. He turned 130 deg into a parked robot's tyres at the end of a `go start` (log section 24) | `robot/nav.py` (or `robot/lite3.py`'s `turn`), `demos/check_motion.py` | Laptop check with a fake costmap: a turn towards a marked cell is refused or reversed; on the robot, the same return with a box at his right-rear |
| 13f | Panel E-STOP pressed by accident (20:58:02): look at how easy it is to hit, keeping the panel look the user approved | `robot/hmi_static/index.html` | The user agrees it can't be hit by a palm or a pocket |

Then the standard run again on both floors, compared with the first
baseline (`bin/report.py --compare`).

**Gate A** (end of phase 2), map mode standard run with the panel open:
- no camera messages dropped once Nav2 has been up 20 s
- controller server never misses its rate
- Jetson idle ≥ 35% during goals

Pass → phase 4 is not started; write the numbers down and stop. Fail →
phase 4.

## Phase 3: The Orin's memory growth (can run beside phase 2; each change needs the user's OK)

| # | Task | Files | Check |
|---|---|---|---|
| 14 | Read-only: `loadwatch.py` on the Orin through a 30 min Live run, standing or walking | none (log) | Memory and CPU curve for `fastlio_mapping` |
| 15 | Read-only: FAST-LIO2's config on the Orin (`pcd_save_en`, `interval`, `runtime_pos_log_enable`, `dense_publish_en`) | none (log) | Hypothesis confirmed or dropped |
| 16 | If confirmed, the user's OK, then the change; repeat task 14 | Orin config (backup `*.pre-mem`) | Memory flat over 30 min, lag under 0.1 s |

## Phase 4: Move work to the Orin (only if gate A fails)

Loose on purpose. AMCL goes first because it needs the fewest crossings:
its scan is made on the Orin already.

The motion computer (RK3588: 4 fast A76 cores + 4 small A55 cores, probably
the fastest cores on the robot) is NOT a home for Nav2: it runs the 1 kHz
locomotion controller, and a stalled gait loop means a fall. It only takes
light jobs whose data starts there anyway, and only on cores task 1 shows
`jy_exe` doesn't use, at low priority with a systemd `CPUQuota` cap:
- tag detection on the front camera (no stream crossing the network);
- talk (the audio already plays there).
Each one is checked with `jy_exe`'s CPU and behaviour, standing and walking,
before and after.

| # | Task | Notes |
|---|---|---|
| 17 | Odometry Jetson → Orin over UDP, 50 Hz, same packet style as `lio_relay` | Needed by anything that localizes on the Orin |
| 18 | AMCL + map server on the Orin (Humble), with the map from `~/maps`; the `map → odom` result goes back over UDP and `lio_relay` publishes it as TF | Nav2 stays on the Jetson; `locate.py`'s fit score moves too or reads the crossing |
| 19 | Standard run: AMCL on the Orin vs on the Jetson | Gate A again |
| 20 | Only if it still fails: write a separate plan for the whole of Nav2 on the Orin (depth as a reduced scan crossing over, `/cmd_vel` back, stale-odometry stop on both sides, Nav2 tuning moved to Humble names) | Several days plus re-testing sections 11-19 of the log |

---

## Parallel vs sequential

| Parallel group | Tasks | Why |
|---|---|---|
| A | 1, 2, 3 | Separate files, all laptop or read-only |
| B | 9-13 and 14-16 | Phase 3 is on the Orin and doesn't touch the Jetson's load |

| Sequential | Depends on | Why |
|---|---|---|
| 4 | 2, 3 | Starts both |
| 5 | 3, 4 | Reads their output |
| 6 | 4, 5 | Needs the robot and the tools |
| 8c-8g | 8a, 8b, 6 | Needs the tag code and the recording setup |
| 7 | 6, 8g | Baseline with tag numbers (a first one without tags was run 2026-10-14) |
| 9-13 | 7 | Compared against the baseline |
| 16 | 14, 15, the user's OK | Changes the Orin |
| 17-20 | Gate A fails | Not started otherwise |

---

## Testing

| What | How | Ties to |
|---|---|---|
| Load sampler | `loadwatch.py check`: parses a known `/proc/stat` pair, CPU % within range, JSON round-trips; feature_version 3.8 parse | Decision 2 |
| Bag topics | Laptop on domain 77 with `ROS_LOCALHOST_ONLY=1`, `CYCLONEDDS_URI` unset: fake publishers, record, read back every type | Decision 1, 5 |
| Run control | Laptop dry run: start, note, stop; files in place; no process left (`ps` before and after); stop works after the ssh session was killed | Decision 3 |
| Report | Built-in check on a synthetic run: known path gives known length, known spread series gives known peak, `--compare` of a run with itself gives zero | Decision 5 |
| Cost of measuring | Task 6 on the robot, lying down | Decision 1 |
| Tag pose | Synthetic rendered tag at known poses (distance 0.5-3 m, angles to 60°): error within 1 cm / 1° close up, flagged as unreliable past the limits we find | Decision 6 |
| Tag edge cases | Tag seen at a steep angle or far off (report drops sightings past a distance/angle limit, the same failure as spec edge case 13); motion blur while trotting (only sightings with low angular speed count); two tags in one frame (both used, and must agree) | Decision 6 |
| Each phase 2 change | Standard run vs baseline; revert if gate A numbers get worse | Gate A |
| Edge cases | ssh drops mid-run (detached + timeout); clock offset between computers (recorded at start); a run with no goal (report still writes load); bag file missing a topic (report says so, doesn't crash) | Decisions 3, 4 |

## Safety rules for this plan

- Changes on the robot only while he's lying down; ask first otherwise (the user's rule of 2026-10-07).
- Nothing on the Orin changes without the user's OK (rule of 2026-10-08); phase 3 reads first.
- Vendor services (`sensor_checker`) get stopped before they get disabled, and only with the user's OK.
- Topic names and Nav2 words live in files, never on a command line.
- Run `sync` on the robot after every pull.
