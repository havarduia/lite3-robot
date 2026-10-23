# Environment, vendor patches and measured numbers

## The environment and our patches to vendor code

### `env/lite3_env.sh`

Source it before anything. It sets CycloneDDS, puts the repo root on
`PYTHONPATH`, and sources five ROS workspaces (ROS Foxy, RealSense, Nav2, the
vendor nav package, the vendor transfer bridge). The RealSense workspace is
needed even when Nav2 doesn't launch the camera, because the launch file looks
the package up before evaluating the condition.

Dependencies come from these ROS workspaces, not PyPI; `pyproject.toml` is
metadata only.

### Patches (`env/*.patch`)

The vendor code lives outside this repo, so our changes are kept here as
diffs against the vendor originals.

| Patch | What it changes | Why |
|---|---|---|
| `transfer-jetson2motion.patch` | the UDP↔ROS bridge | **caps what it republishes**: odometry, sonars, state and handheld at 50 Hz, joint states at 10 Hz, instead of every one of the ~160 packets a second (parameters `state_hz`, `handle_hz`, `joint_hz`; 0 = every packet; the IMU stays at 160 Hz because VOA pairs it with each point cloud). Each subscriber pays per message and together they had the Jetson at 3% idle ([Measured numbers and calibration](#measured-numbers-and-calibration)); publishes the **front** sonar (stock published only the rear); adds **battery**, error, charging to the state array (a flat battery used to be invisible: the robot just silently refused to stand); scales leg odometry x/y by **1.15**; fixes a race where every velocity reached the robot **twice** (raw and obstacle-corrected), so the obstacle avoider could never veto anything |
| `voa-lite3.patch` | vendor obstacle avoider | odometry averaging window 10 → 3 samples, the same ~60 ms now that odometry comes at 50 Hz; the idle handheld publishes zeros at ~160 Hz, which overwrote every ROS velocity; now ignored. A dead sender's last command times out after 500 ms. The config file named a node that doesn't exist, so **every parameter was silently ignored**: fixed |
| `rtsp-stream-push.patch` | front camera push (`~/rtsp_stream/push_video.sh` on the **motion** computer) | **retries**. The boot script starts the stream server and the camera push side by side; when the push got there first (2026-10-07) it failed to connect and exited for good, so the front camera had no picture until the next boot (`mediamtx` logs "no one is publishing to path 'test'"). Now it is a loop that tries again every 2 s. Backup beside the file as `push_video.sh.pre-retry` |
| `nav2-mapless-lite3.patch` | Nav2 config, and the launch file | range layer for the sonars, scan layer for the lidar, voxel decay 2→15 s, inflation 0.30→0.45 m with cost scaling 10→8, the controller is Regulated Pure Pursuit (0.5 m/s, slower in curves, near walls and into the goal, [What "mapless" means](navigation.md#what-mapless-means)) with DWB kept as `FollowPathDWB`: its `PreferForward.strafe_x` 0.3→0.05 and no sideways movement (`max/min_vel_y` 0), goal tolerance in xy left at the vendor's 0.15, footprint padding 0.10→0.05, yaw tolerance "any"; the launch no longer starts `odom_to_tf.py`: `lio_relay.py` publishes `odom -> base_link` itself ([Lidar and FAST-LIO2 odometry (robot/lio_relay.py)](lidar.md)) |
| `realsense-ros-4.58.3-lite3.patch` | camera driver | re-adds the vendor's two cloud changes to the newer driver: steady-clock timestamps (so TF lookups work) and a 5 cm PCL voxel filter |

### Camera stack

`env/camera.launch.py` runs the newer realsense-ros 4.58.3 (with a patched
kernel USB video module) as a drop-in for the vendor's old 3.2.3 fork, with
the same topic names. It also streams colour and the camera's IMU; depth +
colour at 30 fps no longer crashes the USB bus (verified 2026-09-24). Colour
is **off unless someone has the panel's RealSense view open**: `rs_stream.py`
sets the driver's `enable_color` when it starts and clears it when it stops.
The driver takes that while running and the depth cloud carries on (longest
gap between clouds 0.24 s, measured 2026-10-07). Nothing else reads colour,
and off takes the driver from 54% of a core to 48%. Depth
stays at the vendor's 424×240, which Nav2 and voa expect; colour is 1280×720,
since it is only looked at and never crosses the Wi-Fi raw. Depth is clipped
at 5 m before the voxel grid: far points make PCL's 5 cm grid overflow its
index, and it then silently passes the whole raw cloud through. The cloud's
steady-clock stamp and the voxel filter are our patch to
`pointcloud_filter.cpp`; stock 4.58.3 ignores those two parameters.

### Services

`camera()` / `voa()` in `protocol.py` start and stop the systemd units
directly via `sudo -n systemctl`, allowed for exactly those four commands by
`env/sudoers-lite3-camera`. `env/voa_ros2-override.conf` binds voa to the
camera: voa on brings the camera up; camera off takes voa down.

`env/lio_relay.service` runs `robot/lio_relay.py`, the robot's odometry
([Lidar and FAST-LIO2 odometry (robot/lio_relay.py)](lidar.md)). Its header has the install line. `env/lio-send.service` and
`env/lidar-scan.service` are our copies of two units that run on the Orin.

## Measured numbers and calibration

Everything here was measured on the robot. How and when is recorded in these
docs, next to the feature it belongs to; the code keeps only a one-line
reason beside each constant.

| Quantity | Value | Where it's used |
|---|---|---|
| Leg odometry under-count | ×1.15 (1.712 m by odometry = 1.960 m by tape) | Jetson2Motion patch |
| Turn coast after stop | 0.17 s (3.5–4.5° overshoot at 0.4 rad/s) | `TURN_COAST_S` |
| CPU of our ROS nodes | Jetson2Motion publishes odometry, IMU, state, both sonars and the handheld at 160 Hz each. Taking every message in rclpy cost `sonar_range` 68% of a core and a bare `Lite3` 35–46%. With queues of 1 read at a fixed rate: `sonar_range` 14% (20 Hz), `Lite3` 35% at 50 Hz, 25% at 25 Hz, 20% at 15 Hz. An empty `spin_once(timeout_sec=0)` costs as much as a full one, hence `take_ready()` (2026-10-06, Nav2 and the panel running, machine at 3–10% idle) | `lite3.SPIN_HZ`, `CLOUD_HZ`, `sonar_range.RATE_HZ` |
| Turn rate delivered | 0.7 × commanded, the same by odometry and gyro: 0.4→0.28, 0.8→0.56, 1.2→0.84, 1.6→1.12 rad/s. Starts ~0.25 s after the command, coasts 2°, 5°, 8°, 11° after the stop, body tilt under 2° throughout (2026-10-06, turning in place for 2 s each way) | `MAX_YAW_RATE`, `person.MAX_TURN` |
| Stand-up odometry jump | ~1.3 m in one step | `nav_start()` settle check |
| Toggle reaction time | ~25 ms; arming transition ~2 s | `ARM_GRACE` |
| Camera pitch | 20° nose down | `depth.CAM_PITCH` |
| Standing body height | 0.33 m | `depth.STAND_HEIGHT` |
| Body corner turning radius | ~0.36 m → guard 0.43 m | `TURN_SWEEP` |
| Nose ahead of body centre | ~0.33 m | stop distances |
| Sonar min / no-echo | 0.28 m / 4.5 m, accurate to ~2 cm | `sonar_range.py` |
| Sonar beam width | ~±33° | `FOV = 1.15 rad` |
| Pitch stick full scale | ~14° | `MAX_TILT_DEG` |
| Velocity failsafe | robot stops ~1.5 s after commands stop, ~8 cm coast | background for dead-man design |
| Battery below which it won't stand | ~20% | `BATTERY_REFUSE` |
| Speaker amp wake-up | needs ~800 ms lead-in | `LEAD_IN_MS` |
| E-stop mid-Nav2-goal | 0.000 m coast | `estop_nav.py` |
