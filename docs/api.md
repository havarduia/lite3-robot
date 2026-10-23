# The `Lite3` API, the depth camera and the sonars

## The core: `Lite3` (`robot/lite3.py`)

### Structure

```python
class Lite3(Nav, Depth):     # Nav from nav.py, Depth from depth.py
```

`Lite3` is one class assembled from three files using **mixins**: `Nav` adds
`nav_start/nav_stop/cost_ahead/goto`, `Depth` adds `scan/clearance/side_clear`.
They share one ROS node and one set of helpers.

When you create a `Lite3()`:

1. `RMW_IMPLEMENTATION` is forced to CycloneDDS **before `rclpy` is
   imported**. (With the default FastRTPS the node discovers nothing and hangs
   with no error.)
2. `rclpy.init()`, then an internal node `_Node` subscribes to everything
   interesting: state, odometry, point cloud, global costmap, both sonars, IMU
   and handheld sticks.
3. A **background thread spins the node** (`spin_once` every 50 ms). This is
   the key usability trick: callbacks just store the latest message, so
   `bot.battery`, `bot.pose`, `bot.standing` are always-current attributes.
   User code never writes a ROS callback or spin loop.
4. A Ctrl-C handler is installed that runs `estop()`.
5. It waits up to 10 s for `/robot_state_debug`; if nothing arrives it raises
   with the likely cause (transfer service down, or another program holding
   UDP port 43897).
6. It puts the robot in auto mode.

Used as a context manager (`with Lite3() as bot:`), `close()` always runs:
halt, drop the heartbeat, then tear down ROS in a specific order (spin thread
first, then action clients, then executor and node). Each step of that order
fixes a crash that actually happened (segfault, `InvalidHandle`).

### State

| Attribute | Source | Meaning |
|---|---|---|
| `state` | `/robot_state_debug` | dict: `basic, gait, policy, motion, task, need_move, zero_flag, battery, error, charging` |
| `battery` | same | percent |
| `standing` | same | `basic == 6` |
| `pose` | `/odom_fused` ([Lidar and FAST-LIO2 odometry (robot/lio_relay.py)](lidar.md)) | `(x, y, yaw)` in the odom frame |
| `attitude` | `/imu/data` | `(roll, pitch)` in degrees (quaternion → Euler in `_imu_cb`) |
| `ultrasound` | sonar topics | `(front, rear)` metres |
| `nav_running` | `pgrep bt_navigator` | is Nav2 up |
| `status()` | all of the above | one-line summary |

**`basic_state`** is the number everything depends on. Values observed on
this robot:

| basic | Meaning | Can we command it? |
|---|---|---|
| 98 | just powered on | settled, **not armed** (first command is eaten) |
| 8 | interlock dropped (no heartbeat) | settled, **not armed** |
| 9 | arming, in transition 8→9→1 | no, transitional |
| 1 | lying down, armed | yes |
| 17, 4, 5, 7 | standing up / lying down in progress | no, transitional |
| 6 | standing | yes, the only state that accepts velocity |

Toggling from a transitional state can lie the robot back down mid-move, so
the code only toggles from a *settled* lying state (`LYING = (1, 8, 98)`), and
`wait_ready()` waits for a settled state (`READY_STATES = (1, 6, 98)`).

### Posture: `stand()`, `sit()`, `tilt()`, `action()`

- **`stand()`** is idempotent even though the raw command is a toggle:
  it returns at once if already standing, refuses if the state is
  transitional, checks battery, sends the toggle, and watches `basic_state`.
  A toggle that is acted on leaves the lying set within ~25 ms. If after
  `ARM_GRACE` (2 s) the robot is *still* in a lying state, the toggle was
  swallowed by arming, so it sends it **once more**. That retry is the fix
  for "the first stand after boot does nothing".
- **`sit()`**: halt, toggle, wait for a lying state. Idempotent.
- **`upright(heartbeat=True)`** is a context manager for the whole routine:
  start the heartbeat (unless a handheld holds the interlock), wait for
  ready, stand, and sit on the way out whatever happens. The heartbeat is
  left to `close()`, so he always sits first and is disarmed second.
  `bin/tour.py`, `bin/teleop.py` and lesson 10 use it.
- **`tilt(pitch_deg)`** is a context manager that pitches the body nose-up
  while standing still (used so the front camera sees a face, not knees). It
  enters posture mode, re-sends the pitch stick at 10 Hz like the phone app
  does, and on exit levels the body, leaves posture mode and settles. Full
  scale is about 14°; *negative* stick value = nose up (verified with the
  camera).
- **`action(name)`** plays a built-in trick (`hello`, `dance`, `twist`, …)
  from `protocol.ACTIONS`, each tagged with the posture it must start from.
  Sent 3× at 1 Hz like the vendor's own examples. Only `dance` has been
  verified on this robot.
- **`auto()`** sends "leave posture mode" and then "auto mode". Leaving
  posture mode first matters: a robot left in it reports standing but ignores
  every velocity (seen 2026-09-18, probably after the app's posture control).

### Motion: `walk()`, `turn()`, `strafe()`, `steer()`

All motion goes through one loop, `_loop()`. `_run(vx, vy, wz, done, limit,
force, abort)` feeds it a fixed velocity; `steer()` one recomputed every cycle:

```
pre-checks:  battery ≥ 20%  →  standing  →  auto mode  →  have a pose
loop every 50 ms (20 Hz), until deadline (≤ 30 s HARD_TIMEOUT):
    _safety():  handheld stick pushed since we started?  → stop
                body roll/pitch > 30°?                  → stop
    publish Twist(vx, vy, wz)
    abort():    caller-specific check (obstacle, lost camera) → stop
    done():     target reached? → stop
finally:
    halt()  = 20 zero Twists over 0.4 s (best-effort QoS can drop one)
return {'reason': ..., 'start': pose, 'end': pose}
```

The **`finally: halt()`** is the guarantee: exception, Ctrl-C or normal exit,
the last thing published is zero velocity. The returned `reason` tells the
caller *why* it stopped (`'target reached'`, `'obstacle at 0.58 m'`,
`'handheld took over - stopped'`, …), because a walk that stopped short is
normal, not an error.

The tilt limit is 30° (`TILT_LIMIT_DEG`). Walking on the flat stays within a
few degrees; the robot is rated for 40° slopes, so raise it before trying one.
A non-finite velocity (NaN) is sent as zero: Python's `max`/`min` would
otherwise pass it through as the full limit.

**`walk(distance, speed=0.15, stop_distance=0.6)`**
- Distance is measured from odometry (straight-line from the start pose).
- Before moving it checks `clearance()`; if an obstacle is already inside
  `stop_distance` it refuses to start.
- While walking it re-checks clearance at 4 Hz (the point cloud walk is
  too expensive for 20 Hz) and stops if something is inside `stop_distance`,
  or if the depth stream is lost ("stopped rather than walking blind").
- Walking backwards is unguarded (the camera only faces forward) and says so.

**`turn(radians, rate=0.4)`** / **`turn_deg(deg)`**
- Relative to the heading at call time, positive = left, any size (even > 360°).
- Heading is **accumulated per sample**: each 50 ms step is wrapped to
  [-π, π] and added up. Wrapping the *total* instead saturates at π, which
  once made a 180° request spin ~409° until the timeout.
- **Coast compensation**: the robot keeps turning ~0.17 s after the zero
  command, which overshot every turn by 3.5–4.5°. So it stops
  `rate × 0.17 s` early, waits 0.5 s to settle, and reports the settled
  angle. After the fix, ±30° requests settle at 29.3–30.3°.
- **Side guard**: while turning, it checks the side it is turning toward.
  The body corner sweeps a circle of ~0.36 m radius; anything nearer than
  `TURN_SWEEP = 0.43 m` on that side stops the turn.
- **Sweep check** (2026-10-14): before turning, his body (0.7 x 0.5 m, the
  Nav2 footprint with its padding) is swept through the turn in 5° steps
  over Nav2's **local** costmap (depth, sonars and lidar; the global one if
  there is no local). If it would cover an obstacle cell it does not cover
  already, the turn is refused: `turned` 0 and a reason saying how far in
  it would touch. Added after he turned 130° on the spot, at the end of a
  `go start`, into a parked robot's tyres beside him
  (`history/LIDAR_ORIN_LOG.md` section 24). It does not offer the other way round:
  with a rectangular body, anything near enough to touch one way is swept
  the long way too. It sees only what the costmap has: low things beside
  him are under the lidar's lowest side ray (plan task 11a). With Nav2
  down there is no costmap and no sweep check.

**`strafe(distance)`**: sideways, positive = left. No obstacle guard.

All four share one 4 Hz sampler for the depth checks (`depth.sampled`), and
the camera only sees ±45°: something directly beside or behind him is
invisible to the turn guard.

**`steer(control)`**: the closed-loop version. `control()` is called every
cycle and returns `(vx, wz)` to keep going or a string to stop. Same guards,
same clamping to `MAX_SPEED = 0.6 m/s` and `MAX_YAW_RATE = 1.6 rad/s` (a
commanded rate: he delivers 0.7 of it, [Measured numbers and calibration](environment.md#measured-numbers-and-calibration); it was 0.8 until that was
measured), same
guaranteed halt. Teleop and the web joystick are built on `steer()`.

### Emergency stop

Ctrl-C alone is not a stop once Nav2 is driving: Nav2's `controller_server`
is a **separate process** publishing its own `/cmd_vel`, so zero commands from
our process just race it. `estop()` therefore does, in order:

1. Ignore further Ctrl-C (so a second press cannot abort the stop halfway).
2. Cancel any active Nav2 goal.
3. Zero-velocity burst.
4. **Kill the Nav2 stack** (the step that matters).
5. Another zero burst, now that nothing else is publishing.

`estop(disarm=True)` also stops the heartbeat (robot then ignores all network
commands). That is opt-in because what a *standing* robot does when the
interlock drops has never been observed. `demos/estop_nav.py` fires a real
SIGINT mid-goal and measures the coast; it measured **0.000 m**.

### Voice pass-through

`bot.say(text)` and `bot.play(clip)` forward to `talk.Voice`, imported
lazily so `lite3.py` still loads if the audio side is missing.

### Command line

```
python3 -m robot.lite3 status | stand | sit | walk 0.5 | turn 90 | scan |
                       nav-start | nav-stop | goto 1.5 | cost | estop | action dance
```

## Seeing: depth camera and sonars

### Depth camera → obstacle ranges (`robot/depth.py`)

The RealSense D435i publishes a point cloud (`/camera/depth/color/points`),
already thinned to one point per 5 cm cube by our camera patch. `scan()` turns
it into "nearest obstacle per direction":

1. **Frame conversion.** Points arrive in the camera *optical* frame (x right,
   y down, z forward). Convert to a camera body frame (x forward, y left,
   z up): `(zo, -xo, -yo)`.
2. **Undo the 20° pitch.** The camera is mounted nose-down by 20°
   (0.349 rad, from the vendor's launch file). Rotate about y by that angle and
   add the mounting offset (0.255 m forward, 0.072 m up) to get `base_link`
   coordinates. Forgetting the pitch makes the floor look like a wall ~0.54 m
   ahead.
3. **Height filter.** `base_link` is 0.33 m above the floor when standing.
   Keep only points between 8 cm and 60 cm above the floor: drop the floor
   itself and anything higher than the robot (table tops it can walk under).
4. **Bin by bearing.** ±45° field of view in 5° bins (18 bins); keep the
   nearest range in each. Every other point is sampled for speed.

Returns `[(bearing_deg, range_m or None), ...]`, positive bearing = left.

Derived checks:

- **`clearance(half_width=0.225)`**: nearest obstacle *in the robot's path*.
  It uses **lateral offset** `r·sin(bearing)`, not raw range, so a wall to the
  side doesn't count as "ahead". Returns forward distance `r·cos(bearing)`, or
  `inf`. Measured from body centre; the nose is ~0.33 m ahead of that.
- **`side_clear()`**: nearest obstacle in the left half and right half, for
  the turn guard.
- **`scan_text()`**: ASCII bar chart for humans.

### Sonars → Nav2 (`robot/sonar_range.py`)

The robot has front and rear ultrasonic sensors. `Jetson2Motion` publishes
them as bare numbers (`Float64`) at ~160 Hz; Nav2's range layer needs
`sensor_msgs/Range` in a TF frame. This small node:

- republishes both at 20 Hz as `/sonar/front` and `/sonar/rear`;
- publishes static transforms `base_link → sonar_front` (x +0.23 m) and
  `sonar_rear` (x −0.31 m, rotated 180°);
- stamps each `Range` with the latest odometry timestamp, because the robot's
  TF uses the steady clock and a mismatched stamp makes the transform fail;
- clamps readings: the front is cut to **0.8 m max**, the rear keeps 4.0 m.
  A reading at or past the maximum is published *as* the maximum, which the
  range layer reads as "nothing there".

Measured 2026-09-27/28 with tape and the depth camera: readings are good to
~2 cm; 0.28 m is the minimum ("something within ~0.3 m") and, seen lying
down, also "no echo", which is why `Lite3` exposes the rear reading without
using it as a guard; 4.5 (sometimes ~4.7) means no echo; a tilted surface can
deflect the pulse and read far. Objects ~1 m away at ±34° are caught
intermittently and at 27–32° steadily, hence the ±33° beam. The front sonar's
position comes from the camera-sonar offset; the rear is not measured and is
placed at the back of the body (nose 0.274 m ahead of `base_link`, body
0.61 m long).

Why cut the front? The beam was measured at ~±33° wide. Nav2's range layer
spreads a reading over the whole cone, so a chair 30° off to the side at 1 m
became a phantom wall across the aisle. The depth camera handles that range
well; the front sonar is kept for what the camera misses up close (< ~0.6 m,
glass). The rear sonar is the *only* sensor behind the robot, so it keeps its
full range.

It writes `/tmp/sonar_range.ready` on first publish, and
`start_nav2_mapless.sh` waits for that file before launching Nav2, otherwise
Nav2 complains "No range readings received" on every start.
