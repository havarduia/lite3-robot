# Background: design principles, FAQ, history, glossary

## Design principles

These are the ideas behind the code; useful if asked "why is it built like this?"

1. **Turn silent failures into loud ones.** The robot's worst habit is
   ignoring commands without saying why. Every known case is either handled
   automatically (auto mode, posture mode exit, swallowed arming toggle) or
   raises `Lite3Error` with a message that says what to do. `demo.py` lesson 4
   calls the refusals "the docs".
2. **Guarantee the stop.** Every motion call ends in `finally: halt()`. The
   e-stop kills the *other* publisher (Nav2), not just our own. Remote driving
   uses dead-man timeouts, and a Go from the web panel is cancelled when no
   page is left to stop it. Shutdown order is always "stop moving, then
   disarm".
3. **State as attributes, not callbacks.** A background spin thread keeps the
   latest message of each topic, so behaviours read like ordinary sequential
   Python.
4. **Blocking calls that report what happened.** `walk()` returns how far it
   actually moved and why it stopped, because stopping short for an obstacle
   is normal operation.
5. **Measure, don't guess.** Constants come from tape measures, camera checks
   and logs; the docs record the measurement and date.
6. **Keep ROS optional where possible.** `protocol.py` and `talk.py` don't
   import `rclpy`; `lite3.py` imports `talk` lazily.
7. **Change vendor code minimally, and keep the diff.** Every vendor change is
   a patch file in `env/` with the reasoning inside.

## Questions you are likely to be asked

**Why not just use ROS directly?**
You can, but you'd have to rediscover every silent failure: wrong DDS
implementation, not in auto mode, not standing, low battery, posture mode,
unarmed robot, Nav2 outliving your process. `Lite3` encodes all of that once.

**How does it avoid obstacles?**
Three layers. (1) `walk()`/`turn()` check the depth camera at 4 Hz and stop.
(2) Nav2 plans around obstacles in a costmap built from the depth camera
(3D voxels that fade after 15 s), the two sonars and the lidar's flat scan
([navigation.md](navigation.md#what-mapless-means)). (3) Optionally, the
vendor's `voa` node corrects velocities in real time (we fixed it so it
actually can).

**How do you convert the point cloud into obstacles?**
Transform from the camera's optical frame to the robot body frame, correcting
for the 20° downward tilt; keep points 8–60 cm above the floor; bin by bearing;
take the nearest per bin. "In the path" is judged by lateral offset against
the robot's half-width.

**How accurate is odometry?**
Raw leg odometry under-counted by 15%; after scaling, walks agree with tape.
Turns are accurate to about ±1° after coast compensation. Leg odometry
alone drifts over long runs; since 2026-10-08 FAST-LIO2 on the Orin corrects
it into `/odom_fused` (a 10.4 m walked loop closed to 0.05 m, against 0.21 m
by the legs alone; [lidar.md](lidar.md)).

**How does person following work?**
The robot's own NPU detector gives bounding boxes over UDP. A proportional
controller turns to centre the person, walks when centred, slows near the
target, and uses the depth camera for the actual stopping distance.

**What happens if the program crashes or Wi-Fi drops?**
Motion calls always publish zero velocity on exit. Remote driving (teleop,
web) uses dead-man timeouts of 0.3–0.5 s, and a Go started from the web panel
is cancelled 3 s after its last page disconnects. Beyond that, the robot's
own failsafe stops it ~1.5 s after velocity commands stop arriving.

**Why does the robot talk only when standing still?**
The motors are too loud to hear the speaker while walking.

**Why Gemini Live for the voice?**
Live keeps the conversation's context and answers in speech, and since
2026-10-07 that speech is what you hear: sound begins 1.0-1.6 s after
asking. Piper made each sentence itself before that, which put about 3 s
before the first sound. Piper is now the fallback when the session cannot
speak ([people-and-voice.md](people-and-voice.md#talker-gemini-answers-in-its-own-voice)).

**What did you change in vendor code?**
Five patches ([Patches (env/*.patch)](environment.md#patches-envpatch)); the most important fixed a bug where every velocity
command reached the robot twice, which made the vendor obstacle avoider
useless, and one where its configuration was silently never loaded.

## History

| Date | Change |
|---|---|
| 2026-09-14 | Loose scripts in `~` replaced by one API (`lite3.py`); old scripts archived |
| 2026-09-15 | Initial git import; package structure; costmap-settle fix; heartbeat in `shake.py` |
| 2026-09-18 | Posture/tilt, `steer()`, side-check on turns, person following, Piper TTS, Gemini Live, personas |
| 2026-09-21 | Arming fix in `stand()`; speaker lead-in fix |
| 2026-09-24 | `lite3.py` split into `protocol`/`nav`/`depth`; tricks; voice+rocky merged into `talk.py` |
| 2026-09-25 | New camera stack (realsense-ros 4.58), sonars, safety stops, voa fixes |
| 2026-09-27 | Camera independent of voa; Nav2 config tuned and recorded as a patch |
| 2026-09-28 | Odometry ×1.15; turn coast fix; sonars in Nav2; web HMI |

The pre-git history (`.bak` files and the superseded loose scripts) was kept
in `archive/` until commit `c4cfb84`, and `robot/run_robot.py` (`Mission`, a
scripted-run wrapper nothing used) with it: `git show c4cfb84:archive/backups/README.md`.

## Glossary

| Term | Meaning |
|---|---|
| **jy_exe** | the vendor's closed-source locomotion controller on the motion computer |
| **basic_state** | `jy_exe`'s posture/state number (6 = standing; see [State](api.md#state)) |
| **heartbeat / interlock** | keepalive packet without which `jy_exe` ignores commands |
| **transfer_ros2 / Jetson2Motion** | vendor bridge between the robot's UDP protocol and ROS2 |
| **voa** | vendor "visual obstacle avoidance" node: corrects `/cmd_vel` → `/cmd_vel_corrected` |
| **Twist / `/cmd_vel`** | ROS velocity message / topic: `linear.x` forward, `linear.y` left, `angular.z` turn |
| **odom frame** | coordinate frame fixed where odometry started; drifts over time |
| **base_link** | coordinate frame attached to the robot body centre |
| **TF** | ROS's system for transforms between frames |
| **Nav2** | the standard ROS2 navigation stack |
| **costmap** | grid of traversal costs (0 free … 99 lethal) Nav2 plans on |
| **STVL** | Spatio-Temporal Voxel Layer: costmap layer of decaying 3D voxels |
| **inflation** | costmap layer that adds cost near obstacles, keeping the robot away from them |
| **NavFn / RPP / DWB** | Nav2's global path planner / the local controller since 2026-10-15 (Regulated Pure Pursuit) / the one before it |
| **CycloneDDS** | the DDS middleware ROS2 uses here (the default FastRTPS doesn't work on this robot) |
| **RMW** | ROS middleware interface; selects the DDS implementation |
| **Piper** | offline neural text-to-speech |
| **Gemini Live** | Google's streaming WebSocket model API |
| **MJPEG** | video as a stream of JPEG images; plays in any `<img>` tag |
| **Tailscale** | VPN; the HMI is only reachable through it |
| **dead-man switch** | control that stops the machine when the operator stops actively holding it |
