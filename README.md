# lite3-robot

A Python API and a set of behaviours for a **DEEP Robotics Lite3 Pro**
quadruped ("robot dog"). It runs on the robot's perception computer (Jetson
Xavier NX, Ubuntu 20.04, ROS2 Foxy, Python 3.8).

```python
from robot.lite3 import Lite3

with Lite3() as bot:
    bot.stand()
    bot.walk(1.0)              # stops by itself if the depth camera sees an obstacle
    bot.turn_deg(90)           # positive = left
    print(bot.scan_text())     # what the depth camera sees, as a bar chart
```

On top of that API: Nav2 navigation (mapless, or on a floor map), speech
through the robot's speaker, a scripted tour, keyboard
teleop and a web control panel.

## The idea in one paragraph

The robot has two computers. The **motion computer** runs the vendor's
closed controller (`jy_exe`) that balances and walks. The **perception
computer** runs ROS2, the depth camera and this repository. Out of the box
the robot ignores commands without saying why: wrong mode, lying down, low
battery, not "armed". The `Lite3` class turns each of those silent failures
into an automatic fix or a clear `Lite3Error`. It keeps the robot's state
as plain attributes (`bot.battery`, `bot.pose`, `bot.standing`). Every
motion call blocks, checks safety every 50 ms and always ends with the robot
stopped, even after a crash or Ctrl-C.

## Safety first

- **The handheld remote is the only hardware stop.** Keep it in your hand.
- `bot.heartbeat_start()` makes your program the controller: the robot then
  obeys with **no handheld at all**, and your terminal (Ctrl-C) is the stop.
- Ctrl-C in any `Lite3` program runs `estop()`, which also kills Nav2.
- `demos/shake.py`, `demos/estop_nav.py` and `demo.py ... --move` **move the robot**.

## Getting started

1. SSH to the perception computer: `ssh ysc@lite3-perception` (Tailscale),
   or `ssh -J ysc@192.168.2.1 ysc@192.168.1.103` from the robot's own wifi.
   The repository is in `~/robot` there.
2. Set up the shell. Do this in every new shell:

   ```bash
   source ~/robot/env/lite3_env.sh
   ```

3. Look, without moving anything:

   ```bash
   python3 -m robot.lite3 status        # one line: battery, posture, pose, Nav2, sonars
   python3 -m robot.lite3 scan          # the depth camera as a bar chart
   python3 demos/demo.py read           # the API tour, the lessons that do not move
   ```

4. Move, with the handheld in your hand and space around the robot:

   ```bash
   python3 -m robot.lite3 stand
   python3 -m robot.lite3 walk 0.5
   python3 -m robot.lite3 sit
   ```

5. Open the web panel at `http://lite3-perception:8080` (Tailscale) or
   `http://192.168.2.1:8080` (the robot's wifi), and log in with the password
   in `~/.lite3_panel_password` on the perception computer. The panel runs as
   `hmi.service`. Only one program may hold the robot: stop the panel
   (`sudo systemctl stop hmi`) before you run other scripts.

Your first program:

```python
from robot.lite3 import Lite3, Lite3Error

with Lite3() as bot:
    bot.heartbeat_start()        # only if no handheld is on - YOU are now the stop
    bot.wait_ready()
    bot.stand()
    try:
        r = bot.walk(2.0, speed=0.3)
        print('moved %.2f m: %s' % (r['moved'], r['reason']))
    except Lite3Error as e:
        print('refused:', e)     # the message says what to do
    finally:
        bot.sit()
```

Before and after a change to the motion code, run `python3 demos/check_motion.py`.
It tests the motion loop against a fake robot, so it runs anywhere.

## Where to read next

| To... | Read |
|---|---|
| understand the computers, the network ports and the arming interlock | [docs/hardware.md](docs/hardware.md) |
| use the `Lite3` API: state, posture, motion, e-stop, depth camera, sonars | [docs/api.md](docs/api.md) |
| navigate with Nav2, mapless or on a floor map, and save places | [docs/navigation.md](docs/navigation.md) |
| make the robot talk | [docs/voice.md](docs/voice.md) |
| run or change the web panel, cameras and wifi/eduroam networking | [docs/panel.md](docs/panel.md) |
| work with the lidar, the Orin and FAST-LIO2 odometry | [docs/lidar.md](docs/lidar.md) |
| run the tour, teleop, demos, rviz, or measure a run | [docs/tools.md](docs/tools.md) |
| set up the environment, see our vendor patches and measured numbers | [docs/environment.md](docs/environment.md) |
| know why it is built this way, FAQ, glossary | [docs/background.md](docs/background.md) |

Taking over the robot? Start with [HANDOVER.md](HANDOVER.md): the computers,
logins, and what is tied to a person's accounts. The robot films people:
[PRIVACY.md](PRIVACY.md) says what it collects and how to run it in public.

Lab notes and old plans are in [docs/history/](docs/history/README.md).

## Repository layout

```
robot/                 the library: `from robot.lite3 import Lite3`
  lite3.py             THE API: state, heartbeat, posture, tricks, motion, e-stop
  nav.py               Nav2 start/stop, costmap, goto(), floor maps, places   (part of Lite3)
  depth.py             point cloud -> obstacle ranges                          (part of Lite3)
  protocol.py          every UDP code and address; no ROS; also a raw-code CLI
  sonar_range.py       ROS node: sonars -> sensor_msgs/Range for Nav2
  talk.py              speaker: built-in clips, Piper text-to-speech (offline)
  hmi.py               web control panel server; the page is in hmi_static/
  rs_stream.py         RealSense colour -> H.264 video          (started by hmi.py)
  udp_relay.py         WebRTC video to the panel                 (started by hmi.py)
  rviz.py              the live robot in rviz2 (run it on the laptop)
  lio_relay.py         FAST-LIO2's pose from the Orin -> /odom_fused; lidar -> /scan
  locate.py            is the robot localized on the floor map? -> /localized
  places.py            named places per floor, in ~/lite3_maps/<floor>/places.json
  mapmaker.py          a floor map from one walk (the panel's Record map)
  cloud_locate.py      where he is on a floor's 3D map, from lidar scans  (runs on the ORIN)
  cloudreg.py          lidar scan to 3D map matching (ICP), used by cloud_locate.py
  loadwatch.py         CPU and memory log of one computer   (used by bin/run.py)
  frontcam.py          front camera frames during a run      (used by bin/run.py)
bin/                   things you run
  tour.py              walk or navigate a route, stop, look, talk
  teleop.py            drive from the keyboard over SSH
  run.py               record a test run on all three computers   (laptop)
  report.py            numbers and a plot from a recorded run      (laptop)
  tags.py              AprilTag sheets and survey, for ground truth (laptop)
  locate_replay.py     localize a recorded walk on another walk's map, offline
demos/                 teaching and regression scripts
  demo.py              the API in 10 lessons
  check_motion.py      the motion loop against a fake robot        (no robot needed)
  check_map.py         one live lidar scan drawn on a floor map
  shake.py             end-to-end regression test                  (MOVES THE ROBOT)
  estop_nav.py         proves e-stop works under Nav2              (MOVES THE ROBOT)
env/                   environment, launchers, services, and patches to vendor code
  lite3_env.sh         source this first: ROS workspaces, CycloneDDS, PYTHONPATH
  *.patch              our changes to vendor code (docs/environment.md)
  start_*.sh, *.launch.py, nav2_map_params.py   how Nav2 and the camera are started
  get_map.sh, floor_free.py, walked_free.py     fetch a floor map from the Orin and clean it
  *.service, *.conf, *.sh                       systemd units; each file's header says
                                                which computer it is for and how to install it
  tags/                surveyed AprilTag positions per floor
urdf/                  the Lite3 model and meshes for rviz and the panel
docs/                  the documentation; docs/history/ has lab notes
```

`robot/__init__.py` re-exports nothing on purpose: `robot.talk` and
`robot.protocol` must import without `rclpy`.

## Things that will bite you

- **`realsense_ros2.service` does not come back after a reboot.** Start it
  with `python3 -m robot.protocol camera on`, or anything that uses depth
  raises "no point cloud".
- **Only one program may hold the robot at a time.** The panel holds a
  `Lite3` with the heartbeat running.
- **`Lite3()` times out waiting for state?** Usually `transfer_ros2` and the
  vendor's `Lite_motion` are fighting over UDP 43897.
- **Stand before you start Nav2**, and let the pose settle. `nav_start()`
  enforces this.
- **Touching a handheld stick** puts the robot in manual mode. Motion calls
  stop with "handheld took over".
- **No odometry, no motion.** If `lio_relay.service` is down, motion calls
  refuse to start (docs/lidar.md).
- Below **~0.2 m/s** the gait shuffles and drifts sideways.
- The perception computer's clock is skewed: file timestamps are meaningless.
- **Metal-grid fences fool the depth camera.** On the bridge (2026-10-28,
  `logs/2026-10-28_bridge-depth`), the D435 on its default preset reported points
  0.4-0.7 m ahead at lens height, where the lidar saw 1.3-2.6 m. They stayed in the
  costmap for 15 s, he walked into them, and RPP held him at 0.15 m/s for 86% of
  the crossing (0.50 off the bridge). `env/camera.launch.py` now uses the High
  Accuracy preset (3), which keeps about 40% of the points (3900 -> 1600 per cloud).
  Not yet re-tested on the bridge.

## Known limitations

- **Software e-stop only.** The handheld and the power switch are the only
  hardware stops.
- **Blind spots.** The depth camera sees ±45° ahead only. Walking backwards
  with `walk()` and all of `teleop.py` have no obstacle guard. The lidar
  sees round him, but not low things right beside him.
- **Map mode is young.** It works on the floors it was tested on
  (docs/navigation.md); mapless goals are relative to odometry, which drifts.
- **`estop(disarm=True)`** while standing is untested.
- Of the built-in tricks, only `dance` has been run on this robot.
