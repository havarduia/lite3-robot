# The web control panel (`robot/hmi.py`)

```
python3 -m robot.hmi     # then open http://lite3-perception:8080
```

**Login** (2026-10-23, [PRIVACY.md](../PRIVACY.md)). Every
page and request needs a login first: one password, in
`~/.lite3_panel_password` on the perception computer, made at the panel's
first start. A login lasts 30 days on that browser, in an HttpOnly,
SameSite=Strict cookie derived from the password, so changing the password
(delete the file, restart the panel) logs everyone out. A wrong password
costs a second. `robot/rviz.py` sends the same key in an `X-Panel-Key`
header.

An `aiohttp` server on the **Tailscale VPN address**, so nothing on the
public Wi-Fi can reach it, and on the robot's own network (`eth0`,
192.168.1.103) for a phone on **his own wifi** (`YSC-JYML-...-5G`): open
**http://192.168.2.1:8080** there. That works in every `wifi_mode.sh` mode,
`handheld` too (no internet, no Tailscale, the best link): the motion
computer forwards 192.168.2.1:8080 to the panel and UDP 8190 to its camera
relay (`env/robot-nat.sh`, installed as `/usr/local/bin/robot-nat.sh`), and
the page's video goes to whichever address it was opened at
(`video_candidate`). 192.168.2.1, not 192.168.1.103: that one is off the
phone's wifi subnet, and a phone whose wifi has no internet may send it over
mobile data. Anyone with the robot's wifi password can drive him this way.
One self-contained page (phone or laptop, no
CDN: even its typeface, B612, is embedded) with:

- live status (battery, posture, Nav2, camera, heartbeat, sonars, tilt, pose)
  pushed over a WebSocket twice per second, and the depth scan drawn as a
  top-down fan;
- the robot **in 3D**, live (see "Him, in 3D" below);
- two camera views (front wide-angle, RealSense colour) as H.264 over WebRTC,
  running **only while someone is watching** (see "Cameras" below);
- stand/sit, Nav2 on/off, camera/voa on/off, "goto N m";
- the **floor map** ([Map mode: a floor map, and going to a place on it](navigation.md#map-mode-a-floor-map-and-going-to-a-place-on-it)): pick a floor and Load, tell him where he is
  (I am here, then press the spot), then press anywhere on the map to put
  the goal there and confirm "Go here". The goal is a ring to drag and a
  knob to turn; under the map are its x, y and a heading switch ("Any way"
  sends no heading, "Facing" sends the fader's), and with the map focused
  arrows move it, [ ] turn it and Enter sends it. Pinch, the wheel or the
  + − keys zoom (Fit, Him); zoomed in, a drag pans. It shows him, the
  saved places, the point he is going to and Nav2's route. Under the map,
  **Go to** has one key per saved place: one click sends him there, facing
  the place's heading (same goal as Go here). **Save** stores a named place:
  where he stands and faces, or, with a goal ring on the map, the ring's
  point and heading ("Any way": the way he would arrive). The page gets
  the map as the PGM the map server reads (`/map/<floor>`) and paints it
  itself. Not there yet: a new goal while one is under way (Cancel first);
- a **hold-to-drive** joystick (and WASD) built on `steer()`: the page sends
  the stick ~10×/s and the robot halts 0.3 s after the last message (a
  "dead-man" switch: released, tab closed, Wi-Fi lost → stop). Forward stops
  at camera clearance ≤ 0.6 m, backward at rear sonar < 0.5 m. Driving is
  refused while Nav2 runs;
- Say it: text spoken aloud through Piper (offline), with a volume fader;
- an always-visible **E-STOP** (also the space bar) that bypasses the command
  queue and calls `Lite3.estop()` directly;
- **Cancel**, shown while a Go is under way: ends it and leaves Nav2 running.
  A Go does not outlive its operator either: 3 s after the last page went
  away it is cancelled, and if Nav2 will not let go, e-stopped.

Robot commands run in a single-worker thread pool so only one thing moves the
robot at a time; talking has its own pool so it can happen alongside. On
shutdown it sits the robot down before dropping the heartbeat.

The timings: the drive halts 0.3 s after the last stick message (`DEADMAN_S`);
a silent page is noticed within about 7.5 s (`PING_S` × 1.5); a Go is cancelled
3 s after the last page went (`GONE_S`, the page itself reconnects in 1.5 s)
and becomes an E-STOP if Nav2 has not let go 2 s later (`LET_GO_S`). A
malformed message from a page is logged and ignored. A browser button is not
a hardware stop: keep the handheld at hand. For HTTPS, run
`sudo tailscale serve --bg 8080` once and open
`https://lite3-perception.<tailnet>.ts.net`; the server also listens on
localhost for that.

**The robot's internet.** Since 2026-11-09 the **Orin** is on the campus
network `student-FoU` (WPA2 password, NetworkManager connection
`student-FoU` on the Orin, power saving off), on its own 5 GHz radio
(RTL8822CE). `env/orin-nat.service` NATs the switch out over it, and the
perception and motion computers route through it (default via
192.168.1.5). The motion computer's one radio is left to the AP on 5 GHz
ch 36 (`wifi_mode.sh handheld`), which still has internet and Tailscale
through the Orin. Eduroam was removed everywhere that day: it blocked
devices from reaching each other, so Tailscale always went through its
relay in Amsterdam (~45 ms, and dropouts on every roam). Student-FoU does
not: a laptop on it reaches the Orin directly (10.245.30.x), and Tailscale
to the perception computer goes direct, 6 ms (2026-11-09). A laptop on
**his own wifi** goes direct too (3 ms): the motion computer routes it.

**Driving across access points.** When the Orin's Wi-Fi changes access
point it is back within about 1 s, but on eduroam the campus network
dropped our connections, and perception's and motion's cable links never
change, so their Tailscale noticed only ~60 s later (measured 2026-10-15).
`env/roam-rebind.service` watches the Orin's `iw event` and runs
`tailscale debug rebind` on both the same second, which re-checks the path
within 3 s. Not yet measured whether student-FoU needs it; it does no harm.

The Orin's driver roams late: on a test round on 2026-10-19 only 1 of 5
access point changes was a clean hand-over; the rest clung to the old
access point until the gateway went quiet, then were down 13-35 s.
`env/wifi-watchdog.sh` (a service on the ORIN) pings the gateway every 2 s
and runs `nmcli con up student-FoU` after 3 missed pings (a drop once came
back without a fresh DHCP lease and the campus blocked IPv4 for 6 min).
Below -70 dBm it scans every 10 s and `wpa_cli roam`s to an access point
of the same network 8 dB stronger; that move takes ~1 s with no packets
lost. The thresholds are at the top of the script.

**Him, in 3D.** A third monitor under Cameras: the robot model posed from
his joint angles, on a floor grid he walks across, with the two sonar beams
and the depth scan of "Ahead of him" laid on the floor. Drag to look around
him, scroll to move closer; on a phone a sideways drag turns the view and an
up-or-down one still scrolls the page.
- The browser does the drawing. It reads the same `urdf/lite3.urdf` and
  small meshes as rviz (served under `/model`, 3.4 MB, once) with a short
  URDF reader in the page, and `three.js` (r160, MIT, 670 kB) from
  `hmi_static/lib/`, kept in the repo because the panel must work with no
  internet. Both are fetched only when the monitor is opened.
- The robot's part is small: `hmi.py` listens to `/joint_states` (10 Hz) and,
  only to pages with the monitor open, sends the 12 angles, the pose and the
  body height (`rviz.height()`, the same sum rviz uses) 10 times a second,
  about 150 bytes each. Sonars, tilt and scan come from the status message
  the page gets anyway. The page eases between messages so he moves, not
  jumps.
- **What he intends** is drawn on the floor: Nav2's path as a line, and its
  goal as a ring with a stroke the way he will end up facing. They come in
  the status message (`goal`, `plan`, in odom) and go when the goal is over.
  The path is Nav2's `/plan` thinned to about 40 points.
- The panel also puts the goal on `/intent/goal` (2 Hz, while there is one)
  so rviz and recordings get it: [robot/rviz.py: the live robot in rviz2 on the laptop](tools.md#robotrvizpy-the-live-robot-in-rviz2-on-the-laptop).
- Not in it: the full depth cloud and the costmaps. Those cost the robot
  real work per viewer; rviz on the laptop ([robot/rviz.py: the live robot in rviz2 on the laptop](tools.md#robotrvizpy-the-live-robot-in-rviz2-on-the-laptop)) has them.

**Cameras.** Both views come from the motion computer's mediamtx as WebRTC.
The front camera is the robot's own H.264 stream (1280×720, 30 fps). The
RealSense colour image is put there by `robot/rs_stream.py`, which runs only
while a page shows that view and for 10 s after. A browser cannot reach the
motion computer, so `hmi.py` rewrites the WebRTC answer to point at
`robot/udp_relay.py` on the Jetson, which forwards the video. If WebRTC does
not connect, the front camera falls back to MJPEG (ffmpeg, 640 px, 8 fps); the
RealSense view has no fallback.

Why they are built this way (all measured 2026-10-05, at 1280×720):

- `rs_stream.py` reads the camera's **JPEG** topic (needs
  `ros-foxy-compressed-image-transport`): raw frames do not get through ROS
  into Python fast enough, a subscriber that did nothing received 8 of 30 a
  second.
- It uses the Jetson's **hardware** H.264 encoder: libx264 ultrafast managed
  23 fps on 1.5 cores. But it decodes with `jpegdec`, not `nvjpegdec`: the
  hardware JPEG decoder kept handing out its first frame, so the video was a
  still. Baseline profile (WebRTC cannot take B-frames) with a keyframe a
  second, so a new viewer starts at once.
- Both are **processes of their own**. Encoding inside the HMI stuttered
  (22 fps with 150–200 ms gaps against a steady 29 fps). The relay inside the
  HMI shared the interpreter with the ROS threads, was starved and dropped
  packets (its receive queue was full).
