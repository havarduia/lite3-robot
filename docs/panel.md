# The web control panel (`robot/hmi.py`)

```
python3 -m robot.hmi     # then open http://lite3-perception:8080
```

**Login and public mode** (2026-10-23, [PRIVACY.md](../PRIVACY.md)). Every
page and request needs a login first: one password, in
`~/.lite3_panel_password` on the perception computer, made at the panel's
first start. A login lasts 30 days on that browser, in an HttpOnly,
SameSite=Strict cookie derived from the password, so changing the password
(delete the file, restart the panel) logs everyone out. A wrong password
costs a second. `robot/rviz.py` sends the same key in an `X-Panel-Key`
header. **Public mode** (System, on at every boot: its off switch is a file
in `/tmp`) keeps camera frames away from Gemini: Look and comment is greyed
out, `talk.py` refuses to look, and an amber chip in the top bar shows while
it is off.

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
  saved places, the point he is going to and Nav2's route. The page gets
  the map as the PGM the map server reads (`/map/<floor>`) and paints it
  itself. Not there yet: a new goal while one is under way (Cancel first);
- a **hold-to-drive** joystick (and WASD) built on `steer()`: the page sends
  the stick ~10×/s and the robot halts 0.3 s after the last message (a
  "dead-man" switch: released, tab closed, Wi-Fi lost → stop). Forward stops
  at camera clearance ≤ 0.6 m, backward at rear sonar < 0.5 m. Driving is
  refused while Nav2 runs;
- **person detection** on/off (Systems): the built-in tracker of [Finding and following people](people-and-voice.md#finding-and-following-people),
  looking only. The row says whether someone is in view and on which side,
  and the box is drawn on the front camera view, which is the camera the
  tracker looks through. The switch itself only looks;
- **Walk up** and **Follow** (Navigate): `person.approach()` and
  `person.follow()` of [Finding and following people](people-and-voice.md#finding-and-following-people), stopping 0.6 m short. They switch detection
  on if it is off (with Nav2 on, Follow is `follow_nav()` and goes round
  obstacles), and walk at the Drive section's top-speed slider as it is
  when pressed (a Go does not: Nav2's speed is in its own config). One press
  of Follow lasts 30 s, the ceiling on any single
  move. Cancel and E-STOP end either within one 50 ms cycle (the `abort`
  argument of both functions), and so does the page going away, as for a Go;
- say / ask / look with a persona picker, all three in that persona's voice;
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

**Driving across eduroam access points.** Eduroam blocks devices from
reaching each other directly (tested 2026-10-15: laptop and robot on the same
subnet, no IPv4 or IPv6 reply), so the panel always goes through a Tailscale
relay (DERP Amsterdam, ~45 ms; the robot's side of eduroam's NAT also varies
its mapping per destination, so Tailscale cannot punch through). A laptop on
**his own wifi** goes direct instead (3 ms, 2026-10-19): the motion computer
routes it, also to the Orin's eduroam address.

**Who is on eduroam.** Since 2026-10-19 only the **Orin**, on its own 5 GHz
radio (RTL8822CE). `env/orin-nat.service` NATs the switch out over it, and
the perception and motion computers route through it (default via
192.168.1.5). The motion computer's one radio is left to the AP, which
needs it: it can only use one channel at a time, so with eduroam on it the
AP was pulled onto eduroam's 2.4 GHz ch 6, and moved every time eduroam
changed access point (the handheld lost ~5 s each time). Now the AP stays
on 5 GHz ch 36 (`wifi_mode.sh handheld`, but with internet and Tailscale;
eduroam there has autoconnect off, so don't use `online`/`eduroam`). The
switch, measured 2026-10-19:

| | Orin (5 GHz ch 116) | Motion (2.4 GHz ch 6, shared with the AP) |
|---|---|---|
| down / up | ~20 / 27 Mbit/s | ~30 / 60-85 Mbit/s |
| ping 1.1.1.1 (avg / max) | 7.4 / 8.4 ms | 11.2 / 78 ms |
| Gemini Live, first sound | 0.7-0.8 s | 0.6-2.1 s |

Less bandwidth (the Orin's uplink stays at 27 Mbit/s with one stream or
four, and with its power saving off), but steady, which is what the drive
pad and the voice need.

When the Orin's Wi-Fi changes access point it is back within about 1 s,
but the campus network drops our connections, and perception's and
motion's cable links never change, so their Tailscale noticed its relay was
dead only after about 60 s (measured 2026-10-15). `env/roam-rebind.service`
watches the Orin's `iw event` and runs `tailscale debug rebind` on both the
same second, which re-checks the relay within 3 s. Install it with the
commands in the file.

Not every change of access point is a quick hand-over. On 2026-10-15 the access point threw him off, he got back
on 8 s later, but without a fresh DHCP lease, and the campus network
blocked his IPv4 for 6 minutes, until `nmcli con up eduroam` by hand.
`env/eduroam-watchdog.sh` (a service on the ORIN) pings the eduroam
gateway every 2 s and does that reconnect after 3 missed pings (~6-9 s).

It also roams on purpose. On a test round on 2026-10-19 only 1 of 5 access
point changes was a clean hand-over: the rest clung to the old access point
until the gateway went quiet, then were down 13-35 s (the driver roams late,
and wpa_supplicant's background scan runs every 30 s at best). Below -70 dBm
the watchdog scans every 10 s and `wpa_cli roam`s to an eduroam access point
8 dB stronger; that move takes ~1 s with no packets lost. The thresholds are
at the top of the script. Power saving on the Orin's Wi-Fi is off
(`802-11-wireless.powersave 2` on the eduroam connection).

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
- **What he intends** is drawn on the floor: Nav2's path as a line, its goal
  as a ring with a stroke the way he will end up facing, and the person he
  sees as a column of a person's size. They come in the status message
  (`goal`, `plan`, `them`, all in odom) and go when the goal is over or the
  person is no longer seen. The person's place is one fix from the box in
  the front camera and the depth scan (`person.where()`), good to about
  half a metre, and only as fresh as the scan (1 Hz). The path is Nav2's
  `/plan` thinned to about 40 points.
- The panel also puts the goal and the person on `/intent/goal` and
  `/intent/person` (2 Hz, while there is one) so rviz and recordings get
  them: [robot/rviz.py: the live robot in rviz2 on the laptop](tools.md#robotrvizpy-the-live-robot-in-rviz2-on-the-laptop).
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
