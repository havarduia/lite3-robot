# History

Lab notes and day logs, kept as they were written. They are the record of
what was measured and what was tried; the docs one level up say how things
work now.

- [LIDAR_ORIN_LOG.md](LIDAR_ORIN_LOG.md): the lidar, the Orin, FAST-LIO2,
  map mode and Nav2 tuning, by numbered section and date. Code comments
  cite it as "docs/history/LIDAR_ORIN_LOG.md section N".
- [SESSION_2026-10-09.md](SESSION_2026-10-09.md): the day map mode first worked.
- Plans and specs are in `.planning/` at the repository root.

## Old README section numbers

These notes and the plans cite sections of the old, single README
(for example "README §10.7"). This is where each one is now.

| Old README section | Now in |
|---|---|
| §1 The two-minute explanation | [README](../../README.md#the-idea-in-one-paragraph) |
| §2 The hardware and the two computers | [hardware.md](../hardware.md#the-hardware-and-the-two-computers) |
| §3 How a command reaches the legs | [hardware.md](../hardware.md#how-a-command-reaches-the-legs) |
| §4 Repository layout | [README](../../README.md#repository-layout) |
| §5 The core: `Lite3` (`robot/lite3.py`) | [api.md](../api.md#the-core-lite3-robotlite3py) |
| §5.1 Structure | [api.md](../api.md#structure) |
| §5.2 State | [api.md](../api.md#state) |
| §5.3 Posture: `stand()`, `sit()`, `tilt()`, `action()` | [api.md](../api.md#posture-stand-sit-tilt-action) |
| §5.4 Motion: `walk()`, `turn()`, `strafe()`, `steer()` | [api.md](../api.md#motion-walk-turn-strafe-steer) |
| §5.5 Emergency stop | [api.md](../api.md#emergency-stop) |
| §5.6 Voice pass-through | [api.md](../api.md#voice-pass-through) |
| §5.7 Command line | [api.md](../api.md#command-line) |
| §6 Seeing: depth camera and sonars | [api.md](../api.md#seeing-depth-camera-and-sonars) |
| §6.1 Depth camera → obstacle ranges (`robot/depth.py`) | [api.md](../api.md#depth-camera--obstacle-ranges-robotdepthpy) |
| §6.2 Sonars → Nav2 (`robot/sonar_range.py`) | [api.md](../api.md#sonars--nav2-robotsonar_rangepy) |
| §7 Navigation: mapless Nav2 | [navigation.md](../navigation.md) |
| §7.1 What "mapless" means | [navigation.md](../navigation.md#what-mapless-means) |
| §7.2 Starting and stopping Nav2 safely | [navigation.md](../navigation.md#starting-and-stopping-nav2-safely) |
| §7.3 Reading the costmap and choosing goals | [navigation.md](../navigation.md#reading-the-costmap-and-choosing-goals) |
| §7.4 `goto(forward, heading_deg=None)` | [navigation.md](../navigation.md#gotoforward-heading_degnone) |
| §7.5 Map mode: a floor map, and going to a place on it | [navigation.md](../navigation.md#map-mode-a-floor-map-and-going-to-a-place-on-it) |
| §7.6 What he says on a map goal | [navigation.md](../navigation.md#what-he-says-on-a-map-goal) |
| §8 Finding and following people | [people-and-voice.md](../people-and-voice.md#finding-and-following-people) |
| §9 Talking: speaker, TTS and Gemini | [people-and-voice.md](../people-and-voice.md#talking-speaker-tts-and-gemini) |
| §9.1 `Voice`: one sound at a time | [people-and-voice.md](../people-and-voice.md#voice-one-sound-at-a-time) |
| §9.2 `Speaker`: streaming, sentence by sentence | [people-and-voice.md](../people-and-voice.md#speaker-streaming-sentence-by-sentence) |
| §9.3 `Talker`: Gemini answers, in its own voice | [people-and-voice.md](../people-and-voice.md#talker-gemini-answers-in-its-own-voice) |
| §10 Programs you run | [tools.md](../tools.md) |
| §10.1 `bin/tour.py`: walk a route and comment on it | [tools.md](../tools.md#bintourpy-walk-a-route-and-comment-on-it) |
| §10.2 `bin/teleop.py`: keyboard driving over SSH | [tools.md](../tools.md#binteleoppy-keyboard-driving-over-ssh) |
| §10.3 `robot/hmi.py`: web control panel | [panel.md](../panel.md) |
| §10.4 `demos/` | [tools.md](../tools.md#demos) |
| §10.5 `robot/protocol.py` as a CLI | [tools.md](../tools.md#robotprotocolpy-as-a-cli) |
| §10.6 `robot/rviz.py`: the live robot in rviz2 on the laptop | [tools.md](../tools.md#robotrvizpy-the-live-robot-in-rviz2-on-the-laptop) |
| §10.7 `robot/lio_relay.py`: FAST-LIO2's pose from the Orin | [lidar.md](../lidar.md) |
| §10.8 Measuring a run: `bin/run.py`, `bin/report.py`, `bin/tags.py` | [tools.md](../tools.md#measuring-a-run-binrunpy-binreportpy-bintagspy) |
| §11 The environment and our patches to vendor code | [environment.md](../environment.md#the-environment-and-our-patches-to-vendor-code) |
| §11.1 `env/lite3_env.sh` | [environment.md](../environment.md#envlite3_envsh) |
| §11.2 Patches (`env/*.patch`) | [environment.md](../environment.md#patches-envpatch) |
| §11.3 Camera stack | [environment.md](../environment.md#camera-stack) |
| §11.4 Services | [environment.md](../environment.md#services) |
| §12 Measured numbers and calibration | [environment.md](../environment.md#measured-numbers-and-calibration) |
| §13 Design principles | [background.md](../background.md#design-principles) |
| §14 Known limitations | [README](../../README.md#known-limitations) |
| §15 Running things | [README](../../README.md#getting-started) |
| §16 Things that will bite you | [README](../../README.md#things-that-will-bite-you) |
| §17 Questions you are likely to be asked | [background.md](../background.md#questions-you-are-likely-to-be-asked) |
| §18 History | [background.md](../background.md#history) |
| §19 Glossary | [background.md](../background.md#glossary) |
