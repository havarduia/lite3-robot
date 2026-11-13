# Review brief: Lite3 robot, code + hardware stack

You are reviewing a quadruped robot (DeepRobotics Lite3) built as a
bachelor project. **This is a read-only review. Do not edit files, commit,
install, or restart anything.** Output one report (format at the bottom).

## Read these first, in order

1. `README.md`, `HANDOVER.md` (the four computers, who runs what)
2. `docs/hardware.md`, `docs/navigation.md`, `docs/lidar.md`, `docs/panel.md`, `docs/tools.md`
3. `orin/bin/README.md`
4. `docs/history/LIDAR_ORIN_LOG.md` only when you need to know *why* something is the way it is

The code comments are short on purpose; the long explanations and
measurements are in the READMEs and docs. Before you call something a bug or
dead code, check whether the docs explain it.

## The system in one paragraph

Four computers: **perception** (Jetson Xavier NX, Ubuntu 20.04, ROS 2 Foxy,
code in `robot/`, `bin/`, `env/`), **motion** (vendor RK3588, closed
controller, we only talk to it over UDP via `robot/lite3.py`), the **Orin**
Nano (Ubuntu 22.04, ROS 2 Humble, code in `orin/`: Livox MID-360S, FAST-LIO2,
cloud localization, TTS, and the new Nav2 MPPI stack), and the lidar.
Perception and Orin are on different ROS distros, so data crosses between
them over plain UDP relays (`lio_relay`, `nav_link`, `shadow_link`,
`udp_relay`), not DDS. The web panel (`robot/hmi.py` + `robot/hmi_static/`)
is the operator UI.

## Current state (do not fight it)

- Navigation is **mid-migration**: the old Nav2 RPP stack on perception is
  the fallback; Nav2 MPPI + CMU terrainAnalysis on the Orin is being brought
  up (switch `NAV_STACK=jetson|orin`). Both stacks existing at once is
  intentional for now. Review the *seam* between them, don't propose deleting
  either.
- Perception's CPU was saturated by 160 Hz topics; some throttling and odd
  rates are deliberate fixes.
- Several hardware workarounds look strange in code but are measured: the
  lidar flooding ~2000 Hz after boot (livox-driver restart), the front-camera
  boot race, FAST-LIO2 lag, the clock wait before livox-driver
  (`env/livox-driver-wait-clock.conf`). Flag them only if the workaround is
  itself wrong, not because it looks hacky.
- `env/*.patch` are patches to vendor/third-party packages; review them as
  patches, don't suggest upstreaming.

## What I want from you

1. **Bugs** that would bite on the real robot: races at startup, unhandled
   socket/serial errors, a relay that silently drops or stales data, a stop
   path (e-stop, collision stop, teleop timeout) that can fail open, wrong
   frames or units, timestamps compared across computers without sync.
2. **Duplication and drift.** Known lead: `robot/` and `orin/bin/` both carry
   `cloud_locate.py`, `lio_relay.py`, `orin_nav.py` (they differ) and
   `cloudreg.py` (identical). Say which copy is live where, and whether
   they should be one file. `orin/bin/lio_relay.py.pre-cloud` looks like a
   leftover. Find other cases like this.
3. **Structure.** Would a different split of work between perception and
   the Orin (or fewer relays/services) be simpler or cheaper on CPU? Base it
   on what the code actually does; estimate cost, don't guess.
4. **Safety of motion commands**: trace every path that can send velocity
   to the motion computer and check each one has a timeout/stop.

Skip: style nits, type hints, test frameworks, adding abstractions.
(Part 2 covers whether to rebuild the stack; keep the findings here
about the code as it stands.) This is a small team; simpler is better.

## Rules

- No changes to files. No commands on the robot. If you need a runtime fact
  (topic rate, CPU load), write down the exact command to run and why, and
  I'll run it.
- Say "I couldn't verify this" when you couldn't. A short list of real
  findings beats a long list of guesses.

## Report format

For each finding, most important first:

- **What**: one sentence
- **Where**: `file:line`
- **Why it matters on the robot**: concrete failure scenario
- **Fix**: smallest change that solves it
- **Confidence**: verified in code / likely / needs a runtime check (+ the command)

Then a short section **Restructuring ideas**: at most 3, each with what it
removes, what it costs, and whether it should wait until the MPPI migration
is done.

## Part 2: would you rebuild it?

After the findings, step back and answer honestly: **if you were starting
today with this exact hardware, would you build it the same way?** A full
rework is on the table. "Keep it, fix the findings" is an equally good
answer if that's what the code supports. Don't recommend a rework just
because it's allowed.

Consider at least:
- **Distribution of work**: what runs on perception vs the Orin. Could the
  Orin take over everything except what is physically tied to perception
  (cameras, sonars, panel), or could perception be retired entirely?
- **One ROS distro**: what blocks moving perception to Humble (JetPack
  version, vendor packages, `env/*.patch`), and would that remove the UDP
  relays?
- **Process layout**: the number of systemd services, shell launchers and
  relays. Would fewer, bigger processes or ROS launch files be simpler?
- **Navigation**: whether the planned end state (MPPI + terrainAnalysis on
  the Orin) is the right target, given the doorway and ghost-obstacle
  problems in `docs/navigation.md`.

If you recommend a rework, give:
1. **Target architecture**: one diagram (ASCII) showing computers,
   processes and the links between them.
2. **What it removes**: files, services and relays that go away.
3. **What it risks**: the hardware workarounds that must be carried over,
   and anything you couldn't verify without the robot.
4. **Migration in steps**, each one leaving a robot that still walks and
   can be tested, with the old stack as fallback until the new one
   matches it. No big-bang switch.
5. **Rough effort** per step (days), and what to measure on the robot to
   know each step worked.
6. **Verdict**: rework now, rework after the MPPI migration, or don't.
