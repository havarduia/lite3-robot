# Plan: Regulated Pure Pursuit instead of DWB

**Spec**: none. Planned from the 2026-10-15 conversation (four speed test walks, below)
**Epic**: none
**Created**: 2026-10-15
**Status**: done (2026-10-15). Phases 1, 2 and 4 done; walks W1 and W3 passed (W3 with `cost_scaling_gain` 0.5), the user's blocking test covered W4 and the corridor part of W2; W5 and W6 skipped by the user's choice. Results in `LIDAR_ORIN_LOG.md` section 33.

Stack: Nav2 Foxy 0.4.7 on the perception computer, configured through the
vendor's `lite_nav2_mapless.yaml` (two copies on the robot: `src` and
`install`) and our `env/nav2-mapless-lite3.patch`; map mode derives its
parameters from that file (`env/nav2_map_params.py`). Same patterns as
`building-navigation.md`: self-checks instead of a test framework, backups
`*.pre-<name>` on the robot, measurements in `LIDAR_ORIN_LOG.md`, long
explanations in the README. The user commits and pushes; I pull on the robot.

## Goal

He walks up to the slow gait's **0.5 m/s** on open straights and slows down
by himself in turns, near walls and through doorways, and still gets through
the lab door both ways. Today, with DWB, he walks about 0.3 m/s everywhere.

## Why RPP: what the 2026-10-15 walks showed

Same route each time, lab2b start mark to about (2.5, 0.3) through the lab
door and back. Logs `logs/2026-10-15_speed-*`, DWB's per-critic scores in
each folder's `eval.jsonl`.

| Walk | DWB change | Speed | Result |
|---|---|---|---|
| `speed-base` | none (yesterday's) | 0.30 m/s | arrived both ways, 26 s and 22 s |
| `speed-1.6` | `prune_distance` 1.6, top 0.5 | 0.06 | stuck at the door, aborted |
| `speed-sim2` | `sim_time` 2.0, top 0.5 | 0.06 | crawled, cancelled |
| `speed-sim2b` | `sim_time` 2.0, top 1.0 | 0.41-0.52 | cut into the door frame, planner failed |

- DWB's speed is set by geometry, not by its limits: GoalDist aims only
  `prune_distance` (1.0 m) along the route, so the best trajectory travels
  1.0 m in `sim_time`: 3 s gives 0.33 m/s, 2 s gives 0.5.
- A DWB top speed of 0.5 made it crawl, for reasons not fully understood
  (it changes the speeds DWB samples).
- DWB holds one speed everywhere. Fast enough for the straights means too
  fast for the lab door: in `speed-sim2b` he stopped in the doorway on a
  cell of cost 99 and NavFn could not plan from there.

RPP does what DWB cannot: it follows the planned path and **scales its speed
down** by the path's curvature, by the cost near obstacles, and when it
approaches the goal. The Foxy build has these parameters (read from the
library on the robot): `desired_linear_vel`, `max_linear_accel/decel`,
`lookahead_dist`, `min/max_lookahead_dist`, `lookahead_time`,
`use_velocity_scaled_lookahead_dist`, `use_rotate_to_heading`,
`rotate_to_heading_min_angle`, `rotate_to_heading_angular_vel`,
`max_angular_accel`, `use_regulated_linear_velocity_scaling`,
`regulated_linear_scaling_min_radius/min_speed`,
`use_cost_regulated_linear_velocity_scaling`, `cost_scaling_dist/gain`,
`inflation_cost_scaling_factor`, `use_approach_linear_velocity_scaling`,
`min_approach_linear_velocity`, `max_allowed_time_to_collision`,
`goal_dist_tol`, `transform_tolerance`. It has **no reversing and no
angular speed limit** (newer Nav2 has both).

## What changes in behaviour, and what we accept

| Topic | DWB today | RPP | What we do |
|---|---|---|---|
| Speed | ~0.3 m/s everywhere | up to 0.5 on straights, less in turns, near walls, at the goal | the point of the change |
| An obstacle on the path | steers round it inside the local costmap | **stops** before it (collision check over `max_allowed_time_to_collision`) and waits; the planner routes round it only if the **global** costmap sees it | see "Obstacles only the camera or sonars see" |
| Turning to the path | critics (PathAlign/GoalAlign), the 2026-10-14 no-spin fix | `use_rotate_to_heading`: turns on the spot when the path is more than `rotate_to_heading_min_angle` off his heading | the spin fix no longer applies; test W5 |
| Sideways | `max_vel_y` 0 | never (pure pursuit has no y) | nothing |
| Final heading | `yaw_goal_tolerance` 3.14, `turn()` does it | unchanged (goal checker is separate) | nothing |
| Turn rate | `max_vel_theta` 0.8 | **no cap**: wz = v × curvature | measure the largest wz in W1-W2; curvature scaling should keep it low; if not, lower `desired_linear_vel` or raise `regulated_linear_scaling_min_radius` |
| Person following (`follow_nav`), mapless mode | DWB | RPP too: one parameter file serves both modes | test W6 |

### Obstacles only the camera or sonars see

In map mode the global costmap is map + lidar only (13a: the depth camera's
15 s memory closed open doorways). So something only the camera or a sonar
sees (low things under the lidar's cut, glass) makes RPP stop and the
planner keeps sending him the same way: the progress checker then fails
after 10 s and the goal aborts. With DWB he would have steered round it.
Accepted for the first version: stopping is the safe failure, and people
move. If W4 shows it matters, add the sonars (not the camera) to the global
costmap as a follow-up; that is a separate decision.

## Starting values

Sit in the `FollowPath` block of `lite_nav2_mapless.yaml`; the DWB block is
kept under another name (`FollowPathDWB`, not listed in `controller_plugins`)
so going back is one line plus a reload.

| Parameter | Value | Why |
|---|---|---|
| `plugin` | `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` | |
| `desired_linear_vel` | 0.5 | the slow gait's top (user, 2026-10-15) |
| `max_linear_accel` / `max_linear_decel` | 0.5 / 0.8 | gentler than DWB's 1.0 up; stopping may be firmer |
| `lookahead_dist`, `min_`, `max_` | 0.6, 0.3, 0.9 | short enough to follow the doorway turn |
| `use_velocity_scaled_lookahead_dist`, `lookahead_time` | true, 1.5 | 0.5 m/s × 1.5 s = 0.75 m on straights, near the minimum when slow |
| `use_rotate_to_heading`, `rotate_to_heading_min_angle`, `rotate_to_heading_angular_vel` | true, 0.785, 0.4 | 45 deg, same rate as `lite3.turn()` |
| `max_angular_accel` | 0.8 | DWB's `acc_lim_theta` |
| `use_regulated_linear_velocity_scaling`, `regulated_linear_scaling_min_radius`, `_min_speed` | true, 0.9, 0.15 | slow in the doorway turn, never slower than he walks smoothly |
| `use_cost_regulated_linear_velocity_scaling`, `cost_scaling_dist`, `cost_scaling_gain` | true, 0.6, 1.0 | slow when walls are within 0.6 m: the lab door |
| `inflation_cost_scaling_factor` | 8.0 | **must equal** the local costmap's inflation `cost_scaling_factor` (8.0), or the cost scaling misjudges distances; a self-check guards it (task 3) |
| `use_approach_linear_velocity_scaling`, `min_approach_linear_velocity` | true, 0.05 | slow into the goal, like the 2026-10-08 wiggle fix wanted |
| `max_allowed_time_to_collision` | 1.0 | stop 1 s (0.5 m at full speed) before a costmap obstacle; his sonar stop is at 0.6 m |
| `goal_dist_tol` | 0.15 | same as the goal checker |
| `transform_tolerance` | 0.2 | as DWB |

## Files

| File | Where | What changes |
|---|---|---|
| `lite_nav2_mapless.yaml` (src + install) | robot, `~/lite_cog_ros2/nav/.../config/` | `FollowPath` becomes RPP; DWB block renamed `FollowPathDWB`; backups `*.pre-rpp` |
| `env/nav2-mapless-lite3.patch` | repo | regenerated from the vendor file (reverse the old patch on `*.pre-rpp`), checked to reproduce both robot files |
| `env/nav2_map_params.py` | repo | `check`: when `FollowPath` is RPP, its `inflation_cost_scaling_factor` equals the local inflation layer's `cost_scaling_factor`; `map_params()` raises if not |
| `env/runlog_topics.txt` | repo | `/lookahead_point` added (RPP's aim point, a light PointStamped) |
| `bin/report.py` | repo | speed lines in the summary: commanded vx p50/p90/max, share of time at 0.45 m/s or more, largest wz; `--compare` gets them |
| `LIDAR_ORIN_LOG.md` | repo | section 33: the 2026-10-15 speed walks (table above) and the RPP results |
| `README.md` | repo | patch row in the env table; the Nav2 section says RPP and why |

No robot Python changes: nothing in `robot/` depends on DWB. `nav.py`'s
Nav2 detection matches the controller's process name, which RPP keeps.

## Tasks

### Phase 1: tools first (laptop, no robot)

| # | Task | Files | Check |
|---|---|---|---|
| 1 | Speed lines in the report and in `--compare` | `bin/report.py` | its built-in check run extended with known speeds; run on `speed-base` and `speed-sim2b` gives the numbers measured on 2026-10-15 (0.30 / 0.36 max; 0.52 max) |
| 2 | Record `/lookahead_point` | `env/runlog_topics.txt` | `bin/run.py start x --local` dry run lists it |
| 3 | RPP self-check in the map-params builder | `env/nav2_map_params.py` | `python3 env/nav2_map_params.py check` passes with an RPP block, and fails on a mismatched `inflation_cost_scaling_factor` |

Parallel: 1, 2, 3 touch different files.

### Phase 2: switch on the robot (he lies down; the user reloads)

| # | Task | Files | Check |
|---|---|---|---|
| 4 | Back up both YAML copies (`*.pre-rpp`), rename the DWB block `FollowPathDWB`, add the RPP block with the values above | robot YAML ×2 | both copies identical; `yaml.safe_load` finds `FollowPath.plugin` = RPP; `env/nav2_map_params.py` builds `/tmp/nav2_map.yaml` without error |
| 5 | Regenerate the patch, prove it reproduces the robot | `env/nav2-mapless-lite3.patch` | vendor + patch = robot files, byte for byte (`cmp`) |
| 6 | User: Load lab2b; I read the live parameters (from a script file: no Nav2 word on a command line) | none | `FollowPath.plugin` and `desired_linear_vel` live; `controller_server` up with no ERROR in `nav2.log` |

Sequential: 4 → 5 → 6.

### Phase 3: walks (the user says "go" to each; `bin/run.py` records each)

| # | Walk | Pass |
|---|---|---|
| W1 | `speed-base` route: start mark → about (2.5, 0.3) → back | arrives both ways; faster than 26 s / 22 s; commanded vx reaches 0.45+ on the open stretch and drops in the doorway; no planner failures; arrival at the mark within the 3-16 cm seen so far |
| W2 | the long corridor leg (corridor point (2.6, -2.0) and further along the corridor) | 0.45+ for most of the straight; largest wz under 0.8 rad/s |
| W3 | W1 twice more | same result three times (the door is where DWB failed) |
| W4 | a person stands in the path on the open stretch, then steps aside | he stops short of them (sonar 0.6 m or RPP's collision check, whichever first), then carries on; nothing touched |
| W5 | a goal behind him | turns on the spot towards the path, no full circles (the 2026-10-14 spin) |
| W6 | mapless mode: `follow_nav` after a person for 30 s | follows; no worse than before |

Compare each with `bin/report.py --compare` against `speed-base` (and
`2026-10-14_align` for W5).

### Phase 4: write it down

| # | Task | Files |
|---|---|---|
| 7 | Section 33: the four DWB speed walks and the RPP walks, numbers and verdict | `LIDAR_ORIN_LOG.md` |
| 8 | Nav2 section and patch row | `README.md` |

## Stop and go back to DWB if

- he fails the lab door in W1 or W3 more than once after tuning
  `cost_scaling_dist` / `lookahead_dist` once, or
- W4 shows he does not stop for a person, or
- W5 spins.

Going back: rename the blocks back (or restore `*.pre-rpp`), user reloads.
The DWB values stay the 2026-10-14 ones (0.3 m/s, known good).

## Open questions (answer before or during phase 3)

1. Is stopping for camera/sonar-only obstacles acceptable, or must he steer
   round them? (Affects whether the sonars join the global costmap later.)
2. Does the motion computer cap the turn rate itself? RPP has no cap: the
   largest wz in W1-W2 answers it.
3. DWPP (Dynamic Window Pure Pursuit, arXiv 2601.15006, January 2026) is a
   mode of RPP in newer Nav2 (tested on Jazzy and Rolling), NOT in Foxy
   0.4.7. It picks the command inside the velocity/acceleration limits (turn
   rate included) and keeps RPP's slow-down rules. Consider backporting its
   one function only if W1-W2 show turn rates or jerks the gait cannot
   follow. The paper reports it slower than plain pure pursuit through
   corners and tested it without obstacles.

## Gate 2

- [x] Follows the project's patterns: vendor YAML + regenerated patch, backups on the robot, self-checks, run logs and `--compare`
- [x] Every changed file listed; no new files except this plan
- [x] Tasks are small (one file each, or the two robot copies together) with a check each
- [x] Dependencies marked (phase 1 parallel; 4 → 5 → 6; walks after 6)
- [x] Tests: tool checks (1-3), configuration checks (4-6), walks with pass criteria (W1-W6) covering the door, speed, obstacles, spinning, mapless mode
- [x] Edge cases: an obstacle the global costmap does not see, a goal behind him, the turn rate without a cap, a mismatched inflation factor
- [x] A way back at every step
