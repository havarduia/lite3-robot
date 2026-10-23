# Navigation with Nav2

`robot/nav.py`, plus the vendor's `dr_nav2_mapless` package with our config
patch (`env/nav2-mapless-lite3.patch`).

## What "mapless" means

There is no pre-built map and no SLAM. Everything is in the **odom frame**
(leg odometry), and the costmaps are built live from sensors:

| | Global costmap | Local costmap |
|---|---|---|
| Size | 30 × 30 m | 8 × 8 m (rolling) |
| Resolution | 10 cm | 10 cm |
| Layers | STVL (depth camera) → range layer (sonars) → scan layer (lidar) → inflation | same |

- **STVL** (Spatio-Temporal Voxel Layer) stores depth-camera points in a 3D
  voxel grid that **decays** over time. We raised decay from 2 s to **15 s**:
  at 2 s, obstacles beside the robot (out of the camera's narrow view) were
  forgotten and it scraped a chair while turning. 5–15 s is the range the
  STVL author recommends
  ([STVL README](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)).
- **Range layer**: the sonars ([Sonars → Nav2 (robot/sonar_range.py)](api.md#sonars--nav2-robotsonar_rangepy)).
- **Scan layer** (2026-10-08): the lidar, as the flat `/scan` of [Lidar and FAST-LIO2 odometry (robot/lio_relay.py)](lidar.md),
  in both costmaps (`nav2_costmap_2d::ObstacleLayer`, marking and
  clearing). It is what lets Nav2 see beside and behind him; the depth
  camera only looks ahead. Used out to 3 m, like the camera: the floor
  is only about 0.19 m under the scan's lower edge and a pitching body
  tips far beams into it. Behind him the lidar is tilted up, so it only
  catches things taller than itself there. Standing between tables,
  402 of 403 scan returns within 2.5 m sat on an obstacle cell. Trap:
  the per-source `max_obstacle_height` defaults to 0.0, which throws
  every point away without a word in the log; it is set to 2.0.
- **Inflation**: 0.45 m (was 0.30; at 0.30 the planner failed next to box
  corners in 2 of 3 runs). `cost_scaling_factor` 8.0 since 2026-10-08. It
  was 3.0, at which the cost barely fades: 54 or more out of 99 across
  the whole 0.45 m, then zero. Between two tables that left no cheap lane
  down the middle, and with a table 0.5 m to his left he curved 39° to
  the right on a clear 1.5 m goal and looked nervous doing it. At 8.0 the
  cost falls to about 20 at the edge of the zone (calculated; a profile
  read 0 to 29 down a clear aisle). It did NOT stop the turning: the same
  goal at 8.0 again ended 40° to the right. That was `strafe_x`, below.
- **`PreferForward.strafe_x`**: 0.05 since 2026-10-08, was 0.3. DWB's
  PreferForward rule treats "forward slower than `strafe_x` and turning
  slower than `strafe_theta` (0.3)" as strafing and penalises it. At 0.3
  that hit every approach: as soon as he slowed below 0.30 m/s for the
  goal, going straight was penalised and turning at just over 0.3 rad/s
  was the cheapest thing left, either way. Recorded on `/cmd_vel`: the
  turn command flipped sign 6 times in the last 3.5 s of a 1.5 m goal
  (he looked nervous and ended 5° to 40° off). At 0.05: no flips, largest
  turn command 0.06 rad/s, final heading +2°; in the aisle between two
  tables, where it had ended 40° off, 0.10 rad/s and -1°. Adding PathAlign and
  GoalAlign to the critics was tried first and changed nothing.
- **No sideways movement** (2026-10-14): `min_vel_y` and `max_vel_y` 0,
  `vy_samples` 1. The vendor's limits were lopsided (0.2 m/s to the left,
  0.02 to the right), and with `strafe_x` at 0.05 nothing discouraged
  sliding: on a return to the start 51% of the commands were more
  sideways than forward, and he slid left into a table leg that no sensor
  sees (lidar too close and too thin, depth and front sonar too far to
  the side). He now moves where his sensors look and turns in place
  instead. AMCL keeps its sideways-slip model: legs do slip.
- **Controller: Regulated Pure Pursuit** since 2026-10-15 (`FollowPath`;
  DWB is kept beside it as `FollowPathDWB`, and the two bullets above are
  its tuning). DWB walked about 0.3 m/s everywhere: its GoalDist critic
  aims only `prune_distance` (1.0 m) along the route, so the best
  trajectory covers 1.0 m in `sim_time` (3 s). Made faster (`sim_time`
  2 s) it held that speed into the lab door and stopped in the doorway
  where the planner could not start. RPP follows the planned route at up
  to `desired_linear_vel` 0.5 m/s (the slow gait's top) and slows by
  itself: in curves tighter than 0.9 m, within the walls' inflation
  (`cost_scaling_gain` 0.5: about 0.15 m/s through the lab door) and into
  the goal. It turns on the spot first when the route is more than 45°
  off his heading. It never steers round an obstacle: it stops before one
  (`max_allowed_time_to_collision` 1 s ahead) and the planner, which
  replans every second on the map and the lidar, routes round it; what
  only the camera or a sonar sees makes him wait. Its
  `inflation_cost_scaling_factor` must equal the local inflation's
  `cost_scaling_factor`; `env/nav2_map_params.py` refuses to start map
  mode otherwise. Foxy's RPP has no turn-rate limit: 0.72 rad/s at most so
  far. The walks are in `history/LIDAR_ORIN_LOG.md` section 33.
- Planner **NavFn**. Goal tolerance 0.15 m in xy (the
  vendor's value again since 2026-10-08; it was 0.25 while the approach
  wiggle made the last stretch messy. At 0.15: stops 0.15 m from the goal,
  one second later, no stepping around it; one run) and
  **3.14 rad in yaw**, i.e. "any heading". Letting Nav2 rotate a legged robot
  at the goal made it step and drift around the goal instead of stopping, so
  `goto()` does any final heading itself with `turn()`.

## Starting and stopping Nav2 safely

`nav_start()`:

1. Refuses unless standing **and the pose has settled** (hasn't moved 2 cm in
   1.5 s). Standing up jumps leg odometry by ~1.3 m in one step; a costmap
   built across that jump shows a clear path straight into a real wall.
2. Kills any leftover stack.
3. Launches `env/start_nav2_mapless.sh` via **`setsid`**, so the whole stack
   is in its own process group.
4. Waits for `bt_navigator` and for a costmap to arrive.

`nav_stop()` kills by **process group** (SIGINT → SIGTERM → SIGKILL). Why not
`pkill`? Nav2's child processes don't carry the launch name, so killing by name
misses them; widening the pattern hits `static_transform_publisher`, which
the camera and transfer services also use, and ROS launch then takes *those*
services down. So: find process groups containing a Nav2 marker, exclude any
group containing a system-service process (`NAV_NEVER`), kill the rest.
`NAV_NEVER` matches executables, not words, and launch arguments (`foo:=bar`)
are dropped before matching: the Nav2 launch line itself contains
`launch_realsense:=false`, and a bare "realsense" once made the guard exclude
the very group it was meant to kill. A
group counts as gone once only zombies are left in it: the dead leader of a
stack this process launched stays a zombie, and waiting on it cost 8 s a stop.

## Reading the costmap and choosing goals

- `cost_at(x, y)`: cost of one cell (0 free … 99 lethal, `None` off-map).
- `cost_ahead(out_to, step, settle)`: costs along the current heading. With
  `settle=N` it re-reads until two profiles a second apart agree. This is
  needed because the costmap starts empty and, with `track_unknown_space`
  off, **unseen cells read as free**: an immediate read says "clear forever",
  you pick the furthest cell, and `goto()` then refuses it as LETHAL once
  real observations land. The giveaway is cost 0 at distance 0, the robot's
  own cell, which is never really free on a populated map.
- `nav.free_ahead(profile)`: how far such a profile stays plannable, i.e. the
  last distance before the first cell `goto()` would refuse. Free means below
  LETHAL, not "low": 50 was tried first and wrongly refused open floor between
  two tables, whose inflation spreads 50–90 over the whole gap.
  `nav.goal_ahead(profile)` is that distance minus a margin, or `None` when
  too little is left; the demos and the tour's `approach` pick goals with it.

## `goto(forward, heading_deg=None)`

1. Checks standing, battery, Nav2 running, goal ≥ 0.2 m away (a goal at his
   own position gives the planner nothing to plan and aborts; use
   `turn_deg()` to rotate in place).
2. Computes the goal `forward` metres along the current heading.
3. **Refuses a LETHAL goal cell**: NavFn cannot plan into one and would abort.
4. Sends a `NavigateToPose` action goal, stores the handle so `estop()` can
   cancel it, waits for the result.
5. If it succeeded and a heading was requested, corrects it with `turn()`.

Returns the action status: 4 = succeeded, 6 = aborted, 5 = cancelled.
`goal_send(gx, gy, gyaw)` is step 4 on its own: a goal at an odom point,
returned as soon as Nav2 accepts it, replacing any goal under way. No checks.
`goal_active()` says whether Nav2 is still working on it.
`person.follow_nav()` uses it to keep moving the goal.

`goto_cancel()`, from another thread, ends a goal and leaves Nav2 up; `estop()`
ends it and kills Nav2. Either way `goto()` returns 5 and he ends halted.

`bin/tour.py` adds a smarter goal chooser: if the requested spot is inside an
obstacle, pick the *closest free distance* on that line (Nav2 then routes
around), and an `approach` verb that stops short of the *first* obstacle.

## Map mode: a floor map, and going to a place on it

Being built (`.planning/plans/building-navigation.md`); what is here works
on the robot as of 2026-10-09, in one room: two routes of 2 m out and
back, stopping 0.11 to 0.18 m from the target.

```python
bot.floors()                    # ['lab']: floors that have a map
bot.load_floor('lab')           # Nav2 on that map; replaces whatever Nav2 was up
bot.set_pose(-0.2, 0.0, 0)      # "I am here", to within half a metre and 20 degrees
bot.localized, bot.map_pose     # True, (x, y, yaw) on the map
bot.save_place('lab door')      # where he stands; or save_place(name, x, y, yaw_deg)
bot.go_to('lab door')           # returns the action status, like goto()
bot.go_to_point(1.9, -0.3)      # a point on the map
bot.places(); bot.rename_place(old, new); bot.delete_place(name)
```

The same from the command line: `python3 -m robot.lite3 floors`,
`load lab`, `here -0.2 0 0`, `save "lab door"`, `places`, `go "lab door"`,
`go 1.9 -0.3`, `forget "lab door"`.

- **The map** is one floor from the Orin's mapping run, copied with
  `env/get_map.sh` to `~/lite3_maps/<floor>/map.{pgm,yaml}` on the
  perception computer. Record it with the Orin panel's Live run, not
  "Record & map" (history/LIDAR_ORIN_LOG.md section 17). Its origin is where the
  lidar was when the run started; `base_link` was 0.20 m behind that.
  On the way `get_map.sh` makes the cells within 0.6 m of the walked path
  free floor (`env/walked_free.py`): the slicer leaves the run's start and
  cluttered rooms unseen, and unseen is a wall here. Walls stay walls,
  except where the path crosses one: he walked through it, so it is a
  door (open, or its leaf sliced as wall), and a 0.8 m gap is opened
  there. `big_walk_to_arcade` crossed 16 such cells and had no route past
  its first door without it.
- **A walk between levels** (a ramp, a slope): the Orin splits it into
  one floor per storey, and Nav2 cannot route between two floor maps. Make
  it one floor instead: on a copy of the run, `kiss_flatten.py` (takes the
  path's height out, the slope included) then `map_walls.py --one-floor`.
  The panel's Save already asks for one floor; `big_walk_to_arcade` (a
  1.4 m drop over 27 m at 3°) was made this way (history/LIDAR_ORIN_LOG.md
  section 34).
- **What runs** (`env/start_nav2_map.sh <floor>`): the mapless stack with
  its global costmap moved to the `map` frame and the map as its first
  layer, a map server, AMCL, and `robot/locate.py`. The parameters are
  made at each start from the mapless file (`env/nav2_map_params.py`), so
  the tuning of [What "mapless" means](#what-mapless-means) is the only copy. Grey on the map (never seen) is a
  wall to the planner: routes stay on floor that was mapped as free.
- **Telling him where he is:** `here x y deg` takes the pose as a tap on
  the map, ±0.3 m. `here x y deg fit` is for a pose taken from
  `demos/check_map.py`'s best fit, already checked against the walls:
  AMCL is told ±0.1 m. AMCL's motion noise is 0.1 (was 0.3, set for leg
  odometry before FAST-LIO2 corrected it): at 0.3 its estimate never
  narrowed and every turn in the corridor made him "lost"
  (`history/LIDAR_ORIN_LOG.md` section 28).
- **Routes are planned on the map and the lidar only.** The global
  costmap has no depth camera and no sonars (since 2026-10-14); the local
  costmap, which steers, keeps both. With them in the global one, a depth
  mark nobody saw on the lidar (it lasts 15 s) closed an open doorway
  that the map already made narrow (`history/LIDAR_ORIN_LOG.md` section 23).
- **Two steps to start.** Nav2 does not finish starting until AMCL has
  been told where he is, so `load_floor()` returns with him not
  localized, and `set_pose()` returns when he is and Nav2 is ready.
- **Which scan.** AMCL and `locate.py` use `/scan_walls`, the lidar cut
  above the furniture ([Lidar and FAST-LIO2 odometry (robot/lio_relay.py)](lidar.md)). The costmaps keep the low `/scan`.
- **Localized or not** is one node's answer, `robot/locate.py`, on
  `/localized` twice a second: AMCL's spread must be narrow and the wall
  scan, drawn from the pose, must land on the map's walls. `go_to()`
  refuses to start without it and stops, raising, if it is lost on the
  way. Sliding along a corridor is what it cannot see.
- **Facing the route.** `go_to_point()` asks the planner for the route
  first (`route_to()`; no route, no start; before it says so it empties
  the global costmap once and asks again, because a door seen shut from
  further off than the lidar clears stays shut there) and, if it sets off more than
  60° from where he faces, turns him with `turn()` before Nav2 gets the
  goal. Left to itself this Nav2 stands for 20 s and then spins as a
  "recovery": its controller has nothing that rewards turning towards a
  path, and mapless goals were always ahead.
- **Frames.** In map mode Nav2 takes a goal's numbers as map coordinates
  whatever frame its header names, so `goal_send()` and `cost_at()` move
  an odom point onto the map themselves. `goto(forward)`, following and
  the tour keep working unchanged in either mode.
- **Places** are `~/lite3_maps/<floor>/places.json` (`robot/places.py`):
  names unique per floor whatever the case, a damaged file moved aside
  and never written over.
- `demos/check_map.py <floor> <x> <y> <deg>` draws one live scan on the
  map and finds the best-fitting pose nearby; it is how to find the pose
  to give, and the first thing to run when he will not localize.

## What he says on a map goal

A goal pressed on the panel's map (`go_point`) is spoken aloud (`talk.Voice.say`,
Piper, offline), from fixed
lines in `robot/hmi.py`'s `SAY`: "On my way." once a route is found,
"I'm here." on arrival, "I couldn't get there." when it fails or is
refused, nothing when you cancel it.

Anything in the way, a shut door included, is left to Nav2: RPP stops in
front of it and the planner routes round it, or the goal fails. Until
2026-10-16 he also stopped at doors the costmap newly marked shut and asked
for them to be opened, and asked people to let him pass (`robot/doorway.py`,
removed: it cancelled and resent Nav2's goal mid-route, which seemed to confuse Nav2,
and took people and stale marks for doors; it is in git history).
