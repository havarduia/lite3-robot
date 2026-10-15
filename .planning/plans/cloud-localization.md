# Plan: 3D map localization (instead of AMCL on a slice)

**Spec**: none. Planned from the 2026-10-15 conversation (AMCL lost him on the bridge of `big_walk_to_arcade`)
**Epic**: none
**Created**: 2026-10-15
**Status**: in progress (2026-10-15: phases 0-2 built and checked on the laptop; the Orin side installed and tested end to end (load, unload, bad path, reload). The perception side (relay, locate, LOCALIZER switch) needs a push, a pull and a relay restart with him lying down. Phase 3 (walks) next; AMCL stays the default)

Stack: Python on ROS 2 Foxy (perception computer) and Humble (Orin), numpy
and scipy only on the Orin (no open3d, small_gicp or kiss_icp; PCL 1.12 is
there for C++). Same patterns as `rpp-controller.md` and
`building-navigation.md`: self-checks (`python3 -m robot.x check`), files
that run on the Orin live in `robot/` and are copied to `~/bin` there (as
`lio_relay.py` is), services in `env/`, measurements in `LIDAR_ORIN_LOG.md`,
explanations in the README, AMCL kept as the way back.

## Why

AMCL fits a 2D slice of the lidar, 1.2-2.0 m above the floor
(`/scan_walls`), to a 2D wall map. That is the method for 2D lidars. On the
bridge of `big_walk_to_arcade` the structure beside him is a parapet about
1 m high (median 183 cloud points within 1.2 m at 0.3-1.15 m, 41 at
1.2-2.0 m) with an open area above it, so the slice has little to fit and he
was lost (2026-10-15, cause not recorded; LIDAR_ORIN_LOG section 34). Long
plain corridors have the same weakness along their length.

With a 3D lidar and LIO odometry the usual way, and the robot vendor's own
(`dr_nav2` + `hdl_localization`: NDT against a `.pcd`, with the IMU in a
UKF), is to register the live **3D** cloud against a **3D** map of the
building and give Nav2 only the map -> odom correction. Nav2 keeps planning
on the 2D floor maps it has now.

## Design

```
ORIN                                              PERCEPTION COMPUTER
FAST-LIO2 /cloud_registered, /Odometry            lio_relay.py (relay)
        |                                            |  'L' -> TF map->odom, /cloud_fix
cloud_locate.py  -- 'L' fix, ~1 Hz, UDP 8042 -->     |  /initialpose -> 'I' to the Orin
   point-to-plane ICP against the floor's cloud      |  floor loaded  -> 'M' to the Orin
   <-- 'M' which cloud, 'I' initial pose, UDP 8043 --+
                                                  locate.py: judges /cloud_fix (not AMCL + slice)
                                                  Nav2: map_server + costmaps as now, no AMCL
```

- **Where:** the Orin. The full cloud and FAST-LIO2 are there, it is about
  47% busy (perception 70-79%), and full clouds over DDS between the two
  computers are what crashed things before (Cyclone vs Fast DDS). Only a few
  numbers a second cross over, on the UDP channel `lio_relay.py` already uses.
- **What it matches:** FAST-LIO2's registered scans (already in a
  gravity-aligned frame) against the floor's own walk, the **unflattened**
  `gravity_aligned.ply` (flattening changed only z, so x and y are the 2D
  map's). 4 degrees of freedom (x, y, z, yaw): FAST-LIO2 already knows
  gravity, and a free z handles slopes.
- **How:** point-to-plane ICP from a good guess (the last fix carried on by
  FAST-LIO2's odometry), robust weights (Huber) so people and moved furniture
  count for less, voxel 0.2 m. **Degeneracy-aware**: the ICP's Hessian shows
  directions the scene does not pin down (along a plain corridor); there
  the fix keeps the odometry's value instead of drifting, and says so.
- **Output ('L' datagram):** the lidar's map pose at a scan's stamp, the
  inlier share (fitness), RMS error, which directions were weak, a 4x4
  covariance from the Hessian. The relay turns it into map -> odom at that
  stamp, exactly as it already matches LIO poses to leg poses by stamp.
- **Which cloud:** each floor gets `~/lite3_maps/<floor>/cloud.txt`, the
  path of its walk's cloud on the Orin. Sent with 'M' when a floor loads.
- **Initial pose:** "I am here" keeps sending `/initialpose` (nav.set_pose
  unchanged); the relay passes it on ('I'). From a tap (±0.3 m) and a guessed
  heading, the first fix is a small search (±1 m, ±30 deg) before ICP.
- **Way back:** `LOCALIZER=amcl` in `env/start_nav2_map.sh` starts today's
  AMCL stack unchanged. Default stays `amcl` until phase 3 passes.

### Components

| Component | Type | Purpose |
|---|---|---|
| `robot/cloudreg.py` | library (numpy, scipy) | read `.ply` and `scans.bin`, voxel grid, normals, 4-DoF point-to-plane ICP with Huber weights and degeneracy, covariance |
| `robot/cloud_locate.py` | Orin node | loads the floor's cloud on 'M', takes 'I', registers FAST-LIO2 scans ~1 Hz, sends 'L' |
| `bin/locate_replay.py` | laptop tool | localizes one recorded walk's scans against another walk's cloud, offline: fitness, error, weak directions per stretch |
| `robot/lio_relay.py` | extended | 'L' -> TF map->odom and `/cloud_fix`; `/initialpose` -> 'I'; 'M' on floor load |
| `robot/locate.py` | extended | judges from `/cloud_fix` (fitness, covariance, weak directions) when the cloud localizer runs |

### Files

| File | Where | Purpose |
|---|---|---|
| `robot/cloudreg.py` | new | registration library + `check` |
| `robot/cloud_locate.py` | new, runs on the Orin (`~/bin`) | the node |
| `env/cloud-locate.service` | new, Orin systemd | runs it, like `lio-send.service` |
| `bin/locate_replay.py` | new | offline evaluation |
| `robot/lio_relay.py` | change | 'L', 'I', 'M' datagrams; map->odom TF |
| `robot/locate.py` | change | the cloud judge beside the AMCL one |
| `env/start_nav2_map.sh`, `env/nav2_map.launch.py`, `env/nav2_map_params.py` | change | `LOCALIZER=cloud` drops AMCL, sends 'M' |
| `robot/mapmaker.py` | change | writes `cloud.txt` for new floors |
| `~/lite3_maps/<floor>/cloud.txt` | robot data | lab2b, big_walk_to_arcade, the rest as found |
| `README.md`, `LIDAR_ORIN_LOG.md` | docs | how it works; the measurements |

No panel change: the panel reads `/localized` from `locate.py` as now, and
"I am here" still sends `/initialpose`.

## Tasks

### Phase 0: does it work, offline (laptop, no robot)

| # | Task | Files | Check |
|---|---|---|---|
| 1 | Readers: `.ply`, `scans.bin` (sensor position + 10 cm cells per scan), voxel grid, normals | `robot/cloudreg.py` | `check`: a made-up room round-trips; scans.bin of `bigmap_to_arcade` reads 5,8xx scans |
| 2 | 4-DoF point-to-plane ICP, Huber weights, Hessian degeneracy, covariance | `robot/cloudreg.py` | `check`: a known offset (0.5 m, 10 deg) recovered to 2 cm / 0.5 deg in a room; in a plain corridor the along-corridor direction is reported weak and left at the guess |
| 3 | Replay tool: localize walk B's scans against walk A's cloud, starting from one hand-picked pose, carried on by B's own FAST-LIO2 poses | `bin/locate_replay.py` | runs on `live_20261015_143713` (or the walk that overlaps best) against `bigmap_to_arcade_20261015_142950` |
| 4 | **Gate A.** Report per stretch (lab, long corridor, slope corridor, bridge, arcade): fitness, RMS, how far the correction wanders, weak directions, time per scan on the laptop and on the Orin | `LIDAR_ORIN_LOG.md` | pass: no jump over 0.3 m on the bridge or anywhere; under 0.5 s a scan on the Orin |

Gate A fails: try PCL's NDT or GICP in a small C++ node on the Orin (PCL
1.12 is installed), or the vendor's `hdl_localization`, before giving up.
The rest of the plan stays the same: only the matcher changes.

Parallel: 1 then 2 (same file); 3 needs 1-2.

### Phase 1: the Orin node (he can lie down)

| # | Task | Files | Check |
|---|---|---|---|
| 5 | Node: 'M' loads a cloud (voxel 0.2 m, normals, KD-tree; time it), 'I' sets the guess and searches ±1 m / ±30 deg, then ICP on `/cloud_registered` ~1 Hz; sends 'L' | `robot/cloud_locate.py` | `check` with a pretend FAST-LIO2 and a made-up cloud: a fix comes back, a wrong 'I' 0.8 m off is corrected |
| 6 | Service; install on the Orin; load measured | `env/cloud-locate.service` | Orin load with it running, him standing: under 60% busy |

Phase 1 as built (differs from the design above in these points):
- 'L' carries **base_link's pose on the map** (x, y, z, yaw) at a FAST-LIO2 stamp, not a correction,
  so the relay needs nothing of FAST-LIO2's frames: map->odom = that pose against /odom_fused at the stamp.
  Also fitness, wall fitness, rmse, weak directions, variances x / y / yaw and a state (ok, no map,
  no FAST-LIO2, tilted, searching, refused). `cloud_locate.FIX` is the format.
- FAST-LIO2's frame is levelled with its own gravity (`/lio/gravity`), as lio_live.py levels it with
  the first second's; a start tilted over 1.5 deg (the Orin's live state) gives no fixes.
- A guess's height is the map's floor under it plus 0.46 m, so "I am here" works on any level.
- A FAST-LIO2 restart (/Odometry quiet 1 s) re-places him from his last map pose.
- Tracked fixes are smoothed (30% of the way each), searches jump.

### Phase 2: the perception side (he can lie down)

| # | Task | Files | Check |
|---|---|---|---|
| 7 | Relay: 'L' -> map->odom TF (pose at the stamp, through the mount transform) and `/cloud_fix`; `/initialpose` -> 'I'; 'M' command | `robot/lio_relay.py` | its `demo()`: a fix and an odometry history give the map->odom expected; clock offset as for LIO poses |
| 8 | locate.py: the cloud judge (fitness and RMS from the fix, spread from its covariance; weak directions do not make him lost while the odometry has carried him less than 15 m along them) | `robot/locate.py` | `check`: localized / too wide / does not fit / weak-but-fine, each from a made-up fix |
| 9 | `LOCALIZER=cloud`: no AMCL node, map_server as now, 'M' sent on load | `env/start_nav2_map.sh`, `env/nav2_map.launch.py`, `env/nav2_map_params.py` | `nav2_map_params.py check`; Nav2 starts with only map_server + the relay's TF (him lying down) |
| 10 | `cloud.txt`: written by mapmaker for new floors; written by hand for lab2b and big_walk_to_arcade | `robot/mapmaker.py` | `python3 -m robot.mapmaker check` |
| 11 | Runs record `/cloud_fix` | `env/runlog_topics.txt`, `bin/report.py` | report shows fitness and weak directions per goal |

Phase 2 as built:
- `lio_relay.py`: the 'L'/'I' wire format lives here (both computers share it; no scipy on the perception
  side). `CloudFix` turns each 'L' into map -> odom against /odom_fused at the fix's moment; published as
  TF (10 Hz) and on `/cloud_fix`; nothing while the Orin has no map. /initialpose is forwarded as 'I'.
  `python3 -m robot.lio_relay map <run folder or .ply | "">` loads or unloads on the Orin.
- `locate.py <floor> cloud`: `CloudJudge` - lost after 5 s with no accepted fix, walls under 0.5, too wide
  where nothing is weak, or 15 m walked since a fix that pinned every direction. `nav.floor` now reads the
  word after `robot.locate` (it took the last word, which would have been "cloud").
- `LOCALIZER=cloud python3 -m robot.hmi`: start_nav2_map.sh sends the floor's `cloud.txt` and starts locate
  in cloud mode; the launch leaves AMCL out. Default `amcl` sends an empty 'M' (unload).
- `cloud.txt` written by mapmaker for new floors; by hand for lab2b and big_walk_to_arcade.
- Runs record `/cloud_fix`; the report has a "3D localizer" line.

### Phase 3: walks (the user says "go" to each; `bin/run.py` records each)

| # | Walk | Pass |
|---|---|---|
| W1 | standing at the lab2b start mark: I am here from a tap, then from a tap 0.5 m and 20 deg off | localized within 5 s both times, pose within 0.1 m of the tape |
| W2 | lab2b out and back (the `speed-base` route), AMCL then cloud, same day | arrives both ways; AprilTag error (`bin/report.py --tags env/tags/lab2b.json`) no worse than AMCL's |
| W3 | big_walk_to_arcade: lab -> over the bridge -> arcade | never lost; through the bridge without a stop; arrives |
| W4 | the long corridor at y ≈ -12, out and back | weak direction reported along it, no jump when it ends |

Then `LOCALIZER=cloud` becomes the default.

### Phase 4: write it down

| # | Task | Files |
|---|---|---|
| 12 | The numbers (gate A, W1-W4) | `LIDAR_ORIN_LOG.md` |
| 13 | How localization works now; the switch back | `README.md` §7.5 |

| Sequential | Depends on | Why |
|---|---|---|
| 5 | 1, 2 | uses the library |
| 7, 8 | 5's datagram format | |
| 9 | 7 | the TF must exist before AMCL goes |
| W1-W4 | 6, 9, 10 | |

## Testing

| What | How | Covers |
|---|---|---|
| Registration math | `python3 -m robot.cloudreg check`: offsets recovered, corridor degeneracy found, Huber ignores a crowd of outliers | the matcher |
| Real data, offline | `bin/locate_replay.py` on two walks of the same building | the bridge, corridors, slope, before the robot moves |
| Orin node | `python3 -m robot.cloud_locate check` with a pretend FAST-LIO2 | 'M', 'I', 'L', a wrong initial pose |
| Relay math | `lio_relay.py` demo: map->odom from a fix | frames, stamps, clock offset |
| locate's judgement | `python3 -m robot.locate check` | the four states, weak directions |
| On the robot | W1-W4, recorded, tags as ground truth on lab2b | the real thing |

Edge cases: a tap 0.5 m and 20 deg off; a crowd around him (Huber, fitness
drops but the fix holds); a plain corridor (weak direction, odometry
carries); a door closed since mapping (outliers); the Orin rebooting (no 'L':
locate says no fix within 2 s, he stops); clocks (the relay's offset);
leaving the mapped area (fitness falls: lost).

## Risks

| Risk | What we do |
|---|---|
| Python ICP too slow on the Orin | gate A measures it; C++ PCL or hdl_localization instead |
| The map walk drifted (one 235 m walk, no loop closure) | the fix follows the map's own bends, and so does the 2D map: they stay consistent. Gate A shows the size |
| Two walks in different frames for the replay | one hand-picked starting pose, then ICP; its first fix is checked by eye |
| Orin load (FAST-LIO2, frontcam, lio_live) | measured in task 6; 1 Hz is enough because FAST-LIO2 carries the pose in between |

## Gate 2

- [x] Follows the project's patterns: Orin files in `robot/` copied to `~/bin`, a systemd service in `env/`, self-checks, the existing UDP channel, run logs and `--compare`
- [x] Every new and changed file listed, robot data included
- [x] Tasks small (one file each, three at most for the Nav2 switch), each with a check
- [x] Dependencies and order marked
- [x] Tests: library, offline real data, node, relay, judge, walks with pass criteria
- [x] Edge cases from the bridge and corridor findings covered
- [x] A way back (`LOCALIZER=amcl`) at every step
