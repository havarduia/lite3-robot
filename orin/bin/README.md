# Lite3 LiDAR mapping (Jetson)

How the robot builds maps with its Livox MID-360s, live and from recordings, and why it's set up this way. Navigation on the robot PC is covered in `~/robot_pc/README.md`.

Everything runs on the Jetson (`jetson-lite3`, Tailscale `100.114.74.5`). The browser only displays results.

## Use it

Open **http://100.114.74.5:8000** (`lidar-web.service`; it starts at boot, as does `livox-driver.service`).

| Tab | What it does |
|---|---|
| **Scan** | Static scan with the robot still: dense cloud, no SLAM. |
| **Record & map** | Records `/livox/lidar` + `/livox/imu` to `~/bags/<name>_<stamp>`. On Stop, FAST-LIO2 replays the bag into a map. This takes as long as the walk did. |
| **Live** | FAST-LIO2 on the live sensor. The map, path and robot arrow refresh every 0.2 s. On Stop & save, the map goes to the Library and to Navigate's floor maps. |
| **Navigate** | Sends a floor map to Nav2 on the robot PC. |

Live tab, in order:

1. **Name the run** (e.g. `two-floors-stairs-fast`). It's saved as `<name>_<date>_<time>`. Spaces become `-`, other symbols are dropped. You can type it during the run.
2. **Start with the robot standing still.** The first second of IMU data defines "down", and FAST-LIO initialises from the same samples.
3. **Tick "One floor"** before Stop & save if the walk stayed on one level. It removes the remaining height drift and draws one wall plan. Leave it off for walks that change floors.
4. To check the result later, run **Record** at the same time. A bag can be replayed with different settings; a live run cannot.

Losing the connection does not stop a live run. Mapping continues on the Jetson and saves when Stop is pressed, but you can't end the run until you reconnect.

## Pieces

| File | Role |
|---|---|
| `lidar_web.py` | The panel (Flask, runs in `~/kiss_env`). Every button shells out to one of the scripts below.  The page itself is `panel.html`. |
| `lio_live.py` | **Live SLAM.** Starts `fastlio_mapping` and reads `/Odometry`, `/cloud_registered` and `/lio/gravity`. Writes `~/maps/live/{state.json,path.json,tiles/}` and saves on SIGINT. Runs under the system python. |
| `lio_run.sh` | **Bag → map.** Replays a bag through FAST-LIO + `lio_live.py` in ROS domain 43, so it can't collide with the live driver.  `lio-run <bag> --test <dir>` does the same into a test folder, in test ROS domain 77, and prints FAST-LIO's `[lite3]` diagnostics; it replaced `lio_eval.sh`. |
| `fastlio_mid360s.yaml` | FAST-LIO2 parameters (see below). |
| `lio_check.py` | `lio-check <map dir>...`: scores saved maps. Reports jumps over 0.5 m per update (divergence), tilt vs gravity, the floor-slope test, and the end-to-start gap (the drift on a closed loop). |
| `lio_clean.py` | `lio-clean <map>`: removes **ghosts** (people who walked past) by finding map cells that later laser beams passed straight through. Runs automatically on Stop & save and in Record & map; by hand it also rebuilds walls and the viewer. Keeps `gravity_aligned.uncleaned.ply`; what it removed goes to `ghosts.ply`. Needs the run's `scans.bin` (saved since 2026-10-12). |
| `lio_recover.py` | `lio-recover ~/maps/live --name <name>`: rebuilds a saved map from a Live session that never got to Stop & save (battery swap or power cut), using its `scans.bin`, then runs ghost removal, floor maps and the viewer. **Copy `~/maps/live/scans.bin` somewhere safe before the next Live start**, which replaces it. |
| `thermal_node.py` | ROS 2 node for the USB thermal camera: a PureThermal board with a **FLIR Lepton 3.0**, 160×120 at 9 fps. It is **not radiometric**: values are relative heat, not °C. Publishes `/thermal/image_raw` (mono16), `/thermal/image` (mono8), `/thermal/image_color` (rgb8) and an approximate `/thermal/camera_info`, frame `thermal_camera`, on domain 42. It reconnects if the camera drops. Starts at boot from the lite3 user's crontab (`@reboot ~/bin/thermal_start.sh`, no sudo needed). That script restarts the node if it exits, runs only one copy, logs to `~/.thermal_camera.log`, and turns the picture upright: the camera is mounted on its side, so `THERMAL_ROTATE=90` (clockwise) gives 120×160 portrait images. `thermal-camera.service` in `~/bin` is an alternative if you'd rather use systemd (needs sudo; don't use both). It also writes the latest colour frame to `/dev/shm/thermal_color.jpg` (in RAM, 9 fps). The web panel streams that as MJPEG (`/thermal.mjpg`) behind the **Thermal** toggle in its header, which is remembered per browser; `?thermal=1` in the address opens it directly. |
| `kiss_flatten.py` | Removes height drift from a map of **one** level floor; keeps `*.unflattened.*` backups. |
| `map_walls.py` | 2D wall maps (`walls/floorN.pgm/.yaml`) for Nav2. `--one-floor`; the cell size is chosen automatically (10 cm for live maps, 5 cm for dense clouds). A cell is a wall where its column of points spans at least 1 m. For the sparse 10 cm FAST-LIO clouds, gaps of up to 20 cm are allowed within that column: demanding an unbroken 1 m left corridor walls dashed, and allowing gaps adds about 40% more wall cells on FAST-LIO maps without thickening walls. Dense 5 cm clouds keep the strict rule, because old KISS maps stack drifted floor copies close together. |
| `map_web.py` | Standalone 3D viewer HTML. Its right-hand menu has Walls and **Path** toggles; `--one-floor`.  The page itself is `map_viewer.html`. |
| `maplib.py` | Shared helpers for all the mapping scripts: reading and writing map clouds (PLY), 10 cm cell keys, the `scans.bin` scan record, the levelling rotation, PointCloud2 parsing. numpy only, so both Pythons can import it. |

A saved map looks like this (the same layout for live runs and bags):

```
~/maps/<name>_<stamp>/slam_output/latest/
    gravity_aligned.ply          map, 10 cm voxels, levelled, z up
    gravity_aligned_path.npy     sensor path, N x 3, same frame
    <name>_poses.npy             N x 4 x 4 poses
    imu_tilt.npy                 per scan: SLAM attitude vs gravity (deg)
    walls/floorN.{pgm,yaml,png}  Nav2 floor maps
~/maps/<name>_<stamp>.html       3D viewer (Library)
```

## FAST-LIO2 build

Source: `~/livox_ws/src/FAST_LIO`, `hku-mars/FAST_LIO` branch `ROS2` (commit a4743b0) with the `ikd-Tree` submodule. Local changes:

- **`src/preprocess.cpp`, `mid360_handler`:** use the driver's per-point `timestamp` field (absolute ns; the header stamp is the first point) as each point's time. Points are sorted by time, because lines arrive interleaved and FAST-LIO takes the scan's end time from the last point. Upstream guessed the times from yaw, as for a spinning LiDAR, which is wrong for the MID-360. It also read a `reflectivity` field that our driver calls `intensity`.
- **`src/preprocess.h`:** point type `LivoxPointXyzitlt` for that layout.
- **`src/laserMapping.cpp`:** publishes `/lio/gravity` (`Vector3Stamped`, frame `camera_init`), FAST-LIO's own gravity estimate with accelerometer bias removed. `lio_live.py` levels the saved map with it.
- **`src/laserMapping.cpp`, threading (2026-10-07, after the first live runs diverged):** upstream processes scans on the same single thread that receives the IMU, through an IMU queue of 10 messages (50 ms at 200 Hz). A scan here takes about 50 ms at the median and up to about 106 ms while walking, so IMU samples were dropped on roughly every other scan. On a trotting robot that made it "shoot away" (jumps of up to 17.7 m per update, the map falling 155 m). Now:
  - the IMU and LiDAR get their own callback groups on a `MultiThreadedExecutor`, with an IMU queue of 400 (2 s);
  - `sync_packages` locks the buffers;
  - LiDAR preprocessing runs outside the lock.

  Every 100 scans, `fastlio.log` gets a line like `[lite3] last 100 scans: processing p50 .. ms, max .. ms; IMU gaps >15 ms: N`. If N is not about 0, look there first.
- **`CMakeLists.txt` / `package.xml`:** dropped `pcl_ros` (not installed, unused) and declared `tf2_ros` directly.

Rebuild:

```bash
cd ~/livox_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && MAKEFLAGS=-j3 colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`-j3`: the template-heavy build can exhaust the Jetson's memory with more jobs. It takes about 2.5 minutes.

`fastlio_mid360s.yaml` differs from upstream's `mid360.yaml` in these settings:

- `lidar_type: 4`: the patched handler above.
- `extrinsic_est_en: false`: the MID-360's LiDAR-to-IMU offset is fixed; estimating it on a trotting robot only adds drift.
- `filter_size_surf/map: 0.3`: indoors, 0.5 m erases door frames.
- `blind: 0.5`: ignores the robot's own body.
- `det_range: 40`
- `dense_publish_en: true`: the map is built from `/cloud_registered`.
- `map_en` and `pcd_save_en` are off: both grow without bound on a long walk.

Not tuned yet: `acc_cov` / `gyr_cov` (left at 0.1). The trot shakes the body with peaks of about 3 g, so a larger `acc_cov` may help. Tune it against recorded closed loops.

## Why FAST-LIO2 replaced KISS-SLAM (2026-10-07)

KISS-SLAM uses LiDAR only. On this robot:

- **Fake climbing:** its pitch estimate wandered up to ±12°, which turns forward walking into climbing. A 12-minute flat run (`live_20261007_134448`, about 1.4 m/s) gained 16 m of height, split into four fake storeys. The floor fitted under the "climbing" path was level (+1°) where the path rose at 17°, so it was SLAM error, not terrain. Speed changes were not the cause. Speed made it worse: the same tilt error gives twice the height error per minute at twice the speed.
- **Lost track on a fast turn:** 79 m return error on a 129 m closed loop. SLAM disagreed with gravity by 59.5° at the turn.
- **Mitigation on KISS:** pulling KISS's tilt toward gravity every scan (gain 0.5, 5 s accelerometer averaging) halved the error. It left 0.4–3.3% horizontal drift on the closed loops and couldn't prevent the tracking loss.

FAST-LIO2 fuses the IMU into every registration, so tilt is held by gravity and the gyro carries it through turns. On replays of the September walks, the error between the path's climb and the slope of the floor under it (p90) was:

| Walk | KISS | KISS + gravity | FAST-LIO2 |
|---|---|---|---|
| `walk_20260924_165520` | 6.2° | 3.1° | **1.6°** |
| `walk_20260924_170638` | 7.0° | 3.2° | **1.7°** |

FAST-LIO2 has **no loop closure**. Horizontal drift still accumulates, slowly. Snapping loops shut is the next step if closed loops still end far from the start.

## Ghosts (people walking past)

`lio_live.py` logs every scan: the sensor position plus the 10 cm cells it hit, compressed, about 4 KB per scan or roughly 40 MB per 10 minutes. The log goes to `scans.bin`, with the run's re-levelling in `relevel.npy`.

`lio_clean.py` replays that log. A cell counts as a ghost when real beams passed through it (stopping 0.3 m + 3% short of where they hit) at least 3 times, and its 30 cm neighbourhood was passed through more than twice as often as hit. Only cells 0.15–2.1 m above the local floor are considered, which is where people are.

Each of its guards fixes a failure seen while building it:
- **Real beams, not a 3° direction grid:** the grid marked 34% of a map as see-through, because a sloping floor or oblique wall spans metres of range within one grid cell.
- **No clearing next to this scan's hits:** stops beams skimming along a wall from clearing it.
- **Neighbourhoods, not single cells:** with the MID-360's sparse pattern, a 10 cm cell of wall may be hit only once in a whole run.
- **Body-height band only:** floors and ceilings are skimmed at grazing angles and would otherwise be cleared.

On `walk_20260924_165520` it removed 0.69% of the map, nearly all within 2 m of the path, and floor-map walls were unchanged. It takes about 45 s per 4 minutes of walk.

It cannot remove someone who walked beside the robot the whole way, because nothing ever saw past them. Live maps saved before 2026-10-12 cannot be cleaned, since they kept only the merged cells.

## Power cuts

A battery swap kills a Live session before it can save, but `scans.bin` is on disk (written to disk every 2 s since 2026-10-12), so `lio-recover` can rebuild the map.

Compared with a normal save, a recovered map:
- stays levelled by the first second of the run (FAST-LIO's final gravity estimate isn't logged);
- has positions but no orientations in its poses;
- loses the last moments before the cut.

The reader stops cleanly at a half-written last record. First use: `long-walk-recovered_20261012_150849` (13 min, 972 m, four floors).

## Known issues and checks

- **"Shooting away" = FAST-LIO divergence.** Fixed at the cause above (IMU starvation). If it diverges anyway, the Live tab's status turns to `DIVERGED - stop and restart` when the robot moves more than 4 m within 1 s, which only a runaway does. Single 1–3 m steps are FAST-LIO snapping back into line. One long walk had 21 of them, and its map was fine (floor test 1.3°). They are counted as `jumps` but no longer raise the alarm. `lio_live.log` and `lio-check` count the jumps, and `fastlio.log` shows the `[lite3]` IMU-gap lines. The first three live runs (`first-fastlio2-walk*`, 16:20–16:24) were made with the old build and diverged 23–77 s in.

- **The IMU must be continuous.** `walk_20260924_164325` lost about 93% of its `/livox/imu` samples after 75 s while recording (14 Hz, gaps up to 0.8 s). KISS survived that, but FAST-LIO diverged (41 km). Live, `lio_live.py` and FAST-LIO subscribe directly. Check a bag before trusting its map: `ros2 bag info` should show `/livox/imu` at about 200 messages/s.
- **Storeys come from the path walked** (`map_walls.split_by_path`, used by both `map-walls` and `map-web`). A floor is where the robot walked level for at least 10 s, at sensor height minus 0.46 m. Heights are split into storeys wherever at least 0.4 m of height saw less than 2 s of level walking per 0.2 m. A real ramp collects at least 4 s per 0.2 m (`171517`); the way between two floors gets 1–2 s (`long-walk-recovered`). So slow drift and gentle ramps stay one storey, and floors joined by a short ramp don't merge. Each map point goes to the storey of the robot pass nearest to it, and is kept from 0.3 m below to 2.4 m above the floor at that spot. The old height-histogram method (`floors()`) is only a fallback for maps without a path. It mistook ceilings for floors, needed storeys 2.2 m apart, and on `fast-lio-2-multiple-floors-test_20261007_171517` cut the last floor in half so it got no plan.
- **One divergence on the fixed build:** `fast-lio-2-multiple-floors-test_20261007_170413`, at 39–42 s. The robot had stood still for about 12 s, then rose about 0.6 m while the tilt error climbed to 15–21°, and FAST-LIO accelerated away (11 m/s). The cause is not known yet; standing up from a sit is a suspect. Runs now keep their own `fastlio.log`. Record alongside Live to capture the next one.
- **Start standing still.** Two September bags start mid-stride, which tilted FAST-LIO's start-up "down" by 0.3–4.7°. `lio_live.py` re-levels the saved map with FAST-LIO's final gravity estimate. The live view uses the first second.
- **The driver delivers about 8 Hz to Python subscribers** (10 Hz configured); QoS is not the cause.
- **Ground-truth-free drift check:** along the path, compare its climb angle with the slope of the floor fitted under it. Real slopes and stairs agree; SLAM pitch error climbs over level floor. A closed loop's start-to-end gap is the direct measure.
- **Livox MID-360s specifics:** see the driver notes: `MID360s_config.json`, `msg_MID360s_launch.py`, `xfer_format = 0`.
