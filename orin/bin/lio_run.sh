#!/bin/bash
# Map a recorded bag with FAST-LIO2 (replaced kiss_run.sh / KISS-SLAM on
# 2026-10-07; see lio_live.py for why).
#
#   lio-run ~/bags/walk_20260914_1530                 -> ~/maps/walk_20260914_1530/
#   lio-run ~/bags/walk_20260914_1530 --test ~/tests  -> ~/tests/walk_20260914_1530/
#
# Replays the bag at recorded speed through fastlio_mapping + lio_live.py, in
# its own ROS domain (43; 77 with --test, so a test cannot meet a real run)
# because the live driver publishes the same topic names. Then ghost removal
# and floor maps, so map-web, map-walls and the panel get what a Live save
# gives:
#   <out>/slam_output/latest/gravity_aligned.ply      10 cm voxels
#                           /gravity_aligned_path.npy
#                           /walls/floorN.pgm/.yaml   Nav2 maps
# --test also prints FAST-LIO's [lite3] lines (processing time, IMU gaps).
# Takes as long as the walk did. FAST-LIO needs the IMU without gaps: a bag
# whose recorder dropped /livox/imu samples can make it diverge.
set -e
BAG="${1:?usage: lio-run <bagdir> [--test <dir>]}"
[ -d "$BAG" ] || { echo "error: no such bag directory: $BAG" >&2; exit 1; }
BAG=$(realpath "$BAG")
TEST=""
[ "$2" = "--test" ] && TEST=$(realpath -m "${3:?--test needs a directory}")
OUT="${TEST:-$HOME/maps}/$(basename "$BAG")"
LATEST="$OUT/slam_output/latest"
mkdir -p "$OUT"

source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
export ROS_DOMAIN_ID=${LIO_RUN_DOMAIN:-$([ -n "$TEST" ] && echo 77 || echo 43)}

# Drop driver fragments (clock jumps mid-recording) - see bag_clean.py.
BAG=$(/usr/bin/python3 "$HOME/bin/bag_clean.py" "$BAG")
echo "bag : $BAG"
echo "out : $LATEST"
rm -rf "$LATEST"

/usr/bin/python3 -u "$HOME/bin/lio_live.py" --out "$OUT/lio_state" --save-path "$LATEST" &
LIVE=$!
trap 'kill -INT $LIVE 2>/dev/null; wait $LIVE' EXIT
sleep 4                                    # FAST-LIO up before the first scan
ros2 bag play "$BAG" --topics /livox/lidar /livox/imu > "$OUT/play.log" 2>&1
sleep 2
kill -INT $LIVE; wait $LIVE; trap - EXIT
[ -n "$TEST" ] && grep "\[lite3\]" "$OUT/lio_state/fastlio.log" | sed 's/.*\[lite3\] /  fastlio: /'
rm -rf "$OUT/lio_state" "$OUT/play.log"

# Ghosts: people who walked past (see lio_clean.py). Non-fatal.
/usr/bin/python3 "$HOME/bin/lio_clean.py" "$OUT" --no-rebuild || echo "  (ghost removal skipped)"
/usr/bin/python3 "$HOME/bin/map_walls.py" "$OUT" || echo "  (wall maps skipped)"
echo
echo "results in $LATEST"
ls -1 "$LATEST" | sed 's/^/  /'
