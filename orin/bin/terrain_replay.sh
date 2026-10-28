#!/bin/bash
# terrain_replay.sh BAG OUT.npz [-p name:=value ...] - play an Orin bag (bin/run.py --orin-bag) through
# terrain_level (orin/lite3_terrain) and CMU terrainAnalysis, and keep the terrain maps (terrain_snap.py).
# Domain 43, so it cannot meet the live FAST-LIO2 on 42 (nav_stack_migration.md, step 2).
# Logs: /tmp/terrain_{level,analysis,snap}.log. Stopped with TERM: a script's background
# jobs ignore SIGINT.
set -e
bag=$1 out=$2
shift 2 || true             # the rest override the launch file's parameters
[ -d "$bag" ] && [ -n "$out" ] || { echo "usage: $0 BAG OUT.npz"; exit 2; }
source /opt/ros/humble/setup.bash
source ~/cmu_ws/install/setup.bash
export ROS_DOMAIN_ID=43
cd ~/bin
~/cmu_ws/install/lite3_terrain/lib/lite3_terrain/terrain_level > /tmp/terrain_level.log 2>&1 & level=$!
~/cmu_ws/install/terrain_analysis/lib/terrain_analysis/terrainAnalysis --ros-args \
  $(sed -n 's/.*name="\([^"]*\)" value="\([^"]*\)".*/-p \1:=\2/p' \
    ~/cmu_ws/src/terrain_analysis/launch/terrain_analysis.launch) "$@" > /tmp/terrain_analysis.log 2>&1 & ta=$!
python3 terrain_snap.py "$out" > /tmp/terrain_snap.log 2>&1 & snap=$!
trap 'kill -TERM $snap $ta $level 2>/dev/null' EXIT
sleep 3
ros2 bag play "$bag" --topics /cloud_registered /Odometry /lio/gravity > /dev/null
sleep 2
kill -TERM $snap; wait $snap || true
cat /tmp/terrain_level.log /tmp/terrain_snap.log
