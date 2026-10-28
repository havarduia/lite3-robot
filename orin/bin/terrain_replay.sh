#!/bin/bash
# terrain_replay.sh BAG OUT.npz - play an Orin bag (bin/run.py --orin-bag) through
# terrain_level and CMU terrainAnalysis, and keep the terrain maps (terrain_snap.py).
# Domain 43, so it cannot meet the live FAST-LIO2 on 42 (nav_stack_migration.md, step 2).
set -e
bag=$1 out=$2
[ -d "$bag" ] && [ -n "$out" ] || { echo "usage: $0 BAG OUT.npz"; exit 2; }
source /opt/ros/humble/setup.bash
source ~/cmu_ws/install/setup.bash
export ROS_DOMAIN_ID=43
cd ~/bin
python3 terrain_level.py & level=$!
ros2 launch terrain_analysis terrain_analysis.launch > /tmp/terrain_analysis.log 2>&1 & ta=$!
python3 terrain_snap.py "$out" & snap=$!
sleep 3
ros2 bag play "$bag" --topics /cloud_registered /Odometry /lio/gravity
sleep 2
kill -INT $snap; wait $snap
kill -INT $ta $level; wait
