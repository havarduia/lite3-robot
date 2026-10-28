#!/bin/bash
# Read-only check of the robot's other computers, run FROM the Jetson:
# OS, ROS install and network, and whether ROS 2 on domain 42 sees the
# Jetson's LiDAR topics. Changes nothing on those machines. Asks for the
# ysc password once per host.
#
#   robot-pc-check                     both: 192.168.1.103 and .120
#   robot-pc-check 192.168.1.103       one
# From Windows:  ssh -t lite3@100.114.74.5 ~/bin/robot_pc_check.sh
HOSTS=("$@")
[ ${#HOSTS[@]} -eq 0 ] && HOSTS=(192.168.1.103 192.168.1.120)
USER_REMOTE=${ROBOT_PC_USER:-ysc}

read -r -d '' REMOTE <<'EOF'
echo "host:   $(hostname)"
. /etc/os-release; echo "os:     $PRETTY_NAME ($(uname -m), kernel $(uname -r))"
[ -f /etc/nv_tegra_release ] && echo "jetson: $(head -1 /etc/nv_tegra_release)"
[ -f /proc/device-tree/model ] && echo "board:  $(tr -d '\0' < /proc/device-tree/model)"
echo "ros:    $(ls /opt/ros 2>/dev/null | tr '\n' ' ')"
echo "bashrc: $(grep -h 'ROS_\|RMW_' ~/.bashrc 2>/dev/null | tr '\n' ';')"
echo "net:"; ip -br addr | grep -v '^lo' | sed 's/^/        /'
D=$(ls /opt/ros 2>/dev/null | head -1)
if [ -n "$D" ]; then
    source /opt/ros/$D/setup.bash
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # same DDS as the Jetson
    unset ROS_LOCALHOST_ONLY
    echo "--- ROS 2 $D, topics visible on domain 42:"
    timeout 20 ros2 topic list --no-daemon --spin-time 5 2>&1 | sed 's/^/        /'
    # Foxy's CLI needs several seconds just to start and discover on a
    # Xavier NX: give each rate check 25 s, and show its last report.
    echo "--- /livox/imu rate (25 s; expect ~200 Hz):"
    timeout 25 ros2 topic hz /livox/imu 2>&1 | tail -4 | sed 's/^/        /'
    echo "--- /livox/lidar rate (25 s; ~520 KB messages, expect ~10 Hz):"
    timeout 25 ros2 topic hz /livox/lidar 2>&1 | tail -4 | sed 's/^/        /'
    echo "--- /thermal/image_raw rate (25 s; thermal camera, expect ~9 Hz):"
    timeout 25 ros2 topic hz /thermal/image_raw 2>&1 | tail -4 | sed 's/^/        /'
    echo "--- (empty above = no messages arrived)"
fi
EOF

for h in "${HOSTS[@]}"; do
    echo "================ $h"
    ssh -o ConnectTimeout=5 "$USER_REMOTE@$h" bash -s <<< "$REMOTE"
done
