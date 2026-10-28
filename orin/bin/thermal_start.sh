#!/bin/bash
# Start the thermal camera node and keep it running. Launched at boot by the
# lite3 user's crontab (@reboot) - no sudo needed, unlike a systemd unit.
#
#   ~/bin/thermal_start.sh            (cron does this at boot)
#   pkill -f thermal_start.sh; pkill -f thermal_node.py     to stop it
#
# THERMAL_ROTATE: the camera is mounted on its side; 90 = turn the picture
# 90 deg clockwise (chosen by the user from a test frame, 2026-10-15).
THERMAL_ROTATE=${THERMAL_ROTATE:-90}
LOG="$HOME/.thermal_camera.log"

# Only one copy: a second start (cron + by hand) just exits.
exec 9>"$HOME/.thermal_camera.lock"
flock -n 9 || { echo "thermal camera already running"; exit 0; }

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
sleep 10                                   # let USB and the network settle after boot
while true; do
    [ -f "$LOG" ] && [ "$(stat -c %s "$LOG")" -gt 1000000 ] && mv "$LOG" "$LOG.old"
    echo "$(date '+%F %T') starting thermal_node.py (rotate $THERMAL_ROTATE)" >> "$LOG"
    /usr/bin/python3 "$HOME/bin/thermal_node.py" --ros-args -p rotate:="$THERMAL_ROTATE" >> "$LOG" 2>&1
    echo "$(date '+%F %T') thermal_node.py exited ($?), restarting in 3 s" >> "$LOG"
    sleep 3
done
