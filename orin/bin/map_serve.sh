#!/bin/bash
# Serve the rig's control panel over the tailnet: scan, record, view.
# No display, no X server, no RViz on the Jetson.
#
#   map-serve          panel + ~/maps on port 8000
#   map-serve 8080     different port
#   map-serve --static plain file listing, no buttons (old behaviour)
#
# The panel needs Flask, which lives in ~/kiss_env. That venv was built
# --system-site-packages, so rclpy stays visible from inside it and one
# interpreter can do both.
PORT=8000
STATIC=0
for a in "$@"; do
    case "$a" in
        --static) STATIC=1 ;;
        *)        PORT="$a" ;;
    esac
done
DIR="${MAP_DIR:-$HOME/maps}"
mkdir -p "$DIR"
TS=$(tailscale ip -4 2>/dev/null | head -1)

if [ "$STATIC" = 1 ]; then
    echo "serving $DIR (static listing)"
    [ -n "$TS" ] && echo "  tailnet : http://$TS:$PORT/" || echo "  (tailscale not up)"
    echo "  local   : http://127.0.0.1:$PORT/"
    echo "  Ctrl-C to stop"
    exec python3 -m http.server "$PORT" --directory "$DIR"
fi

[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f "$HOME/livox_ws/install/setup.bash" ] && source "$HOME/livox_ws/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
source "$HOME/kiss_env/bin/activate"
exec python3 "$HOME/bin/lidar_web.py" "$PORT"
