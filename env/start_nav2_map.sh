#!/bin/bash
# Launch Nav2 on a floor map: start_nav2_map.sh <floor>. Normally called by
# Lite3.nav_start(floor), which enforces stand-first ordering and cleans up
# orphaned nodes. The floor's map is ~/lite3_maps/<floor>/map.yaml (env/get_map.sh).
ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
source "$ENV_DIR/lite3_env.sh"
FLOOR="${1:?usage: start_nav2_map.sh <floor>}"
# Who places him on the map: amcl (the 2D lidar slice, as before) or cloud (the Orin's 3D
# localizer, robot/cloud_locate.py; .planning/plans/cloud-localization.md). From the panel:
# LOCALIZER=cloud python3 -m robot.hmi
LOCALIZER="${LOCALIZER:-amcl}"
MAP="$HOME/lite3_maps/$FLOOR/map.yaml"
[ -f "$MAP" ] || { echo "no map for floor '$FLOOR' ($MAP)" >&2; exit 1; }
# Map mode's parameters are the mapless ones with the map added, made afresh
# each start so the tuning in the mapless file is the only copy.
SHARE=$(python3 -c "from ament_index_python.packages import get_package_share_directory as d; print(d('dr_nav2_mapless'))")
PARAMS=/tmp/nav2_map.yaml
python3 "$ENV_DIR/nav2_map_params.py" "$SHARE/config/lite_nav2_mapless.yaml" "$MAP" "$PARAMS" || exit 1
# The sonars for the range layer, and "is he localized" (robot/locate.py). Same
# process group as the launch below, so nav_stop() kills them with the stack.
rm -f /tmp/sonar_range.ready
python3 -m robot.sonar_range &
if [ "$LOCALIZER" = cloud ]; then
    CLOUD=$(cat "$HOME/lite3_maps/$FLOOR/cloud.txt" 2>/dev/null) \
        || { echo "floor '$FLOOR' has no cloud.txt (its walk on the Orin): LOCALIZER=cloud needs one" >&2; exit 1; }
    python3 -m robot.lio_relay map "$CLOUD"
    python3 -m robot.locate "$FLOOR" cloud &
else
    python3 -m robot.lio_relay map ""       # the Orin's localizer lets go: AMCL owns map -> odom
    python3 -m robot.locate "$FLOOR" &
fi
for _ in $(seq 100); do [ -e /tmp/sonar_range.ready ] && break; sleep 0.1; done
exec ros2 launch "$ENV_DIR/nav2_map.launch.py" params:="$PARAMS" localizer:="$LOCALIZER"
