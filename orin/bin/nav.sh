#!/bin/bash
# nav.sh run FLOOR | stop | state - Nav2 on the Orin, driving (nav_stack_migration.md, step 5b).
# Started over ssh by perception's env/start_nav2_map.sh (NAV_STACK=orin), which first copies the
# floor's map to ~/lite3_maps/FLOOR/. `run` lives as long as that ssh: when perception's Nav2 stops,
# ssh ends, stdin closes and everything here stops with it. Needs FAST-LIO2 (lio_live) and
# cloud_locate with the floor's cloud (perception's lio_relay map). Output: ~/nav/FLOOR_<time>.log.
PIDS=/tmp/orin_nav.pids
stop() {
    [ -s $PIDS ] || return 0
    kill -TERM $(cat $PIDS) 2>/dev/null
    sleep 3
    kill -KILL $(cat $PIDS) 2>/dev/null
    rm -f $PIDS
}
case "$1" in
run)
    floor=${2:?usage: nav.sh run FLOOR}
    map=~/lite3_maps/$floor/map.yaml
    [ -f "$map" ] || { echo "no map $map"; exit 1; }
    stop
    ~/bin/shadow.sh stop > /dev/null       # one of them on the ports
    source /opt/ros/humble/setup.bash
    source ~/cmu_ws/install/setup.bash
    export ROS_DOMAIN_ID=42
    mkdir -p ~/nav
    log=~/nav/${floor}_$(date +%Y%m%d_%H%M%S).log
    cd ~/bin
    P="--ros-args --params-file orin_nav2.yaml"
    # terrainAnalysis as shadow.sh has it (the reasons are there)
    ta=$(sed -n 's/.*name="\([^"]*\)" value="\([^"]*\)".*/-p \1:=\2/p' \
         ~/cmu_ws/src/terrain_analysis/launch/terrain_analysis.launch)
    {
        ~/cmu_ws/install/lite3_terrain/lib/lite3_terrain/terrain_level >> $log 2>&1 & echo $!
        ~/cmu_ws/install/terrain_analysis/lib/terrain_analysis/terrainAnalysis --ros-args $ta \
            -p maxElevBelowVeh:=-0.9 -p noDecayDis:=1.5 -p vehicleHeight:=0.7 -p clearDyObs:=true >> $log 2>&1 & echo $!
        python3 orin_nav.py >> $log 2>&1 & echo $!
        L=/opt/ros/humble/lib
        $L/nav2_map_server/map_server $P -p yaml_filename:=$map >> $log 2>&1 & echo $!
        $L/nav2_planner/planner_server $P >> $log 2>&1 & echo $!
        $L/nav2_controller/controller_server $P -r cmd_vel:=cmd_vel_nav >> $log 2>&1 & echo $!
        $L/nav2_behaviors/behavior_server $P -r cmd_vel:=cmd_vel_nav >> $log 2>&1 & echo $!
        $L/nav2_bt_navigator/bt_navigator $P >> $log 2>&1 & echo $!
        $L/nav2_collision_monitor/collision_monitor $P >> $log 2>&1 & echo $!
        $L/nav2_lifecycle_manager/lifecycle_manager --ros-args -r __node:=lifecycle_manager_orin_nav \
            --params-file orin_nav2.yaml >> $log 2>&1 & echo $!
    } > $PIDS
    echo "orin nav up on $floor: $(tr '\n' ' ' < $PIDS), log $log"
    trap 'stop; exit 0' TERM INT HUP
    # until perception's ssh ends; waited on so a TERM is heard at once. Through fd 3: a background
    # job's own stdin is /dev/null
    exec 3<&0
    cat <&3 > /dev/null &
    wait $!
    stop
    echo "orin nav stopped" ;;
stop)
    stop; echo stopped ;;
state)
    [ -s $PIDS ] && ps -o pid,pcpu,rss,cmd -p "$(paste -sd, $PIDS)" | cut -c1-110 || echo "not running" ;;
*)
    echo "usage: nav.sh run FLOOR | stop | state"; exit 2 ;;
esac
