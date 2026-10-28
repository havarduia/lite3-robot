#!/bin/bash
# shadow.sh start NAME | stop | state - MPPI in shadow on the Orin (nav_stack_migration.md, step 3):
# terrain_level -> terrainAnalysis -> controller_server (MPPI), following the Jetson's route
# (robot/shadow_link.py on the Jetson). Its commands go to /shadow/cmd_vel and a log, never
# to the robot. Needs FAST-LIO2 running (lio_live). Log: ~/shadow/NAME.csv; nodes' output
# in ~/shadow/NAME.log. Stopped with TERM: a script's background jobs ignore SIGINT.
PIDS=/tmp/shadow.pids
case "$1" in
start)
    name=${2:?usage: shadow.sh start NAME}
    [ -s $PIDS ] && { echo "already running: $PIDS"; exit 1; }
    source /opt/ros/humble/setup.bash
    source ~/cmu_ws/install/setup.bash
    export ROS_DOMAIN_ID=${SHADOW_DOMAIN:-42}     # 43 for a bag replay (terrain_replay.sh)
    mkdir -p ~/shadow
    log=~/shadow/$name.log
    cd ~/bin
    # CMU's terrainAnalysis parameters, with the two the bridge showed wrong for our lidar height
    ta=$(sed -n 's/.*name="\([^"]*\)" value="\([^"]*\)".*/-p \1:=\2/p' \
         ~/cmu_ws/src/terrain_analysis/launch/terrain_analysis.launch)
    {
        python3 terrain_level.py >> $log 2>&1 & echo $!
        ~/cmu_ws/install/terrain_analysis/lib/terrain_analysis/terrainAnalysis --ros-args $ta \
            -p maxElevBelowVeh:=-0.9 -p noDecayDis:=1.5 >> $log 2>&1 & echo $!
        python3 shadow_nav.py ~/shadow/$name.csv >> $log 2>&1 & echo $!
        /opt/ros/humble/lib/nav2_controller/controller_server --ros-args --params-file shadow_nav2.yaml \
            -r cmd_vel:=/shadow/cmd_vel >> $log 2>&1 & echo $!
        /opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager --ros-args \
            -r __node:=lifecycle_manager_shadow --params-file shadow_nav2.yaml >> $log 2>&1 & echo $!
    } > $PIDS
    echo "started: $(tr '\n' ' ' < $PIDS)" ;;
stop)
    [ -s $PIDS ] || { echo "not running"; exit 0; }
    kill -TERM $(cat $PIDS) 2>/dev/null
    sleep 3
    kill -KILL $(cat $PIDS) 2>/dev/null
    rm -f $PIDS
    echo stopped ;;
state)
    [ -s $PIDS ] && ps -o pid,pcpu,rss,cmd -p "$(paste -sd, $PIDS)" | cut -c1-110 || echo "not running" ;;
*)
    echo "usage: shadow.sh start NAME | stop | state"; exit 2 ;;
esac
