"""Nav2 on a floor map: the mapless stack, plus a map server and AMCL.

    ros2 launch env/nav2_map.launch.py params:=/tmp/nav2_map.yaml [localizer:=cloud]

localizer:=cloud leaves AMCL out: lio_relay.py publishes map -> odom from the
Orin's 3D fixes instead (robot/cloud_locate.py).

nav_stack:=orin (step 4 of ~/nav_stack_migration.md): RPP and the recoveries publish on
/rpp/cmd_vel instead, and robot/shadow_link.py drive puts the Orin's MPPI on /cmd_vel.

Started by env/start_nav2_map.sh, which makes that parameter file first
(env/nav2_map_params.py). The vendor's own map launch (dr_nav2) is not used:
it has no AMCL, and its lidar layer is for another lidar.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap

TF = [('/tf', 'tf'), ('/tf_static', 'tf_static')]      # as nav2_bringup's own launch files do


def generate_launch_description():
    params = LaunchConfiguration('params')
    amcl = PythonExpression(["'", LaunchConfiguration('localizer'), "' == 'amcl'"])
    orin = IfCondition(PythonExpression(["'", LaunchConfiguration('nav_stack'), "' == 'orin'"]))
    mapless = os.path.join(get_package_share_directory('dr_nav2_mapless'), 'launch', 'dr_nav2_mapless.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('params'),
        DeclareLaunchArgument('localizer', default_value='amcl'),
        DeclareLaunchArgument('nav_stack', default_value='jetson'),
        # every node the include starts below; not scoped, so it holds there
        SetRemap('cmd_vel', '/rpp/cmd_vel', condition=orin),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapless),
            launch_arguments=[
                ('dr_nav2_config_file', params), ('launch_realsense', 'false'), ('use_rviz', 'false'),
                # For nav2_bringup's navigation launch two includes down, which writes this
                # into the parameters, false by default. With false the costmap subscribes
                # after the map server has sent the map, never gets it, and stays an empty
                # 30 m square with its corner at the origin (seen 2026-10-09).
                ('map_subscribe_transient_local', 'true')]),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[params], remappings=TF),
        Node(package='nav2_amcl', executable='amcl', name='amcl', condition=IfCondition(amcl),
             output='screen', parameters=[params], remappings=TF),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', condition=IfCondition(amcl),
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server', 'amcl']}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', condition=UnlessCondition(amcl),
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server']}]),
    ])
