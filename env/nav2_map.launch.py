"""Nav2 on a floor map: the mapless stack, plus a map server and AMCL.

    ros2 launch env/nav2_map.launch.py params:=/tmp/nav2_map.yaml [localizer:=cloud]

localizer:=cloud leaves AMCL out: lio_relay.py publishes map -> odom from the
Orin's 3D fixes instead (robot/cloud_locate.py).

nav_stack:=orin (step 5b of ~/nav_stack_migration.md): the Orin's Nav2 drives (robot/orin_nav.py
passes the goals). Here only the map, the costmaps and the planner run, for the panel, cost_at,
turn_blocked and route_to: planner_server and controller_server (which holds the local costmap and
its camera layer, sent on to the Orin) without bt_navigator. The controller gets no goal; its
cmd_vel goes to /rpp/cmd_vel all the same.

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
# base_link -> camera_link, the vendor's mount (dr_nav2_mapless.launch.py, which says why the pitch matters)
CAMERA_TF = ['0.25489', '0', '0.07249', '0', '0.34907', '0', 'base_link', 'camera_link']


def generate_launch_description():
    params = LaunchConfiguration('params')
    amcl = PythonExpression(["'", LaunchConfiguration('localizer'), "' == 'amcl'"])
    is_orin = PythonExpression(["'", LaunchConfiguration('nav_stack'), "' == 'orin'"])
    orin, jetson = IfCondition(is_orin), UnlessCondition(is_orin)
    mapless = os.path.join(get_package_share_directory('dr_nav2_mapless'), 'launch', 'dr_nav2_mapless.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('params'),
        DeclareLaunchArgument('localizer', default_value='amcl'),
        DeclareLaunchArgument('nav_stack', default_value='jetson'),
        # every node started below; not scoped, so it holds there
        SetRemap('cmd_vel', '/rpp/cmd_vel', condition=orin),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapless), condition=jetson,
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
        # orin: what the include gave that is still wanted here
        Node(package='tf2_ros', executable='static_transform_publisher', name='base2camera_tf_broadcaster',
             condition=orin, arguments=CAMERA_TF),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', condition=orin,
             output='screen', parameters=[params], remappings=TF),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', condition=orin,
             output='screen', parameters=[params], remappings=TF),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', condition=orin,
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['planner_server', 'controller_server']}]),
    ])
