import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_mino_control')
    default_map = os.path.join(pkg_share, 'maps', 'map_mino.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    lifecycle_nodes = [
        'map_server', 'amcl',
        'controller_server', 'planner_server',
        'behavior_server', 'bt_navigator',
        'velocity_smoother', 'collision_monitor'
    ]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={},
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_file}],
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'diff_drive_controller/cmd_vel')],
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')],
        ),
        Node(
            package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
            output='screen',
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes
            }],
        ),
    ])