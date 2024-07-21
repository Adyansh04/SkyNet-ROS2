import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    """
    Generates the launch description for the skynet package.

    Returns:
        LaunchDescription: The launch description object.
    """
    
    # crazyflies params
    crazyflies_yaml = os.path.join(
        get_package_share_directory('skynet'),
        'config',
        'crazyflies.yaml')

    with open(crazyflies_yaml, 'r') as ymlfile:
        crazyflies = yaml.safe_load(ymlfile)

    server_params = crazyflies

    # robot description
    urdf = os.path.join(
        get_package_share_directory('skynet'),
        'urdf',
        'crazyflie_description.urdf')
    with open(urdf, 'r') as f:
        robot_desc = f.read()
    server_params['robot_description'] = robot_desc

    cf_examples_dir = get_package_share_directory('skynet')
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')
    map_name = 'map'

    return LaunchDescription([

        # Start the crazyflie server node
        Node(
            package='skynet',
            executable='crazyflie_server.py',
            name='crazyflie_server',
            output='screen',
            parameters=[{'world_tf_name': 'map'},
                        server_params],
        ),

        # Start the velocity multiplexer node
        Node(
            package='skynet',
            executable='vel_mux.py',
            name='vel_mux',
            output='screen',
            parameters=[{'hover_height': 0.3},
                        {'incoming_twist_topic': '/cmd_vel'},
                        {'robot_prefix': '/cf1'}]
        ),

        # Start the SLAM toolbox node
        Node(
            parameters=[
                {'odom_frame': 'odom'},
                {'map_frame': 'map'},
                {'base_frame': 'cf1'},
                {'scan_topic': '/cf1/scan'},
                {'use_scan_matching': False},
                {'max_laser_range': 3.5},
                {'resolution': 0.1},
                {'minimum_travel_distance': 0.01},
                {'minimum_travel_heading': 0.001},
                {'map_update_interval': 0.1},
                {'mode': 'localization'},
                {'map_file_name': cf_examples_dir + '/data/' + map_name},
                {'map_start_pose': [0.0, 0.0, 0.0]}],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        ),

        # Include the bringup launch description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',
                'use_sim_time': 'False',
                'map': cf_examples_dir + '/data/' + map_name + '.yaml',
                'params_file': os.path.join(cf_examples_dir, 'config/nav2_params.yaml'),
                'autostart': 'True',
                'use_composition': 'True',
                'transform_publish_period': '0.02'
            }.items()
        ),

        # Include the RViz launch description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_launch_dir, 'rviz_launch.py')),
            launch_arguments={
                'rviz_config': os.path.join(
                    bringup_dir, 'rviz', 'nav2_default_view.rviz')
            }.items()
        )
    ])
