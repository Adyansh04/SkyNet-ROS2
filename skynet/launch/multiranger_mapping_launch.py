import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import yaml


def generate_launch_description():
    """
    Generate the launch description for the skynet package.

    Returns:
        LaunchDescription: The launch description object.
    """
    # load crazyflies
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

    return LaunchDescription([

        # Start the crazyflie server node
        Node(
            package='skynet',
            executable='crazyflie_server.py',
            name='crazyflie_server',
            output='screen',
            parameters=[server_params],
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

        #SLAM Toolbox
        Node(
            parameters=[
                {'odom_frame': 'odom'},
                {'map_frame': 'world'},
                {'base_frame': 'cf1'},
                {'scan_topic': '/cf1/scan'},
                {'use_scan_matching': False},
                {'max_laser_range': 3.5},
                {'resolution': 0.1},
                {'minimum_travel_distance': 0.01},
                {'minimum_travel_heading': 0.001},
                {'map_update_interval': 0.1},
                {'scan_queue_size': 10}
            ],
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'),

        # Start the rviz2 node
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('skynet'), 'config', 'slam.rviz')],
        ),
        
        # Open a new terminal for teleop_twist_keyboard
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', 'ros2 run teleop_twist_keyboard teleop_twist_keyboard'],
            output='screen',
            
        )
    ])
