import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def generate_launch_description():
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
        Node(
            package='skynet',
            executable='crazyflie_server.py',
            name='crazyflie_server',
            output='screen',
            parameters=[server_params]
        ),
        Node(
            package='skynet',
            executable='vel_mux.py',
            name='vel_mux',
            output='screen',
            parameters=[{'hover_height': 0.5},
                        {'incoming_twist_topic': '/cmd_vel'},
                        {'robot_prefix': '/cf1'}]
        ),
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d' + os.path.join(get_package_share_directory('skynet'), 'config', 'config.rviz')],
        # )
    ])
