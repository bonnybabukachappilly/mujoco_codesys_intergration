import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cfg = os.path.join(
        get_package_share_directory('opcua_server'),
        'config', 'opcua_server.yaml')

    return LaunchDescription([
        Node(
            package='opcua_server', executable='opcua_server', output='screen',
            parameters=[{'config_file': cfg, 'use_sim_time': True}]),
    ])
