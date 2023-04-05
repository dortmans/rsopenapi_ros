from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from math import pi


def generate_launch_description():
    namespace = ['robot', LaunchConfiguration('robot')]
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='1',
            description='Robot id'
        ),
        DeclareLaunchArgument(
            'hash', default_value="'0x7d9066e102eb9a4f'",
            description='Robot hash'
        ),
        Node(
            # namespace=namespace,
            package='rs_bridge',
            executable='rs_bridge_node',
            name='rs_bridge',
            parameters=[
                {'robot': LaunchConfiguration('robot')},
                {'hash': LaunchConfiguration('hash')}
            ],
            # remappings=remappings
        ),
    ])
