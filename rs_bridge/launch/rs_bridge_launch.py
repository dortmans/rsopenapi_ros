from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from math import pi


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='1',
            description='Robot id'
        ),
        DeclareLaunchArgument(
            'hash', default_value="'0x7d9066e102eb9a4f'",
            description='Robot hash'
        ),
        DeclareLaunchArgument(
            'tf', default_value='false',
            description='TF broadcast enabled'
        ),
        Node(
            package='rs_bridge',
            executable='rs_bridge_node',
            name='rs_bridge',
            parameters=[
                {'robot': LaunchConfiguration('robot')},
                {'hash': LaunchConfiguration('hash')},
                {'tf': LaunchConfiguration('tf')}
            ],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', 
            						'--yaw', '-1.57079632679', '--pitch', '0', '--roll', '0', 
            						'--frame-id', 'base_link', 
            						'--child-frame-id', 'base_link_msl']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', 
            						'--yaw', '-1.57079632679', '--pitch', '0', '--roll', '0', 
            						'--frame-id', 'map', 
            						'--child-frame-id', 'map_msl']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', 
            						'--yaw', '0', '--pitch', '0', '--roll', '0', 
            						'--frame-id', 'map', 
            						'--child-frame-id', 'odom']
        ),
    ])
