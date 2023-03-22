from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


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
    ])
