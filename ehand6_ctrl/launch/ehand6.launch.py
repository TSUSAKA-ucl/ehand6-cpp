import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('ehand6_ctrl'), 'config', 'ehand6.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        Node(
            package='ehand6_ctrl',
            executable='ehand6_ctrl_node',
            name='ehand6_ctrl',
            output='screen',
            parameters=[cfg, {'side': LaunchConfiguration('side'),
                              'can_interface': LaunchConfiguration('can_interface')}],
        ),
    ])
