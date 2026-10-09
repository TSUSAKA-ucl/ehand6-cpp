import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 各ファイルのパスを取得
    pkg_share = get_package_share_directory('ehand6_ctrl')
    cfg = os.path.join(pkg_share, 'config', 'ehand6.yaml')
    urdf_file = os.path.join(pkg_share, 'config', 'dummy.urdf.xml')

    return LaunchDescription([
        DeclareLaunchArgument('side', default_value='right'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        
        # 1. 元々のコントロールノード
        Node(
            package='ehand6_ctrl',
            executable='ehand6_ctrl_node',
            name='ehand6_ctrl',
            output='screen',
            parameters=[cfg, {'side': LaunchConfiguration('side'),
                              'can_interface': LaunchConfiguration('can_interface')}],
        ),

        # 2. 追加：joint_state_publisher_gui ノード
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            # arguments 引数を使って、ダミーのURDFファイルを直接流し込みます
            arguments=[urdf_file],
            parameters=[{
                'rate': 2  # デフォルトは10 (Hz)。必要に応じて数値を変更してください。
            }],
            # デフォルトの /joint_states トピックを、ehand6_ctrl_node が待つ /joint_command にリマップします
            remappings=[
                ('/joint_states', '/joint_command')
            ]
        ),
    ])
