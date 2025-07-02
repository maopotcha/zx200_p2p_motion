from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        GroupAction([
            PushRosNamespace('zx200'),
            Node(
                package='zx200_p2p_motion',
                executable='moveit_p2p_node',  # 実際のノード名に変更
#                name='p2p_motion',             # ノード名（/zx200/p2p_motion になる）
                output='screen',
                parameters=[{
                    'some_param': 'value'      # 必要ならパラメータをここに
                }]
            ),
        ])
    ])
