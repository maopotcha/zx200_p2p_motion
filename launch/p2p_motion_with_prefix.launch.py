#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 起動時に prefix (= namespace) を指定
    declare_prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Namespace prefix to apply to all topics, services, and actions'
    )
    prefix = LaunchConfiguration('prefix')

    # p2p_motion ノード本体
    p2p_node = Node(
        package='zx200_p2p_motion',
        executable='moveit_p2p_node',   # 実際のバイナリ名に合わせてください
        name='moveit_p2p_node',
        namespace=prefix,               # まずは Node の namespace を設定
        output='screen',
        # MoveGroupInterface が使う主な名詞を remap
        remappings=[
            # トピック
            ('/attached_collision_object', [prefix, '/attached_collision_object']),
            ('/trajectory_execution_event', [prefix, '/trajectory_execution_event']),
            # アクション
            ('move_action', [prefix, 'move_action']),
            # サービス（例）
            ('/apply_planning_scene', [prefix, '/apply_planning_scene']),
            ('/get_planning_scene', [prefix, '/get_planning_scene']),
            ('execute_trajectory', [prefix, 'execute_trajectory']),
            # ...必要に応じて他の /plan_XXX, /goal_state, /joint_states なども追加
        ]
    )

    return LaunchDescription([
        declare_prefix_arg,
        p2p_node,
    ])
