import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get configuration file path
    config_file = os.path.join(
        get_package_share_directory('remote_control_controller'),
        'config',
        'gamepad_params.yaml'
    )

    # 1. Joy driver node to interface with the physical gamepad
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 50.0,  # 50 Hz repeat rate
        }]
    )

    # 2. Gamepad control mapping node
    gamepad_control_node = Node(
        package='remote_control_controller',
        executable='gamepad_control_node',
        name='gamepad_control_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        gamepad_control_node
    ])
