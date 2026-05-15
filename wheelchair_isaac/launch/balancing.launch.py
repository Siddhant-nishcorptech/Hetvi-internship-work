"""
balancing.launch.py — Launch balance controller with PID params
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('wheelchair_isaac')
    params_file = os.path.join(pkg_dir, 'config', 'balance_params.yaml')

    balance_node = Node(
        package='wheelchair_isaac',
        executable='balance_controller',
        name='balance_controller',
        output='screen',
        parameters=[params_file],
    )

    teleop_node = Node(
        package='wheelchair_isaac',
        executable='teleop_keyboard',
        name='isaac_teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # needs its own terminal for raw input
    )

    return LaunchDescription([
        balance_node,
        teleop_node,
    ])
