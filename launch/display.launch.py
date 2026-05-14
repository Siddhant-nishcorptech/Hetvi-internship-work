from launch_ros.actions import Node
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    share_dir = get_package_share_directory('my_bot_urdf_description')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'config.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],  
        parameters=[{'use_sim_time': True}], 
        output='screen'
    )

    return LaunchDescription([
        rviz_node
    ])