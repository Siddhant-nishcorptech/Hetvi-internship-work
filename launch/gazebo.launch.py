from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():

    gz_launch_path        = os.path.join(get_package_share_path('ros_gz_sim'),
                                         'launch', 'gz_sim.launch.py')
    gz_config_path        = os.path.join(get_package_share_path('my_bot_urdf_description'),
                                         'config', 'controllers.yaml')
    gz_bridge_config_path = os.path.join(get_package_share_path('my_bot_urdf_description'),
                                         'config', 'gz_bridge.yaml')
    urdf_path             = os.path.join(get_package_share_path('my_bot_urdf_description'),
                                         'urdf', 'my_bot_urdf.xacro')
    world_path            = os.path.join(get_package_share_path('my_bot_urdf_description'),
                                         'worlds', 'world.sdf')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': True}]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={'gz_args': f'{world_path} -r'}.items()
    )

    ros_gz_sim_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description'],
        output='screen'
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            gz_config_path
        ],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gz_bridge_config_path}],
        output='screen'
    )

    spawners = TimerAction(
        period=10.0,
        actions=[
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster'], output='screen',
                 parameters=[{'use_sim_time': True}]),
            Node(package='controller_manager', executable='spawner',
                 arguments=['wheel_controller'], output='screen',
                 parameters=[{'use_sim_time': True}]),
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_controller'], output='screen',
                 parameters=[{'use_sim_time': True}]),
        ]
    )

    controller_node = Node(
        package='my_bot_urdf_description',
        executable='controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    teleop_node = Node(
        package='my_bot_urdf_description',
        executable='teleop_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    voxel_filter_node = Node(
        package="my_bot_urdf_description",
        executable="voxel_filter_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    stair_detector_node = Node(
        package="my_bot_urdf_description",
        executable="stair_detector_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        ros_gz_sim_node,
        ros2_control_node,
        spawners,
        ros_gz_bridge,
        controller_node,
        teleop_node,
        voxel_filter_node,
        stair_detector_node,
    ])
    