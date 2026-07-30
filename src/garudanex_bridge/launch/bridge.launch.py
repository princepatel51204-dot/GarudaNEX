"""Launch the GarudaNEX PX4 to ROS 2 bridges."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('garudanex_bridge')
    params = os.path.join(pkg_share, 'config', 'bridge_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock from Gazebo. Must be true in simulation.',
        ),
        Node(
            package='garudanex_bridge',
            executable='odometry_bridge_node',
            name='garudanex_odom_bridge',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
            emulate_tty=True,
        ),
        Node(
            package='garudanex_bridge',
            executable='cmd_vel_bridge_node',
            name='garudanex_cmd_vel_bridge',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
            emulate_tty=True,
        ),
    ])
