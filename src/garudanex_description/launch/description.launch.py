"""Publish the GarudaNEX robot description and its static TF chain.

All joints are fixed, so robot_state_publisher emits the entire tree once
on /tf_static and nothing on /tf. No joint_state_publisher is needed.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('garudanex_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'garudanex.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # value_type=str is MANDATORY. Without it the launch system may pass the
    # expanded XML as the wrong type and robot_state_publisher fails silently.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use /clock from Gazebo. Must be true in simulation.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
