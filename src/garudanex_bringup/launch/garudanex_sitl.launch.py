"""GarudaNEX: everything except PX4 and Gazebo themselves.

PX4 stays in its own terminal because it needs its interactive pxh shell:

    PX4_GZ_WORLD=walls make px4_sitl gz_x500_lidar_2d

Startup is STAGED with timers. That is a pragmatic hack, not a design: PX4
SITL does not expose a readiness signal to the ROS graph, and the gz bridge
must be publishing /clock before any node with use_sim_time:=true starts, or
those nodes sit on a stopped clock and tf2 throws extrapolation errors
everywhere. The principled fix is a small lifecycle node that polls for the
first /fmu/out message and then triggers downstream activation.
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world        = LaunchConfiguration('world')

    pkg_sim    = FindPackageShare('garudanex_sim')
    pkg_desc   = FindPackageShare('garudanex_description')
    pkg_bridge = FindPackageShare('garudanex_bridge')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='walls'),

        # T+0: DDS agent and the Gazebo bridge (which produces /clock).
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            name='uxrce_dds_agent', output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_sim, 'launch', 'gz_bridge.launch.py'])),
            launch_arguments={'world': world,
                              'use_sim_time': use_sim_time}.items(),
        ),

        # T+4: consumers of /clock.
        TimerAction(period=4.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_desc, 'launch',
                                          'description.launch.py'])),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_bridge, 'launch',
                                          'bridge.launch.py'])),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            ),
        ]),
    ])
