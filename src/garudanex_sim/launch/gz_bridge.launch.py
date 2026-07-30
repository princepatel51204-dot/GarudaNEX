"""Bridge Gazebo Transport topics into the ROS 2 graph, and fix the scan frame.

Gazebo topic names embed the world name, the model name and the spawn index:

    /world/<WORLD>/model/<MODEL>/link/link/sensor/lidar_2d_v2/scan

Hardcoding that string means the bridge silently stops working the moment you
change worlds. Both are launch arguments here instead. Verify yours with:

    gz topic -l | grep -iE "lidar.*scan$"

The clock and lidar are the ONLY things bridged. IMU, barometer, magnetometer
and GNSS are consumed directly by PX4's own gz interface and never enter the
ROS graph - routing them through DDS would add latency and jitter to the
flight-control loop for no benefit, since EKF2 is their only consumer.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world  = LaunchConfiguration('world')
    model  = LaunchConfiguration('model')
    sensor = LaunchConfiguration('lidar_sensor')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Gazebo topic, assembled from the arguments.
    lidar_gz_topic = ['/world/', world, '/model/', model,
                      '/link/link/sensor/', sensor, '/scan']

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='walls',
                              description='Gazebo world name'),
        DeclareLaunchArgument('model', default_value='x500_lidar_2d_0',
                              description='Spawned model name incl. index'),
        DeclareLaunchArgument('lidar_sensor', default_value='lidar_2d_v2',
                              description='SDF sensor name of the 2D lidar'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # ros_gz_bridge is the node that PRODUCES /clock, so it must not
        # CONSUME sim time - that would have it wait for a clock it is itself
        # responsible for publishing. Hardcoded False on purpose.
        #
        # The CLI form: <topic>@<ros_type>[<gz_type>  where '[' means GZ_TO_ROS.
        # The published ROS topic keeps the Gazebo name; the relay below
        # subscribes to it by name and republishes on /scan.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                [*lidar_gz_topic, '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            ],
            parameters=[{'use_sim_time': False}],
        ),

        Node(
            package='garudanex_bridge',
            executable='scan_frame_relay_node',
            name='garudanex_scan_frame_relay',
            output='screen',
            parameters=[{
                'target_frame': 'lidar_link',
                'input_topic': lidar_gz_topic,
                'output_topic': '/scan',
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
