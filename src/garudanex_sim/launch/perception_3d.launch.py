"""3D perception chain: gz point cloud -> ROS -> flattened scan + Octomap.

The 3D cloud drives Octomap and (later) Nav2's 3D obstacle layer, while a
flattened 2D scan keeps SLAM Toolbox - a 2D algorithm - working unchanged.
Layering rather than replacing is what keeps the 2D baseline measurable.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    sensor = LaunchConfiguration('sensor')

    gz_points = ['/world/', world, '/model/', model,
                 '/link/link/sensor/', sensor, '/scan/points']

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='garudanex_points_bridge', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[[*gz_points, '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked']],
        remappings=[(gz_points, '/points_raw')],
    )

    flatten = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='garudanex_cloud_to_scan', output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'lidar_link',
            'min_height': -0.15, 'max_height': 0.15,
            'angle_min': -3.14159, 'angle_max': 3.14159,
            'angle_increment': 0.0174533,
            'range_min': 0.15, 'range_max': 30.0,
            'scan_time': 0.1,
        }],
        remappings=[('cloud_in', '/points_raw'), ('scan', '/scan')],
    )

    octomap = Node(
        package='octomap_server', executable='octomap_server_node',
        name='garudanex_octomap', output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'map',
            'base_frame_id': 'base_footprint',
            'resolution': 0.15,
            'sensor_model.max_range': 25.0,
            'filter_ground_plane': False,
        }],
        remappings=[('cloud_in', '/points_raw')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='garudanex_facility'),
        DeclareLaunchArgument('model', default_value='x500_lidar_2d_0'),
        DeclareLaunchArgument('sensor', default_value='garudanex_lidar_3d'),
        bridge, flatten, octomap,
    ])
