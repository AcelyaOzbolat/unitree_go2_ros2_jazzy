"""Turn the Velodyne point cloud into a 2D /scan.

Shared by slam_2d.launch.py and localization_2d.launch.py so both pipelines are
guaranteed to consume exactly the same scan.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    scan_config = os.path.join(
        get_package_share_directory("unitree_go2_sim"),
        "config", "scan", "pointcloud_to_laserscan.yaml",
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[scan_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cloud_in", cloud_topic),
            ("scan", scan_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "cloud_topic", default_value="/velodyne_points/points",
            description="3D LiDAR point cloud to flatten",
        ),
        DeclareLaunchArgument(
            "scan_topic", default_value="/scan",
            description="LaserScan topic to publish",
        ),
        pointcloud_to_laserscan_node,
    ])
