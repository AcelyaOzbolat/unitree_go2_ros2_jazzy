"""2D localization on a previously saved map: /scan + odom -> AMCL.

Run this instead of slam_2d.launch.py once you have a map. AMCL takes over the
map -> odom transform that slam_toolbox used to publish.

    ros2 launch unitree_go2_sim unitree_go2_launch.py
    ros2 launch unitree_go2_sim localization_2d.launch.py map:=/abs/path/to/simple_room.yaml

Then give it a starting guess with the "2D Pose Estimate" button in RViz (or the
/initialpose topic) and walk the robot a few metres so the particle cloud converges.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    set_initial_pose = LaunchConfiguration("set_initial_pose")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    pkg_share = get_package_share_directory("unitree_go2_sim")

    amcl_params = os.path.join(pkg_share, "config", "amcl", "amcl.yaml")
    default_map = os.path.join(pkg_share, "maps", "simple_room.yaml")

    scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "scan_2d.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            amcl_params,
            {"use_sim_time": use_sim_time},
            {"yaml_filename": map_yaml},
        ],
    )

    # Seeded at the spawn pose by default. slam_toolbox built this map from a robot
    # standing at the world origin facing +x, so the map frame lines up with the
    # world and (0, 0, 0) is the right guess for a freshly launched sim (verified:
    # best alignment of the saved map against the world was 0 deg, 0 m offset).
    # Pass initial_pose_x/y/yaw to start somewhere else, or just use RViz's
    # "2D Pose Estimate" - a pose published on /initialpose overrides this.
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_params,
            {"use_sim_time": use_sim_time},
            {"set_initial_pose": set_initial_pose},
            {"initial_pose.x": initial_pose_x},
            {"initial_pose.y": initial_pose_y},
            {"initial_pose.z": 0.0},
            {"initial_pose.yaw": initial_pose_yaw},
        ],
    )

    # map_server and amcl are lifecycle nodes: without a manager they load their
    # parameters and then sit inactive, publishing nothing.
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_2d",
        output="screen",
        arguments=["-d", os.path.join(pkg_share, "rviz", "localization_2d.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Open RViz with the 2D map view. Launch the sim with rviz:=false to avoid two windows.",
        ),
        DeclareLaunchArgument(
            "map", default_value=default_map,
            description="Path to the map .yaml saved by map_saver_cli",
        ),
        DeclareLaunchArgument(
            "set_initial_pose", default_value="true",
            description="Seed AMCL at launch instead of waiting for a pose in RViz.",
        ),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument(
            "initial_pose_yaw", default_value="0.0",
            description="Starting heading in radians.",
        ),
        scan_launch,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz_node,
    ])
