"""Nav2 navigation on top of the 2D localization stack.

Run all three, in this order:

    ros2 launch unitree_go2_sim unitree_go2_launch.py rviz:=false
    ros2 launch unitree_go2_sim localization_2d.launch.py rviz:=false
    ros2 launch unitree_go2_sim navigation_2d.launch.py

Then use RViz's "2D Goal Pose" button to send the robot somewhere.

The controller publishes to /cmd_vel_nav and nav2_velocity_smoother turns that
into the /cmd_vel that CHAMP consumes. The indirection is deliberate: stepping
this robot's velocity command abruptly puts it on its back, and the smoother's
acceleration limits are what prevent that.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LIFECYCLE_NODES = [
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    pkg_share = get_package_share_directory("unitree_go2_sim")
    default_params = os.path.join(pkg_share, "config", "nav2", "nav2_params.yaml")

    common = [params_file, {"use_sim_time": use_sim_time}]

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=common,
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=common,
    )

    behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=common,
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=common,
    )

    smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=common,
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": LIFECYCLE_NODES},
            # 4.0 s default is tight when the machine is already busy running
            # Gazebo; a slow configure then looks like a dead node.
            {"bond_timeout": 10.0},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_nav",
        output="screen",
        arguments=["-d", os.path.join(pkg_share, "rviz", "nav_2d.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "params_file", default_value=default_params,
            description="Nav2 parameter file.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Open RViz with costmaps, path and the goal tool.",
        ),
        controller,
        planner,
        behaviors,
        bt_navigator,
        smoother,
        lifecycle_manager,
        rviz,
    ])
