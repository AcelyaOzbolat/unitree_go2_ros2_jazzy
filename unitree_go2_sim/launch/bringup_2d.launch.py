"""Localization + navigation in one command, started in the right order.

    ros2 launch unitree_go2_sim unitree_go2_launch.py rviz:=false
    ros2 launch unitree_go2_sim bringup_2d.launch.py

Running localization_2d and navigation_2d as separate commands works too, but
navigation has a hard dependency on localization that fails confusingly when it
is missed: with no map_server there is no /map and no map frame, so every Nav2
costmap waits on a transform that will never arrive and the lifecycle manager
eventually gives up with "Failed to bring up all requested nodes". Nothing in
that message says "the localization stack is not running", so starting them
together removes the chance to get it wrong.

The delay before navigation exists for the same reason: the costmaps want /map
and map -> odom to already be there when they activate.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    nav_delay = LaunchConfiguration("nav_delay")
    rviz_delay = LaunchConfiguration("rviz_delay")

    pkg_share = get_package_share_directory("unitree_go2_sim")
    default_map = os.path.join(pkg_share, "maps", "simple_room.yaml")

    # GroupAction(scoped=True) matters here. launch_arguments passed to an include
    # are plain SetLaunchConfiguration actions that leak into the parent scope, so
    # without the group this "rviz: false" would overwrite the top-level rviz
    # argument - and the navigation include, evaluated 8 s later, would read false
    # and never open RViz.
    localization = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "localization_2d.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map": map_yaml,
                    "rviz": "false",   # navigation_2d brings the RViz that shows both
                }.items(),
            )
        ],
        scoped=True,
    )

    navigation = TimerAction(
        period=nav_delay,
        actions=[
            GroupAction(
                [
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(pkg_share, "launch", "navigation_2d.launch.py")
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            # RViz is started separately below, after Nav2 has
                            # finished coming up.
                            "rviz": "false",
                        }.items(),
                    )
                ],
                scoped=True,
            )
        ],
    )

    # RViz last, and on its own timer. Loading Ogre, the shaders and the map is
    # heavy, and on a machine already running Gazebo at ~0.6x real time that cost
    # lands exactly while the lifecycle manager is configuring controller_server.
    # Measured: the change_state service response timed out and the whole Nav2
    # bringup stalled with planner_server left unconfigured.
    rviz = TimerAction(
        period=rviz_delay,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_nav",
                output="screen",
                arguments=["-d", os.path.join(pkg_share, "rviz", "nav_2d.rviz")],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "map", default_value=default_map,
            description="Map .yaml for AMCL and the Nav2 costmaps.",
        ),
        DeclareLaunchArgument(
            "nav_delay", default_value="8.0",
            description="Seconds to let localization publish /map and map -> odom "
                        "before Nav2's costmaps try to activate.",
        ),
        DeclareLaunchArgument(
            "rviz_delay", default_value="22.0",
            description="Seconds before RViz starts - long enough that Nav2 has "
                        "finished its lifecycle transitions first.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Open RViz with costmaps, path, particles and the goal tool.",
        ),
        localization,
        navigation,
        rviz,
    ])
