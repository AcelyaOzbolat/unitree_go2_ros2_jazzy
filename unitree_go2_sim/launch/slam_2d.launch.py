"""2D mapping: Velodyne cloud -> /scan -> slam_toolbox.

Run this alongside unitree_go2_launch.py. slam_toolbox owns the map -> odom
transform, which is why the static map -> odom publisher in unitree_go2_launch.py
must stay disabled.

    ros2 launch unitree_go2_sim unitree_go2_launch.py rviz:=false
    ros2 launch unitree_go2_sim slam_2d.launch.py

Drive the robot around, then save the map:

    ros2 run nav2_map_server map_saver_cli -f <path>/unitree_go2_sim/maps/simple_room
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")

    pkg_share = get_package_share_directory("unitree_go2_sim")
    slam_params = os.path.join(
        pkg_share, "config", "slam_toolbox", "mapper_params_online_async.yaml"
    )

    scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "scan_2d.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # slam_toolbox is a lifecycle node on Jazzy. Launched as a plain Node it stays
    # unconfigured: it never subscribes to /scan and never publishes /map or the
    # map -> odom transform. The configure/activate events below drive it up.
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_params,
            {
                "use_sim_time": use_sim_time,
                "use_lifecycle_manager": use_lifecycle_manager,
            },
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] slam_toolbox is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_2d",
        output="screen",
        arguments=["-d", os.path.join(pkg_share, "rviz", "slam_2d.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="Configure and activate slam_toolbox automatically.",
        ),
        DeclareLaunchArgument(
            "use_lifecycle_manager", default_value="false",
            description="Let an external lifecycle manager drive slam_toolbox instead.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Open RViz with the 2D map view. Launch the sim with rviz:=false to avoid two windows.",
        ),
        scan_launch,
        slam_toolbox_node,
        configure_event,
        activate_event,
        rviz_node,
    ])
