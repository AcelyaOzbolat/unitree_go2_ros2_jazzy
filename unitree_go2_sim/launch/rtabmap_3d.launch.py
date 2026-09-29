import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    parameters = [{
        'frame_id': 'base_link',
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan_cloud': True,
        'approx_sync': True,
        'use_sim_time': use_sim_time,
        'Reg/Strategy': '1', 
        'Grid/Sensor': '2',  
        'Icp/VoxelSize': '0.1', 
    }]

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=parameters,
        remappings=[
            ('scan_cloud', '/velodyne_points/points'), 
            ('odom', '/odom')
        ],
        arguments=['-d']
    )

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=parameters,
        remappings=[
            ('scan_cloud', '/velodyne_points/points'),
            ('odom', '/odom')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        rtabmap_node,
        rtabmap_viz
    ])
