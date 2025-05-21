#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def create_lifecycle_node(context, *args, **kwargs):
    # 解析 args 傳入的 LaunchConfigurations
    filter_mask_name = args[0].perform(context)
    costmap_info_name = args[1].perform(context)
    lifecycle_manager_name = args[2].perform(context)
    namespace = args[3].perform(context)
    use_sim_time = args[4].perform(context) == 'true'
    autostart = args[5].perform(context) == 'true'

    return [
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name=lifecycle_manager_name,
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [filter_mask_name, costmap_info_name]
            }]
        )
    ]

def generate_launch_description():
    # Declare Launch Arguments
    declare_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        
        DeclareLaunchArgument('params_file', description='ROS2 parameters file'),

        DeclareLaunchArgument('filter_mask_server_name', default_value='filter_mask_server'),
        DeclareLaunchArgument('costmap_filter_info_server_name', default_value='costmap_filter_info_server'),
        DeclareLaunchArgument('lifecycle_manager_name', default_value='lifecycle_manager_filter'),
    ]

    # LaunchConfigurations (for readability)
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    filter_mask_server_name = LaunchConfiguration('filter_mask_server_name')
    costmap_filter_info_server_name = LaunchConfiguration('costmap_filter_info_server_name')
    lifecycle_manager_name = LaunchConfiguration('lifecycle_manager_name')

    # Group with Nodes
    filter_group = GroupAction([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name=filter_mask_server_name,
            namespace=namespace,
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name=costmap_filter_info_server_name,
            namespace=namespace,
            output='screen',
            parameters=[params_file]
        ),
        OpaqueFunction(
            function=create_lifecycle_node,
            args=[
                filter_mask_server_name,
                costmap_filter_info_server_name,
                lifecycle_manager_name,
                namespace,
                use_sim_time,
                autostart
            ]
        )
    ])

    return LaunchDescription(declare_args + [filter_group])
