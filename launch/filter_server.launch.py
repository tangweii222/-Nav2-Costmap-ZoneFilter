#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from launch.actions import OpaqueFunction


def create_lifecycle_node(context, *args, **kwargs):
    # 展開字串
    lifecycle_manager_name = LaunchConfiguration('lifecycle_manager_name').perform(context)
    filter_mask_name = LaunchConfiguration('filter_mask_server_name').perform(context)
    costmap_info_name = LaunchConfiguration('costmap_filter_info_server_name').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    autostart = LaunchConfiguration('autostart').perform(context) == 'true'

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
    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    mask_yaml = LaunchConfiguration('mask_yaml')
    params_file = LaunchConfiguration('params_file')
    filter_info_topic = LaunchConfiguration('filter_info_topic')
    mask_topic = LaunchConfiguration('mask_topic')
    filter_mask_server_name = LaunchConfiguration('filter_mask_server_name')
    costmap_filter_info_server_name = LaunchConfiguration('costmap_filter_info_server_name')
    lifecycle_manager_name = LaunchConfiguration('lifecycle_manager_name')


    # Declare launch arguments
    declare_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('mask_yaml', description='YAML file for the mask (pgm + metadata)'),
        DeclareLaunchArgument('params_file', description='ROS2 parameters file'),
        DeclareLaunchArgument('filter_info_topic', description='Topic name to publish CostmapFilterInfo'),
        DeclareLaunchArgument('mask_topic', description='Topic name to publish the filter mask'),
        # for server
        DeclareLaunchArgument('filter_mask_server_name', default_value='filter_mask_server'),
        DeclareLaunchArgument('costmap_filter_info_server_name', default_value='costmap_filter_info_server'),
        DeclareLaunchArgument('lifecycle_manager_name', default_value='lifecycle_manager_filter')
    ]
    
    # Use param substitution for dynamic yaml rewrite
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': mask_yaml,
        'topic_name': mask_topic,
        'filter_info_topic': filter_info_topic
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    lifecycle_nodes = [filter_mask_server_name, costmap_filter_info_server_name]

    # Define node group
    filter_group = GroupAction([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name=filter_mask_server_name,
            namespace=namespace,
            output='screen',
            parameters=[configured_params]
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name=costmap_filter_info_server_name,
            namespace=namespace,
            output='screen',
            parameters=[configured_params]
        ),
        OpaqueFunction(function=create_lifecycle_node)
    ])

    return LaunchDescription(declare_args + [filter_group])
