#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('zone_filter')
    filter_server_path = os.path.join(pkg, 'launch', 'filter_server.launch.py')

    # basic dirs
    maps_dir = os.path.join(pkg, 'maps')
    params_dir = os.path.join(pkg, 'params')
    # default params
    zone_mask_yaml_default = os.path.join(maps_dir, 'zone_mask.yaml')
    zone_params_yaml_default = os.path.join(params_dir, 'zone_params.yaml')
    keepout_mask_yaml_default = os.path.join(maps_dir, 'keepout_mask.yaml')
    keepout_params_yaml_default = os.path.join(params_dir, 'keepout_params.yaml')


    # launch args
    declare_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),

        DeclareLaunchArgument('use_zone', default_value='true'),
        DeclareLaunchArgument('use_keepout', default_value='true'),

        DeclareLaunchArgument('zone_mask_yaml', default_value=zone_mask_yaml_default),
        DeclareLaunchArgument('zone_params_yaml', default_value=zone_params_yaml_default),
        DeclareLaunchArgument('zone_filter_info_topic', default_value='/zone_filter_info'),
        DeclareLaunchArgument('zone_mask_topic', default_value='/zone_filter_mask'),
        DeclareLaunchArgument('zone_filter_mask_server', default_value='zone_filter_mask_server'),
        DeclareLaunchArgument('zone_costmap_filter_info_server', default_value='zone_costmap_filter_info_server'),
        DeclareLaunchArgument('zone_lifecycle_manager', default_value='zone_lifecycle_manager'),

        DeclareLaunchArgument('keepout_mask_yaml', default_value=keepout_mask_yaml_default),
        DeclareLaunchArgument('keepout_params_yaml', default_value=keepout_params_yaml_default),
        DeclareLaunchArgument('keepout_filter_info_topic', default_value='/keepout_filter_info'),
        DeclareLaunchArgument('keepout_mask_topic', default_value='/keepout_filter_mask'),
        DeclareLaunchArgument('keepout_filter_mask_server', default_value='keepout_filter_mask_server'),
        DeclareLaunchArgument('keepout_costmap_filter_info_server', default_value='keepout_costmap_filter_info_server'),
        DeclareLaunchArgument('keepout_lifecycle_manager', default_value='keepout_lifecycle_manager')
    ]

    zone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(filter_server_path),
        condition=IfCondition(LaunchConfiguration('use_zone')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'mask_yaml': LaunchConfiguration('zone_mask_yaml'),
            'params_file': LaunchConfiguration('zone_params_yaml'),
            'filter_info_topic': LaunchConfiguration('zone_filter_info_topic'),
            'mask_topic': LaunchConfiguration('zone_mask_topic'),
            'filter_mask_server_name': LaunchConfiguration('zone_filter_mask_server'),
            'costmap_filter_info_server_name': LaunchConfiguration('zone_costmap_filter_info_server'),
            'lifecycle_manager_name': LaunchConfiguration('zone_lifecycle_manager')
        }.items()
    )

    keepout_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(filter_server_path),
        condition=IfCondition(LaunchConfiguration('use_keepout')),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'mask_yaml': LaunchConfiguration('keepout_mask_yaml'),
            'params_file': LaunchConfiguration('keepout_params_yaml'),
            'filter_info_topic': LaunchConfiguration('keepout_filter_info_topic'),
            'mask_topic': LaunchConfiguration('keepout_mask_topic'),
            'filter_mask_server_name': LaunchConfiguration('keepout_filter_mask_server'),
            'costmap_filter_info_server_name': LaunchConfiguration('keepout_costmap_filter_info_server'),
            'lifecycle_manager_name': LaunchConfiguration('keepout_lifecycle_manager')
        }.items()
    )

    return LaunchDescription(declare_args + [zone_launch, keepout_launch])
