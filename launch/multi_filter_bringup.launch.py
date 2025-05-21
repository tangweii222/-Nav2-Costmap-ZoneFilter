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
    params_dir = os.path.join(pkg, 'params')

    # Declare base arguments
    declare_args = [
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
    ]

    # Declare filter-specific arguments
    filter_types = ['zone', 'keepout']
    for name in filter_types:
        declare_args += [
            DeclareLaunchArgument(f'use_{name}', default_value='true'),
            DeclareLaunchArgument(f'{name}_params_yaml', default_value=os.path.join(params_dir, f'{name}_params.yaml')),
            DeclareLaunchArgument(f'{name}_filter_mask_server', default_value=f'{name}_filter_mask_server'),
            DeclareLaunchArgument(f'{name}_costmap_filter_info_server', default_value=f'{name}_costmap_filter_info_server'),
            DeclareLaunchArgument(f'{name}_lifecycle_manager', default_value=f'{name}_lifecycle_manager'),
        ]

    # Create reusable IncludeLaunchDescription for each filter type
    def create_filter_include(name_prefix):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(filter_server_path),
            condition=IfCondition(LaunchConfiguration(f'use_{name_prefix}')),
            launch_arguments={
                'namespace': LaunchConfiguration('namespace'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
                'params_file': LaunchConfiguration(f'{name_prefix}_params_yaml'),
                'filter_mask_server_name': LaunchConfiguration(f'{name_prefix}_filter_mask_server'),
                'costmap_filter_info_server_name': LaunchConfiguration(f'{name_prefix}_costmap_filter_info_server'),
                'lifecycle_manager_name': LaunchConfiguration(f'{name_prefix}_lifecycle_manager')
            }.items()
        )

    # Generate launch description
    return LaunchDescription(
        declare_args + [
            create_filter_include('zone'),
            create_filter_include('keepout')
        ]
    )
