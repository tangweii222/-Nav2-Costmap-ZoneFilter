#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 預設設定
    default_pkg = get_package_share_directory('zone_filter')  # 地圖跟param還是同一個package
    # 預設參數檔案
    default_params_file = os.path.join(default_pkg, 'params', 'keepout_params.yaml')
    default_mask_file = os.path.join(default_pkg, 'maps', 'keepout_mask.yaml')
    default_filter_info_topic = '/keepout_filter_info'  # 🔥 改成keepout用的filter_info topic
    default_mask_topic = '/keepout_filter_mask'

    lifecycle_nodes = ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server']

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    mask_yaml_file = LaunchConfiguration('mask')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=default_params_file, description='Path to the parameters file')

    declare_mask_yaml_file_cmd = DeclareLaunchArgument(
        'mask', default_value=default_mask_file, description='Path to the filter mask yaml file')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True', description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container', description='Container name if using composition')

    declare_filter_info_topic_cmd = DeclareLaunchArgument(
        'filter_info_topic', default_value=default_filter_info_topic, description='Topic for keepout filter info')

    declare_mask_topic_cmd = DeclareLaunchArgument(
        'mask_topic', default_value=default_mask_topic, description='Topic for the filter mask')

    # 製作RewrittenYaml
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': mask_yaml_file,
        'topic_name': LaunchConfiguration('mask_topic'),
        'filter_info_topic': LaunchConfiguration('filter_info_topic')
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='keepout_filter_mask_server',  # 🔥 改名 keepout
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[configured_params]),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='keepout_costmap_filter_info_server',  # 🔥 改名 keepout
                namespace=namespace,
                output='screen',
                emulate_tty=True,
               parameters=[configured_params]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_keepout_filter',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            PushRosNamespace(
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('namespace'), "' != ''"])),
                namespace=namespace),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='keepout_filter_mask_server',
                        parameters=[configured_params]),
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::CostmapFilterInfoServer',
                        name='keepout_costmap_filter_info_server',
                        parameters=[configured_params]),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_keepout_filter',
                        parameters=[{'use_sim_time': use_sim_time},
                                    {'autostart': autostart},
                                    {'node_names': lifecycle_nodes}])
                ]
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mask_yaml_file_cmd)
    ld.add_action(declare_filter_info_topic_cmd)
    ld.add_action(declare_mask_topic_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
