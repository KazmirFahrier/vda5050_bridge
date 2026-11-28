# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the fault manager with a parameter file and a fault catalogue."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('fault_manager')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'fault_manager.yaml'),
            description='YAML file with the fault manager parameters'),
        DeclareLaunchArgument(
            'catalog_file',
            default_value=os.path.join(share, 'config', 'fault_catalog.json'),
            description='JSON fault catalogue'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace of the robot (for multi-robot setups)'),
        Node(
            package='fault_manager',
            executable='fault_manager_node',
            name='fault_manager',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'catalog_file': LaunchConfiguration('catalog_file')},
            ],
        ),
    ])
