# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the VDA 5050 adapter with a parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('vda5050_nav2_adapter'), 'config', 'adapter.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML file with the adapter parameters'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace of the robot (for multi-robot setups)'),
        Node(
            package='vda5050_nav2_adapter',
            executable='vda5050_adapter',
            name='vda5050_adapter',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
