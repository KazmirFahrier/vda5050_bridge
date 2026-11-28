# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Launch the fault gateway (web API) with a parameter file."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('fault_gateway'), 'config', 'fault_gateway.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML file with the gateway parameters'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace of the robot (for multi-robot setups)'),
        Node(
            package='fault_gateway',
            executable='fault_gateway',
            name='fault_gateway',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
