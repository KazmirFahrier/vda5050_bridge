# Copyright 2026 vda5050_bridge contributors
# SPDX-License-Identifier: Apache-2.0
"""Package setup for fault_gateway."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fault_gateway'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['static/*.html']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vda5050_bridge maintainers',
    maintainer_email='maintainer@example.com',
    description='REST API, event stream and status page for the fault manager.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fault_gateway = fault_gateway.gateway_node:main',
        ],
    },
)
