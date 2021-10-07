# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch the emergency_handler node."""
    emergency_handler_pkg_prefix = get_package_share_directory('emergency_handler')

    emergency_handler_param_file = os.path.join(
        emergency_handler_pkg_prefix, 'param/defaults.param.yaml')

    # Arguments
    emergency_handler_param = DeclareLaunchArgument(
        'emergency_handler_param_file',
        default_value=emergency_handler_param_file,
        description='Path to config file for Emergency Handler'
    )

    # Nodes
    emergency_handler = Node(
        package='emergency_handler',
        executable='emergency_handler_exe',
        namespace='system',
        parameters=[LaunchConfiguration('emergency_handler_param_file')],
        remappings=[
            ('input/autoware_state', '/autoware/state'),
            ('input/driving_capability', '/vehicle/driving_capability'),
            ('input/prev_control_command', '/vehicle/vehicle_command'),
            ('input/state_report', '/vehicle/state_report'),
            ('input/odometry', '/vehicle/odometry'),
            ('output/control_command', '/vehicle/emergency/vehicle_command'),
            ('output/state_command', '/vehicle/emergency/state_command'),
            ('output/is_emergency', '/vehicle/emergency/is_emergency'),
            ('output/hazard_status', '/vehicle/emergency/hazard_status'),
            ('output/diagnostics_err', '/diagnostics')
        ]
    )

    return LaunchDescription([
        emergency_handler_param,
        emergency_handler
    ])
