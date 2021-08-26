# Copyright 2020-2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
import launch
import launch_ros.actions
import launch.substitutions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import ament_index_python
import os


def get_param(package_name, param_file):
    return os.path.join(ament_index_python.get_package_share_directory(package_name), param_file)


def generate_launch_description():

    # --------------------------------- Params -------------------------------

    # In combination 'raw', 'basic' and 'high_level' control
    # in what mode of control comands to operate in,
    # only one of them can be active at a time with a value
    control_command_param = DeclareLaunchArgument(
        'control_command',
        default_value="raw",  # use "raw", "basic" or "high_level"
        description='command control mode topic name')

    # Default joystick translator params
    joy_translator_param = DeclareLaunchArgument(
        'joy_translator_param',
        default_value=[
            get_param('joystick_vehicle_interface_nodes', 'param/logitech_f310_raw.param.yaml')
        ],
        description='Path to config file for joystick translator')

    # -------------------------------- Nodes-----------------------------------

    # joystick driver node
    joy = launch_ros.actions.Node(
        package='joy_linux',
        executable='joy_linux_node',
        output='screen')

    # joystick translator node
    joy_translator = launch_ros.actions.Node(
        package='joystick_vehicle_interface_nodes',
        executable='joystick_vehicle_interface_node_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('joy_translator_param'),
            # overwrite parameters from yaml here
            {"control_command": LaunchConfiguration('control_command')}
        ],
        remappings=[
            ("raw_command", "/vehicle/raw_command"),
            ("state_command", "/vehicle/state_command")
        ])

    ld = launch.LaunchDescription([
        control_command_param,
        joy_translator_param,
        joy,
        joy_translator])
    return ld
