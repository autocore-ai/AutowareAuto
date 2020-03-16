# Copyright 2020 Apex.AI, Inc.
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

    # In combination 'raw_command', 'basic_command' and 'high_level_command' control 
    # in what mode of control comands to operate in, 
    # only one of them can be active at a time with a topics name 
    # other should be blank/null which is achieved by used ="''"
    high_level_command_param = DeclareLaunchArgument(
        'high_level_command', 
        default_value="''", # use "high_level_command" or "''" 
        description='high_level_command control mode topic name')

    basic_command_param = DeclareLaunchArgument(
        'basic_command', 
        default_value="''", # use "vehicle_command" or "''" 
        description='basic_command control mode topic name')

    raw_command_param = DeclareLaunchArgument(
        'raw_command', 
        default_value="raw_command",  # use "raw_command" or "''" 
        description='raw_command control mode topic name')
    
    # Default joystick translator params
    joy_translator_param = DeclareLaunchArgument(
        'joy_translator_param',
        default_value=[
            get_param('joystick_vehicle_interface', 'logitech_f310.default.param.yaml')
        ],
        description='Path to config file for joystick translator')

    # -------------------------------- Nodes-----------------------------------
    
    # joystick driver node
    joy = launch_ros.actions.Node(
        package='joy',
        node_executable='joy_node',
        output='screen')

    # joystick translator node
    joy_translator = launch_ros.actions.Node(
        package='joystick_vehicle_interface',
        node_executable='joystick_vehicle_interface_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('joy_translator_param'),
            # overwrite parameters from yaml here
            {"high_level_command_topic" : LaunchConfiguration('high_level_command')},
            {"basic_command_topic" : LaunchConfiguration('basic_command')},
            {"raw_command_topic" :  LaunchConfiguration('raw_command')}
        ])

    ld = launch.LaunchDescription([
        high_level_command_param,
        basic_command_param,
        raw_command_param,
        joy_translator_param,
        joy,
        joy_translator])
    return ld
