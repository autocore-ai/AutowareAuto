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
import ament_index_python
import os


def get_param(package_name, param_file):
    return os.path.join(ament_index_python.get_package_share_directory(package_name), param_file)


def generate_launch_description():
    # CLI
    joy_translator_param = launch.actions.DeclareLaunchArgument(
        'joy_translator_param',
        default_value=[
            get_param('joystick_vehicle_interface', 'logitech_f310.default.param.yaml')
        ],
        description='Path to config file for joystick translator')
    # Nodes
    joy = launch_ros.actions.Node(
        package='joy',
        node_executable='joy_node',
        output='screen')
    joy_translator = launch_ros.actions.Node(
        package='joystick_vehicle_interface',
        node_executable='joystick_vehicle_interface_exe',
        output='screen',
        parameters=[launch.substitutions.LaunchConfiguration('joy_translator_param')])

    ld = launch.LaunchDescription([
        joy_translator_param,
        joy,
        joy_translator])
    return ld
