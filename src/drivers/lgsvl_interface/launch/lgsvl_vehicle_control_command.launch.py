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
    """
    Launches a minimal joystick + LGSVL demo. The joystick_vehicle_interface and the lgsvl_interface
    are modified via parameter remapping to use VehicleControlCommand as an output. The vehicle can
    be controlled by manipulating the left joystick of the gamepad.
    """
    # TODO(c.ho) The dictionary parameter remapping doesn't work for some inscrutable reason
    raise NotImplementedError
    # Nodes
    command_topic = "vehicle_control_command"
    joy = launch_ros.actions.Node(
        package='joy',
        node_executable='joy_node',
        output='screen')
    joy_translator = launch_ros.actions.Node(
        package='joystick_vehicle_interface',
        node_executable='joystick_vehicle_interface_exe',
        output='screen',
        parameters=[
            get_param('joystick_vehicle_interface', 'logitech_f310.default.param.yaml'),
            {
                "raw_command_topic": "null",
                "basic_command_topic": command_topic,
            },
        ])
    lgsvl_interface = launch_ros.actions.Node(
        package='lgsvl_interface',
        node_executable='lgsvl_interface_exe',
        output='screen',
        paramters=[
            get_param('lgsvl_interface', 'lgsvl.param.yaml'),
            {
                "raw_command": {"name": "null"},
                "basic_command": {"name": command_topic},
            },
        ])
    lgsvl_bridge = launch.actions.ExecuteProcess(cmd="rosbridge")

    ld = launch.LaunchDescription([
        joy,
        joy_translator,
        lgsvl_interface])
        # lgsvl_bridge]) # TODO(c.ho) bring back when ADE version is built correctly
    return ld
