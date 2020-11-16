# Copyright 2019-2021 Apex.AI, Inc., Arm Limited
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    """
    Launch a minimal joystick + LGSVL demo.

    Under the default configuration, the joystick
    translator outputs and the LGSVL interface expects RawControlCommand. Controlling the vehicle
    can happen via the gamepad triggers and left joystick.
    """
    # --------------------------------- Params -------------------------------

    # Default joystick translator params
    joy_translator_param = DeclareLaunchArgument(
        'joy_translator_param',
        default_value=[
            get_share_file('joystick_vehicle_interface_nodes',
                           'param/logitech_f310.default.param.yaml')
        ],
        description='Path to config file for joystick translator')

    # Default lgsvl_interface params
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param',
        default_value=[
            get_share_file('lgsvl_interface', 'param/lgsvl.param.yaml')
        ],
        description='Path to config file for lgsvl interface')

    # -------------------------------- Nodes-----------------------------------

    # Include Joystick launch
    joystick_launch_file_path = get_share_file('joystick_vehicle_interface_nodes',
                                               'launch/joystick_vehicle_interface_node.launch.py')
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch_file_path),
        launch_arguments=[
            (
                "joy_translator_param",
                LaunchConfiguration("joy_translator_param")
            )
        ]
    )

    # Include LGSVL interface launch
    lgsvl_launch_file_path = get_share_file('lgsvl_interface',
                                            'launch/lgsvl.launch.py')
    lgsvl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lgsvl_launch_file_path),
        launch_arguments=[
            (
                "lgsvl_interface_param",
                LaunchConfiguration("lgsvl_interface_param")
            )
        ]
    )

    return LaunchDescription([
      joy_translator_param,
      lgsvl_interface_param,
      joystick,
      lgsvl
    ])
