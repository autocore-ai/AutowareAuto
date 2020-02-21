# Copyright 2020, The Autoware Foundation
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

"""Launch Modules for Milestone 2 of the AVP 2020 Demo."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Milestone 2 of the AVP 2020 Demo.

    More details about what is included can
    be found at https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/24.
    """
    lgsvl_pkg_prefix = get_package_share_directory('lgsvl_interface')
    lgsvl_param_file = os.path.join(lgsvl_pkg_prefix, 'lgsvl.param.yaml')

    return LaunchDescription([
        # LGSVL Interface
        DeclareLaunchArgument(
            'lgsvl_interface_param',
            default_value=lgsvl_param_file,
            description='Path to config file for lgsvl interface'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lgsvl_pkg_prefix, '/lgsvl.launch.py']),
            launch_arguments={'lgsvl_interface_param': lgsvl_param_file}.items(),
        ),
    ])
