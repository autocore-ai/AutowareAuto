# Copyright 2020 Tier IV, Inc.
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

import os

from ament_index_python import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    map_data_prefix = get_package_share_directory('lanelet2_map_provider')

    # map_provider parameter file definition
    lanelet2_map_provider_param_file = os.path.join(
        map_data_prefix,
        "param",
        "lanelet2_map_provider.param.yaml")
    map_provider_param = DeclareLaunchArgument(
        'lanelet2_map_provider_param_file',
        default_value=lanelet2_map_provider_param_file,
        description='Path to parameter file for map provider'
    )
    # map_provider and visualizer node execution definition
    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_provider_exe',
        namespace='had_maps',
        parameters=[LaunchConfiguration('lanelet2_map_provider_param_file')]
    )
    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        executable='lanelet2_map_visualizer_exe',
        namespace='had_maps')
    # launch nodes
    return launch.LaunchDescription([
        map_provider_param,
        lanelet2_map_provider,
        lanelet2_map_visualizer])
