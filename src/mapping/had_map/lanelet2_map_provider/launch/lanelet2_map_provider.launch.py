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
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():

    map_data_prefix = get_package_share_directory('lanelet2_map_provider')

    # map_provider parameter file definition
    lanelet2_map_provider_file_path = os.path.join(
        map_data_prefix,
        "param",
        "lanelet2_map_provider.param.yaml")
    map_provider_param_file = LaunchConfiguration(
        "params", default=[lanelet2_map_provider_file_path])

    # map provide map file arguments
    map_osm_file_path = os.path.join(
        map_data_prefix, "data/autonomoustuff_parking_lot_lgsvl.osm"
    )
    map_osm_file_param = DeclareLaunchArgument(
        'map_osm_file_arg',
        default_value=map_osm_file_path,
        description='OSM file describing semantic map'
    )

    # map_provide node execution definition
    lanelet2_map_provider_node_runner = launch_ros.actions.Node(
        package="lanelet2_map_provider",
        node_executable="lanelet2_map_provider_exe",
        node_namespace="had_maps",
        parameters=[map_provider_param_file,
                    {"map_osm_file": LaunchConfiguration('map_osm_file_arg')}])

    had_map_visualizer_runner = launch_ros.actions.Node(
        package="lanelet2_map_provider",
        node_executable="had_map_visualizer_exe",
        node_namespace="had_maps")

    # require a map file location
    return launch.LaunchDescription([
        map_osm_file_param,
        lanelet2_map_provider_node_runner,
        had_map_visualizer_runner])
