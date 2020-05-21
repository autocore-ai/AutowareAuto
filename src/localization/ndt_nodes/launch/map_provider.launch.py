# Copyright 2020 Apex.AI, Inc.
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
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    # map_provider parameter file definition
    ndt_map_provider_file_path = os.path.join(
        get_package_share_directory("ndt_nodes"),
        "param",
        "map_publisher.param.yaml")
    map_provider_param_file = launch.substitutions.LaunchConfiguration(
        "params", default=[ndt_map_provider_file_path])

    # map_provide node execution definition
    map_provider_node_runner = launch_ros.actions.Node(
        package="ndt_nodes",
        node_executable="ndt_map_publisher_exe",
        parameters=[map_provider_param_file])

    # map downsampler paramter file definition
    ndt_map_downsampler_file_path = os.path.join(
        get_package_share_directory("ndt_nodes"),
        "param",
        "pcl_map_voxel_grid_downsample.param.yaml")
    map_downsampler_param_file = launch.substitutions.LaunchConfiguration(
        "params", default=[ndt_map_downsampler_file_path])

    # map downsample node execution definition
    map_downsampler_node_runner = launch_ros.actions.Node(
        package="voxel_grid_nodes",
        node_executable="voxel_grid_cloud_node_exe",
        parameters=[map_downsampler_param_file])

    return launch.LaunchDescription([
        map_provider_node_runner,
        map_downsampler_node_runner])
