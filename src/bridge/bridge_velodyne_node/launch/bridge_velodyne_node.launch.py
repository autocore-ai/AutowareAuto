# Copyright 2018 Tier IV, Inc.
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

"""Launch Velodyne VLP16 driver node with ros1_bridge."""

import os

from ament_index_python import get_package_share_directory
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    """Launch VLP16 driver and ros1_bridge."""
    # parameter file defintion.
    vlp_test_param_file_path = os.path.join(
        get_package_share_directory('velodyne_node'),
        'param',
        'vlp16_test.param.yaml')
    parameter_args = " __params:=" + vlp_test_param_file_path

    # velodyne node execution definition.
    velodyne_node_runner = launch_ros.actions.Node(
        package='velodyne_node',
        node_executable='velodyne_cloud_node_exe',
        arguments=["/test_velodyne_node_cloud_front:=/raw_points",
                   parameter_args])

    # ros1 bridge runner definition.
    ros1_bridge_runner = launch_ros.actions.Node(
        package='ros1_bridge',
        node_executable='dynamic_bridge',
        arguments=["--bridge-all-topics"])

    return launch.LaunchDescription([
        velodyne_node_runner,
        ros1_bridge_runner])
