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

"""Launch a few nodes for easy debugging and testing of the kinematic tracker."""

import launch
import launch_ros.actions


def generate_launch_description():
    driver = launch_ros.actions.Node(
        package='velodyne_node', node_executable='velodyne_block_node_exe',
        node_name='vlp16_front', node_namespace='lidar_front')
    classifier = launch_ros.actions.Node(
        package='ray_ground_classifier_nodes',
        node_executable='ray_ground_classifier_block_node_exe',
        node_name='ray_ground_classifier',
        remappings=[("points_in", "lidar_front/points_filtered")])
    return launch.LaunchDescription([driver, classifier])
