# Copyright 2021 the Autoware Foundation
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

"""Launch lidar segmentation and ray ground classifier nodes."""

import os

from ament_index_python import get_package_share_directory
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    """Launch lidar segmentation and ray ground classifier nodes."""
    # ray ground classifier parameter file definition.
    vlp16_lexus_ray_ground_file_path = os.path.join(
        get_package_share_directory('ray_ground_classifier_nodes'),
        'param',
        'vlp16_lexus.param.yaml')
    ray_ground_param_file = launch.substitutions.LaunchConfiguration(
        'params', default=[vlp16_lexus_ray_ground_file_path])

    # lidar segmentation node execution definition.
    apollo_lidar_segmentation_node_runner = launch_ros.actions.Node(
        package='apollo_lidar_segmentation_nodes',
        executable='apollo_lidar_segmentation_nodes_exe',
        remappings=[("points_in", "points_nonground")])

    # ros1 bridge runner definition.
    ray_ground_runner = launch_ros.actions.Node(
        package='ray_ground_classifier_nodes',
        executable='ray_ground_classifier_cloud_node_exe',
        parameters=[ray_ground_param_file],
        remappings=[("points_in", "lidar_front/points_filtered")])

    return launch.LaunchDescription([
        apollo_lidar_segmentation_node_runner,
        ray_ground_runner])
