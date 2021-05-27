# Copyright 2020 the Autoware Foundation
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

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    driver = launch_ros.actions.Node(
        package='velodyne_nodes', executable='velodyne_cloud_node_exe',
        namespace='lidar_front',
        parameters=[get_package_share_directory('velodyne_nodes') +
                    '/param/vlp16_test.param.yaml'])
    classifier = launch_ros.actions.Node(
        package='ray_ground_classifier_nodes',
        executable='ray_ground_classifier_cloud_node_exe',
        name='ray_ground_classifier',
        parameters=[get_package_share_directory('ray_ground_classifier_nodes') +
                    '/param/vlp16_lexus.param.yaml'],
        remappings=[('points_in', '/lidar_front/points_xyzi')])
    return launch.LaunchDescription([driver, classifier])
