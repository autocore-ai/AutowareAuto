# Copyright 2020-2021, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

import launch
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os

point_cloud_fusion_node_pkg_prefix = get_package_share_directory('point_cloud_fusion_nodes')
point_cloud_fusion_node_param_file = os.path.join(point_cloud_fusion_node_pkg_prefix,
                                                  'param/vlp16_sim_lexus_pc_fusion.param.yaml')


def generate_launch_description():
    point_cloud_fusion_nodes = Node(
        package='point_cloud_fusion_nodes',
        executable='pointcloud_fusion_node_exe',
        namespace='lidars',
        parameters=[point_cloud_fusion_node_param_file],
        remappings=[
            ("output_topic", "points_fused"),
            ("input_topic1", "/lidar_front/points_filtered"),
            ("input_topic2", "/lidar_rear/points_filtered")
        ]
    )

    return launch.LaunchDescription([point_cloud_fusion_nodes])
