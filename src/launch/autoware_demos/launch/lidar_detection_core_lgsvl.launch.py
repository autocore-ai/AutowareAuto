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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.


"""
This launch file launches all the nodes necessary to produce DetectedObjects out of raw lidar
points from the SVL simulator. This does not launch rviz. It is meant to be included in other
launch files that require lidar object detection
"""


import os

from ament_index_python import get_package_share_directory
import launch.substitutions
from launch_ros.actions import Node
from launch.actions import Shutdown


def get_param_file(package_name, file_name):
    """Pass the given param file as a LaunchConfiguration."""
    file_path = os.path.join(
        get_package_share_directory(package_name),
        'param',
        file_name)
    return launch.substitutions.LaunchConfiguration(
        'params', default=[file_path])


def generate_launch_description():
    # point cloud filter transform param file shared by front and rear instances
    filter_transform_param = get_param_file(
        'point_cloud_filter_transform_nodes',
        'vlp16_sim_lexus_filter_transform.param.yaml')

    filter_transform_vlp16_front = Node(
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16_front',
        namespace='lidar_front',
        on_exit=Shutdown(),
        package='point_cloud_filter_transform_nodes',
        parameters=[filter_transform_param],
        remappings=[
            ("points_in", "points_raw")
        ])

    filter_transform_vlp16_rear = Node(
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16_rear',
        namespace='lidar_rear',
        on_exit=Shutdown(),
        package='point_cloud_filter_transform_nodes',
        parameters=[filter_transform_param],
        remappings=[
            ("points_in", "points_raw")
        ])

    pointcloud_fusion = Node(
        executable='pointcloud_fusion_node_exe',
        name='pointcloud_fusion',
        namespace='lidars',
        on_exit=Shutdown(),
        package='point_cloud_fusion_nodes',
        parameters=[get_param_file('point_cloud_fusion_nodes',
                                   'vlp16_sim_lexus_pc_fusion.param.yaml')],
        remappings=[
            ("output_topic", "points_filtered"),
            ("input_topic1", "/lidar_front/points_filtered"),
            ("input_topic2", "/lidar_rear/points_filtered")
        ]
    )

    ray_ground_classifier = Node(
        executable='ray_ground_classifier_cloud_node_exe',
        name='ray_ground_classifier',
        namespace='lidars',
        on_exit=Shutdown(),
        package='ray_ground_classifier_nodes',
        parameters=[get_param_file('ray_ground_classifier_nodes',
                                   'vlp16_sim_lexus_ray_ground.param.yaml')],
        remappings=[
            ("points_in", "points_filtered")
        ]
    )

    euclidean_clustering = Node(
        executable='euclidean_cluster_node_exe',
        name='euclidean_clustering',
        namespace='lidars',
        on_exit=Shutdown(),
        package='euclidean_cluster_nodes',
        parameters=[get_param_file('euclidean_cluster_nodes',
                                   'vlp16_sim_lexus_cluster.param.yaml')],
        remappings=[
            ("points_in", "points_nonground"),
            ("points_clustered", "cluster_points")
        ])

    return launch.LaunchDescription([
        filter_transform_vlp16_front,
        filter_transform_vlp16_rear,
        pointcloud_fusion,
        ray_ground_classifier,
        euclidean_clustering
    ])