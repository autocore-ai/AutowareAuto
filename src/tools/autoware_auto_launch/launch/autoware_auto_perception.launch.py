# Copyright 2021 the Autoware Foundation
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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch perception nodes.

     * euclidean_cluster
     * off_map_obstacles_filter
     * ray_ground_classifier
    """
    autoware_auto_launch_pkg_prefix = get_package_share_directory(
        'autoware_auto_launch')
    euclidean_cluster_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/euclidean_cluster.param.yaml')
    off_map_obstacles_filter_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/off_map_obstacles_filter.param.yaml')
    ray_ground_classifier_param_file = os.path.join(
        autoware_auto_launch_pkg_prefix, 'param/ray_ground_classifier.param.yaml')

    # Arguments
    with_obstacles_param = DeclareLaunchArgument(
        'with_obstacles',
        default_value='True',
        description='Enable obstacle detection'
    )
    euclidean_cluster_param = DeclareLaunchArgument(
        'euclidean_cluster_param_file',
        default_value=euclidean_cluster_param_file,
        description='Path to config file for Euclidean Clustering'
    )
    off_map_obstacles_filter_param = DeclareLaunchArgument(
        'off_map_obstacles_filter_param_file',
        default_value=off_map_obstacles_filter_param_file,
        description='Path to parameter file for off-map obstacle filter'
    )
    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )

    # Nodes
    euclidean_clustering = Node(
        package='euclidean_cluster_nodes',
        executable='euclidean_cluster_node_exe',
        namespace='perception',
        condition=IfCondition(LaunchConfiguration('with_obstacles')),
        parameters=[LaunchConfiguration('euclidean_cluster_param_file')],
        remappings=[
            ("points_in", "points_nonground")
        ]
    )
    off_map_obstacles_filter = Node(
        package='off_map_obstacles_filter_nodes',
        name='off_map_obstacles_filter_node',
        namespace='perception',
        executable='off_map_obstacles_filter_nodes_exe',
        condition=IfCondition(LaunchConfiguration('with_obstacles')),
        parameters=[LaunchConfiguration('off_map_obstacles_filter_param_file')],
        output='screen',
        remappings=[
            ('bounding_boxes_in', 'lidar_bounding_boxes'),
            ('bounding_boxes_out', 'lidar_bounding_boxes_filtered'),
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
        ]
    )
    ray_ground_classifier = Node(
        package='ray_ground_classifier_nodes',
        executable='ray_ground_classifier_cloud_node_exe',
        namespace='perception',
        condition=IfCondition(LaunchConfiguration('with_obstacles')),
        parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
        remappings=[("points_in", "/lidars/points_fused")]
    )

    return LaunchDescription([
        euclidean_cluster_param,
        ray_ground_classifier_param,
        with_obstacles_param,
        off_map_obstacles_filter_param,
        euclidean_clustering,
        ray_ground_classifier,
        off_map_obstacles_filter,
    ])
