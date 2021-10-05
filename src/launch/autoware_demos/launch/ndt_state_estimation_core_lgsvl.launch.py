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

import os

from ament_index_python import get_package_share_directory
import launch.substitutions
from launch_ros.actions import Node
from launch.actions import Shutdown


"""
This launch file launches all the nodes necessary to produce Odometry out of filtered lidar 
points from the SVL simulator. This does not launch rviz. It is meant to be included in other 
launch files that require lidar object detection. This launch file assumes 
point_cloud_filter_transform_nodes is running. 
"""


def get_param_file(package_name, file_name):
    """Pass the given param file as a LaunchConfiguration."""
    file_path = os.path.join(
        get_package_share_directory(package_name),
        'param',
        file_name)
    return launch.substitutions.LaunchConfiguration(
        'params', default=[file_path])


def generate_launch_description():
    # only the front lidar is used for localization and thus needs downsampling
    voxel_grid_downsampling = Node(
        executable='voxel_grid_node_exe',
        name='voxel_grid_downsampling',
        namespace='lidar_front',
        on_exit=Shutdown(),
        package='voxel_grid_nodes',
        parameters=[get_param_file('autoware_demos',
                                   'autoware_academy_demo/scan_downsampler.param.yaml')],
        remappings=[
            ("points_in", "points_filtered"),
            ("points_downsampled", "points_filtered_downsampled")
        ]
    )

    covariance_insertion = Node(
        executable='covariance_insertion_node_exe',
        name='covariance_insertion',
        namespace='localization',
        on_exit=Shutdown(),
        output="screen",
        package='covariance_insertion_nodes',
        parameters=[get_param_file('autoware_demos',
                                   'ndt_smoothing/ndt_covariance_override.param.yaml')],
        remappings=[
            ("messages", "/localization/ndt_pose"),
            ("messages_with_overriden_covariance", "ndt_pose_with_covariance")
        ]
    )

    map_publisher = Node(
        executable='ndt_map_publisher_exe',
        name='map_publisher',
        namespace='localization',
        on_exit=Shutdown(),
        package='ndt_nodes',
        parameters=[
            get_param_file('autoware_demos',
                           'autoware_academy_demo/map_publisher.param.yaml')
        ]
    )

    ndt_localization = Node(
        executable='p2d_ndt_localizer_exe',
        name='ndt_localization',
        namespace='localization',
        on_exit=Shutdown(),
        package='ndt_nodes',
        parameters=[get_param_file('autoware_demos',
                                   'autoware_academy_demo/ndt_localizer.param.yaml')],
        remappings=[
            ("points_in", "/lidar_front/points_filtered_downsampled")
        ]
    )

    state_estimation = Node(
        executable='state_estimation_node_exe',
        name='state_estimation',
        namespace='localization',
        on_exit=Shutdown(),
        output="screen",
        package='state_estimation_nodes',
        parameters=[get_param_file('autoware_demos',
                                   'ndt_smoothing/tracking.param.yaml')],
        remappings=[
            ("filtered_state", "/localization/odometry"),
        ]
    )

    return launch.LaunchDescription([
        voxel_grid_downsampling,
        covariance_insertion,
        map_publisher,
        ndt_localization,
        state_estimation
    ])