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

"""Launch P2D NDT mapper."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch all nodes required for mapping. This launch file is for pure ndt-mapping.

    If odometry is available, remove the static tf publication and add the odometry node(s)
    to this launch file.
    """
    ndt_mapper_param_file = os.path.join(
        get_package_share_directory('ndt_mapping_nodes'),
        'param/ndt_mapper.param.yaml')

    scan_downsampler_param_file = os.path.join(
        get_package_share_directory('ndt_mapping_nodes'),
        'param/scan_downsampler.param.yaml')

    pc_filter_transform_param_file = os.path.join(
        get_package_share_directory('point_cloud_filter_transform_nodes'),
        'param/vlp16_sim_lexus_filter_transform.param.yaml')

    # Arguments
    pc_filter_transform_param = DeclareLaunchArgument(
        'pc_filter_transform_param_file',
        default_value=pc_filter_transform_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )
    ndt_mapper_param = DeclareLaunchArgument(
        'ndt_param_param_file',
        default_value=ndt_mapper_param_file,
        description='Path to config file for ndt mapper'
    )
    scan_downsampler_param = DeclareLaunchArgument(
        'scan_downsampler_param_file',
        default_value=scan_downsampler_param_file,
        description='Path to config file for lidar scan downsampler'
    )

    # Nodes
    filter_transform_vlp16_front = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16_front',
        namespace='lidar_front',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[("points_in", "points_xyzi")]
    )

    scan_downsampler = Node(
        package='voxel_grid_nodes',
        executable='voxel_grid_node_exe',
        name='voxel_grid_cloud_node',
        namespace='lidar_front',
        parameters=[LaunchConfiguration('scan_downsampler_param_file')],
        remappings=[
            ("points_in", "points_filtered"),
            ("points_downsampled", "points_filtered_downsampled")
        ]
    )
    ndt_mapper = Node(
        package='ndt_mapping_nodes',
        executable='ndt_mapper_node_exe',
        name='ndt_mapper_node',
        namespace='mapper',
        output='screen',
        parameters=[LaunchConfiguration('ndt_param_param_file')],
        remappings=[
            ("points_in", "/lidar_front/points_filtered_downsampled"),
            ("points_registered", "/lidar_front/points_registered")
        ]
    )

    # This is a hack to make the mapper purely rely on the ndt localizer without using any
    # odometry source.
    # TODO(yunus.caliskan): Revisit after #476
    odom_bl_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )

    return LaunchDescription([
        pc_filter_transform_param,
        scan_downsampler_param,
        ndt_mapper_param,
        filter_transform_vlp16_front,
        scan_downsampler,
        odom_bl_publisher,
        ndt_mapper
    ])
