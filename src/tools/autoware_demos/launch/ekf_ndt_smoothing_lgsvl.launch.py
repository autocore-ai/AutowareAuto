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

"""Launch P2D NDT localizer and map publisher nodes."""

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage
from pathlib import Path

import os


context = LaunchContext()


def get_package_share_directory(package_name):
    """Return the absolute path to the share directory of the given package."""
    return os.path.join(
        Path(FindPackage(package_name).perform(context)),
        'share',
        package_name)


def generate_launch_description():
    # Boilerplate to fetch the necessary parameter files:

    scan_downsampler_param_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param/autoware_academy_demo/scan_downsampler.param.yaml')
    ndt_localizer_param_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param/autoware_academy_demo/ndt_localizer.param.yaml')
    pc_filter_transform_param_file = os.path.join(
        get_package_share_directory('point_cloud_filter_transform_nodes'),
        'param/vlp16_sim_lexus_filter_transform.param.yaml')

    map_publisher_param_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param/autoware_academy_demo/map_publisher.param.yaml')

    ndt_cov_insertion_param_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param/ndt_smoothing/ndt_covariance_override.param.yaml')

    ndt_ekf_filtering_param_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param/ndt_smoothing/ndt_ekf_filtering.param.yaml')

    rviz_cfg_path = os.path.join(get_package_share_directory('autoware_demos'),
                                 'rviz2/autoware_academy_ekf_demo.rviz')

    pc_filter_transform_param = DeclareLaunchArgument(
        'pc_filter_transform_param_file',
        default_value=pc_filter_transform_param_file,
        description='Path to config file for Point Cloud Filter Nodes'
    )

    scan_downsampler_param = DeclareLaunchArgument(
        'scan_downsampler_param_file',
        default_value=scan_downsampler_param_file,
        description='Path to config file for lidar scan downsampler'
    )

    ndt_localizer_param = DeclareLaunchArgument(
        'ndt_localizer_param_file',
        default_value=ndt_localizer_param_file,
        description='Path to config file for ndt localizer'
    )

    ndt_cov_insertion_param = DeclareLaunchArgument(
        'ndt_cov_insertion_param_file',
        default_value=ndt_cov_insertion_param_file,
        description='Path to config file for covariance insertion params'
    )

    ndt_ekf_filtering_param = DeclareLaunchArgument(
        'ndt_ekf_filtering_param_file',
        default_value=ndt_ekf_filtering_param_file,
        description='Path to config file for covariance insertion params'
    )

    map_publisher_param = DeclareLaunchArgument(
        'map_publisher_param_file',
        default_value=map_publisher_param_file,
        description='Path to config file for Map Publisher'
    )

    # Node definitions.

    filter_transform_vlp16_front = Node(
        package='point_cloud_filter_transform_nodes',
        node_executable='point_cloud_filter_transform_node_exe',
        node_name='filter_transform_vlp16_front',
        node_namespace='lidar_front',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[("points_in", "points_raw")]
    )

    scan_downsampler = Node(
        package='voxel_grid_nodes',
        node_executable='voxel_grid_node_exe',
        node_namespace='lidar_front',
        node_name='voxel_grid_cloud_node',
        parameters=[LaunchConfiguration('scan_downsampler_param_file')],
        remappings=[
            ("points_in", "points_filtered"),
            ("points_downsampled", "points_filtered_downsampled")
        ]
    )

    ndt_localizer = Node(
        package='ndt_nodes',
        node_executable='p2d_ndt_localizer_exe',
        node_namespace='localization',
        node_name='p2d_ndt_localizer_node',
        parameters=[LaunchConfiguration('ndt_localizer_param_file')],
        remappings=[
            ("points_in", "/lidar_front/points_filtered_downsampled")
        ]
    )

    covariance_override_node = Node(
        package='covariance_insertion_node',
        node_executable='covariance_insertion_node_exe',
        node_namespace='localization',
        output="screen",
        node_name='covariance_insertion_node',
        parameters=[LaunchConfiguration('ndt_cov_insertion_param_file')],
        remappings=[
            ("messages", "/localization/ndt_pose"),
            ("messages_with_overriden_covariance", "ndt_pose_with_covariance")
        ]
    )

    ekf_smoother_node = Node(
        package='state_estimation_node',
        node_executable='state_estimation_node_exe',
        node_namespace='localization',
        output="screen",
        node_name='state_estimation_node',
        parameters=[LaunchConfiguration('ndt_ekf_filtering_param_file')],
        remappings=[
            ("filtered_state", "/localization/ndt_pose_filtered"),
        ]
    )

    map_publisher = Node(
        package='ndt_nodes',
        node_executable='ndt_map_publisher_exe',
        node_namespace='localization',
        parameters=[LaunchConfiguration('map_publisher_param_file')]
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
    )

    # Since we don't use an odometry source,
    # odometry frame is statically defined to be
    # overlapping with the base_link.
    # TODO(yunus.caliskan): To be removed after #476
    odom_bl_publisher = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )

    return LaunchDescription([
        map_publisher_param,
        pc_filter_transform_param,
        scan_downsampler_param,
        ndt_localizer_param,
        ndt_cov_insertion_param,
        ndt_ekf_filtering_param,
        filter_transform_vlp16_front,
        map_publisher,
        scan_downsampler,
        covariance_override_node,
        ekf_smoother_node,
        odom_bl_publisher,
        ndt_localizer,
        rviz2
    ])
