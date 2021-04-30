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
#
# Developed by Apex.AI, Inc.

"""Launch P2D NDT localizer and map publisher nodes."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path

import os

def generate_launch_description():
    # Boilerplate to fetch the necessary parameter files:

    scan_downsampler_param_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param/autoware_academy_demo/scan_downsampler.param.yaml')
    ndt_localizer_param_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param/ndt_with_odom_state_estimator/ndt_localizer.param.yaml')

    pc_filter_transform_param_file = os.path.join(
        get_package_share_directory('point_cloud_filter_transform_nodes'),
        'param/vlp16_sim_lexus_filter_transform.param.yaml')

    map_publisher_param_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param/autoware_academy_demo/map_publisher.param.yaml')

    ndt_cov_insertion_param_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param/ndt_with_odom_state_estimator/ndt_covariance_override.param.yaml')

    ndt_ekf_filtering_param_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param/ndt_with_odom_state_estimator/ndt_ekf_filtering.param.yaml')

    lgsvl_param_file = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param/ndt_with_odom_state_estimator/lgsvl_interface.param.yaml')

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

    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_param_file,
        description='Path to config file for LGSVL Interface'
    )

    # Node definitions.

    filter_transform_vlp16_front = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16_front',
        namespace='lidar_front',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[("points_in", "points_raw")]
    )

    scan_downsampler = Node(
        package='voxel_grid_nodes',
        executable='voxel_grid_node_exe',
        namespace='lidar_front',
        name='voxel_grid_cloud_node',
        parameters=[LaunchConfiguration('scan_downsampler_param_file')],
        remappings=[
            ("points_in", "points_filtered"),
            ("points_downsampled", "points_filtered_downsampled")
        ]
    )

    ndt_localizer = Node(
        package='ndt_nodes',
        executable='p2d_ndt_localizer_exe',
        namespace='localization',
        name='p2d_ndt_localizer_node',
        parameters=[LaunchConfiguration('ndt_localizer_param_file')],
        remappings=[
            ("points_in", "/lidar_front/points_filtered_downsampled")
        ]
    )

    covariance_override_node = Node(
        package='covariance_insertion_nodes',
        executable='covariance_insertion_node_exe',
        namespace='localization',
        output="screen",
        name='covariance_insertion_node',
        parameters=[LaunchConfiguration('ndt_cov_insertion_param_file')],
        remappings=[
            ("messages", "/localization/ndt_pose"),
            ("messages_with_overriden_covariance", "ndt_pose_with_covariance")
        ]
    )

    lgsvl_interface = Node(
        package='lgsvl_interface',
        executable='lgsvl_interface_exe',
        namespace='vehicle',
        output='screen',
        parameters=[
            LaunchConfiguration('lgsvl_interface_param_file')
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "/vehicle/state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ]
    )

    ekf_smoother_node = Node(
        package='state_estimation_nodes',
        executable='state_estimation_node_exe',
        namespace='localization',
        output="screen",
        name='state_estimation_node',
        parameters=[LaunchConfiguration('ndt_ekf_filtering_param_file')],
        remappings=[
            ("filtered_state", "/localization/ndt_pose_filtered"),
        ]
    )

    map_publisher = Node(
        package='ndt_nodes',
        executable='ndt_map_publisher_exe',
        namespace='localization',
        parameters=[LaunchConfiguration('map_publisher_param_file')]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
    )

    urdf_pkg_prefix = get_package_share_directory('lexus_rx_450h_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/lexus_rx_450h.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )

    return LaunchDescription([
        map_publisher_param,
        pc_filter_transform_param,
        scan_downsampler_param,
        ndt_localizer_param,
        ndt_cov_insertion_param,
        ndt_ekf_filtering_param,
        lgsvl_interface_param,
        filter_transform_vlp16_front,
        map_publisher,
        scan_downsampler,
        covariance_override_node,
        ekf_smoother_node,
        lgsvl_interface,
        ndt_localizer,
        urdf_publisher,
        rviz2
    ])
