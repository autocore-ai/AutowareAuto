# Copyright 2020-2021, The Autoware Foundation
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

"""Launch Modules for Milestone 3 of the AVP 2020 Demo."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import launch.actions
import os


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Milestone 3 of the AVP 2020 Demo.

    More details about what is included can
    be found at https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/25.
    """
    avp_demo_pkg_prefix = get_package_share_directory('autoware_auto_avp_demo')
    euclidean_cluster_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/euclidean_cluster.param.yaml')
    lgsvl_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/lgsvl_interface.param.yaml')
    map_publisher_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/map_publisher.param.yaml')
#    odom_state_estimator_param_file = os.path.join(
#        avp_demo_pkg_prefix, 'param/odom_state_estimator.param.yaml')
    ray_ground_classifier_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/ray_ground_classifier.param.yaml')
    rviz_cfg_path = os.path.join(avp_demo_pkg_prefix, 'config/ms3.rviz')
    scan_downsampler_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/scan_downsampler_ms3.param.yaml')
    ndt_localizer_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/ndt_localizer.param.yaml')
    mpc_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/mpc.param.yaml')
    recordreplay_planner_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/recordreplay_planner.param.yaml')

    pc_filter_transform_pkg_prefix = get_package_share_directory(
        'point_cloud_filter_transform_nodes')
    pc_filter_transform_param_file = os.path.join(
        pc_filter_transform_pkg_prefix, 'param/vlp16_sim_lexus_filter_transform.param.yaml')

    point_cloud_fusion_node_pkg_prefix = get_package_share_directory(
        'point_cloud_fusion_nodes')
    point_cloud_fusion_node_param_file = os.path.join(
        point_cloud_fusion_node_pkg_prefix, 'param/vlp16_sim_lexus_pc_fusion.param.yaml')

    urdf_pkg_prefix = get_package_share_directory('lexus_rx_450h_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/lexus_rx_450h.urdf')

    # Arguments

    euclidean_cluster_param = DeclareLaunchArgument(
        'euclidean_cluster_param_file',
        default_value=euclidean_cluster_param_file,
        description='Path to config file for Euclidean Clustering'
    )
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_param_file,
        description='Path to config file for LGSVL Interface'
    )
    map_publisher_param = DeclareLaunchArgument(
        'map_publisher_param_file',
        default_value=map_publisher_param_file,
        description='Path to config file for Map Publisher'
    )
#    odom_state_estimator_param = DeclareLaunchArgument(
#        'odom_state_estimator_param_file',
#        default_value=odom_state_estimator_param_file,
#        description='Path to config file for Odometry State Estimator'
#    )
    pc_filter_transform_param = DeclareLaunchArgument(
        'pc_filter_transform_param_file',
        default_value=pc_filter_transform_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )
    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )
    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='True',
        description='Launch RVIZ2 in addition to other nodes'
    )
    ndt_localizer_param = DeclareLaunchArgument(
        'ndt_localizer_param_file',
        default_value=ndt_localizer_param_file,
        description='Path to config file for ndt localizer'
    )
    scan_downsampler_param = DeclareLaunchArgument(
        'scan_downsampler_param_file',
        default_value=scan_downsampler_param_file,
        description='Path to config file for lidar scan downsampler'
    )
    mpc_param = DeclareLaunchArgument(
        'mpc_param_file',
        default_value=mpc_param_file,
        description='Path to config file for MPC'
    )
    recordreplay_planner_param = DeclareLaunchArgument(
        'recordreplay_planner_param_file',
        default_value=recordreplay_planner_param_file,
        description='Path to config file for record/replay planner'
    )
    point_cloud_fusion_node_param = DeclareLaunchArgument(
        'point_cloud_fusion_node_param_file',
        default_value=point_cloud_fusion_node_param_file,
        description='Path to config file for point cloud fusion'
    )

    # Nodes

    euclidean_clustering = Node(
        package='euclidean_cluster_nodes',
        node_executable='euclidean_cluster_node_exe',
        node_namespace='perception',
        parameters=[LaunchConfiguration('euclidean_cluster_param_file')],
        remappings=[
            ("points_in", "points_nonground")
        ]
    )
    filter_transform_vlp16_front = Node(
        package='point_cloud_filter_transform_nodes',
        node_executable='point_cloud_filter_transform_node_exe',
        node_name='filter_transform_vlp16_front',
        node_namespace='lidar_front',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[("points_in", "points_raw")]
    )
    filter_transform_vlp16_rear = Node(
        package='point_cloud_filter_transform_nodes',
        node_executable='point_cloud_filter_transform_node_exe',
        node_name='filter_transform_vlp16_rear',
        node_namespace='lidar_rear',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[("points_in", "points_raw")]
    )
    # point cloud fusion runner to fuse front and rear lidar
    point_cloud_fusion_node = Node(
        package='point_cloud_fusion_nodes',
        node_executable='pointcloud_fusion_node_exe',
        node_namespace='lidars',
        parameters=[LaunchConfiguration('point_cloud_fusion_node_param_file')],
        remappings=[
            ("output_topic", "points_fused"),
            ("input_topic1", "/lidar_front/points_filtered"),
            ("input_topic2", "/lidar_rear/points_filtered")
        ]
    )
    lgsvl_interface = Node(
        package='lgsvl_interface',
        node_executable='lgsvl_interface_exe',
        node_namespace='vehicle',
        output='screen',
        parameters=[
            LaunchConfiguration('lgsvl_interface_param_file'),
            {"lgsvl.publish_tf": True}
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
    map_publisher = Node(
        package='ndt_nodes',
        node_executable='ndt_map_publisher_exe',
        node_namespace='localization',
        parameters=[LaunchConfiguration('map_publisher_param_file')]
    )
#    odom_state_estimator = Node(
#        package='robot_localization',
#        node_executable='ekf_node',
#        node_namespace='localization/odom',
#        parameters=[LaunchConfiguration('odom_state_estimator_param_file')]
#    )
    ray_ground_classifier = Node(
        package='ray_ground_classifier_nodes',
        node_executable='ray_ground_classifier_cloud_node_exe',
        node_namespace='perception',
        parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
        remappings=[("points_in", "/lidars/points_fused")]
    )
    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        condition=IfCondition(LaunchConfiguration('with_rviz')),
        remappings=[("initialpose", "/localization/initialpose")]
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        arguments=[str(urdf_path)]
    )

    scan_downsampler = Node(
        package='voxel_grid_nodes',
        node_executable='voxel_grid_node_exe',
        node_namespace='lidars',
        node_name='voxel_grid_cloud_node',
        parameters=[LaunchConfiguration('scan_downsampler_param_file')],
        remappings=[
            ("points_in", "points_fused"),
            ("points_downsampled", "points_fused_downsampled")
        ]
    )
    ndt_localizer = Node(
        package='ndt_nodes',
        node_executable='p2d_ndt_localizer_exe',
        node_namespace='localization',
        node_name='p2d_ndt_localizer_node',
        parameters=[LaunchConfiguration('ndt_localizer_param_file')],
        remappings=[
            ("points_in", "/lidars/points_fused_downsampled")
        ]
    )
    mpc = Node(
        package='mpc_controller_nodes',
        node_executable='mpc_controller_node_exe',
        node_name='mpc_controller',
        node_namespace='control',
        parameters=[LaunchConfiguration('mpc_param_file')]
    )
    recordreplay_planner = Node(
        package='recordreplay_planner_nodes',
        node_executable='recordreplay_planner_node_exe',
        node_name='recordreplay_planner',
        node_namespace='planning',
        parameters=[LaunchConfiguration('recordreplay_planner_param_file')],
        remappings=[
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('planned_trajectory', '/planning/trajectory'),
            ('obstacle_bounding_boxes', '/perception/lidar_bounding_boxes'),
        ]
    )

    nodes = [
        urdf_publisher,
        euclidean_clustering,
        filter_transform_vlp16_front,
        filter_transform_vlp16_rear,
        lgsvl_interface,
        map_publisher,
        ray_ground_classifier,
        scan_downsampler,
        ndt_localizer,
        mpc,
        recordreplay_planner,
        point_cloud_fusion_node,
        rviz2
    ]

    event_handlers = [
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=n,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())]
            )) for n in nodes]

    return LaunchDescription([
        euclidean_cluster_param,
        lgsvl_interface_param,
        map_publisher_param,
        pc_filter_transform_param,
        ray_ground_classifier_param,
        scan_downsampler_param,
        ndt_localizer_param,
        with_rviz_param,
        mpc_param,
        recordreplay_planner_param,
        point_cloud_fusion_node_param,
        *nodes,
        *event_handlers
    ])
