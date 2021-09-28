# Copyright 2020-2021 the Autoware Foundation
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
from launch.actions import DeclareLaunchArgument
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration


def get_param_file(package_name, file_name):
    """Pass the given param file as a LaunchConfiguration."""
    file_path = os.path.join(
        get_package_share_directory(package_name),
        'param',
        file_name)
    return launch.substitutions.LaunchConfiguration(
        'params', default=[file_path])


def generate_launch_description():
    use_ndt = DeclareLaunchArgument(
        'use_ndt',
        default_value='True',
        description='Set this to "False" to use Ground truth odometry instead of odom from NDT.'
                    ' Note the option is case sensitive. Use only "True" or "False"'
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
        ],
        condition=IfCondition(LaunchConfiguration('use_ndt'))
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

    vision_detections = Node(
            name='vision_detections',
            executable='ground_truth_detections_node_exe',
            package='ground_truth_detections',
            on_exit=Shutdown()
    )

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

    lgsvl_interface = Node(
        executable='lgsvl_interface_exe',
        namespace='vehicle',
        on_exit=Shutdown(),
        output='screen',
        package='lgsvl_interface',
        parameters=[
            get_param_file('autoware_demos', 'lgsvl_interface.param.yaml'),
            {"lgsvl.publish_tf": True}  # Needed for RViz and NDT initialization
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "/vehicle/state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ])

    map_publisher = Node(
        executable='ndt_map_publisher_exe',
        name='map_publisher',
        namespace='localization',
        on_exit=Shutdown(),
        package='ndt_nodes',
        parameters=[
            get_param_file('autoware_demos',
                           'autoware_academy_demo/map_publisher.param.yaml')
        ],
        condition=IfCondition(LaunchConfiguration('use_ndt'))
    )

    multi_object_tracker_using_lgsvl = Node(
        executable='multi_object_tracker_node_exe',
        name='multi_object_tracker',
        namespace='perception',
        on_exit=Shutdown(),
        package='tracking_nodes',
        parameters=[
            get_param_file('autoware_demos', 'multi_object_tracker.param.yaml'),
            {
                'use_ndt': False,
                'track_frame_id': "odom",
            }
        ],
        remappings=[
            ("detected_objects", "/lidars/lidar_detected_objects"),
            ("ego_state", "/vehicle/odom_pose"),
            ("classified_rois", "/perception/ground_truth_detections_2d")
        ],
        condition=LaunchConfigurationEquals('use_ndt', 'False')
    )

    multi_object_tracker_using_ndt = Node(
        executable='multi_object_tracker_node_exe',
        name='multi_object_tracker',
        namespace='perception',
        on_exit=Shutdown(),
        package='tracking_nodes',
        parameters=[
            get_param_file('autoware_demos', 'multi_object_tracker.param.yaml'),
            {
                'use_ndt': True,
                'track_frame_id': "map",
            }
        ],
        remappings=[
            ("detected_objects", "/lidars/lidar_detected_objects"),
            ("ego_state", "/localization/odometry"),
            ("classified_rois", "/perception/ground_truth_detections_2d")
        ],
        condition=IfCondition(LaunchConfiguration('use_ndt'))
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
        ],
        condition=IfCondition(LaunchConfiguration('use_ndt'))
    )

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
        ],
        condition=IfCondition(LaunchConfiguration('use_ndt'))
    )

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
        ],
        condition=IfCondition(LaunchConfiguration('use_ndt'))
    )

    # Setup robot state publisher
    vehicle_description_pkg_path = get_package_share_directory(
        'lexus_rx_450h_description')
    urdf_path = os.path.join(vehicle_description_pkg_path, 'urdf',
                             'lexus_rx_450h.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()
    robot_state_publisher_runner = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )

    lidar_projector = Node(
        name='lidar_projector',
        package='cluster_projection_node',
        executable='cluster_projection_node_exe',
        parameters=[get_param_file('cluster_projection_node',
                                   'cluster_projection_node.param.yaml')],
        remappings=[
            ("/clusters_in", "/perception/associated_detections"),
            ("/projected_clusters", "/track_creating_projections"),
        ],
        on_exit=Shutdown()
    )

    image_visualizer = Node(
        name='image_visualizer',
        package='detection_2d_visualizer',
        executable='detection_2d_visualizer_node_exe',
        on_exit=Shutdown(),
        parameters=[get_param_file('autoware_demos',
                                   'tracker_detection_visualization.param.yaml')],
        remappings=[
            ("/projections", "/track_creating_projections"),
            ("/rois", "perception/ground_truth_detections_2d"),
            ("/image_with_detections", "/image_with_detections")
        ]
    )

    # Run rviz
    examples_pkg_path = get_package_share_directory(
        'autoware_demos')
    rviz_cfg_path = os.path.join(examples_pkg_path, 'rviz2',
                                 'lidar_tracking.rviz')
    rviz_runner = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)])

    return launch.LaunchDescription([
        use_ndt,
        covariance_insertion,
        euclidean_clustering,
        vision_detections,
        filter_transform_vlp16_front,
        filter_transform_vlp16_rear,
        lgsvl_interface,
        map_publisher,
        multi_object_tracker_using_lgsvl,
        multi_object_tracker_using_ndt,
        ndt_localization,
        pointcloud_fusion,
        ray_ground_classifier,
        robot_state_publisher_runner,
        rviz_runner,
        state_estimation,
        voxel_grid_downsampling,
        lidar_projector,
        image_visualizer
    ])
