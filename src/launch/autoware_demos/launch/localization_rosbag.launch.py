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
import launch
import launch_ros.actions
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    autoware_demos_pkg_path = get_package_share_directory(
        'autoware_demos'
    )

    pc_filter_transform_param_file_path = os.path.join(
        get_package_share_directory('point_cloud_filter_transform_nodes'),
        'param',
        'vlp16_lexus_filter_transform.param.yaml'
    )

    scan_downsampler_param_file_path = os.path.join(
        get_package_share_directory('autoware_auto_launch'),
        'param',
        'scan_downsampler.param.yaml'
    )

    ndt_localizer_param_file_path = os.path.join(
        get_package_share_directory('autoware_auto_launch'),
        'param',
        'ndt_localizer.param.yaml'
    )

    ndt_localizer_init_hack_param = {
        "initial_pose.quaternion.x": 0.0,
        "initial_pose.quaternion.y": 0.0,
        "initial_pose.quaternion.z": 0.91,
        "initial_pose.quaternion.w": 0.41,
        "initial_pose.translation.x": -25.0,
        "initial_pose.translation.y": 102.5,
        "initial_pose.translation.z": 0.0,
        "initial_pose.enabled": True
    }

    map_pcd_file = os.path.join(
        autoware_demos_pkg_path,
        'data',
        'autonomoustuff_parking_lot.pcd'
    )

    map_yaml_file = os.path.join(
        autoware_demos_pkg_path,
        'data',
        'autonomoustuff_parking_lot.yaml'
    )

    ndt_map_publisher_param = {
        'map_pcd_file': map_pcd_file,
        'map_yaml_file': map_yaml_file,
        'map_frame': 'map',
        'map_config.capacity': 1000000,
        'map_config.min_point.x': -1000.0,
        'map_config.min_point.y': -1000.0,
        'map_config.min_point.z': -3.0,
        'map_config.max_point.x': 1000.0,
        'map_config.max_point.y': 1000.0,
        'map_config.max_point.z': 3.0,
        'map_config.voxel_size.x': 3.5,
        'map_config.voxel_size.y': 3.5,
        'map_config.voxel_size.z': 3.5,
        'viz_map': True
    }

    point_cloud_fusion_param_file_path = os.path.join(
        get_package_share_directory('point_cloud_fusion_nodes'),
        'param',
        'vlp16_sim_lexus_pc_fusion.param.yaml'
    )

    urdf_path = os.path.join(
        get_package_share_directory('lexus_rx_450h_description'),
        'urdf',
        'lexus_rx_450h_vehicle.urdf'
    )
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

    lanelet2_osm_file_path = os.path.join(
        autoware_demos_pkg_path,
        'data',
        'autonomoustuff_parking_lot.osm'
    )

    rosbag_file_path_default = os.path.join(
        os.environ['HOME'],
        'rosbag2_2020_09_23-15_58_07/rosbag2_2020_09_23-15_58_07.db3'
    )

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            'rosbag_file_path',
            default_value=rosbag_file_path_default,
            description='Path to the rosbag file to play'
        ),

        launch.actions.DeclareLaunchArgument(
            'launch_rviz',
            default_value='True',
            description='Launch RViz if it is true',
        ),

        launch.actions.DeclareLaunchArgument(
            "rviz_cfg_path",
            default_value=os.path.join(
                                get_package_share_directory('autoware_auto_launch'),
                                'config',
                                'avp.rviz'
                            )
        ),

        # Nodes

        launch_ros.actions.Node(
            package='point_cloud_filter_transform_nodes',
            executable='point_cloud_filter_transform_node_exe',
            name='filter_transform_vlp16_front',
            namespace='lidar_front',
            parameters=[pc_filter_transform_param_file_path],
            remappings=[("points_in", "points_raw")]
        ),

        launch_ros.actions.Node(
            package='point_cloud_filter_transform_nodes',
            executable='point_cloud_filter_transform_node_exe',
            name='filter_transform_vlp16_rear',
            namespace='lidar_rear',
            parameters=[pc_filter_transform_param_file_path],
            remappings=[("points_in", "points_raw")]
        ),

        launch_ros.actions.Node(
            package='ndt_nodes',
            executable='ndt_map_publisher_exe',
            namespace='localization',
            parameters=[ndt_map_publisher_param]
        ),

        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_file}],
        ),

        launch_ros.actions.Node(
            package='ndt_nodes',
            executable='p2d_ndt_localizer_exe',
            namespace='localization',
            name='p2d_ndt_localizer_node',
            parameters=[ndt_localizer_param_file_path,
                        ndt_localizer_init_hack_param,
                        {"load_initial_pose_from_parameters": True}],
            remappings=[
                ("points_in", "/lidars/points_fused_downsampled"),
                ("observation_republish", "/lidars/points_fused_viz")
            ]
        ),

        # Since we don't use an odometry source, odometry frame is statically defined to be
        # overlapping with the base_link.
        # TODO(yunus.caliskan): To be removed after #476
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
        ),

        launch_ros.actions.Node(
            package='point_cloud_fusion_nodes',
            executable='pointcloud_fusion_node_exe',
            namespace='lidars',
            parameters=[point_cloud_fusion_param_file_path],
            remappings=[
                ("output_topic", "points_fused"),
                ("input_topic1", "/lidar_front/points_filtered"),
                ("input_topic2", "/lidar_rear/points_filtered")
            ]
        ),

        launch_ros.actions.Node(
            package='voxel_grid_nodes',
            executable='voxel_grid_node_exe',
            namespace='lidars',
            name='voxel_grid_cloud_node',
            parameters=[scan_downsampler_param_file_path],
            remappings=[
                ("points_in", "points_fused"),
                ("points_downsampled", "points_fused_downsampled")
            ]
        ),

        launch_ros.actions.Node(
            package='lanelet2_map_provider',
            executable='lanelet2_map_provider_exe',
            namespace='had_maps',
            parameters=[
                {'map_osm_file': lanelet2_osm_file_path}
            ]
        ),

        launch_ros.actions.Node(
            package='lanelet2_map_provider',
            executable='lanelet2_map_visualizer_exe',
            namespace='had_maps'
        ),

        # play the rosbag data
        ExecuteProcess(
            cmd=[
                'ros2',
                'bag',
                'play',
                LaunchConfiguration('rosbag_file_path'),
            ],
            prefix=[
                'bash -c \'sleep 5; $0 $@\' '
            ],
            on_exit=[
                launch.actions.LogInfo(msg="rosbag2 exited")
            ]
        ),

        # launch RViz visualization
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration("rviz_cfg_path")],
            condition=IfCondition(LaunchConfiguration("launch_rviz"))
        )
    ])
