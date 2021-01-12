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

import os
import launch
import launch_ros.actions
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    autoware_auto_avp_demo_pkg_path = get_package_share_directory(
        'autoware_auto_avp_demo'
    )

    pc_filter_transform_param_file_path = os.path.join(
        get_package_share_directory('point_cloud_filter_transform_nodes'),
        'param',
        'vlp16_lexus_filter_transform.param.yaml'
    )

    scan_downsampler_param_file_path = os.path.join(
        autoware_auto_avp_demo_pkg_path,
        'param',
        'scan_downsampler_ms3.param.yaml'
    )

    ndt_localizer_param_file_path = os.path.join(
        autoware_auto_avp_demo_pkg_path,
        'param',
        'ndt_localizer.param.yaml'
    )

    ndt_localizer_init_hack_param = {
        "init_hack.quaternion.x": 0.0,
        "init_hack.quaternion.y": 0.0,
        "init_hack.quaternion.z": 0.91,
        "init_hack.quaternion.w": 0.41,
        "init_hack.translation.x": -25.0,
        "init_hack.translation.y": 102.5,
        "init_hack.translation.z": 0.0,
        "init_hack.enabled": True
    }

    map_pcd_file = os.path.join(
        autoware_auto_avp_demo_pkg_path,
        'data',
        'autonomoustuff_parking_lot.pcd'
    )

    map_yaml_file = os.path.join(
        autoware_auto_avp_demo_pkg_path,
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

    urdf_file_path = os.path.join(
        get_package_share_directory('lexus_rx_450h_description'),
        'urdf',
        'lexus_rx_450h_vehicle.urdf'
    )

    lanelet2_osm_file_path = os.path.join(
        autoware_auto_avp_demo_pkg_path,
        'data',
        'autonomoustuff_parking_lot.osm'
    )

    rosbag_file_path_default = os.path.join(
        os.environ['HOME'],
        'rosbag2_2020_09_23-15_58_07'
    )

    if os.environ["ROS_DISTRO"] > "dashing":
        rosbag_file_path_default = os.path.join(
            rosbag_file_path_default,
            'rosbag2_2020_09_23-15_58_07.db3'
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
                                autoware_auto_avp_demo_pkg_path,
                                'config',
                                'ms3.rviz'
                            )
        ),

        # Nodes

        launch_ros.actions.Node(
            package='point_cloud_filter_transform_nodes',
            node_executable='point_cloud_filter_transform_node_exe',
            node_name='filter_transform_vlp16_front',
            node_namespace='lidar_front',
            parameters=[pc_filter_transform_param_file_path],
            remappings=[("points_in", "points_raw")]
        ),

        launch_ros.actions.Node(
            package='point_cloud_filter_transform_nodes',
            node_executable='point_cloud_filter_transform_node_exe',
            node_name='filter_transform_vlp16_rear',
            node_namespace='lidar_rear',
            parameters=[pc_filter_transform_param_file_path],
            remappings=[("points_in", "points_raw")]
        ),

        launch_ros.actions.Node(
            package='ndt_nodes',
            node_executable='ndt_map_publisher_exe',
            node_namespace='localization',
            parameters=[ndt_map_publisher_param]
        ),

        launch_ros.actions.Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            arguments=[
                str(urdf_file_path)
            ]
        ),

        launch_ros.actions.Node(
            package='ndt_nodes',
            node_executable='p2d_ndt_localizer_exe',
            node_namespace='localization',
            node_name='p2d_ndt_localizer_node',
            parameters=[ndt_localizer_param_file_path,
                        ndt_localizer_init_hack_param],
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
            node_executable='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
        ),

        launch_ros.actions.Node(
            package='point_cloud_fusion_nodes',
            node_executable='pointcloud_fusion_node_exe',
            node_namespace='lidars',
            parameters=[point_cloud_fusion_param_file_path],
            remappings=[
                ("output_topic", "points_fused"),
                ("input_topic1", "/lidar_front/points_filtered"),
                ("input_topic2", "/lidar_rear/points_filtered")
            ]
        ),

        launch_ros.actions.Node(
            package='voxel_grid_nodes',
            node_executable='voxel_grid_node_exe',
            node_namespace='lidars',
            node_name='voxel_grid_cloud_node',
            parameters=[scan_downsampler_param_file_path],
            remappings=[
                ("points_in", "points_fused"),
                ("points_downsampled", "points_fused_downsampled")
            ]
        ),

        launch_ros.actions.Node(
            package='lanelet2_map_provider',
            node_executable='lanelet2_map_provider_exe',
            node_namespace='had_maps',
            parameters=[
                {'map_osm_file': lanelet2_osm_file_path}
            ]
        ),

        launch_ros.actions.Node(
            package='lanelet2_map_provider',
            node_executable='lanelet2_map_visualizer_exe',
            node_namespace='had_maps'
        ),

        # play the rosbag data
        ExecuteProcess(
            cmd=['ros2',
                'bag',
                'play',
                LaunchConfiguration('rosbag_file_path')
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
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', LaunchConfiguration("rviz_cfg_path")],
            condition=IfCondition(LaunchConfiguration("launch_rviz"))
        )
    ])
