# Copyright (c) 2020-2021, Arm Limited
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

import os
import launch
import launch_ros.actions
from os.path import join as joinPath
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pc_filter_transform_param_file_path = os.path.join(
        get_package_share_directory('point_cloud_filter_transform_nodes'),
        'param',
        'vlp16_sim_lexus_filter_transform.param.yaml'
    )

    scan_downsampler_param_file_path = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param',
        'autoware_academy_demo',
        'scan_downsampler.param.yaml'
    )

    ndt_localizer_param_file_path = os.path.join(
        get_package_share_directory('autoware_demos'),
        'param',
        'autoware_academy_demo',
        'ndt_localizer.param.yaml'
    )

    map_pcd_file_path = os.path.join(
        get_package_share_directory('autoware_demos'),
        'data',
        'autonomoustuff_parking_lot.pcd'
    )

    map_yaml_file_path = os.path.join(
        get_package_share_directory('autoware_demos'),
        'data',
        'autonomoustuff_parking_lot.yaml'
    )

    return launch.LaunchDescription([

        # benchmark_tool_nodes arguments

        launch.actions.DeclareLaunchArgument(
            'dataset_path',
            default_value=joinPath(os.environ['HOME'], 'kitti_data', '3d_bench'),
            description='Path of the dataset in the system',
        ),
        launch.actions.DeclareLaunchArgument(
            'input_topic',
            default_value='/lidar_front/points_filtered_downsampled',
            description='Input topic of the blackbox system to be benchmarked',
        ),
        launch.actions.DeclareLaunchArgument(
            'output_topic',
            default_value='/localization/ndt_pose',
            description='Output topic of the blackbox system to be benchmarked',
        ),
        launch.actions.DeclareLaunchArgument(
            'benchmarked_input_topic',
            default_value=[LaunchConfiguration('input_topic')],
            description='The input topic of the benchmarked node',
        ),
        launch.actions.DeclareLaunchArgument(
            'benchmarked_output_topic',
            default_value=[LaunchConfiguration('output_topic')],
            description='The output topic of the benchmarked node',
        ),
        launch.actions.DeclareLaunchArgument(
            'result_path',
            default_value=joinPath(os.environ['HOME'], 'benchmark_result'),
            description='',
        ),
        launch.actions.DeclareLaunchArgument(
            'force_end_at_frame_n',
            default_value='200',
            description='Limit the number of played frames (-1 means unlimited)',
        ),
        launch.actions.DeclareLaunchArgument(
            'node_name',
            default_value='benchmark_tool_node',
            description='The name of the node',
        ),
        launch.actions.DeclareLaunchArgument(
            'node_output',
            default_value='screen',
            description='Where to display running informations (screen or log)',
        ),
        launch.actions.DeclareLaunchArgument(
            'rosbag_record',
            default_value='False',
            description='Record on rosbag the input and output topic during the benchmark',
        ),
        launch.actions.DeclareLaunchArgument(
            'rosbag_record_subfolder',
            default_value='The subfolder on filesystem where to save the rosbag record file, it ' \
                          'must be a subfolder of result_path folder',
            description='Record on rosbag the input and output topic during the benchmark',
        ),
        launch.actions.DeclareLaunchArgument(
            'ros_info_record',
            default_value='False',
            description='Record ROS node topology and bandwidth information during the benchmark',
        ),
        launch.actions.DeclareLaunchArgument(
            'sys_info_record',
            default_value='False',
            description='Record system metrics during the benchmark',
        ),
        launch.actions.DeclareLaunchArgument(
            'cyclone_dds_info_record',
            default_value='False',
            description='Record DDS metrics during the benchmark',
        ),
        launch.actions.DeclareLaunchArgument(
            'rosbag_file_path',
            default_value=joinPath(os.environ['HOME'], 'rosbag2_2020_09_23-15_58_07'),
            description='Path to the rosbag file to play'
        ),
        launch.actions.DeclareLaunchArgument(
            'map_pcd_file',
            default_value=[map_pcd_file_path],
            description='Path to the pcd file of the map'
        ),
        launch.actions.DeclareLaunchArgument(
            'map_yaml_file',
            default_value=[map_yaml_file_path],
            description='Path to the yaml file related to the pcd file of the map'
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
            package='voxel_grid_nodes',
            executable='voxel_grid_node_exe',
            namespace='lidar_front',
            name='voxel_grid_cloud_node',
            parameters=[scan_downsampler_param_file_path],
            remappings=[
                ("points_in", "points_filtered"),
                ("points_downsampled", "points_filtered_downsampled")
            ]
        ),
        launch_ros.actions.Node(
            package='ndt_nodes',
            executable='ndt_map_publisher_exe',
            namespace='localization',
            parameters=[
                {'map_pcd_file': LaunchConfiguration('map_pcd_file')},
                {'map_yaml_file': LaunchConfiguration('map_yaml_file')},
                {'map_frame': 'map'},
                {'map_config.capacity': 1000000},
                {'map_config.min_point.x': -1000.0},
                {'map_config.min_point.y': -1000.0},
                {'map_config.min_point.z': -3.0},
                {'map_config.max_point.x': 1000.0},
                {'map_config.max_point.y': 1000.0},
                {'map_config.max_point.z': 3.0},
                {'map_config.voxel_size.x': 3.5},
                {'map_config.voxel_size.y': 3.5},
                {'map_config.voxel_size.z': 3.5},
                {'viz_map': True}
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
            package='ndt_nodes',
            executable='p2d_ndt_localizer_exe',
            namespace='localization',
            name='p2d_ndt_localizer_node',
            parameters=[ndt_localizer_param_file_path],
            remappings=[
                ("points_in", "/replay/lidar_front/points_filtered_downsampled")
            ]
        ),
        ExecuteProcess(
            cmd=['ros2',
                 'bag',
                 'play',
                 LaunchConfiguration('rosbag_file_path')
                 ],
            prefix=[
                'bash -c \'sleep 10; $0 $@\' '
            ]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    get_package_share_directory('benchmark_tool_nodes'),
                    '/benchmark_task.launch.py'
                ]
            ),
            launch_arguments={
                'benchmark_task': 'ndt_matching_task',
                'dataset_path': LaunchConfiguration('dataset_path'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'benchmarked_input_topic': LaunchConfiguration('benchmarked_input_topic'),
                'benchmarked_output_topic': LaunchConfiguration('benchmarked_output_topic'),
                'result_path': LaunchConfiguration('result_path'),
                'force_end_at_frame_n': LaunchConfiguration('force_end_at_frame_n'),
                'node_start_delay': '0',
                'node_name': LaunchConfiguration('node_name'),
                'node_output': LaunchConfiguration('node_output'),
                'rosbag_record': LaunchConfiguration('rosbag_record'),
                'rosbag_record_subfolder': LaunchConfiguration('rosbag_record_subfolder'),
                'ros_info_record': LaunchConfiguration('ros_info_record'),
                'sys_info_record': LaunchConfiguration('sys_info_record'),
                'cyclone_dds_info_record': LaunchConfiguration('cyclone_dds_info_record'),
            }.items()
        )
    ])
