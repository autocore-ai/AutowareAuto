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
import yaml
from os.path import join as joinPath
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # euclidean cluster parameter file definition.
    vlp16_lexus_cluster_file_path = os.path.join(
        get_package_share_directory('euclidean_cluster_nodes'),
        'param',
        'vlp16_lexus_cluster.param.yaml')
    with open(vlp16_lexus_cluster_file_path, 'r') as file:
        configParamsEuclideanCluster = yaml.safe_load(file)['/**']['ros__parameters']

    # ray ground classifier parameter file definition.
    vlp16_lexus_ray_ground_file_path = os.path.join(
        get_package_share_directory('ray_ground_classifier_nodes'),
        'param',
        'vlp16_lexus.param.yaml')
    with open(vlp16_lexus_ray_ground_file_path, 'r') as file:
        configParamsRayGround = yaml.safe_load(file)['/**']['ros__parameters']

    return launch.LaunchDescription([

        # benchmark_tool_nodes arguments

        launch.actions.DeclareLaunchArgument(
            'dataset_path',
            default_value=joinPath(os.environ['HOME'], 'kitti_data', '3d_bench'),
            description='Path of the dataset in the system',
        ),
        launch.actions.DeclareLaunchArgument(
            'input_topic',
            default_value='/points_in',
            description='Input topic of the blackbox system to be benchmarked',
        ),
        launch.actions.DeclareLaunchArgument(
            'output_topic',
            default_value='/lidar_bounding_boxes',
            description='Output topic of the blackbox system to be benchmarked',
        ),
        launch.actions.DeclareLaunchArgument(
            'benchmarked_input_topic',
            default_value='/points_nonground',
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
            default_value='7481',
            description='Limit the number of played frames',
        ),
        launch.actions.DeclareLaunchArgument(
            'node_start_delay',
            default_value='0',
            description='',
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

        # Nodes

        launch_ros.actions.ComposableNodeContainer(
           package='rclcpp_components', executable='component_container',
           name='ray_euclidean_container', namespace=''
        ),
        launch_ros.actions.LoadComposableNodes(
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='ray_ground_classifier_nodes',
                    plugin=('autoware::perception::filters::ray_ground_classifier_nodes'
                            '::RayGroundClassifierCloudNode'),
                    name='ray_ground_classifier_node',
                    parameters=[configParamsRayGround, {"pcl_size": 210000}]
                ),
                launch_ros.descriptions.ComposableNode(
                    package='euclidean_cluster_nodes',
                    plugin=('autoware::perception::segmentation::euclidean_cluster_nodes'
                            '::EuclideanClusterNode'),
                    name='euclidean_cluster_node',
                    parameters=[configParamsEuclideanCluster, {"max_cloud_size": 210000}],
                    remappings=[("points_in", "/points_nonground")]
                ),
            ],
            target_container='ray_euclidean_container'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    get_package_share_directory('benchmark_tool_nodes'),
                    '/benchmark_task.launch.py'
                ]
            ),
            launch_arguments={
                'benchmark_task': 'euclidean_cluster_node_task',
                'dataset_path': LaunchConfiguration('dataset_path'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'benchmarked_input_topic': LaunchConfiguration('benchmarked_input_topic'),
                'benchmarked_output_topic': LaunchConfiguration('benchmarked_output_topic'),
                'result_path': LaunchConfiguration('result_path'),
                'force_end_at_frame_n': LaunchConfiguration('force_end_at_frame_n'),
                'node_start_delay': LaunchConfiguration('node_start_delay'),
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
