# Copyright 2018 Tier IV, Inc.
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
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

import os

from ament_index_python import get_package_share_directory
import launch_ros.actions
import launch_testing
import lidar_integration


def generate_test_description():
    point_cloud_filter_transform_node = launch_ros.actions.Node(
        package="point_cloud_filter_transform_nodes",
        executable="point_cloud_filter_transform_node_exe",
        name="point_cloud_filter_transform_node",
        namespace="lidar_front",
        parameters=[
            os.path.join(
                get_package_share_directory('point_cloud_filter_transform_nodes'),
                'param/test.param.yaml'
            ),
            {
                "expected_num_subscribers": 1,
                "input_frame_id": "frameid",
                "output_frame_id": "frameid",
            }
        ],
        remappings=[("points_in", "points_xyzi")]
    )

    # Fallback on data from transform publisher if transform not found in test.param.yaml
    lidar_bl_publisher = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "lidar_front", "base_link"]
    )

    filtered_points_checker = lidar_integration.make_pcl_checker(
        topic="points_filtered",
        size=30000,
        size_tolerance=1.0,
        period=5000,
        period_tolerance=1.0,
        runtime=10.0,
    )

    spoofer_ = launch_ros.actions.Node(
        package="lidar_integration",
        executable="point_cloud_mutation_spoofer_exe",
        arguments=[
            "--topic", "/lidar_front/points_xyzi",
            "--freq", "10",
            "--runtime", "10",
            "--mean", "30000",
            "--std", "5000",
        ],
    )

    return lidar_integration.get_point_cloud_mutation_launch_description(
        test_nodes=[point_cloud_filter_transform_node, lidar_bl_publisher],
        checkers=[filtered_points_checker],
        other_actions=[
            launch_testing.actions.ReadyToTest()
        ],
        spoofer=spoofer_
    )


active = lidar_integration.make_active_mutation_tests()

after_shutdown = lidar_integration.make_post_shutdown_mutation_tests()
