# Copyright 2018 Tier IV, Inc.
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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

import ament_index_python
import launch
import launch.actions
import launch_ros.actions

import lidar_integration


def generate_test_description(ready_fn):
    PORT = lidar_integration.get_open_port()

    # The nodes under test:
    velodyne_block_node = launch_ros.actions.Node(
        package="velodyne_node",
        node_executable="velodyne_cloud_node_exe",
        node_name="vlp16_test_node",
        node_namespace="lidar_front",
        parameters=[
            os.path.join(
                ament_index_python.get_package_share_directory("velodyne_node"),
                "vlp16_test.param.yaml"
            ),
            {
                "port": PORT,
                "topic": "points_raw"
            }
        ]
    )

    point_cloud_filter_transform_node = launch_ros.actions.Node(
        package="point_cloud_filter_transform_nodes",
        node_executable="point_cloud_filter_transform_node_exe",
        node_name="point_cloud_filter_transform_node",
        node_namespace="lidar_front",
        parameters=[
            os.path.join(
                ament_index_python.get_package_share_directory(
                    "point_cloud_filter_transform_nodes"),
                "param",
                "test.param.yaml"
            ),
            {
                "expected_num_ground_subscribers": 1,
                "expected_num_nonground_subscribers": 1,
                "raw_topic": "points_raw"
            }
        ]
    )

    filtered_points_checker = lidar_integration.make_pcl_checker(
        topic="points_filtered",
        size=16930,
        period=100,
        period_tolerance=1.0
    )

    return lidar_integration.get_lidar_launch_description(
        test_nodes=[velodyne_block_node, point_cloud_filter_transform_node],
        checkers=[filtered_points_checker],
        other_actions=[
            launch.actions.OpaqueFunction(function=lambda context: ready_fn())
        ],
        port=PORT
    )


# Test cases are created automatically by the lidar_integration package.  We just need to
# instantiate them
active = lidar_integration.make_active_tests()

after_shutdown = lidar_integration.make_post_shutdown_tests()
