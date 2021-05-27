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
    PORT = lidar_integration.get_open_port()

    # The nodes under test:
    velodyne_block_node = launch_ros.actions.Node(
        package="velodyne_nodes",
        executable="velodyne_cloud_node_exe",
        name="vlp16_driver_node",
        namespace="lidar_front",
        parameters=[
            os.path.join(
                get_package_share_directory("velodyne_nodes"),
                "param",
                "vlp16_test.param.yaml"
            ),
            {
                "port": PORT,
                "cloud_size": 10700,
                "expected_num_subscribers": 1,
            }
        ],
        arguments=["--model", "vlp16"]
    )

    point_cloud_filter_transform_node = launch_ros.actions.Node(
        package="point_cloud_filter_transform_nodes",
        executable="point_cloud_filter_transform_node_exe",
        name="point_cloud_filter_transform_node",
        namespace="lidar_front",
        parameters=[
            os.path.join(
                get_package_share_directory('point_cloud_filter_transform_nodes'),
                "param",
                "tf_test.param.yaml"
            ),
            {
                "expected_num_subscribers": 1,
                "expected_num_publishers":  1,
            }
        ],
        remappings=[("points_in", "points_xyzi")]
    )

    filtered_points_checker = lidar_integration.make_pcl_checker(
        topic="points_filtered",
        size=16930,
        period=100,
        period_tolerance=1.0,
        size_tolerance=1.0
    )

    lidar_bl_publisher = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "lidar_front", "base_link"]
    )

    return lidar_integration.get_lidar_launch_description(
        test_nodes=[velodyne_block_node, point_cloud_filter_transform_node, lidar_bl_publisher],
        checkers=[filtered_points_checker],
        other_actions=[
            launch_testing.actions.ReadyToTest()
        ],
        port=PORT
    )


# Test cases are created automatically by the lidar_integration package.  We just need to
# instantiate them
active = lidar_integration.make_active_tests()

after_shutdown = lidar_integration.make_post_shutdown_tests()
