# Copyright 2018 the Autoware Foundation
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
import ament_index_python
import launch_ros.actions
import launch_testing
import lidar_integration


def generate_test_description():
    PORT = lidar_integration.get_open_port()

    # The node under test and the checker node that will pass/fail our tests:
    test_topic = "veloyne_cloud_node_test_topic"
    velodyne_cloud_node = launch_ros.actions.Node(
        package="velodyne_nodes",
        executable="velodyne_cloud_node_exe",
        name="vlp16_driver_node",
        namespace="lidar_front",
        parameters=[
            "{}/param/vlp16_test.param.yaml".format(
                ament_index_python.get_package_share_directory("velodyne_nodes")
            ),
            {
                "port": PORT,
                "cloud_size": 10700,
                "expected_num_subscribers": 1,
            }
        ],
        remappings=[("points_xyzi", test_topic)],
        arguments=["--model", "vlp16"]
    )

    pcl_checker = lidar_integration.make_pcl_checker(
        topic=test_topic,
        size=10700,
        period=50,
    )

    return lidar_integration.get_lidar_launch_description(
        test_nodes=[velodyne_cloud_node],
        checkers=[pcl_checker],
        other_actions=[
            launch_testing.actions.ReadyToTest()
        ],
        port=PORT
    )


# Test cases are created automatically by the lidar_integration package.  We just need to
# instantiate them
active = lidar_integration.make_active_tests()

after_shutdown = lidar_integration.make_post_shutdown_tests()
