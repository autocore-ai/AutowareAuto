# Copyright 2020 Silexica GmbH, Lichtstr. 25, Cologne, Germany. All rights reserved.
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

from ament_index_python import get_package_share_directory

import launch_ros.actions
import launch_testing

import lidar_integration


def generate_test_description():
    # The node under test and the checker node that will pass/fail our tests:
    test_topic = 'veloyne_cloud_node_test_topic'
    node = launch_ros.actions.Node(
        package='ray_ground_classifier_nodes',
        executable='ray_ground_classifier_cloud_node_exe',
        name='ray_ground_classifier',
        parameters=[os.path.join(
            get_package_share_directory('ray_ground_classifier_nodes'),
            'param/test.param.yaml'
        )],
        remappings=[
            ('points_in', test_topic)
        ])

    ray_ground_spoofer = launch_ros.actions.Node(
        package='lidar_integration',
        executable='point_cloud_mutation_spoofer_exe',
        arguments=[
            '--topic', test_topic,
            '--freq', '10',
            '--runtime', '10',
            '--mean', '300000',
            '--std', '50000',
        ],
    )

    ld, context = lidar_integration.get_point_cloud_mutation_launch_description(
        test_nodes=[node],
        checkers=[],  # TODO we only check that node does not crash for now
        topic=test_topic,
        other_actions=[
            launch_testing.actions.ReadyToTest()
        ],
        spoofer=ray_ground_spoofer)

    return ld, context


# Test cases are created automatically by the lidar_integration package.  We just need to
# instantiate them
active = lidar_integration.make_active_mutation_tests()

after_shutdown = lidar_integration.make_post_shutdown_mutation_tests()
