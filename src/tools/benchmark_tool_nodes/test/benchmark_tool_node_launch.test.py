# Copyright 2021 Arm Limited
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

from benchmark_tool.dataset.kitti_3d_benchmark_dataset import Kitti3DBenchmarkDataset
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pathlib

import os
import pytest
import unittest


@pytest.mark.launch_test
def generate_test_description():

    share_dir = get_package_share_directory('benchmark_tool_nodes')

    artifacts_dir = os.path.join(share_dir, 'artifacts')
    for kitti_dir in Kitti3DBenchmarkDataset.KITTI_3D_BENCH_FOLDER_STRUCTURE:
        pathlib.Path(artifacts_dir + kitti_dir).mkdir(parents=False, exist_ok=True)

    result_path = os.path.join(share_dir, 'test_result')
    pathlib.Path(result_path).mkdir(parents=False, exist_ok=True)

    benchmark_tool = Node(
        package='benchmark_tool_nodes',
        executable='benchmark_tool_node.py',
        name='benchmark_tool_node',
        namespace='benchmark',
        output='screen',
        parameters=[
            {'benchmark_task': 'ray_ground_classifier_task'},
            {'dataset_path': artifacts_dir},
            {'input_topic': '/points_in'},
            {'output_topic': '/points_nonground'},
            {'benchmarked_input_topic': '/points_in'},
            {'benchmarked_output_topic': '/points_nonground'},
            {'result_path': result_path},
            {'force_end_at_frame_n': 1}
        ],
    )

    context = {'benchmark_tool': benchmark_tool}

    launch_description = LaunchDescription([
        benchmark_tool,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()])

    return launch_description, context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, benchmark_tool):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=benchmark_tool)
