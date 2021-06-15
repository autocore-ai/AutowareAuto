#! /usr/bin/env python3

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

from abc import ABCMeta, abstractmethod


class BenchmarkTask(metaclass=ABCMeta):
    """
    The BenchmarkTask class is a generic interface to build a task.

    A generic task holds a dataset containing the data used for the benchmark, has information
    about the input/output topic from the blackbox system to benchmark and knows where to save the
    benchmark results.
    """

    def __init__(self, node):
        """
        Create a BenchmarkTask object.

        @param node: ROS2 node
        @type  node: rclpy.node.Node
        """
        super(BenchmarkTask, self).__init__()
        self._dataset = None
        self._output_formatter = None
        self.node = node

    @abstractmethod
    def init(self):
        """
        Initialize the task structure.

        @return: True on success, False on failure
        """
        pass

    @abstractmethod
    def run(self):
        """
        Run the task.

        This function is called repeatedly in a loop until the task has finished its operations.

        @return: True if the task has not finished operation, False when the task has nothing more
            to do
        """
        pass

    @abstractmethod
    def compute_results(self):
        """
        Compute the final benchmark results.

        @return: None
        """
        pass
