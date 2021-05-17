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


class Metric(metaclass=ABCMeta):
    """
    The Metric class is a generic interface.

    It holds the path on the filesystem for the data to be analyzed and a path for the output to be
    generated, if any.
    """

    def __init__(self, result_folder, output_folder):
        """
        Create a Metric object.

        @param self: The object pointer
        @param result_folder: The path on filesystem for the data to be analyzed
        @type  result_folder: str
        @param output_folder: The path on filesystem for the output files after the computation of
            the metric
        @type  output_folder: str
        """
        super(Metric, self).__init__()
        self._result_folder = result_folder
        self._output_folder = output_folder

    @abstractmethod
    def compute_metric(self):
        """
        Start the computation of the metric.

        @param self: The object pointer
        @return: True on success, False on failure
        """
        pass
