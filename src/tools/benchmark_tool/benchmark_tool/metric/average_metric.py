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

import os
from benchmark_tool.metric.metric import Metric
from benchmark_tool.utility import error, info


class AverageMetric(Metric):
    """
    The AverageMetric class represents the computation of the average of a metric.

    It reads a file providing float or integer value and computes a simple average on these values.
    It creates an output file containing the minimum, average, and maximum.
    """

    DEFAULT_RESULTS_FILE = "average.txt"
    DEFAULT_OUTPUT_FILE = "average_metric.txt"

    def __init__(self, node, result_folder, output_folder,
                 result_file_name=None,
                 metric_file_name=None):
        """
        Create a AverageMetric object.

        @param node: ROS2 node
        @type  node: rclpy.node.Node
        @param result_folder: The path on filesystem for the data to be analyzed
        @type  result_folder: str
        @param output_folder: The path on filesystem for the output files after the computation of
            the metric
        @type  output_folder: str
        @param result_file_name: The file name for the file containing the data used to compute
            this metric
        @type  result_file_name: str
        @param metric_file_name: The file name for the computed metric metric
        @type  metric_file_name: str
        """
        super(AverageMetric, self).__init__(result_folder, output_folder)
        self.node = node

        if result_file_name is None:
            self._result_file_name = self.DEFAULT_RESULTS_FILE
        else:
            self._result_file_name = result_file_name

        if metric_file_name is None:
            self._metric_file_name = self.DEFAULT_OUTPUT_FILE
        else:
            self._metric_file_name = metric_file_name

    def compute_metric(self, reportMsg=""):
        """
        Start the computation of the metric.

        It reads the file that shall contain one value for each line, computes the average and
        creates a new file with the minimum value, the average and the maximum value in this order.

        @param reportMsg: The message to print. It must have three %f fields for min, average and
            max value, in this order.
        @return: True on success, False on failure
        """
        filename = self._result_folder + "/" + self._result_file_name

        if os.path.isfile(filename):
            # Compute the average
            try:
                file = open(filename, "r")
                file_lines = file.readlines()
            except Exception as e:
                error(self.node, "%s" % str(e))
                return False
            finally:
                file.close()

            av_value = 0.0
            min_value = float("inf")
            max_value = -float("inf")

            for val in file_lines:

                val = float(val.strip("\n"))

                if val < min_value:
                    min_value = val

                if val > max_value:
                    max_value = val

                av_value += val

            av_value = av_value / len(file_lines)

            # Display summary in milliseconds
            min_value /= 1e3
            av_value /= 1e3
            max_value /= 1e3

            if not self._save_metric_file(min_value, av_value, max_value):
                error(self.node, "Error saving metric file.")

            # Print average to the node output
            if("" == reportMsg):
                reportMsg = "metric : \nMin: %.3f\nAverage: %.3f \nMax: %.3f"
            info(self.node,
                 reportMsg % (min_value, av_value, max_value))

        return True

    def _save_metric_file(self, min_value, average_value, max_value):
        """
        Save the output files with the minimum value, the average and the maximum value.

        @param min_value: The minimum value to be saved in the output file
        @type  min_value: int
        @param average_value: The average value to be saved in the output file
        @type  average_value: int
        @param max_value: The maximum value to be saved in the output file
        @type  max_value: int
        @return: True on success, False on failure
        """
        filename = self._output_folder + "/" + self._metric_file_name
        try:
            file = open(filename, "w")
            file.write(str(min_value) + "\n")
            file.write(str(average_value) + "\n")
            file.write(str(max_value) + "\n")
        except Exception as e:
            error(self.node, "%s" % str(e))
            return False
        finally:
            file.close()

        return True
