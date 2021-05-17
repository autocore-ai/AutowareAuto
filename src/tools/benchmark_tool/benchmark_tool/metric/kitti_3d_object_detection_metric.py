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
from benchmark_tool.kittiobjdetsdk import kittiobjeval
from benchmark_tool.utility import error, info


class Kitti3dObjectDetectionMetric(Metric):
    """
    The Kitti3dObjectDetectionMetric class computes the precision metric using the kitti format.

    Kitti analyzes three main type of objects: car, pedestrian and cyclist.
    The kitti benchmark distinguishes also three class of detection based on the size of the
    detected object, occlusion and truncation.
    These classes are: easy, moderate and hard.
    """

    OUTPUT_FILES_KITTIOBJEVAL = {
        "car": "stats_car_detection_3d.txt",
        "pedestrian": "stats_pedestrian_detection_3d.txt",
        "cyclist": "stats_cyclist_detection_3d.txt"
    }

    def __init__(self, node, result_folder, ground_truth_folder, output_folder):
        """
        Create a Kitti3dObjectDetectionMetric object.

        @param self: The object pointer
        @param node: ROS2 node
        @type  node: rclpy.node.Node
        @param result_folder: The path on filesystem for the data to be analyzed
        @type  result_folder: str
        @param ground_truth_folder: The path on filesystem for the kitti 3d object detection ground
            truth data
        @type  ground_truth_folder: str
        @param output_folder: The path on filesystem for the output files after the computation of
            the metric
        @type  output_folder: str
        """
        super(Kitti3dObjectDetectionMetric, self).__init__(result_folder,
                                                           output_folder)
        self._ground_truth_folder = ground_truth_folder
        self.node = node

    def compute_metric(self):
        """
        Start the computation of the metric.

        It uses the official C++ kitti sdk code for the computation, the code is wrapped into a
        Python module.

        @param self: The object pointer
        @return: True on success, False on failure
        """
        # Remove kitti sdk files from output folder to prevent errors
        for key in self.OUTPUT_FILES_KITTIOBJEVAL:
            file_path = self._output_folder + \
                self.OUTPUT_FILES_KITTIOBJEVAL[key]
            if os.path.exists(file_path):
                os.remove(file_path)

        # Call kitti obj evaluation sdk
        if not kittiobjeval.eval(str(self._ground_truth_folder),
                                 str(self._result_folder),
                                 str(self._output_folder),
                                 False,    # Compute 2d obj detection metric
                                 False,    # Compute ground obj detection metric
                                 True,     # Compute 3d obj detection metric
                                 True,     # Output to stdout
                                 False     # Create plot of the results
                                 ):
            error(self.node, "Error computing Kitti results.")
            return False

        if not self._parse_result_file("car"):
            return False

        if not self._parse_result_file("pedestrian"):
            return False

        if not self._parse_result_file("cyclist"):
            return False

        return True

    def _parse_result_file(self, result_class):
        """
        Parse the kitti sdk result file.

        Extract the 41 precision/recall scores and compute the final metric.

        @param self: The object pointer
        @param result_class: The type of the object: car, pedestrian, cyclist
        @type  result_class: str
        @return: True on success, False on failure
        """
        filename = self._output_folder + "/" + \
            self.OUTPUT_FILES_KITTIOBJEVAL[result_class]

        if os.path.isfile(filename):
            # Compute metric precision for class EASY, MODERATE, HARD
            try:
                file = open(filename, "r")
                file_lines = file.readlines()
            except Exception as e:
                error(self.node, "%s" % str(e))
                return False
            finally:
                file.close()

            if len(file_lines) < 3:
                error(self.node,
                      "Malformed result file: %s." % filename)
                return False
            # Compute precision from result file
            precision = {}
            precision["EASY"] = self._compute_precision(file_lines[0])
            precision["MODERATE"] = self._compute_precision(file_lines[1])
            precision["HARD"] = self._compute_precision(file_lines[2])

            if (precision["EASY"] == -1) or (precision["MODERATE"] == -1) \
                    or (precision["HARD"] == -1):
                error(self.node,
                      "Malformed lines file: %s.", filename)
                return False

            # Print precision to the node output
            info(self.node, "Kitti 3D object evaluation for label: %s\n"
                 "Easy: %.2f \nModerate: %.2f \nHard: %.2f" %
                 (result_class,
                  round(precision["EASY"], 2),
                  round(precision["MODERATE"], 2),
                  round(precision["HARD"], 2))
                 )

            try:
                file = open(filename, "w")
                file.write(str(round(precision["EASY"], 2)) + "\n")
                file.write(str(round(precision["MODERATE"], 2)) + "\n")
                file.write(str(round(precision["HARD"], 2)) + "\n")
            except Exception as e:
                error(self.node, "%s" % str(e))
                return False
            finally:
                file.close()

        return True

    def _compute_precision(self, precision_file_line):
        """
        Compute the precision score given the 41 values in the kitti sdk output file.

        @param self: The object pointer
        @param precision_file_line: A string line with 41 values space separated
        @type  precision_file_line: str
        @return: int -1 on Failure, >=0 on successfully computed metric
        """
        # Precision results are 41 values separated by spaces
        values = precision_file_line.rstrip(" \n").split(" ")

        if len(values) < 41:
            error(self.node, "Expected 41 values for precision.")
            return -1

        # Cast values to float
        values = [float(i) for i in values]

        # Kitti sdk declares in the README.txt file that:
        # from 8.10.2019 we compute the average precision not like in the
        # PASCAL VOC protocol, but as follows:
        # ======================
        # sum = 0;
        # for (i=1; i<=40; i++)
        #   sum += vals[i];
        # average = sum/40.0;
        # ======================
        #
        # So we do the same in Python
        precision = sum(values[1::]) / 40.0

        return precision
