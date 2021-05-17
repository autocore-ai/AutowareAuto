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
import shutil
from abc import ABCMeta, abstractmethod


class OutputFormatter(metaclass=ABCMeta):
    """
    The OutputFormatter class is a generic interface.

    The object should listen to an output topic of a blackbox system, interpret the received data
    and structure it into a specific format.
    The class holds a filesystem path for the output path and a member for the subscriber object,
    it also provides utility methods to handle folders.
    """

    def __init__(self, node, result_path):
        """
        Create a OutputFormatter object.

        @param self: The object pointer
        @param node: ROS2 node
        @type  node: rclpy.node.Node
        @param result_path: The path on the filesystem for the result folder
        @type  result_path: str
        """
        super(OutputFormatter, self).__init__()
        self._result_path = result_path
        self._subscriber = 0
        self.node = node

    @abstractmethod
    def start_output_listener(self, topic):
        """
        Start the subscriber on the specified topic and initialize internal structures.

        @param self: The object pointer
        @param topic: The topic to listen for the data
        @type  topic: str
        @return: True on success, False on failure
        """
        pass

    @staticmethod
    def clean_folder(folder):
        """
        Remove any file or folder into the specified path.

        @param folder: The path on filesystem of the folder to clean
        @type  folder: str
        @return: True on success, False on failure
        """
        for filename in os.listdir(folder):
            file_path = os.path.join(folder, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception:
                return False

        return True

    @staticmethod
    def create_folder(folder):
        """
        Create the specified folder and subfolder if the path does not exist.

        @param folder: The path on filesystem to be created
        @type  folder: str
        @return: True on success, False on failure
        """
        if not os.path.isdir(folder):
            try:
                os.makedirs(folder)
            except Exception:
                return False

        return True
