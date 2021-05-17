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
from rclpy.qos import QoSProfile
from benchmark_tool.output_formatter.output_formatter import OutputFormatter
from benchmark_tool.utility import error


class GenericStreamFormatter(OutputFormatter):
    """
    The GenericStreamFormatter class is a specialized OutputFormatter class.

    It listens to topics where a stream is published.
    It saves the received values on a file with one value for each line.
    """

    def __init__(self, node, result_path, result_file_name="stream.txt"):
        """
        Create a GenericStreamFormatter object.

        @param self: The object pointer
        @param node: ROS2 node
        @type  node: rclpy.node.Node
        @param result_path: The path on filesystem where to save the received stream
        @type  result_path: str
        @param result_file_name: The name of the file where to save the output
        @type  result_file_name: str
        """
        super(GenericStreamFormatter, self).__init__(node, result_path)
        self._result_file_name = result_file_name

    def start_output_listener(self, topic, topic_datatype):
        """
        Start the subscriber on the specified topic.

        @param self: The object pointer
        @param topic: The topic to listen for the data
        @type  topic: str
        @param topic_datatype: The data type of the topic
        @type  topic_datatype: Depends on the topic
        @return: True on success, False on failure
        """
        # Creates the results folder if not exists
        if not OutputFormatter.create_folder(self._result_path):
            return False

        try:
            # Subscribe to topic
            self.node.create_subscription(
                topic_datatype,
                topic,
                self._handle_callback,
                qos_profile=QoSProfile(depth=1)
            )
        except Exception as e:
            error(self.node, "%s" % str(e))
            return False

        # Remove everything inside that folder
        if not OutputFormatter.clean_folder(self._result_path):
            return False

        return True

    def _handle_callback(self, msg):
        """
        Save the received metric into a file, one value for each line.

        Callback for the metric topic.

        @param self: The object pointer
        @param msg: Message containing the int value
        @type  msg: Depends on the topic
        @return: None
        """
        # create or open path and filename for the output file
        # the format is: <root_path>/<result_folder>/<result_file_name>.txt
        file_name = os.path.join(self._result_path,
                                 self._result_file_name)
        try:
            # Create the file or append
            file = open(file_name, "a")

            # Write the message content
            file.write(str(msg.data) + "\n")

            # Close the file
            file.close()
        except Exception as e:
            error(self.node, "%s" % str(e))
