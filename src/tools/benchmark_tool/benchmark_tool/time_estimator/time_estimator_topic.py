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

import threading
from rclpy.qos import QoSProfile
from ros2topic.api import get_msg_class
from std_msgs.msg import Int64
from benchmark_tool.utility import error, warning


class TimeEstimatorTopic(object):
    """
    The TimeEstimatorTopic class.

    It measures the time elapsed between the reception of a message from an input topic to an
    output topic and publish the measure in millisecond resolution to another topic.
    """

    def __init__(self, node):
        """
        Create a TimeEstimatorTopic object.

        @param self: The object pointer
        @param node: ROS2 node
        @type  node: rclpy.node.Node
        """
        self._sub_input_topic = None
        self._sub_output_topic = None
        self._publisher = None
        self._time_received_input = 0
        self.node = node
        self.callback_lock = threading.RLock()

    def start_listener(self, input_topic, output_topic, publish_topic):
        """
        Start the time measurement.

        @param self: The object pointer
        @param input_topic: Topic to be listened to start the time measurement
        @type  input_topic: str
        @param output_topic: Topic to be listened to stop the time measurement
        @type  output_topic: str
        @param publish_topic: Topic where the measurement is published
        @type  publish_topic: str
        @return: True on success, False on failure
        """
        try:
            # Create publisher to publish the speed measure
            self._publisher = self.node.create_publisher(
                Int64,
                publish_topic,
                qos_profile=QoSProfile(depth=1)
            )

            # Get type of the input topic and subscribe to it
            input_topic_type = get_msg_class(self.node, input_topic,
                                             blocking=True)
            self._sub_input_topic = self.node.create_subscription(
                input_topic_type,
                input_topic,
                self.input_topic_callback,
                qos_profile=QoSProfile(depth=1)
            )

            # Get type of the output topic and subscribe to it
            output_topic_type = get_msg_class(self.node, output_topic,
                                              blocking=True)
            self._sub_output_topic = self.node.create_subscription(
                output_topic_type,
                output_topic,
                self.output_topic_callback,
                qos_profile=QoSProfile(depth=1)
            )
        except Exception as e:
            error(self.node, "%s" % str(e))
            return False

        return True

    def input_topic_callback(self, msg):
        """
        Update received time.

        Callback function triggered by the reception of a message from the input topic.

        @param self: The object pointer
        @param msg: The topic message
        @type  msg: The type can vary depending on the listened topic
        @return: None
        """
        with self.callback_lock:
            if self._time_received_input != 0:
                warn = "[TimeEstimatorTopic] Input time overwritten by another"\
                    + " input message, consider slowing down the rate of " \
                    + "the published data to let the receiving node handle it"
                warning(self.node, warn)
            self._time_received_input = self.node.get_clock().now().nanoseconds

    def output_topic_callback(self, msg):
        """
        Compute the time since the last message on the input topic and publish it.

        Callback function triggered by the reception of a message from the output topic.

        @param self: The object pointer
        @param msg: The topic message
        @type  msg: The type can vary depending on the listened topic
        @return: None
        """
        with self.callback_lock:
            if self._time_received_input != 0:
                # Get actual time from ROS
                time_now = self.node.get_clock().now().nanoseconds

                # Compute the amount of time elapsed from receiving the last
                # message in the input topic
                measure = time_now - self._time_received_input

                # Transform from nanoseconds to milliseconds
                measure = measure / (1000 * 1000)

                publish_msg = Int64()
                publish_msg.data = int(measure)

                # Publish the measurement
                self._publisher.publish(publish_msg)

                self._time_received_input = 0
