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

from rclpy.qos import QoSProfile
from ros2topic.api import get_msg_class


class RelayPlayer(object):
    """
    The RelayPlayer class is used to replay the data from some specified topics.

    The main things that it does is to update the timestamp header of the message.
    An application of this class is when the benchmarked node is not the direct receiver of the
    data from the benchmark tool, meaning so that the data sent by the tool with a certain
    timestamp has to traverse some preprocessing node before reach the benchmarked one, leading to
    unprecise iteration speed computation.
    Using this class instead, it is possible to listen for data from the preprocessing node,
    intercept it and send it to the benchmarked node with a correct timestamp.
    """

    def __init__(self, node, input_topic, output_topic):
        """
        Create a GenericPlayer object.

        @param node: ROS2 node
        @type  node: rclpy.node.Node
        @param input_topic: The topic where the preprocessing node sends the data
        @type  input_topic: str
        @param output_topic: The input topic of the benchmarked node
        @type  output_topic: str
        """
        self.node = node

        # Get type of the input topic and subscribe to it
        topic_type = get_msg_class(self.node, input_topic,
                                   blocking=True)
        self._sub_input_topic = self.node.create_subscription(
            topic_type,
            input_topic,
            self.input_topic_callback,
            qos_profile=QoSProfile(depth=1)
        )

        # Get type of the output topic and subscribe to it
        self._publisher = self.node.create_publisher(
            topic_type,
            output_topic,
            qos_profile=QoSProfile(depth=1)
        )

    def input_topic_callback(self, msg):
        """
        Update timestamp and send message to the benchmarked node.

        Callback function triggered by the reception of a message from the preprocessing node.

        @param msg: The topic message
        @type  msg: The type can vary depending on the listened topic
        @return: None
        """
        msg.header.stamp = self.node.get_clock().now().to_msg()
        self._publisher.publish(msg)
