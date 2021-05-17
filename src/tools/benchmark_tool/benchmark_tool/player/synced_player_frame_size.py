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
from std_msgs.msg import Int64
from benchmark_tool.player.synced_player import SyncedPlayer


class SyncedPlayerFrameSize(SyncedPlayer):

    def __init__(self, node, dataset, sizeTopic, tracksToBroadcastSize):
        """
        Create a SyncedPlayerFrameSize object.

        @param self: The object pointer
        @param node: ROS2 node
        @type  node: rclpy.node.Node
        @param dataset: The dataset object where to retrieve the data
        @type  dataset: Dataset class
        @param sizeTopic: The name of the topic where publish the size
        @type  sizeTopic: str
        @param tracksToBroadcastSize: The array containing the indexes of each track whose size
            must be broadcasted
        @type  tracksToBroadcastSize: int array or int
        """
        super(SyncedPlayerFrameSize, self).__init__(node, dataset)

        if(type(tracksToBroadcastSize) is int):
            singleIndex = tracksToBroadcastSize
            tracksToBroadcastSize = []
            tracksToBroadcastSize.append(singleIndex)

        self._tracksToBroadcastSize = tracksToBroadcastSize
        self._sizePublisher = node.create_publisher(
            Int64,
            sizeTopic,
            qos_profile=QoSProfile(depth=1)
        )

    def _publish_track(self, track, frame):
        """
        Publish the given frame on the topic of the given track.

        @param self: The object pointer
        @param track: The track related to the transmission
        @type  track: SyncedTrack
        @param frame: The data to be transmitted
        @type  frame: Can vary depending on the transmission
        @return: None
        """
        super(SyncedPlayerFrameSize, self)._publish_track(track, frame)

        if(track.track_index in self._tracksToBroadcastSize):
            size = self._compute_size(frame)
            publish_msg = Int64()
            publish_msg.data = int(size)
            self._sizePublisher.publish(publish_msg)

    def _compute_size(self, frame):
        """
        Compute the size of a given frame.

        In this case, the frame is assumed to be a pointcloud.

        @param self: The object pointer
        @param frame: The data to be sized
        @type  frame: Depends on the data source
        """
        return len(frame.data)
