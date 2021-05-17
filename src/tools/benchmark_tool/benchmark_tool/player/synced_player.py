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
from benchmark_tool.player.generic_player import GenericPlayer
from benchmark_tool.utility import error, warning


class SyncedPlayer(GenericPlayer):
    """
    The SyncedPlayer class is used to play the data to some specified topics.

    The class holds a Dataset object used to retrieve the data.
    It has lists of tracks to be played, each track could be a point cloud track or an image track
    and so on.
    """

    class SyncedTrack(GenericPlayer.Track):
        """
        The SyncedTrack object contains all the information of a GenericPlayer.Track object.

        It extends its behaviour using a synchronization mechanism that allows the player not to
        overwhelm the receiving node with data.
        The SynchedPlayer uses a synchronization topic to know when to send the data.
        """

        def __init__(self, node, track_index, publish_topic, publish_topic_type,
                     sync_topic, min_subscribers, rate_hz, timeout_ms=10000):
            """
            Create a SyncedTrack object.

            @param self: The object pointer
            @param node: ROS2 node
            @type  node: rclpy.node.Node
            @param track_index: The index of the data source in the Dataset
            @type  track_index: int
            @param publish_topic: The name of the topic where publish the data
            @type  publish_topic: str
            @param publish_topic_type: The data type of the published data
            @type  publish_topic_type: Depends on the published data
            @param sync_topic: The name of the topic on which sync the transmission of the data
            @type  sync_topic: str
            @param min_subscribers: Minumum number of subscriber before publishing the data
            @type  min_subscribers: int
            @param rate_hz: Frequency for the transmission
            @type  rate_hz: int
            @param timeout_ms: Timeout to retrigger a retransmission
            @type  timeout_ms: int
            """
            super(SyncedPlayer.SyncedTrack, self).__init__(node, track_index,
                                                           publish_topic, publish_topic_type,
                                                           min_subscribers, rate_hz)
            self.clear_to_send = True
            self.callback_lock = threading.RLock()
            self.timeout_ns = timeout_ms * 10**6
            self.last_transmission = 0
            self.enable_guard = False
            # Get type of the sync topic and subscribe to it
            sync_topic_type = get_msg_class(node, sync_topic,
                                            blocking=True)
            self.sync_sub = node.create_subscription(
                sync_topic_type,
                sync_topic,
                self._sync_topic_callback,
                qos_profile=QoSProfile(depth=1)
            )

        def _sync_topic_callback(self, msg):
            """
            Mark the track as clear to send.

            Callback for the synchronization topic.

            @param self: The object pointer
            @param msg: The message from the synchronization topic
            @type  msg: Depends on the topic's data
            @return: None
            """
            with self.callback_lock:
                self.clear_to_send = True

    def __init__(self, node, dataset):
        """
        Create a SyncedPlayer object.

        @param self: The object pointer
        @param node: ROS2 node
        @type  node: rclpy.node.Node
        @param dataset: The dataset object where to retrieve the data
        @type  dataset: Dataset class
        """
        super(SyncedPlayer, self).__init__(node, dataset)

    def add_track(self, track_idx, publish_topic, publish_topic_type,
                  sync_topic, min_subscribers, rate_hz, timeout_ms=10000):
        """
        Add a track to the player object.

        @param self: The object pointer
        @param track_idx: The index of the data source in the Dataset
        @type  track_idx: int
        @param publish_topic: The name of the topic where publish the data
        @type  publish_topic: str
        @param publish_topic_type: The data type of the published data
        @type  publish_topic_type: Depends on the published data
        @param sync_topic: The name of the topic on which sync the transmission of the data
        @type  sync_topic: str
        @param min_subscribers: Minumum number of subscriber before publishing the data
        @type  min_subscribers: int
        @param rate_hz: Frequency for the transmission
        @type  rate_hz: int
        @param timeout_ms: Timeout to retrigger a retransmission
        @type  timeout_ms: int
        @return: True on success, False on failure
        """
        track = None
        try:
            # Create the track
            track = self.SyncedTrack(self.node, track_idx, publish_topic,
                                     publish_topic_type, sync_topic, min_subscribers, rate_hz,
                                     timeout_ms)

        except Exception as e:
            error(self.node, "%s" % str(e))
            return False

        if not self._append_track(track, track_idx):
            error(self.node,
                  "Error initializing track index: %d" % track_idx)
            error(self.node,
                  "player type : {}".format(track.track_type_str))
            return False

        return True

    def _play_data(self):
        """
        Play all the tracks available.

        When all data is played from the track, it is removed from the list.

        @param self: The object pointer
        @return: None
        """
        for track in list(self._tracks):
            with track.callback_lock:
                data = self._dataset.get_sources(track.track_type_str)[track.track_index]
                time_now = self.node.get_clock().now().nanoseconds
                if track.clear_to_send is True:
                    if not self._play_track(track, data):
                        self._tracks.remove(track)
                elif (track.enable_guard is True) and \
                        (time_now - track.last_transmission) > track.timeout_ns:
                    # The data is not arrived yet, maybe the packet was lost,
                    # re-send the same packet
                    warning(self.node,
                            ("Response is not arrived on time. Timeout: %d ms. "
                             + "Triggering re-send.") % (track.timeout_ns // 10**6))
                    data.back()
                    if self._frame_played > 0:
                        self._frame_played -= 1
                    track.enable_guard = False
                    track.clear_to_send = True

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
        with track.callback_lock:
            track.publisher.publish(frame)
            track.last_transmission = self.node.get_clock().now().nanoseconds
            track.enable_guard = True
            track.clear_to_send = False
