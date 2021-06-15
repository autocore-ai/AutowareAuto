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
from benchmark_tool.player.external_player import ExternalPlayer
from benchmark_tool.utility import error, warning
from ros2topic.api import get_msg_class


class SyncedExternalPlayer(ExternalPlayer):
    """
    The SyncedExternalPlayer class is bridging class.

    It is used to emulate playing the data despite the actual player being an external node.
    """

    class SyncedReplayTrack(ExternalPlayer.ReplayTrack):
        """
        The SyncedReplayTrack object.

        It contains information about the topic where to listen for data, the topic where to
        publish the data, the minimum number of subscribers before publishing the data and the rate
        in Hz for the frequency of the transmission.
        It listens for data on a topic and saves frames on an internal structure, then it publishes
        the data at its own rate synching with an external topic.
        A mechanism of timeout is provided, when it's triggered it resends the last frame.
        """

        def __init__(self, node, listen_topic, publish_topic, sync_topic,
                     min_subscribers, rate_hz, frame_number, timeout_ms=10000):
            """
            Create a SyncedReplayTrack object.

            @param node: ROS2 node
            @type  node: rclpy.node.Node
            @param listen_topic: The topic where to listen for data to be replayed
            @type  listen_topic: str
            @param publish_topic: The name of the topic where publish the data
            @type  publish_topic: str
            @param sync_topic: The name of the topic on which sync the transmission of the data
            @type  sync_topic: str
            @param min_subscribers: Minumum number of subscriber before publishing the data
            @type  min_subscribers: int
            @param rate_hz: Frequency for the transmission
            @type  rate_hz: int
            @param frame_number: Frame number expected
            @type  frame_number: int
            @param timeout_ms: Timeout to retrigger a retransmission
            @type  timeout_ms: int
            """
            super(SyncedExternalPlayer.SyncedReplayTrack, self).__init__(node,
                                                                         listen_topic,
                                                                         publish_topic,
                                                                         min_subscribers,
                                                                         rate_hz,
                                                                         frame_number
                                                                         )
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
            Move forward to the next frame.

            Callback for the synchronization topic.

            @param msg: The message from the synchronization topic
            @type  msg: Depends on the topic's data
            @return: None
            """
            with self.callback_lock:
                self.clear_to_send = True
                if len(self.replay_data) > 0:
                    self.replay_data.pop(0)
                    self._trk_frame_played += 1

        def next(self):
            """
            In this implementation this function shall do nothing.

            The frame is moved forward in the _sync_topic_callback upon receiving data from sync
            topic.

            @return: None
            """
            # In this object this function shall do nothing
            pass

    def __init__(self, node):
        """
        Create a SyncedExternalPlayer object.

        @param node: ROS2 node
        @type  node: rclpy.node.Node
        """
        super(SyncedExternalPlayer, self).__init__(node)

    def add_track(self, listen_topic, publish_topic, sync_topic,
                  min_subscribers, rate_hz, frame_number, timeout_ms=10000):
        """
        Add a track to the player object.

        @param listen_topic: The topic where to listen for data to be replayed
        @type  listen_topic: str
        @param publish_topic: The name of the topic where publish the data
        @type  publish_topic: str
        @param sync_topic: The name of the topic on which sync the transmission of the data
        @type  sync_topic: str
        @param min_subscribers: Minumum number of subscriber before publishing the data
        @type  min_subscribers: int
        @param rate_hz: Frequency for the transmission
        @type  rate_hz: int
        @param frame_number: Frame number expected
        @type  frame_number: int
        @param timeout_ms: Timeout to retrigger a retransmission
        @type  timeout_ms: int
        @return: True on success, False on failure
        """
        track = None
        try:
            # Create the track
            track = self.SyncedReplayTrack(
                self.node, listen_topic, publish_topic, sync_topic,
                min_subscribers, rate_hz, frame_number, timeout_ms
            )

        except Exception as e:
            error(self.node, "[add_track] %s" % str(e))
            return False

        if not self._append_track(track):
            error(self.node, "Error adding track")
            return False

        return True

    def _play_data(self):
        """
        Play all the point cloud tracks available.

        When all data is played from the track, it is removed from the list.

        @return: None
        """
        for track in list(self._tracks):
            if not self._play_track(track):
                self._tracks.remove(track)

        for track in list(self._tracks):
            with track.callback_lock:
                time_now = self.node.get_clock().now().nanoseconds
                if track.clear_to_send is True:
                    if not self._play_track(track):
                        self._tracks.remove(track)
                elif (track.enable_guard is True) and \
                        (time_now - track.last_transmission) > track.timeout_ns:
                    # The data is not arrived yet, maybe the packet was lost,
                    # re-send the same packet
                    warning(self.node,
                            ("Response is not arrived on time. Timeout: %d ms. "
                             + "Triggering re-send.") % (track.timeout_ns // 10**6))

                    if self._frame_played > 0:
                        self._frame_played -= 1
                    track.enable_guard = False
                    track.clear_to_send = True

    def _publish_track(self, track, frame):
        """
        Publish the given frame on the topic of the given track.

        @param track: The track related to the transmission
        @type  track: Track
        @param frame: The data to be transmitted
        @type  frame: Can vary depending on the transmission
        @return: None
        """
        with track.callback_lock:
            # Update the header stamp
            frame.header.stamp = self.node.get_clock().now().to_msg()
            track.publisher.publish(frame)
            track.last_transmission = self.node.get_clock().now().nanoseconds
            track.enable_guard = True
            track.clear_to_send = False
