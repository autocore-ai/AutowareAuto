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
from benchmark_tool.player.generic_player import GenericPlayer
from benchmark_tool.utility import error, info
from ros2topic.api import get_msg_class


class ExternalPlayer(GenericPlayer):
    """
    The ExternalPlayer class is a bridging class.

    Is is used to emulate playing the data despite the actual player being an external node.
    """

    class ReplayTrack(object):
        """
        The ReplayTrack object.

        It contains information about the topic on whoich to listen for data, the topic on which to
        publish the data, the minimum number of subscribers before publishing the data and the rate
        in Hz for the frequency of the transmission.
        It listens for data on a topic and saves frames on an internal structure, then it publishes
        the data at its own rate.
        """

        def __init__(self, node, listen_topic, publish_topic, min_subscribers,
                     rate_hz, frame_number):
            """
            Create a ReplayTrack object.

            @param self: The object pointer
            @param node: ROS2 node
            @type  node: rclpy.node.Node
            @param listen_topic: The topic where to listen for data to be replayed
            @type  listen_topic: str
            @param publish_topic: The name of the topic where publish the data
            @type  publish_topic: str
            @param min_subscribers: Minumum number of subscriber before publishing the data
            @type  min_subscribers: int
            @param rate_hz: Frequency for the transmission
            @type  rate_hz: int
            @param frame_number: Frame number expected
            @type  frame_number: int
            """
            self.period_ns = ((1.0/rate_hz)*1000*1000*1000)
            self.last_run_time_ns = 0
            self.replay_data = []
            topic_type = get_msg_class(
                node,
                listen_topic,
                blocking=True
            )
            self.subscriber = node.create_subscription(
                topic_type,
                listen_topic,
                self.listen_topic_callback,
                qos_profile=QoSProfile(depth=10)
            )
            self.publisher = node.create_publisher(
                topic_type,
                publish_topic,
                qos_profile=QoSProfile(depth=1)
            )
            self.min_subscribers = min_subscribers
            self._total_frame_number = frame_number
            self._trk_frame_played = 0
            self.publish_topic = publish_topic

        def listen_topic_callback(self, msg):
            """
            Append a message to replay_data.

            Callback for the listened topic from the external node providing data.

            @param self: The object pointer
            @param msg: The message from the synchronization topic
            @type  msg: Depends on the topic's data
            @return: None
            """
            self.replay_data.append(msg)

        def is_end(self):
            """
            Check if the track has played the expected amount of frames.

            @param self: The object pointer
            @return: True if the data is finished, False otherwise
            """
            return (self._trk_frame_played >= self._total_frame_number)

        def get_frame(self):
            """
            Retrieve the first frame from the internal data structure.

            @param self: The object pointer
            @return: Depends on the data
            """
            frame = None
            if len(self.replay_data) > 0:
                frame = self.replay_data[0]

            return frame

        def next(self):
            """
            Remove the first entry of the internal data structure.

            @param self: The object pointer
            @return: None
            """
            if len(self.replay_data) > 0:
                self.replay_data.pop(0)
                self._trk_frame_played += 1

    def __init__(self, node):
        """
        Create a ExternalPlayer object.

        @param self: The object pointer
        @param node: ROS2 node
        @type  node: rclpy.node.Node
        """
        self.node = node
        self._tracks = []
        self._frame_played = 0
        self._frame_to_play = 0

    def add_track(self, listen_topic, publish_topic, min_subscribers,
                  rate_hz, frame_number):
        """
        Add a track to the player object.

        @param self: The object pointer
        @param listen_topic: The topic where to listen for data to be replayed
        @type  listen_topic: str
        @param publish_topic: The name of the topic where publish the data
        @type  publish_topic: str
        @param min_subscribers: Minumum number of subscriber before publishing the data
        @type  min_subscribers: int
        @param rate_hz: Frequency for the transmission
        @type  rate_hz: int
        @param frame_number: Frame number expected
        @type  frame_number: int
        @return: True on success, False on failure
        """
        track = None
        try:
            # Create the track
            track = self.ReplayTrack(self.node, listen_topic, publish_topic,
                                     min_subscribers, rate_hz, frame_number)

        except Exception as e:
            error(self.node, "%s" % str(e))
            return False

        if not self._append_track(track):
            error(self.node, "Error adding track")
            return False

        return True

    def _append_track(self, track):
        """
        Append a track to the internal list and update the internal data counter.

        @param self: The object pointer
        @param track: The track to append to the internal list
        @type  track: Track
        @return: True on success, False on failure
        """
        # Get the frame count for this track, used with a
        # progress bar when data is played
        self._frame_to_play += track._total_frame_number

        # Append to the list of track to be played
        self._tracks.append(track)
        return True

    def _play_data(self):
        """
        Play all the point cloud tracks available.

        When all data is played from the track, it is removed from the list.

        @param self: The object pointer
        @return: None
        """
        for track in list(self._tracks):
            if not self._play_track(track):
                self._tracks.remove(track)

    def _play_track(self, track):
        """
        Play the given track.

        @param self: The object pointer
        @param track: The track to be played
        @type  track: Track
        @return: False when the track is ended, True otherwise
        """
        time_now = self.node.get_clock().now().nanoseconds

        # Check if this is the period to publish the data
        if (time_now - track.last_run_time_ns) >= track.period_ns:

            # Save in the internal structure that it is executing now
            track.last_run_time_ns = time_now

            # Check if there is still data, otherwise delete the track
            if not track.is_end():

                # Wait for subscriber to publish the data
                if self.node.count_subscribers(track.publish_topic) \
                        >= track.min_subscribers:

                    # Retrieve the frame from dataset
                    publish_frame = track.get_frame()

                    if publish_frame is not None:
                        # Publish the frame
                        self._publish_track(track, publish_frame)

                        self._frame_played += 1

                        # Move cursor to the next frame
                        track.next()

                else:
                    info(self.node,
                         "Waiting subscriber on topic: %s" % track.publisher.name)
            else:
                # Remove this track
                return False

        return True

    def _publish_track(self, track, frame):
        """
        Publish the given frame on the topic of the given track.

        @param self: The object pointer
        @param track: The track related to the transmission
        @type  track: Track
        @param frame: The data to be transmitted
        @type  frame: Can vary depending on the transmission
        @return: None
        """
        # Update the header stamp
        frame.header.stamp = self.node.get_clock().now().to_msg()
        track.publisher.publish(frame)
