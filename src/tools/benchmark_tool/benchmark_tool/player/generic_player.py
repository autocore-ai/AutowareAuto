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

import sys
from rclpy.qos import QoSProfile
from benchmark_tool.dataset.dataset import Dataset
from benchmark_tool.utility import error, info


class GenericPlayer(object):
    """
    The GenericPlayer class is used to play the data to some specified topics.

    The class holds a Dataset object used to retrieve the data.
    It has lists of track to be played, each track could be a point cloud track or an image track
    and so on.
    """

    class Track(object):
        """
        The Track object.

        It contains information about the index of the data into the Dataset, the topic where to
        publish the data, the minimum number of subscribers before publishing the data and the rate
        in Hz for the frequency of the transmission.
        """

        def __init__(self, node, track_index, publish_topic, publish_topic_type,
                     min_subscribers, rate_hz):
            """
            Create a Track object.

            @param node: ROS2 node
            @type  node: rclpy.node.Node
            @param track_index: The index of the data source in the Dataset
            @type  track_index: int
            @param publish_topic: The name of the topic where publish the data
            @type  publish_topic: str
            @param publish_topic_type: The data type of the published data
            @type  publish_topic_type: Depends on the published data
            @param min_subscribers: Minumum number of subscriber before publishing the data
            @type  min_subscribers: int
            @param rate_hz: Frequency for the transmission
            @type  rate_hz: int
            """
            self.track_index = track_index
            self.period_ns = ((1.0/rate_hz)*1000*1000*1000)
            self.last_run_time_ns = 0
            self.publisher = node.create_publisher(
                publish_topic_type,
                publish_topic,
                qos_profile=QoSProfile(depth=1)
            )
            self.min_subscribers = min_subscribers
            self.track_type_str = publish_topic_type.__name__
            self.publish_topic = publish_topic

    def __init__(self, node, dataset):
        """
        Create a GenericPlayer object.

        @param node: ROS2 node
        @type  node: rclpy.node.Node
        @param dataset: The dataset object where to retrieve the data
        @type  dataset: Dataset class
        """
        assert isinstance(dataset, Dataset)
        self.node = node
        self._dataset = dataset
        self._tracks = []
        self._frame_to_play = 0
        self._frame_played = 0

    def add_track(self, track_idx, publish_topic, publish_topic_type,
                  min_subscribers, rate_hz):
        """
        Add a track to the player object.

        @param track_idx: The index of the data source in the Dataset
        @type  track_idx: int
        @param publish_topic: The name of the topic where publish the data
        @type  publish_topic: str
        @param publish_topic_type: The data type of the published data
        @type  publish_topic_type: Depends on the published data
        @param min_subscribers: Minumum number of subscriber before publishing the data
        @type  min_subscribers: int
        @param rate_hz: Frequency for the transmission
        @type  rate_hz: int
        @return: True on success, False on failure
        """
        track = None
        try:
            # Create the track
            track = self.Track(self.node, track_idx, publish_topic,
                               publish_topic_type, min_subscribers, rate_hz)

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

    def _append_track(self, track, track_idx):
        """
        Append a track to the internal list and update the internal data counter.

        @param track: The track to append to the internal list
        @type  track: Track
        @param track_idx: The index of the data source in the Dataset
        @type  track_idx: int
        @return: True on success, False on failure
        """
        if track_idx < len(self._dataset.get_sources(track.track_type_str)):
            # Get the dataset count for this track, used with a
            # progress bar when data is played
            self._frame_to_play += self._dataset.get_source_length(
                track.track_type_str, track_idx)

            # Append to the list of track to be played
            self._tracks.append(track)
            return True

        return False

    def print_progress_bar(self):
        """
        Print on the standard output a progress bar based on the played data.

        @return: None
        """
        bar_len = 50
        filled_len = int(
            round(
                (bar_len * self._frame_played) / float(self._frame_to_play)
            )
        )

        percents = round(
            (100.0 * self._frame_played) / float(self._frame_to_play), 1
        )
        bar = '=' * filled_len + '-' * (bar_len - filled_len)

        sys.stdout.write('Progress: [%s] %s%s\r' % (bar, percents, '%'))
        sys.stdout.flush()

    def play_data(self):
        """
        Play all the track.

        @return: True when there are still tracks to be played, False when there is no more data to
        play
        """
        # Play the point cloud traks if any
        if len(self._tracks) > 0:
            self._play_data()
        else:
            return False

        # Display an animation to ensure the user knows that the node is
        # working well
        self.print_progress_bar()

        return True

    def _play_data(self):
        """
        Play all the point cloud tracks available.

        When all data is played from the track, it is removed from the list.

        @return: None
        """
        for track in list(self._tracks):
            data = self._dataset.get_sources(track.track_type_str)[track.track_index]
            if not self._play_track(track, data):
                self._tracks.remove(track)

    def _play_track(self, track, data):
        """
        Play the given track.

        @param track: The track to be played
        @type  track: Track
        @param data: The data source of the dataset relative to this track
        @param data: DatasetElem
        @return: False when the track is ended, True otherwise
        """
        time_now = self.node.get_clock().now().nanoseconds

        # Check if this is the period to publish the data
        if (time_now - track.last_run_time_ns) >= track.period_ns:

            # Save in the internal structure that it is executing now
            track.last_run_time_ns = time_now

            # Check if there is still data, otherwise delete the track
            if not data.is_end():

                # Wait for subscriber to publish the data
                if self.node.count_subscribers(track.publish_topic) \
                        >= track.min_subscribers:

                    # Retrieve the frame from dataset
                    publish_frame = data.get_frame()

                    if publish_frame is not None:
                        # Publish the frame
                        self._publish_track(track, publish_frame)

                        self._frame_played += 1
                    else:
                        error(self.node,
                              "Failed to read %s file." % data.get_cursor_filename())

                    # Move cursor to the next frame
                    data.next()
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

        @param track: The track related to the transmission
        @type  track: Track
        @param frame: The data to be transmitted
        @type  frame: Can vary depending on the transmission
        @return: None
        """
        track.publisher.publish(frame)
