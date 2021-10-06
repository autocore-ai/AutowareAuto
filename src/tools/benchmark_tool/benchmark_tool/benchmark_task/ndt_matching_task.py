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

from benchmark_tool.benchmark_task.benchmark_task import BenchmarkTask
from benchmark_tool.output_formatter.generic_stream_formatter \
    import GenericStreamFormatter
from benchmark_tool.metric.average_metric import AverageMetric
from benchmark_tool.player.synced_external_player \
    import SyncedExternalPlayer
from benchmark_tool.time_estimator.time_estimator_header \
    import TimeEstimatorHeader
from benchmark_tool.utility import getParameter, info, error

from std_msgs.msg import Int64


class NdtMatchingTask(BenchmarkTask):
    """
    The NdtMatchingTask class benchmarks the lidar localization.

    It is a specialized BenchmarkTask class holding all the peculiarities of the ndt_matching node.
    """

    # Main loop is at 1000 Hz, so counting to 1000 will be a wait of
    # 1 seconds before shutdown
    SHUTDOWN_TICK_WAIT = 1000

    POINT_CLOUD_TRACK_MIN_SUBS = 1
    POINT_CLOUD_TRACK_RATE_HZ = 10

    SPEED_METRIC_TOPIC = "/ndt_matching_speed_metric"

    SPEED_METRIC_RESULT_PATH = "/ndt_matching/speed"
    SPEED_METRIC_OUTPUT_PATH = "/ndt_matching"
    SPEED_MEASURE_FILE_NAME = "speed.txt"
    SPEED_METRIC_OUTPUT_NAME = "speed_metric.txt"

    def __init__(self, node):
        """
        Create a NdtMatchingTask object.

        @param node: ROS2 node
        @type  node: rclpy.node.Node
        """
        super(NdtMatchingTask, self).__init__(node)
        self._data_player = None
        self._speed_metric = None
        self._shutdown_counter = 0
        self._speed_formatter = None

    def init(self):
        """
        Initialize the task structure.

        The task computes one metric:
            - speed

        @return: True on success, False on failure
        """
        input_topic = getParameter(self.node, "input_topic")
        benchmarked_out_topic = getParameter(self.node, "benchmarked_output_topic")
        result_path = getParameter(self.node, "result_path")
        end_frame = getParameter(self.node, "force_end_at_frame_n")

        if end_frame <= 0:
            error(self.node, "The force_end_at_frame_n parameter has to be positive" +
                  " and greater from zero.")
            return False

        # Load TimeEstimator to get the speed metric
        self._time_estimator = TimeEstimatorHeader(self.node)

        # Load the speed output formatter to get the output of the TimeEstimator
        # and save it to a file
        self._speed_formatter = GenericStreamFormatter(
            self.node,
            result_path + self.SPEED_METRIC_RESULT_PATH,
            self.SPEED_MEASURE_FILE_NAME
        )

        # Prepare the speed metric object
        self._speed_metric = AverageMetric(
            self.node,
            (result_path + self.SPEED_METRIC_RESULT_PATH),
            (result_path + self.SPEED_METRIC_OUTPUT_PATH),
            self.SPEED_MEASURE_FILE_NAME,
            self.SPEED_METRIC_OUTPUT_NAME
        )

        # Create the Player to play the data to the ndt_matching node
        self._data_player = SyncedExternalPlayer(self.node)
        if not self._data_player.add_track(
                input_topic,
                "/replay" + input_topic,
                benchmarked_out_topic,
                self.POINT_CLOUD_TRACK_MIN_SUBS,
                self.POINT_CLOUD_TRACK_RATE_HZ,
                end_frame,
                120000):
            error(self.node, "There is a problem initializing the player")
            return False

        if not self._time_estimator.start_listener(benchmarked_out_topic,
                                                   self.SPEED_METRIC_TOPIC):
            error(self.node,
                  "There is a problem initializing the TimeEstimator")
            return False

        # Listen to the speed metric
        if not self._speed_formatter.start_output_listener(
                self.SPEED_METRIC_TOPIC,
                Int64
        ):
            error(self.node, "Problem initializing the speed metric listener")
            return False

        # Everything is fine
        return True

    def run(self):
        """
        Run the task.

        This function is called repeatedly in a loop until the task has finished its operations.
        It uses a Player class instance to play the pointcloud data to the blackbox system.
        When there is no more data to play, the task waits SHUTDOWN_TICK_WAIT before notifying the
        end of any operation using the return value.

        @return True: if the task has not finished operation, False when the task has nothing more
            to do
        """
        # Check if the player has data to play
        if not self._data_player.play_data():
            if self._shutdown_counter > self.SHUTDOWN_TICK_WAIT:
                # Return running state of the task: stopping
                return False
            elif self._shutdown_counter == 0:
                print("")  # needed to keep visualizing the progress bar
                info(self.node, "No more data to play, waiting last frames...")

            self._shutdown_counter += 1

        # Return running state of the task: running
        return True

    def compute_results(self):
        """
        Compute the final benchmark results.

        @return: None
        """
        info(self.node, "Start metric computation...")
        # Compute the speed metric
        if not self._speed_metric.compute_metric(
            "Time of iteration (ms) :\nMin: %.3f" +
                "\nAverage: %.3f \nMax: %.3f"):
            error(self.node, "Problem computing speed metrics")
