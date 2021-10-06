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
from benchmark_tool.dataset.kitti_3d_benchmark_dataset \
    import Kitti3DBenchmarkDataset
from benchmark_tool.output_formatter.generic_stream_formatter \
    import GenericStreamFormatter
from benchmark_tool.output_formatter.euclidean_cluster_node_output_formatter \
    import EuclideanClusterNodeOutputFormatter
from benchmark_tool.metric.kitti_3d_object_detection_metric \
    import Kitti3dObjectDetectionMetric
from benchmark_tool.metric.average_metric import AverageMetric
from benchmark_tool.time_estimator.time_estimator_header \
    import TimeEstimatorHeader
from benchmark_tool.player.synced_player import SyncedPlayer
from benchmark_tool.player.relay_player import RelayPlayer
from benchmark_tool.utility import getParameter, info, error
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int64


class EuclideanClusterNodeTask(BenchmarkTask):
    """
    The EuclideanClusterNodeTask class benchmarks the lidar_euclidean_cluster_detect node.

    It is a specialized BenchmarkTask class holding all the peculiarities of that node.
    """

    # Main loop is at 1000 Hz, so counting to 10000 will be a wait of
    # 10 seconds before shutdown
    SHUTDOWN_TICK_WAIT = 10000
    POINT_CLOUD_DATASET_TRACK = 0
    POINT_CLOUD_TRACK_RATE_HZ = 10
    POINT_CLOUD_TRACK_TIMEOUT_MS = 120000

    # POINT_CLOUD_TRACK_MIN_SUBS is 1 because the minimum number of
    # subscriber before publish the data is 1: the node under test
    POINT_CLOUD_TRACK_MIN_SUBS = 1
    SPEED_METRIC_TOPIC = "/euclidean_cluster_node_speed_metric"
    SPEED_METRIC_RESULT_PATH = "/euclidean_cluster_node/speed"
    OBJDET_METRIC_RESULT_PATH = "/euclidean_cluster_node/objdet"
    SPEED_METRIC_OUTPUT_PATH = "/euclidean_cluster_node"
    OBJDET_METRIC_OUTPUT_PATH = "/euclidean_cluster_node"
    SPEED_MEASURE_FILE_NAME = "speed.txt"
    SPEED_METRIC_OUTPUT_FILENAME = "speed_metric.txt"

    EXPECTED_KITTI_FRAME_ACCURACY = 7481

    def __init__(self, node):
        """
        Create a EuclideanClusterNodeTask object.

        @param node: ROS2 node
        @type  node: rclpy.node.Node
        """
        super(EuclideanClusterNodeTask, self).__init__(node)
        self._data_player = None
        self._time_estimator = None
        self._kitti_3d_metric = None
        self._speed_metric = None
        self._shutdown_counter = 0
        self._speed_formatter = None
        self._limit_frame = -1

    def init(self):
        """
        Initialize the task structure.

        Uses the Kitti object detection benchmark dataset.
        The task computes two metrics:
            - speed
            - detection precision (Kitti measure)

        @return: True on success, False on failure
        """
        dataset_path = getParameter(self.node, "dataset_path")
        input_topic = getParameter(self.node, "input_topic")
        output_topic = getParameter(self.node, "output_topic")
        benchmarked_in_topic = getParameter(self.node, "benchmarked_input_topic")
        benchmarked_out_topic = getParameter(self.node, "benchmarked_output_topic")
        result_path = getParameter(self.node, "result_path")
        end_frame = getParameter(self.node, "force_end_at_frame_n")

        if end_frame is not None and end_frame >= 0:
            self._limit_frame = end_frame
        else:
            error(self.node,
                  "A positive limit frame number must be provided")
            return False

        # Load the Kitti 3D object detection benchmark dataset
        self._dataset = Kitti3DBenchmarkDataset(self.node, dataset_path)
        if not self._dataset.init():
            error(self.node,
                  "There is a problem initializing the dataset")
            return False

        if (self._limit_frame >= 0):
            self._dataset.set_frame_limit(end_frame)

        # Load TimeEstimator to get the speed metric
        self._time_estimator = TimeEstimatorHeader(self.node)

        # Load the lidar_euclidean_cluster_detect output formatter to get the
        # output of the lidar node and translate it to the kitti format
        self._output_formatter = EuclideanClusterNodeOutputFormatter(
            self.node,
            (result_path + self.OBJDET_METRIC_RESULT_PATH),
            dataset_path +
            Kitti3DBenchmarkDataset.KITTI_3D_BENCH_FOLDER_STRUCTURE[
                Kitti3DBenchmarkDataset.KITTI_3D_CALIB_PATH_IDX]
        )

        # Load the speed output formatter to get the output of the TimeEstimator
        # and save it to a file
        self._speed_formatter = GenericStreamFormatter(
            self.node,
            result_path + self.SPEED_METRIC_RESULT_PATH,
            self.SPEED_MEASURE_FILE_NAME
        )

        # Prepare the 3d object detection metric object
        self._kitti_3d_metric = Kitti3dObjectDetectionMetric(
            self.node,
            (result_path + self.OBJDET_METRIC_RESULT_PATH),
            self._dataset.get_ground_truth_path(),
            (result_path + self.OBJDET_METRIC_OUTPUT_PATH)
        )

        # Prepare the speed metric object
        self._speed_metric = AverageMetric(
            self.node,
            (result_path + self.SPEED_METRIC_RESULT_PATH),
            (result_path + self.SPEED_METRIC_OUTPUT_PATH),
            self.SPEED_MEASURE_FILE_NAME,
            self.SPEED_METRIC_OUTPUT_FILENAME
        )

        # Create the Player to play the data to the lidar node
        self._data_player = SyncedPlayer(self.node, self._dataset)
        if not self._data_player.add_track(
                self.POINT_CLOUD_DATASET_TRACK,
                input_topic,
                PointCloud2,
                output_topic,
                self.POINT_CLOUD_TRACK_MIN_SUBS,
                self.POINT_CLOUD_TRACK_RATE_HZ,
                self.POINT_CLOUD_TRACK_TIMEOUT_MS
        ):
            error(self.node,
                  "There is a problem initializing the player")
            return False

        # Create the relay player listening to the ray ground classifier
        # output topic, it is the preprocessing stage of this benchmarked node
        self._relay_player = RelayPlayer(
            self.node,
            benchmarked_in_topic,
            "/relay" + benchmarked_in_topic
        )

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
            error(self.node,
                  "Problem initializing the speed metric listener")
            return False

        # Listen the lidar node output
        if not self._output_formatter.start_output_listener(output_topic):
            error(self.node,
                  "Problem initializing the speed metric listener")
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
                info(self.node,
                     "No more data to play, waiting last frames...")

            self._shutdown_counter += 1

        # Return running state of the task: running
        return True

    def compute_results(self):
        """
        Compute the final benchmark results.

        @return: None
        """
        info(self.node, "Start metric computation...")
        # Compute the kitti 3D object detection metrics
        if self._limit_frame >= self.EXPECTED_KITTI_FRAME_ACCURACY:
            if not self._kitti_3d_metric.compute_metric():
                error(self.node,
                      "Problem computing 3D object detection metrics")
        else:
            info(self.node, "Accuracy computation needs a number of frame " +
                 "equal to %d. Skipping..." % self.EXPECTED_KITTI_FRAME_ACCURACY)

        # Compute the speed metric
        if not self._speed_metric.compute_metric("Speed during benchmark (milliseconds): " +
                                                 "\nMin: %.3f\nAverage: %.3f\nMax: %.3f"):
            error(self.node,
                  "Problem computing speed metrics")
