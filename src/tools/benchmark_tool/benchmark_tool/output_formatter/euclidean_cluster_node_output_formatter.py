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
import numpy as np
from benchmark_tool.output_formatter.output_formatter import OutputFormatter
from benchmark_tool.utility import error, euler_from_quaternion
from autoware_auto_msgs.msg import BoundingBoxArray
from benchmark_tool.kittiobjdetsdk.kitti_obj_detection_utils \
    import KittiObjDetectionUtils


class EuclideanClusterNodeOutputFormatter(OutputFormatter):
    """
    The EuclideanClusterNodeOutputFormatter class is a specialized OutputFormatter class.

    It listens to the output topic of the euclidean_cluster_node node and transform the received
    data into the kitti object detection format.
    """

    AUTOWARE_AUTO_MSG_BOUNDING_BOX_LABELS = [
        "NO_LABEL",
        "Car",
        "Pedestrian",
        "Cyclist",
        "MOTORCYCLE",
        "NO_SIGNAL",
        "LEFT_SIGNAL",
        "RIGHT_SIGNAL",
        "BRAKE",
        "POSE_X",
        "POSE_Y",
        "VELOCITY",
        "HEADING",
        "TURN_RATE",
        "SIZE_X",
        "SIZE_Y",
        "ACCELERATION"
    ]

    def __init__(self, node, result_path, calibration_path):
        """
        Create a EuclideanClusterNodeOutputFormatter object.

        @param node: ROS2 node
        @type  node: rclpy.node.Node
        @param result_path: The path on filesystem where to save the received output in kitti
            object detection format
        @type  result_path: str
        @param calibration_path: The path on filesystem where the calibration files are saved
        @type  calibration_path: str
        """
        super(EuclideanClusterNodeOutputFormatter, self).__init__(node,
                                                                  result_path)
        self._file_index = 0
        self._calibration_path = calibration_path

    def start_output_listener(self, topic):
        """
        Start the subscriber on the specified topic.

        Check the existence of the calibration folder path on filesystem

        @param topic: The topic to listen for the data
        @type  topic: str
        @return: True on success, False on failure
        """
        # Check the existance of the calibration folder
        if not os.path.isdir(self._calibration_path):
            error(self.node,
                  "Calibration path %s does not exists" % self._calibration_path)
            error(self.node,
                  "Please create an instance with a correct folder path")
            return False

        # Creates the results folder if not exists
        if not OutputFormatter.create_folder(self._result_path):
            return False

        try:
            # Subscribe to Object Detection topic
            self._subscriber = self.node.create_subscription(
                BoundingBoxArray,
                topic,
                self._handle_object_detection_output,
                qos_profile=QoSProfile(depth=1)
            )
        except Exception as e:
            error(self.node, "%s" % str(e))
            return False

        self._file_index = 0

        # Remove everything inside that folder
        if not OutputFormatter.clean_folder(self._result_path):
            return False

        return True

    def _handle_object_detection_output(self, msg):
        """
        Save a new text files containing all the detection using the kitti object detection format.

        This function is a callback for the euclidean_cluster_node output topic node.
        Each autoware_auto_msgs.msg.BoundingBoxArray message received creates a text file whose
        name is incremental.
        In this function the output of the node is transformed and formatted into the format
        expected by the kitti object detection SDK.
        The kitti SDK expects data in camera rectified reference frame for the 3d data and it
        expects 2d coordinates into left color camera reference frame.

        @param msg: Received message from euclidean_cluster_node node
        @type  msg: autoware_auto_msgs.msg.BoundingBoxArray
        @return: None
        """
        assert isinstance(msg, BoundingBoxArray)

        # create path and filename for the output file
        # the format is:
        # <root_path>/<result_folder>/<zero padded incremental number>.txt
        file_name = os.path.join(self._result_path, "%06d.txt" %
                                 self._file_index)

        try:
            # Create the file
            file = open(file_name, "w")
            calibration_matrices = {}

            # Load Kitti calibration matrices only if there are detections
            if len(msg.boxes) > 0:
                calibration_matrices = \
                    KittiObjDetectionUtils.load_calibration_matrices(
                        self._calibration_path,
                        self._file_index)

            for detection in msg.boxes:
                kitti_det = KittiObjDetectionUtils.KittiDetection()

                # Extract yaw angle of the bounding box
                (_, _, yaw_angle) = euler_from_quaternion(
                    detection.orientation.w,
                    detection.orientation.x,
                    detection.orientation.y,
                    detection.orientation.z
                )

                # Load detection position to a vector
                position = np.zeros((3, 1))
                position[0, 0] = detection.centroid.x
                position[1, 0] = detection.centroid.y
                position[2, 0] = detection.centroid.z

                # Transform reference frame from velodine to camera
                position = \
                    KittiObjDetectionUtils.transform_velodyne_to_camera_frame(
                        position,
                        calibration_matrices)

                # Euclidean cluster node doesn't specify if the item is a car,
                # pedestrian or anything, it just clusters a cloud of points.
                # Here we hardcode the label Car
                kitti_det.obj_type = "Car"
                kitti_det.box3d_pos_x = position[0, 0]
                kitti_det.box3d_pos_y = position[1, 0]
                kitti_det.box3d_pos_z = position[2, 0]
                kitti_det.box3d_height = detection.size.z
                kitti_det.box3d_width = detection.size.y
                kitti_det.box3d_length = detection.size.x
                kitti_det.box3d_yaw = yaw_angle

                # The euclidean cluster node doesn't set a score, we put 1.0
                kitti_det.score = 1.0

                # Compute 3D Bounding box
                bbox_3d = KittiObjDetectionUtils.compute_3d_bbox(kitti_det)

                # Project 3D bounding box to camera image plane
                proj_bbox_3d = \
                    KittiObjDetectionUtils.project_to_camera_left_color(bbox_3d,
                                                                        calibration_matrices)

                # Get top left corner and bottom right corner of the 2D
                # bounding box that encloses the projection of the 3D box
                # on the camera image plane
                kitti_det.box2d_x1, kitti_det.box2d_y1, kitti_det.box2d_x2, \
                    kitti_det.box2d_y2 = \
                    KittiObjDetectionUtils.bbox_2d_from_3d_bbox_projection(
                        proj_bbox_3d)

                # Write the detection to the output file
                file.write(kitti_det.to_string() + "\n")

            file.close()
        except Exception as e:
            error(self.node, "%s" % str(e))

        self._file_index += 1
