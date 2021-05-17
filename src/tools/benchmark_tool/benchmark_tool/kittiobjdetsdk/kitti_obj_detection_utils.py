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
import numpy as np


class KittiObjDetectionUtils:
    """
    A collection of utility functions aimed at managing the kitti object detection output format.

    It provides the right structure for the detection and also methods to construct 2d/3d bounding
    boxes, transform reference frame and load calibration matrices.
    """

    class KittiDetection(object):
        """
        The KittiDetection class represents an entry in the kitti object detection results file.

        It provides default values and the right structure for the string line to be saved in an
        output file in the kitti object detection format.
        """

        def __init__(self):
            """
            Create a KittiDetection object.

            @param self: The object pointer
            """
            self.obj_type = ""
            self.truncated = float(-1.0)
            self.occlusion = int(-1)
            self.observ_angle = float(-10.0)
            self.box2d_x1 = float(0.0)
            self.box2d_y1 = float(0.0)
            self.box2d_x2 = float(0.0)
            self.box2d_y2 = float(0.0)
            self.box3d_height = float(-1)
            self.box3d_width = float(-1)
            self.box3d_length = float(-1)
            self.box3d_pos_x = float(-1000)
            self.box3d_pos_y = float(-1000)
            self.box3d_pos_z = float(-1000)
            self.box3d_yaw = float(-10)
            self.score = float(0.0)

        def to_string(self):
            """
            Return a string representing a detection.

            It is properly formatted for kitti object detection benchmark.

            @param self: The object pointer
            @return: str
            """
            kitti_entry = [
                self.obj_type,
                round(self.truncated, 2),
                self.occlusion,
                round(self.observ_angle, 2),
                round(self.box2d_x1, 2),
                round(self.box2d_y1, 2),
                round(self.box2d_x2, 2),
                round(self.box2d_y2, 2),
                round(self.box3d_height, 2),
                round(self.box3d_width, 2),
                round(self.box3d_length, 2),
                round(self.box3d_pos_x, 2),
                round(self.box3d_pos_y, 2),
                round(self.box3d_pos_z, 2),
                round(self.box3d_yaw, 2),
                round(self.score, 2)
            ]
            kitti_entry = [str(x) for x in kitti_entry]
            return " ".join(kitti_entry)

    @staticmethod
    def load_calibration_matrices(calibration_path, frame_number):
        """
        Return a dictionary containing the calibration matrices for the provided frame number.

        @param calibration_path: The path on the filesystem for the folder containing the
            calibration files for each frame
        @type  calibration_path:str
        @param frame_number: The frame number
        @type  frame_number: int
        @return: dict containing the calibration matrices
        """
        file_path = os.path.join(calibration_path, "%06d.txt" %
                                 frame_number)
        matrices = {}
        for line in open(file_path, "r"):
            line = line.rstrip()
            if line:
                key, val = line.split(":", 1)
                matrices[key] = np.array([float(x) for x in val.split()])

                # Shape the matrix, R0_rect is the only 3x3, others are 3x4
                if key == "R0_rect":
                    matrices[key] = np.reshape(matrices[key], [3, 3])
                else:
                    matrices[key] = np.reshape(matrices[key], [3, 4])

        return matrices

    @staticmethod
    def compute_3d_bbox(kitti_detection_elem):
        """
        Provide the 3d point corners of the bounding box of the specified kitti detection object.

        @param kitti_detection_elem: The kitti detection object
        @type  kitti_detection_elem: KittiDetection
        @return: numpy array 3x8
        """
        c = np.cos(kitti_detection_elem.box3d_yaw)
        s = np.sin(kitti_detection_elem.box3d_yaw)
        # Build rotation matrix
        Rot = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

        l = kitti_detection_elem.box3d_length  # noqa: E741
        w = kitti_detection_elem.box3d_width
        h = kitti_detection_elem.box3d_height

        # 3d bounding box corners
        x_edges = [l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2]
        y_edges = [0, 0, 0, 0, -h, -h, -h, -h]
        z_edges = [w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2]

        # apply rotation to the bounding box
        # (3x3) dot (3x8) = (3x8)
        bbox_coords = np.dot(Rot, np.vstack([x_edges, y_edges, z_edges]))

        # apply translation
        bbox_coords[0, :] = bbox_coords[0, :] + kitti_detection_elem.box3d_pos_x
        bbox_coords[1, :] = bbox_coords[1, :] + kitti_detection_elem.box3d_pos_y
        bbox_coords[2, :] = bbox_coords[2, :] + kitti_detection_elem.box3d_pos_z

        return bbox_coords

    @staticmethod
    def transform_velodyne_to_camera_frame(points_3d, calibration_matrices):
        """
        Transform the provided 3d points from velodyne to camera-rectified reference frame.

        @param points_3d: The point in 3d space to be transform
        @type  points_3d: numpy array 3xN
        @param calibration_matrices: The calibration matrices dictionary
        @type  calibration_matrices: dict
        @return: numpy array 3xN
        """
        assert (points_3d.shape[0] == 3)
        # (3xn) => (4xn)
        n = points_3d.shape[1]
        ones = np.ones((1, n))
        points_3d = np.vstack([points_3d[0, :], points_3d[1, :],
                               points_3d[2, :], ones])

        # Velo to cam matrix
        v2c = calibration_matrices["Tr_velo_to_cam"]  # 3x4
        v2c = np.vstack([v2c[0, :], v2c[1, :], v2c[2, :], np.zeros((1, 4))])  # 4x4
        v2c[3, 3] = 1

        # Rect image
        R0_rect = calibration_matrices["R0_rect"]  # 3x3
        R0_rect = np.vstack([R0_rect[0, :], R0_rect[1, :], R0_rect[2, :],
                             np.zeros((1, 3))])  # 4x3
        R0_rect = np.hstack((R0_rect, np.zeros((4, 1))))  # 4x4
        R0_rect[3, 3] = 1

        # velo_to_cam dot pointcloud_ref_frame
        intermediate = np.dot(v2c, points_3d)  # 4x4 dot 4xn = 4xn

        # R0_rect dot (velo_to_cam dot pointcloud_ref_frame)
        intermediate = np.dot(R0_rect, intermediate)  # 4x4 dot 4xn = 4xn

        intermediate = np.vstack([intermediate[0, :], intermediate[1, :],
                                  intermediate[2, :]])  # 3xn

        return intermediate

    @staticmethod
    def project_to_camera(points_3d, proj_matrix):
        """
        Project the provided 3d points into the camera image plane using a projection matrix.

        @param points_3d: The point in 3d space to be projected into the plane
        @type  points_3d: numpy array 3xN
        @param proj_matrix: The projection matrix
        @type  proj_matrix: numpy array 3x4
        @return: numpy array 2xN
        """
        assert (points_3d.shape[0] == 3)
        # (3xn) => (4xn)
        n = points_3d.shape[1]
        ones = np.ones((1, n))
        points_3d = np.vstack([points_3d[0, :], points_3d[1, :],
                               points_3d[2, :], ones])

        # Project to the 2d plane
        # (3x4) dot (4xn) = (3xn)
        projection_2d = np.dot(proj_matrix, points_3d)

        # normalize along z axis
        projection_2d[0, :] /= projection_2d[2, :]
        projection_2d[1, :] /= projection_2d[2, :]

        return projection_2d[0:2, :]

    @staticmethod
    def project_to_camera_left_grayscale(points_3d, calibration_matrices):
        """
        Project the provided 3d points into the camera left grayscale image plane.

        The projection is done using the kitti object detection projection matrix P0.

        @param points_3d: The point in 3d space to be projected into the plane
        @type  points_3d: numpy array 3xN
        @param calibration_matrices: The calibration matrices dictionary
        @type  calibration_matrices: dict
        @return: numpy array 2xN
        """
        return KittiObjDetectionUtils.project_to_camera(points_3d,
                                                        calibration_matrices["P0"])

    @staticmethod
    def project_to_camera_right_grayscale(points_3d, calibration_matrices):
        """
        Project the provided 3d points into the camera right grayscale image plane.

        The projection is done using the kitti object detection projection matrix P1.

        @param points_3d: The point in 3d space to be projected into the plane
        @type  points_3d: numpy array 3xN
        @param calibration_matrices: The calibration matrices dictionary
        @type  calibration_matrices: dict
        @return: numpy array 2xN
        """
        return KittiObjDetectionUtils.project_to_camera(points_3d,
                                                        calibration_matrices["P1"])

    @staticmethod
    def project_to_camera_left_color(points_3d, calibration_matrices):
        """
        Project the provided 3d points into the camera left color image plane.

        The projection is done using the kitti object detection projection matrix P2.

        @param points_3d: The point in 3d space to be projected into the plane
        @type  points_3d: numpy array 3xN
        @param calibration_matrices: The calibration matrices dictionary
        @type  calibration_matrices: dict
        @return: numpy array 2xN
        """
        return KittiObjDetectionUtils.project_to_camera(points_3d,
                                                        calibration_matrices["P2"])

    @staticmethod
    def project_to_camera_right_color(points_3d, calibration_matrices):
        """
        Project the provided 3d points into the camera right color image plane.

        The projection is done using the kitti object detection projection matrix P3.

        @param points_3d: The point in 3d space to be projected into the plane
        @type  points_3d: numpy array 3xN
        @param calibration_matrices: The calibration matrices dictionary
        @type  calibration_matrices: dict
        @return: numpy array 2xN
        """
        return KittiObjDetectionUtils.project_to_camera(points_3d,
                                                        calibration_matrices["P3"])

    @staticmethod
    def bbox_2d_from_3d_bbox_projection(bbox_3d_proj):
        """
        Provide a 2d bounding box that encloses the 3d bounding box projection into the plane.

        @param bbox_3d_proj: 2d coordinates on the plane of the 3d bounding box projection
        @type  bbox_3d_proj: numpy array 2xN
        @return: int/float - top left corner X axis coordinate, top left corner Y axis coordinate,
            bottom right corner X axis coordinate, bottom right corner Y axis coordinate
        """
        assert (bbox_3d_proj.shape[0] == 2)
        assert (bbox_3d_proj.shape[1] >= 0)

        top_left_corner = np.array([2**32, 2**32])
        bottom_right_corner = np.array([0, 0])

        for i in range(0, bbox_3d_proj.shape[1]):
            point = np.array([bbox_3d_proj[0, i], bbox_3d_proj[1, i]])

            if top_left_corner[0] > point[0]:
                top_left_corner[0] = point[0]

            if top_left_corner[1] > point[1]:
                top_left_corner[1] = point[1]

            if bottom_right_corner[0] < point[0]:
                bottom_right_corner[0] = point[0]

            if bottom_right_corner[1] < point[1]:
                bottom_right_corner[1] = point[1]

        # x1, y1, x2, y2
        return top_left_corner[0], top_left_corner[1], bottom_right_corner[0], \
            bottom_right_corner[1]
