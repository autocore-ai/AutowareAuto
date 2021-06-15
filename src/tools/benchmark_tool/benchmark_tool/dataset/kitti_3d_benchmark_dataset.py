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
from benchmark_tool.dataset.dataset import Dataset
from benchmark_tool.dataset.dataset import DatasetElem
from sensor_msgs.msg import PointCloud2, PointField
from benchmark_tool.utility import error


class Kitti3DBenchmarkDataset(Dataset):
    """
    The Kitti3DBenchmarkDataset class represents the kitti 3D object detection benchmark.

    It provides access to the kitti data, the class holds the information about the kitti dataset
    filesystem structure.
    """

    KITTI_3D_BENCH_FOLDER_STRUCTURE = [
        "",
        "/testing",
        "/testing/calib",
        "/testing/image_2",
        "/testing/image_3",
        "/testing/velodyne",
        "/training",
        "/training/calib",
        "/training/image_2",
        "/training/image_3",
        "/training/label_2",
        "/training/velodyne"
    ]

    KITTI_3D_BENCHMARK_PATH_IDX = 6
    KITTI_3D_CALIB_PATH_IDX = 7
    KITTI_3D_GROUNDTRUTH_PATH_IDX = 10
    KITTI_3D_POINTCLOUD_PATH_IDX = 11

    def __init__(self, node, path):
        """
        Create a Kitti3DBenchmarkDataset object.

        @param node: ROS2 node
        @type  node: rclpy.node.Node
        @param path: The path on filesystem for the kitti benchmark
        @type  path: str
        """
        super(Kitti3DBenchmarkDataset, self).__init__(node, path)

    def init(self):
        """
        Load the data files composing the kitti benchmark.

        @return: True on success, False on failure
        """
        # Check filesystem folder structure
        if not self._check_structure():
            return False

        # Load the point cloud dataset
        velodyne_data = KittiPointCloud(
            self.node,
            self._root_path + self.KITTI_3D_BENCH_FOLDER_STRUCTURE[
                self.KITTI_3D_POINTCLOUD_PATH_IDX]
        )

        velodyne_data.sort_data()

        self._pointcloud_sources.append(velodyne_data)

        return True

    def get_ground_truth_path(self):
        """
        Return the path on filesystem of the ground truth of the kitti 3D.

        object detection benchmark
        @return: str
        """
        return self.get_root_path() + self.KITTI_3D_BENCH_FOLDER_STRUCTURE[
            self.KITTI_3D_GROUNDTRUTH_PATH_IDX
        ]

    def _check_structure(self):
        """
        Check if the provided folder has the required folder structure.

        @return: True on success, False on failure
        """
        for folder in self.KITTI_3D_BENCH_FOLDER_STRUCTURE:
            path_to_check = self._root_path + folder
            if not os.path.isdir(path_to_check):
                error(self.node,
                      "Found problem in the folder structure!")
                error(self.node,
                      "The path: %s does not exists" % path_to_check)
                return False
        return True


class KittiPointCloud(DatasetElem):
    """
    The KittiPointCloud class is a data source for the kitti 3D object detection benchmark.

    It offers an interface to browse and retrieve the data in PointCloud2 message format.
    """

    def __init__(self, node, folder):
        """
        Create a KittiPointCloud object.

        @param node: ROS2 node
        @type  node: rclpy.node.Node
        @param folder: The path on filesystem of the point cloud data
        @type  folder: str
        """
        super(KittiPointCloud, self).__init__(folder)
        self.node = node

    def get_frame(self):
        """
        Retrieve the point cloud binary data pointed by the cursor in PointCloud2 message format.

        @return: sensor_msgs.msg.PointCloud2 message on success, None on failure
        """
        pc2_frame = PointCloud2()

        if not self.is_end():
            file_name = self._folder_files[self._current_elem]
            file_name = self._folder + '/' + file_name

            try:
                # Open the file as binary
                file = open(file_name, "rb")

                # Get the file size
                file_size = os.path.getsize(file_name)

                # Fill the PointCloud2 fields
                pc2_frame.header.frame_id = "base_link"
                pc2_frame.height = 1
                pc2_frame.point_step = 16
                pc2_frame.width = int(file_size / pc2_frame.point_step)

                # Data has x,y,z and intensity
                pc2_frame.fields = [
                    PointField(name='x', offset=0,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8,
                               datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12,
                               datatype=PointField.FLOAT32, count=1)
                ]

                pc2_frame.is_bigendian = False
                pc2_frame.row_step = pc2_frame.width * \
                    pc2_frame.point_step
                pc2_frame.is_dense = False

                # Copy the content of the file into the message data
                pc2_frame.data = file.read()

                # Set header with current time
                pc2_frame.header.stamp = self.node.get_clock().now().to_msg()

                # Close the file
                file.close()
            except Exception as e:
                error(self.node, "Exception when reading %s" % file_name)
                error(self.node, "get_frame: %s" % str(e))
                return None

        else:
            return None

        return pc2_frame
