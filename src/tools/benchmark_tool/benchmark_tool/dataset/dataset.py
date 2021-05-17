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

from abc import ABCMeta, abstractmethod
from os import listdir
from os.path import isfile, join
from sensor_msgs.msg import PointCloud2


class Dataset(metaclass=ABCMeta):
    """
    The Dataset class is a generic interface to build a data source used during the benchmark.

    A class conform to this interface holds various lists to data sources like point cloud, images
    and so on.
    """

    def __init__(self, node, path):
        """
        Create a Dataset object.

        @param self: The object pointer
        @param node: ROS2 node
        @type  node: rclpy.node.Node
        @param path: The path on filesystem of the dataset used for the benchmark
        @type  path: str
        """
        super(Dataset, self).__init__()
        self._root_path = path
        self._sources = {PointCloud2.__name__: []}
        self._pointcloud_sources = self._sources[PointCloud2.__name__]
        self.node = node

    @abstractmethod
    def init(self):
        """
        Initialize the internal structure of the dataset.

        @param self: The object pointer
        @return: True on success, False on failure
        """
        pass

    def set_frame_limit(self, end_frame=-1):
        """
        Set a frame limitation to the sources of this dataset.

        @param self: The object pointer
        @param end_frame: The limit number, negative if unlimited
        @return: None
        """
        if(end_frame >= 0):
            for elem in self.get_pointcloud_sources():
                elem.set_max_frame_count(end_frame)

    def get_root_path(self):
        """
        Return the path on filesystem of the dataset used for the benchmark.

        @param self: The object pointer
        @return: str
        """
        return self._root_path

    @abstractmethod
    def get_ground_truth_path(self):
        """
        Return the path on filesystem of the ground truth for this dataset.

        @param self: The object pointer
        @return: str
        """
        pass

    def get_sources(self, data_type):
        """
        Return a list of DatasetElem objects holding information about the data source.

        @param self: The object pointer
        @param data_type: The data type of the pointed data sources
        @return: List of DatasetElem objects
        """
        if (not isinstance(data_type, str)):
            data_type = data_type.__name__
        sources = []
        try:
            sources = self._sources[data_type]
        except Exception:
            sources = []
        return sources

    def get_pointcloud_sources(self):
        """
        Return a list of DatasetElem objects holding information about the data sources.

        @param self: The object pointer
        @return: List of DatasetElem objects
        """
        return self._pointcloud_sources

    def get_source_length(self, data_type, sourceIndex):
        """
        Return the size in frames of the pointed data source.

        @param self: The object pointer
        @param data_type: The data type of the pointed data sources
        @param sourceIndex: The index of the data source
        @return: Int
        """
        sources = self.get_sources(data_type)
        if((sourceIndex > len(sources)) or (sourceIndex < 0)):
            return 0
        return sources[sourceIndex].count()


class DatasetElem(object):
    """
    The DatasetElem class represents a generic data source.

    It is a cursor that abstracts the underlying data.
    This object takes a path on the filesystem and reads all the files inside, then it holds a list
    of these files, giving access through simple generic functions.
    """

    def __init__(self, folder):
        """
        Create a DatasetElem object.

        Reads a given folder and creates a list of the files.

        @param self: The object pointer
        @param folder: The path on filesystem of the data source
        @type  folder: str
        @return: None
        """
        self._folder = folder
        self._current_elem = 0
        self._folder_files = []

        for filename in listdir(self._folder):
            if isfile(join(self._folder, filename)):
                self._folder_files.append(filename)
        self._max_frame_count = len(self._folder_files)

    def begin(self):
        """
        Seek the beginning of the data source.

        @param self: The object pointer
        @return: None
        """
        self._current_elem = 0

    def is_end(self):
        """
        Check if the data source reached its end.

        @param self: The object pointer
        @return: True if the data is finished, False otherwise
        """
        return (self._current_elem >= self._max_frame_count)

    def set_max_frame_count(self, max_frame_count):
        """
        Set the max number of frames considered for this element.

        It effectively limits the number of frames playable.

        @param self: The object pointer
        @param max_frame_count: The new max frame count.
        @return: True if the new max_frame_count is valid, False otherwise
        """
        if ((max_frame_count <= len(self._folder_files))
                and (max_frame_count >= 0)):
            self._max_frame_count = max_frame_count
            return True
        return False

    def next(self):
        """
        Make the cursor move forward of one position.

        @param self: The object pointer
        @return: None
        """
        if not self.is_end():
            self._current_elem += 1

    def back(self):
        """
        Make the cursor move backward of one position.

        @param self: The object pointer
        @return: None
        """
        if self._current_elem > 0:
            self._current_elem -= 1

    def sort_data(self):
        """
        Sort the files of this data source in alphabetical order.

        @param self: The object pointer
        @return: None
        """
        self._folder_files.sort()

    def count(self):
        """
        Return the number of element composing this data source.

        @param self: The object pointer
        @return: int representing the number of element of this data source
        """
        return self._max_frame_count

    def get_cursor_filename(self):
        """
        Return the file name of the file currently pointed by this object.

        @param self: The object pointer
        @return: str representing the file name or EOF if the data source reached its end
        """
        file_name = 'EOF'
        if (not self.is_end()) and (len(self._folder_files) > 0):
            file_name = self._folder + '/' + \
                self._folder_files[self._current_elem]
        return file_name

    @abstractmethod
    def get_frame(self):
        """
        Retrieve the current data pointed by the cursor.

        @param self: The object pointer
        @return: Depends on the data
        """
        pass
