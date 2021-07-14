// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "outlier_filter/voxel_grid_outlier_filter.hpp"

#include <memory>

namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter
{
namespace voxel_grid_outlier_filter
{

VoxelGridOutlierFilter::VoxelGridOutlierFilter(
  float voxel_size_x, float voxel_size_y, float voxel_size_z,
  uint32_t voxel_points_threshold)
: voxel_size_x_(voxel_size_x), voxel_size_y_(voxel_size_y), voxel_size_z_(voxel_size_z),
  voxel_points_threshold_(voxel_points_threshold)
{
  voxel_filter_ = std::make_shared<pcl::VoxelGrid<pcl::PointXYZ>>();
}

void VoxelGridOutlierFilter::filter(
  const pcl::PointCloud<pcl::PointXYZ> & input,
  pcl::PointCloud<pcl::PointXYZ> & output)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_voxelized_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_voxelized_input->points.reserve(input.points.size());

  // Set parameters for voxel filter object
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc_input(new pcl::PointCloud<pcl::PointXYZ>(input));
  voxel_filter_->setInputCloud(pc_input);
  // Leaf size set to true to save a copy of the tree layout for faster access when invoking
  // getCentroidIndexAt
  // Ref: https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html#a57f1511c023294989ab5ae83bc439ae3
  voxel_filter_->setSaveLeafLayout(true);
  voxel_filter_->setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
  voxel_filter_->setMinimumPointsNumberPerVoxel(voxel_points_threshold_);
  voxel_filter_->filter(*pcl_voxelized_input);

  output.points.reserve(input.points.size());
  for (size_t i = 0; i < input.points.size(); ++i) {
    const int index = voxel_filter_->getCentroidIndexAt(
      voxel_filter_->getGridCoordinates(
        input.points.at(i).x, input.points.at(i).y, input.points.at(i).z));
    if (index != -1) {  // not empty voxel
      output.points.push_back(input.points.at(i));
    }
  }
}
}  // namespace voxel_grid_outlier_filter
}  // namespace outlier_filter
}  // namespace filters
}  // namespace perception
}  // namespace autoware
