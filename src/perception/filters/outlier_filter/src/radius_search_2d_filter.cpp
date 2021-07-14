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

#include <vector>
#include <memory>

#include "outlier_filter/radius_search_2d_filter.hpp"

#include "pcl/search/kdtree.h"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter
{
namespace radius_search_2d_filter
{

RadiusSearch2DFilter::RadiusSearch2DFilter(double search_radius, int min_neighbors)
: search_radius_(search_radius), min_neighbors_(min_neighbors)
{
  kd_tree_ = std::make_shared<pcl::search::KdTree<pcl::PointXY>>(false);
}

void RadiusSearch2DFilter::filter(
  const pcl::PointCloud<pcl::PointXYZ> & input,
  pcl::PointCloud<pcl::PointXYZ> & output)
{
  pcl::PointCloud<pcl::PointXY>::Ptr xy_cloud(new pcl::PointCloud<pcl::PointXY>());
  xy_cloud->points.resize(input.points.size());
  for (size_t i = 0; i < input.points.size(); ++i) {
    xy_cloud->points[i].x = input.points[i].x;
    xy_cloud->points[i].y = input.points[i].y;
  }

  std::vector<int> k_indices(xy_cloud->points.size());
  std::vector<float> k_dists(xy_cloud->points.size());
  kd_tree_->setInputCloud(xy_cloud);
  for (size_t i = 0; i < xy_cloud->points.size(); ++i) {
    auto k = kd_tree_->radiusSearch(static_cast<int>(i), search_radius_, k_indices, k_dists);
    if (k >= min_neighbors_) {
      output.points.push_back(input.points.at(i));
    }
  }
}

}  // namespace radius_search_2d_filter
}  // namespace outlier_filter
}  // namespace filters
}  // namespace perception
}  // namespace autoware
