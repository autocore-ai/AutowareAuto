// Copyright 2019 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
/// \file
/// \brief This file implements a clustering node that published colored point clouds and convex
///        hulls

#include <geometry/bounding_box_2d.hpp>
#include <memory>
#include "euclidean_cluster_nodes/euclidean_cluster_node.hpp"

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace euclidean_cluster_nodes
{
namespace details
{
////////////////////////////////////////////////////////////////////////////////
BoundingBox compute_eigenbox(const euclidean_cluster::Cluster & cls)
{
  using euclidean_cluster::PointXYZI;
  //lint -e{826, 9176} NOLINT I claim this is ok and tested
  const auto begin = reinterpret_cast<const PointXYZI *>(&cls.data[0U]);
  //lint -e{826, 9176} NOLINT I claim this is ok and tested
  const auto end = reinterpret_cast<const PointXYZI *>(&cls.data[cls.row_step]);
  return common::geometry::bounding_box::eigenbox_2d(begin, end);
}
////////////////////////////////////////////////////////////////////////////////
BoundingBox compute_lfit_bounding_box(euclidean_cluster::Cluster & cls)
{
  using euclidean_cluster::PointXYZI;
  //lint -e{826, 9176} NOLINT I claim this is ok and tested
  const auto begin = reinterpret_cast<PointXYZI *>(&cls.data[0U]);
  //lint -e{826, 9176} NOLINT I claim this is ok and tested
  const auto end = reinterpret_cast<PointXYZI *>(&cls.data[cls.row_step]);
  return common::geometry::bounding_box::lfit_bounding_box_2d(begin, end);
}
////////////////////////////////////////////////////////////////////////////////
void compute_eigenboxes(const Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.size = 0U;
  for (auto & cls : clusters.clusters) {
    try {
      boxes.boxes[boxes.size] = compute_eigenbox(cls);
      ++boxes.size;
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void compute_eigenboxes_with_z(const Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.size = 0U;
  for (auto & cls : clusters.clusters) {
    try {
      auto & box = boxes.boxes[boxes.size];
      box = compute_eigenbox(cls);
      using euclidean_cluster::PointXYZI;
      //lint -e{826, 9176} NOLINT I claim this is ok and tested
      const auto begin = reinterpret_cast<const PointXYZI *>(&cls.data[0U]);
      //lint -e{826, 9176} NOLINT I claim this is ok and tested
      const auto end = reinterpret_cast<const PointXYZI *>(&cls.data[cls.row_step]);
      common::geometry::bounding_box::compute_height(begin, end, box);
      ++boxes.size;
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void compute_lfit_bounding_boxes(Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.size = 0U;
  for (auto & cls : clusters.clusters) {
    try {
      boxes.boxes[boxes.size] = compute_lfit_bounding_box(cls);
      ++boxes.size;
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void compute_lfit_bounding_boxes_with_z(Clusters & clusters, BoundingBoxArray & boxes)
{
  boxes.size = 0U;
  for (auto & cls : clusters.clusters) {
    try {
      auto & box = boxes.boxes[boxes.size];
      box = compute_lfit_bounding_box(cls);
      using euclidean_cluster::PointXYZI;
      //lint -e{826, 9176} NOLINT I claim this is ok and tested
      const auto begin = reinterpret_cast<const PointXYZI *>(&cls.data[0U]);
      //lint -e{826, 9176} NOLINT I claim this is ok and tested
      const auto end = reinterpret_cast<const PointXYZI *>(&cls.data[cls.row_step]);
      common::geometry::bounding_box::compute_height(begin, end, box);
      ++boxes.size;
    } catch (const std::exception & e) {
      std::cerr << e.what() << "\n";
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
}  // namespace details
}  // namespace euclidean_cluster_nodes
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware
