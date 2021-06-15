// Copyright 2019-2021 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#include <lidar_utils/point_cloud_utils.hpp>
#include <cstring>
//lint -e537 NOLINT Repeated include file: pclint vs cpplint
#include <algorithm>
#include <string>
//lint -e537 NOLINT Repeated include file: pclint vs cpplint
#include <utility>
#include "euclidean_cluster/euclidean_cluster.hpp"
#include "geometry/bounding_box_2d.hpp"

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace euclidean_cluster
{
////////////////////////////////////////////////////////////////////////////////
PointXYZIR::PointXYZIR(const common::types::PointXYZIF & pt)
: m_point{pt.x, pt.y, pt.z, pt.intensity},
  m_r_xy{sqrtf((pt.x * pt.x) + (pt.y * pt.y))}
{
}
////////////////////////////////////////////////////////////////////////////////
PointXYZIR::PointXYZIR(const PointXYZI & pt)
: m_point{pt},
  m_r_xy{sqrtf((pt.x * pt.x) + (pt.y * pt.y))}
{
}
////////////////////////////////////////////////////////////////////////////////
PointXYZIR::PointXYZIR(
  const float32_t x,
  const float32_t y,
  const float32_t z,
  const float32_t intensity)
: m_point{x, y, z, intensity},
  m_r_xy{std::hypotf(x, y)}
{
}
////////////////////////////////////////////////////////////////////////////////
float32_t PointXYZIR::get_r() const
{
  return m_r_xy;
}
////////////////////////////////////////////////////////////////////////////////
const PointXYZI & PointXYZIR::get_point() const
{
  return m_point;
}
////////////////////////////////////////////////////////////////////////////////
PointXYZIR::operator autoware_auto_msgs::msg::PointXYZIF() const
{
  /*lint -e{1793} it's safe to call non-const member function on a temporary in this case*/
  autoware_auto_msgs::msg::PointXYZIF ret;

  ret.x = m_point.x;
  ret.y = m_point.y;
  ret.z = m_point.z;
  ret.intensity = m_point.intensity;
  return ret;
}
////////////////////////////////////////////////////////////////////////////////
Config::Config(
  const std::string & frame_id,
  const std::size_t min_cluster_size,
  const std::size_t max_num_clusters,
  const float32_t min_cluster_threshold_m,
  const float32_t max_cluster_threshold_m,
  const float32_t cluster_threshold_saturation_distance_m)
: m_frame_id(frame_id),
  m_min_cluster_size(min_cluster_size),
  m_max_num_clusters(max_num_clusters),
  m_min_thresh_m(min_cluster_threshold_m),
  m_max_distance_m(cluster_threshold_saturation_distance_m),
  m_thresh_rate((max_cluster_threshold_m - min_cluster_threshold_m) /
    cluster_threshold_saturation_distance_m)
{
  // TODO(c.ho) sanity checking
}
////////////////////////////////////////////////////////////////////////////////
std::size_t Config::min_cluster_size() const
{
  return m_min_cluster_size;
}
////////////////////////////////////////////////////////////////////////////////
std::size_t Config::max_num_clusters() const
{
  return m_max_num_clusters;
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::threshold(const PointXYZIR & pt) const
{
  return threshold(pt.get_r());
}
////////////////////////////////////////////////////////////////////////////////
float32_t Config::threshold(const float32_t r) const
{
  return m_min_thresh_m + (std::min(m_max_distance_m, r) * m_thresh_rate);
}
////////////////////////////////////////////////////////////////////////////////
const std::string & Config::frame_id() const
{
  return m_frame_id;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
EuclideanCluster::EuclideanCluster(const Config & cfg, const HashConfig & hash_cfg)
: m_config(cfg),
  m_hash(hash_cfg),
  m_last_error(Error::NONE)
{}
////////////////////////////////////////////////////////////////////////////////
bool Config::match_clusters_size(const Clusters & clusters) const
{
  bool ret = true;
  if (clusters.cluster_boundary.capacity() < m_max_num_clusters) {
    ret = false;
  }
  if (clusters.points.capacity() < (m_max_num_clusters * m_min_cluster_size)) {
    ret = false;
  }
  return ret;
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::insert(const PointXYZIR & pt)
{
  // can't do anything with return values
  (void)m_hash.insert(pt);
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cluster(Clusters & clusters)
{
  // Clean the previous clustering result
  clusters.points.clear();
  clusters.cluster_boundary.clear();
  cluster_impl(clusters);
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::throw_stored_error() const
{
  // Error handling after publishing
  switch (get_error()) {
    case Error::TOO_MANY_CLUSTERS:
      throw std::runtime_error{"EuclideanCluster: Too many clusters"};
    case Error::NONE:
    default:
      break;
  }
}
////////////////////////////////////////////////////////////////////////////////
EuclideanCluster::Error EuclideanCluster::get_error() const
{
  return m_last_error;
}
////////////////////////////////////////////////////////////////////////////////
const Config & EuclideanCluster::get_config() const
{
  return m_config;
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cluster_impl(Clusters & clusters)
{
  m_last_error = Error::NONE;
  auto it = m_hash.begin();
  while (it != m_hash.end()) {
    cluster(clusters, it);
    // Go to next point still in hash; points assigned to a cluster are removed from hash
    it = m_hash.begin();
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cluster(Clusters & clusters, const Hash::IT it)
{
  // init new cluster
  if (clusters.cluster_boundary.size() >= m_config.max_num_clusters()) {
    m_last_error = Error::TOO_MANY_CLUSTERS;
    // Flush the remaining points in the hash for the new scan
    m_hash.clear();
  } else {
    // initialize new cluster in clusters
    if (clusters.cluster_boundary.empty()) {
      clusters.cluster_boundary.emplace_back(0U);
    } else {
      clusters.cluster_boundary.emplace_back(clusters.cluster_boundary.back());
    }
    // Seed cluster with new point
    add_point_to_last_cluster(clusters, it->second);
    // Erase returns the element after the removed element but it is not useful here
    (void)m_hash.erase(it);
    // Start clustering process
    std::size_t last_cls_pt_idx = 0U;
    while (last_cls_pt_idx < last_cluster_size(clusters)) {
      const auto pt = get_point_from_last_cluster(clusters, last_cls_pt_idx);
      add_neighbors_to_last_cluster(clusters, pt);
      // Increment seed point
      ++last_cls_pt_idx;
    }
    // check if cluster is large enough
    if (last_cls_pt_idx < m_config.min_cluster_size()) {
      // reject the cluster if too small
      clusters.cluster_boundary.pop_back();
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::add_neighbors_to_last_cluster(
  Clusters & clusters,
  const EuclideanCluster::PointXY pt)
{
  // TODO(c.ho) make this more generic... also duplicated work..
  const float32_t r = sqrtf((pt.x * pt.x) + (pt.y * pt.y));
  const float32_t thresh1 = m_config.threshold(r);
  // z is not needed since it's a 2d hash
  const auto & nbrs = m_hash.near(pt.x, pt.y, thresh1);
  // For each point within a fixed radius, check for connectivity
  for (const auto itd : nbrs) {
    const auto & qt = itd.get_point();
    // Ensure that threshold is satisfied bidirectionally
    const float32_t thresh2 = m_config.threshold(qt);
    if (itd.get_distance() <= thresh2) {
      // Add to the last cluster
      add_point_to_last_cluster(clusters, qt);
      // Remove from hash: point is already assigned to a cluster; never need to see again
      // (equivalent to marking a point as "seen")
      (void)m_hash.erase(itd);
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::add_point_to_last_cluster(Clusters & clusters, const PointXYZIR & pt)
{
  // If there are non-valid points in the container due to rejecting small clusters,
  // overwrite the non-valid points, otherwise emplace new point
  if (clusters.cluster_boundary.back() < clusters.points.size()) {
    clusters.points[clusters.cluster_boundary.back()] =
      static_cast<autoware_auto_msgs::msg::PointXYZIF>(pt);
  } else {
    clusters.points.emplace_back(static_cast<autoware_auto_msgs::msg::PointXYZIF>(pt));
  }
  clusters.cluster_boundary.back() = clusters.cluster_boundary.back() + 1U;
}
////////////////////////////////////////////////////////////////////////////////
EuclideanCluster::PointXY EuclideanCluster::get_point_from_last_cluster(
  const Clusters & clusters,
  const std::size_t cls_pt_idx)
{
  const std::size_t num_of_clusters = clusters.cluster_boundary.size();
  // num_of_clusters will be at least 1
  const auto points_idx = (num_of_clusters < 2U) ?
    cls_pt_idx : (cls_pt_idx + clusters.cluster_boundary[num_of_clusters - 2U]);
  return PointXY{clusters.points[points_idx].x, clusters.points[points_idx].y};
}
////////////////////////////////////////////////////////////////////////////////
std::size_t EuclideanCluster::last_cluster_size(const Clusters & clusters)
{
  const std::size_t num_of_clusters = clusters.cluster_boundary.size();
  if (num_of_clusters < 2U) {
    return clusters.cluster_boundary.front();
  } else {
    const auto cluster_size =
      clusters.cluster_boundary[num_of_clusters - 1U] -
      clusters.cluster_boundary[num_of_clusters - 2U];
    return cluster_size;
  }
}
////////////////////////////////////////////////////////////////////////////////
namespace details
{
BoundingBoxArray compute_bounding_boxes(
  Clusters & clusters, const BboxMethod method,
  const bool compute_height)
{
  BoundingBoxArray boxes;
  for (uint32_t cls_id = 0U; cls_id < clusters.cluster_boundary.size(); cls_id++) {
    try {
      const auto iter_pair = common::lidar_utils::get_cluster(clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }

      switch (method) {
        case BboxMethod::Eigenbox: boxes.boxes.push_back(
            common::geometry::bounding_box::eigenbox_2d(
              iter_pair.first,
              iter_pair.second));
          break;
        case BboxMethod::LFit:     boxes.boxes.push_back(
            common::geometry::bounding_box::lfit_bounding_box_2d(
              iter_pair.first,
              iter_pair.second));
          break;
      }

      if (compute_height) {
        common::geometry::bounding_box::compute_height(
          iter_pair.first, iter_pair.second, boxes.boxes.back());
      }
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }
  return boxes;
}
////////////////////////////////////////////////////////////////////////////////
BoundingBoxArray compute_lfit_bounding_boxes(Clusters & clusters, const bool compute_height)
{
  BoundingBoxArray boxes;
  for (uint32_t cls_id = 0U; cls_id < clusters.cluster_boundary.size(); cls_id++) {
    try {
      const auto iter_pair = common::lidar_utils::get_cluster(clusters, cls_id);
      if (iter_pair.first == iter_pair.second) {
        continue;
      }
      boxes.boxes.push_back(
        common::geometry::bounding_box::lfit_bounding_box_2d(iter_pair.first, iter_pair.second));
      if (compute_height) {
        common::geometry::bounding_box::compute_height(
          iter_pair.first, iter_pair.second, boxes.boxes.back());
      }
    } catch (const std::exception & e) {
      std::cerr << e.what() << std::endl;
    }
  }
  return boxes;
}
////////////////////////////////////////////////////////////////////////////////
DetectedObjects convert_to_detected_objects(const BoundingBoxArray & boxes)
{
  DetectedObjects detected_objects;
  detected_objects.objects.reserve(boxes.boxes.size());
  detected_objects.header = boxes.header;
  std::transform(
    boxes.boxes.begin(), boxes.boxes.end(), std::back_inserter(detected_objects.objects),
    common::geometry::bounding_box::details::make_detected_object);
  return detected_objects;
}

////////////////////////////////////////////////////////////////////////////////
}  // namespace details
}  // namespace euclidean_cluster
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware
