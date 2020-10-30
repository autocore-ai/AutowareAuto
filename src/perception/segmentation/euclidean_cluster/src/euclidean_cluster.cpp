// Copyright 2019 the Autoware Foundation
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

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace euclidean_cluster
{
////////////////////////////////////////////////////////////////////////////////
PointXYZII::PointXYZII(const PointXYZI & pt, const uint32_t id)
: m_point{pt},
  m_id{id}
{
}
////////////////////////////////////////////////////////////////////////////////
PointXYZII::PointXYZII(
  const float32_t x,
  const float32_t y,
  const float32_t z,
  const float32_t intensity,
  const uint32_t id)
: m_point{x, y, z, intensity},
  m_id{id}
{
}
////////////////////////////////////////////////////////////////////////////////
uint32_t PointXYZII::get_id() const
{
  return m_id;
}
////////////////////////////////////////////////////////////////////////////////
const PointXYZI & PointXYZII::get_point() const
{
  return m_point;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
Config::Config(
  const std::string & frame_id,
  const std::size_t min_cluster_size,
  const std::size_t max_num_clusters)
: m_frame_id(frame_id),
  m_min_cluster_size(min_cluster_size),
  m_max_num_clusters(max_num_clusters)
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
const std::string & Config::frame_id() const
{
  return m_frame_id;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
EuclideanCluster::EuclideanCluster(const Config & cfg, const HashConfig & hash_cfg)
: m_config(cfg),
  m_hash(hash_cfg),
  m_clusters(),
  m_cluster_pool(),
  m_last_error(Error::NONE),
  m_seen{}
{
  // Reservation
  m_clusters.clusters.reserve(m_config.max_num_clusters());
  m_cluster_pool.resize(m_config.max_num_clusters());
  m_seen.reserve(hash_cfg.get_capacity());
  // initialize clusters
  for (auto & cls : m_cluster_pool) {
    common::lidar_utils::init_pcl_msg(cls, cfg.frame_id(), hash_cfg.get_capacity());
    cls.width = 0U;
    // check pointstep vs sizeof(PointXY) and sizeof(PointXYZIF)
    if (cls.point_step < sizeof(PointXY)) {
      throw std::domain_error{"Cluster initialized with point size smaller than PointXY"};
    }
    if (cls.point_step != sizeof(PointXYZI)) {
      throw std::domain_error{"Cluster initialized with point size != PointXYZI"};
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cluster(Clusters & clusters)
{
  if (clusters.clusters.capacity() < m_config.max_num_clusters()) {
    throw std::domain_error{"EuclideanCluster: Provided clusters must have sufficient capacity"};
  }
  cluster_impl(clusters);
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::return_clusters(Clusters & clusters)
{
  for (std::size_t idx = 0U; idx < clusters.clusters.size(); ++idx) {
    m_cluster_pool[idx] = std::move(clusters.clusters[idx]);
    m_cluster_pool[idx].width = 0U;
  }
  clusters.clusters.resize(0U);
  m_seen.clear();
}
////////////////////////////////////////////////////////////////////////////////
const Clusters & EuclideanCluster::cluster(const builtin_interfaces::msg::Time stamp)
{
  // Reset clusters to pool
  return_clusters(m_clusters);
  // Actual clustering process
  cluster_impl(m_clusters);
  // Assign time stamp
  for (auto & cls : m_clusters.clusters) {
    cls.header.stamp = stamp;
  }
  return m_clusters;
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cleanup(Clusters & clusters)
{
  // Return data to algorithm
  return_clusters(clusters);
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
  for (const auto & kv : m_hash) {
    const auto & pt = kv.second;
    if (!m_seen[pt.get_id()]) {
      cluster(clusters, pt);
    }
  }
  m_hash.clear();
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::cluster(Clusters & clusters, const PointXYZII & pt)
{
  // init new cluster
  const auto num_clusters = clusters.clusters.size();
  if (num_clusters >= m_config.max_num_clusters()) {
    m_last_error = Error::TOO_MANY_CLUSTERS;
  } else {
    clusters.clusters.emplace_back(std::move(m_cluster_pool[num_clusters]));
    // Seed cluster with new point
    auto & cluster = clusters.clusters.back();
    add_point(cluster, pt);
    m_seen[pt.get_id()] = true;
    // Start clustering process
    std::size_t last_seed_idx = 0U;
    while (last_seed_idx < cluster.width) {
      const auto pt = get_point(cluster, last_seed_idx);
      add_neighbors(cluster, pt);
      // Increment seed point
      ++last_seed_idx;
    }
    // check if cluster is large enough: roll back pointer if so
    if (last_seed_idx < m_config.min_cluster_size()) {
      // return cluster to pool
      m_cluster_pool[num_clusters] = std::move(clusters.clusters[num_clusters]);
      m_cluster_pool[num_clusters].width = 0U;
      clusters.clusters.resize(num_clusters);
    } else {
      // finalize cluster
      cluster.row_step = cluster.point_step * cluster.width;
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::add_neighbors(Cluster & cls, const EuclideanCluster::PointXY pt)
{
  // z is not needed since it's a 2d hash
  const auto & nbrs = m_hash.near(pt.x, pt.y);
  // For each point within a fixed radius, check if already seen
  for (const auto itd : nbrs) {
    const auto & qt = itd.get_point();
    const auto id = qt.get_id();
    if (!m_seen[id]) {
      // Add to cluster
      add_point(cls, qt);
      // Mark point as seen
      m_seen[id] = true;
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void EuclideanCluster::add_point(Cluster & cls, const PointXYZII & pt)
{
  // Clustering cannot overrun cluster capacity since each cluster is preallocated with the
  // max capacity of the hash, so the data structure would throw before you overrun the cluster
  using Size = decltype(Cluster::data)::size_type;
  const auto idx = static_cast<Size>(cls.width) * static_cast<Size>(cls.point_step);
  cls.data.resize(idx + static_cast<Size>(cls.point_step));
  // Placement new to ensure dynamic type is accurate (allowing for reinterpret_cast to not be UB)
  (void)new(&cls.data[idx]) PointXYZI(pt.get_point());
  ++cls.width;
}
////////////////////////////////////////////////////////////////////////////////
EuclideanCluster::PointXY EuclideanCluster::get_point(const Cluster & cls, const std::size_t idx)
{
  PointXY ret{};
  //lint -e{586, 925} NOLINT guaranteed not to have overlap, so it's fine; no other way to do
  (void)memcpy(
    static_cast<void *>(&ret),
    static_cast<const void *>(&cls.data[idx * cls.point_step]),
    sizeof(ret));
  return ret;
}
}  // namespace euclidean_cluster
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware
