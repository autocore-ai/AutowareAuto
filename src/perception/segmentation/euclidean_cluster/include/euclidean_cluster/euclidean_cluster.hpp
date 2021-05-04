// Copyright 2019-2020 the Autoware Foundation
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
/// \file
/// \brief This file defines the euclidean cluster algorithm for object detection

#ifndef EUCLIDEAN_CLUSTER__EUCLIDEAN_CLUSTER_HPP_
#define EUCLIDEAN_CLUSTER__EUCLIDEAN_CLUSTER_HPP_

#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_msgs/msg/point_clusters.hpp>
#include <geometry/spatial_hash.hpp>
#include <euclidean_cluster/visibility_control.hpp>
#include <common/types.hpp>
#include <string>
#include <vector>
#include <utility>

namespace autoware
{
namespace perception
{
namespace segmentation
{
/// \brief Supporting classes for euclidean clustering, an object detection algorithm
namespace euclidean_cluster
{
using autoware::common::types::float32_t;
using autoware::common::types::bool8_t;
/// \brief Simple point struct for memory mapping to and from PointCloud2 type
struct PointXYZI
{
  float32_t x = 0.0f;
  float32_t y = 0.0f;
  float32_t z = 0.0f;
  float32_t intensity = 0.0f;
};  // struct PointXYZI

/// \brief Helper point for which euclidean distance is computed only once
class EUCLIDEAN_CLUSTER_PUBLIC PointXYZIR
{
public:
  PointXYZIR() = default;
  /// \brief Conversion constructor
  /// \param[in] pt The point to convert
  explicit PointXYZIR(const common::types::PointXYZIF & pt);
  /// \brief Conversion constructor
  /// \param[in] pt The point to convert
  explicit PointXYZIR(const PointXYZI & pt);
  /// \brief Constructor
  /// \param[in] x The x position of the point
  /// \param[in] y The y position of the point
  /// \param[in] z The z position of the point
  /// \param[in] intensity The intensity value of the point
  PointXYZIR(const float32_t x, const float32_t y, const float32_t z, const float32_t intensity);
  /// \brief Getter for radius
  /// \return The projected radial distance
  float32_t get_r() const;
  /// \brief Get core point
  /// \return Reference to internally stored point
  const PointXYZI & get_point() const;
  /// \brief Explicit conversion operator from PointXYZIR to msg type PointXYZIF
  /// \return a PointXYZIF type
  explicit operator autoware_auto_msgs::msg::PointXYZIF() const;

private:
  // This could instead be a pointer; I'm pretty sure ownership would work out, but I'm
  // uncomfortable doing it that way (12 vs 20 bytes)
  PointXYZI m_point;
  float32_t m_r_xy;
};  // class PointXYZIR

using HashConfig = autoware::common::geometry::spatial_hash::Config2d;
using Hash = autoware::common::geometry::spatial_hash::SpatialHash2d<PointXYZIR>;
using Clusters = autoware_auto_msgs::msg::PointClusters;

/// \brief Configuration class for euclidean cluster
/// In the future this can become a base class with subclasses defining different
/// threshold functions. This configuration's threshold function currently assumes isotropy, and
/// minor details in the clustering implementation also assume this property.
class EUCLIDEAN_CLUSTER_PUBLIC Config
{
public:
  /// \brief Constructor
  /// \param[in] frame_id The frame id for which all clusters are initialized with
  /// \param[in] min_cluster_size The number of points that must be in a cluster before it is not
  ///                             considered noise
  /// \param[in] max_num_clusters The maximum preallocated number of clusters in a scene
  /// \param[in] min_cluster_threshold_m The minimum connectivity threshold when r = 0
  /// \param[in] max_cluster_threshold_m The maximum connectivity threshold when
  ///                                    r = cluster_threshold_saturation_distance
  /// \param[in] cluster_threshold_saturation_distance_m The distance at which the cluster threshold
  ///                                                    is clamped to the maximum value
  Config(
    const std::string & frame_id,
    const std::size_t min_cluster_size,
    const std::size_t max_num_clusters,
    const float32_t min_cluster_threshold_m,
    const float32_t max_cluster_threshold_m,
    const float32_t cluster_threshold_saturation_distance_m);
  /// \brief Gets minimum number of points needed for a cluster to not be considered noise
  /// \return Minimum cluster size
  std::size_t min_cluster_size() const;
  /// \brief Gets maximum preallocated number of clusters
  /// \return Maximum number of clusters
  std::size_t max_num_clusters() const;
  /// \brief Compute the connectivity threshold for a given point
  /// \param[in] pt The point whose connectivity criterion will be calculated
  /// \return The connectivity threshold, in meters
  float32_t threshold(const PointXYZIR & pt) const;
  /// \brief Compute the connectivity threshold for a given point
  /// \param[in] r The projected radial distance of the point
  /// \return The connectivity threshold, in meters
  float32_t threshold(const float32_t r) const;
  /// \brief Get frame id
  /// \return The frame id
  const std::string & frame_id() const;
  /// \brief Check the external clusters size with the EuclideanClusters configuration
  /// \param[in] clusters The clusters object
  /// \return True if config is valid
  bool match_clusters_size(const Clusters & clusters) const;

private:
  const std::string m_frame_id;
  const std::size_t m_min_cluster_size;
  const std::size_t m_max_num_clusters;
  const float32_t m_min_thresh_m;
  const float32_t m_max_distance_m;
  const float32_t m_thresh_rate;
};  // class Config

/// \brief implementation of euclidean clustering for point cloud segmentation
/// This clas implicitly projects points onto a 2D (x-y) plane, and segments
/// according to euclidean distance. This can be thought of as a graph-based
/// approach where points are vertices and edges are defined by euclidean distance
/// The input to this should be nonground points pased through a voxel grid.
class EUCLIDEAN_CLUSTER_PUBLIC EuclideanCluster
{
public:
  enum class Error : uint8_t
  {
    NONE = 0U,
    TOO_MANY_CLUSTERS
  };  // enum class Error
  /// \brief Constructor
  /// \param[in] cfg The configuration of the clustering algorithm, contains threshold function
  /// \param[in] hash_cfg The configuration of the underlying spatial hash, controls the maximum
  ///                     number of points in a scene
  EuclideanCluster(const Config & cfg, const HashConfig & hash_cfg);
  /// \brief Insert an individual point
  /// \param[in] pt The point to insert
  /// \throw std::length_error If the underlying spatial hash is full
  void insert(const PointXYZIR & pt);
  /// \brief Multi-insert
  /// \param[in] begin Iterator pointing to to the first point to insert
  /// \param[in] end Iterator pointing to one past the last point to insert
  /// \throw std::length_error If the underlying spatial hash is full
  /// \tparam IT The type of the iterator
  template<typename IT>
  void insert(const IT begin, const IT end)
  {
    if ((static_cast<std::size_t>(std::distance(begin, end)) + m_hash.size()) > m_hash.capacity()) {
      throw std::length_error{"EuclideanCluster: Multi insert would overrun capacity"};
    }
    for (auto it = begin; it != end; ++it) {
      insert(PointXYZIR{*it});
    }
  }

  /// \brief Compute the clusters from the inserted points, where the final clusters object lives in
  ///        another scope.
  /// \param[inout] clusters The clusters object
  void cluster(Clusters & clusters);

  /// \brief Gets last error, intended to be used with clustering with internal cluster result
  /// This is a separate function rather than using an exception because the main error mode is
  /// exceeding preallocated cluster capacity. However, throwing an exception would throw away
  /// perfectly valid information that is still usable in an error state.
  Error get_error() const;

  /// \brief Gets the internal configuration class, for use when it was inline generated
  /// \return Internal configuration class
  const Config & get_config() const;

  /// \brief Throw the stored error during clustering process
  /// \throw std::runtime_error If the maximum number of clusters may have been exceeded
  void throw_stored_error() const;

private:
  /// \brief Internal struct instead of pair since I can guarantee some memory stuff
  struct PointXY
  {
    float32_t x = 0.0f;
    float32_t y = 0.0f;
  };  // struct PointXYZ
  /// \brief Do the clustering process, with no error checking
  EUCLIDEAN_CLUSTER_LOCAL void cluster_impl(Clusters & clusters);
  /// \brief Compute the next cluster, seeded by the given point, and grown using the remaining
  ///         points still contained in the hash
  EUCLIDEAN_CLUSTER_LOCAL void cluster(Clusters & clusters, const Hash::IT it);
  /// \brief Add all near neighbors of a point to a given cluster
  EUCLIDEAN_CLUSTER_LOCAL void add_neighbors_to_last_cluster(
    Clusters & clusters, const PointXY pt);
  /// \brief Adds a point to the last cluster, internal version since no error checking is needed
  EUCLIDEAN_CLUSTER_LOCAL static void add_point_to_last_cluster(
    Clusters & clusters, const PointXYZIR & pt);
  /// \brief Get a specified point from the cluster
  EUCLIDEAN_CLUSTER_LOCAL static PointXY get_point_from_last_cluster(
    const Clusters & clusters, const std::size_t cls_pt_idx);
  EUCLIDEAN_CLUSTER_LOCAL static std::size_t last_cluster_size(const Clusters & clusters);

  const Config m_config;
  Hash m_hash;
  Error m_last_error;
  std::vector<bool8_t> m_seen;
};  // class EuclideanCluster

/// \brief Common euclidean cluster functions not intended for external use
namespace details
{
using BoundingBox = autoware_auto_msgs::msg::BoundingBox;
using BoundingBoxArray = autoware_auto_msgs::msg::BoundingBoxArray;
///// \brief Compute lfit bounding boxes from clusters
///// \param[out] boxes Message that gets filled with the resulting bounding boxes
///// \param[inout] clusters A set of clusters for which to compute the bounding boxes. Individual
/////                        clusters get their points shuffled
EUCLIDEAN_CLUSTER_PUBLIC
void compute_lfit_bounding_boxes(Clusters & clusters, BoundingBoxArray & boxes);
/// \brief Compute lfit bounding boxes from clusters, including z coordinate
/// \param[out] boxes Message that gets filled with the resulting bounding boxes
/// \param[inout] clusters A set of clusters for which to compute the bounding boxes. Individual
///                        clusters get their points shuffled
EUCLIDEAN_CLUSTER_PUBLIC
void compute_lfit_bounding_boxes_with_z(Clusters & clusters, BoundingBoxArray & boxes);
/// \brief Compute eigenboxes from clusters
/// \param[out] boxes Message that gets filled with the resulting bounding boxes
/// \param[in] clusters A set of clusters for which to compute the bounding boxes
EUCLIDEAN_CLUSTER_PUBLIC
void compute_eigenboxes(const Clusters & clusters, BoundingBoxArray & boxes);
/// \brief Compute eigenboxes from clusters, including z coordinate
/// \param[out] boxes Message that gets filled with the resulting bounding boxes
/// \param[in] clusters A set of clusters for which to compute the bounding boxes
EUCLIDEAN_CLUSTER_PUBLIC
void compute_eigenboxes_with_z(const Clusters & clusters, BoundingBoxArray & boxes);
}  // namespace details
}  // namespace euclidean_cluster
}  // namespace segmentation
}  // namespace perception
namespace common
{
namespace geometry
{
namespace point_adapter
{
template<>
inline EUCLIDEAN_CLUSTER_PUBLIC auto x_(
  const perception::segmentation::euclidean_cluster::PointXYZIR & pt)
{
  return pt.get_point().x;
}
template<>
inline EUCLIDEAN_CLUSTER_PUBLIC auto y_(
  const perception::segmentation::euclidean_cluster::PointXYZIR & pt)
{
  return pt.get_point().y;
}
template<>
inline EUCLIDEAN_CLUSTER_PUBLIC auto z_(
  const perception::segmentation::euclidean_cluster::PointXYZIR & pt)
{
  return pt.get_point().z;
}
}  // namespace point_adapter
}  // namespace geometry
}  // namespace common
}  // namespace autoware
#endif  // EUCLIDEAN_CLUSTER__EUCLIDEAN_CLUSTER_HPP_
