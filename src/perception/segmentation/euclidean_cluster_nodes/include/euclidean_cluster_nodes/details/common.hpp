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

#ifndef EUCLIDEAN_CLUSTER_NODES__DETAILS__COMMON_HPP_
#define EUCLIDEAN_CLUSTER_NODES__DETAILS__COMMON_HPP_
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <euclidean_cluster/euclidean_cluster.hpp>

namespace autoware
{
namespace perception
{
namespace segmentation
{
namespace euclidean_cluster_nodes
{
using Clusters = euclidean_cluster::Clusters;
using BoundingBox = autoware_auto_msgs::msg::BoundingBox;
using BoundingBoxArray = autoware_auto_msgs::msg::BoundingBoxArray;
/// \brief Common euclidean cluster nodes functions not intended for external use
namespace details
{
/// \brief Compute lfit bounding box from individual cluster
/// \param[inout] cls The cluster for which to compute the bounding box, gets shuffled
/// \return Lfit bounding box
EUCLIDEAN_CLUSTER_NODES_PUBLIC
BoundingBox compute_lfit_bounding_box(euclidean_cluster::Cluster & cls);
/// \brief Compute eigenbox from individual cluster
/// \param[in] cls The cluster for which to compute the bounding box
/// \return Best fit eigenbox
EUCLIDEAN_CLUSTER_NODES_PUBLIC BoundingBox compute_eigenbox(const euclidean_cluster::Cluster & cls);
/// \brief Compute lfit bounding boxes from clusters
/// \param[out] boxes Message that gets filled with the resulting bounding boxes
/// \param[inout] clusters A set of clusters for which to compute the bounding boxes. Individual
///                        clusters get their points shuffled
EUCLIDEAN_CLUSTER_NODES_PUBLIC
void compute_lfit_bounding_boxes(Clusters & clusters, BoundingBoxArray & boxes);
/// \brief Compute lfit bounding boxes from clusters, including z coordinate
/// \param[out] boxes Message that gets filled with the resulting bounding boxes
/// \param[inout] clusters A set of clusters for which to compute the bounding boxes. Individual
///                        clusters get their points shuffled
EUCLIDEAN_CLUSTER_NODES_PUBLIC
void compute_lfit_bounding_boxes_with_z(Clusters & clusters, BoundingBoxArray & boxes);
/// \brief Compute eigenboxes from clusters
/// \param[out] boxes Message that gets filled with the resulting bounding boxes
/// \param[in] clusters A set of clusters for which to compute the bounding boxes
EUCLIDEAN_CLUSTER_NODES_PUBLIC
void compute_eigenboxes(const Clusters & clusters, BoundingBoxArray & boxes);
/// \brief Compute eigenboxes from clusters, including z coordinate
/// \param[out] boxes Message that gets filled with the resulting bounding boxes
/// \param[in] clusters A set of clusters for which to compute the bounding boxes
EUCLIDEAN_CLUSTER_NODES_PUBLIC
void compute_eigenboxes_with_z(const Clusters & clusters, BoundingBoxArray & boxes);
}  // namespace details
}  // namespace euclidean_cluster_nodes
}  // namespace segmentation
}  // namespace perception
}  // namespace autoware
#endif  // EUCLIDEAN_CLUSTER_NODES__DETAILS__COMMON_HPP_
