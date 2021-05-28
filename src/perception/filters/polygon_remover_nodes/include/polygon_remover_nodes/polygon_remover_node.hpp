// Copyright 2021 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the polygon_remover_nodes_node class.

#ifndef POLYGON_REMOVER_NODES__POLYGON_REMOVER_NODE_HPP_
#define POLYGON_REMOVER_NODES__POLYGON_REMOVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <polygon_remover/polygon_remover.hpp>
#include <memory>
#include <map>
#include <string>
#include "polygon_remover_nodes/visibility_control.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace polygon_remover_nodes
{

/// \class PolygonRemoverNodesNode
/// \brief Is used for removing a polygon from point clouds.
class POLYGON_REMOVER_NODES_PUBLIC PolygonRemoverNode : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Polygon = geometry_msgs::msg::Polygon;
  using bool8_t = autoware::common::types::bool8_t;

  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit PolygonRemoverNode(const rclcpp::NodeOptions & options);

  void callback_cloud(const PointCloud2::ConstSharedPtr cloud_in_ptr);

  void callback_polygon(const Polygon::ConstSharedPtr polygon_in_ptr);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_ptr_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_ptr_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_ptr_;
  rclcpp::Subscription<Polygon>::SharedPtr sub_polygon_ptr_;

  polygon_remover::PolygonRemover::SharedPtr polygon_remover_;

  bool8_t will_visualize_;

  enum class WorkingMode
  {
    kStatic,
    kPolygonSub
  };
};
}  // namespace polygon_remover_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // POLYGON_REMOVER_NODES__POLYGON_REMOVER_NODE_HPP_
