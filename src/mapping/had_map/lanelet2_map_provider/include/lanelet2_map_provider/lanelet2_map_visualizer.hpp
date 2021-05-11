// Copyright 2020 The Autoware Foundation
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

/// \brief This file defines the lanelet2_map_provider_node class.

#ifndef LANELET2_MAP_PROVIDER__LANELET2_MAP_VISUALIZER_HPP_
#define LANELET2_MAP_PROVIDER__LANELET2_MAP_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <common/types.hpp>

#include <string>
#include <memory>

#include "lanelet2_map_provider/visibility_control.hpp"

namespace autoware
{
namespace lanelet2_map_provider
{

/// \class Lanelet2MapVisualizaer
/// \brief ROS 2 Node for visualization of lanelet2 semantic map.

class LANELET2_MAP_PROVIDER_PUBLIC Lanelet2MapVisualizer : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  explicit Lanelet2MapVisualizer(const rclcpp::NodeOptions & options);

private:
  rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedPtr m_client;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_viz_pub;

  void visualize_map_callback(
    rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedFuture response);
};

}  // namespace lanelet2_map_provider

}  // namespace autoware

#endif  // LANELET2_MAP_PROVIDER__LANELET2_MAP_VISUALIZER_HPP_
