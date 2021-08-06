// Copyright 2021 Tier IV, Inc
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

#include <memory>
#include <vector>

#include "outlier_filter_nodes/radius_search_2d_filter_node.hpp"

#include "pcl_conversions/pcl_conversions.h"


namespace autoware
{
namespace perception
{
namespace filters
{
namespace outlier_filter_nodes
{

using float64_t = autoware::common::types::float64_t;

using RadiusSearch2DFilter =
  autoware::perception::filters::outlier_filter::radius_search_2d_filter::
  RadiusSearch2DFilter;

RadiusSearch2DFilterNode::RadiusSearch2DFilterNode(const rclcpp::NodeOptions & options)
:  FilterNodeBase("radius_search_2d_filter_node", options),
  search_radius_(declare_parameter("search_radius").get<float64_t>()),
  min_neighbors_(declare_parameter("min_neighbors").get<int>())
{
  radius_search_2d_filter_ = std::make_shared<RadiusSearch2DFilter>(
    search_radius_,
    min_neighbors_
  );

  set_param_callback();
}

void RadiusSearch2DFilterNode::filter(
  const sensor_msgs::msg::PointCloud2 & input,
  sensor_msgs::msg::PointCloud2 & output)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_input;
  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  pcl::fromROSMsg(input, pcl_input);

  // Perform filtering
  radius_search_2d_filter_->filter(pcl_input, pcl_output);

  pcl::toROSMsg(pcl_output, output);
}

rcl_interfaces::msg::SetParametersResult RadiusSearch2DFilterNode::get_node_parameters(
  const std::vector<rclcpp::Parameter> & p)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  {
    using namespace autoware::perception::filters::filter_node_base; //NOLINT

    if (get_param<float64_t>(p, "search_radius", search_radius_)) {
      result.successful = false;
      result.reason += "Failed to retrieve search_radius parameter. ";
      RCLCPP_DEBUG(get_logger(), "Setting new search radius to: %f.", search_radius_);
    }
    if (get_param<std::int64_t>(p, "min_neighbors", min_neighbors_)) {
      result.successful = false;
      result.reason += "Failed to retrieve min_neighbors parameter. ";
      RCLCPP_DEBUG(get_logger(), "Setting new min neighbors to: %d.", min_neighbors_);
    }
  }

  // Call update method in filter class object
  radius_search_2d_filter_->update_parameters(search_radius_, static_cast<int>(min_neighbors_));

  return result;
}

}  // namespace outlier_filter_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::filters::outlier_filter_nodes::RadiusSearch2DFilterNode)
