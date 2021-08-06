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

#include "outlier_filter_nodes/voxel_grid_outlier_filter_node.hpp"

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
using float32_t = autoware::common::types::float32_t;

using VoxelGridOutlierFilter =
  autoware::perception::filters::outlier_filter::voxel_grid_outlier_filter::
  VoxelGridOutlierFilter;

VoxelGridOutlierFilterNode::VoxelGridOutlierFilterNode(const rclcpp::NodeOptions & options)
: FilterNodeBase("voxel_grid_outlier_filter_node", options),
  voxel_size_x_(declare_parameter("voxel_size_x").get<float64_t>()),
  voxel_size_y_(declare_parameter("voxel_size_y").get<float64_t>()),
  voxel_size_z_(declare_parameter("voxel_size_z").get<float64_t>()),
  voxel_points_threshold_(declare_parameter("voxel_points_threshold").get<uint32_t>())
{
  voxel_grid_outlier_filter_ = std::make_shared<VoxelGridOutlierFilter>(
    voxel_size_x_,
    voxel_size_y_,
    voxel_size_z_,
    voxel_points_threshold_);

  set_param_callback();
}

void VoxelGridOutlierFilterNode::filter(
  const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_input;
  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  pcl::fromROSMsg(input, pcl_input);

  // Perform filtering
  voxel_grid_outlier_filter_->filter(pcl_input, pcl_output);

  pcl::toROSMsg(pcl_output, output);
}

rcl_interfaces::msg::SetParametersResult VoxelGridOutlierFilterNode::get_node_parameters(
  const std::vector<rclcpp::Parameter> & p)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  {
    using namespace autoware::perception::filters::filter_node_base; //NOLINT

    if (get_param<float64_t>(p, "voxel_size_x", voxel_size_x_)) {
      result.successful = false;
      result.reason += "Failed to retrieve voxel_size_x parameter. ";
      RCLCPP_DEBUG(get_logger(), "Setting new voxel leaf size (x) to: %f.", voxel_size_x_);
    }
    if (get_param<float64_t>(p, "voxel_size_y", voxel_size_y_)) {
      result.successful = false;
      result.reason += "Failed to retrieve voxel_size_y parameter. ";
      RCLCPP_DEBUG(get_logger(), "Setting new voxel leaf size (y) to: %f.", voxel_size_y_);
    }
    if (get_param<float64_t>(p, "voxel_size_z", voxel_size_z_)) {
      result.successful = false;
      result.reason += "Failed to retrieve voxel_size_z parameter. ";
      RCLCPP_DEBUG(get_logger(), "Setting new voxel leaf size (z) to: %f.", voxel_size_z_);
    }
    if (get_param<std::int64_t>(p, "voxel_points_threshold", voxel_points_threshold_)) {
      result.successful = false;
      result.reason += "Failed to retrieve voxel_points_threshold parameter. ";
      RCLCPP_DEBUG(
        get_logger(), "Setting new voxel points threshold to: %d.", voxel_points_threshold_);
    }
  }

  // Call update method in filter class object
  voxel_grid_outlier_filter_->update_parameters(
    static_cast<float32_t>(voxel_size_x_), static_cast<float32_t>(voxel_size_y_),
    static_cast<float32_t>(voxel_size_z_),
    static_cast<std::uint32_t>(voxel_points_threshold_));

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
  autoware::perception::filters::outlier_filter_nodes::VoxelGridOutlierFilterNode)
