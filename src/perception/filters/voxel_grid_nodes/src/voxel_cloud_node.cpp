// Copyright 2019 Apex.AI, Inc.
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


//lint -e537 NOLINT cpplint wants this due to std::make_unique
#include <rclcpp/node_options.hpp>
#include <voxel_grid_nodes/voxel_cloud_node.hpp>
#include <voxel_grid_nodes/algorithm/voxel_cloud_approximate.hpp>
#include <voxel_grid_nodes/algorithm/voxel_cloud_centroid.hpp>
#include <common/types.hpp>
#include <memory>
#include <string>
#include <algorithm>

using autoware::common::types::bool8_t;
using autoware::common::types::uchar8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace voxel_grid_nodes
{
////////////////////////////////////////////////////////////////////////////////
VoxelCloudNode::VoxelCloudNode(
  const std::string & node_name,
  const std::string & node_namespace)
: LifecycleNode(
    node_name.c_str(), node_namespace.c_str()),
  m_sub_ptr(create_subscription<Message>("points_in",
    parse_qos(declare_parameter("subscription.qos.durability"),
    declare_parameter("subscription.qos.history_depth")),
    std::bind(&VoxelCloudNode::callback, this, std::placeholders::_1))),
  m_pub_ptr(create_publisher<Message>("points_downsampled",
    parse_qos(declare_parameter("publisher.qos.durability"),
    declare_parameter("publisher.qos.history_depth")))),
  m_has_failed(false)
{
  // Build config manually (messages only have default constructors)
  voxel_grid::PointXYZ min_point;
  min_point.x = static_cast<float32_t>(declare_parameter("config.min_point.x").get<float32_t>());
  min_point.y = static_cast<float32_t>(declare_parameter("config.min_point.y").get<float32_t>());
  min_point.z = static_cast<float32_t>(declare_parameter("config.min_point.z").get<float32_t>());
  voxel_grid::PointXYZ max_point;
  max_point.x = static_cast<float32_t>(declare_parameter("config.max_point.x").get<float32_t>());
  max_point.y = static_cast<float32_t>(declare_parameter("config.max_point.y").get<float32_t>());
  max_point.z = static_cast<float32_t>(declare_parameter("config.max_point.z").get<float32_t>());
  voxel_grid::PointXYZ voxel_size;
  voxel_size.x = static_cast<float32_t>(declare_parameter("config.voxel_size.x").get<float32_t>());
  voxel_size.y = static_cast<float32_t>(declare_parameter("config.voxel_size.y").get<float32_t>());
  voxel_size.z = static_cast<float32_t>(declare_parameter("config.voxel_size.z").get<float32_t>());
  const std::size_t capacity =
    static_cast<std::size_t>(declare_parameter("config.capacity").get<std::size_t>());
  const voxel_grid::Config cfg{min_point, max_point, voxel_size, capacity};
  // Init
  init(cfg, declare_parameter("is_approximate").get<bool8_t>());
}
////////////////////////////////////////////////////////////////////////////////
VoxelCloudNode::VoxelCloudNode(
  const std::string & node_name,
  const std::string & sub_topic,
  const std::string & pub_topic,
  const voxel_grid::Config & cfg,
  const bool8_t is_approximate,
  const rclcpp::QoS sub_qos,
  const rclcpp::QoS pub_qos)
: LifecycleNode(node_name.c_str()),
  m_sub_ptr(create_subscription<Message>(sub_topic.c_str(),
    sub_qos, std::bind(&VoxelCloudNode::callback, this, std::placeholders::_1))),
  m_pub_ptr(create_publisher<Message>(pub_topic.c_str(), pub_qos)),
  m_has_failed(false)
{
  init(cfg, is_approximate);
}

////////////////////////////////////////////////////////////////////////////////
void VoxelCloudNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  try {
    m_voxelgrid_ptr->insert(*msg);
    m_pub_ptr->publish(m_voxelgrid_ptr->get());
  } catch (const std::exception & e) {
    std::string err_msg{get_name()};
    err_msg += ": " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    m_has_failed = true;
  } catch (...) {
    std::string err_msg{"Unknown error occurred in "};
    err_msg += get_name();
    RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    throw;
  }
}
////////////////////////////////////////////////////////////////////////////////
void VoxelCloudNode::init(const voxel_grid::Config & cfg, const bool8_t is_approximate)
{
  // construct voxel grid
  if (is_approximate) {
    m_voxelgrid_ptr = std::make_unique<algorithm::VoxelCloudApproximate>(cfg);
  } else {
    m_voxelgrid_ptr = std::make_unique<algorithm::VoxelCloudCentroid>(cfg);
  }
  // register callbacks
  using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
  // Activation
  const auto activate_fn = [this](const rclcpp_lifecycle::State &)
    {
      std::string status_msg{get_name()};
      status_msg += " has activated";
      RCLCPP_INFO(this->get_logger(), status_msg.c_str());
      this->m_pub_ptr->on_activate();
      return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    };
  if (!register_on_activate(activate_fn)) {
    throw std::runtime_error("Could not register activate callback");
  }
  // Deactivation
  const auto deactivate_fn = [this](const rclcpp_lifecycle::State &)
    {
      std::string status_msg{get_name()};
      status_msg += " has deactivated";
      RCLCPP_INFO(this->get_logger(), status_msg.c_str());
      this->m_pub_ptr->on_deactivate();
      return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    };
  if (!register_on_deactivate(deactivate_fn)) {
    throw std::runtime_error("Could not register deactivate callback");
  }
}

/////////////////////////////////////////////////////////////////////////////
rclcpp::QoS parse_qos(
  const rclcpp::ParameterValue & durability_param,
  const rclcpp::ParameterValue & depth_param, const rclcpp::QoS & default_qos)
{
  if ((durability_param.get_type() == rclcpp::PARAMETER_NOT_SET) ||
    (depth_param.get_type() == rclcpp::PARAMETER_NOT_SET))
  {
    RCLCPP_DEBUG(rclcpp::get_logger("qos_parse_logger"),
      "Cannot parse the qos as there are missing fields in the parameter file. "
      "A preset qos profile is used instead.");
    return default_qos;
  }
  int32_t depth = depth_param.get<int32_t>();
  std::string durability = durability_param.get<std::string>();

  std::transform(durability.begin(), durability.end(), durability.begin(),
    [](uchar8_t c) {return std::tolower(c);}
  );
  auto qos = rclcpp::QoS(depth);
  if (durability == "transient_local") {
    qos.transient_local();
  } else if (durability == "volatile") {
    qos.durability_volatile();
  } else {
    throw std::runtime_error("Durability setting '" + durability + "' is not supported."
            "Please try 'volatile' or 'transient_local'.");
  }
  return qos;
}
}  // namespace voxel_grid_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware
