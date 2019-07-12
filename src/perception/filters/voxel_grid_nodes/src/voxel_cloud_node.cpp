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


//lint -e537 NOLINT cpplint wants this due to std::make_unique
#include <rclcpp/node_options.hpp>
#include <voxel_grid_nodes/voxel_cloud_node.hpp>
#include <voxel_grid_nodes/algorithm/voxel_cloud_approximate.hpp>
#include <voxel_grid_nodes/algorithm/voxel_cloud_centroid.hpp>
#include <memory>
#include <string>

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
    node_name.c_str(),
    node_namespace.c_str(),
    rclcpp::NodeOptions()),
  // TODO(esteve): Pass empty NodeOptions as workaround for https://github.com/ros2/rclcpp/pull/775
  m_sub_ptr(create_subscription<Message>(declare_parameter("input_topic").get<std::string>(),
    rclcpp::QoS(10), std::bind(&VoxelCloudNode::callback, this, std::placeholders::_1))),
  m_pub_ptr(create_publisher<Message>(declare_parameter("downsample_topic").get<std::string>(),
    rclcpp::QoS(10))),
  m_has_failed(false)
{
  // Build config manually (messages only have default constructors)
  voxel_grid::PointXYZ min_point;
  min_point.x = static_cast<float>(declare_parameter("config.min_point.x").get<float>());
  min_point.y = static_cast<float>(declare_parameter("config.min_point.y").get<float>());
  min_point.z = static_cast<float>(declare_parameter("config.min_point.z").get<float>());
  voxel_grid::PointXYZ max_point;
  max_point.x = static_cast<float>(declare_parameter("config.max_point.x").get<float>());
  max_point.y = static_cast<float>(declare_parameter("config.max_point.y").get<float>());
  max_point.z = static_cast<float>(declare_parameter("config.max_point.z").get<float>());
  voxel_grid::PointXYZ voxel_size;
  voxel_size.x = static_cast<float>(declare_parameter("config.voxel_size.x").get<float>());
  voxel_size.y = static_cast<float>(declare_parameter("config.voxel_size.y").get<float>());
  voxel_size.z = static_cast<float>(declare_parameter("config.voxel_size.z").get<float>());
  const std::size_t capacity =
    static_cast<std::size_t>(declare_parameter("config.capacity").get<std::size_t>());
  const voxel_grid::Config cfg{min_point, max_point, voxel_size, capacity};
  // Init
  init(cfg, declare_parameter("is_approximate").get<bool>());
}
////////////////////////////////////////////////////////////////////////////////
VoxelCloudNode::VoxelCloudNode(
  const std::string & node_name,
  const std::string & sub_topic,
  const std::string & pub_topic,
  const voxel_grid::Config & cfg,
  const bool is_approximate)
: LifecycleNode(node_name.c_str()),
  m_sub_ptr(create_subscription<Message>(sub_topic.c_str(),
    rclcpp::QoS(10), std::bind(&VoxelCloudNode::callback, this, std::placeholders::_1))),
  m_pub_ptr(create_publisher<Message>(pub_topic.c_str(), rclcpp::QoS(10))),
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
void VoxelCloudNode::init(const voxel_grid::Config & cfg, const bool is_approximate)
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
}  // namespace voxel_grid_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware
