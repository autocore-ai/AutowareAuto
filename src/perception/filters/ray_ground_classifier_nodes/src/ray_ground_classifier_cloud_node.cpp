// Copyright 2018 Apex.AI, Inc.
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
#include <string>

#include "common/types.hpp"
#include "lidar_utils/point_cloud_utils.hpp"
#include "ray_ground_classifier_nodes/ray_ground_classifier_cloud_node.hpp"

namespace autoware
{
namespace perception
{
namespace filters
{
namespace ray_ground_classifier_nodes
{
////////////////////////////////////////////////////////////////////////////////
using autoware::common::types::PointXYZIF;
using autoware::perception::filters::ray_ground_classifier::PointBlock;

using std::placeholders::_1;

using autoware::common::lidar_utils::init_pcl_msg;

// Check the pointcloud msg has x, y, z fields, otherwise throw an exception; check
// the pointcloud msg has intensity field, otherwise return false
bool8_t has_intensity_and_throw_if_no_xyz(
  const PointCloud2::SharedPtr & cloud)
{
  bool8_t ret = true;
  // Validate point step
  if (cloud->fields.size() < 3U) {
    throw std::runtime_error("RayGroundClassifierNode: invalid PointCloud msg");
  }

  const auto check_field = [](
    const sensor_msgs::msg::PointField & field,
    const char8_t * const name,
    const uint32_t offset) -> bool8_t {
      bool8_t res = true;
      if ((name != field.name) || (offset != field.offset) ||
        (sensor_msgs::msg::PointField::FLOAT32 != field.datatype) || (1U != field.count))
      {
        res = false;
      }
      return res;
    };

  if (!check_field(cloud->fields[0U], "x", 0U)) {
    throw std::runtime_error("RayGroundClassifierNode: PointCloud doesn't have correct x field");
  } else if (!check_field(cloud->fields[1U], "y", 4U)) {
    throw std::runtime_error("RayGroundClassifierNode: PointCloud doesn't have correct y field");
  } else if (!check_field(cloud->fields[2U], "z", 8U)) {
    throw std::runtime_error("RayGroundClassifierNode: PointCloud doesn't have correct z field");
  } else {
    // do nothing
  }
  if (cloud->fields.size() >= 4U) {
    if (!check_field(cloud->fields[3U], "intensity", 12U)) {
      ret = false;
    }
  } else {
    ret = false;
  }
  return ret;
}

RayGroundClassifierCloudNode::RayGroundClassifierCloudNode(
  const std::string & node_name,
  const std::string & node_namespace)
: LifecycleNode(
    node_name.c_str(),
    node_namespace.c_str()),
  m_classifier(ray_ground_classifier::Config{
          static_cast<float>(declare_parameter("classifier.sensor_height_m").get<float>()),
          static_cast<float>(declare_parameter("classifier.max_local_slope_deg").get<float>()),
          static_cast<float>(declare_parameter("classifier.max_global_slope_deg").get<float>()),
          static_cast<float>(declare_parameter(
            "classifier.nonground_retro_thresh_deg").get<float>()),
          static_cast<float>(declare_parameter("classifier.min_height_thresh_m").get<float>()),
          static_cast<float>(declare_parameter(
            "classifier.max_global_height_thresh_m").get<float>()),
          static_cast<float>(declare_parameter(
            "classifier.max_last_local_ground_thresh_m").get<float>()),
          static_cast<float>(declare_parameter(
            "classifier.max_provisional_ground_distance_m").get<float>()),
          static_cast<float>(declare_parameter("classifier.min_height_m").get<float>()),
          static_cast<float>(declare_parameter("classifier.max_height_m").get<float>())
        }),
  m_aggregator(ray_ground_classifier::RayAggregator::Config{
          static_cast<float>(declare_parameter("aggregator.min_ray_angle_rad").get<float>()),
          static_cast<float>(declare_parameter("aggregator.max_ray_angle_rad").get<float>()),
          static_cast<float>(declare_parameter("aggregator.ray_width_rad").get<float>()),
          static_cast<std::size_t>(
            declare_parameter("aggregator.max_ray_points").get<std::size_t>())
        }),
  m_pcl_size(static_cast<std::size_t>(declare_parameter("pcl_size").get<std::size_t>())),
  m_frame_id(declare_parameter("frame_id").get<std::string>().c_str()),
  m_has_failed(false),
  m_timeout(std::chrono::milliseconds{declare_parameter("cloud_timeout_ms").get<uint16_t>()}),
  m_raw_sub_ptr(create_subscription<PointCloud2>(declare_parameter("raw_topic").get<std::string>(),
    rclcpp::QoS(10), std::bind(&RayGroundClassifierCloudNode::callback, this, _1))),
  m_ground_pub_ptr(
    create_publisher<PointCloud2>(declare_parameter("ground_topic").get<std::string>(),
    rclcpp::QoS(10))),
  m_nonground_pub_ptr(
    create_publisher<PointCloud2>(declare_parameter("nonground_topic").get<std::string>(),
    rclcpp::QoS(10))),
  m_ground_pc_idx{0},
  m_nonground_pc_idx{0}
{
  register_callbacks_preallocate();
}
////////////////////////////////////////////////////////////////////////////////
RayGroundClassifierCloudNode::RayGroundClassifierCloudNode(
  const std::string & node_name,
  const std::string & raw_topic,
  const std::string & ground_topic,
  const std::string & nonground_topic,
  const std::string & frame_id,
  const std::chrono::nanoseconds & timeout,
  const std::size_t pcl_size,
  const ray_ground_classifier::Config & cfg,
  const ray_ground_classifier::RayAggregator::Config & agg_cfg)
: LifecycleNode(node_name.c_str()),
  m_classifier(cfg),
  m_aggregator(agg_cfg),
  m_pcl_size(pcl_size),
  m_frame_id(frame_id),
  m_has_failed(false),
  m_timeout(timeout),
  m_raw_sub_ptr(create_subscription<PointCloud2>(raw_topic.c_str(),
    rclcpp::QoS(10), std::bind(&RayGroundClassifierCloudNode::callback, this, _1))),
  m_ground_pub_ptr(create_publisher<PointCloud2>(ground_topic.c_str(),
    rclcpp::QoS(10))),
  m_nonground_pub_ptr(create_publisher<PointCloud2>(nonground_topic.c_str(),
    rclcpp::QoS(10))),
  m_ground_pc_idx{0},
  m_nonground_pc_idx{0}
{
  register_callbacks_preallocate();
}
////////////////////////////////////////////////////////////////////////////////
void
RayGroundClassifierCloudNode::callback(const PointCloud2::SharedPtr msg)
{
  PointXYZIF pt_tmp;
  pt_tmp.id = static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID);
  const ray_ground_classifier::PointXYZIFR eos_pt{pt_tmp};

  try {
    // Reset messages and aggregator to ensure they are in a good state
    reset();
    // Verify header
    if (msg->header.frame_id != m_ground_msg.header.frame_id) {
      throw std::runtime_error("RayGroundClassifierCloudNode: raw topic from unexpected frame");
    }
    // Verify the consistency of PointCloud msg
    const auto data_length = msg->width * msg->height * msg->point_step;
    if ((msg->data.size() != msg->row_step) || (data_length != msg->row_step)) {
      throw std::runtime_error("RayGroundClassifierCloudNode: Malformed PointCloud2");
    }
    // Verify the point cloud format and assign correct point_step
    constexpr auto field_size = sizeof(decltype(PointXYZIF::x));
    auto point_step = 4U * field_size;
    if (!has_intensity_and_throw_if_no_xyz(msg)) {
      point_step = 3U * field_size;
      RCLCPP_WARN(this->get_logger(),
        "RayGroundClassifierNode Warning: PointCloud doesn't have intensity field");
    }
    // Harvest timestamp
    m_nonground_msg.header.stamp = msg->header.stamp;
    m_ground_msg.header.stamp = msg->header.stamp;
    // Add all points to aggregator
    for (std::size_t idx = 0U; idx < msg->data.size(); idx += msg->point_step) {
      PointXYZIF pt;
      // TODO(c.ho) Fix below deviation after #2131 is in
      //lint -e{925, 9110} Need to convert pointers and use bit for external API NOLINT
      (void)memmove(
        static_cast<void *>(&pt.x),
        static_cast<const void *>(&msg->data[idx]),
        point_step);
      m_aggregator.insert(pt);
    }
    // Add end of scan
    m_aggregator.insert(eos_pt);
    // Partition each ray
    while (m_aggregator.is_ray_ready()) {
      // Note: if an exception occurs in this loop, the aggregator can get into a bad state
      // (e.g. overrun capacity)
      PointBlock ground_blk;
      PointBlock nonground_blk;
      // partition: should never fail, guaranteed to have capacity via other checks
      m_classifier.partition(m_aggregator.get_next_ray(), ground_blk, nonground_blk);
      // Add ray to point clouds
      for (auto & ground_point : ground_blk) {
        if (!add_point_to_cloud(m_ground_pc_its, ground_point, m_ground_pc_idx)) {
          throw std::runtime_error("RayGroundClassifierNode: Overran ground msg point capacity");
        }
      }
      for (auto & nonground_point : nonground_blk) {
        if (!add_point_to_cloud(m_nonground_pc_its, nonground_point, m_nonground_pc_idx)) {
          throw std::runtime_error("RayGroundClassifierNode: Overran nonground msg point capacity");
        }
      }
    }
    // Resize the clouds down to their actual sizes.
    autoware::common::lidar_utils::resize_pcl_msg(m_ground_msg, m_ground_pc_idx);
    m_ground_pc_its.reset(m_ground_msg, m_ground_pc_idx);
    autoware::common::lidar_utils::resize_pcl_msg(m_nonground_msg, m_nonground_pc_idx);
    m_nonground_pc_its.reset(m_nonground_msg, m_nonground_pc_idx);
    // publish: nonground first for the possible microseconds of latency
    m_nonground_pub_ptr->publish(m_nonground_msg);
    m_ground_pub_ptr->publish(m_ground_msg);
  } catch (const std::runtime_error & e) {
    m_has_failed = true;
    RCLCPP_INFO(this->get_logger(), e.what());
  } catch (const std::exception & e) {
    m_has_failed = true;
    RCLCPP_INFO(this->get_logger(), e.what());
  } catch (...) {
    RCLCPP_INFO(
      this->get_logger(),
      "RayGroundClassifierCloudNode has encountered an unknown failure");
    throw;
  }
}
////////////////////////////////////////////////////////////////////////////////
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RayGroundClassifierCloudNode::on_activate_internal(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "RayGroundClassifier has activated");
  m_ground_pub_ptr->on_activate();
  m_nonground_pub_ptr->on_activate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
////////////////////////////////////////////////////////////////////////////////
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RayGroundClassifierCloudNode::on_deactivate_internal(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "RayGroundClassifier has deactivated");
  m_ground_pub_ptr->on_deactivate();
  m_nonground_pub_ptr->on_deactivate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifierCloudNode::reset()
{
  // reset aggregator: Needed in case an error is thrown during partitioning of cloud
  //                   which would lead to filled rays and overflow during next callback
  while (m_aggregator.is_ray_ready()) {
    (void)m_aggregator.get_next_ray();
  }
  // reset messages
  autoware::common::lidar_utils::reset_pcl_msg(m_ground_msg, m_pcl_size, m_ground_pc_idx);
  m_ground_pc_its.reset(m_ground_msg, m_ground_pc_idx);
  autoware::common::lidar_utils::reset_pcl_msg(m_nonground_msg, m_pcl_size, m_nonground_pc_idx);
  m_nonground_pc_its.reset(m_nonground_msg, m_nonground_pc_idx);
}
////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifierCloudNode::register_callbacks_preallocate()
{
  if (!register_on_activate(
      std::bind(&RayGroundClassifierCloudNode::on_activate_internal, this, std::placeholders::_1)))
  {
    throw std::runtime_error("Could not register activate callback");
  }
  if (!register_on_deactivate(
      std::bind(&RayGroundClassifierCloudNode::on_deactivate_internal,
      this,
      std::placeholders::_1)))
  {
    throw std::runtime_error("Could not register deactivate callback");
  }
  // initialize messages
  init_pcl_msg(m_ground_msg, m_frame_id.c_str(), m_pcl_size);
  m_ground_pc_its.reset(m_ground_msg, 0);
  init_pcl_msg(m_nonground_msg, m_frame_id.c_str(), m_pcl_size);
  m_nonground_pc_its.reset(m_nonground_msg, 0);
}
}  // namespace ray_ground_classifier_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware
