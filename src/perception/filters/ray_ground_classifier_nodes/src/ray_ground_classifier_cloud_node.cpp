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

#include "lidar_utils/lidar_types.hpp"
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
using autoware::common::lidar_utils::PointXYZIF;
using autoware::perception::filters::ray_ground_classifier::PointBlock;

using std::placeholders::_1;

using autoware::common::lidar_utils::init_pcl_msg;

RayGroundClassifierCloudNode::RayGroundClassifierCloudNode(
  const std::string & node_name,
  const std::string & node_namespace,
  const std::string & param_file)
: LifecycleNode(
    node_name.c_str(),
    node_namespace.c_str(),
    rclcpp::contexts::default_context::get_global_default_context(),
    {(std::string{"__params:="} + param_file).c_str()},
    {}),
  m_classifier(ray_ground_classifier::Config{
          static_cast<float>(get_parameter("classifier.sensor_height_m").as_double()),
          static_cast<float>(get_parameter("classifier.max_local_slope_deg").as_double()),
          static_cast<float>(get_parameter("classifier.max_global_slope_deg").as_double()),
          static_cast<float>(get_parameter(
            "classifier.nonground_retro_thresh_deg").as_double()),
          static_cast<float>(get_parameter("classifier.min_height_thresh_m").as_double()),
          static_cast<float>(get_parameter(
            "classifier.max_global_height_thresh_m").as_double()),
          static_cast<float>(get_parameter(
            "classifier.max_last_local_ground_thresh_m").as_double()),
          static_cast<float>(get_parameter(
            "classifier.max_provisional_ground_distance_m").as_double()),
          static_cast<float>(get_parameter("classifier.min_height_m").as_double()),
          static_cast<float>(get_parameter("classifier.max_height_m").as_double())
        }),
  m_aggregator(ray_ground_classifier::RayAggregator::Config{
          static_cast<float>(get_parameter("aggregator.min_ray_angle_rad").as_double()),
          static_cast<float>(get_parameter("aggregator.max_ray_angle_rad").as_double()),
          static_cast<float>(get_parameter("aggregator.ray_width_rad").as_double()),
          static_cast<std::size_t>(get_parameter("aggregator.max_ray_points").as_int())
        }),
  m_pcl_size(static_cast<std::size_t>(get_parameter("pcl_size").as_int())),
  m_frame_id(get_parameter("frame_id").as_string().c_str()),
  m_has_failed(false),
  m_timeout(std::chrono::milliseconds{get_parameter("cloud_timeout_ms").as_int()}),
  m_raw_sub_ptr(create_subscription<PointCloud2>(get_parameter("raw_topic").as_string(),
    std::bind(&RayGroundClassifierCloudNode::callback, this, _1))),
  m_ground_pub_ptr(create_publisher<PointCloud2>(get_parameter("ground_topic").as_string())),
  m_nonground_pub_ptr(create_publisher<PointCloud2>(get_parameter("nonground_topic").as_string()))
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
    std::bind(&RayGroundClassifierCloudNode::callback, this, _1))),
  m_ground_pub_ptr(create_publisher<PointCloud2>(ground_topic.c_str())),
  m_nonground_pub_ptr(create_publisher<PointCloud2>(nonground_topic.c_str()))
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
        msg->point_step);
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
        if (!add_point_to_cloud(m_ground_msg, ground_point, m_point_cloud_idx)) {
          throw std::runtime_error("RayGroundClassifierNode: Overran ground msg point capacity");
        }
      }
      reset_cloud_idx();
      for (auto & nonground_point : nonground_blk) {
        if (!add_point_to_cloud(m_nonground_msg, nonground_point, m_point_cloud_idx)) {
          throw std::runtime_error("RayGroundClassifierNode: Overran nonground msg point capacity");
        }
      }
      reset_cloud_idx();
    }
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
  RCLCPP_INFO(this->get_logger(), "RayGroundClassifier has activated");
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
  m_nonground_msg.data.clear();
  m_nonground_msg.width = 0U;
  m_ground_msg.data.clear();
  m_ground_msg.width = 0U;
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
  init_pcl_msg(m_nonground_msg, m_frame_id.c_str(), m_pcl_size);
}
////////////////////////////////////////////////////////////////////////////////
void RayGroundClassifierCloudNode::reset_cloud_idx()
{
  m_point_cloud_idx = 0;
}
}  // namespace ray_ground_classifier_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware
