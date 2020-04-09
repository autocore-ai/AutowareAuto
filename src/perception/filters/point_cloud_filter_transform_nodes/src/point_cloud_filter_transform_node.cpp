// Copyright 2017-2020 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <common/types.hpp>
#include <point_cloud_filter_transform_nodes/point_cloud_filter_transform_node.hpp>
#include <memory>
#include <string>

namespace autoware
{
namespace perception
{
namespace filters
{
/// \brief Boilerplate Apex.OS nodes around point_cloud_filter_transform_nodes
namespace point_cloud_filter_transform_nodes
{
using autoware::common::lidar_utils::add_point_to_cloud;
using autoware::common::lidar_utils::has_intensity_and_throw_if_no_xyz;
using autoware::common::lidar_utils::reset_pcl_msg;
using autoware::common::lidar_utils::resize_pcl_msg;
using autoware::common::lidar_utils::sanitize_point_cloud;
using autoware::common::types::float64_t;
using autoware::common::types::PointXYZIF;
using geometry_msgs::msg::Transform;
using sensor_msgs::msg::PointCloud2;

Transform PointCloud2FilterTransformNode::get_transform_from_parameters(const std::string & prefix)
{
  Transform ret;
  ret.rotation.x = declare_parameter(prefix + ".quaternion.x").get<float64_t>();
  ret.rotation.y = declare_parameter(prefix + ".quaternion.y").get<float64_t>();
  ret.rotation.z = declare_parameter(prefix + ".quaternion.z").get<float64_t>();
  ret.rotation.w = declare_parameter(prefix + ".quaternion.w").get<float64_t>();
  ret.translation.x = declare_parameter(prefix + ".translation.x").get<float64_t>();
  ret.translation.y = declare_parameter(prefix + ".translation.y").get<float64_t>();
  ret.translation.z = declare_parameter(prefix + ".translation.z").get<float64_t>();
  return ret;
}

Transform get_transform(
  double r_x, double r_y, double r_z, double r_w, double t_x,
  double t_y, double t_z)
{
  Transform ret;
  ret.rotation.x = r_x;
  ret.rotation.y = r_y;
  ret.rotation.z = r_z;
  ret.rotation.w = r_w;
  ret.translation.x = t_x;
  ret.translation.y = t_y;
  ret.translation.z = t_z;
  return ret;
}

PointCloud2FilterTransformNode::PointCloud2FilterTransformNode(
  const std::string & node_name,
  const std::string & node_namespace)
: PointCloudFilterTransformNodeBase(
    node_name, node_namespace),
  m_input_frame_id{declare_parameter("input_frame_id").get<std::string>()},
  m_output_frame_id{declare_parameter("output_frame_id").get<std::string>()},
  m_pcl_size{static_cast<size_t>(declare_parameter("pcl_size").get<int32_t>())}
{
  common::lidar_utils::init_pcl_msg(m_filtered_transformed_msg,
    m_output_frame_id.c_str(), m_pcl_size);
}

PointCloud2FilterTransformNode::PointCloud2FilterTransformNode(
  const std::string & node_name,
  const std::string & node_namespace,
  const std::chrono::nanoseconds & init_timeout,
  const std::chrono::nanoseconds & timeout,
  const std::string & input_frame_id,
  const std::string & output_frame_id,
  const std::string & raw_topic,
  const std::string & filtered_topic,
  const float32_t start_angle,
  const float32_t end_angle,
  const float32_t min_radius,
  const float32_t max_radius,
  const geometry_msgs::msg::Transform & tf,
  const size_t pcl_size,
  const size_t expected_num_publishers,
  const size_t expected_num_subscribers)
: PointCloudFilterTransformNodeBase(node_name, node_namespace, init_timeout,
    timeout, raw_topic, filtered_topic, start_angle, end_angle, min_radius, max_radius,
    tf, expected_num_publishers, expected_num_subscribers),
  m_input_frame_id{input_frame_id}, m_output_frame_id(output_frame_id),
  m_pcl_size{pcl_size}
{
  common::lidar_utils::init_pcl_msg(m_filtered_transformed_msg,
    m_output_frame_id.c_str(), m_pcl_size);
}

const PointCloud2 & PointCloud2FilterTransformNode::filter_and_transform(const PointCloud2 & msg)
{
  // Verify frame_id
  if (msg.header.frame_id != m_input_frame_id) {
    throw std::runtime_error("Raw topic from unexpected frame");
  }
  // Sanitize indexing for iteration; warn if sanitation occured
  const auto indices = sanitize_point_cloud(msg);
  if (indices.point_step != msg.point_step) {
    RCLCPP_WARN(get_logger(), "Using only a subset of Point cloud fields");
  }
  if (indices.data_length != msg.data.size()) {
    RCLCPP_WARN(get_logger(), "Misaligned data: Using only a subset of Point cloud data");
  }

  auto point_cloud_idx = 0U;
  reset_pcl_msg(m_filtered_transformed_msg, m_pcl_size, point_cloud_idx);
   m_filtered_transformed_msg.header.stamp = msg.header.stamp;
  for (auto idx = 0U; idx < indices.data_length; idx += msg.point_step) {
    PointXYZIF pt;
    //lint -e{925, 9110} Need to convert pointers and use bit for external API NOLINT
    (void)memmove(
      static_cast<void *>(&pt.x),
      static_cast<const void *>(&msg.data[idx]),
      indices.point_step);
    if (point_not_filtered(pt)) {
      if (!add_point_to_cloud(
          m_filtered_transformed_msg, transform_point(pt), point_cloud_idx))
      {
        throw std::runtime_error(
                "Overran cloud msg point capacity");
      }
    }
  }
  resize_pcl_msg(m_filtered_transformed_msg, point_cloud_idx);
  return m_filtered_transformed_msg;
}

}  // namespace point_cloud_filter_transform_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware
