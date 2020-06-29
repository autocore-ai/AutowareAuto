// Copyright 2017-2020 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef POINT_CLOUD_FILTER_TRANSFORM_NODES__POINT_CLOUD_FILTER_TRANSFORM_NODE_HPP_
#define POINT_CLOUD_FILTER_TRANSFORM_NODES__POINT_CLOUD_FILTER_TRANSFORM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <lidar_utils/point_cloud_utils.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <point_cloud_filter_transform_nodes/visibility_control.hpp>
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

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using geometry_msgs::msg::Transform;
using geometry_msgs::msg::TransformStamped;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;

TransformStamped get_transform(
  const std::string & input_frame_id,
  const std::string & output_frame_id,
  float64_t r_x, float64_t r_y, float64_t r_z, float64_t r_w, float64_t t_x,
  float64_t t_y, float64_t t_z);

/// \brief Base class to subscribe to raw point cloud and transform and filter it to publish
///        filtered point cloud. Calls angle filter, distance filter and static transformer.
class POINT_CLOUD_FILTER_TRANSFORM_NODES_PUBLIC PointCloud2FilterTransformNode
  : public rclcpp::Node
{
public:
  /// \brief Explicit constructor
  /// \param init_timeout Timeout for initialization
  /// \param timeout Timeout for waitset to receive raw point cloud message
  /// \param raw_topic Name of the input topic containing raw point cloud
  /// \param filtered_topic Name of the output topic containing filtered point cloud
  /// \param start_angle Minimum angle in radians
  /// \param end_angle Maximum angle in radians. Points outside of the sector going counter
  ///                  clockwise from start and end angle will be discarded
  /// \param min_radius Radius in meters of the minimum point
  /// \param max_radius Radius in meters of the maximum point. Any point with radius less than
  ///                   min and greater than max will be discarded
  /// \param tf Transform msg to be applied to the raw points
  /// \param pcl_size Number of points to preallocate for filtered point cloud message
  /// \param expected_num_publishers Expected number of publishers for the raw point cloud topic
  /// \param expected_num_subscribers Expected number of subscribers for the filtered point topic
  /// \param node_options Additional options to control creation of the node.
  PointCloud2FilterTransformNode(
    const std::chrono::nanoseconds & init_timeout,
    const std::chrono::nanoseconds & timeout,
    const std::string & raw_topic,
    const std::string & filtered_topic,
    const float32_t start_angle,
    const float32_t end_angle,
    const float32_t min_radius,
    const float32_t max_radius,
    const geometry_msgs::msg::TransformStamped & tf,
    const size_t pcl_size,
    const size_t expected_num_publishers = 1U,
    const size_t expected_num_subscribers = 0U,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions{});

  /// \brief Parameter constructor
  /// \param node_options Additional options to control creation of the node.
  PointCloud2FilterTransformNode(
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions{});

protected:
  /// \brief Call distance & angle filter and then static transformer for all the points
  /// \param msg Raw point cloud
  /// \return Filtered and Transformed point cloud.
  /// \throws std::runtime_error on unexpected input contents or not enough output capacity
  const PointCloud2 & filter_and_transform(const PointCloud2 & msg);

  /// \brief Run main subscribe -> filter & transform -> publish loop
  void process_filtered_transformed_message(
    const PointCloud2::SharedPtr msg);

  template<typename PointType>
  /// \brief Check if the point is within the specified angle and radius limits
  /// \tparam PointType type with x, y, z.
  /// \param pt point with x, y, z
  /// \return True if the point is within the desired radius and angle limits. False otherwise.
  bool8_t point_not_filtered(const PointType & pt) const
  {
    return m_angle_filter(pt) && m_distance_filter(pt);
  }

  template<typename PointType>
  /// \brief Apply static transform to the given point
  /// \tparam PointType type with x, y, z
  /// \param pt point with x, y, z
  /// \return Transformed point
  PointType transform_point(const PointType & pt) const
  {
    PointType pt_final;
    m_static_transformer.transform(pt, pt_final);
    return pt_final;
  }

  class intensity_iterator_wrapper
  {
private:
    sensor_msgs::PointCloud2ConstIterator<uint8_t> m_intensity_it_uint8;
    sensor_msgs::PointCloud2ConstIterator<float32_t> m_intensity_it_float32;
    decltype(sensor_msgs::msg::PointField::datatype) m_intensity_datatype;

public:
    explicit intensity_iterator_wrapper(const PointCloud2 & msg);

    void next();

    bool8_t eof();

    template<typename PointFieldValueT>
    void get_curent_value(PointFieldValueT & point_field_value)
    {
      switch (m_intensity_datatype) {
        case sensor_msgs::msg::PointField::UINT8:
          point_field_value = *m_intensity_it_uint8;
          break;
        case sensor_msgs::msg::PointField::FLOAT32:
          point_field_value = *m_intensity_it_float32;
          break;
        default:
          throw std::runtime_error("Intensity type not supported: " + m_intensity_datatype);
      }
    }
  };

private:
  using AngleFilter = autoware::common::lidar_utils::AngleFilter;
  using DistanceFilter = autoware::common::lidar_utils::DistanceFilter;
  using StaticTransformer = autoware::common::lidar_utils::StaticTransformer;
  AngleFilter m_angle_filter;
  DistanceFilter m_distance_filter;
  StaticTransformer m_static_transformer;
  const std::chrono::nanoseconds m_init_timeout;
  const std::chrono::nanoseconds m_timeout;
  const typename rclcpp::Subscription<PointCloud2>::SharedPtr m_sub_ptr;
  const typename std::shared_ptr<rclcpp::Publisher<PointCloud2>> m_pub_ptr;
  const size_t m_expected_num_publishers;
  const size_t m_expected_num_subscribers;
  const std::string m_input_frame_id;
  const std::string m_output_frame_id;
  const std::size_t m_pcl_size;
  PointCloud2 m_filtered_transformed_msg;
};

}  // namespace point_cloud_filter_transform_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

#endif  // POINT_CLOUD_FILTER_TRANSFORM_NODES__POINT_CLOUD_FILTER_TRANSFORM_NODE_HPP_
