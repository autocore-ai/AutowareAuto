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

using autoware::common::types::float64_t;
using geometry_msgs::msg::Transform;
using sensor_msgs::msg::PointCloud2;
using std::placeholders::_1;

Transform get_transform(
  float64_t r_x, float64_t r_y, float64_t r_z, float64_t r_w, float64_t t_x,
  float64_t t_y, float64_t t_z);

/// \brief Base class to subscribe to raw point cloud and transform and filter it to publish
///        filtered point cloud. Calls angle filter, distance filter and static transformer.
/// \tparam PointCloudT desired point cloud type. Currently specialized for PointCloud2.
template<class PointCloudT>
class POINT_CLOUD_FILTER_TRANSFORM_NODES_PUBLIC PointCloudFilterTransformNodeBase
  : public rclcpp::Node
{
public:
  /// \brief Explicit constructor
  /// \param node_name Name of this node
  /// \param node_namespace Name of this node's namespace
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
  /// \param expected_num_publishers Expected number of publishers for the raw point cloud topic
  /// \param expected_num_subscribers Expected number of subscribers for the filtered point topic
  PointCloudFilterTransformNodeBase(
    const std::string & node_name,
    const std::string & node_namespace,
    const std::chrono::nanoseconds & init_timeout,
    const std::chrono::nanoseconds & timeout,
    const std::string & raw_topic,
    const std::string & filtered_topic,
    const float32_t start_angle,
    const float32_t end_angle,
    const float32_t min_radius,
    const float32_t max_radius,
    const Transform & tf,
    const size_t expected_num_publishers,
    const size_t expected_num_subscribers)
  : Node(node_name.c_str(), node_namespace.c_str()),
    m_angle_filter{start_angle, end_angle}, m_distance_filter{min_radius, max_radius},
    m_static_transformer{tf}, m_init_timeout{init_timeout}, m_timeout{timeout},
    m_sub_ptr{create_subscription<PointCloudT>(
        raw_topic.c_str(), rclcpp::QoS{10},
        std::bind(
          &PointCloudFilterTransformNodeBase::process_filtered_transformed_message, this, _1))},
    m_pub_ptr{create_publisher<PointCloudT>(filtered_topic.c_str(), rclcpp::QoS{10})},
    m_expected_num_publishers{expected_num_publishers},
    m_expected_num_subscribers{expected_num_subscribers}
  {
  }

  /// \brief Parameter constructor
  /// \param node_name Name of this node
  /// \param node_namespace Name of this node's namespace
  PointCloudFilterTransformNodeBase(
    const std::string & node_name,
    const std::string & node_namespace)
  : Node(node_name.c_str(), node_namespace.c_str()),
    m_angle_filter{static_cast<float32_t>(declare_parameter("start_angle").get<float64_t>()),
      static_cast<float32_t>(declare_parameter("end_angle").get<float64_t>())},
    m_distance_filter{static_cast<float32_t>(declare_parameter("min_radius").get<float64_t>()),
      static_cast<float32_t>(declare_parameter("max_radius").get<float64_t>())},
    m_static_transformer{get_transform(declare_parameter(
          "static_transformer.quaternion.x").get<float64_t>(),
        declare_parameter("static_transformer.quaternion.y").get<float64_t>(),
        declare_parameter("static_transformer.quaternion.z").get<float64_t>(),
        declare_parameter("static_transformer.quaternion.w").get<float64_t>(),
        declare_parameter("static_transformer.translation.x").get<float64_t>(),
        declare_parameter("static_transformer.translation.y").get<float64_t>(),
        declare_parameter("static_transformer.translation.z").get<float64_t>())},
    m_init_timeout{std::chrono::milliseconds{declare_parameter("init_timeout_ms").get<int32_t>()}},
    m_timeout{std::chrono::milliseconds{declare_parameter("timeout_ms").get<int32_t>()}},
    m_sub_ptr{create_subscription<PointCloudT>(
        declare_parameter("raw_topic").get<std::string>(), rclcpp::QoS{10},
        std::bind(
          &PointCloudFilterTransformNodeBase::process_filtered_transformed_message, this, _1))},
    m_pub_ptr{create_publisher<PointCloudT>(
        declare_parameter("filtered_topic").get<std::string>(), rclcpp::QoS{10})},
    m_expected_num_publishers{
      static_cast<size_t>(declare_parameter("expected_num_publishers").get<int32_t>())},
    m_expected_num_subscribers{
      static_cast<size_t>(declare_parameter("expected_num_subscribers").get<int32_t>())}
  {
  }

protected:
  /// \brief Run main subscribe -> filter & transform -> publish loop
  void process_filtered_transformed_message(
    const typename PointCloudT::SharedPtr msg)
  {
    const auto filtered_transformed_msg = filter_and_transform(*msg);
    m_pub_ptr->publish(filtered_transformed_msg);
  }

  virtual const PointCloudT & filter_and_transform(const PointCloudT & msg) = 0;

  template<typename PointType>
  /// \brief Check if the point is within the specified angle and radius limits
  /// \tparam PointType type with x, y, z.
  /// \param pt point with x, y, z
  /// \return True if the point is within the desired radius and angle limits. False otherwise.
  bool point_not_filtered(const PointType & pt) const
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
    intensity_iterator_wrapper(const PointCloud2 & msg)
    : m_intensity_it_uint8(msg, "intensity"),
      m_intensity_it_float32(msg, "intensity")
    {
      auto && intensity_field_it =
        std::find_if(std::cbegin(msg.fields), std::cend(msg.fields),
          [](const sensor_msgs::msg::PointField & field) {return field.name == "intensity";});
      m_intensity_datatype = (*intensity_field_it).datatype;
      switch (m_intensity_datatype) {
        case sensor_msgs::msg::PointField::UINT8:
        case sensor_msgs::msg::PointField::FLOAT32:
          break;
        default:
          throw std::runtime_error("Intensity type not supported: " + m_intensity_datatype);
      }
    }

    void next()
    {
      switch (m_intensity_datatype) {
        case sensor_msgs::msg::PointField::UINT8:
          ++m_intensity_it_uint8;
          break;
        case sensor_msgs::msg::PointField::FLOAT32:
          ++m_intensity_it_float32;
          break;
        default:
          throw std::runtime_error("Intensity type not supported: " + m_intensity_datatype);
      }
    }

    bool8_t eof()
    {
      switch (m_intensity_datatype) {
        // For some reason, the equality operator (==) does not work with PointCloud2ConstIterator
        case sensor_msgs::msg::PointField::UINT8:
          return !(m_intensity_it_uint8 != m_intensity_it_uint8.end());
        case sensor_msgs::msg::PointField::FLOAT32:
          return !(m_intensity_it_float32 != m_intensity_it_float32.end());
        default:
          throw std::runtime_error("Intensity type not supported: " + m_intensity_datatype);
      }
    }

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
  const typename rclcpp::Subscription<PointCloudT>::SharedPtr m_sub_ptr;
  const typename std::shared_ptr<rclcpp::Publisher<PointCloudT>> m_pub_ptr;
  const size_t m_expected_num_publishers;
  const size_t m_expected_num_subscribers;
};

/// \brief Specialized filter/transform class for PointCloud2
class POINT_CLOUD_FILTER_TRANSFORM_NODES_PUBLIC PointCloud2FilterTransformNode : public
  PointCloudFilterTransformNodeBase<PointCloud2>
{
public:
  /// \brief Parameter constructor
  /// \param node_name Name of this node.
  /// \param node_namespace Name of this node's namespace
  PointCloud2FilterTransformNode(
    const std::string & node_name,
    const std::string & node_namespace = "");

  /// \brief Explicit constructor
  /// \param node_name Name of this node
  /// \param node_namespace Name of this node's namespace
  /// \param init_timeout Timeout for initialization
  /// \param timeout Timeout for waitset to receive raw point cloud message
  /// \param input_frame_id Expected frame_id of the input point cloud message
  /// \param output_frame_id frame_id of the point cloud message after it is filtered & transformed
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
  PointCloud2FilterTransformNode(
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
    const size_t expected_num_publishers = 1U,
    const size_t expected_num_subscribers = 0U);

protected:
  /// \brief Call distance & angle filter and then static transformer for all the points
  /// \param msg Raw point cloud
  /// \return Filtered and Transformed point cloud.
  /// \throws std::runtime_error on unexpected input contents or not enough output capacity
  const PointCloud2 & filter_and_transform(const PointCloud2 & msg) override;

  Transform get_transform_from_parameters(const std::string & prefix);

private:
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
