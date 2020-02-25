// Copyright 2017-2018 Apex.AI, Inc.
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
/// \file
/// \brief This class defines common functions and classes to work with pointclouds

#ifndef LIDAR_UTILS__POINT_CLOUD_UTILS_HPP_
#define LIDAR_UTILS__POINT_CLOUD_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <common/types.hpp>
#include <lidar_utils/visibility_control.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace autoware
{
namespace common
{
namespace lidar_utils
{
using sensor_msgs::msg::PointCloud2;

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

/// max number of points in a scan for VLP16s, assuming 300 rpm = 5hz: 57870.3703 points per full
/// rotation
static const uint32_t MAX_SCAN_POINTS = 57872U;

/// Point cloud iterator wrapper, that allows to reuse iterators. Reinitializing
/// of iterators is quite costly due to the implementation of the
/// PointCloud2IteratorBase<...>::set_field method.
class LIDAR_UTILS_PUBLIC PointCloudIts
{
public:
  /// \brief Creates new iterator wrapper with space reserved for 4 iterators,
  /// namely x, y, z and intensity
  PointCloudIts();

  /// \brief Resets the iterators for the given cloud to the given index
  /// \param[in] cloud the point cloud to reset the iterators to
  /// \param[in] idx the index/offset to advance the iterators to
  void reset(sensor_msgs::msg::PointCloud2 & cloud, uint32_t idx = 0);

  /// \brief Returns iterator for the "x" field
  inline sensor_msgs::PointCloud2Iterator<float32_t> & x_it()
  {
    return m_its[0];
  }

  /// \brief Returns iterator for the "y" field
  inline sensor_msgs::PointCloud2Iterator<float32_t> & y_it()
  {
    return m_its[1];
  }

  /// \brief Returns iterator for the "z" field
  inline sensor_msgs::PointCloud2Iterator<float32_t> & z_it()
  {
    return m_its[2];
  }

  /// \brief Returns iterator for the "intensity" field
  sensor_msgs::PointCloud2Iterator<float32_t> & intensity_it()
  {
    return m_its[3];
  }

private:
  /// Internal storage of the iterators
  ::std::vector<sensor_msgs::PointCloud2Iterator<float32_t>> m_its;
};

/// \brief initializes header information for point cloud for x, y, z and intensity
/// \param[out] msg a point cloud message to initialize
/// \param[in] frame_id the name of the frame for the point cloud (assumed fixed)
/// \param[in] size number of points to preallocate for underyling data array
LIDAR_UTILS_PUBLIC void init_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::string & frame_id,
  const std::size_t size = static_cast<std::size_t>(MAX_SCAN_POINTS));

/// initializes header information for point cloud given frame id, size, number of frames and
///  a parameter pack of fields.
/// \tparam Fields Template paramater pack containing field types.
/// \param msg Point cloud message.
/// \param frame_id Frame ID of the point cloud.
/// \param size Size of the initialized point cloud.
/// \param num_fields Number of fields.
/// \param fields Set of parameters defining the fields. Each field must contain the following
/// parameters in strict order: `field_name, count, data_type`. These parameters should
/// be provided for each field
template<typename ... Fields>
LIDAR_UTILS_PUBLIC void init_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::string & frame_id,
  const std::size_t size,
  const uint32_t num_fields,
  Fields const & ... fields
)
{
  msg.height = 1U;
  msg.is_bigendian = false;
  msg.is_dense = false;
  msg.header.frame_id = frame_id;
  // set the fields
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(num_fields, fields ...);
  // allocate memory so that iterators can be used
  modifier.resize(size);
}

LIDAR_UTILS_PUBLIC bool8_t add_point_to_cloud(
  PointCloudIts & cloud_its,
  const autoware::common::types::PointXYZIF & pt,
  uint32_t & point_cloud_idx);

LIDAR_UTILS_PUBLIC bool8_t add_point_to_cloud(
  sensor_msgs::msg::PointCloud2 & cloud,
  const autoware::common::types::PointXYZIF & pt,
  uint32_t & point_cloud_idx);

LIDAR_UTILS_PUBLIC void reset_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::size_t size,
  uint32_t & point_cloud_idx);

LIDAR_UTILS_PUBLIC void resize_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::size_t new_size);

// Check the pointcloud msg has x, y, z fields, otherwise throw an exception; check
// the pointcloud msg has intensity field, otherwise return false
LIDAR_UTILS_PUBLIC bool8_t
has_intensity_and_throw_if_no_xyz(const PointCloud2::SharedPtr & cloud);

LIDAR_UTILS_PUBLIC bool8_t
has_intensity_and_throw_if_no_xyz(const PointCloud2 & cloud);

template<typename T>
struct _create_custom_pcl_datatype;

template<>
struct _create_custom_pcl_datatype<int8_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::INT8;
};

template<>
struct _create_custom_pcl_datatype<uint8_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::UINT8;
};

template<>
struct _create_custom_pcl_datatype<int16_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::INT16;
};

template<>
struct _create_custom_pcl_datatype<uint16_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::UINT16;
};

template<>
struct _create_custom_pcl_datatype<int32_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::INT32;
};

template<>
struct _create_custom_pcl_datatype<uint32_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::UINT32;
};

template<>
struct _create_custom_pcl_datatype<float32_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::FLOAT32;
};

template<>
struct _create_custom_pcl_datatype<float64_t>
{
  static constexpr auto DATATYPE = sensor_msgs::msg::PointField::FLOAT64;
};

template<typename T>
LIDAR_UTILS_PUBLIC
sensor_msgs::msg::PointCloud2::SharedPtr create_custom_pcl(
  const std::vector<std::string> & field_names,
  const uint32_t cloud_size)
{
  using sensor_msgs::msg::PointCloud2;
  PointCloud2::SharedPtr msg = std::make_shared<PointCloud2>();
  const auto field_size = field_names.size();
  msg->height = 1U;
  msg->width = cloud_size;
  msg->fields.resize(field_size);
  for (uint32_t i = 0U; i < field_size; i++) {
    msg->fields[i].name = field_names[i];
  }
  msg->point_step = 0U;
  for (uint32_t idx = 0U; idx < field_size; ++idx) {
    msg->fields[idx].offset = static_cast<uint32_t>(idx * sizeof(T));
    msg->fields[idx].datatype = _create_custom_pcl_datatype<T>::DATATYPE;
    msg->fields[idx].count = 1U;
    msg->point_step += static_cast<uint32_t>(sizeof(T));
  }
  const std::size_t capacity = msg->point_step * cloud_size;
  msg->data.clear();
  msg->data.reserve(capacity);
  for (std::size_t i = 0; i < capacity; ++i) {
    msg->data.emplace_back(0U);  // initialize all values equal to 0
  }
  msg->row_step = msg->point_step * msg->width;
  msg->is_bigendian = false;
  msg->is_dense = false;
  msg->header.frame_id = "base_link";
  return msg;
}

}  // namespace lidar_utils
}  // namespace common
}  // namespace autoware

#endif  // LIDAR_UTILS__POINT_CLOUD_UTILS_HPP_
