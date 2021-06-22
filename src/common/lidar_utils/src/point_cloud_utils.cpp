// Copyright 2017-2020 the Autoware Foundation, Arm Limited
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

#include <common/types.hpp>
#include <helper_functions/float_comparisons.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lidar_utils/point_cloud_utils.hpp"

static constexpr auto EPSf = std::numeric_limits<autoware::common::types::float32_t>::epsilon();

namespace autoware
{
namespace common
{
namespace comp = helper_functions::comparisons;
namespace lidar_utils
{

using autoware::common::types::bool8_t;
using autoware::common::types::char8_t;
using autoware::common::types::float32_t;

std::pair<autoware_auto_msgs::msg::PointClusters::_points_type::iterator,
  autoware_auto_msgs::msg::PointClusters::_points_type::iterator>
get_cluster(autoware_auto_msgs::msg::PointClusters & clusters, const std::size_t cls_id)
{
  if (cls_id >= clusters.cluster_boundary.size()) {
    return {clusters.points.end(), clusters.points.end()};
  }

  uint32_t cls_begin_offset;
  uint32_t cls_end_offset;  // this offset is the past-the-end element of the target cluster
  if (cls_id == 0U) {
    cls_begin_offset = 0U;
    cls_end_offset = clusters.cluster_boundary[cls_id];
  } else {
    cls_begin_offset = clusters.cluster_boundary[cls_id - 1U];
    cls_end_offset = clusters.cluster_boundary[cls_id];
  }

  return {clusters.points.begin() + cls_begin_offset, clusters.points.begin() + cls_end_offset};
}

std::pair<autoware_auto_msgs::msg::PointClusters::_points_type::const_iterator,
  autoware_auto_msgs::msg::PointClusters::_points_type::const_iterator>
get_cluster(const autoware_auto_msgs::msg::PointClusters & clusters, const std::size_t cls_id)
{
  if (cls_id >= clusters.cluster_boundary.size()) {
    return {clusters.points.end(), clusters.points.end()};
  }

  uint32_t cls_begin_offset;
  uint32_t cls_end_offset;  // this offset is the past-the-end element of the target cluster
  if (cls_id == 0U) {
    cls_begin_offset = 0U;
    cls_end_offset = clusters.cluster_boundary[cls_id];
  } else {
    cls_begin_offset = clusters.cluster_boundary[cls_id - 1U];
    cls_end_offset = clusters.cluster_boundary[cls_id];
  }

  return {clusters.points.begin() + cls_begin_offset, clusters.points.begin() + cls_end_offset};
}

bool8_t has_intensity_and_throw_if_no_xyz(
  const PointCloud2::SharedPtr & cloud)
{
  return has_intensity_and_throw_if_no_xyz(*cloud);
}

bool8_t has_intensity_and_throw_if_no_xyz(
  const PointCloud2 & cloud)
{
  bool8_t ret = true;
  // Validate point step
  if (cloud.fields.size() < 3U) {
    throw std::runtime_error("Invalid PointCloud msg");
  }

  const auto check_field = [](
    const sensor_msgs::msg::PointField & field,
    const char8_t * const name,
    const uint32_t offset,
    const decltype(sensor_msgs::msg::PointField::datatype) datatype) -> bool8_t {
      bool8_t res = true;
      if ((name != field.name) || (offset != field.offset) ||
        (datatype != field.datatype) || (1U != field.count))
      {
        res = false;
      }
      return res;
    };

  if (!check_field(cloud.fields[0U], "x", 0U, sensor_msgs::msg::PointField::FLOAT32)) {
    throw std::runtime_error("PointCloud doesn't have correct x field");
  } else if (!check_field(cloud.fields[1U], "y", 4U, sensor_msgs::msg::PointField::FLOAT32)) {
    throw std::runtime_error("PointCloud doesn't have correct y field");
  } else if (!check_field(cloud.fields[2U], "z", 8U, sensor_msgs::msg::PointField::FLOAT32)) {
    throw std::runtime_error("PointCloud doesn't have correct z field");
  } else {
    // do nothing
  }
  if (cloud.fields.size() >= 4U) {
    if (!check_field(cloud.fields[3U], "intensity", 12U, sensor_msgs::msg::PointField::FLOAT32)) {
      if (!check_field(cloud.fields[3U], "intensity", 16U, sensor_msgs::msg::PointField::UINT8)) {
        ret = false;
      }
    }
  } else {
    ret = false;
  }
  return ret;
}

PointCloudIts::PointCloudIts() {m_its.reserve(4);}

void PointCloudIts::reset(sensor_msgs::msg::PointCloud2 & cloud, uint32_t idx)
{
  // Destroy the old iterators
  m_its.clear();

  // Create new iterators
  m_its.emplace_back(cloud, "x");
  m_its.emplace_back(cloud, "y");
  m_its.emplace_back(cloud, "z");
  m_its.emplace_back(cloud, "intensity");

  // Advance iterators to given index
  // TODO(vrichard) replace explicit check by safe_cast
  // See https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1027
  if (idx > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
    // prevent future access to random memory value or segmentation fault
    throw std::runtime_error(
            "converting " + std::to_string(
              idx) + " to int would change sign of value");
  }
  const int i = static_cast<int>(idx);
  x_it() += i;
  y_it() += i;
  z_it() += i;
  intensity_it() += i;
}

std::size_t index_after_last_safe_byte_index(const sensor_msgs::msg::PointCloud2 & msg) noexcept
{
  // Count expected amount of data from various source of truths
  const auto expected_total_data1 =
    static_cast<std::size_t>(msg.point_step * (msg.width * msg.height));
  const auto expected_total_data2 = static_cast<std::size_t>(msg.row_step * msg.height);
  const auto actual_total_data = msg.data.size();
  // Get the smallest of these
  const auto min_data =
    std::min(std::min(expected_total_data1, expected_total_data2), actual_total_data);
  // Remove any data that doesn't align correctly with point_step
  const auto last_index = min_data - (min_data % msg.point_step);
  return last_index;
}

SafeCloudIndices sanitize_point_cloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  /// XYZI or XYZ, or throw
  auto num_floats = 3U;
  if (has_intensity_and_throw_if_no_xyz(msg)) {
    num_floats = 4U;
  }
  return SafeCloudIndices{num_floats * sizeof(float32_t), index_after_last_safe_byte_index(msg)};
}

void init_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::string & frame_id,
  const std::size_t size)
{
  init_pcl_msg(
    msg, frame_id, size, 4U,
    "x", 1U, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1U, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1U, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1U, sensor_msgs::msg::PointField::FLOAT32);
}

bool8_t add_point_to_cloud_raw(
  sensor_msgs::msg::PointCloud2 & cloud,
  const autoware::common::types::PointXYZIF & pt,
  uint32_t point_cloud_idx)
{
  bool8_t ret = false;

  // Actual size is 20 due to padding by compilers for the memory alignment boundary.
  // This check is to make sure that when we do a insert of 16 bytes, we will not stride
  // past the bounds of the structure.
  static_assert(
    sizeof(autoware::common::types::PointXYZIF) >= ((4U * sizeof(float32_t)) + sizeof(uint16_t)),
    "PointXYZF is not expected size: ");
  static_assert(
    std::is_trivially_copyable<autoware::common::types::PointXYZIF>::value,
    "PointXYZF is not trivial, add_point_to_cloud instead of add_point_to_cloud_raw");

  const uint8_t * casted_point_ptr = reinterpret_cast<const uint8_t *>(&pt);
  uint8_t * const cloud_insertion_slot = &cloud.data[cloud.point_step * point_cloud_idx];
  std::size_t copy_size = std::min(static_cast<std::size_t>(cloud.point_step), sizeof(pt));

  // make sure we don't overflow
  uint8_t * const one_past_last_modified_address = &(cloud_insertion_slot[copy_size]);
  uint8_t * const vector_one_past_last_address = &(*cloud.data.end());

  if (one_past_last_modified_address <= vector_one_past_last_address) {
    // add the point data
    std::copy(casted_point_ptr, casted_point_ptr + copy_size, cloud_insertion_slot);
    ret = true;
  }
  return ret;
}

bool8_t add_point_to_cloud(
  PointCloudIts & cloud_its,
  const autoware::common::types::PointXYZIF & pt,
  uint32_t & point_cloud_idx)
{
  bool8_t ret = false;

  auto & x_it = cloud_its.x_it();
  auto & y_it = cloud_its.y_it();
  auto & z_it = cloud_its.z_it();
  auto & intensity_it = cloud_its.intensity_it();

  // Actual size is 20 due to padding by compilers for the memory alignment boundary.
  // This check is to make sure that when we do a insert of 16 bytes, we will not stride
  // past the bounds of the structure.
  static_assert(
    sizeof(autoware::common::types::PointXYZIF) >= ((4U * sizeof(float32_t)) + sizeof(uint16_t)),
    "PointXYZF is not expected size: ");

  if (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end() &&
    intensity_it != intensity_it.end())
  {
    // add the point data
    *x_it = pt.x;
    *y_it = pt.y;
    *z_it = pt.z;
    *intensity_it = pt.intensity;

    // increment the index to keep track of the pointcloud's size
    x_it += 1;
    y_it += 1;
    z_it += 1;
    intensity_it += 1;
    ++point_cloud_idx;

    ret = true;
  }
  return ret;
}

bool8_t add_point_to_cloud(
  sensor_msgs::msg::PointCloud2 & cloud,
  const autoware::common::types::PointXYZIF & pt,
  uint32_t & point_cloud_idx)
{
  bool8_t ret = false;

  sensor_msgs::PointCloud2Iterator<float32_t> x_it(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float32_t> y_it(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float32_t> z_it(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float32_t> intensity_it(cloud, "intensity");

  // TODO(vrichard) replace explicit check by safe_cast
  // See https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1027
  if (point_cloud_idx > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
    // prevent future access to random memory value or segmentation fault
    throw std::runtime_error(
            "converting " + std::to_string(
              point_cloud_idx) + " to int would change sign of value");
  }
  const int idx = static_cast<int>(point_cloud_idx);
  x_it += idx;
  y_it += idx;
  z_it += idx;
  intensity_it += idx;

  // Actual size is 20 due to padding by compilers for the memory alignment boundary.
  // This check is to make sure that when we do a insert of 16 bytes, we will not stride
  // past the bounds of the structure.
  static_assert(
    sizeof(autoware::common::types::PointXYZIF) >= ((4U * sizeof(float32_t)) + sizeof(uint16_t)),
    "PointXYZF is not expected size: ");

  if (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end() &&
    intensity_it != intensity_it.end())
  {
    // add the point data
    *x_it = pt.x;
    *y_it = pt.y;
    *z_it = pt.z;
    *intensity_it = pt.intensity;

    // increment the index to keep track of the pointcloud's size
    ++point_cloud_idx;
    ret = true;
  }
  return ret;
}

bool8_t add_point_to_cloud(
  sensor_msgs::msg::PointCloud2 & cloud,
  const autoware::common::types::PointXYZF & pt,
  uint32_t & point_cloud_idx)
{
  bool8_t ret = false;

  sensor_msgs::PointCloud2Iterator<float32_t> x_it(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float32_t> y_it(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float32_t> z_it(cloud, "z");

  // TODO(vrichard) replace explicit check by safe_cast
  // See https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1027
  if (point_cloud_idx > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
    // prevent future access to random memory value or segmentation fault
    throw std::runtime_error(
            "converting " + std::to_string(
              point_cloud_idx) + " to int would change sign of value");
  }
  const int idx = static_cast<int>(point_cloud_idx);
  x_it += idx;
  y_it += idx;
  z_it += idx;

  static_assert(
    sizeof(autoware::common::types::PointXYZIF) >= ((3U * sizeof(float32_t)) + sizeof(uint16_t)),
    "PointXYZF is not expected size: ");

  if (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end())
  {
    // add the point data
    *x_it = pt.x;
    *y_it = pt.y;
    *z_it = pt.z;

    // increment the index to keep track of the pointcloud's size
    ++point_cloud_idx;
    ret = true;
  }
  return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////

void reset_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::size_t size,
  uint32_t & point_cloud_idx)
{
  sensor_msgs::PointCloud2Modifier pc_modifier(msg);
  pc_modifier.clear();
  point_cloud_idx = 0;
  pc_modifier.resize(size);
}

/////////////////////////////////////////////////////////////////////////////////////////

void resize_pcl_msg(
  sensor_msgs::msg::PointCloud2 & msg,
  const std::size_t new_size)
{
  sensor_msgs::PointCloud2Modifier pc_modifier(msg);
  pc_modifier.resize(new_size);
}

DistanceFilter::DistanceFilter(float32_t min_radius, float32_t max_radius)
: m_min_r2(min_radius * min_radius), m_max_r2(max_radius * max_radius)
{
  if (m_max_r2 < m_min_r2) {
    throw std::domain_error("DistanceFilter: max_radius cannot be less than min_radius");
  }
}

// odr-used by comp::abs_gte
constexpr float32_t DistanceFilter::FEPS;


StaticTransformer::StaticTransformer(const geometry_msgs::msg::Transform & tf)
{
  Eigen::Quaternionf rotation{
    static_cast<float>(tf.rotation.w),
    static_cast<float>(tf.rotation.x),
    static_cast<float>(tf.rotation.y),
    static_cast<float>(tf.rotation.z)};
  if (!comp::rel_eq(rotation.norm(), 1.0f, EPSf)) {
    throw std::domain_error("StaticTransformer: quaternion is not normalized");
  }
  m_tf.setIdentity();
  m_tf.linear() = rotation.toRotationMatrix();
  m_tf.translation() = Eigen::Vector3f{
    static_cast<float>(tf.translation.x),
    static_cast<float>(tf.translation.y),
    static_cast<float>(tf.translation.z)};
}

AngleFilter::AngleFilter(float32_t start_angle, float32_t end_angle)
{
  using autoware::common::geometry::make_unit_vector2d;
  using autoware::common::geometry::cross_2d;
  using autoware::common::geometry::get_normal;
  using autoware::common::geometry::plus_2d;
  using autoware::common::geometry::times_2d;
  using autoware::common::geometry::norm_2d;
  using autoware::common::geometry::dot_2d;

  const auto start_vec = make_unit_vector2d<VectorT>(start_angle);
  const auto end_vec = make_unit_vector2d<VectorT>(end_angle);

  // Handle the case where two angles are in opposite direction (small_angle_dir = 0)
  if (std::fabs(dot_2d(start_vec, end_vec) - (-1.0F)) < FEPS) {
    m_range_normal = get_normal(start_vec);
  } else {
    // range normal is the unit vector in the middle of the accepted range.(normalize(start + end))
    m_range_normal = plus_2d(start_vec, end_vec);
    m_range_normal = times_2d(
      m_range_normal,
      (1.0F / norm_2d(m_range_normal)));

    // If the small angle is not in CCW direction, then we need the wide angle, so the normal is
    // inverted.
    const auto small_angle_dir = cross_2d(start_vec, end_vec);
    if ((small_angle_dir + FEPS) < 0.0F) {
      m_range_normal = times_2d(m_range_normal, -1.0F);
    }
  }

  // Threshold is the cosine of the half of the accepted angle range.
  // The geometrical interpretation:
  // The closer a point's azimuth to the range normal is, the bigger its projection length on the
  // range normal will be. (proportional to the cosine of the angle between them). Hence the
  // projection length is an indicator of a point's angular distance to the angle range.
  // The min and max of the angle range define a lower bound on this metric.
  const auto thresh = dot_2d(start_vec, m_range_normal);
  m_threshold_negative = (thresh + FEPS) < 0.0F;
  // square is pre-computed as the check will happen in the square space to avoid sqrt() calls.
  m_threshold2 = thresh * thresh;
}

bool8_t IntensityIteratorWrapper::eof()
{
  switch (m_intensity_datatype) {
    // For some reason, the equality operator (==) does not work with PointCloud2ConstIterator
    case sensor_msgs::msg::PointField::UINT8:
      return !(m_intensity_it_uint8 != m_intensity_it_uint8.end());
    case sensor_msgs::msg::PointField::FLOAT32:
      return !(m_intensity_it_float32 != m_intensity_it_float32.end());
    default:
      throw std::runtime_error(
              "Intensity type not supported: " +
              std::to_string(m_intensity_datatype));
  }
}

void IntensityIteratorWrapper::next()
{
  switch (m_intensity_datatype) {
    case sensor_msgs::msg::PointField::UINT8:
      ++m_intensity_it_uint8;
      break;
    case sensor_msgs::msg::PointField::FLOAT32:
      ++m_intensity_it_float32;
      break;
    default:
      throw std::runtime_error(
              "Intensity type not supported: " +
              std::to_string(m_intensity_datatype));
  }
}

IntensityIteratorWrapper::IntensityIteratorWrapper(
  const PointCloud2 & msg)
: m_intensity_it_uint8(msg, "intensity"),
  m_intensity_it_float32(msg, "intensity")
{
  auto && intensity_field_it =
    std::find_if(
    std::cbegin(msg.fields), std::cend(msg.fields),
    [](const sensor_msgs::msg::PointField & field) {return field.name == "intensity";});
  m_intensity_datatype = (*intensity_field_it).datatype;
  switch (m_intensity_datatype) {
    case sensor_msgs::msg::PointField::UINT8:
    case sensor_msgs::msg::PointField::FLOAT32:
      break;
    default:
      throw std::runtime_error(
              "Intensity type not supported: " +
              std::to_string(m_intensity_datatype));
  }
}

}  // namespace lidar_utils
}  // namespace common
}  // namespace autoware
