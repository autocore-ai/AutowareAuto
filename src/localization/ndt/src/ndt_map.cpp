// Copyright 2019 the Autoware Foundation
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

#include <ndt/ndt_map.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <string>

namespace autoware
{
namespace localization
{
namespace ndt
{
uint32_t validate_pcl_map(const sensor_msgs::msg::PointCloud2 & msg)
{
  // lambda to check the fields of a PointField
  auto field_valid = [](const auto & field, auto data_type, auto offset, auto count) {
      return (field.datatype == data_type) && (field.offset == offset) && (field.count == count);
    };

  auto ret = 0U;
  const auto double_field_size =
    static_cast<uint32_t>(sizeOfPointField(sensor_msgs::msg::PointField::FLOAT64));

  // TODO(cvasfi): Possibly allow additional fields that don't change the order of the fields
  constexpr auto expected_num_fields = 9U;

  // Check general pc metadata
  if ((msg.fields.size()) != expected_num_fields ||
    (msg.point_step != (expected_num_fields * double_field_size)) ||
    (msg.height != 1U))
  {
    return 0U;
  }

  // check PointField fields
  // Get ID of the last field before cell_ID for reverse iterating. (used to calculate offset)
  auto double_field_idx = expected_num_fields - 1U;
  if (!std::all_of(
      msg.fields.rbegin(), msg.fields.rend(),               // check all float fields
      [&double_field_idx, &field_valid, double_field_size](auto & field) {
        return field_valid(
          field, sensor_msgs::msg::PointField::FLOAT64,
          ((double_field_idx--) * double_field_size), 1U);
      }))
  {
    return 0U;
  }

  // Check field names
  if ((msg.fields[0U].name != "x") ||
    (msg.fields[1U].name != "y") ||
    (msg.fields[2U].name != "z") ||
    (msg.fields[3U].name != "icov_xx") ||
    (msg.fields[4U].name != "icov_xy") ||
    (msg.fields[5U].name != "icov_xz") ||
    (msg.fields[6U].name != "icov_yy") ||
    (msg.fields[7U].name != "icov_yz") ||
    (msg.fields[8U].name != "icov_zz"))
  {
    return 0U;
  }

  // If the actual size and the meta data is in conflict, use the minimum length to be safe.
  const auto min_data_length = std::min(
    static_cast<decltype(msg.row_step)>(msg.data.size()),
    std::min(msg.row_step, msg.width * msg.point_step));
  // Trim the length to make it divisible to point_step, excess data cannot be read.
  const auto safe_data_length = min_data_length - (min_data_length % msg.point_step);
  // Return number of points that can safely be read from the point cloud
  ret = safe_data_length / msg.point_step;

  return ret;
}

DynamicNDTMap::DynamicNDTMap(const Config & voxel_grid_config)
: m_grid{voxel_grid_config} {}

const std::string & DynamicNDTMap::frame_id() const noexcept
{
  return m_frame_id;
}

DynamicNDTMap::TimePoint DynamicNDTMap::stamp() const noexcept
{
  return m_stamp;
}

bool DynamicNDTMap::valid() const noexcept
{
  return (m_grid.size() > 0U) && (!m_frame_id.empty());
}

const DynamicNDTMap::ConfigPoint & DynamicNDTMap::cell_size() const noexcept
{
  return m_grid.cell_size();
}

void DynamicNDTMap::set(const sensor_msgs::msg::PointCloud2 & msg)
{
  m_grid.clear();
  insert(msg);
}

void DynamicNDTMap::insert(const sensor_msgs::msg::PointCloud2 & msg)
{
  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it(msg, "z");

  while (x_it != x_it.end() &&
    y_it != y_it.end() &&
    z_it != z_it.end())
  {
    const auto pt = Point({*x_it, *y_it, *z_it});
    m_grid.add_observation(pt);  // Add or insert new voxel.

    ++x_it;
    ++y_it;
    ++z_it;
  }
  // try to stabilizie the covariance after inserting all the points
  for (auto & vx_it : m_grid) {
    auto & vx = vx_it.second;
    (void) vx.try_stabilize();
  }
  m_stamp = ::time_utils::from_message(msg.header.stamp);
  m_frame_id = msg.header.frame_id;
}

void push_back(
  std::array<sensor_msgs::PointCloud2Iterator<ndt::Real>, 9U> & pc_its,
  const SerializedNDTMapPoint & point)
{
  *pc_its[0U] = point.x;
  *pc_its[1U] = point.y;
  *pc_its[2U] = point.z;
  *pc_its[3U] = point.icov_xx;
  *pc_its[4U] = point.icov_xy;
  *pc_its[5U] = point.icov_xz;
  *pc_its[6U] = point.icov_yy;
  *pc_its[7U] = point.icov_yz;
  *pc_its[8U] = point.icov_zz;
  std::for_each(pc_its.begin(), pc_its.end(), [](auto & it) {++it;});
}

/// The resulting point cloud has the following fields: x, y, z, cov_xx, cov_xy, cov_xz, cov_yy,
/// cov_yz, cov_zz, cell_id.
/// \param msg_out Reference to the pointcloud message that will store
/// the serialized map data. The message will be initialized before use.
template<>
void DynamicNDTMap::serialize_as<StaticNDTMap>(sensor_msgs::msg::PointCloud2 & msg_out) const
{
  auto dummy_idx{0U};
  common::lidar_utils::reset_pcl_msg(msg_out, 0U, dummy_idx);
  common::lidar_utils::init_pcl_msg(
    msg_out, frame_id(), m_grid.size() + kNumConfigPoints, 9U,
    "x", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "y", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "z", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "icov_xx", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "icov_xy", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "icov_xz", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "icov_yy", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "icov_yz", 1U, sensor_msgs::msg::PointField::FLOAT64,
    "icov_zz", 1U, sensor_msgs::msg::PointField::FLOAT64);

  msg_out.header.stamp = time_utils::to_message(m_stamp);
  auto pc_its = get_iterators(msg_out);

  const auto min_point = m_grid.config().get_min_point();
  const auto max_point = m_grid.config().get_max_point();
  const auto size = m_grid.config().get_voxel_size();

  // Serialize the configuration to be reconstructed
  push_back(pc_its, {min_point.x, min_point.y, min_point.z});
  push_back(pc_its, {max_point.x, max_point.y, max_point.z});
  push_back(pc_its, {size.x, size.y, size.z});

  auto num_used_cells = 0U;
  for (const auto & vx_it : m_grid) {
    if (
      !std::all_of(
        pc_its.begin(), pc_its.end(), [](auto & it) {
          return it != it.end();
        }))
    {
      // This should not occur as the cloud is resized to the map's size.
      throw std::length_error("NDT map is larger than the map point cloud.");
    }
    const auto & vx = vx_it.second;
    if (!vx.usable()) {
      // Voxel doesn't have enough points to be used in NDT
      continue;
    }

    const auto inv_covariance_opt = vx.inverse_covariance();
    if (!inv_covariance_opt) {
      // Voxel covariance is not invertible
      continue;
    }

    const auto & centroid = vx.centroid();
    const auto & inv_covariance = inv_covariance_opt.value();
    push_back(
      pc_its, {
            centroid(0U), centroid(1U), centroid(2U),
            inv_covariance(0U, 0U), inv_covariance(0U, 1U), inv_covariance(0U, 2U),
            inv_covariance(1U, 1U), inv_covariance(1U, 2U),
            inv_covariance(2U, 2U)
          });

    ++num_used_cells;
  }
  // Resize to throw out unused cells.
  common::lidar_utils::resize_pcl_msg(msg_out, num_used_cells + kNumConfigPoints);
}

const DynamicNDTMap::VoxelViewVector & DynamicNDTMap::cell(const Point & pt) const
{
  return m_grid.cell(pt);
}

const DynamicNDTMap::VoxelViewVector & DynamicNDTMap::cell(float32_t x, float32_t y, float32_t z)
const
{
  return cell(Point({x, y, z}));
}

std::size_t DynamicNDTMap::size() const noexcept
{
  return m_grid.size();
}

typename DynamicNDTMap::VoxelGrid::const_iterator DynamicNDTMap::begin() const noexcept
{
  return m_grid.cbegin();
}

typename DynamicNDTMap::VoxelGrid::const_iterator DynamicNDTMap::end() const noexcept
{
  return m_grid.cend();
}

void DynamicNDTMap::clear() noexcept
{
  m_grid.clear();
}

const std::string & StaticNDTMap::frame_id() const noexcept
{
  return m_frame_id;
}

StaticNDTMap::TimePoint StaticNDTMap::stamp() const noexcept
{
  return m_stamp;
}

bool StaticNDTMap::valid() const noexcept
{
  return m_grid && (m_grid->size() > 0U) && (!m_frame_id.empty());
}

const StaticNDTMap::ConfigPoint & StaticNDTMap::cell_size() const
{
  if (!m_grid) {
    throw std::runtime_error("Static ndt map was attempted to be used before a map was set.");
  }
  return m_grid->cell_size();
}

void StaticNDTMap::set(const sensor_msgs::msg::PointCloud2 & msg)
{
  if (m_grid) {
    m_grid->clear();
  }
  deserialize_from(msg);
  m_stamp = ::time_utils::from_message(msg.header.stamp);
  m_frame_id = msg.header.frame_id;
}

void StaticNDTMap::deserialize_from(const sensor_msgs::msg::PointCloud2 & msg)
{
  using PointXYZ = geometry_msgs::msg::Point32;
  constexpr auto num_config_fields = 3U;
  const auto size = validate_pcl_map(msg);
  if (size < num_config_fields) {
    // throwing rather than silently failing since ndt matching cannot be done with an
    // empty/incorrect map
    throw std::runtime_error(
            "Point cloud representing the ndt map is either empty"
            "or does not have the correct format.");
  }
  const auto map_size = size - num_config_fields;

  auto pc_its = get_iterators(msg);

  const auto min_point = next(pc_its);
  const auto max_point = next(pc_its);
  const auto voxel_size = next(pc_its);

  const Config config{
    PointXYZ{}.set__x(min_point.x).set__y(min_point.y).set__z(min_point.z),
    PointXYZ{}.set__x(max_point.x).set__y(max_point.y).set__z(max_point.z),
    PointXYZ{}.set__x(voxel_size.x).set__y(voxel_size.y).set__z(voxel_size.z),
    map_size};

  // Either update the map config or initialize the map.
  if (m_grid) {
    m_grid->set_config(config);
  } else {
    m_grid.emplace(config);
  }

  while (
    std::all_of(
      pc_its.begin(), pc_its.end(), [](auto & it) {
        return it != it.end();
      }))
  {
    const auto voxel_point = next(pc_its);
    const Point centroid{voxel_point.x, voxel_point.y, voxel_point.z};
    const auto voxel_idx = m_grid->index(centroid);

    Eigen::Matrix3d inv_covariance;
    inv_covariance <<
      voxel_point.icov_xx, voxel_point.icov_xy, voxel_point.icov_xz,
      voxel_point.icov_xy, voxel_point.icov_yy, voxel_point.icov_yz,
      voxel_point.icov_xz, voxel_point.icov_yz, voxel_point.icov_zz;
    const Voxel vx{centroid, inv_covariance};

    const auto insert_res = m_grid->emplace_voxel(voxel_idx, Voxel{centroid, inv_covariance});
    if (!insert_res.second) {
      // if a voxel already exist at this point, replace.
      insert_res.first->second = vx;
    }
  }
}
const StaticNDTMap::VoxelViewVector & StaticNDTMap::cell(const Point & pt) const
{
  if (!m_grid) {
    throw std::runtime_error("Static ndt map was attempted to be used before a map was set.");
  }
  return m_grid->cell(pt);
}

const StaticNDTMap::VoxelViewVector & StaticNDTMap::cell(float32_t x, float32_t y, float32_t z)
const
{
  return cell(Point({x, y, z}));
}

std::size_t StaticNDTMap::size() const
{
  if (!m_grid) {
    throw std::runtime_error("Static ndt map was attempted to be used before a map was set.");
  }
  return m_grid->size();
}

typename StaticNDTMap::VoxelGrid::const_iterator StaticNDTMap::begin() const
{
  if (!m_grid) {
    throw std::runtime_error("Static ndt map was attempted to be used before a map was set.");
  }
  return m_grid->cbegin();
}

typename StaticNDTMap::VoxelGrid::const_iterator StaticNDTMap::end() const
{
  if (!m_grid) {
    throw std::runtime_error("Static ndt map was attempted to be used before a map was set.");
  }
  return m_grid->cend();
}

void StaticNDTMap::clear()
{
  if (!m_grid) {
    throw std::runtime_error("Static ndt map was attempted to be used before a map was set.");
  }
  m_grid->clear();
}
}  // namespace ndt
}  // namespace localization
}  // namespace autoware
