// Copyright 2020 the Autoware Foundation
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

#ifndef POINT_CLOUD_MAPPING__POINT_CLOUD_MAP_HPP_
#define POINT_CLOUD_MAPPING__POINT_CLOUD_MAP_HPP_

#include <point_cloud_mapping/visibility_control.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <voxel_grid/voxel_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <helper_functions/crtp.hpp>
#include <helper_functions/template_utils.hpp>
#include <common/types.hpp>
#include <time_utils/time_utils.hpp>
#pragma GCC diagnostic push
// silence unsafe signed <-> unsigned conversion
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wuseless-cast"
#include <pcl/io/pcd_io.h>
#pragma GCC diagnostic pop
#include <vector>
#include <string>
#include <unordered_map>
#include <utility>

namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping
{

/// Enum representing the type of update the observation has caused in the map.
enum class MapUpdateType
{
  NEW,
  UPDATE,
  PARTIAL_UPDATE,
  NO_CHANGE
};

/// Struct containing information on the attempt to push a new observation to a map.
struct POINT_CLOUD_MAPPING_PUBLIC MapUpdateSummary
{
  MapUpdateType update_type;
  std::size_t num_added_pts;
};
using common::types::float32_t;

enum class Requires {};

/// \brief This class encapsulates the static assertions that express the interface requirements
/// of a localizer map to be able to be used with `DualVoxelMap`.
/// \tparam LocalizerMapT Map type used by the localization algorithm.
template<typename LocalizerMapT>
struct LocalizationMapConstraint
{
  using Cloud = sensor_msgs::msg::PointCloud2;

  /// \brief This expression requires a method that inserts a given pointcloud2 message into the
  /// map.
  /// \param[in] msg Pointcloud2 message instance containing the map data.
  template<typename Map>
  using call_insert = decltype(std::declval<Map>().insert(std::declval<const Cloud &>()));

  /// \brief This expression requires a method that clears the map.
  template<typename Map>
  using call_clear = decltype(std::declval<Map>().clear());

  static_assert(
    ::autoware::common::helper_functions::expression_valid<call_insert, LocalizerMapT>::value,
    "The map should provide a `insert(msg)` method to be used for mapping");
  static_assert(
    ::autoware::common::helper_functions::expression_valid<call_clear, LocalizerMapT>::value,
    "The map should provide a `clear()` method to be used for mapping");

  static constexpr Requires value{};
};

/// A map that accumulates lidar scans in a downsampled format using a voxel grid.
/// A voxel grid is used for accumulating the lidar scans in a downsampled manner. A separate
/// map is stored for the localizer implementation. The expected interface is defined via the
/// `Requires` keyword.
template<typename LocalizerMapT, Requires = LocalizationMapConstraint<LocalizerMapT>::value>
class POINT_CLOUD_MAPPING_PUBLIC DualVoxelMap
{
public:
  static constexpr auto NUM_FIELDS{4U};
  using Cloud = sensor_msgs::msg::PointCloud2;

  /// Constructor
  /// \param grid_config Grid configuration of the underlying voxel grid.
  /// \param frame_id Frame id of the map.
  /// \param localizer_map Localizer map to be stored.
  explicit DualVoxelMap(
    const perception::filters::voxel_grid::Config & grid_config,
    const std::string & frame_id,
    LocalizerMapT && localizer_map
  )
  : m_grid_config{grid_config}, m_frame_id{frame_id},
    m_localizer_map{std::forward<LocalizerMapT>(localizer_map)} {}

  /// Try to extend the map with the given point cloud.
  /// \param observation Point cloud in the "map" frame to add to the map.
  /// \return A struct summarizing the outcome of the insertion attempt.
  MapUpdateSummary update(const Cloud & observation)
  {
    if (observation.header.frame_id != m_frame_id) {
      throw std::runtime_error("pointcloud map and the updates should be on the same frame");
    }

    MapUpdateSummary ret{MapUpdateType::NO_CHANGE, 0U};

    using PointXYZI = autoware::common::types::PointXYZI;
    point_cloud_msg_wrapper::PointCloud2View<PointXYZI> observation_view{observation};

    ret.update_type = m_grid.empty() ? MapUpdateType::NEW : MapUpdateType::UPDATE;
    auto obs_idx = 0U;
    for (const auto & pt : observation_view) {
      const auto pt_key = m_grid_config.index(pt);
      if (m_grid.size() >= capacity() &&
        m_grid.find(pt_key) == m_grid.end())
      {
        if (obs_idx == 0U) {
          ret.update_type = MapUpdateType::NO_CHANGE;
        } else {
          ret.update_type = MapUpdateType::PARTIAL_UPDATE;
        }
        break;
      }
      m_grid[pt_key].add_observation(pt);
      ++obs_idx;
    }

    m_localizer_map.insert(observation);
    ret.num_added_pts = obs_idx;
    return ret;
  }

  /// Convert the voxel grid to a point cloud and write it to a pcd file.
  /// \param file_name_prefix File name prefix of the file.
  void write(const std::string & file_name_prefix) const
  {
    // TODO(yunus.caliskan) Remove dynamic allocations.
    // pcl cloud is constructed here and the map is copied to it for sake
    // of simplicity to be able to use the pcl library's pcd writer.
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(m_grid.size());
    for (const auto & vx : m_grid) {
      pcl::PointXYZI pt;
      const auto & vx_pt = vx.second.get();
      pt.x = vx_pt.x;
      pt.y = vx_pt.y;
      pt.z = vx_pt.z;
      pt.intensity = vx_pt.intensity;
      cloud.push_back(pt);
    }
    pcl::io::savePCDFile(file_name_prefix + ".pcd", cloud);
  }

  /// Size of the voxel grid.
  std::size_t size() const noexcept
  {
    return m_grid.size();
  }
  /// Capacity of the voxel grid.
  std::size_t capacity() const noexcept
  {
    return m_grid_config.get_capacity();
  }
  /// Clear the voxel grid.
  void clear()
  {
    m_grid.clear();
    m_localizer_map.clear();
  }
  /// Get the localizer map
  const LocalizerMapT & localizer_map() const noexcept
  {
    return m_localizer_map;
  }
  /// Get the frame ID of the map.
  const std::string & frame_id() const noexcept
  {
    return m_frame_id;
  }

  /// Get if the map is empty
  bool empty()
  {
    return m_grid.empty();
  }

private:
  perception::filters::voxel_grid::Config m_grid_config;
  std::unordered_map<uint64_t,
    perception::filters::voxel_grid::CentroidVoxel<common::types::PointXYZI>> m_grid;
  std::string m_frame_id;
  LocalizerMapT m_localizer_map;
};
}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware

#endif  // POINT_CLOUD_MAPPING__POINT_CLOUD_MAP_HPP_
