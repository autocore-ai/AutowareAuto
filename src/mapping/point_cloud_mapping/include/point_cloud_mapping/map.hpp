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

#ifndef POINT_CLOUD_MAPPING__MAP_HPP_
#define POINT_CLOUD_MAPPING__MAP_HPP_

#include <point_cloud_mapping/visibility_control.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <voxel_grid/voxel_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <common/types.hpp>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <string>
#include <unordered_map>

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

/// Enum defining if the stored map is uniquely owned by the MapRepresentation
/// implementation or is shared with the localizer.
enum class MapStorageMode
{
  Independent,
  Shared
};

/// Struct containing information on the attempt to push a new observation to a map.
struct POINT_CLOUD_MAPPING_PUBLIC MapUpdateSummary
{
  MapUpdateType update_type;
  std::size_t num_added_pts;
};
using common::types::float32_t;

/// Virtual base class of a map to be used for mapping. The map should provide the
/// following functionality: observe new data, extend itself and export itself.
/// \tparam IncrementT Type of increment that is used to extend the map.
template<typename IncrementT, MapStorageMode StorageModeT>
class POINT_CLOUD_MAPPING_PUBLIC MapRepresentationBase
{
public:
  using Increment = IncrementT;
/// Try to extend the map with the given observation.
/// \param observation Observation to add to the map.
/// \param pose Pose associated with the observation.
/// \return A struct summarizing the outcome of the insertion attempt.
  virtual MapUpdateSummary try_add_observation(
    const IncrementT & observation,
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose) = 0;

  /// Export the map and write it to a file.
  /// \param file_name_prefix File name prefix before the filename postfix is added.
  virtual void write(const std::string & file_name_prefix) const = 0;

  /// Return size of the map. This is implementation defined.
  /// \return Size of the map.
  virtual std::size_t size() const noexcept = 0;

  /// Get the capacity of the map. The meaning is implementation defined.
  /// \return Capacity of the map.
  virtual std::size_t capacity() const noexcept = 0;

  /// Get storage mode of the map representation. If MapStorageMode::independent,
  /// Then this class is the unique owner of the map data. Otherwise the map is shared
  /// with an external entity.
  static constexpr MapStorageMode storage_mode() noexcept
  {
    return StorageModeT;
  }

  virtual ~MapRepresentationBase() = default;

  /// Clear the map.
  virtual void clear() = 0;
};

/// A map that accumulates raw lidar scans and uses a raw point cloud to store the map.
/// A pcl pointcloud is used as the underlying data storage to avoid the cost to convert
/// while writing.
class POINT_CLOUD_MAPPING_PUBLIC PlainPointCloudMap
  : public MapRepresentationBase<sensor_msgs::msg::PointCloud2, MapStorageMode::Independent>
{
public:
  using Base = MapRepresentationBase<sensor_msgs::msg::PointCloud2,
      MapStorageMode::Independent>;
  static constexpr auto NUM_FIELDS{4U};
  // pcl cloud is used to seamlessly write the file without needing to copy/allocate.
  using PCLCloud = pcl::PointCloud<pcl::PointXYZI>;
  using Cloud = sensor_msgs::msg::PointCloud2;
  using CloudIt = sensor_msgs::PointCloud2Iterator<float32_t>;
  using CloudConstIt = sensor_msgs::PointCloud2ConstIterator<float32_t>;

  /// Constructor
  /// \param capacity Capacity of the map.
  /// \param frame Frame id of the map.
  explicit PlainPointCloudMap(std::size_t capacity, const std::string & frame = "map");

  /// Try to extend the map with the given point cloud.
  /// \param observation Point cloud in the "map" frame to add to the map.
  /// \return A struct summarizing the outcome of the insertion attempt.
  MapUpdateSummary try_add_observation(
    const Cloud & observation,
    const geometry_msgs::msg::PoseWithCovarianceStamped &) override;

  /// Write the point cloud map to a `.pcd` file.
  /// \param file_name_prefix File name prefix of the file.
  void write(const std::string & file_name_prefix) const override;
  std::size_t size() const noexcept override;
  std::size_t capacity() const noexcept override;
  void clear() override;

private:
  PCLCloud m_cloud;
  std::size_t m_capacity;
  std::string m_frame_id;
};

/// A map that accumulates lidar scans in a downsampled format using a voxel grid.
/// A voxel grid is used
class POINT_CLOUD_MAPPING_PUBLIC VoxelMap
  : public MapRepresentationBase<sensor_msgs::msg::PointCloud2, MapStorageMode::Independent>
{
public:
  using Base = MapRepresentationBase<sensor_msgs::msg::PointCloud2,
      MapStorageMode::Independent>;
  static constexpr auto NUM_FIELDS{4U};
  using PCLCloud = pcl::PointCloud<pcl::PointXYZI>;
  using Cloud = sensor_msgs::msg::PointCloud2;
  using CloudIt = sensor_msgs::PointCloud2Iterator<float32_t>;
  using CloudConstIt = sensor_msgs::PointCloud2ConstIterator<float32_t>;

  /// Constructor
  /// \param grid_config Grid configuration of the underlying voxel grid.
  /// \param frame Frame id of the map.
  explicit VoxelMap(
    const perception::filters::voxel_grid::Config & grid_config,
    const std::string & frame = "map");

  /// Try to extend the map with the given point cloud.
  /// \param observation Point cloud in the "map" frame to add to the map.
  /// \return A struct summarizing the outcome of the insertion attempt.
  MapUpdateSummary try_add_observation(
    const Cloud & observation,
    const geometry_msgs::msg::PoseWithCovarianceStamped &) override;
  /// Convert the voxel grid to a point cloud and write it to a pcd file.
  /// \param file_name_prefix File name prefix of the file.
  void write(const std::string & file_name_prefix) const override;
  /// Size of the voxel grid.
  std::size_t size() const noexcept override;
  /// Capacity of the voxel grid.
  std::size_t capacity() const noexcept override;
  /// Clear the voxel grid.
  void clear() override;

private:
  perception::filters::voxel_grid::Config m_grid_config;
  std::unordered_map<uint64_t,
    perception::filters::voxel_grid::CentroidVoxel<common::types::PointXYZIF>> m_grid;
  std::string m_frame_id;
};


}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware

#endif  // POINT_CLOUD_MAPPING__MAP_HPP_
