// Copyright 2020 Apex.AI, Inc.
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

#ifndef POINT_CLOUD_MAPPING__MAP_HPP_
#define POINT_CLOUD_MAPPING__MAP_HPP_

#include <point_cloud_mapping/visibility_control.hpp>
#include <localization_common/localizer_base.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <string>

namespace autoware
{
namespace mapping
{
namespace point_cloud_mapping
{

enum class MapUpdateType
{
  NEW,
  UPDATE,
  PARTIAL_UPDATE,
  NO_CHANGE
};

struct POINT_CLOUD_MAPPING_PUBLIC MapUpdateSummary
{
  MapUpdateType update_type;
  std::size_t num_added_pts;
};


template<typename MapMsgT>
class POINT_CLOUD_MAPPING_PUBLIC MapRepresentationBase
{
public:
/// Try to extend the map with the given observation.
/// \param observation Observation to add to the map.
/// \param pose Pose associated with the observation.
/// \return A struct summarizing the outcome of the insertion attempt.
  virtual MapUpdateSummary try_add_observation(
    const MapMsgT & observation,
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose) = 0;

  /// Export the map as the `MapMsgT`
  /// \return Map data.
  virtual const MapMsgT & get() = 0;

  /// Return true if the map is full.
  virtual bool full() = 0;
  /// Clear the map.
  virtual void clear() = 0;
};


class POINT_CLOUD_MAPPING_PUBLIC PlainPointCloudMap
  : public MapRepresentationBase<sensor_msgs::msg::PointCloud2>
{
public:
  static constexpr auto NUM_FIELDS{4U};
  using Cloud = sensor_msgs::msg::PointCloud2;
  using CloudIt = sensor_msgs::PointCloud2Iterator<float_t>;
  using CloudConstIt = sensor_msgs::PointCloud2ConstIterator<float_t>;
  explicit PlainPointCloudMap(std::size_t capacity, const std::string & frame = "map");

  MapUpdateSummary try_add_observation(
    const Cloud & observation,
    const geometry_msgs::msg::PoseWithCovarianceStamped &) override;

  const Cloud & get() override;
  bool full() override;
  void clear() override;

private:
  Cloud m_cloud;
  std::size_t m_capacity;
  std::string m_frame_id;
  std::vector<CloudIt> m_cloud_its;
  sensor_msgs::PointCloud2Modifier m_pc_modifier;
  std::size_t m_map_size{0U};
};

}  // namespace point_cloud_mapping
}  // namespace mapping
}  // namespace autoware

#endif  // POINT_CLOUD_MAPPING__MAP_HPP_
