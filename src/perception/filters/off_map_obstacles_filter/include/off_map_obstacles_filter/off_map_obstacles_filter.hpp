// Copyright 2021 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the off_map_obstacles_filter class.

#ifndef OFF_MAP_OBSTACLES_FILTER__OFF_MAP_OBSTACLES_FILTER_HPP_
#define OFF_MAP_OBSTACLES_FILTER__OFF_MAP_OBSTACLES_FILTER_HPP_

#include <memory>

#include "autoware_auto_msgs/msg/bounding_box_array.hpp"
#include "common/types.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "off_map_obstacles_filter/visibility_control.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autoware
{
/// \brief A namespace for the temporary off-map obstacles filter.
namespace off_map_obstacles_filter
{

using float64_t = autoware::common::types::float64_t;

/// \brief Class to filter out bounding boxes that are not on the map.
class OFF_MAP_OBSTACLES_FILTER_PUBLIC OffMapObstaclesFilter
{
public:
  /// \brief Constructor.
  /// \param map The lanelet map, correctly transformed into the map frame.
  /// \param overlap_threshold What fraction of a bbox needs to overlap the map to be considered
  /// "on the map".
  OffMapObstaclesFilter(std::shared_ptr<lanelet::LaneletMap> map, float64_t overlap_threshold);

  /// \brief A function for debugging the transformation and conversion of boxes in the base_link
  /// frame to lanelet polygons in the map frame.
  /// \param map_from_base_link The transform that transforms things from base_link to map.
  /// \param msg The bounding boxes to visualize.
  /// \return A marker array of linestrings, one for each bbox.
  visualization_msgs::msg::MarkerArray bboxes_in_map_frame_viz(
    const geometry_msgs::msg::TransformStamped & map_from_base_link,
    const autoware_auto_msgs::msg::BoundingBoxArray & msg) const;

  /// \param map_from_base_link The transform that transforms things from base_link to map.
  /// \param msg The bounding boxes array to filter – will be modified.
  /// \return True if the bbox overlaps any map element.
  /// This function assumes that bboxes are 2.5d, i.e. only have yaw but no roll or pitch.
  void remove_off_map_bboxes(
    const geometry_msgs::msg::TransformStamped & map_from_base_link,
    autoware_auto_msgs::msg::BoundingBoxArray & msg) const;

private:
  /// The full lanelet map.
  const std::shared_ptr<lanelet::LaneletMap> m_map;
  /// What fraction of a bbox needs to overlap the map to be considered "on the map".
  /// Note that the default value will always be overwritten by the constructor, it's just here to
  /// be safe.
  const float64_t m_overlap_threshold {1.0};
};

}  // namespace off_map_obstacles_filter
}  // namespace autoware

#endif  // OFF_MAP_OBSTACLES_FILTER__OFF_MAP_OBSTACLES_FILTER_HPP_
