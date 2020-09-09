// Copyright 2020 The Autoware Foundation
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

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the lanelet2_map_provider class.

#ifndef LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_HPP_
#define LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_HPP_

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <common/types.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_map_provider/visibility_control.hpp>

#include <iostream>
#include <string>
#include <memory>

#include "autoware_auto_msgs/msg/had_map_bin.hpp"

using autoware::common::types::float64_t;

namespace autoware
{
/// \brief TODO(simon.thompson): Document namespaces!
namespace lanelet2_map_provider
{

/// \class Lanelet2MapProvider
/// \brief Provides functoins to load and access a lanelet2 OSM map.
class LANELET2_MAP_PROVIDER_PUBLIC Lanelet2MapProvider
{
public:
  Lanelet2MapProvider(const std::string & map_filename);
  /// \brief set the transform between earth and map frames for projection of map data
  /// \param stf the earth to map transform
  void set_earth_to_map_transform(const geometry_msgs::msg::TransformStamped & stf)
  {
    m_earth_to_map = stf;
  }
  /// \brief load the lanelet map and project into the coordinates of the origin
  void load_map();
  /// \brief calculate the latitude, longitude and elevation of the map orgin
  /// from the origin transform
  void calculate_geographic_coords();
  /// \brief directly set hte geographic coordinates of the map orgin
  /// \param lat map origin latitude
  /// \param lon map orgin longitude
  /// \param ele map orgin elevation
  void set_geographic_coords(const float64_t lat, const float64_t lon, const float64_t ele)
  {
    m_origin_lat = lat;
    m_origin_lon = lon;
    m_origin_ele = ele;
  }
  std::shared_ptr<lanelet::LaneletMap> m_map;

private:
  std::string m_map_filename;
  // map orgin as a transform from earth center
  geometry_msgs::msg::TransformStamped m_earth_to_map;
  float64_t m_origin_lat;  // map orgin in latitude, longitude and elevation
  float64_t m_origin_lon;
  float64_t m_origin_ele;
};

}  // namespace lanelet2_map_provider

}  // namespace autoware

#endif  // LANELET2_MAP_PROVIDER__LANELET2_MAP_PROVIDER_HPP_
