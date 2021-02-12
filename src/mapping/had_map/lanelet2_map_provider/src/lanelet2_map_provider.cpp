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

#include "lanelet2_map_provider/lanelet2_map_provider.hpp"

#include <string>

#include "common/types.hpp"
#include "had_map_utils/had_map_utils.hpp"

#include "GeographicLib/Geocentric.hpp"
#include "lanelet2_core/primitives/GPSPoint.h"
#include "lanelet2_io/Io.h"
#include "lanelet2_projection/UTM.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using autoware::common::types::float64_t;

namespace autoware
{

namespace lanelet2_map_provider
{

Lanelet2MapProvider::Lanelet2MapProvider(
  const std::string & map_filename,
  const geometry_msgs::msg::TransformStamped & stf, const float64_t offset_lat,
  const float64_t offset_lon)
{
  GeographicLib::Geocentric earth(
    GeographicLib::Constants::WGS84_a(),
    GeographicLib::Constants::WGS84_f());

  float64_t origin_lat = 0.0;
  float64_t origin_lon = 0.0;
  float64_t origin_alt = 0.0;
  earth.Reverse(
    stf.transform.translation.x,
    stf.transform.translation.y,
    stf.transform.translation.z,
    origin_lat, origin_lon, origin_alt);
  this->load_map(map_filename, {origin_lat + offset_lat, origin_lon + offset_lon, origin_alt});
}

Lanelet2MapProvider::Lanelet2MapProvider(
  const std::string & map_filename,
  const LatLonAlt map_frame_origin,
  const float64_t offset_lat, const float64_t offset_lon)
{
  LatLonAlt adjusted_origin{map_frame_origin.lat + offset_lat, map_frame_origin.lon + offset_lon,
    map_frame_origin.alt};
  this->load_map(map_filename, adjusted_origin);
}

void Lanelet2MapProvider::load_map(
  const std::string & map_filename, const LatLonAlt map_frame_origin)
{
  lanelet::ErrorMessages errors;
  lanelet::GPSPoint originGps{map_frame_origin.lat, map_frame_origin.lon, map_frame_origin.alt};
  lanelet::Origin origin{originGps};

  lanelet::projection::UtmProjector projector(origin);
  m_map = lanelet::load(map_filename, projector, &errors);
  autoware::common::had_map_utils::overwriteLaneletsCenterline(m_map, true);
}

}  // namespace lanelet2_map_provider

}  // namespace autoware
