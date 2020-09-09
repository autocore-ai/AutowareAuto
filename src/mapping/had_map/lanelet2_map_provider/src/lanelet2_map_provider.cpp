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

#include <GeographicLib/Geocentric.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <common/types.hpp>
#include <string>

#include "had_map_utils/had_map_utils.hpp"

using autoware::common::types::float64_t;

namespace autoware
{

namespace lanelet2_map_provider
{

Lanelet2MapProvider::Lanelet2MapProvider(const std::string & map_filename)
: m_map_filename(map_filename)
{
}

void Lanelet2MapProvider::calculate_geographic_coords()
{
  // need to convert earth to map transform back to lat lon for lanelet2 projector
  GeographicLib::Geocentric earth(
    GeographicLib::Constants::WGS84_a(),
    GeographicLib::Constants::WGS84_f());

  earth.Reverse(
    m_earth_to_map.transform.translation.x,
    m_earth_to_map.transform.translation.y,
    m_earth_to_map.transform.translation.z,
    m_origin_lat, m_origin_lon, m_origin_ele);
}
void Lanelet2MapProvider::load_map()
{
  lanelet::ErrorMessages errors;
  lanelet::GPSPoint originGps{m_origin_lat, m_origin_lon, m_origin_ele};
  lanelet::Origin origin{originGps};

  lanelet::projection::UtmProjector projector(origin);
  m_map = lanelet::load(m_map_filename, projector, &errors);
  autoware::common::had_map_utils::overwriteLaneletsCenterline(m_map, true);
}

}  // namespace lanelet2_map_provider

}  // namespace autoware
