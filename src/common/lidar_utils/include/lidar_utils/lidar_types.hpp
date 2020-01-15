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
/// \brief This file defines common types and definitions, specifically for the VLP16

#ifndef LIDAR_UTILS__LIDAR_TYPES_HPP_
#define LIDAR_UTILS__LIDAR_TYPES_HPP_

#include <cstdint>
#include <vector>

#include "lidar_utils/visibility_control.hpp"

namespace autoware
{
namespace common
{
namespace lidar_utils
{
/// pi = tau / 2
constexpr float PI = 3.14159265359F;
/// pi/2
constexpr float PI_2 = 1.5707963267948966F;
/// tau = 2 pi
constexpr float TAU = 6.283185307179586476925286766559F;
/// arbitrary small constant: 1.0E-6F
constexpr float FEPS = 0.000001F;

struct LIDAR_UTILS_PUBLIC PointXYZIF
{
  float x, y, z, intensity;
  uint16_t id;
  static constexpr uint16_t END_OF_SCAN_ID = 65535u;
};

using PointBlock = std::vector<PointXYZIF>;
/// \brief Stores basic configuration information, does some simple validity checking
static constexpr uint16_t POINT_BLOCK_CAPACITY = 512U;

}  // namespace lidar_utils
}  // namespace common
}  // namespace autoware

#endif  // LIDAR_UTILS__LIDAR_TYPES_HPP_
