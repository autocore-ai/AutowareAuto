/// \copyright Copyright 2017-2018 Apex.AI, Inc.
/// All rights reserved.
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
