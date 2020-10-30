// Copyright 2018-2020 the Autoware Foundation
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

/// \copyright Copyright 2020 the Autoware Foundation
/// \file
/// \brief This file defines a driver for Velodyne LiDARs

#ifndef VELODYNE_DRIVER__COMMON_HPP_
#define VELODYNE_DRIVER__COMMON_HPP_

#include <velodyne_driver/visibility_control.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <common/types.hpp>

namespace autoware
{
/// \brief Libraries, ROS nodes, and other functionality relating to
///         sensor drivers or actuation.
namespace drivers
{
/// \brief Classes, types, and definitions specifically relating to
///        Velodyne LiDARs. In it's current incarnation, we consider Velodyne to be synonymous
///        with LiDARs. In the future, this namespace will diverge to LiDAR and Velodyne for
///        general LiDAR point cloud functionality, and specific driver functionality for
///        velodne LiDARs respectively.
namespace velodyne_driver
{
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

// These constants seem to be shared among velodyne drivers. If a model with different specs is
// to be supported, they should be refactored.

/// resolution of azimuth angle: number of points in a full rotation
static constexpr uint32_t AZIMUTH_ROTATION_RESOLUTION = 36000U;
/// conversion from a degree (vlp) to idx
static constexpr float32_t DEG2IDX = static_cast<float32_t>(AZIMUTH_ROTATION_RESOLUTION) / 360.0F;
/// how intensity is quantized: 1 byte = 256 possible values
static constexpr uint32_t NUM_INTENSITY_VALUES = 256U;

/// number of data blocks per data packet
static constexpr uint16_t NUM_BLOCKS_PER_PACKET = 12U;
/// number of points stored in a data block
static constexpr uint16_t NUM_POINTS_PER_BLOCK = 32U;

/// \brief computes 2 byte representation of two bytes from out of order velodyne packet
inline uint32_t to_uint32(const uint8_t first, const uint8_t second)
{
  // probably ok since uint8_t<<8 =>uint32_t, this is to get around
  // warning due to implicit promotion to int
  const uint32_t ret = static_cast<uint32_t>(first) << 8U;
  return ret + static_cast<uint32_t>(second);
}

using autoware::common::types::PointXYZIF;
}  // namespace velodyne_driver
}  // namespace drivers
}  // namespace autoware

#endif  // VELODYNE_DRIVER__COMMON_HPP_
