// Copyright 2018-2020 Apex.AI, Inc.
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

/// \copyright Copyright 2018-2020 Apex.AI, Inc.
/// \file
/// \brief This file defines a driver for Velodyne LiDARs

#ifndef VELODYNE_DRIVER__VLP16_DATA_HPP_
#define VELODYNE_DRIVER__VLP16_DATA_HPP_

#include <velodyne_driver/visibility_control.hpp>
#include <velodyne_driver/common.hpp>
#include <cmath>

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

/// Class implementing VLP16 specific computation and caches
class VELODYNE_DRIVER_PUBLIC VLP16Data
{
public:
  /// full (16 point) fire sequence takes this long to cycle
  static constexpr float32_t FIRE_SEQ_OFFSET_US = 55.296F;
  /// one laser fires for this long
  static constexpr float32_t FIRE_DURATION_US = 2.304F;
  static constexpr uint16_t NUM_LASERS{16U};
  static constexpr float32_t NUM_SEQUENCES_PER_BLOCK{NUM_POINTS_PER_BLOCK / NUM_LASERS};
  static constexpr float32_t DISTANCE_RESOLUTION{0.002f};

  explicit VLP16Data(const float32_t rpm);

  /// Get the azimuth offset for a given point in the given block.
  /// \param block_id Block ID within the packet.
  /// \param pt_id Point ID within the block.
  /// \return Azimuth offset for the given laser.
  uint32_t azimuth_offset(uint32_t block_id, uint32_t pt_id) const;

  /// Get the altitude angle for a given point in the given block.
  /// \param block_id Block ID within the packet.
  /// \param pt_id Point ID within the block.
  /// \return Altitude angle for the given laser.
  uint32_t altitude(uint32_t block_id, uint32_t pt_id) const;

  /// Get total number of firing sequences in the number of blocks + number of points.
  /// \param num_blocks Total number of blocks.
  /// \param pt_id Residue number of points
  /// \return Total number of sequences (floor) in the range pointed by the given indices.
  uint16_t seq_id(uint16_t num_blocks, uint32_t pt_id) const noexcept;

  /// Get number of blocks generated in a single revolution.
  uint16_t num_blocks_per_revolution() const noexcept;

  /// Get distance resolution of VLP16
  static constexpr float32_t distance_resolution() noexcept
  {
    return DISTANCE_RESOLUTION;
  }

private:
  /// \brief precomputes the number of points in a complete rotation and the azimuth offset for
  ///        each firing in a block
  /// \param[in] rpm the LiDAR's spin rate in revolutions per minute
  /// \return none
  VELODYNE_DRIVER_LOCAL void init_azimuth_table(const float32_t rpm);

  /// \brief initializes the fixed altitude angles for each firing in a block. This only needs to
  ///        run once and happens in the constructor
  /// \return none
  VELODYNE_DRIVER_LOCAL void init_altitude_table();

  /// lookup table for azimuth offset for each point index in a block
  std::array<uint32_t, NUM_POINTS_PER_BLOCK> m_azimuth_ind;
  /// lookup table for altitude angle for each firing in a fire sequence (2 per block)
  std::array<uint32_t, NUM_LASERS> m_altitude_ind;

  uint16_t m_num_blocks_per_revolution;
};
}  // namespace velodyne_driver
}  // namespace drivers
}  // namespace autoware

#endif  // VELODYNE_DRIVER__VLP16_DATA_HPP_
