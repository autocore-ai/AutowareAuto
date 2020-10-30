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

/// \copyright Copyright 2020 the Autoware Foundation
/// \file
/// \brief This file defines a driver for Velodyne LiDARs

#ifndef VELODYNE_DRIVER__VLP32C_DATA_HPP_
#define VELODYNE_DRIVER__VLP32C_DATA_HPP_

#include <velodyne_driver/visibility_control.hpp>
#include <velodyne_driver/common.hpp>
#include <cmath>
#include <utility>

namespace autoware
{
namespace drivers
{
namespace velodyne_driver
{

/// Class implementing vlp32c specific computation and caches.
/// Following manual is used as reference during development:
/// https://icave2.cse.buffalo.edu/resources/sensor-modeling/VLP32CManual.pdf
class VELODYNE_DRIVER_PUBLIC VLP32CData
{
public:
  /// full (32 point) fire sequence takes this long to cycle
  static constexpr float32_t FIRE_SEQ_OFFSET_US = 55.296F;
  /// A group of 2 lasers takes this much time:
  static constexpr float32_t FIRE_DURATION_US = 2.304F;
  static constexpr uint16_t NUM_GROUPS_PER_SEQ{16U};
  static constexpr uint16_t NUM_LASERS{32U};
  static constexpr uint16_t GROUP_SIZE{2U};
  using BlockFlag = uint8_t[2U];

  static_assert(NUM_GROUPS_PER_SEQ * GROUP_SIZE == NUM_LASERS,
    "VLP32C driver has incorrect configuration. "
    "Check the manual for the group size and number of lasers.");

  static constexpr float32_t NUM_SEQUENCES_PER_BLOCK{float32_t(NUM_POINTS_PER_BLOCK) / NUM_LASERS};
  static constexpr float32_t DISTANCE_RESOLUTION{0.004f};

  explicit VLP32CData(const float32_t rpm);

  /// Get the azimuth offset for a given point in the given block.
  /// \param num_banked_pts Number of points from the sequence that were
  /// transferred on previous blocks.
  /// \param block_id Block ID within the packet.
  /// \param pt_id Point ID within the block.
  /// \return Azimuth offset for the given laser.
  uint32_t azimuth_offset(uint16_t num_banked_pts, uint32_t block_id, uint32_t pt_id) const;

  /// Get the altitude angle for a given point in the given block.
  /// \param num_banked_pts Number of points from the sequence that were
  /// transferred on previous blocks.
  /// \param block_id Block ID within the packet.
  /// \param pt_id Point ID within the block.
  /// \return Altitude angle for the given laser.
  uint32_t altitude(uint16_t num_banked_pts, uint32_t block_id, uint32_t pt_id) const;

  /// Get total number of firing sequences in the number of blocks + number of points.
  /// \param num_blocks Total number of blocks.
  /// \param pt_id Residue number of points
  /// \return Total number of sequences (floor) in the range pointed by the given indices.
  uint16_t seq_id(uint16_t num_blocks, uint32_t pt_id) const noexcept;

  /// Get number of blocks generated in a single revolution.
  uint16_t num_blocks_per_revolution() const noexcept;

  /// Get distance resolution of VLP32C
  static constexpr float32_t distance_resolution() noexcept
  {
    return DISTANCE_RESOLUTION;
  }

  /// Check the block flag for validity and return the number of banked points.
  /// \param flag 2 byte flag from the block. Should be one of `0xFFEE` `0xFFDD` `0xFFCC` `0xFFBB`
  /// \return A pair that contains a bool representing the validity of the flag and an integer
  /// containing the number of banked points.
  std::pair<bool8_t, uint16_t> check_flag(const BlockFlag & flag);

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
  std::array<uint32_t, NUM_LASERS> m_azimuth_ind;
  /// lookup table for altitude angle for each firing in a fire sequence (2 per block)
  std::array<uint32_t, NUM_LASERS> m_altitude_ind;

  uint16_t m_num_blocks_per_revolution;
};

}  // namespace velodyne_driver
}  // namespace drivers
}  // namespace autoware

#endif  // VELODYNE_DRIVER__VLP32C_DATA_HPP_
