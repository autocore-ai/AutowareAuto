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

/// \copyright Copyright 2017-2018 the Autoware Foundation
/// \file
/// \brief This file defines a driver for Velodyne LiDARs

#ifndef VELODYNE_DRIVER__VELODYNE_TRANSLATOR_HPP_
#define VELODYNE_DRIVER__VELODYNE_TRANSLATOR_HPP_

#include <velodyne_driver/visibility_control.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <velodyne_driver/common.hpp>
#include <velodyne_driver/vlp16_data.hpp>
#include <velodyne_driver/vlp32c_data.hpp>
#include <velodyne_driver/vls128_data.hpp>
#include <vector>

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
/// \brief This class handles converting packets from a velodyne lidar into cartesian points.
/// \tparam SensorData Data class representing a specific sensor model.
template<typename SensorData>
class VELODYNE_DRIVER_PUBLIC VelodyneTranslator
{
public:
  /// \brief Stores basic configuration information, does some simple validity checking
  static constexpr uint16_t POINT_BLOCK_CAPACITY = 512U;

  class Config
  {
public:
    /// \brief Constructor
    /// \param[in] rpm rotation speed of the velodyne, determines how many points per scan
    explicit Config(const float32_t rpm)
    : m_rpm(rpm)
    {
    }
    /// \brief Gets rpm value
    /// \return rpm
    float32_t get_rpm() const
    {
      return m_rpm;
    }

private:
    /// rotation speed of the velodyne, determines how many points per scan
    float32_t m_rpm;
  };
  /// \brief corresponds to an individual laser's firing and return
  /// First two bytes are distance, last byte is intensity
  struct DataChannel
  {
    uint8_t data[3U];
  };

  /// \brief corresponds to a velodyne data block.
  struct DataBlock
  {
    uint8_t flag[2U];
    uint8_t azimuth_bytes[2U];
    DataChannel channels[NUM_POINTS_PER_BLOCK];
  };

  /// \brief stores a Velodyne data packet
  struct Packet
  {
    DataBlock blocks[NUM_BLOCKS_PER_PACKET];
    uint8_t timestamp_bytes[4U];
    uint8_t factory_bytes[2U];
  };

  /// \brief default constructor
  /// \param[in] config config struct with rpm, transform, radial and angle pruning params
  /// \throw std::runtime_error if pruning parameters are inconsistent
  explicit VelodyneTranslator(const Config & config)
  : m_sensor_data(config.get_rpm())
  {
    init_trig_tables();
    init_intensity_table();
  }

  /// \brief Convert a packet into a block of cartesian points
  /// \param[in] pkt A packet from a VLP16 HiRes sensor for conversion
  /// \param[out] output Gets filled with cartesian points and any additional flags
  void convert(const Packet & pkt, std::vector<autoware::common::types::PointXYZIF> & output)
  {
    output.clear();

    for (uint32_t block_id = 0U; block_id < NUM_BLOCKS_PER_PACKET; ++block_id, ++m_block_counter) {
      const DataBlock & block = pkt.blocks[block_id];
      const auto flag_check_result = m_sensor_data.check_flag(block.flag);
      // Ignore block with invalid flag.
      if (!flag_check_result.first) {
        continue;
      }
      // Number of points from the sequence that has already been delivered in previous blocks.
      const auto num_banked_pts = flag_check_result.second;
      const uint32_t azimuth_base = to_uint32(block.azimuth_bytes[1U], block.azimuth_bytes[0U]);

      for (uint16_t pt_id = 0U; pt_id < NUM_POINTS_PER_BLOCK; ++pt_id) {
        const DataChannel & channel = block.channels[pt_id];
        const uint32_t th = (azimuth_base + m_sensor_data.azimuth_offset(
            num_banked_pts, block_id, pt_id)) % AZIMUTH_ROTATION_RESOLUTION;
        const float32_t r = compute_distance_m(channel.data[1U], channel.data[0U]);
        const uint32_t phi = m_sensor_data.altitude(num_banked_pts, block_id, pt_id);

        // Compute the point
        PointXYZIF pt;
        polar_to_xyz(pt, r, th, phi);
        pt.intensity = m_intensity_table[channel.data[2U]];
        pt.id = m_sensor_data.seq_id(m_block_counter, pt_id);

        output.push_back(pt);
      }

      if (static_cast<float32_t>(m_block_counter) > m_sensor_data.num_blocks_per_revolution()) {
        // full revolution reached.
        PointXYZIF pt;
        pt.id =
          static_cast<uint16_t>(PointXYZIF::END_OF_SCAN_ID);
        output.push_back(pt);
        m_block_counter = uint16_t{0U};
      }
    }
  }

private:
  // make sure packet sizes are correct
  static_assert(sizeof(DataChannel) == 3U, "Error velodyne data channel size is incorrect");
  static_assert(sizeof(DataBlock) == 100U, "Error velodyne data block size is incorrect");
  static_assert(sizeof(Packet) == 1206U, "Error velodyne packet size is incorrect");
  // Ensure that a full packet will fit into a point block
  static_assert(
    static_cast<uint32_t>(POINT_BLOCK_CAPACITY) >=
    ((NUM_POINTS_PER_BLOCK * NUM_BLOCKS_PER_PACKET) + 1U),
    "Number of points from one VLP16 packet cannot fit into a point block");

  /// \brief converts polar coordinates into cartesian (xyz), not threadsafe: modifies a
  ///        preallocated workspace member variable (m_point)
  /// \param[out] pt Gets filled with the requisite cartesian information
  /// \param[in] r_m the radius in meters
  /// \param[in] th_ind the index of the azimuth in lookup tables (angle about the z-axis)
  /// \param[in] phi_ind the altitude angle index from lookup tables(angle orthogonal to z-axis)
  inline void polar_to_xyz(
    autoware::common::types::PointXYZIF & pt,
    const float32_t r_m,
    const uint32_t th_ind,
    const uint32_t phi_ind) const
  {
    const float32_t r_xy = r_m * m_cos_table[phi_ind];
    pt.x = r_xy * m_cos_table[th_ind];  // y (vlp-frame)
    pt.y = -r_xy * m_sin_table[th_ind];  // -x (vlp-frame)
    pt.z = r_m * m_sin_table[phi_ind];
  }

  /// \brief converts the two byte representation of distance into meters
  /// \param[in] first the byte representing the most significant bits
  /// \param[in] second the byte representing the least significant bits
  /// \return the radial distance in meters
  inline float32_t compute_distance_m(const uint8_t first, const uint8_t second) const
  {
    const uint32_t dist_scaled = to_uint32(first, second);
    return static_cast<float32_t>(dist_scaled) * m_sensor_data.distance_resolution();
  }

  template<typename T>
  inline T clamp(const T val, const T min, const T max) const
  {
    return (val < min) ? min : ((val > max) ? max : val);
  }

  /// \brief initializes sin and cosine lookup tables
  VELODYNE_DRIVER_LOCAL void init_trig_tables()
  {
    constexpr float32_t IDX2RAD =
      TAU / static_cast<float32_t>(AZIMUTH_ROTATION_RESOLUTION);
    for (uint64_t idx = 0U; idx < AZIMUTH_ROTATION_RESOLUTION; ++idx) {
      m_cos_table[idx] = cosf((static_cast<float32_t>(idx)) * IDX2RAD);
      m_sin_table[idx] = sinf((static_cast<float32_t>(idx)) * IDX2RAD);
    }
  }

  /// \brief initializes intensity lookup table
  VELODYNE_DRIVER_LOCAL void init_intensity_table()
  {
    for (uint64_t idx = 0U; idx < NUM_INTENSITY_VALUES; ++idx) {
      m_intensity_table[idx] = static_cast<float32_t>(idx);
    }
  }

  /// tau = 2 pi
  static constexpr float32_t TAU = 6.283185307179586476925286766559F;

  /// parameters
  /// lookup table for sin
  std::array<float32_t, AZIMUTH_ROTATION_RESOLUTION> m_sin_table;
  /// lookup table for cos
  std::array<float32_t, AZIMUTH_ROTATION_RESOLUTION> m_cos_table;
  /// lookup table for intensity
  std::array<float32_t, AZIMUTH_ROTATION_RESOLUTION> m_intensity_table;

  /// mask to avoid modulo: packet id can go up to 3617: 0000 1111 1111 1111 = 4096
  uint16_t m_block_counter{0U};
  SensorData m_sensor_data;
};  // class Driver
using Vlp16Translator = VelodyneTranslator<VLP16Data>;
using Vlp32CTranslator = VelodyneTranslator<VLP32CData>;
using Vls128Translator = VelodyneTranslator<VLS128Data>;

}  // namespace velodyne_driver
}  // namespace drivers
}  // namespace autoware

#endif  // VELODYNE_DRIVER__VELODYNE_TRANSLATOR_HPP_
