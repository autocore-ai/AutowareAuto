// Copyright 2018 Apex.AI, Inc.
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

/// \copyright Copyright 2017-2018 Apex.AI, Inc.
/// \file
/// \brief This file defines a driver for Velodyne LiDARs

#ifndef VELODYNE_DRIVER__VLP16_TRANSLATOR_HPP_
#define VELODYNE_DRIVER__VLP16_TRANSLATOR_HPP_

#include <velodyne_driver/visibility_control.hpp>
#include <cstdint>
#include <vector>
#include "geometry_msgs/msg/point32.hpp"

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
struct PointXYZIF
{
  float x, y, z, intensity;
  uint16_t id;
  static constexpr uint16_t END_OF_SCAN_ID = 65535u;
};

inline uint32_t to_uint32(const uint8_t first, const uint8_t second)
{
  // probably ok since uint8_t<<8 =>uint32_t, this is to get around
  // warning due to implicit promotion to int
  const uint32_t ret = static_cast<uint32_t>(first) << 8U;
  return ret + static_cast<uint32_t>(second);
}

inline geometry_msgs::msg::Point32 make_point(const float x, const float y, const float z)
{
  geometry_msgs::msg::Point32 point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

/// \brief This class handles converting packets from a Velodyne VLP16 LiDAR into cartesian points
class VELODYNE_DRIVER_PUBLIC Vlp16Translator
{
public:
  // SENSOR SPECIFIC CONSTANTS
  /// resolution of azimuth angle: number of points in a full rotation
  static constexpr uint32_t AZIMUTH_ROTATION_RESOLUTION = 36000U;
  /// conversion from a degree (vlp) to idx
  static constexpr float DEG2IDX = static_cast<float>(AZIMUTH_ROTATION_RESOLUTION) / 360.0F;
  /// how intensity is quantized: 1 byte = 256 possible values
  static constexpr uint32_t NUM_INTENSITY_VALUES = 256U;

  /// All of these hardcoded values should remain fixed unless the VLP16 packet spec changes ///
  /// number of data blocks per data packet
  static constexpr uint16_t NUM_BLOCKS_PER_PACKET = 12U;
  /// number of points stored in a data block
  static constexpr uint16_t NUM_POINTS_PER_BLOCK = 32U;

  /// full (16 point) fire sequence takes this long to cycle
  static constexpr float FIRE_SEQ_OFFSET_US = 55.296F;
  /// one laser fires for this long
  static constexpr float FIRE_DURATION_US = 2.304F;
  /// rpm min speed
  static constexpr float MIN_RPM = 300.0F;
  /// rpm max speed
  static constexpr float MAX_RPM = 1200.0F;
  /// \brief Stores basic configuration information, does some simple validity checking
  static constexpr uint16_t POINT_BLOCK_CAPACITY = 512U;

  class Config
  {
public:
    /// \brief Constructor
    /// \param[in] rpm rotation speed of the velodyne, determines how many points per scan
    /// \param[in] offset_m Linear offset from the origin of the frame the sensor lives in
    /// \param[in] rotation_rad Rotational offset: z-y-x euler angles in radians
    /// \param[in] min_distance_m Points with distances below this are ignored
    /// \param[in] max_distance_m Points with distances above this are ignored
    /// \param[in] min_angle_deg Points with angles below this are ignored (IN THE SENSOR'S FRAME)
    /// \param[in] max_angle_deg Points with angles above this are ignored (IN THE SENSOR'S FRAME)
    Config(
      const float rpm,
      const geometry_msgs::msg::Point32 offset_m,
      const geometry_msgs::msg::Point32 rotation_rad,
      const float min_distance_m,
      const float max_distance_m,
      const float min_angle_deg,
      const float max_angle_deg);
    /// \brief Gets rpm value
    /// \return rpm
    float get_rpm() const;
    /// \brief Gets translational offset
    /// \return Translation of point frame relative to sensor origin in meters
    const geometry_msgs::msg::Point32 & get_offset() const;
    /// \brief Gets rotational offset
    /// \return z-y-x Euler angles in radians
    const geometry_msgs::msg::Point32 & get_rotation() const;
    /// \brief Gets distance below which points are filtered
    /// \return Minimum point distance in meters
    float get_min_distance() const;
    /// \brief Gets distance above which points are filtered
    /// \return Maximum point distance in meters
    float get_max_distance() const;
    /// \brief Gets angle below which points are filtered
    /// \return Minimum point azimuth angle (in sensor's x-y plane) in degrees
    float get_min_angle() const;
    /// \brief Gets angle above which points are filtered
    /// \return Maximum point azimuth angle (in sensor's x-y plane) in degrees
    float get_max_angle() const;

private:
    /// rotation speed of the velodyne, determines how many points per scan
    float m_rpm;
    /// linear offset from the origin of the frame the sensor lives in
    geometry_msgs::msg::Point32 m_offset_m;
    /// rotational offset: z-y-x euler angles in radians
    geometry_msgs::msg::Point32 m_rotation_rad;
    /// points with distances below this are ignored
    float m_min_distance_m;
    /// points with distances above this are ignored
    float m_max_distance_m;
    /// points with angles below this are ignored (IN THE SENSOR'S FRAME)
    float m_min_angle_deg;
    /// points with angles above this are ignored (IN THE SENSOR'S FRAME)
    float m_max_angle_deg;
  };

  /// \brief corresponds to an individual laser's firing and return
  /// First two bytes are distance, last byte is intensity
  struct DataChannel
  {
    uint8_t data[3U];
  };

  /// \brief corresponds to a vlp16 data block, which represents two full firings
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
  explicit Vlp16Translator(const Config & config);

  /// \brief Convert a packet into a block of cartesian points
  /// \param[in] pkt A packet from a VLP16 HiRes sensor for conversion
  /// \param[out] output Gets filled with cartesian points and any additional flags
  void convert(const Packet & pkt, std::vector<PointXYZIF> & output);

private:
  // make sure packet sizes are correct
  static_assert(sizeof(DataChannel) == 3U, "Error VLP16 data channel size is incorrect");
  static_assert(sizeof(DataBlock) == 100U, "Error VLP16 data block size is incorrect");
  static_assert(sizeof(Packet) == 1206U, "Error VLP16 packet size is incorrect");
  // Ensure that a full packet will fit into a point block
  static_assert(static_cast<uint32_t>(POINT_BLOCK_CAPACITY) >=
    ((NUM_POINTS_PER_BLOCK * NUM_BLOCKS_PER_PACKET) + 1U),
    "Number of points from one VLP16 packet cannot fit into a point block");

  /// \brief converts polar coordinates into cartesian (xyz), not threadsafe: modifies a
  ///        preallocated workspace member variable (m_point)
  /// \param[out] pt Gets filled with the requisite cartesian information
  /// \param[in] r_m the radius in meters
  /// \param[in] th_ind the index of the azimuth in lookup tables (angle about the z-axis)
  /// \param[in] phi_ind the altitude angle index from lookup tables(angle orthogonal to z-axis)
  /// \return none
  inline void polar_to_xyz(
    PointXYZIF & pt,
    const float r_m,
    const uint32_t th_ind,
    const uint32_t phi_ind)
  {
    const float r_xy = r_m * m_cos_table[phi_ind];
    // get nominal x, y in ROS frame (x forward), translate
    const float x_tmp = r_xy * m_cos_table[th_ind];  // y (vlp-frame)
    const float y_tmp = -r_xy * m_sin_table[th_ind];  // -x (vlp-frame)
    const float z_tmp = r_m * m_sin_table[phi_ind];
    // rotate z-y-x, then translate
    // Note for small matrices, naive approach is probably fine or better
    pt.x =
      (m_rot_mat[0U][0U] * x_tmp) + (m_rot_mat[0U][1U] * y_tmp) + (m_rot_mat[0U][2U] * z_tmp) +
      m_offset_m.x;
    pt.y =
      (m_rot_mat[1U][0U] * x_tmp) + (m_rot_mat[1U][1U] * y_tmp) + (m_rot_mat[1U][2U] * z_tmp) +
      m_offset_m.y;
    pt.z =
      (m_rot_mat[2U][0U] * x_tmp) + (m_rot_mat[2U][1U] * y_tmp) + (m_rot_mat[2U][2U] * z_tmp) +
      m_offset_m.z;
  }

  /// \brief converts the two byte representation of distance into meters
  /// \param[in] first the byte representing the most significant bits
  /// \param[in] second the byte representing the least significant bits
  /// \return the radial distance in meters
  inline float compute_distance_m(const uint8_t first, const uint8_t second) const
  {
    const uint32_t dist_2mm = to_uint32(first, second);
    return static_cast<float>(dist_2mm) * 0.002F;    // convert from units of 2mm to m
  }

  /// \brief checks point radial distance, and angle, ignores if necessary
  inline bool accept_point(const float r_m, const uint32_t th_ind) const
  {
    return (r_m >= m_min_radius_m) &&
           (r_m <= m_max_radius_m) &&
           (((th_ind >= m_min_azimuth_ind) && (th_ind <= m_max_azimuth_ind)) !=
           m_exclude_ranges);
  }

  template<typename T>
  inline T clamp(const T val, const T min, const T max)
  {
    return (val < min) ? min : ((val > max) ? max : val);
  }

  ///// initialization/precomputation functions
  /// \brief runs all the table initialization functions in the right order
  VELODYNE_DRIVER_LOCAL void init_tables(const Config & config);

  /// \brief precomputes the number of points in a complete rotation and the azimuth offset for
  ///        each firing in a block
  /// \param[in] rpm the LiDAR's spin rate in revolutions per minute
  /// \return none
  VELODYNE_DRIVER_LOCAL void init_azimuth_table_num_points(const float rpm);

  /// \brief initializes the fixed altitude angles for each firing in a block. This only needs to
  ///        run once and happens in the constructor
  /// \return none
  VELODYNE_DRIVER_LOCAL void init_altitude_table();

  /// \brief initializes sin and cosine lookup tables
  VELODYNE_DRIVER_LOCAL void init_trig_tables();

  /// \brief initializes intensity lookup table
  VELODYNE_DRIVER_LOCAL void init_intensity_table();

  /// \brief sets max/min index
  VELODYNE_DRIVER_LOCAL void init_extreme_azimuth_indices(
    const float min_azimuth_deg,
    const float max_azimuth_deg);

  /// \brief precompute rotation matrix
  VELODYNE_DRIVER_LOCAL void init_rotation_matrix(
    const float roll_rad,
    const float pitch_rad,
    const float yaw_rad);


  /// tau = 2 pi
  static constexpr float TAU = 6.283185307179586476925286766559F;

  /// parameters
  /// lookup table for azimuth offset for each firing in a block
  uint32_t m_azimuth_ind[NUM_POINTS_PER_BLOCK];
  /// lookup table for altitude angle for each firing in a fire sequence (2 per block)
  uint32_t m_altitude_ind[NUM_POINTS_PER_BLOCK];

  /// lookup table for sin
  float m_sin_table[AZIMUTH_ROTATION_RESOLUTION];
  /// lookup table for cos
  float m_cos_table[AZIMUTH_ROTATION_RESOLUTION];
  /// lookup table for intensity
  float m_intensity_table[NUM_INTENSITY_VALUES];

  /// mask to avoid modulo: packet id can go up to 3617: 0000 1111 1111 1111 = 4096
  uint16_t m_fire_id;
  uint16_t m_num_firing_per_scan;

  /// linear translation for all points
  geometry_msgs::msg::Point32 m_offset_m;
  /// rotation matrix
  float m_rot_mat[3U][3U];
  /// min radius
  float m_min_radius_m;
  /// max radius
  float m_max_radius_m;
  /// min angle
  uint32_t m_min_azimuth_ind;
  /// max angle
  uint32_t m_max_azimuth_ind;
  /// flag to denote if being inside min/max angle is considered ok (false) or not ok (true)
  bool m_exclude_ranges;
};  // class Driver

}  // namespace velodyne_driver
}  // namespace drivers
}  // namespace autoware

#endif  // VELODYNE_DRIVER__VLP16_TRANSLATOR_HPP_
