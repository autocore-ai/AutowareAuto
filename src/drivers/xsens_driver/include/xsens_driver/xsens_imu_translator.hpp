// Copyright 2018 Apex.AI, Inc.
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

/// \copyright Copyright 2017-2018 Apex.AI, Inc.

#ifndef XSENS_DRIVER__XSENS_IMU_TRANSLATOR_HPP_
#define XSENS_DRIVER__XSENS_IMU_TRANSLATOR_HPP_

#include <xsens_driver/visibility_control.hpp>
#include <xsens_driver/xsens_common.hpp>
#include <xsens_driver/xsens_base_translator.hpp>
#include <cstdint>
#include <mutex>
#include <vector>
#include "sensor_msgs/msg/imu.hpp"
#include "helper_functions/byte_reader.hpp"

namespace autoware
{
/// \brief Libraries, ROS nodes, and other functionality relating to
///         sensor drivers or actuation.
namespace drivers
{
namespace xsens_driver
{

class XSENS_DRIVER_PUBLIC XsensImuTranslator
  : public XsensBaseTranslator<XsensImuTranslator, sensor_msgs::msg::Imu>
{
public:
  class Config
  {
public:
    /// \brief Constructor
    Config();

private:
  };

  explicit XsensImuTranslator(const Config & config);

  void parse_timestamp(
    sensor_msgs::msg::Imu & message,
    int32_t data_id,
    const std::vector<uint8_t> & content);

  void parse_acceleration(
    sensor_msgs::msg::Imu & message,
    int32_t data_id,
    const std::vector<uint8_t> & content);

  template<typename MessageT>
  void parse_acceleration_internal(
    sensor_msgs::msg::Imu & message,
    const std::vector<uint8_t> & content);

  void parse_orientation_data(
    sensor_msgs::msg::Imu & message,
    int32_t data_id,
    const std::vector<uint8_t> & content);

  void parse_angular_velocity(
    sensor_msgs::msg::Imu & message,
    int32_t data_id,
    const std::vector<uint8_t> & content);

  template<typename MessageT>
  void parse_orientation_quaternion(
    sensor_msgs::msg::Imu & message,
    const std::vector<uint8_t> & content);

  template<typename MessageT>
  void parse_angular_velocity_rate_of_turn(
    sensor_msgs::msg::Imu & message,
    const std::vector<uint8_t> & content);

  void parse_xdigroup_mtdata2(
    XDIGroup xdigroup,
    sensor_msgs::msg::Imu & message,
    int32_t data_id,
    const std::vector<uint8_t> & content);

  void parse_xdi_coordinates(
    int32_t data_id,
    sensor_msgs::msg::Imu & message);

  template<typename T, std::size_t kNumber_of_values>
  std::array<T, kNumber_of_values> read_values(const std::vector<uint8_t> & content)
  {
    std::array<T, kNumber_of_values> values;

    common::helper_functions::ByteReader byte_reader(content);

    for (std::size_t i = 0; i < kNumber_of_values; ++i) {
      byte_reader.read(values[i]);
    }
    return values;
  }
};  // class Driver
}  // namespace xsens_driver
}  // namespace drivers
}  // namespace autoware

#endif  // XSENS_DRIVER__XSENS_IMU_TRANSLATOR_HPP_
