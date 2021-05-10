// Copyright 2018 the Autoware Foundation
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

#include <cstring>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>
#include <iostream>

#include <iomanip>

#include "sensor_msgs/msg/imu.hpp"
#include "xsens_driver/xsens_common.hpp"
#include "xsens_driver/xsens_imu_translator.hpp"

#include "helper_functions/byte_reader.hpp"
#include "common/types.hpp"

using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace drivers
{
namespace xsens_driver
{

////////////////////////////////////////////////////////////////////////////////
XsensImuTranslator::XsensImuTranslator(const Config &)
: XsensBaseTranslator()
{}

void XsensImuTranslator::parse_xdigroup_mtdata2(
  XDIGroup xdigroup,
  sensor_msgs::msg::Imu & message,
  int32_t data_id,
  const std::vector<uint8_t> & content)
{
  switch (xdigroup) {
    case XDIGroup::TEMPERATURE:
      break;
    case XDIGroup::TIMESTAMP:
      parse_timestamp(message, data_id, content);
      break;
    case XDIGroup::ORIENTATION_DATA:
      parse_orientation_data(message, data_id, content);
      break;
    case XDIGroup::PRESSURE:
      break;
    case XDIGroup::ACCELERATION:
      parse_acceleration(message, data_id, content);
      break;
    case XDIGroup::POSITION:
      break;
    case XDIGroup::GNSS:
      break;
    case XDIGroup::ANGULAR_VELOCITY:
      parse_angular_velocity(message, data_id, content);
      break;
    case XDIGroup::GPS:
      break;
    case XDIGroup::SENSOR_COMPONENT_READOUT:
      break;
    case XDIGroup::ANALOG_IN:
      break;
    case XDIGroup::MAGNETIC:
      break;
    case XDIGroup::VELOCITY:
      break;
    case XDIGroup::STATUS:
      break;
    default:
      throw std::runtime_error("Unknown group");
  }
}

void XsensImuTranslator::parse_timestamp(
  sensor_msgs::msg::Imu & message,
  int32_t data_id,
  const std::vector<uint8_t> & content)
{
  (void)message;
  (void)data_id;
  (void)content;
}

void XsensImuTranslator::parse_acceleration(
  sensor_msgs::msg::Imu & message,
  int32_t data_id,
  const std::vector<uint8_t> & content)
{
  parse_xdi_coordinates(data_id, message);

  if (use_double_precision(data_id)) {
    parse_acceleration_internal<float64_t>(message, content);
  } else {
    parse_acceleration_internal<float32_t>(message, content);
  }
}

template<typename MessageT>
void XsensImuTranslator::parse_acceleration_internal(
  sensor_msgs::msg::Imu & message,
  const std::vector<uint8_t> & content)
{
  std::array<MessageT, 3> values = read_values<MessageT, 3>(content);

  message.linear_acceleration.x = values[0];
  message.linear_acceleration.y = values[1];
  message.linear_acceleration.z = values[2];
}

void XsensImuTranslator::parse_orientation_data(
  sensor_msgs::msg::Imu & message,
  int32_t data_id,
  const std::vector<uint8_t> & content)
{
  parse_xdi_coordinates(data_id, message);

  switch (data_id & 0x00F0) {
    case 0x10:
      // Quaternion
      if (use_double_precision(data_id)) {
        parse_orientation_quaternion<float64_t>(message, content);
      } else {
        parse_orientation_quaternion<float32_t>(message, content);
      }
      break;
    case 0x20:
      // TODO(esteve): implement
      // Rotation Matrix
      throw std::runtime_error("Rotation matrix not supported yet");
    case 0x30:
      // TODO(esteve): implement
      // Euler Angles
      throw std::runtime_error("Euler angles not supported yet");
    default:
      throw std::runtime_error("Unknown data_id: " + std::to_string(data_id));
  }
}

void XsensImuTranslator::parse_angular_velocity(
  sensor_msgs::msg::Imu & message,
  int32_t data_id,
  const std::vector<uint8_t> & content)
{
  parse_xdi_coordinates(data_id, message);

  switch (data_id & 0x00F0) {
    case 0x20:
      // Rate of Turn
      if (use_double_precision(data_id)) {
        parse_angular_velocity_rate_of_turn<float64_t>(message, content);
      } else {
        parse_angular_velocity_rate_of_turn<float32_t>(message, content);
      }
      break;
    case 0x30:
      // Delta Q
      break;
    case 0x40:
      // RateOfTurnHR
      if (use_double_precision(data_id)) {
        parse_angular_velocity_rate_of_turn<float64_t>(message, content);
      } else {
        parse_angular_velocity_rate_of_turn<float32_t>(message, content);
      }
      break;
    default:
      throw std::runtime_error("Unknown data_id: " + std::to_string(data_id));
  }
}

template<typename MessageT>
void XsensImuTranslator::parse_orientation_quaternion(
  sensor_msgs::msg::Imu & message,
  const std::vector<uint8_t> & content)
{
  std::array<MessageT, 4> values = read_values<MessageT, 4>(content);

  message.orientation.x = values[0];
  message.orientation.y = values[1];
  message.orientation.z = values[2];
  message.orientation.w = values[3];
}

template<typename MessageT>
void XsensImuTranslator::parse_angular_velocity_rate_of_turn(
  sensor_msgs::msg::Imu & message,
  const std::vector<uint8_t> & content)
{
  std::array<MessageT, 3> values = read_values<MessageT, 3>(content);

  message.angular_velocity.x = values[0];
  message.angular_velocity.y = values[1];
  message.angular_velocity.z = values[2];
}

void XsensImuTranslator::parse_xdi_coordinates(
  int32_t data_id, sensor_msgs::msg::Imu & message)
{
  switch (data_id & 0x000C) {
    case 0x00:
      message.header.frame_id = "ENU";
      break;
    case 0x04:
      message.header.frame_id = "NED";
      break;
    case 0x08:
      message.header.frame_id = "NWU";
      break;
  }
}

}  // namespace xsens_driver
}  // namespace drivers
}  // namespace autoware
