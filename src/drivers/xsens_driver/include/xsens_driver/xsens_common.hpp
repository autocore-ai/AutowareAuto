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

#ifndef XSENS_DRIVER__XSENS_COMMON_HPP_
#define XSENS_DRIVER__XSENS_COMMON_HPP_

#include <xsens_driver/visibility_control.hpp>
#include <cstdint>
#include <mutex>
#include <numeric>
#include <vector>

namespace autoware
{
/// \brief Libraries, ROS nodes, and other functionality relating to
///         sensor drivers or actuation.
namespace drivers
{
namespace xsens_driver
{

enum class MID : uint8_t
{
  // Error message, 1 data byte
  ERROR = 0x42,

  // State MID
  // Wake up procedure
  WAKE_UP = 0x3E,
  // Wake up ack to put device in config mode
  WAKE_UP_ACK = 0x3F,
  // Switch to config state
  GO_TO_CONFIG = 0x30,
  // Switch to measurement state
  GO_TO_MEASUREMENT = 0x10,
  // Reset device
  RESET = 0x40,

  // Informational messages
  // Request device id
  REQ_DID = 0x00,
  // DeviceID, 4 bytes: HH HL LH LL
  DEVICE_ID = 0x01,
  // Request product code in plain text
  REQ_PRODUCT_CODE = 0x1C,
  // Product code (max 20 bytes data)
  PRODUCT_CODE = 0x1D,
  // Request hardware version
  REQ_HARDWARE_VERSION = 0x1E,
  // Hardware version
  HARDWARE_VERSION = 0x1F,
  // Request firmware revision
  REQ_FW_REV = 0x12,
  // Firmware revision, 3 bytes: major minor rev
  FIRMWARE_REV = 0x13,

  // Device specific messages
  // Restore factory defaults
  RESTORE_FACTORY_DEF = 0x0E,
  // Baudrate, 1 byte
  SET_BAUDRATE = 0x18,
  // Run the built-in self test (MTi-1/10/100 series)
  RUN_SELFTEST = 0x24,
  // Self test results, 2 bytes
  SELFTEST_ACK = 0x25,
  // GNSS platform setting, 2 bytes (only MTi-G-700/710 with FW1.7 or higher)
  SET_GNSS_PLATFORM = 0x76,
  // Error mode, 2 bytes, 0000, 0001, 0002, 0003 (default 0001)
  SET_ERROR_MODE = 0xDA,
  // Transmit delay (RS485), 2 bytes, number of clock ticks (1/29.4912 MHz)
  SET_TRANSMIT_DELAY = 0xDC,
  // Set state of OptionFlags (MTi-1/2/3), 4 + 4 bytes
  SET_OPTION_FLAGS = 0x48,
  // Location ID, 2 bytes, arbitrary, default is 0
  SET_LOCATION_ID = 0x84,

  // Synchronization messages
  // Synchronization settings (MTi-1/10/100 series only), N*12 bytes
  SET_SYNC_SETTINGS = 0x2C,

  // Configuration messages
  // Request configuration
  REQ_CONFIGURATION = 0x0C,
  // Configuration, 118 bytes
  CONFIGURATION = 0x0D,
  // Sampling period (MTi/MTi-G only), 2 bytes
  SET_PERIOD = 0x04,
  // Extended output mode (MTi-10/100), 2 bytes, bit 4 for extended UART
  SET_EXT_OUTPUT_MODE = 0x86,
  // Output configuration (MTi-1/10/100 series only), N*4 bytes
  SET_OUTPUT_CONFIGURATION = 0xC0,
  // Configure NMEA data output (MTi-10/100), 2 bytes
  SET_STRING_OUTPUT_TYPE = 0x8E,
  // Set sensor of local alignment quaternion
  SET_ALIGNMENT_ROTATION = 0xEC,
  // Output mode (MTi/MTi-G only), 2 bytes
  SET_OUTPUT_MODE = 0xD0,
  // Output settings (MTi/MTi-G only), 4 bytes
  SET_OUTPUT_SETTINGS = 0xD2,

  // Data messages
  // Request MTData message (for 65535 skip factor)
  REQ_DATA = 0x34,
  // Legacy data packet
  MT_DATA = 0x32,
  // Newer data packet (MTi-10/100 series only)
  MT_DATA2 = 0x36,

  // Filter messages
  // Reset orientation, 2 bytes
  RESET_ORIENTATION = 0xA4,
  // Request or set UTC time from sensor (MTI-G and MTi-10/100 series)
  SET_UTC_TIME = 0x60,
  // Set correction ticks to UTC time
  ADJUST_UTC_TIME = 0xA8,
  // UTC Time (MTI-G and MTi-10/100 series), 12 bytes
  UTC_TIME = 0x61,
  // Request the available XKF scenarios on the device
  REQ_AVAILABLE_SCENARIOS = 0x62,
  // Available Scenarios
  AVAILABLE_SCENARIOS = 0x63,
  // Current XKF scenario, 2 bytes
  SET_CURRENT_SCENARIO = 0x64,
  // Magnitude of the gravity used for the sensor fusion mechanism, 4 bytes
  SET_GRAVITY_MAGNITUDE = 0x66,
  // Latitude, Longitude and Altitude for local declination and gravity
  // (MTi-10/100 series only), 24 bytes
  SET_LAT_LON_ALT = 0x6E,
  // Initiate No Rotation procedure (not on MTi-G), 2 bytes
  SET_NO_ROTATION = 0x22,
  // In-run Compass Calibration (ICC) command, 1 byte
  ICC_COMMAND = 0x74,
};

XSENS_DRIVER_PUBLIC MID MID_from_int(uint16_t value);

enum class XDIGroup : uint16_t
{
  // Values for the XDI groups.
  TEMPERATURE = 0x0800,
  TIMESTAMP = 0x1000,
  ORIENTATION_DATA = 0x2000,
  PRESSURE = 0x3000,
  ACCELERATION = 0x4000,
  POSITION = 0x5000,
  GNSS = 0x7000,
  ANGULAR_VELOCITY = 0x8000,
  GPS = 0x8800,
  SENSOR_COMPONENT_READOUT = 0xA000,
  ANALOG_IN = 0xB000,    // deprecated
  MAGNETIC = 0xC000,
  VELOCITY = 0xD000,
  STATUS = 0xE000,
};

XSENS_DRIVER_PUBLIC XDIGroup XDIGroup_from_int(uint16_t value);

enum class GNSS : uint8_t
{
  PVT_DATA = 0x10,
  SATELLITES_INFO = 0x20,
};

XSENS_DRIVER_PUBLIC GNSS GNSS_from_int(uint8_t value);

}  // namespace xsens_driver
}  // namespace drivers
}  // namespace autoware

#endif  // XSENS_DRIVER__XSENS_COMMON_HPP_
