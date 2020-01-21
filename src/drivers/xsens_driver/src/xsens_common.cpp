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

#include <cstring>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>
#include <iostream>

#include <iomanip>

#include "xsens_driver/xsens_common.hpp"

namespace autoware
{
namespace drivers
{
namespace xsens_driver
{

MID MID_from_int(uint16_t value)
{
  switch (value) {
    case 0x42:
      return MID::ERROR;
    case 0x3E:
      return MID::WAKE_UP;
    case 0x3F:
      return MID::WAKE_UP_ACK;
    case 0x30:
      return MID::GO_TO_CONFIG;
    case 0x10:
      return MID::GO_TO_MEASUREMENT;
    case 0x40:
      return MID::RESET;
    case 0x00:
      return MID::REQ_DID;
    case 0x01:
      return MID::DEVICE_ID;
    case 0x1C:
      return MID::REQ_PRODUCT_CODE;
    case 0x1D:
      return MID::PRODUCT_CODE;
    case 0x12:
      return MID::REQ_FW_REV;
    case 0x13:
      return MID::FIRMWARE_REV;
    case 0x0E:
      return MID::RESTORE_FACTORY_DEF;
    case 0x18:
      return MID::SET_BAUDRATE;
    case 0x24:
      return MID::RUN_SELFTEST;
    case 0x25:
      return MID::SELFTEST_ACK;
    case 0xDA:
      return MID::SET_ERROR_MODE;
    case 0xDC:
      return MID::SET_TRANSMIT_DELAY;
    case 0x48:
      return MID::SET_OPTION_FLAGS;
    case 0x84:
      return MID::SET_LOCATION_ID;
    case 0x2C:
      return MID::SET_SYNC_SETTINGS;
    case 0x0C:
      return MID::REQ_CONFIGURATION;
    case 0x0D:
      return MID::CONFIGURATION;
    case 0x04:
      return MID::SET_PERIOD;
    case 0x86:
      return MID::SET_EXT_OUTPUT_MODE;
    case 0xC0:
      return MID::SET_OUTPUT_CONFIGURATION;
    case 0x8E:
      return MID::SET_STRING_OUTPUT_TYPE;
    case 0xEC:
      return MID::SET_ALIGNMENT_ROTATION;
    case 0xD0:
      return MID::SET_OUTPUT_MODE;
    case 0xD2:
      return MID::SET_OUTPUT_SETTINGS;
    case 0x34:
      return MID::REQ_DATA;
    case 0x32:
      return MID::MT_DATA;
    case 0x36:
      return MID::MT_DATA2;
    case 0xA4:
      return MID::RESET_ORIENTATION;
    case 0x60:
      return MID::SET_UTC_TIME;
    case 0xA8:
      return MID::ADJUST_UTC_TIME;
    case 0x61:
      return MID::UTC_TIME;
    case 0x62:
      return MID::REQ_AVAILABLE_SCENARIOS;
    case 0x63:
      return MID::AVAILABLE_SCENARIOS;
    case 0x64:
      return MID::SET_CURRENT_SCENARIO;
    case 0x66:
      return MID::SET_GRAVITY_MAGNITUDE;
    case 0x6E:
      return MID::SET_LAT_LON_ALT;
    case 0x22:
      return MID::SET_NO_ROTATION;
    default:
      throw std::runtime_error("Unknown value: " + std::to_string(value));
  }
}

XDIGroup XDIGroup_from_int(uint16_t value)
{
  switch (value) {
    case 0x0800:
      return XDIGroup::TEMPERATURE;
    case 0x1000:
      return XDIGroup::TIMESTAMP;
    case 0x2000:
      return XDIGroup::ORIENTATION_DATA;
    case 0x3000:
      return XDIGroup::PRESSURE;
    case 0x4000:
      return XDIGroup::ACCELERATION;
    case 0x5000:
      return XDIGroup::POSITION;
    case 0x7000:
      return XDIGroup::GNSS;
    case 0x8000:
      return XDIGroup::ANGULAR_VELOCITY;
    case 0x8800:
      return XDIGroup::GPS;
    case 0xA000:
      return XDIGroup::SENSOR_COMPONENT_READOUT;
    case 0xB000:
      return XDIGroup::ANALOG_IN;
    case 0xC000:
      return XDIGroup::MAGNETIC;
    case 0xD000:
      return XDIGroup::VELOCITY;
    case 0xE000:
      return XDIGroup::STATUS;
    default:
      throw std::runtime_error("Unknown value: " + std::to_string(value));
  }
}

GNSS GNSS_from_int(uint8_t value)
{
  switch (value) {
    case 0x10:
      return GNSS::PVT_DATA;
    case 0x20:
      return GNSS::SATELLITES_INFO;
    default:
      throw std::runtime_error("Unknown value: " + std::to_string(value));
  }
}

}  // namespace xsens_driver
}  // namespace drivers
}  // namespace autoware
