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

#include <cstring>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>
#include <iostream>

#include <iomanip>

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "xsens_driver/xsens_common.hpp"
#include "xsens_driver/xsens_gps_translator.hpp"

#include "helper_functions/byte_reader.hpp"
#include "common/types.hpp"

using autoware::common::types::float64_t;

namespace autoware
{
namespace drivers
{
namespace xsens_driver
{

////////////////////////////////////////////////////////////////////////////////
XsensGpsTranslator::XsensGpsTranslator(const Config &)
: XsensBaseTranslator()
{
}

void XsensGpsTranslator::parse_xdigroup_mtdata2(
  XDIGroup xdigroup,
  sensor_msgs::msg::NavSatFix & message,
  int32_t data_id,
  const std::vector<uint8_t> & content)
{
  switch (xdigroup) {
    case XDIGroup::TEMPERATURE:
      break;
    case XDIGroup::TIMESTAMP:
      break;
    case XDIGroup::ORIENTATION_DATA:
      break;
    case XDIGroup::PRESSURE:
      break;
    case XDIGroup::ACCELERATION:
      break;
    case XDIGroup::POSITION:
      break;
    case XDIGroup::GNSS:
      parse_gnss(message, data_id, content);
      break;
    case XDIGroup::ANGULAR_VELOCITY:
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

void XsensGpsTranslator::parse_gnss(
  sensor_msgs::msg::NavSatFix & message,
  int32_t data_id,
  const std::vector<uint8_t> & content)
{
  const GNSS value = GNSS_from_int(static_cast<uint8_t>(data_id & 0x00F0));

  switch (value) {
    case GNSS::PVT_DATA:
      {
        autoware::common::helper_functions::ByteReader byte_reader(content);

        uint32_t itow = 0;
        byte_reader.read(itow);

        uint16_t year = 0;
        byte_reader.read(year);

        uint8_t month = 0;
        byte_reader.read(month);

        uint8_t day = 0;
        byte_reader.read(day);

        uint8_t hour = 0;
        byte_reader.read(hour);

        uint8_t minute = 0;
        byte_reader.read(minute);

        uint8_t second = 0;
        byte_reader.read(second);

        uint8_t valid = 0;
        byte_reader.read(valid);

        uint32_t tAcc = 0;
        byte_reader.read(tAcc);

        int32_t nano = 0;
        byte_reader.read(nano);

        uint8_t fixtype = 0;
        byte_reader.read(fixtype);

        uint8_t flags = 0;
        byte_reader.read(flags);

        uint8_t numSV = 0;
        byte_reader.read(numSV);

        // Skip pad byte
        byte_reader.skip(1);

        int32_t lon = 0;
        byte_reader.read(lon);

        int32_t lat = 0;
        byte_reader.read(lat);

        int32_t height = 0;
        byte_reader.read(height);

        int32_t hMSL = 0;
        byte_reader.read(hMSL);

        uint32_t hAcc = 0;
        byte_reader.read(hAcc);

        uint32_t vAcc = 0;
        byte_reader.read(vAcc);

        int32_t velN = 0;
        byte_reader.read(velN);

        int32_t velE = 0;
        byte_reader.read(velE);

        int32_t velD = 0;
        byte_reader.read(velD);

        int32_t gSpeed = 0;
        byte_reader.read(gSpeed);

        int32_t headMot = 0;
        byte_reader.read(headMot);

        uint32_t sAcc = 0;
        byte_reader.read(sAcc);

        uint32_t headAcc = 0;
        byte_reader.read(headAcc);

        int32_t headVeh = 0;
        byte_reader.read(headVeh);

        uint16_t gdop = 0;
        byte_reader.read(gdop);

        uint16_t pdop = 0;
        byte_reader.read(pdop);

        uint16_t tdop = 0;
        byte_reader.read(tdop);

        uint16_t vdop = 0;
        byte_reader.read(vdop);

        uint16_t hdop = 0;
        byte_reader.read(hdop);

        uint16_t ndop = 0;
        byte_reader.read(ndop);

        uint16_t edop = 0;
        byte_reader.read(edop);

        //  scaling correction
        float64_t dlon = lon * 1e-7;
        float64_t dlat = lat * 1e-7;
        float64_t dheadMot = headMot * 1e-5;
        float64_t dheadVeh = headVeh * 1e-5;
        float64_t dgdop = gdop * 0.01;
        float64_t dpdop = pdop * 0.01;
        float64_t dtdop = tdop * 0.01;
        float64_t dvdop = vdop * 0.01;
        float64_t dhdop = hdop * 0.01;
        float64_t dndop = ndop * 0.01;
        float64_t dedop = edop * 0.01;

        // NOTE(esteve): None of this is actually necessary for the ROS NavSatFix message
        (void)dheadMot;
        (void)dheadVeh;
        (void)dgdop;
        (void)dpdop;
        (void)dtdop;
        (void)dvdop;
        (void)dhdop;
        (void)dndop;
        (void)dedop;

        if (fixtype == 0x00) {
          message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
          message.status.service = 0;
        } else {
          message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
          message.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        }
        message.latitude = dlat;
        message.longitude = dlon;
        message.altitude = height / 1e3;
      }
      break;
    case GNSS::SATELLITES_INFO:
      break;
  }
}

}  // namespace xsens_driver
}  // namespace drivers
}  // namespace autoware
