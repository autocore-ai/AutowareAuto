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

#ifndef XSENS_DRIVER__XSENS_GPS_TRANSLATOR_HPP_
#define XSENS_DRIVER__XSENS_GPS_TRANSLATOR_HPP_

#include <xsens_driver/visibility_control.hpp>
#include <xsens_driver/xsens_common.hpp>
#include <xsens_driver/xsens_base_translator.hpp>
#include <cstdint>
#include <mutex>
#include <vector>
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace autoware
{
/// \brief Libraries, ROS nodes, and other functionality relating to
///         sensor drivers or actuation.
namespace drivers
{
namespace xsens_driver
{

class XSENS_DRIVER_PUBLIC XsensGpsTranslator
  : public XsensBaseTranslator<XsensGpsTranslator, sensor_msgs::msg::NavSatFix>
{
public:
  class Config
  {
public:
    /// \brief Constructor
    Config();

private:
  };

  explicit XsensGpsTranslator(const Config & config);

  void parse_xdigroup_mtdata2(
    XDIGroup xdigroup,
    sensor_msgs::msg::NavSatFix & message,
    int32_t data_id,
    const std::vector<uint8_t> & content);

  void parse_gnss(
    sensor_msgs::msg::NavSatFix & message,
    int32_t data_id,
    const std::vector<uint8_t> & content);
};  // class Driver
}  // namespace xsens_driver
}  // namespace drivers
}  // namespace autoware

#endif  // XSENS_DRIVER__XSENS_GPS_TRANSLATOR_HPP_
