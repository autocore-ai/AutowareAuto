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


/// \copyright Copyright 2017-2018 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief This file defines a simple ROS 2 xsens driver that publishes full point clouds

#ifndef XSENS_NODES__XSENS_IMU_NODE_HPP_
#define XSENS_NODES__XSENS_IMU_NODE_HPP_

#include <string>
#include <vector>
#include "serial_driver/serial_driver_node.hpp"
#include "xsens_driver/xsens_imu_translator.hpp"
#include "xsens_nodes/xsens_common_node.hpp"
#include "xsens_nodes/visibility_control.hpp"
#include "sensor_msgs/msg/imu.hpp"


namespace autoware
{
namespace drivers
{
/// \brief Resources for nodes that use the `xsens_driver`
namespace xsens_nodes
{

using XsensImuNode = XsensCommonNode<
  xsens_driver::XsensImuTranslator, sensor_msgs::msg::Imu
>;

}  // namespace xsens_nodes
}  // namespace drivers
}  // namespace autoware

#endif  // XSENS_NODES__XSENS_IMU_NODE_HPP_
