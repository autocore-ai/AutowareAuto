// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#ifndef EMERGENCY_HANDLER__HEARTBEAT_CHECKER_HPP_
#define EMERGENCY_HANDLER__HEARTBEAT_CHECKER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "emergency_handler/visibility_control.hpp"

namespace autoware
{
namespace emergency_handler
{

/// \brief HeartbeatChecker checks if messages appears on specified topic with specified frequency
///
/// It subscribes to specific topic and measures time from the last message received on this topic.
/// If the measured time is greater than defined timeout value then it returns true on isTimeout().
template<class MsgType>
class EMERGENCY_HANDLER_PUBLIC HeartbeatChecker
{
public:
  /// \brief Constructor
  ///
  /// \param[in] node is a node handler
  /// \param[in] topic_name is the name of monitored topic
  /// \param[in] timeout is a parameter that defines max allowable time between consequent messages
  HeartbeatChecker(
    rclcpp::Node & node, const std::string & topic_name, const double timeout)
  : clock_(node.get_clock()),
    timeout_(timeout)
  {
    using std::placeholders::_1;
    sub_heartbeat_ = node.create_subscription<MsgType>(
      topic_name, rclcpp::QoS{1},
      std::bind(&HeartbeatChecker::onHeartbeat, this, _1));
  }

  /// \brief Determines if the timeout state has been reached
  /// \return Returns true if timeout occurred, otherwise false.
  bool isTimeout()
  {
    const auto time_from_last_heartbeat = clock_->now() - last_heartbeat_time_;
    return time_from_last_heartbeat.seconds() > timeout_;
  }

private:
  void onHeartbeat(const typename MsgType::ConstSharedPtr msg)
  {
    (void)msg;
    last_heartbeat_time_ = clock_->now();
  }

  rclcpp::Clock::SharedPtr clock_;

  double timeout_;

  typename rclcpp::Subscription<MsgType>::SharedPtr sub_heartbeat_;

  rclcpp::Time last_heartbeat_time_ = rclcpp::Time(0);
};

}  // namespace emergency_handler
}  // namespace autoware

#endif  // EMERGENCY_HANDLER__HEARTBEAT_CHECKER_HPP_
