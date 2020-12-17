// Copyright 2020 Christopher Ho
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
#ifndef MOTION_TESTING_NODES__WAIT_FOR_MATCHED_HPP_
#define MOTION_TESTING_NODES__WAIT_FOR_MATCHED_HPP_

#include <motion_testing_nodes/visibility_control.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <thread>

namespace motion
{
namespace motion_testing_nodes
{

namespace details
{

/// Implementation of wait_for_matched
template<typename ConditionF>
std::chrono::nanoseconds wait_for_matched(
  const std::chrono::nanoseconds timeout,
  const std::chrono::nanoseconds poll_period,
  const ConditionF & condition)
{
  const auto start = std::chrono::steady_clock::now();
  const auto end = start + timeout;
  auto now = start;
  auto matched = condition();
  while ((now < end) && (!matched)) {
    std::this_thread::sleep_for(poll_period);
    matched = condition();
    now = std::chrono::steady_clock::now();
  }
  if (!matched) {
    throw std::runtime_error{"Timeout waiting for match"};
  }

  return end - now;
}


}  // namespace details

/// Block thread until publisher has matched with at least a certain number of participants
/// \param[in] pub The publisher to check for number of matching participants
/// \param[in] minimum_match_count Minimum number of matching participants to wait for
/// \param[in] timeout Maximum time to block for
/// \param[in] poll_period Time to sleep between checks
/// \throw std::runtime_error on timeout
/// \return Time remaining before timeout, for chaining convenience
MOTION_TESTING_NODES_PUBLIC std::chrono::nanoseconds wait_for_matched(
  const rclcpp::PublisherBase & pub,
  const std::size_t minimum_match_count,
  const std::chrono::nanoseconds timeout = std::chrono::seconds{5LL},
  const std::chrono::nanoseconds poll_period = std::chrono::milliseconds{1LL});

/// Block thread until subscription has matched with at least a certain number of participants
/// \param[in] sub The subscription to check for number of matching participants
/// \param[in] minimum_match_count Minimum number of matching participants to wait for
/// \param[in] timeout Maximum time to block for
/// \param[in] poll_period Time to sleep between checks
/// \throw std::runtime_error on timeout
/// \return Time remaining before timeout, for chaining convenience
MOTION_TESTING_NODES_PUBLIC std::chrono::nanoseconds wait_for_matched(
  const rclcpp::SubscriptionBase & sub,
  const std::size_t minimum_match_count,
  const std::chrono::nanoseconds timeout = std::chrono::seconds{5LL},
  const std::chrono::nanoseconds poll_period = std::chrono::milliseconds{1LL});

}  // namespace motion_testing_nodes
}  // namespace motion

#endif  // MOTION_TESTING_NODES__WAIT_FOR_MATCHED_HPP_
