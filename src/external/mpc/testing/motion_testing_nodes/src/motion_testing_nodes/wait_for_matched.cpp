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

#include "motion_testing_nodes/wait_for_matched.hpp"

#include <chrono>

namespace motion
{
namespace motion_testing_nodes
{

std::chrono::nanoseconds wait_for_matched(
  const rclcpp::PublisherBase & pub,
  const std::size_t minimum_match_count,
  const std::chrono::nanoseconds timeout,
  const std::chrono::nanoseconds poll_period)
{
  const auto match_count = [&pub, minimum_match_count]() -> bool {
      return pub.get_subscription_count() >= minimum_match_count;
    };
  return details::wait_for_matched(timeout, poll_period, match_count);
}

////////////////////////////////////////////////////////////////////////////////
std::chrono::nanoseconds wait_for_matched(
  const rclcpp::SubscriptionBase & sub,
  const std::size_t minimum_match_count,
  const std::chrono::nanoseconds timeout,
  const std::chrono::nanoseconds poll_period)
{
  const auto match_count = [&sub, minimum_match_count]() -> bool {
      return sub.get_publisher_count() >= minimum_match_count;
    };
  return details::wait_for_matched(timeout, poll_period, match_count);
}

}  // namespace motion_testing_nodes
}  // namespace motion
