// Copyright 2020 Apex.AI, Inc.
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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#include <covariance_insertion_nodes/convert.hpp>

namespace autoware
{
namespace covariance_insertion_nodes
{

template<>
typename geometry_msgs::msg::PoseWithCovariance convert(
  const geometry_msgs::msg::Pose & input_msg) noexcept
{
  geometry_msgs::msg::PoseWithCovariance out_msg;
  out_msg.pose = input_msg;
  return out_msg;
}

template<>
typename geometry_msgs::msg::TwistWithCovariance convert(
  const geometry_msgs::msg::Twist & input_msg) noexcept
{
  geometry_msgs::msg::TwistWithCovariance out_msg;
  out_msg.twist = input_msg;
  return out_msg;
}

template<>
typename geometry_msgs::msg::PoseWithCovarianceStamped  convert(
  const geometry_msgs::msg::PoseStamped & input_msg) noexcept
{
  geometry_msgs::msg::PoseWithCovarianceStamped out_msg;
  out_msg.pose.pose = input_msg.pose;
  out_msg.header = input_msg.header;
  return out_msg;
}

template<>
typename geometry_msgs::msg::TwistWithCovarianceStamped convert(
  const geometry_msgs::msg::TwistStamped & input_msg) noexcept
{
  geometry_msgs::msg::TwistWithCovarianceStamped out_msg;
  out_msg.twist.twist = input_msg.twist;
  out_msg.header = input_msg.header;
  return out_msg;
}

}  // namespace covariance_insertion_nodes
}  // namespace autoware
