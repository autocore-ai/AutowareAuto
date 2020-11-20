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

#ifndef COVARIANCE_INSERTION_NODE__CONVERT_HPP_
#define COVARIANCE_INSERTION_NODE__CONVERT_HPP_

#include <covariance_insertion_node/output_type_trait.hpp>

namespace autoware
{
namespace covariance_insertion_node
{

template<typename InputT>
constexpr typename output<InputT>::type convert(const InputT & input_msg) noexcept
{
  static_assert(
    !needs_conversion<InputT>::value,
    "The default implementation is only good when the types are the same. "
    "Otherwise this function has to be specialized.");
  return input_msg;
}

template<>
typename geometry_msgs::msg::PoseWithCovariance convert(
  const geometry_msgs::msg::Pose & input_msg) noexcept;

template<>
typename geometry_msgs::msg::TwistWithCovariance convert(
  const geometry_msgs::msg::Twist & input_msg) noexcept;

template<>
typename geometry_msgs::msg::PoseWithCovarianceStamped  convert(
  const geometry_msgs::msg::PoseStamped & input_msg) noexcept;

template<>
typename geometry_msgs::msg::TwistWithCovarianceStamped convert(
  const geometry_msgs::msg::TwistStamped & input_msg) noexcept;

}  // namespace covariance_insertion_node
}  // namespace autoware

#endif  // COVARIANCE_INSERTION_NODE__CONVERT_HPP_
