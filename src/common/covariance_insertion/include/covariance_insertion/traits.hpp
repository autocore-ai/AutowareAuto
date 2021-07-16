// Copyright 2020 Apex.AI, Inc., Arm Limited
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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#ifndef COVARIANCE_INSERTION__TRAITS_HPP_
#define COVARIANCE_INSERTION__TRAITS_HPP_

#include <type_traits>

namespace autoware
{
namespace covariance_insertion
{

template<typename T, typename = void>
struct has_covariance_member : std::false_type {};

template<typename T>
struct has_covariance_member<T, decltype((void)T::covariance, void())>: std::true_type
{
};

template<typename T, typename = void>
struct has_pose_member : std::false_type {};

template<typename T>
struct has_pose_member<T, decltype((void)T::pose, void())>: std::true_type
{
};

template<typename T, typename = void>
struct has_twist_member : std::false_type {};

template<typename T>
struct has_twist_member<T, decltype((void)T::twist, void())>: std::true_type
{
};

}  // namespace covariance_insertion
}  // namespace autoware

#endif  // COVARIANCE_INSERTION__TRAITS_HPP_
