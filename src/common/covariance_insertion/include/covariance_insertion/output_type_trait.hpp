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

#ifndef COVARIANCE_INSERTION__OUTPUT_TYPE_TRAIT_HPP_
#define COVARIANCE_INSERTION__OUTPUT_TYPE_TRAIT_HPP_


#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>


template<typename InputT>
struct output
{
  using type = InputT;
};

template<typename InputT, typename Enable = void>
struct needs_conversion : public std::true_type {};

template<typename InputT>
struct needs_conversion<InputT, std::enable_if_t<
    std::is_same<typename output<InputT>::type, InputT>::value>>: public std::false_type {};

// Specializations.
template<>
struct output<geometry_msgs::msg::Pose>
{
  using type = geometry_msgs::msg::PoseWithCovariance;
};

template<>
struct output<geometry_msgs::msg::PoseStamped>
{
  using type = geometry_msgs::msg::PoseWithCovarianceStamped;
};

template<>
struct output<geometry_msgs::msg::Twist>
{
  using type = geometry_msgs::msg::TwistWithCovariance;
};

template<>
struct output<geometry_msgs::msg::TwistStamped>
{
  using type = geometry_msgs::msg::TwistWithCovarianceStamped;
};

#endif  // COVARIANCE_INSERTION__OUTPUT_TYPE_TRAIT_HPP_
