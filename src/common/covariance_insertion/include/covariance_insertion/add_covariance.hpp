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

#ifndef COVARIANCE_INSERTION__ADD_COVARIANCE_HPP_
#define COVARIANCE_INSERTION__ADD_COVARIANCE_HPP_

#include <covariance_insertion/traits.hpp>
#include <covariance_insertion/output_type_trait.hpp>

#include <string>
#include <vector>

namespace autoware
{
namespace covariance_insertion
{

namespace detail
{
static constexpr auto kPoseTag = "pose";
static constexpr auto kTwistTag = "twist";
static constexpr auto kDirectlyTag = "directly";
}  // namespace detail

template<typename MsgT, typename ScalarT>
void add_covariance(
  MsgT * msg,
  const std::vector<ScalarT> & covariance,
  const std::enable_if_t<has_covariance_member<MsgT>::value, std::string> & field)
{
  if (!msg) {return;}
  if (field != detail::kDirectlyTag) {
    throw std::runtime_error("Message has covariance directly, but asked for field: " + field);
  }
  if (msg->covariance.size() != covariance.size()) {
    throw std::runtime_error(
            "Number of covariance entries does not match. The message has " +
            std::to_string(msg->covariance.size()) + " entries, while there are " +
            std::to_string(covariance.size()) + " entries in parameters of this node.");
  }
  for (auto i = 0U; i < covariance.size(); ++i) {
    msg->covariance[i] = covariance[i];
  }
}

template<typename MsgT, typename ScalarT>
void add_covariance_to_field(
  MsgT * msg,
  const std::vector<ScalarT> & covariance,
  const std::string & field)
{
  if (!msg) {return;}
  if (has_covariance_member<MsgT>::value) {
    add_covariance(msg, covariance, detail::kDirectlyTag);
  } else {
    add_covariance(msg, covariance, field);
  }
}

template<typename MsgT, typename ScalarT>
void add_covariance(
  MsgT * msg,
  const std::vector<ScalarT> & covariance,
  const std::enable_if_t<
    has_twist_member<MsgT>::value &&
    !has_pose_member<MsgT>::value &&
    !has_covariance_member<MsgT>::value, std::string> & field)
{
  if (!msg) {return;}
  if (field != detail::kTwistTag) {
    throw std::runtime_error("Cannot set: " + field);
  }
  add_covariance_to_field(&msg->twist, covariance, field);
}

template<typename MsgT, typename ScalarT>
void add_covariance(
  MsgT * msg,
  const std::vector<ScalarT> & covariance,
  const std::enable_if_t<
    !has_twist_member<MsgT>::value &&
    has_pose_member<MsgT>::value &&
    !has_covariance_member<MsgT>::value, std::string> & field)
{
  if (!msg) {return;}
  if (field != detail::kPoseTag) {
    throw std::runtime_error("Cannot set: " + field);
  }
  add_covariance_to_field(&msg->pose, covariance, field);
}

template<typename MsgT, typename ScalarT>
void add_covariance(
  MsgT * msg,
  const std::vector<ScalarT> & covariance,
  const std::enable_if_t<
    has_twist_member<MsgT>::value &&
    has_pose_member<MsgT>::value &&
    !has_covariance_member<MsgT>::value, std::string> & field)
{
  if (!msg) {return;}
  if (field == detail::kTwistTag) {
    add_covariance_to_field(&msg->twist, covariance, field);
  } else if (field == detail::kPoseTag) {
    add_covariance_to_field(&msg->pose, covariance, field);
  } else {
    throw std::runtime_error("Cannot set: " + field);
  }
}

}  // namespace covariance_insertion
}  // namespace autoware

#endif  // COVARIANCE_INSERTION__ADD_COVARIANCE_HPP_
