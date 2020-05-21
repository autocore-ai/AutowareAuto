// Copyright 2019 Apex.AI, Inc.
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

#ifndef TEST_RELATIVE_LOCALIZER_NODE_HPP_
#define TEST_RELATIVE_LOCALIZER_NODE_HPP_

#include <localization_common/localizer_base.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <memory>
#include <string>

#include "common/types.hpp"

using autoware::common::types::bool8_t;
using autoware::localization::localization_common::RelativeLocalizerBase;
using autoware::localization::localization_nodes::RelativeLocalizerNode;
using autoware::localization::localization_nodes::TopicQoS;

using MsgWithHeader = geometry_msgs::msg::TransformStamped;
using TestObservation = MsgWithHeader;
using TestMap = MsgWithHeader;

using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using Transform = geometry_msgs::msg::TransformStamped;

constexpr int TEST_ERROR_ID = -9999;

class MockRelativeLocalizer : public RelativeLocalizerBase<TestObservation, TestMap, int>
{
public:
  MockRelativeLocalizer(
    std::shared_ptr<TestMap> obs_ptr,
    std::shared_ptr<TestObservation> map_ptr);
  // constructor when the tracking is not needed.
  MockRelativeLocalizer() = default;

  RegistrationSummary register_measurement_impl(
    const TestObservation & msg,
    const Transform & transform_initial, PoseWithCovarianceStamped & pose_out) override;

  void set_map_impl(const TestMap & msg) override;

  const std::string & map_frame_id() const noexcept override;

  std::chrono::system_clock::time_point map_stamp() const noexcept override;

private:
  TestMap m_map;
  std::shared_ptr<TestMap> m_map_tracking_ptr;
  std::shared_ptr<TestObservation> m_observation_tracking_ptr;
};

class MockRelativeLocalizerConfig {};


class MockInitializer
{
public:
  Transform guess(
    const tf2::BufferCore &, tf2::TimePoint stamp,
    const std::string & id1, const std::string & id2);
};

class TestRelativeLocalizerNode : public RelativeLocalizerNode<TestObservation, TestMap,
    MockRelativeLocalizer, MockRelativeLocalizerConfig, MockInitializer>
{
public:
  using RelativeLocalizerNode::RelativeLocalizerNode;

  // Expose protected method for convenience
  void set_localizer_(std::unique_ptr<MockRelativeLocalizer> && localizer);

  bool8_t register_exception();
  bool8_t map_exception();
  bool8_t register_on_invalid_map();

protected:
  void on_bad_registration(std::exception_ptr eptr) override;

  /// Handle the exceptions during map setting.
  void on_bad_map(std::exception_ptr eptr) override;

  void on_observation_with_invalid_map(TestObservation::ConstSharedPtr msg) override;

private:
  bool8_t m_map_exception{false};
  bool8_t m_register_exception{false};
  bool8_t m_register_on_invalid_map{false};
};

/// Wait until publisher reaches desired num. of subscriptions.
template<typename T>
void wait_for_matched(
  const T & pub_ptr,
  const uint32_t num_expected_subs = 1U,
  std::chrono::milliseconds match_timeout = std::chrono::seconds{10U})
{
  const auto match_start = std::chrono::steady_clock::now();
  // Ensure map publisher has a map that is listening.
  while (pub_ptr->get_subscription_count() < num_expected_subs) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (std::chrono::steady_clock::now() - match_start > match_timeout) {
      throw std::runtime_error("");
    }
  }
}
///// Test exceptions

class TestRegistrationException : public std::exception {};
class TestMapException : public std::exception {};

/// Abstraction functions to tag and track dummy messages.
inline int64_t get_msg_id(const MsgWithHeader & msg)
{
  return msg.transform.translation.x;
}

inline void set_msg_id(MsgWithHeader & msg, int64_t id)
{
  msg.transform.translation.x = id;
}

inline int64_t get_msg_id(const PoseWithCovarianceStamped & msg)
{
  return msg.pose.pose.position.x;
}

inline void set_msg_id(PoseWithCovarianceStamped & msg, int64_t id)
{
  msg.pose.pose.position.x = id;
}

#endif  // TEST_RELATIVE_LOCALIZER_NODE_HPP_
