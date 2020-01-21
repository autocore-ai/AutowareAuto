// Copyright 2020 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
#ifndef TEST_VI_NODE_HPP_
#define TEST_VI_NODE_HPP_

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "vehicle_interface/vehicle_interface_node.hpp"

using autoware::drivers::vehicle_interface::Limits;
using autoware::drivers::vehicle_interface::VehicleInterfaceNode;
using autoware::drivers::vehicle_interface::PlatformInterface;
using autoware::drivers::vehicle_interface::FilterConfig;
using autoware::drivers::vehicle_interface::TopicNumMatches;

using autoware_auto_msgs::msg::RawControlCommand;
using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware_auto_msgs::msg::VehicleStateCommand;

/// Fake instantiation of interface, only checks that certain things were ever called
/// Each of the overloaded functions fails in a rotating manner. After 5 iterations, the
/// sequence goes: no failures, send raw fails, send basic fails, send state fails, and update fails
class FakeInterface : public PlatformInterface
{
public:
  explicit FakeInterface(bool fail)
  : PlatformInterface{},
    m_fail{fail} {}

  bool update(std::chrono::nanoseconds timeout) override
  {
    (void)timeout;
    m_update_called = true;
    return (m_count % 5) != 4;
  }
  bool send_state_command(const autoware_auto_msgs::msg::VehicleStateCommand & msg) override
  {
    (void)msg;
    m_state_called = true;
    m_state = msg;
    return (m_count % 5) != 3;
  }
  bool send_control_command(const autoware_auto_msgs::msg::VehicleControlCommand & msg) override
  {
    if (m_fail) {
      ++m_count;
    }
    m_basic_called = true;
    m_control = msg;
    m_controls.push_back(msg);
    return (m_count % 5) != 2;
  }
  bool send_control_command(const autoware_auto_msgs::msg::RawControlCommand & msg) override
  {
    if (m_fail) {
      ++m_count;
    }
    // Slightly sketchy because of threading, but I claim it's ok since I stop the threading
    // before checking this
    m_msg = msg;
    m_raw_called = true;
    return (m_count % 5) != 1;
  }

  const RawControlCommand & msg() const noexcept {return m_msg;}
  const VehicleControlCommand & control() const noexcept {return m_control;}
  const VehicleStateCommand & state() const noexcept {return m_state;}
  const std::vector<VehicleControlCommand> & controls() const noexcept {return m_controls;}

  bool update_called() const noexcept {return m_update_called;}
  bool state_called() const noexcept {return m_state_called;}
  bool basic_called() const noexcept {return m_basic_called;}
  bool raw_called() const noexcept {return m_raw_called;}
  int32_t count() const noexcept {return m_count;}

private:
  std::atomic<bool> m_update_called{false};
  std::atomic<bool> m_state_called{false};
  std::atomic<bool> m_basic_called{false};
  std::atomic<bool> m_raw_called{false};
  RawControlCommand m_msg{};
  VehicleControlCommand m_control{};
  VehicleStateCommand m_state{};
  std::vector<VehicleControlCommand> m_controls{};
  int32_t m_count{};
  bool m_fail;
};

/// Simple instantiation of vehicle interface using FakeInterface
class TestVINode : public VehicleInterfaceNode
{
public:
  TestVINode(
    const std::string & node_name,
    const std::string & node_namespace,
    const TopicNumMatches & raw_command,
    const TopicNumMatches & basic_command,
    const TopicNumMatches & high_level_command,
    const TopicNumMatches & state_command,
    const TopicNumMatches & odometry,
    const TopicNumMatches & state_report,
    const FilterConfig & longitudinal_filter,
    const FilterConfig & curvature_filter,
    const FilterConfig & front_steer_filter,
    const FilterConfig & rear_steer_filter,
    bool fail = false)
  : VehicleInterfaceNode{
      node_name,
      node_namespace,
      std::chrono::milliseconds{30LL},
      raw_command,
      basic_command,
      high_level_command,
      state_command,
      odometry,
      state_report,
      autoware::drivers::vehicle_interface::StateMachineConfig{
      0.5F,  // gear shift velocity threshold
      Limits<float>{-3.0F, 3.0F, 1.0F},  // accel limits
      Limits<float>{-0.331F, 0.331F, 0.3F},  // front steer limits
      std::chrono::milliseconds{100LL},  // time_step
      3.0F,  // timeout acceleration
      std::chrono::seconds{3LL},  // state transition timeout
      0.5F,  // gear shift accel deadzone
    },
      longitudinal_filter,
      curvature_filter,
      front_steer_filter,
      rear_steer_filter}
  {
    // sketchy, but this is because the PlatformInterface generally shouldn't be exposed
    auto interface = std::make_unique<FakeInterface>(fail);
    m_interface = interface.get();
    set_interface(std::move(interface));
  }

  const FakeInterface & interface() const noexcept {return *m_interface;}
  bool error_handler_called() const noexcept {return m_error_handler_called;}
  bool control_handler_called() const noexcept {return m_control_send_error_handler_called;}
  bool state_handler_called() const noexcept {return m_state_send_error_handler_called;}
  bool timeout_handler_called() const noexcept {return m_read_timeout_handler_called;}

protected:
  class ControlError : public std::logic_error
  {
public:
    ControlError()
    : logic_error{"control"} {}
  };
  class ReadError : public std::logic_error
  {
public:
    ReadError()
    : logic_error{"read"} {}
  };
  class StateError : public std::logic_error
  {
public:
    StateError()
    : logic_error{"state"} {}
  };
  void on_control_send_failure() override
  {
    static bool flag{false};
    flag = !flag;
    if (flag) {
      throw ControlError{};
    } else {
      VehicleInterfaceNode::on_control_send_failure();
    }
  }
  void on_state_send_failure() override
  {
    static bool flag{false};
    flag = !flag;
    if (flag) {
      throw StateError{};
    } else {
      VehicleInterfaceNode::on_state_send_failure();
    }
  }
  void on_read_timeout() override
  {
    static bool flag{false};
    flag = !flag;
    if (flag) {
      throw ReadError{};
    } else {
      VehicleInterfaceNode::on_read_timeout();
    }
  }
  void on_error(std::exception_ptr eptr) override
  {
    m_error_handler_called = true;
    try {
      std::rethrow_exception(eptr);
    } catch (const std::runtime_error &) {
      // Generic; do nothing
    } catch (const ReadError &) {
      m_read_timeout_handler_called = true;
    } catch (const ControlError &) {
      m_control_send_error_handler_called = true;
    } catch (const StateError &) {
      m_state_send_error_handler_called = true;
    }
  }

private:
  const FakeInterface * m_interface;
  bool m_error_handler_called{false};
  bool m_control_send_error_handler_called{false};
  bool m_state_send_error_handler_called{false};
  bool m_read_timeout_handler_called{false};
};  // class TestVINode

class sanity_checks : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown()
  {
    (void)rclcpp::shutdown();
  }
};

#endif  // TEST_VI_NODE_HPP_
