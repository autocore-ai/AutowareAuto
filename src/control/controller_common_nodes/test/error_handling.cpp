// Copyright 2019 Christopher Ho
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
#include <gtest/gtest.h>
#include <controller_common_nodes/controller_base_node.hpp>
#include <motion_testing/motion_testing.hpp>
#include <motion_testing_nodes/motion_testing_publisher.hpp>
#include <motion_testing_nodes/wait_for_matched.hpp>
#include <time_utils/time_utils.hpp>

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

using geometry_msgs::msg::TransformStamped;
using motion::motion_testing_nodes::TFMessage;
using motion::motion_testing_nodes::Trajectory;
using motion::motion_testing_nodes::TrajectoryProfile;
using motion::motion_testing::make_state;
using time_utils::from_message;
using motion::control::controller_common::BehaviorConfig;
using motion::control::controller_common::ControlReference;
using motion::control::controller_common::ControllerBase;
using motion::control::controller_common_nodes::ControllerBaseNode;
using motion::control::controller_common_nodes::Command;
using motion::control::controller_common_nodes::Diagnostic;
using motion::control::controller_common_nodes::State;
using motion::control::controller_common_nodes::ControllerPtr;

class ErrorHandling : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown()
  {
    (void)rclcpp::shutdown();
  }
};  // ErrorHandling

constexpr auto cmd_topic = "test_error_controller_node_cmd";
constexpr auto state_topic = "test_error_controller_node_state";
constexpr auto tf_topic = "test_error_controller_node_tf";
constexpr auto traj_topic = "test_error_controller_node_traj";
constexpr auto diagnostic_topic = "test_error_controller_node_diag";

class command_error : public std::exception {};
class trajectory_error : public std::exception {};

// A simple controller instance to pass-through the transformed stuff
class ErrorController : public ControllerBase
{
public:
  ErrorController()
  : ControllerBase{BehaviorConfig{3.0F, std::chrono::milliseconds(100L), ControlReference::SPATIAL}}
  {
  }

protected:
  Command compute_command_impl(const State & state) override
  {
    (void)state;
    throw command_error{};
  }

  bool check_new_trajectory(const Trajectory & trajectory) const override
  {
    if ("map" == trajectory.header.frame_id) {
      throw trajectory_error{};
    }
    return true;
  }
};  // class ErrorController

class ErrorControllerNode : public ControllerBaseNode
{
public:
  ErrorControllerNode()
  : ControllerBaseNode{
      "test_error_controller_node",
      "",
      cmd_topic,
      state_topic,
      tf_topic,
      traj_topic,
      diagnostic_topic
  }
  {
    ControllerBaseNode::set_controller(std::make_unique<ErrorController>());
  }
  bool control_error() const {return m_control_error;}
  bool traj_error() const {return m_trajectory_error;}

protected:
  void on_bad_trajectory(std::exception_ptr eptr) override
  {
    try {
      std::rethrow_exception(eptr);
    } catch (const trajectory_error &) {
      m_trajectory_error = true;
    }
  }

  void on_bad_compute(std::exception_ptr eptr) override
  {
    try {
      std::rethrow_exception(eptr);
    } catch (const command_error &) {
      m_control_error = true;
    }
  }

private:
  bool m_control_error{false};
  bool m_trajectory_error{false};
};  // class ErrorControllerNode

class ErrorListener : public rclcpp::Node
{
public:
  ErrorListener()
  : Node{"test_error_controller_node_listener"},
    m_state_sub{create_subscription<State>(
        state_topic, rclcpp::QoS{50},
        [this](const State::SharedPtr msg) {
          m_states.push_back(*msg);
        })},
    m_diag_sub{create_subscription<Diagnostic>(
      diagnostic_topic, rclcpp::QoS{50},
      [this](const Diagnostic::SharedPtr msg) {
        m_diags.push_back(*msg);
      })}
  {
  }
  virtual ~ErrorListener() = default;
  const std::vector<State> & states() const
  {
    return m_states;
  }
  const std::vector<Diagnostic> & diagnostics() const
  {
    return m_diags;
  }

  void match() const
  {
    (void)::motion::motion_testing_nodes::wait_for_matched(*m_state_sub, 1U);
    (void)::motion::motion_testing_nodes::wait_for_matched(*m_diag_sub, 1U);
  }

private:
  std::vector<State> m_states{};
  std::vector<Diagnostic> m_diags{};
  rclcpp::Subscription<State>::SharedPtr m_state_sub;
  rclcpp::Subscription<Diagnostic>::SharedPtr m_diag_sub;
};  // class ErrorListener


TEST_F(ErrorHandling, Basic)
{
  constexpr auto total_msgs = 10U;
  constexpr auto TOLI = 2U;
  // Exemplar states: simple accelerate-constant-decelerate profile
  const auto state1 =
    make_state(0.0F, 0.0F, -1.0F, 0.0F, 1.0F, 0.0F, std::chrono::system_clock::now());
  // Publisher Node
  const auto ms100 = std::chrono::milliseconds(100LL);
  const auto pub = std::make_shared<motion::motion_testing_nodes::MotionTestingPublisher>(
    "test_error_controller_pub",
    traj_topic,
    state_topic,
    tf_topic,
    std::vector<TrajectoryProfile>{
    TrajectoryProfile{state1, 10 * ms100, ms100, ms100, "odom", "base_link"},
    TrajectoryProfile{state1, 10 * ms100, ms100, ms100, "map", "base_link"}
  }
  );
  // ErrorListener node
  const auto sub = std::make_shared<ErrorListener>();
  // Controller node
  const auto ctrl = std::make_shared<ErrorControllerNode>();
  EXPECT_FALSE(ctrl->control_error());
  EXPECT_FALSE(ctrl->traj_error());
  // Match
  pub->match(1U, 2U, 1U);
  sub->match();
  // Spin
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(pub);
    exec.add_node(sub);
    exec.add_node(ctrl);
    while (!pub->done() || (sub->diagnostics().size() + TOLI < total_msgs)) {
      EXPECT_NO_THROW(exec.spin_some(std::chrono::milliseconds(10LL)));
    }
    // spin one more time for good measure
    EXPECT_NO_THROW(exec.spin_some(std::chrono::milliseconds(100LL)));
  }
  // Check results
  EXPECT_TRUE(ctrl->control_error());
  EXPECT_TRUE(ctrl->traj_error());
  {
    // States
    const auto & states = sub->states();
    EXPECT_GT(states.size(), 0U);
    // diagnostics: just a coarse check, got _something_
    const auto & diags = sub->diagnostics();
    EXPECT_GT(diags.size() + TOLI, total_msgs) <<
      states.size() << ", " << diags.size();
    // ^^ On bad trajectory case, there's transforms to support the new state messages
    // So it doesn't even get executed
  }
}
