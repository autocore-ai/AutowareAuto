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
using motion::motion_testing_nodes::State;
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
using motion::control::controller_common_nodes::State;
using motion::control::controller_common_nodes::Diagnostic;
using motion::control::controller_common_nodes::ControllerPtr;

class SanityCheck : public ::testing::Test
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
};  // SanityCheck

constexpr auto cmd_topic = "test_controller_base_node_cmd";
constexpr auto state_topic = "test_controller_base_node_state";
constexpr auto tf_topic = "test_controller_base_node_tf";
constexpr auto traj_topic = "test_controller_base_node_traj";
constexpr auto diagnostic_topic = "test_controller_base_node_diag";

class Listener : public rclcpp::Node
{
public:
  Listener()
  : Node{"test_controller_base_node_listener"},
    m_traj_sub{create_subscription<Trajectory>(
        traj_topic, rclcpp::QoS{5},
        [this](const Trajectory::SharedPtr msg) {
          m_trajectories.push_back(*msg);
        })},
    m_state_sub{create_subscription<State>(
      state_topic, rclcpp::QoS{50},
      [this](const State::SharedPtr msg) {
        m_states.push_back(*msg);
      })},
  m_tf_sub{create_subscription<TFMessage>(
      tf_topic, rclcpp::QoS{50},
      [this](const TFMessage::SharedPtr msg) {
        for (const auto & tf : msg->transforms) {
          m_tfs.push_back(tf);
        }
      })},
  m_cmd_sub{create_subscription<Command>(
      cmd_topic, rclcpp::QoS{50},
      [this](const Command::SharedPtr msg) {
        m_cmds.push_back(*msg);
      })},
  m_diag_sub{create_subscription<Diagnostic>(
      diagnostic_topic, rclcpp::QoS{50},
      [this](const Diagnostic::SharedPtr msg) {
        m_diags.push_back(*msg);
      })}
  {
  }
  virtual ~Listener() = default;
  const std::vector<Trajectory> & trajectories() const
  {
    return m_trajectories;
  }
  const std::vector<State> & states() const
  {
    return m_states;
  }
  const std::vector<TransformStamped> & tfs() const
  {
    return m_tfs;
  }
  const std::vector<Command> & commands() const
  {
    return m_cmds;
  }
  const std::vector<Diagnostic> & diagnostics() const
  {
    return m_diags;
  }

  void match() const
  {
    (void)::motion::motion_testing_nodes::wait_for_matched(*m_traj_sub, 1U);
    (void)::motion::motion_testing_nodes::wait_for_matched(*m_state_sub, 1U);
    (void)::motion::motion_testing_nodes::wait_for_matched(*m_tf_sub, 1U);
    (void)::motion::motion_testing_nodes::wait_for_matched(*m_cmd_sub, 1U);
    (void)::motion::motion_testing_nodes::wait_for_matched(*m_diag_sub, 1U);
  }

private:
  std::vector<Trajectory> m_trajectories{};
  std::vector<State> m_states{};
  std::vector<TransformStamped> m_tfs{};
  std::vector<Command> m_cmds{};
  std::vector<Diagnostic> m_diags{};
  rclcpp::Subscription<Trajectory>::SharedPtr m_traj_sub;
  rclcpp::Subscription<State>::SharedPtr m_state_sub;
  rclcpp::Subscription<TFMessage>::SharedPtr m_tf_sub;
  rclcpp::Subscription<Command>::SharedPtr m_cmd_sub;
  rclcpp::Subscription<Diagnostic>::SharedPtr m_diag_sub;
};  // class Listener

// A simple controller instance to pass-through the transformed stuff
class TestController : public ControllerBase
{
public:
  TestController()
  : ControllerBase{
      BehaviorConfig{3.0F, std::chrono::milliseconds(100L), ControlReference::SPATIAL}}
  {
  }

protected:
  Command compute_command_impl(const State & state) override
  {
    Command ret{};
    ret.long_accel_mps2 = state.state.x;
    ret.front_wheel_angle_rad = state.state.y;
    ret.rear_wheel_angle_rad = state.state.heading.real;
    ret.stamp = state.header.stamp;
    return ret;
  }
};  // class TestController

class TestControllerNode : public ControllerBaseNode
{
public:
  TestControllerNode()
  : ControllerBaseNode{
      "test_controller_base_node",
      "",
      cmd_topic,
      state_topic,
      tf_topic,
      traj_topic,
      diagnostic_topic
  }
  {
    ControllerBaseNode::set_controller(std::make_unique<TestController>());
  }
};  // class TestControllerNode

TEST_F(SanityCheck, Basic)
{
  const auto total_msgs = 10U;
  constexpr auto TOL = 1.0E-3F;
  constexpr auto TOLD = static_cast<double>(TOL);
  constexpr auto TOLI = 2U;
  // Exemplar states: simple accelerate-constant-decelerate profile
  const auto state1 =
    make_state(0.0F, 0.0F, -1.0F, 0.0F, 1.0F, 0.0F, std::chrono::system_clock::now());
  // Publisher Node
  const auto ms100 = std::chrono::milliseconds(100LL);
  const auto pub = std::make_shared<motion::motion_testing_nodes::MotionTestingPublisher>(
    "test_motion_testing_node",
    traj_topic,
    state_topic,
    tf_topic,
    std::vector<TrajectoryProfile>{
    TrajectoryProfile{state1, 10 * ms100, ms100, ms100, "odom", "base_link"}
  }
  );
  // Listener node
  const auto sub = std::make_shared<Listener>();
  // Controller node
  const auto ctrl = std::make_shared<TestControllerNode>();
  // Match
  sub->match();
  pub->match(2U, 2U, 2U);
  // Spin
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(pub);
    exec.add_node(sub);
    exec.add_node(ctrl);
    auto got_enough_commands = false;
    auto got_enough_tfs = false;
    auto got_enough_states = false;
    while (!pub->done() ||
      !got_enough_commands ||
      !got_enough_tfs ||
      !got_enough_states)
    {
      exec.spin_some(std::chrono::milliseconds(10LL));
      got_enough_commands = sub->commands().size() + TOLI > total_msgs;
      got_enough_tfs = sub->tfs().size() + TOLI > total_msgs;
      got_enough_states = sub->states().size() + TOLI > total_msgs;
      // got_enough_trajectories = sub->trajectories().size() + TOLI > total_msgs;
    }
    // spin one more time for good measure
    exec.spin_some(std::chrono::milliseconds(100LL));
  }
  // Check results
  {
    // Trajectories
    const auto & trajs = sub->trajectories();
    EXPECT_EQ(trajs.size(), 2U);  // Initial publish, nominal publish
    // TODO(c.ho) more checks
  }
  // Cpmmand, and tf should have consistent values, state should be zero
  {
    // commands
    const auto & cmds = sub->commands();
    EXPECT_GT(cmds.size() + TOLI, total_msgs) << cmds.size();
    // TODO(c.ho) match against tfs
    for (auto idx = 1U; idx < cmds.size(); ++idx) {
      const auto & cmd_curr = cmds[idx];
      const auto & cmd_prev = cmds[idx - 1U];
      EXPECT_GE(cmd_curr.long_accel_mps2, cmd_prev.long_accel_mps2) << idx;  // x
      EXPECT_LE(cmd_curr.front_wheel_angle_rad, cmd_prev.front_wheel_angle_rad) << idx;  // y
      EXPECT_LT(fabsf(cmd_curr.rear_wheel_angle_rad - state1.state.heading.real), TOL);  // real
      // TODO(c.ho) rotation, etc.
    }
    // Tfs
    const auto & tfs = sub->tfs();
    EXPECT_GT(tfs.size() + TOLI, total_msgs) << tfs.size();
    for (auto idx = 1U; idx < tfs.size(); ++idx) {
      const auto & tf_curr = tfs[idx];
      EXPECT_LT(fabs(tf_curr.transform.translation.z), TOLD);
      EXPECT_LT(fabs(tf_curr.transform.rotation.y), TOLD);
      EXPECT_LT(fabs(tf_curr.transform.rotation.x), TOLD);
      EXPECT_LT(fabs(tf_curr.transform.rotation.w - state1.state.heading.real), TOLD);
      EXPECT_LT(fabs(tf_curr.transform.rotation.z - state1.state.heading.imag), TOLD);
      const auto & tf_prev = tfs[idx - 1U];
      EXPECT_GE(tf_curr.transform.translation.x, tf_prev.transform.translation.x) << idx;
      EXPECT_LE(tf_curr.transform.translation.y, tf_prev.transform.translation.y) << idx;
      // TODO(c.ho) rotation, etc.
    }
    // No strict equality check because tf's need interpolation..
  }
  {
    // States
    const auto & states = sub->states();
    EXPECT_GT(states.size() + TOLI, total_msgs) << states.size();
    for (const auto & s : states) {
      EXPECT_LT(fabsf(s.state.x), TOL);
      EXPECT_LT(fabsf(s.state.y), TOL);
      // Heading is assumed to be 0
      EXPECT_LT(fabsf(s.state.heading.real - 1.0F), TOL);
      EXPECT_LT(fabsf(s.state.heading.imag), TOL);
      // velocity etc
    }
    // diagnostics: just a coarse check, got _something_
    const auto & diags = sub->diagnostics();
    EXPECT_LE(std::abs(static_cast<int>(states.size()) - static_cast<int>(diags.size())), 2) <<
      states.size() << ", " << diags.size();
  }
}
