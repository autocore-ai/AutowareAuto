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
#include <sstream>
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

constexpr auto cmd_topic = "bad_controller_base_node_cmd";
constexpr auto state_topic = "bad_controller_base_node_state";
constexpr auto tf_topic = "bad_controller_base_node_tf";
constexpr auto traj_topic = "bad_controller_base_node_traj";
constexpr auto diagnostic_topic = "bad_controller_base_node_diag";

class BadListener : public rclcpp::Node
{
public:
  BadListener()
  : Node{"bad_controller_base_node_listener"},
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
  virtual ~BadListener() = default;
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
};  // class BadListener

// A simple controller instance to pass-through the transformed stuff
class BadController : public ControllerBase
{
public:
  BadController()
  : ControllerBase{
      BehaviorConfig{3.0F, std::chrono::milliseconds(100L), ControlReference::SPATIAL}}
  {
  }

protected:
  Command compute_command_impl(const State & state) override
  {
    (void)state;
    throw std::logic_error{"Should not hit"};
  }
};  // class BadController

class BadControllerNode : public ControllerBaseNode
{
public:
  BadControllerNode()
  : ControllerBaseNode{
      "bad_controller_base_node",
      "",
      cmd_topic,
      state_topic,
      tf_topic,
      traj_topic,
      diagnostic_topic
  }
  {
    ControllerBaseNode::set_controller(std::make_unique<BadController>());
  }
};  // class BadControllerNode

class BadCases : public ::testing::Test
{
protected:
  // Ugly global because lambdas can only convert to function pointers if they dont capture
  // Also ugly memory leak to avoid -Wexit-time-destructors
  static std::stringstream * ss;

  // Override ROS 2 logging to point to some static ostream which we can then read
  static void fake_log(
    const rcutils_log_location_t *,
    int,
    const char *,
    rcutils_time_point_value_t,
    const char * format,
    va_list *)
  {
    (*ss) << format << "\n";
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    sub_ = std::make_shared<BadListener>();
    ctrl_ = std::make_shared<BadControllerNode>();
    state_pub_ = ctrl_->create_publisher<State>(state_topic, rclcpp::QoS{10LL});
    traj_pub_ = ctrl_->create_publisher<Trajectory>(traj_topic, rclcpp::QoS{10LL});
    sub_->match();
    // TODO(c.ho) more matching
    // Redirect ROS 2 logging to some global stringsteam
    // Clear underlying string AND error bit
    ss->str(std::string{});
    ss->clear();

    rcutils_logging_set_output_handler(BadCases::fake_log);
  }
  void TearDown() override
  {
    // Print out all the junk we picked up during the test
    std::cout << ss->str();
    (void)rclcpp::shutdown();
  }

  rclcpp::Publisher<State>::SharedPtr state_pub_{};
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_{};
  std::shared_ptr<BadListener> sub_{};
  std::shared_ptr<BadControllerNode> ctrl_{};
};  // BadCases

std::stringstream * BadCases::ss = new std::stringstream;

TEST_F(BadCases, EmptyTrajectoryFrame)
{
  // Publish bad trajectory
  Trajectory traj{};
  traj.header.frame_id = "";
  traj.points.resize(3U);
  traj_pub_->publish(traj);
  // Publish some random states
  auto state = make_state(0.0F, 0.0F, -1.0F, 9.0F, 1.0F, 0.0F, std::chrono::system_clock::now());
  state.header.frame_id = "foo";
  ASSERT_FALSE(state.header.frame_id.empty());
  // Spin
  {
    rclcpp::executors::SingleThreadedExecutor exec{};
    exec.add_node(sub_);
    exec.add_node(ctrl_);
    auto count = 0;
    constexpr auto MAX_ITER = 1000;
    while (sub_->trajectories().empty() || sub_->states().empty() || ss->str().empty()) {
      traj_pub_->publish(traj);
      state_pub_->publish(state);
      EXPECT_NO_THROW(exec.spin_once());
      // TODO(fix?)
      if (++count > MAX_ITER) {
        std::cout << "Break due to max iters\n";
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
    }
    // spin one more time for good measure
    EXPECT_NO_THROW(exec.spin_some(std::chrono::milliseconds(100LL)));
  }
  // Should result in no command
  EXPECT_EQ(sub_->commands().size(), 0U);
  // TODO(c.ho) Expect following string:
  // try_compute: empty trajectory frame, ignoring
  // TODO(Takamasa Horibe): enable with RCLCPP_WARN_THROTTLE after Foxy
  // std::string last_line{};
  // (void)std::getline(*ss, last_line);
  // EXPECT_EQ(last_line, "try_compute: empty trajectory frame, possibly uninitialized, deferring");
}

TEST_F(BadCases, EmptyTrajectory)
{
  // Should result in only stop command
  // Publish bad trajectory
  Trajectory traj{};
  traj.header.frame_id = "bar";
  traj.points.clear();
  traj_pub_->publish(traj);
  ASSERT_TRUE(traj.points.empty());
  ASSERT_FALSE(traj.header.frame_id.empty());
  // Publish some random states
  auto state = make_state(0.0F, 0.0F, -1.0F, 9.0F, 1.0F, 0.0F, std::chrono::system_clock::now());
  state.header.frame_id = "bar";
  ASSERT_FALSE(state.header.frame_id.empty());
  ASSERT_EQ(traj.header.frame_id, state.header.frame_id);
  // Spin
  {
    rclcpp::executors::SingleThreadedExecutor exec{};
    exec.add_node(sub_);
    exec.add_node(ctrl_);
    auto count = 0;
    constexpr auto MAX_ITER = 1000;
    while (sub_->trajectories().empty() || sub_->states().empty() || sub_->commands().empty()) {
      traj_pub_->publish(traj);
      state_pub_->publish(state);
      EXPECT_NO_THROW(exec.spin_once());
      if (++count > MAX_ITER) {
        std::cout << "Break due to max iterations\n";
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
    }
    // spin one more time for good measure
    EXPECT_NO_THROW(exec.spin_some(std::chrono::milliseconds(100LL)));
  }
  // Should result in no command
  EXPECT_EQ(sub_->commands().size(), 0U);
  // Check logging output
  std::string last_line{};
  (void)std::getline(*ss, last_line);
  EXPECT_EQ(last_line, "ControllerBase: Zero length trajectory is not expected");
}

TEST_F(BadCases, EmptyStateFrame)
{
  // Publish trajectory
  Trajectory traj{};
  traj.header.frame_id = "bar";
  traj.points.resize(3U);
  traj_pub_->publish(traj);
  // Publish some bad states
  auto state = make_state(0.0F, 0.0F, -1.0F, 0.0F, 1.0F, 0.0F, std::chrono::system_clock::now());
  state.header.frame_id = "";
  ASSERT_TRUE(state.header.frame_id.empty());
  // Spin
  {
    rclcpp::executors::SingleThreadedExecutor exec{};
    exec.add_node(sub_);
    exec.add_node(ctrl_);
    auto count = 0;
    constexpr auto MAX_ITER = 1000;
    while (sub_->trajectories().empty() || sub_->states().empty()) {
      traj_pub_->publish(traj);
      state_pub_->publish(state);
      EXPECT_NO_THROW(exec.spin_once());
      std::this_thread::sleep_for(std::chrono::milliseconds{10LL});
      if (++count > MAX_ITER) {
        std::cout << "Break due to max iters\n";
        break;
      }
    }
    // spin one more time for good measure
    EXPECT_NO_THROW(exec.spin_some(std::chrono::milliseconds(100LL)));
  }
  // Should result in no command
  EXPECT_EQ(sub_->commands().size(), 0U);
  // Check logging output
  std::string last_line{};
  (void)std::getline(*ss, last_line);
  EXPECT_EQ(last_line, "try_compute: empty state frame, ignoring");
}
