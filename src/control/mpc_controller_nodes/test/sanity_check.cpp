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
#include <motion_testing/motion_testing.hpp>
#include <motion_testing_nodes/motion_testing_publisher.hpp>
#include <motion_testing_nodes/wait_for_matched.hpp>
#include <mpc_controller_nodes/mpc_controller_node.hpp>
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
using motion::control::controller_common_nodes::ControllerPtr;
using motion::control::mpc_controller::Real;
using motion::control::mpc_controller::StateWeight;
using motion::control::mpc_controller::Config;
using motion::control::mpc_controller::BehaviorConfig;
using motion::control::mpc_controller::LimitsConfig;
using motion::control::mpc_controller::OptimizationConfig;
using motion::control::mpc_controller::VehicleConfig;
using motion::control::mpc_controller::Interpolation;
using motion::control::mpc_controller::MpcController;

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

constexpr auto cmd_topic = "test_mpc_ctrl_node_cmd";
constexpr auto state_topic = "test_mpc_ctrl_node_state";
constexpr auto tf_topic = "test_mpc_ctrl_node_tf";
constexpr auto traj_topic = "test_mpc_ctrl_node_traj";
constexpr auto diagnostic_topic = "test_mpc_ctrl_node_diag";

class Listener : public rclcpp::Node
{
public:
  Listener()
  : Node{"test_mpc_ctrl_node_listener"},
    m_trajectories{},
    m_states{},
    m_tfs{},
    m_cmds{},
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

  void match() const
  {
    (void)::motion::motion_testing_nodes::wait_for_matched(*m_traj_sub, 1U);
    (void)::motion::motion_testing_nodes::wait_for_matched(*m_state_sub, 1U);
    (void)::motion::motion_testing_nodes::wait_for_matched(*m_tf_sub, 1U);
    (void)::motion::motion_testing_nodes::wait_for_matched(*m_cmd_sub, 1U);
  }

private:
  std::vector<Trajectory> m_trajectories;
  std::vector<State> m_states;
  std::vector<TransformStamped> m_tfs;
  std::vector<Command> m_cmds;
  rclcpp::Subscription<Trajectory>::SharedPtr m_traj_sub;
  rclcpp::Subscription<State>::SharedPtr m_state_sub;
  rclcpp::Subscription<TFMessage>::SharedPtr m_tf_sub;
  rclcpp::Subscription<Command>::SharedPtr m_cmd_sub;
};  // class Listener


TEST_F(SanityCheck, Basic)
{
  constexpr auto total_msgs = 10U;
  constexpr auto TOLI = 2U;
  // Exemplar states: simple accelerate-constant-decelerate profile
  const auto state1 =
    make_state(0.0F, 0.0F, 1.0F, 10.0F, 0.0F, 0.0F, std::chrono::system_clock::now());
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
  const Config cfg{
    LimitsConfig{
      // Real Bounds
      {0.01F, 35.0F},  // Longitudinal velocity
      {-3.0F, 3.0F},  // Lateral velocity
      {-3.0F, 3.0F},  // Acceleation
      {-3.0F, 3.0F},  // yaw rate
      {-10.0F, 10.0F},  // Jerk
      {-0.331F, 0.331F},  // Steer angle
      {-0.331F, 0.331F}  // Steer angle rate
    },
    VehicleConfig{
      // Parameters from LaValle
      1.2F,  // CG to front
      1.5F,  // CG to rear
      17000.0F,  // front cornering
      20000.0F,  // rear cornering
      1460.0F,  // mass
      2170.0F,  // Inertia
      2.0F,  // width
      0.5F,  // front overhang
      0.5F  // rear overhang
    },
    BehaviorConfig{
      3.0F,  // Stop rate
      std::chrono::milliseconds(100LL),  // time step
      ControlReference::SPATIAL},
    OptimizationConfig{
      StateWeight{
        10.0F,  // pose
        10.0F,  // heading
        10.0F,  // longitudinal velocity
        10.0F,  // lateral velocity
        10.0F,  // yaw rate
        10.0F,  // acceleration
        10.0F,  // jerk
        10.0F,  // steer angle
        10.0F  // steer angle rate
      },
      StateWeight{
        1000.0F,  // pose
        1000.0F,  // heading
        1000.0F,  // longitudinal velocity
        1000.0F,  // lateral velocity
        1000.0F,  // yaw rate
        1000.0F,  // acceleration
        1000.0F,  // jerk
        1000.0F,  // steer angle
        1000.0F  // steer angle rate
      }
    },
    std::chrono::milliseconds(5LL),  // sample_period_tolerance
    std::chrono::milliseconds(100LL),  // control_lookahead_duration
    Interpolation::NO
  };

  using motion::control::mpc_controller_nodes::MpcControllerNode;
  const auto ctrl = std::make_shared<MpcControllerNode>(
    "mpc_ctrl_test_node",
    "",
    cmd_topic,
    state_topic,
    tf_topic,
    traj_topic,
    diagnostic_topic,
    "static_tf_mpc_test",
    cfg);
  // match
  sub->match();
  pub->match(2U, 2U, 2U);
  // Spin
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(pub);
    exec.add_node(sub);
    exec.add_node(ctrl);
    auto got_enough_commands = false;
    auto got_enough_states = false;
    auto got_enough_tfs = false;
    while (!pub->done() ||
      !got_enough_commands ||
      !got_enough_states ||
      !got_enough_tfs)
    {
      exec.spin_some(std::chrono::milliseconds(10LL));
      got_enough_commands = sub->commands().size() + TOLI > total_msgs;
      got_enough_tfs = sub->tfs().size() + TOLI > total_msgs;
      got_enough_states = sub->states().size() + TOLI > total_msgs;
    }
    // spin one more time for good measure
    exec.spin_some(std::chrono::milliseconds(100LL));
  }
  // Check results
  {
    // Trajectories
    const auto & trajs = sub->trajectories();
    EXPECT_EQ(trajs.size(), 2U);  // initial publish and nominal publish
    const auto & tfs = sub->tfs();
    EXPECT_GT(tfs.size() + TOLI, total_msgs) << tfs.size();
    const auto & states = sub->states();
    EXPECT_GT(states.size() + TOLI, total_msgs) << states.size();
    // TODO(c.ho) more checks
  }
  constexpr auto TOL = 1.0E-3F;
  // commands
  const auto & cmds = sub->commands();
  EXPECT_GT(cmds.size() + TOLI, total_msgs) << cmds.size();
  // TODO(c.ho) match against tfs
  auto idx = 0U;
  for (const auto & cmd : cmds) {
    // Just needs to be approximate: there's some temporal offset making things slightly off
    // Acceleration should be smaller than only acceleration limit
    // since jerk constraint is not supported yet
    EXPECT_LT(fabsf(cmd.long_accel_mps2 - 0.0F), 3.0F) << idx;
    EXPECT_LT(fabsf(cmd.front_wheel_angle_rad - 0.0F), TOL) << idx;
    EXPECT_LT(fabsf(cmd.rear_wheel_angle_rad - 0.0F), TOL) << idx;  // real
    ++idx;
  }
}
