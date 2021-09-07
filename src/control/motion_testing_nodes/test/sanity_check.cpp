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

class Listener : public rclcpp::Node
{
public:
  Listener(
    const std::string & node_name,
    const std::string & traj_topic,
    const std::string & state_topic,
    const std::string & tf_topic)
  : Node{node_name},
    m_trajectories{},
    m_states{},
    m_tfs{},
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

private:
  std::vector<Trajectory> m_trajectories;
  std::vector<State> m_states;
  std::vector<TransformStamped> m_tfs;
  rclcpp::Subscription<Trajectory>::SharedPtr m_traj_sub;
  rclcpp::Subscription<State>::SharedPtr m_state_sub;
  rclcpp::Subscription<TFMessage>::SharedPtr m_tf_sub;
};  // class Listener

TEST_F(SanityCheck, Basic)
{
  const auto tf_topic = "motion_testing_node_tf";
  const auto traj_topic = "motion_test_node_traj";
  const auto state_topic = "motion_testing_node_state";
  // Exemplar states: simple accelerate-constant-decelerate profile
  const auto state1 =
    make_state(0.0F, 0.0F, -1.0F, 0.0F, 1.0F, 0.0F, std::chrono::system_clock::now());
  const auto state2 =
    make_state(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, std::chrono::system_clock::now());
  const auto state3 =
    make_state(0.0F, 0.0F, 0.0F, 0.0F, -0.2F, 0.0F, std::chrono::system_clock::now());
  // Publisher Node
  const auto ms100 = std::chrono::milliseconds(100LL);
  const auto pub = std::make_shared<motion::motion_testing_nodes::MotionTestingPublisher>(
    "test_motion_testing_node",
    traj_topic,
    state_topic,
    tf_topic,
    std::vector<TrajectoryProfile>{
    TrajectoryProfile{state1, 5 * ms100, ms100, ms100, "foo", "bar"},
    TrajectoryProfile{state2, 10 * ms100, ms100, ms100, "baz", "bar"},
    TrajectoryProfile{state3, 15 * ms100, 2 * ms100, 2 * ms100, "foo", "foo"}
  }
  );
  // Listener node
  const auto sub = std::make_shared<Listener>(
    "test_motion_testing_node_listener",
    traj_topic,
    state_topic,
    tf_topic);
  // Spin
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(pub);
    exec.add_node(sub);
    while (!pub->done()) {
      exec.spin_some(std::chrono::milliseconds(10LL));
    }
    // spin one more time for good measure
    exec.spin_some(std::chrono::milliseconds(100LL));
  }
  // Check results
  {
    // Trajectories
    const auto & trajs = sub->trajectories();
    EXPECT_EQ(trajs.size(), 3U);
    // TODO(c.ho) more checks
  }
  const auto total_msgs = 5 + 10 + (15 / 2);
  constexpr auto TOL = 1.0E-3F;
  constexpr auto TOLD = static_cast<double>(TOL);
  {
    // Tfs
    const auto & tfs = sub->tfs();
    // TFs with same fram don't get published--doesn't make sense
    EXPECT_LE(labs(static_cast<decltype(total_msgs)>(tfs.size()) - (5 + 10)), 2) << tfs.size();
    for (auto idx = 1U; idx < tfs.size(); ++idx) {
      const auto & tf_curr = tfs[idx];
      EXPECT_LT(fabs(tf_curr.transform.translation.z), TOLD);
      EXPECT_LT(fabs(tf_curr.transform.rotation.y), TOLD);
      EXPECT_LT(fabs(tf_curr.transform.rotation.x), TOLD);
      EXPECT_LT(
        fabs(tf_curr.transform.rotation.w - static_cast<double>(state1.state.heading.real)), TOLD);
      EXPECT_LT(
        fabs(tf_curr.transform.rotation.z - static_cast<double>(state1.state.heading.imag)), TOLD);
      const auto & tf_prev = tfs[idx - 1U];
      EXPECT_GE(tf_curr.transform.translation.x, tf_prev.transform.translation.x) << idx;
      EXPECT_LE(tf_curr.transform.translation.y, tf_prev.transform.translation.y) << idx;
    }
  }
  {
    // States
    const auto & states = sub->states();
    EXPECT_LE(labs(static_cast<decltype(total_msgs)>(states.size()) - total_msgs), 2);
    for (const auto & s : states) {
      EXPECT_LT(fabsf(s.state.x), TOL);
      EXPECT_LT(fabsf(s.state.y), TOL);
      // Heading is assumed to be 0
      EXPECT_LT(fabsf(s.state.heading.real - 1.0F), TOL);
      EXPECT_LT(fabsf(s.state.heading.imag), TOL);
      // velocity etc
    }
  }
}
