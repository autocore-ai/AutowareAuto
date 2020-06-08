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

#include "motion_testing_nodes/motion_testing_publisher.hpp"

#include <motion_common/motion_common.hpp>
#include <motion_testing/motion_testing.hpp>
#include <time_utils/time_utils.hpp>

#include <memory>
#include <string>
#include <vector>

#include "motion_testing_nodes/wait_for_matched.hpp"

namespace motion
{
namespace motion_testing_nodes
{

using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;

MotionTestingPublisher::MotionTestingPublisher(
  const std::string & node_name,
  const std::string & traj_topic,
  const std::string & state_topic,
  const std::string & tf_topic,
  const std::vector<TrajectoryProfile> & profiles)
: Node(node_name, rclcpp::NodeOptions{rcl_get_default_allocator()}),
  m_tf_pub{create_publisher<TFMessage>(
      tf_topic, rclcpp::QoS{10}.transient_local(), PubAllocT{})},
  m_state_pub{create_publisher<State>(
      state_topic, rclcpp::QoS{10}.transient_local(), PubAllocT{})},
  m_traj_pub{create_publisher<Trajectory>(
      traj_topic, rclcpp::QoS{10}.transient_local(), PubAllocT{})},
  m_timer{nullptr}
{
  if (profiles.empty()) {
    throw std::domain_error{"Profiles cannot be empty"};
  }
  // Build timer
  m_timer = create_wall_timer(min_period(profiles), [this]() -> void {this->on_timer();}, nullptr);
  // Precompute trajectories, transforms, and transforms
  const auto & s0 = profiles[0].start_state;
  auto x0 = s0.state.x;
  auto y0 = s0.state.y;
  auto yaw0 = s0.state.heading;
  auto v0 = s0.state.longitudinal_velocity_mps;
  auto t0 = s0.header.stamp;
  for (const auto & prof : profiles) {
    if (prof.state_update_period != prof.trajectory_sample_period) {
      throw std::domain_error{"Update period != trajectory sample period is not yet supported"};
    }
    m_state_period.push_back(prof.state_update_period);
    m_traj_period.push_back(prof.active_time);
    const auto & prof_s0 = prof.start_state;
    auto s = motion_testing::make_state(x0, y0,
        motion_common::to_angle(yaw0),
        v0,
        prof_s0.state.acceleration_mps2,
        prof_s0.state.heading_rate_rps,
        time_utils::from_message(t0));
    // Trajectory
    m_trajectories.emplace_back(
      motion_testing::constant_trajectory(s, prof.trajectory_sample_period));
    auto & traj_tmp = m_trajectories.back();
    traj_tmp.header.frame_id = prof.trajectory_frame;
    traj_tmp.header.stamp = prof_s0.header.stamp;
    const auto & traj = traj_tmp;
    // Find end
    const auto end_idx = end_index(prof, traj);
    // set s0
    const auto end_pt = traj.points[end_idx];
    x0 = end_pt.x;
    y0 = end_pt.y;
    yaw0 = end_pt.heading;
    v0 = end_pt.longitudinal_velocity_mps;
    for (auto idx = 0U; idx < end_idx; ++idx) {
      const auto & pt = traj.points[idx];
      add_state_and_tf(traj.header, prof, pt);
    }
  }
  m_last_traj_time = std::chrono::system_clock::now();
  m_last_state_time = std::chrono::system_clock::now();
}

void MotionTestingPublisher::match(
  const std::size_t tf_subs,
  const std::size_t state_subs,
  const std::size_t traj_subs) const
{
  std::chrono::nanoseconds timeout = std::chrono::seconds{10LL};
  timeout = wait_for_matched(*m_tf_pub, tf_subs, timeout);
  timeout = wait_for_matched(*m_state_pub, state_subs, timeout);
  (void)wait_for_matched(*m_traj_pub, traj_subs, timeout);
  // WARNING Dumb hack: publish trajectory early
  {
    m_traj_pub->publish(m_trajectories.front());
    auto tf = m_tfs.front();
    tf.transforms[0].header.stamp = time_utils::to_message(
      time_utils::from_message(tf.transforms[0].header.stamp) -
      std::chrono::milliseconds(2LL));
    m_tf_pub->publish(tf);
    m_tf_pub->publish(m_tfs.front());
  }
}

////////////////////////////////////////////////////////////////////////////////
std::chrono::nanoseconds MotionTestingPublisher::min_period(
  const TrajectoryProfilesVec & profiles) const noexcept
{
  auto min_time = std::chrono::nanoseconds::max();
  for (const auto & prof : profiles) {
    const auto f = [&min_time](auto val) {
        if (val < min_time) {
          min_time = val;
        }
      };
    f(prof.active_time);
    f(prof.trajectory_sample_period);
    f(prof.state_update_period);
  }
  return min_time;
}

////////////////////////////////////////////////////////////////////////////////
std::size_t MotionTestingPublisher::end_index(
  const TrajectoryProfile & prof, const Trajectory & traj) const noexcept
{
  auto end_idx = 0U;
  for (auto idx = 0U; idx < traj.points.size(); ++idx) {
    if (time_utils::from_message(traj.points[idx].time_from_start) >= prof.active_time) {
      end_idx = idx;
      break;
    }
  }
  return end_idx;
}

////////////////////////////////////////////////////////////////////////////////
void MotionTestingPublisher::add_state_and_tf(
  const Header & header,
  const TrajectoryProfile & prof,
  const TrajectoryPoint & pt) noexcept
{
  // Make transforms
  TransformStamped tf{rosidl_generator_cpp::MessageInitialization::ALL};
  tf.header.stamp = time_utils::to_message(
    time_utils::from_message(header.stamp) +
    time_utils::from_message(pt.time_from_start) +
    std::chrono::milliseconds(1LL));
  tf.header.frame_id = prof.trajectory_frame;
  tf.child_frame_id = prof.ego_frame;
  using Real = double;
  tf.transform.translation.x = static_cast<Real>(pt.x);
  tf.transform.translation.y = static_cast<Real>(pt.y);
  tf.transform.translation.z = {};
  tf.transform.rotation.x = {};
  tf.transform.rotation.y = {};
  tf.transform.rotation.z = static_cast<Real>(pt.heading.imag);
  tf.transform.rotation.w = static_cast<Real>(pt.heading.real);
  TFMessage msg{rosidl_generator_cpp::MessageInitialization::ALL};
  if (tf.child_frame_id != tf.header.frame_id) {
    msg.transforms.push_back(tf);
    m_tfs.push_back(msg);
  }
  // Make states
  State s{rosidl_generator_cpp::MessageInitialization::ALL};
  s.state = pt;
  s.header.stamp = time_utils::to_message(
    time_utils::from_message(header.stamp) +
    time_utils::from_message(pt.time_from_start));
  s.header.frame_id = prof.ego_frame;
  s.state.x = {};
  s.state.y = {};
  s.state.heading.real = 1.0F;
  s.state.heading.imag = {};
  m_states.push_back(s);
}

////////////////////////////////////////////////////////////////////////////////
bool MotionTestingPublisher::done() const noexcept
{
  return m_states.empty() && m_trajectories.empty() && m_tfs.empty();
}

////////////////////////////////////////////////////////////////////////////////
void MotionTestingPublisher::on_timer()
{
  const auto try_pub = [](const auto pub, auto & list) -> void {
      if (!list.empty()) {
        const auto & val = list.front();
        pub->publish(val);
        list.pop_front();
      }
    };
  // Check trajectory
  const auto now = std::chrono::system_clock::now();
  if (now - m_last_traj_time >= m_traj_period.front()) {
    try_pub(m_traj_pub, m_trajectories);
    m_last_traj_time = now;
    if (m_traj_period.size() > 1U) {
      m_traj_period.pop_front();
    }
    if (m_state_period.size() > 1U) {
      m_state_period.pop_front();
    }
  }
  // Check state
  if (now - m_last_state_time >= m_state_period.front()) {
    try_pub(m_state_pub, m_states);
    try_pub(m_tf_pub, m_tfs);
    m_last_state_time = now;
  }
}
}  // namespace motion_testing_nodes
}  // namespace motion
