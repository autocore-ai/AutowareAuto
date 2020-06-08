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

#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>

#include <motion_testing/motion_testing.hpp>
#include <motion_testing_nodes/periodic_publisher.hpp>
#include <time_utils/time_utils.hpp>

#include <list>
#include <memory>
#include <string>

using motion::motion_testing_nodes::PeriodCountTopic;

using Command = autoware_auto_msgs::msg::VehicleControlCommand;
using State = autoware_auto_msgs::msg::VehicleKinematicState;
using Trajectory = autoware_auto_msgs::msg::Trajectory;

using motion::planning::planning_common::EnvironmentConfig;

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher(
    const std::string & name,
    const PeriodCountTopic & ego,
    const PeriodCountTopic & target)
  : Node{name}
  {
    using motion::motion_testing::make_state;
    const auto frame = "baz";
    const auto now = std::chrono::system_clock::now();
    {
      std::list<State> egos{};
      for (auto idx = 0U; idx < ego.count; ++idx) {
        const auto idf = static_cast<float>(idx);
        auto s = make_state(idf, {}, {}, {}, {}, {}, now + (idx * ego.period));
        s.header.frame_id = frame;
        egos.emplace_back(s);
      }
      m_ego_pub = std::make_unique<Pub>(*this, ego, egos);
    }
    {
      std::list<State> targets{};
      const auto target_x = static_cast<float>(ego.count + 3U);
      for (auto idx = 0U; idx < target.count; ++idx) {
        auto s = make_state(target_x, {}, {}, {}, {}, {}, now + (idx * target.period));
        s.header.frame_id = frame;
        targets.emplace_back(s);
      }
      m_target_pub = std::make_unique<Pub>(*this, target, targets);
    }
  }

  bool done() const noexcept {return m_ego_pub->done() && m_target_pub->done();}

  void match() const
  {
    // For the purposes of this test, it's one
    m_ego_pub->match(1U);
    m_target_pub->match(1U);
  }

private:
  using Pub = motion::motion_testing_nodes::PeriodicPublisher<State>;
  std::unique_ptr<Pub> m_ego_pub{nullptr};
  std::unique_ptr<Pub> m_target_pub{nullptr};
};  // class TestPublisher

#endif  // PUBLISHER_HPP_
