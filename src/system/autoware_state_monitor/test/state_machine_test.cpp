// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#include "autoware_state_monitor/state_machine.hpp"

#include <memory>

#include "gtest/gtest.h"

#include "test_utils.hpp"

using autoware::state_monitor::State;
using autoware::state_monitor::StateInput;
using autoware::state_monitor::StateMachine;
using autoware::state_monitor::StateMachineParams;

using autoware_auto_msgs::msg::AutowareState;
using autoware_auto_msgs::msg::Engage;
using autoware_auto_msgs::msg::HADMapRoute;
using autoware_auto_msgs::msg::RoutePoint;
using autoware_auto_msgs::msg::VehicleOdometry;
using autoware_auto_msgs::msg::VehicleStateReport;

using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;

class StateMachineTest : public ::testing::Test
{
public:
  StateMachineTest()
  {
    default_params.arrived_distance_threshold = 1.0;
    default_params.stopped_velocity_threshold_mps = 0.1;
    default_params.wait_time_after_initializing = 1.0;
    default_params.wait_time_after_planning = 3.0;
    default_params.wait_time_after_arrived_goal = 2.0;

    state_machine.reset(new StateMachine(default_params));
  }
  ~StateMachineTest() = default;

protected:
  StateInput initializeWithStateAndParams(
    const State & state, const StateMachineParams & params)
  {
    if (state_machine->getCurrentState() != AutowareState::INITIALIZING) {
      state_machine.reset(new StateMachine(params));
    }

    StateInput input;
    input.current_time = toTime(0.0);
    state_machine->updateState(input);
    if (state == AutowareState::INITIALIZING) {
      return input;
    }

    input.current_time = toTime(2.0);
    state_machine->updateState(input);
    if (state == AutowareState::WAITING_FOR_ROUTE) {
      return input;
    }

    input.route = std::make_shared<HADMapRoute>();
    state_machine->updateState(input);
    if (state == AutowareState::PLANNING) {
      return input;
    }

    state_machine->updateState(input);
    input.current_time = toTime(7.0);
    state_machine->updateState(input);
    if (state == AutowareState::WAITING_FOR_ENGAGE) {
      return input;
    }

    input.engage = prepareEngageMsg(true);
    input.vehicle_state_report = prepareVehicleStateReportMsg(true);
    state_machine->updateState(input);
    if (state == AutowareState::DRIVING) {
      return input;
    }

    input.current_pose = preparePoseStampedMsg(getPoint(0.0, 0.0));
    input.goal_pose = prepareRoutePointMsg(getPoint(0.0, 0.0));
    state_machine->updateState(input);
    if (state == AutowareState::ARRIVED_GOAL) {
      return input;
    }

    return input;
  }

  StateInput initializeWithState(const State & state)
  {
    return initializeWithStateAndParams(state, default_params);
  }

  std::shared_ptr<StateMachine> state_machine;
  StateMachineParams default_params;
};

TEST_F(StateMachineTest, create_destroy)
{
}

TEST_F(StateMachineTest, initialization_sequence)
{
  StateInput input;
  // time: 0s, start initialization
  input.current_time = toTime(0.0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::INITIALIZING);

  // time: 0s, after initialization SM waits 1s
  EXPECT_EQ(state_machine->updateState(input), AutowareState::INITIALIZING);

  // time: 2s, initialization state should be changed after 2s
  input.current_time = toTime(2.0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::WAITING_FOR_ROUTE);
}

TEST_F(StateMachineTest, default_full_sequence)
{
  // sequence: Initializing -> WaitingForRoute -> Planning
  //           -> WaitingForEngage -> Driving -> ArrivedGoal --> WaitingForRoute ...
  StateInput input;
  // time: 0s, start initialization
  input.current_time = toTime(0.0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::INITIALIZING);

  // time: 2s, initialization state should be changed after 2s
  input.current_time = toTime(2.0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::WAITING_FOR_ROUTE);

  // time: 2s, when new route is received then system starts planning
  input.route = std::make_shared<HADMapRoute>();
  EXPECT_EQ(state_machine->updateState(input), AutowareState::PLANNING);

  // time: 3s, if planning completed, the system waits 3s before state transition
  input.current_time = toTime(3.0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::PLANNING);

  // time: 7s
  input.current_time = toTime(7.0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::WAITING_FOR_ENGAGE);

  // time 7s, engage set to true and start driving
  const bool engage = true;
  const bool autonomous_mode = true;
  input.engage = prepareEngageMsg(engage);
  input.vehicle_state_report = prepareVehicleStateReportMsg(autonomous_mode);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::DRIVING);

  // time 7s, current pose equal to goal pose -> vehicle arrived goal
  input.current_pose = preparePoseStampedMsg(getPoint(1.0, 1.0));
  input.goal_pose = prepareRoutePointMsg(getPoint(1.0, 1.0));
  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(0.0, 0.0));
  EXPECT_EQ(state_machine->updateState(input), AutowareState::ARRIVED_GOAL);

  // time 10s, the system waits 2s after goal is reached, then back to WaitingForRoute
  input.current_time = toTime(10.0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::WAITING_FOR_ROUTE);
}

TEST_F(StateMachineTest, new_route_received_during_waiting_for_engage)
{
  auto input = initializeWithState(AutowareState::WAITING_FOR_ENGAGE);
  input.route = std::make_shared<HADMapRoute>();

  EXPECT_EQ(state_machine->updateState(input), AutowareState::PLANNING);
}

TEST_F(StateMachineTest, goal_at_current_pose_during_waiting_for_engage)
{
  auto input = initializeWithState(AutowareState::WAITING_FOR_ENGAGE);
  input.current_pose = preparePoseStampedMsg(getPoint(1.0, 1.0));
  input.goal_pose = prepareRoutePointMsg(getPoint(1.0, 1.0));
  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(0.0, 0.0));

  EXPECT_EQ(state_machine->updateState(input), AutowareState::ARRIVED_GOAL);
}

TEST_F(StateMachineTest, new_route_received_during_driving)
{
  auto input = initializeWithState(AutowareState::DRIVING);
  input.route = std::make_shared<HADMapRoute>();

  EXPECT_EQ(state_machine->updateState(input), AutowareState::PLANNING);
}

TEST_F(StateMachineTest, disengaged_during_driving)
{
  auto input = initializeWithState(AutowareState::DRIVING);
  const bool engage = false;
  input.engage = prepareEngageMsg(engage);

  EXPECT_EQ(state_machine->updateState(input), AutowareState::WAITING_FOR_ENGAGE);
}

TEST_F(StateMachineTest, goal_unreached_vehicle_not_stopped_pos_velocity)
{
  auto input = initializeWithState(AutowareState::DRIVING);
  const auto point = getPoint(1.0, 1.0);
  input.current_pose = preparePoseStampedMsg(point);
  input.goal_pose = prepareRoutePointMsg(point);
  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(1.0, 0.0));

  EXPECT_EQ(state_machine->updateState(input), AutowareState::DRIVING);
}

TEST_F(StateMachineTest, goal_unreached_vehicle_not_stopped_neg_velocity)
{
  auto input = initializeWithState(AutowareState::DRIVING);
  const auto point = getPoint(1.0, 1.0);
  input.current_pose = preparePoseStampedMsg(point);
  input.goal_pose = prepareRoutePointMsg(point);
  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(-1.0, 0.0));

  EXPECT_EQ(state_machine->updateState(input), AutowareState::DRIVING);
}

TEST_F(StateMachineTest, goal_unreached_vehicle_not_stopped_multiple_odom_in_buffer)
{
  auto input = initializeWithState(AutowareState::DRIVING);
  const auto point = getPoint(1.0, 1.0);
  input.current_pose = preparePoseStampedMsg(point);
  input.goal_pose = prepareRoutePointMsg(point);

  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(0.0, 0.0));
  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(1.0, 0.0));
  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(-1.0, 0.0));

  EXPECT_EQ(state_machine->updateState(input), AutowareState::DRIVING);
}

TEST_F(StateMachineTest, goal_reached_vehicle_stopped_velocity_zero)
{
  auto input = initializeWithState(AutowareState::DRIVING);
  const auto point = getPoint(1.0, 1.0);
  input.current_pose = preparePoseStampedMsg(point);
  input.goal_pose = prepareRoutePointMsg(point);
  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(0.0, 0.0));

  EXPECT_EQ(state_machine->updateState(input), AutowareState::ARRIVED_GOAL);
}

TEST_F(StateMachineTest, goal_reached_velocity_non_zero_but_below_threshold)
{
  StateMachineParams params;
  params.arrived_distance_threshold = 1.0;
  params.stopped_velocity_threshold_mps = 0.5;

  auto input = initializeWithStateAndParams(AutowareState::DRIVING, params);
  const auto point = getPoint(1.0, 1.0);
  input.current_pose = preparePoseStampedMsg(point);
  input.goal_pose = prepareRoutePointMsg(point);
  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(0.05));

  EXPECT_EQ(state_machine->updateState(input), AutowareState::ARRIVED_GOAL);
}

TEST_F(StateMachineTest, goal_reached_zero_distance)
{
  const auto current_point = getPoint(-1.0, 1.0);
  const auto goal_point = getPoint(-1.0, 1.0);

  auto input = initializeWithState(AutowareState::DRIVING);
  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(0.0));
  input.current_pose = preparePoseStampedMsg(current_point);
  input.goal_pose = prepareRoutePointMsg(goal_point);

  const auto dist = distance(current_point, goal_point);
  EXPECT_FLOAT_EQ(dist, 0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::ARRIVED_GOAL);
}

TEST_F(StateMachineTest, goal_reached_non_zero_distance_below_threshold)
{
  StateMachineParams params;
  params.arrived_distance_threshold = 1.0;
  params.stopped_velocity_threshold_mps = 0.5;

  const auto current_point = getPoint(-0.1, 1.0);
  const auto goal_point = getPoint(0.5, 0.5);

  auto input = initializeWithStateAndParams(AutowareState::DRIVING, params);
  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(0.0));
  input.current_pose = preparePoseStampedMsg(current_point);
  input.goal_pose = prepareRoutePointMsg(goal_point);

  const auto dist = distance(current_point, goal_point);
  EXPECT_GT(dist, 0);
  EXPECT_LE(dist, params.arrived_distance_threshold);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::ARRIVED_GOAL);
}

TEST_F(StateMachineTest, goal_unreached_distance_above_threshold)
{
  StateMachineParams params;
  params.arrived_distance_threshold = 1.0;
  params.stopped_velocity_threshold_mps = 0.5;

  const auto current_point = getPoint(1.0, -1.0);
  const auto goal_point = getPoint(0.5, 0.5);

  auto input = initializeWithStateAndParams(AutowareState::DRIVING, params);
  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(0.0));
  input.current_pose = preparePoseStampedMsg(current_point);
  input.goal_pose = prepareRoutePointMsg(goal_point);

  const auto dist = distance(current_point, goal_point);
  EXPECT_GE(dist, params.arrived_distance_threshold);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::DRIVING);
}

TEST_F(StateMachineTest, goal_unreached_distance_equal_to_threshold)
{
  StateMachineParams params;
  params.arrived_distance_threshold = 1.0;
  params.stopped_velocity_threshold_mps = 0.5;

  const auto current_point = getPoint(1.0, 0.0);
  const auto goal_point = getPoint(0.0, 0.0);

  auto input = initializeWithStateAndParams(AutowareState::DRIVING, params);
  input.odometry_buffer.push_back(prepareVehicleOdometryMsg(0.0));
  input.current_pose = preparePoseStampedMsg(current_point);
  input.goal_pose = prepareRoutePointMsg(goal_point);

  const auto dist = distance(current_point, goal_point);
  EXPECT_FLOAT_EQ(dist, params.arrived_distance_threshold);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::DRIVING);
}

TEST_F(StateMachineTest, waiting_for_engage_missing_engage_in_input)
{
  auto input = initializeWithState(AutowareState::WAITING_FOR_ENGAGE);
  input.engage = nullptr;
  const bool autonomous_mode = true;
  input.vehicle_state_report = prepareVehicleStateReportMsg(autonomous_mode);

  EXPECT_EQ(state_machine->updateState(input), AutowareState::WAITING_FOR_ENGAGE);
}

TEST_F(StateMachineTest, waiting_for_engage_missing_vehicle_mode_in_input)
{
  auto input = initializeWithState(AutowareState::WAITING_FOR_ENGAGE);
  const bool engage = true;
  input.engage = prepareEngageMsg(engage);
  input.vehicle_state_report = nullptr;

  EXPECT_EQ(state_machine->updateState(input), AutowareState::WAITING_FOR_ENGAGE);
}

// TODO(mdrwiega): Enable test after enabling mode checking in state machine
TEST_F(StateMachineTest, DISABLED_waiting_for_engage_not_in_autonomous_mode)
{
  auto input = initializeWithState(AutowareState::WAITING_FOR_ENGAGE);
  const bool engage = true;
  input.engage = prepareEngageMsg(engage);
  const bool autonomous_mode = false;
  input.vehicle_state_report = prepareVehicleStateReportMsg(autonomous_mode);

  EXPECT_EQ(state_machine->updateState(input), AutowareState::WAITING_FOR_ENGAGE);
}

TEST_F(StateMachineTest, waiting_after_INITIALIZING)
{
  auto input = initializeWithState(AutowareState::INITIALIZING);
  const float start_time = input.current_time.seconds();
  input.current_time = toTime(start_time + 0.5);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::INITIALIZING);
  input.current_time = toTime(start_time + 1.0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::WAITING_FOR_ROUTE);
}

TEST_F(StateMachineTest, waiting_after_planning)
{
  auto input = initializeWithState(AutowareState::PLANNING);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::PLANNING);
  const float start_time = input.current_time.seconds();
  input.current_time = toTime(start_time + 2.0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::PLANNING);
  input.current_time = toTime(start_time + 3.0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::WAITING_FOR_ENGAGE);
}

TEST_F(StateMachineTest, waiting_after_arrived_goal)
{
  auto input = initializeWithState(AutowareState::ARRIVED_GOAL);
  const float start_time = input.current_time.seconds();
  input.current_time = toTime(start_time + 1.0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::ARRIVED_GOAL);
  input.current_time = toTime(start_time + 2.0);
  EXPECT_EQ(state_machine->updateState(input), AutowareState::WAITING_FOR_ROUTE);
}

TEST_F(StateMachineTest, finalizing_during_driving)
{
  auto input = initializeWithState(AutowareState::DRIVING);
  input.is_finalizing = true;
  EXPECT_EQ(state_machine->updateState(input), AutowareState::FINALIZING);
}
