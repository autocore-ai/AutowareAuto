// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
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
#include <recordreplay_planner/recordreplay_planner.hpp>
#include <recordreplay_planner/vehicle_bounding_box.hpp>
#include <motion_testing/motion_testing.hpp>
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_msgs/msg/bounding_box_array.hpp>
#include <autoware_auto_msgs/msg/bounding_box.hpp>
#include <geometry/common_2d.hpp>
#include <motion_common/config.hpp>
#include <motion_common/motion_common.hpp>
#include <geometry_msgs/msg/point32.hpp>

#include <geometry/intersection.hpp>
#include <common/types.hpp>

#include <chrono>
#include <set>
#include <algorithm>
#include <string>
#include <cstdio>

using motion::planning::recordreplay_planner::RecordReplayPlanner;
using motion::planning::recordreplay_planner::compute_boundingbox_from_trajectorypoint;
using std::chrono::system_clock;
using motion::motion_testing::make_state;
using autoware_auto_msgs::msg::Trajectory;
using autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::BoundingBoxArray;
using autoware_auto_msgs::msg::BoundingBox;
using motion::motion_common::VehicleConfig;
using geometry_msgs::msg::Point32;
using motion::motion_common::from_angle;
using autoware::common::geometry::intersect;
using autoware::common::geometry::norm_2d;
using autoware::common::geometry::minus_2d;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

const VehicleConfig test_vehicle_params{1.0, 1.0, 0.5, 0.5, 1500, 12, 2.0, 0.5, 0.2};

class sanity_checks_base : public ::testing::Test
{
protected:
  RecordReplayPlanner planner_{test_vehicle_params};
};


//------------------ Test basic properties of a recorded, then replayed trajectory
struct PropertyTestParameters
{
  std::chrono::milliseconds time_spacing_ms;
  system_clock::time_point starting_time;
};

class sanity_checks_trajectory_properties
  : public sanity_checks_base, public testing::WithParamInterface<PropertyTestParameters>
{};

TEST_P(sanity_checks_trajectory_properties, basicproperties)
{
  const auto p = GetParam();
  auto t0 = p.starting_time;

  // Build a trajectory
  constexpr auto N = 10;
  const auto time_increment = p.time_spacing_ms;
  for (uint32_t k = {}; k < N; ++k) {
    const auto next_state = make_state(1.0F * k, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
        t0 + k * time_increment);
    planner_.record_state(next_state);
  }

  // Test: Check that the length is equal to the number of states we fed in
  EXPECT_EQ(planner_.get_record_length(), static_cast<std::size_t>(N));

  // Test: Check that the plan returned has the expected time length
  auto trajectory = planner_.plan(make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
  float64_t trajectory_time_length = trajectory.points[N - 1].time_from_start.sec + 1e-9F *
    trajectory.points[N - 1].time_from_start.nanosec;
  EXPECT_EQ(std::chrono::duration<float32_t>(trajectory_time_length),
    1.0F * (N - 1) * time_increment);
}

INSTANTIATE_TEST_CASE_P(
  trajectory_properties,
  sanity_checks_trajectory_properties,
  testing::Values(
    PropertyTestParameters{std::chrono::milliseconds(100), system_clock::from_time_t({})},
    PropertyTestParameters{std::chrono::milliseconds(200), system_clock::from_time_t({})},
    PropertyTestParameters{std::chrono::milliseconds(100), system_clock::from_time_t(10)},
    PropertyTestParameters{std::chrono::milliseconds(200), system_clock::from_time_t(10)}
  ), );


//------------------ Test that length cropping properly works
struct LengthTestParameters
{
  // The number of points to be recorded
  uint32_t number_of_points;
};


class sanity_checks_trajectory_length
  : public sanity_checks_base, public testing::WithParamInterface<LengthTestParameters>
{};

TEST_P(sanity_checks_trajectory_length, length)
{
  const auto p = GetParam();
  const auto N = p.number_of_points;
  const auto dummy_state = make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      system_clock::from_time_t({}));

  for (uint32_t k = {}; k < N; ++k) {
    planner_.record_state(dummy_state);
  }

  // Test: Check that the length is equal to the number of states we fed in
  EXPECT_EQ(planner_.get_record_length(), N);
  auto trajectory = planner_.plan(dummy_state);

  EXPECT_EQ(trajectory.points.size(),
    std::min(N, static_cast<uint32_t>(trajectory.points.max_size())));
}

INSTANTIATE_TEST_CASE_P(
  trajectory_length,
  sanity_checks_trajectory_length,
  testing::Values(
    LengthTestParameters{80},
    LengthTestParameters{200}
  ), );


// Test setup helper function. This creates a planner and records a trajectory
// that goes along the points (0,0), (1,0), .... (N-1,0) with the heading set to
// 0 throughout - for testing purposes
RecordReplayPlanner helper_create_and_record_example(uint32_t N)
{
  auto planner = RecordReplayPlanner(test_vehicle_params);
  auto t0 = system_clock::from_time_t({});

  // Record some states going from
  for (uint32_t k = {}; k < N; ++k) {
    planner.record_state(make_state(1.0F * k, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      t0 + k * std::chrono::milliseconds{100LL}));
  }

  return planner;
}


//------------------ Test that "receding horizon" planning properly works: happy case
TEST(recordreplay_sanity_checks, receding_horizon_happycase)
{
  const auto N = 3;
  auto planner = helper_create_and_record_example(N);

  // Call "plan" multiple times in sequence, expecting the states to come back out in order
  const auto t0 = system_clock::from_time_t({});
  for (uint32_t k = {}; k < N; ++k) {
    auto trajectory = planner.plan(make_state(1.0F * k, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    // normally don't check float equality but we _just_ pushed this float so it ought not
    // to have changed
    EXPECT_EQ(1.0F * k, trajectory.points[0].x);
    EXPECT_EQ(N - k, trajectory.points.size());
  }
}

//------------------ Test that "receding horizon" planning properly works:
TEST(recordreplay_sanity_checks, receding_horizon_cornercases)
{
  const auto N = 3;
  auto planner = helper_create_and_record_example(N);

  const auto t0 = system_clock::from_time_t({});

  // Check: State we have not recorded, but is closest to the (0,0) state
  {
    auto trajectory = planner.plan(make_state(-1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    EXPECT_EQ(0.0F, trajectory.points[0].x);
  }

  // Check: State we have not recorded, but is closest to the (0,0) state
  {
    auto trajectory = planner.plan(make_state(0.1F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    EXPECT_EQ(0.0F, trajectory.points[0].x);
    EXPECT_EQ(0.0F, trajectory.points[0].y);
  }

  // Check: State we have not recorded, but is closest to the (N,0) state
  {
    auto trajectory = planner.plan(make_state(1.0F * N + 5.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    EXPECT_EQ((N - 1) * 1.0F, trajectory.points[0].x);
    EXPECT_EQ(0.0F, trajectory.points[0].y);
  }
}

//------------------ Test that "receding horizon" planning properly works:
TEST(recordreplay_sanity_checks, obstacle_stopping)
{
  const auto N = 10;
  const auto t0 = system_clock::from_time_t({});
  auto planner = helper_create_and_record_example(N);


  // Check: Trajectory without box in place is N long
  {
    auto trajectory = planner.plan(make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
    EXPECT_EQ(trajectory.points.size(), static_cast<std::size_t>(N));
  }

  // Add a box that intersects with the trajectory
  const auto state = make_state(N, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
  const VehicleConfig test_params{1.0, 1.0, 0.5, 0.5, 1500, 12, 2.0, 0.5, 0.2};
  const auto aligned_box = compute_boundingbox_from_trajectorypoint(state.state, test_params);
  auto boxes_list = BoundingBoxArray{};
  boxes_list.boxes.push_back(aligned_box);
  planner.update_bounding_boxes(boxes_list);

  // Check: Trajectory with box added is shorter
  auto trajectory = planner.plan(make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));
  EXPECT_EQ(trajectory.points.size(), static_cast<std::size_t>(N - 2));
}

TEST(recordreplay_sanity_checks, state_setting_mechanism)
{
  auto planner = RecordReplayPlanner{test_vehicle_params};

  // Make sure setting and reading the recording state works
  EXPECT_FALSE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );

  planner.start_recording();

  EXPECT_TRUE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );

  planner.stop_recording();

  EXPECT_FALSE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );

  // Make sure setting and reading the replaying state works
  EXPECT_FALSE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );

  planner.start_replaying();

  EXPECT_FALSE(planner.is_recording() );
  EXPECT_TRUE(planner.is_replaying() );

  planner.stop_replaying();

  EXPECT_FALSE(planner.is_recording() );
  EXPECT_FALSE(planner.is_replaying() );
}

TEST(recordreplay_sanity_checks, heading_weight_setting)
{
  auto planner = RecordReplayPlanner{test_vehicle_params};

  planner.set_heading_weight(0.5);
  EXPECT_EQ(planner.get_heading_weight(), 0.5);
  EXPECT_THROW(planner.set_heading_weight(-1.0), std::domain_error);
}

TEST(recordreplay_sanity_checks, adding_bounding_boxes)
{
  auto planner = RecordReplayPlanner{test_vehicle_params};
  EXPECT_EQ(planner.get_number_of_bounding_boxes(), 0UL);

  auto dummy_boxes = BoundingBoxArray{};
  dummy_boxes.boxes.push_back(BoundingBox{});
  dummy_boxes.boxes.push_back(BoundingBox{});

  planner.update_bounding_boxes(dummy_boxes);

  EXPECT_EQ(planner.get_number_of_bounding_boxes(), 2UL);
}


const BoundingBox get_test_box(const TrajectoryPoint state)
{
  const VehicleConfig test_params{1.0, 1.0, 0.5, 0.5, 1500, 12, 2.0, 0.5, 0.2};
  return compute_boundingbox_from_trajectorypoint(state, test_params);
}


//------------------ Test that bounding box creation works
TEST(recordreplay_geometry_checks, bounding_box_creation)
{
  // Point construction helper
  const auto make_point = [](auto x, auto y) {
      Point32 point{};
      point.x = x;
      point.y = y;
      return point;
    };

  // Point comparison, for set membership checks. Defined as a struct because set wants one
  // https://stackoverflow.com/questions/2620862/using-custom-stdset-comparator
  struct point_compare
  {
    bool8_t operator()(const Point32 & p1, const Point32 & p2) const noexcept
    {
      const auto eps = 1e-6;  // close enough
      if (std::abs(p2.x - p1.x) > eps) {
        return p1.x > (p2.x + eps);
      } else {
        return p2.y > (p1.y + eps);
      }
    }
  };

  // Check a set of corners for being in a set of expected points
  const auto corners_check = [](auto expected_set, auto check_set) {
      for (std::size_t k = 0; k < check_set.size(); ++k) {
        EXPECT_EQ(expected_set.count(check_set[k]), static_cast<std::size_t>(1));
      }
    };


  const auto t0 = system_clock::from_time_t({});

  // First case: box aligned with axes
  {
    const auto state = make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
    auto aligned_box = get_test_box(state.state);

    EXPECT_EQ(aligned_box.corners.size(), static_cast<std::size_t>(4));
    const auto expected_corners = std::set<Point32, point_compare>({
      make_point(1.5f, 1.0f),
      make_point(-1.2f, 1.0f),
      make_point(-1.2f, -1.0f),
      make_point(1.5f, -1.0f),
    });

    corners_check(expected_corners, aligned_box.corners);
  }

  // Test case 2: box rotated by 90 degrees
  {
    const auto rotated_state = make_state(0.0F, 0.0F, 1.5707963267948966, 0.0F, 0.0F, 0.0F, t0);
    auto rotated_box = get_test_box(rotated_state.state);

    EXPECT_EQ(rotated_box.corners.size(), static_cast<std::size_t>(4));

    const auto expected_corners = std::set<Point32, point_compare>({
      make_point(+1.0f, +1.5f),
      make_point(-1.0f, +1.5f),
      make_point(-1.0f, -1.2f),
      make_point(+1.0f, -1.2f),
    });

    corners_check(expected_corners, rotated_box.corners);
  }
}

TEST(recordreplay_geometry_checks, collision_check) {
  // First case: box aligned with axes
  const auto t0 = system_clock::from_time_t({});
  const auto state = make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
  auto aligned_box = get_test_box(state.state);

  { // Intersection
    const auto state_shifted = make_state(1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
    auto aligned_box_shifted = get_test_box(state_shifted.state);
    auto collision = intersect(aligned_box.corners.begin(), aligned_box.corners.end(),
        aligned_box_shifted.corners.begin(), aligned_box_shifted.corners.end());
    EXPECT_TRUE(collision);
  }

  { // No intersection
    const auto state_shifted = make_state(50.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
    auto aligned_box_shifted = get_test_box(state_shifted.state);
    auto collision = intersect(aligned_box.corners.begin(), aligned_box.corners.end(),
        aligned_box_shifted.corners.begin(), aligned_box_shifted.corners.end());
    EXPECT_FALSE(collision);
  }

  { // Small intersection in a corner
    const auto state_shifted = make_state(2.6F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0);
    auto aligned_box_shifted = get_test_box(state_shifted.state);
    auto collision = intersect(aligned_box.corners.begin(), aligned_box.corners.end(),
        aligned_box_shifted.corners.begin(), aligned_box_shifted.corners.end());
    EXPECT_TRUE(collision);
  }
}

// Test write/read trajectory to/from file
TEST(RecordreplayWriteReadTrajectory, WriteReadTrajectory)
{
  std::string file_name("write_test.trajectory");

  const auto N = 5;
  auto planner = helper_create_and_record_example(N);

  // Write, clear buffer and read the written data again
  planner.writeTrajectoryBufferToFile(file_name);

  planner.clear_record();
  EXPECT_EQ(planner.get_record_length(), static_cast<std::size_t>(0));

  planner.readTrajectoryBufferFromFile(file_name);

  EXPECT_EQ(std::remove(file_name.c_str()), 0);
  EXPECT_EQ(planner.get_record_length(), static_cast<std::size_t>(5));

  const auto t0 = system_clock::from_time_t({});
  auto trajectory = planner.plan(make_state(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, t0));

  for (uint32_t k = {}; k < N; ++k) {
    EXPECT_EQ(1.0F * k, trajectory.points[k].x);
  }
}

TEST(RecordreplayWriteReadTrajectory, writeTrajectoryEmptyPath)
{
  const auto N = 5;
  auto planner = helper_create_and_record_example(N);

  ASSERT_THROW(planner.writeTrajectoryBufferToFile(""), std::runtime_error);
}

TEST(RecordreplayWriteReadTrajectory, readTrajectoryEmptyPath)
{
  const auto N = 5;
  auto planner = helper_create_and_record_example(N);

  ASSERT_THROW(planner.readTrajectoryBufferFromFile(""), std::runtime_error);
}
