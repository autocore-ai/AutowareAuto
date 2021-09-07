// Copyright 2020 Embotech AG, Zurich, Switzerland. All rights reserved.
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
#include <gtest/gtest.h>
#include <common/types.hpp>
#include <algorithm>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include "parking_planner/geometry.hpp"
#include "parking_planner/parking_planner_types.hpp"
#include "parking_planner/bicycle_model.hpp"
#include "parking_planner/parking_planner.hpp"

using autoware::common::types::float64_t;
using Point2D = autoware::motion::planning::parking_planner::Point2D<float64_t>;
using Polytope2D = autoware::motion::planning::parking_planner::Polytope2D<float64_t>;
using VehicleState = autoware::motion::planning::parking_planner::VehicleState<float64_t>;
using VehicleCommand = autoware::motion::planning::parking_planner::VehicleCommand<float64_t>;
using BicycleModelParameters =
  autoware::motion::planning::parking_planner::BicycleModelParameters<float64_t>;
using BicycleModel =
  autoware::motion::planning::parking_planner::BicycleModel<float64_t, float64_t>;
using Trajectory = autoware::motion::planning::parking_planner::Trajectory<float64_t>;
using TrajectoryStep = autoware::motion::planning::parking_planner::TrajectoryStep<float64_t>;
using autoware::motion::planning::parking_planner::ParkingPlanner;
using autoware::motion::planning::parking_planner::PlanningStatus;
using autoware::motion::planning::parking_planner::NLPCostWeights;


// --- Plotting facilities, only built if plotting is enabled -----
#ifdef PARKING_PLANNER_TEST_PLOTS
#include <matplotlibcpp.h>

static float64_t get_point_x(Point2D point)
{return point.get_coord().first;}

static float64_t get_point_y(Point2D point)
{return point.get_coord().second;}


template<typename Iter, typename Y>
std::vector<float64_t> extract_scalar(Iter start, Iter end, float64_t (* f)(Y y), bool loopback)
{
  std::vector<float64_t> scalars{};
  std::transform(start, end, std::back_inserter(scalars), f);
  if (loopback) {
    scalars.push_back(scalars[0]);
  }
  return scalars;
}

void plot_polytope2D(const Polytope2D & polytope, const std::string & color)
{
  const auto & vertices = polytope.get_vertices();
  const auto extract_point_scalar = extract_scalar<decltype(vertices.begin()), Point2D>;

  const auto x_vertices = extract_point_scalar(vertices.begin(), vertices.end(), get_point_x, true);
  const auto y_vertices = extract_point_scalar(vertices.begin(), vertices.end(), get_point_y, true);

  std::map<std::string, std::string> plot_keywords{};
  plot_keywords["color"] = color;
  matplotlibcpp::plot(x_vertices, y_vertices, plot_keywords);
}


void plot_trajectory(const Trajectory & trajectory)
{
  std::vector<float64_t> x_states{};
  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(x_states),
    [](auto x) -> float64_t {return x.get_state().get_x();});

  std::vector<float64_t> y_states{};
  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(y_states),
    [](auto x) -> float64_t {return x.get_state().get_y();});

  std::map<std::string, std::string> plot_keywords{};
  plot_keywords["marker"] = "x";
  matplotlibcpp::plot({x_states[0]}, {y_states[0]}, plot_keywords);
  matplotlibcpp::plot(x_states, y_states);
}

void show_plot(const bool blocking_mode)
{
  matplotlibcpp::axis("equal");
  matplotlibcpp::tight_layout();
  matplotlibcpp::show(blocking_mode);
}

void ensure_blank_plot()
{
  matplotlibcpp::close();
}

void set_plot_size(std::size_t width_pixels, std::size_t height_pixels)
{
  matplotlibcpp::figure_size(width_pixels, height_pixels);
}

void save_plot(const std::string & path)
{
  matplotlibcpp::axis("equal");
  matplotlibcpp::tight_layout();
  matplotlibcpp::save(path);
}
#endif


struct IntegrationTestParams
{
  std::string scenario_name;
  std::vector<Polytope2D> obstacles;
  VehicleState current_state;
  VehicleState goal_state;
  BicycleModelParameters model_parameters;
  PlanningStatus expected_status;
};

class TestIntegration : public ::testing::TestWithParam<IntegrationTestParams> {};

// TODO(jwhitleywork): Re-enable this test when fixed on arm64. Relevant issue is #551.
// This test runs the parking planner like a user would
TEST_P(TestIntegration, DISABLED_with_obstacles) {
  const auto params = GetParam();
  RecordProperty("scenario_name", params.scenario_name);

  // Configure the vehicle capabilities
  const VehicleState lower_state_bounds(-100, -100, -12, -2 * 3.14156, -0.52);
  const VehicleState upper_state_bounds(+100, +100, +12, +2 * 3.14156, +0.52);
  const VehicleCommand lower_command_bounds(-10.0, -15);
  const VehicleCommand upper_command_bounds(+10.0, +15);
  const NLPCostWeights<float64_t> weights(1.0, 1.0, 0.0);

  auto planner = ParkingPlanner(
    params.model_parameters, weights, lower_state_bounds,
    upper_state_bounds, lower_command_bounds, upper_command_bounds);

  const auto result = planner.plan(params.current_state, params.goal_state, params.obstacles);
  const auto result_trajectory = result.get_trajectory();

  // Record NLP iterations as well as process time. The latter is a float64_t and
  // RecordProperty only does int or string, so we build a string.
  RecordProperty("iterations", result.get_nlp_iterations() );
  std::ostringstream process_time_str;
  process_time_str << result.get_nlp_proc_time();
  RecordProperty("process_time", process_time_str.str());

#ifdef PARKING_PLANNER_TEST_PLOTS
  // Do the plotting setup and draw the obstacles
  ensure_blank_plot();
  set_plot_size(1650, 1080);
  for (const auto & obstacle : params.obstacles) {
    plot_polytope2D(obstacle, "blue");
  }

  // Draw the result trajectory, with the model box for each discrete step
  const auto model = BicycleModel(params.model_parameters);
  for (const auto & trajectory_step : result_trajectory) {
    plot_polytope2D(model.compute_bounding_box(trajectory_step.get_state()), "gray");
  }
  plot_trajectory(result_trajectory);

  // Plot start and goal states as well
  plot_polytope2D(
    BicycleModel(params.model_parameters).compute_bounding_box(
      params.current_state), "black");
  plot_polytope2D(
    BicycleModel(params.model_parameters).compute_bounding_box(
      params.goal_state), "black");

  std::string file_name = params.scenario_name + std::string{".png"};
  std::replace(file_name.begin(), file_name.end(), '/', '_');
  save_plot(std::string{"/tmp/"} + file_name);
#endif

  EXPECT_EQ(result.get_status(), params.expected_status);
}

const auto side_gap_obstacles = std::vector<Polytope2D>{
  Polytope2D{std::vector<Point2D>({{-4.0, -0.8}, {-12.0, -0.8}, {-12.0, -2.5}, {-4.0, -2.5}})},
  Polytope2D{std::vector<Point2D>({{10.0, -0.8}, {0.0, -0.8}, {0.0, -2.5}, {10.0, -2.5}})},
  Polytope2D{std::vector<Point2D>({{10.0, -2.5}, {-12.0, -2.5}, {-12.0, -4.0}, {10.0, -4.0}})},
};

const auto orthogonal_gap_obstacles = std::vector<Polytope2D>{
  Polytope2D{std::vector<Point2D>({{-3.0, -0.8}, {-12.0, -0.8}, {-12.0, -4.0}, {-3.0, -4.0}})},
  Polytope2D{std::vector<Point2D>({{10.0, -0.8}, {-1.0, -0.8}, {-1.0, -4.0}, {10.0, -4.0}})},
  Polytope2D{std::vector<Point2D>({{10.0, -4}, {-12.0, -4}, {-12.0, -6}, {10.0, -6}})},
};

const auto slanted_gap_obstacles = std::vector<Polytope2D>{
  Polytope2D{std::vector<Point2D>({{-5.0, -0.8}, {-14.0, -0.8}, {-12.0, -4.0}, {-3.0, -4.0}})},
  Polytope2D{std::vector<Point2D>({{8.0, -0.8}, {-3.0, -0.8}, {-1.0, -4.0}, {10.0, -4.0}})},
  Polytope2D{std::vector<Point2D>({{10.0, -4}, {-12.0, -4}, {-12.0, -6}, {10.0, -6}})},
};

// This would be INSTANTIATE_TEST_SUITE_P in gtest 1.10 and up, but Autoware uses gtest 1.8.
INSTANTIATE_TEST_CASE_P(
  Integration, TestIntegration, ::testing::Values(
    // Parallel, close, and car is somewhat small
    IntegrationTestParams{"parallel_close_small",
      side_gap_obstacles,
      VehicleState(0.0, 0.0, 0.0, 0, 0.0),
      VehicleState(-2.0, -1.5, 0.0, 0, 0.0),
      BicycleModelParameters(1.0, 1.0, 1.3, 0.1, 0.1),
      PlanningStatus::OK
    },
    // Parallel, a bit further away, and car is a bit bigger
    IntegrationTestParams{"parallel_from_front",
      side_gap_obstacles,
      VehicleState(4.0, 0.0, 0.0, 0, 0.0),
      VehicleState(-2.0, -1.5, 0.0, 0, 0.0),
      BicycleModelParameters(1.1, 1.1, 1.2, 0.1, 0.1),
      PlanningStatus::OK
    },
    // Parallel: From behind the gap
    IntegrationTestParams{"parallel_from_behind",
      side_gap_obstacles,
      VehicleState(-4.0, 0.0, 0.0, 0, 0.0),
      VehicleState(-2.0, -1.5, 0.0, 0, 0.0),
      BicycleModelParameters(1.1, 1.1, 1.2, 0.1, 0.1),
      PlanningStatus::OK
    },
    // Orthogonal case, forward parking
    IntegrationTestParams{"orthorognal_forward_from_behind",
      orthogonal_gap_obstacles,
      VehicleState(-6.0, 0.0, 0.0, 0, 0.0),
      VehicleState(-2.0, -2.5, 0.0, -1.57, 0.0),
      BicycleModelParameters(1.1, 1.1, 1.0, 0.1, 0.1),
      PlanningStatus::OK
    },
    // Orthogonal case, forward parking, closer to cap
    IntegrationTestParams{"orthogonal_forward_from_next_to_gap",
      orthogonal_gap_obstacles,
      VehicleState(-2.0, 0.0, 0.0, 0, 0.0),
      VehicleState(-2.0, -2.5, 0.0, -1.57, 0.0),
      BicycleModelParameters(1.1, 1.1, 1.0, 0.1, 0.1),
      PlanningStatus::OK
    },
    // Orthogonal case, backwards parking
    IntegrationTestParams{"orthogonal_backwards_from_behind",
      orthogonal_gap_obstacles,
      VehicleState(-6.0, 0.0, 0.0, 0, 0.0),
      VehicleState(-2.0, -2.5, 0.0, 1.57, 0.0),
      BicycleModelParameters(1.1, 1.1, 1.0, 0.1, 0.1),
      PlanningStatus::OK
    },
    // Orthogonal case, backwards parking
    IntegrationTestParams{"orthogonal_backwards_from_next_to_gap",
      orthogonal_gap_obstacles,
      VehicleState(-2.0, 0.0, 0.0, 0, 0.0),
      VehicleState(-2.0, -2.5, 0.0, 1.57, 0.0),
      BicycleModelParameters(1.1, 1.1, 1.0, 0.1, 0.1),
      PlanningStatus::OK
    },
    // Slanted case, forward parking, after the gap
    IntegrationTestParams{"slanted_forward_from_front",
      slanted_gap_obstacles,
      VehicleState(-2.0, 0.0, 0.0, 0, 0.0),
      VehicleState(-3.0, -2.5, 0.0, -1.1, 0.0),
      BicycleModelParameters(1.1, 1.1, 1.0, 0.1, 0.1),
      PlanningStatus::OK
    }
    // TODO(feature,s.me) add infeasible cases to ensure those get handled properly

    // add extra comma to silence warning about missing argument to variadic macro
    // https://github.com/google/googletest/issues/1419#issuecomment-381423292
    // cppcheck-suppress syntaxError
  ),
);
