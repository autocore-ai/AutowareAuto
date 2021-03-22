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

// This file is to be built and run standalone to produce code for an NLP solver.
// The NLP solver is then to be used at runtime by the rest of the code.
// This is done by the CMake build process.
//
// The code here is inherently linked with what is used in the main code by:
// * positioning of states in serialization
// * initial guess structure
// * result extraction
// These links are automatically kept consistent by the serialization interfaces
// defined for the states involved, along with the templates defined in nlp_adapters.hpp.
// One exception to this is are the constraint bounds - the constraints are, as a
// convention, put into a list of "equality constraints first, then inequality
// constraints". The other convention is that the equality constraints are all of the form
//     f(variables) = 0,
// and the inequality constraints are of the form
//     g(variables) <= 0.
// This does not lead to a loss of generality, and a fancier implementation with
// parametric was not necessary at this point.
//
// In order to follow what's going on in the code here, it is advised to look at the
// main() function first, then at sub-functions when they are used.

#include <casadi/casadi.hpp>
#include <common/types.hpp>

#include <vector>
#include <functional>
#include <string>
#include <sstream>
#include <iostream>
#include <limits>

#include "parking_planner/configuration.hpp"
#include "parking_planner/parking_planner_types.hpp"
#include "parking_planner/bicycle_model.hpp"
#include "parking_planner/nlp_adapters.hpp"
#include "parking_planner/rungekutta.hpp"

// Make data structures from the parking_planner namespace available - the code here is
// for a standalone binary, hence it is not in a namespace.

using autoware::common::types::float64_t;
using autoware::motion::planning::parking_planner::RK4;
using autoware::motion::planning::parking_planner::VehicleState;
using autoware::motion::planning::parking_planner::VehicleCommand;
using autoware::motion::planning::parking_planner::NLPObstacle;
using autoware::motion::planning::parking_planner::NLPCostWeights;
using autoware::motion::planning::parking_planner::NLPObstacleStageVariables;
using autoware::motion::planning::parking_planner::Halfplane2D;
using autoware::motion::planning::parking_planner::Polytope2D;
using autoware::motion::planning::parking_planner::Point2D;
using autoware::motion::planning::parking_planner::BicycleModel;
using autoware::motion::planning::parking_planner::BicycleModelParameters;
using autoware::motion::planning::parking_planner::Trajectory;
using autoware::motion::planning::parking_planner::TrajectoryStep;
using autoware::motion::planning::parking_planner::assemble_variable_vector_and_bounds;
using autoware::motion::planning::parking_planner::assemble_parameter_vector;
using autoware::motion::planning::parking_planner::HORIZON_LENGTH;
using autoware::motion::planning::parking_planner::MAX_NUMBER_OF_OBSTACLES;
using autoware::motion::planning::parking_planner::MAX_HYPERPLANES_PER_OBSTACLE;
using autoware::motion::planning::parking_planner::MAX_EGO_HYPERPLANES;
using autoware::motion::planning::parking_planner::INTEGRATION_STEP_SIZE;
using autoware::motion::planning::parking_planner::NUMBER_OF_INTEGRATION_STEPS;

// Define some shorthand notation for the serialization lengths
constexpr auto Nstates = VehicleState<float64_t>::get_serialized_length();
constexpr auto Ncommands = VehicleCommand<float64_t>::get_serialized_length();
constexpr auto Nvehicleparams = BicycleModelParameters<float64_t>::get_serialized_length();

using casadi::MX;
using casadi::DM;
using casadi::Function;
using casadi::nlpsol;
using casadi::Dict;
using casadi::MXDict;

// ---- Variable creation ---------------------------------------------------------------
// Create a a single-stage variable as an MX-based template instantiation of the
// template parameter T. T has to support deserialize of std::vector<MX>.
template<class T>
T make_single_stage_variable(const std::string & name)
{
  std::vector<MX> mx_variable = MX::sym(name, 1, 1, T::get_serialized_length());
  return T::deserialize(mx_variable);
}

// Create a named variable of a certain size. This is as a vector of MX instead of
// just an MX with many dimensions because that allows us to mix it better with
// templates and use std::vector<float64_t> as another instantiation.
std::vector<MX> make_named_variable(const std::string & name, const std::size_t dimension)
{
  if (dimension > static_cast<std::size_t>(std::numeric_limits<int64_t>::max())) {
    throw std::domain_error{"Variable dimension too large"};
  }

  return MX::sym(name, 1, 1, static_cast<int64_t>(dimension));
}

// Convenience function to create a name string with an index added to the end.
std::string indexed_name(const std::string & name, const std::size_t index)
{
  std::ostringstream assembled_name;
  assembled_name << name << "_" << index;
  return assembled_name.str();
}

// Take a class T that serializes to std::vector<MX>, and turn it into
// a single stacked MX variable for use in CasADi expressions
template<template<class> class T, typename Y>
Y convert_to_mx(T<Y> value)
{
  return vertcat(value.serialize());
}

// Turn a "vector of vectors" of MX variables into a 2D matrix for CasADi purposes.
// This is done so matrix products can be used in constraints.
MX matrix_from_vec_of_vec(const std::vector<std::vector<MX>> & vecvec)
{
  if (vecvec.size() == 0) {
    throw std::length_error{"Vector has to be longer than 0"};
  }
  MX matrix = horzcat(vecvec[0]);
  for (std::size_t k = 1; k < vecvec.size(); k++) {
    matrix = vertcat(matrix, horzcat(vecvec[k]));
  }
  return matrix;
}

Trajectory<MX> make_trajectory_variables(std::size_t number_of_stages)
{
  Trajectory<MX> stage_variables;
  stage_variables.reserve(number_of_stages);
  for (std::size_t k = {}; k < number_of_stages; ++k) {
    const auto states = make_single_stage_variable<VehicleState<MX>>(indexed_name("state", k));
    const auto commands =
      make_single_stage_variable<VehicleCommand<MX>>(indexed_name("command", k));
    stage_variables.emplace_back(TrajectoryStep<MX>(commands, states));
  }
  return stage_variables;
}

std::vector<Halfplane2D<MX>> make_halfplane_variables(
  const std::string & basename,
  const std::size_t max_obstacle_halfplanes)
{
  std::vector<Halfplane2D<MX>> halfplanes{};
  for (std::size_t k = 0; k < max_obstacle_halfplanes; k++) {
    auto coefficients = make_named_variable(indexed_name(basename + "_halfplane_lhs", k), 2);
    auto right_hand_side = MX::sym(indexed_name(basename + "_halfplane_rhs", k), 1);
    halfplanes.emplace_back(
      Halfplane2D<MX>(
        Point2D<MX>(coefficients[0], coefficients[1]),
        right_hand_side));
  }
  return halfplanes;
}

std::vector<NLPObstacleStageVariables<MX>> make_obstacle_stage_variables(
  const std::string & basename,
  const std::size_t max_obstacle_halfplanes,
  const std::size_t max_ego_halfplanes)
{
  std::vector<NLPObstacleStageVariables<MX>> variables{};
  variables.reserve(HORIZON_LENGTH);
  for (std::size_t k = 0; k < HORIZON_LENGTH; k++) {
    auto lambda =
      make_named_variable(indexed_name(basename + "_lambda", k), max_obstacle_halfplanes);

    auto mu =
      make_named_variable(indexed_name(basename + "_mu", k), max_ego_halfplanes);

    variables.push_back(NLPObstacleStageVariables<MX>(lambda, mu));
  }

  return variables;
}


NLPObstacle<MX> create_abstract_obstacle(
  const std::string & basename,
  const std::size_t max_obstacle_halfplanes,
  const std::size_t max_ego_halfplanes)
{
  // Create a polytope with the appropriate number of halfplanes
  auto halfplanes = make_halfplane_variables(basename, max_obstacle_halfplanes);

  auto stage_variables = make_obstacle_stage_variables(
    basename, max_obstacle_halfplanes,
    max_ego_halfplanes);

  return NLPObstacle<MX>(stage_variables, halfplanes);
}


// ---- Constraint creation -------------------------------------------------------------
// Wrapper around our dynamics function with MX variables
MX casadi_dynamics_wrapper(MX state, MX command, MX params)
{
  constexpr auto nparams = BicycleModelParameters<MX>::get_serialized_length();
  std::vector<MX> param_vec{};
  param_vec.reserve(nparams);
  for (std::size_t k = 0; k < nparams; ++k) {
    param_vec.push_back(params(k));
  }
  const auto parameters = BicycleModelParameters<MX>::deserialize(param_vec);
  const auto model = BicycleModel<MX, MX>(parameters);
  // Turn the MX variables into vectors
  std::vector<MX> state_vec{};
  state_vec.reserve(Nstates);
  for (std::size_t k = 0; k < Nstates; ++k) {
    state_vec.push_back(state(k));
  }
  std::vector<MX> command_vec{};
  command_vec.reserve(Nstates);
  for (std::size_t k = 0; k < Ncommands; ++k) {
    command_vec.push_back(command(k));
  }

  // Call the serialized dynamics function
  std::vector<MX> result = model.dynamics_serialized(state_vec, command_vec);

  // Turn the result back into an MX variable
  return vertcat(result);
}

void create_dynamics_constraints(
  const Trajectory<MX> & trajectory,
  const BicycleModelParameters<MX> & vehicle_parameters,
  const VehicleState<MX> & current_state,
  std::vector<MX> & equality_constraints
)
{
  // Create local helper variables for use in the callbacks
  MX x = MX::sym("x", Nstates);  // A set of states
  MX u = MX::sym("u", Ncommands);  // A set of commands
  MX p = MX::sym("p", Nvehicleparams);  // A set of parameters

  // Create an std::function to pass to casadi::Function, since the automatic cast
  // does not seem to work.
  const std::function<MX(MX, MX, MX)> casadi_dynamics_function =
    [](auto X, auto U, auto P) {return casadi_dynamics_wrapper(X, U, P);};

  // Create a casadi function object that integrates the dynamics for one horizon step
  Function dynamics = Function(
    "F",
    {x, u, p},    // Argument list
    {RK4(
        x, u, p, casadi_dynamics_function,
        INTEGRATION_STEP_SIZE, NUMBER_OF_INTEGRATION_STEPS)},  // Function to evaluate
    {"x", "u", "p"},    // Input names
    {"xf"},    // Output name
    {{}});    // Options dictionary

  // Create the actual constraints
  // - Initial state constraints
  const auto initial_constraints = convert_to_mx(trajectory[0].get_state()) -
    convert_to_mx(current_state);
  for (std::size_t k = 0; k < trajectory[0].get_state().get_serialized_length(); k++) {
    equality_constraints.push_back(initial_constraints(k) );
  }

  // - Dynamics consistency constraints along the rest of the trajectory
  for (std::size_t k = 0; k < HORIZON_LENGTH - 1; ++k) {
    auto evaluated_dynamics = dynamics(
      MXDict{
      {"x", convert_to_mx(trajectory[k].get_state())},
      {"u", convert_to_mx(trajectory[k].get_command())},
      {"p", convert_to_mx(vehicle_parameters)}
    });
    const auto stage_constraints =
      convert_to_mx(trajectory[k + 1].get_state()) - evaluated_dynamics["xf"];
    for (std::size_t k = 0; k < trajectory[0].get_state().get_serialized_length(); k++) {
      equality_constraints.push_back(stage_constraints(k) );
    }
  }

  // Don't return anything, we've modified equality_constraints in place
}

void create_terminal_state_constraint(
  const Trajectory<MX> & trajectory,
  const VehicleState<MX> & goal_state,
  std::vector<MX> & equality_constraints
)
{
  const auto constraint_expression =
    convert_to_mx(trajectory.back().get_state()) - convert_to_mx(goal_state);
  for (std::size_t k = 0; k < trajectory[0].get_state().get_serialized_length(); k++) {
    equality_constraints.push_back(constraint_expression(k) );
  }
}

// For the math of this, see the Zhang/Liniger/Borelli paper, in particular the section
// on how to formulate obstacle constraints for the general case. The same letters
// are used here as in the paper for easier reading.
void create_single_stage_obstacle_constraints(
  const VehicleState<MX> & states,
  const NLPObstacleStageVariables<MX> & obstacle_variables,
  const DM & G,  // Vehicle polyhedron left-hand-side matrix
  const MX & g,  // Vehicle polyhedron right-hand-side vector
  const MX & A,  // Obstacle polyhedron left-hand-side matrix, built from variables
  const MX & b,  // Obstacle polyhedron right-hand-side vector, built from variables
  std::vector<MX> & equality_constraints,
  std::vector<MX> & inequality_constraints
)
{
  // Assemble R and t, they depend on the states
  const auto heading = states.get_heading();
  const MX row1 = horzcat(std::vector<MX>({cos(heading), -1.0 * sin(heading)}));
  const MX row2 = horzcat(std::vector<MX>({sin(heading), cos(heading)}));
  auto R = vertcat(std::vector<MX>({row1, row2}));
  auto t = vertcat(std::vector<MX>({states.get_x(), states.get_y()}) );

  // Introduce shorthands for mu and lambda
  auto mu = vertcat(obstacle_variables.get_mu());
  auto lambda = vertcat(obstacle_variables.get_lambda());

  // - Distance constraint: -g^T\mu + (At(x) -b)^T\lambda_k^m >= 0
  constexpr auto minimum_distance = 0.1;  // meters
  inequality_constraints.push_back(
    -1.0 *
    (-dot(g, mu) + dot((mtimes(A, t) - b), lambda) - minimum_distance));

  // - Equality constraints: G^T\mu + R(x)^TA^T\lambda_k^m == 0
  const auto duality_constraint = mtimes(G.T(), mu) + mtimes(mtimes(R.T(), A.T()), lambda);
  for (std::size_t k = 0; k < 2; k++) {
    equality_constraints.push_back(duality_constraint(k));
  }

  // - 2-norm constraints on : \|A^T\lambda\|_2 <= 1
  inequality_constraints.push_back(dot(mtimes(A.T(), lambda), mtimes(A.T(), lambda)) - 1.0);
}

void create_obstacle_constraints(
  const Trajectory<MX> & trajectory,
  const BicycleModelParameters<MX> & vehicle_parameters,
  const std::vector<NLPObstacle<MX>> & obstacles,
  std::vector<MX> & equality_constraints,
  std::vector<MX> & inequality_constraints
)
{
  // Preparations: Build a vehicle polyhedron centered at (0,0) and heading 0
  // - Build G and g representing obstacle box at (x,y,heading) = (0,0,0)
  auto G = DM(std::vector<std::vector<float64_t>>({{1., 0.}, {-1., 0.}, {0, 1}, {0, -1}}));
  auto g = vertcat(
    std::vector<MX>(
  {
    vehicle_parameters.get_length_front() + vehicle_parameters.get_front_overhang(),
    vehicle_parameters.get_length_rear() + vehicle_parameters.get_rear_overhang(),
    0.5 * vehicle_parameters.get_vehicle_width(),
    0.5 * vehicle_parameters.get_vehicle_width()
  }));

  // - Add constraints for each combination of obstacle and problem stage
  for (const auto & obstacle : obstacles) {
    const auto A = obstacle.build_A();
    const auto b = obstacle.build_b();
    for (std::size_t k = {}; k < trajectory.size(); ++k) {
      create_single_stage_obstacle_constraints(
        trajectory[k].get_state(), obstacle.get_stage_variables()[k],
        G, g, matrix_from_vec_of_vec(A), vertcat(b), equality_constraints, inequality_constraints);
    }
  }
}

// ---- Cost function creation ----------------------------------------------------------
MX create_cost_function(
  const Trajectory<MX> & trajectory,
  const VehicleState<MX> & goal_state,
  const NLPCostWeights<MX> & cost_weights
)
{
  // Terminal cost: Try to reach the goal state if at all possible
  auto cost_function = cost_weights.get_goal_weight() *
    dot(
    convert_to_mx(goal_state) - convert_to_mx(trajectory[HORIZON_LENGTH - 1].get_state()),
    convert_to_mx(goal_state) - convert_to_mx(trajectory[HORIZON_LENGTH - 1].get_state()));

  // Input cost: Try to not use tons of actuation
  const auto throttle_cost = cost_weights.get_throttle_weight();
  const auto steering_cost = cost_weights.get_steering_weight();
  for (std::size_t k = {}; k < HORIZON_LENGTH - 1; k++) {
    const auto commands_k = trajectory[k].get_command();
    cost_function += throttle_cost * commands_k.get_throttle() * commands_k.get_throttle();
    cost_function += steering_cost * commands_k.get_steering_rate() *
      commands_k.get_steering_rate();
  }

  return cost_function;
}

// This generates a header that will be used by the code calling into the solver.
// This is only required because the number of equality and inequality constraints
// is not a straightforward definition, but results from the code. We hence create
// a header with the definitions once the number of constraints can be determined.
// This allows anyone to add more constraints to the lists without touching any
// other code.
void generate_info_header(
  const std::string & shared_library_directory,
  const std::string & output_filename,
  const std::size_t number_of_equality_constraints,
  const std::size_t number_of_inequality_constraints
)
{
  std::ofstream info_header;
  info_header.open(output_filename, std::ios::out | std::ios::trunc);
  info_header <<
    "// Copyright 2020 Embotech AG" << std::endl <<
    "// This header is auto-generated by the solver generator" << std::endl <<
    "#ifndef NLP_CPP_INFO_HPP_" << std::endl <<
    "#define NLP_CPP_INFO_HPP_ 1" << std::endl <<
    "constexpr std::size_t NUMBER_OF_EQUALITIES = " <<
    number_of_equality_constraints << ";" << std::endl <<
    "constexpr std::size_t NUMBER_OF_INEQUALITIES = " <<
    number_of_inequality_constraints << ";" << std::endl <<
    "const std::string SHARED_LIBRARY_DIRECTORY = \"" <<
    shared_library_directory << "\";" << std::endl <<
    "#endif  // NLP_CPP_INFO_HPP_" << std::endl;

  info_header.close();
}


// ---- Main solver construction --------------------------------------------------------
int main(int argc, char * argv[])
{
  if (argc != 2) {
    std::cerr << "Path argument missing. The path argument specifies where the shared " <<
      "library will be put by the build system. This is then told to the actual " <<
      "NLP code via a generated header." << std::endl;
    return 1;
  }

  const auto shared_library_directory = std::string(argv[1]);

  // Create variables
  auto trajectory = make_trajectory_variables(HORIZON_LENGTH);
  std::vector<NLPObstacle<MX>> obstacles{};
  obstacles.reserve(MAX_NUMBER_OF_OBSTACLES);
  for (std::size_t k = {}; k < MAX_NUMBER_OF_OBSTACLES; k++) {
    obstacles.push_back(
      create_abstract_obstacle(
        "obstacle_0", MAX_HYPERPLANES_PER_OBSTACLE,
        MAX_EGO_HYPERPLANES));
  }

  // Create run-time parameters, those will be adjustable at every solve call
  auto current_state = make_single_stage_variable<VehicleState<MX>>("current_state");
  auto goal_state = make_single_stage_variable<VehicleState<MX>>("goal_state");
  auto vehicle_params =
    make_single_stage_variable<BicycleModelParameters<MX>>("vehicle_parameters");
  auto cost_weights = make_single_stage_variable<NLPCostWeights<MX>>("cost_weights");

  // Create empty constraints collections
  std::vector<MX> equality_constraints{};  // for type "==" constraints
  std::vector<MX> inequality_constraints{};  // for type "<=" constraints

  // Create the individual
  create_dynamics_constraints(
    trajectory, vehicle_params, current_state,
    equality_constraints);

  create_terminal_state_constraint(trajectory, goal_state, equality_constraints);

  create_obstacle_constraints(
    trajectory, vehicle_params, obstacles,
    equality_constraints, inequality_constraints);

  // Create cost function
  const auto cost_function = create_cost_function(trajectory, goal_state, cost_weights);

  // Serialize data for the academic-style solver object
  // - Variables, using the adapter from nlp_adapters.hpp
  const auto variables_and_bounds = assemble_variable_vector_and_bounds<MX>(
    trajectory, obstacles, VehicleState<float64_t>{}, VehicleState<float64_t>{},
    VehicleCommand<float64_t>{}, VehicleCommand<float64_t>{});
  const MX variable_vector = vertcat(variables_and_bounds.variables);

  // - Parameters, using the adapter from nlp_adapters.hpp
  const MX parameter_vector =
    vertcat(
    assemble_parameter_vector<MX>(
      current_state, goal_state,
      vehicle_params, obstacles, cost_weights));

  // - Constraints - equality first, then inequality.
  const MX constraint_vector =
    vertcat(vertcat(equality_constraints), vertcat(inequality_constraints));

  // Put it all together into an object describing the problem we want to solve
  MXDict nlp = {
    {"x", variable_vector},  // Variables to be optimized over
    {"p", parameter_vector},  // Solver parameters
    {"f", cost_function},  // Cost function
    {"g", constraint_vector}};  // Constraint function

  // Define some options for the code generation. The hessian approximation setting has
  // to be kept the same as in generate_nlp_planner_solver, because different settings
  // lead to different callbacks being created.
  Dict ipopt_options = {{"hessian_approximation", "limited-memory"}};
  Function solver = nlpsol("solver", "ipopt", nlp, {{"ipopt", ipopt_options}});

  // Generate code and a header with the relevant info
  // Calling this with something other than just a filename ending in C errors
  // with a cryptic assertion, and this just generates everything into the current
  // working directory, hence the executable has to be run with the working directory
  // set to where we want this file to go.
  std::cout << "Creating solver code" << std::endl;
  solver.generate_dependencies("parking_planner_callbacks.c");

  // This keeps the same approach as above for consistency.
  generate_info_header(
    shared_library_directory, "nlp_cpp_info.hpp",
    equality_constraints.size(), inequality_constraints.size() );

  // Everything went well
  return 0;
}
