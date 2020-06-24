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
#include <acado_common.h>

#include <ostream>

#include "mpc_planner/mpc_planner.hpp"

namespace motion
{
namespace planning
{
namespace mpc_planner
{
////////////////////////////////////////////////////////////////////////////////
void MpcPlanner::debug_print(std::ostream & out) const
{
  {
    out << "x0:\n";
    for (std::size_t jdx = {}; jdx < ACADO_NX; ++jdx) {
      out << acadoVariables.x0[jdx] << "\t";
    }
    out << "\n";
  }
  {
    out << "x:\n";
    for (std::size_t idx = {}; idx < ACADO_N; ++idx) {
      for (std::size_t jdx = {}; jdx < ACADO_NX; ++jdx) {
        out << acadoVariables.x[(idx * ACADO_NX) + jdx] << "\t";
      }
      out << "\n";
    }
  }
  {
    out << "u:\n";
    for (std::size_t idx = {}; idx < ACADO_N; ++idx) {
      for (std::size_t jdx = {}; jdx < ACADO_NU; ++jdx) {
        out << acadoVariables.u[(idx * ACADO_NU) + jdx] << "\t";
      }
      out << "\n";
    }
  }
  {
    out << "y:\n";
    for (std::size_t idx = {}; idx < ACADO_N; ++idx) {
      for (std::size_t jdx = {}; jdx < ACADO_NY; ++jdx) {
        out << acadoVariables.y[(idx * ACADO_NY) + jdx] << "\t";
      }
      out << "\n";
    }
  }
  {
    out << "yN:\n";
    for (std::size_t jdx = {}; jdx < ACADO_NYN; ++jdx) {
      out << acadoVariables.yN[jdx] << "\t";
    }
    out << "\n";
  }
  {
    out << "W:\n";
    for (std::size_t jdx = {}; jdx < ACADO_NY; ++jdx) {
      out << acadoVariables.W[(jdx * ACADO_NY) + jdx] <<
        "\t";
    }
  }
  {
    out << "WN:\n";
    for (std::size_t jdx = {}; jdx < ACADO_NYN; ++jdx) {
      out << acadoVariables.WN[(jdx * ACADO_NYN) + jdx] << "\t";
    }
    out << "\n";
  }
}
}  // namespace mpc_planner
}  // namespace planning
}  // namespace motion
