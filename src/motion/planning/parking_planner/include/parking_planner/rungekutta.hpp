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
#ifndef PARKING_PLANNER__RUNGEKUTTA_HPP_
#define PARKING_PLANNER__RUNGEKUTTA_HPP_

#include <common/types.hpp>
#include <functional>
#include <vector>

namespace autoware
{

namespace motion
{

namespace planning
{

namespace parking_planner
{

using autoware::common::types::float64_t;

// Implement some operations on vectors of scalars
template<typename T>
std::vector<T> operator+(const std::vector<T> & vector1, const std::vector<T> & vector2)
{
  std::vector<T> result = {};
  result.reserve(vector1.size());
  for (std::size_t k = 0; k < vector2.size(); k++) {
    result.push_back(vector1[k] + vector2[k]);
  }
  return result;
}

template<typename T>
std::vector<T> operator*(const T & scalar, const std::vector<T> & vector)
{
  std::vector<T> result = {};
  result.reserve(vector.size());
  for (std::size_t k = 0; k < vector.size(); k++) {
    result.push_back(scalar * vector[k]);
  }
  return result;
}


/// \brief Template for an explicit Runge-Kutta integrator. CasADi has its own, but it
///        for some reason doesn't compose with the IPOPT solver generator :shrug:
template<class X, class U, typename P>
X RK4(X x, U u, P p, std::function<X(X, U, P)> f, const float64_t stepsize, const std::size_t steps)
{
  const auto dt = stepsize / static_cast<float64_t>(steps);
  for (std::size_t k = 0; k < steps; ++k) {
    const auto k1 = f(x, u, p);
    const auto k2 = f(x + (dt / 2.0) * k1, u, p);
    const auto k3 = f(x + (dt / 2.0) * k2, u, p);
    const auto k4 = f(x + dt * k3, u, p);
    const auto xnext = x + (dt / 6.0) * (k1 + (2.0 * k2) + (2.0 * k3) + k4);
    x = xnext;
  }
  return x;
}

// \brief Version where the function closes over the parameters, this is useful where we want to
//        for example have the function be a member of an object (like BicycleModel). The reason
//        this is not used everywhere is that CasADi wants its parameters to be passed
//        in explicitly, otherwise its AD doesn't realize the parameter is there.
template<class X, class U>
X RK4(X x, U u, std::function<X(X, U)> f, const float64_t stepsize, const std::size_t steps)
{
  // Create a shim that takes a dummy parameter, then ignores it so we can call the above
  const std::function<X(X, U, uint8_t)> shim = [&f](X xx, U uu, uint8_t pp) {
      (void)&pp; return f(xx, uu);
    };

  return RK4(x, u, {}, shim, stepsize, steps);
}

}  // namespace parking_planner
}  // namespace planning
}  // namespace motion
}  // namespace autoware

#endif  // PARKING_PLANNER__RUNGEKUTTA_HPP_
