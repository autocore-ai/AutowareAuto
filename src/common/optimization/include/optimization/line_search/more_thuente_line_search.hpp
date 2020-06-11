// Copyright 2020 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.


// This file contains modified code from the following open source projects
// published under the licenses listed below:
//
// Software License Agreement (BSD License)
//
//  Point Cloud Library (PCL) - www.pointclouds.org
//  Copyright (c) 2010-2011, Willow Garage, Inc.
//  Copyright (c) 2012-, Open Perception, Inc.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of the copyright holder(s) nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#ifndef OPTIMIZATION__LINE_SEARCH__MORE_THUENTE_LINE_SEARCH_HPP_
#define OPTIMIZATION__LINE_SEARCH__MORE_THUENTE_LINE_SEARCH_HPP_

#include <optimization/line_search/line_search.hpp>
#include <optimization/utils.hpp>

#include <limits>
#include <algorithm>

namespace autoware
{
namespace common
{
namespace optimization
{

class OPTIMIZATION_PUBLIC MoreThuenteLineSearch : public LineSearch<MoreThuenteLineSearch>
{
public:
  explicit MoreThuenteLineSearch(const StepT max_step, const StepT min_step)
  : LineSearch{max_step}, m_step_min{min_step} {}

  template<typename DomainValueT, typename OptimizationProblemT>
  DomainValueT compute_step_length_(
    const DomainValueT & x0, const DomainValueT & step_direction,
    OptimizationProblemT & optimization_problem)
  {
    DomainValueT step_out = step_direction.normalized();

    const auto step_max = get_step_max();
    const auto step_min = m_step_min;
    optimization_problem.evaluate(x0, ComputeMode{}.set_score().set_jacobian());
    auto score = optimization_problem(x0);
    typename OptimizationProblemT::Jacobian score_gradient;
    optimization_problem.jacobian(x0, score_gradient);

    double phi_0 = -score;
    double d_phi_0 = -(score_gradient.dot(step_out));

    typename OptimizationProblemT::DomainValue x_t;

    if (d_phi_0 >= 0) {
      if (d_phi_0 == 0) {
        return DomainValueT{}.setZero();
      } else {
        d_phi_0 *= -1;
        step_out *= -1;
      }
    }

    int max_step_iterations = 10;
    int step_iterations = 0;

    double mu = 1.e-4;
    double nu = 0.9;
    double a_l = 0, a_u = 0;

    double f_l = auxilaryFunction_PsiMT(a_l, phi_0, phi_0, d_phi_0, mu);
    double g_l = auxilaryFunction_dPsiMT(d_phi_0, d_phi_0, mu);

    double f_u = auxilaryFunction_PsiMT(a_u, phi_0, phi_0, d_phi_0, mu);
    double g_u = auxilaryFunction_dPsiMT(d_phi_0, d_phi_0, mu);

    bool interval_converged = false, open_interval = true;

    double a_t = step_direction.norm();      // initial step length
    a_t = std::min(a_t, static_cast<double>(step_max));
    a_t = std::max(a_t, static_cast<double>(step_min));

    x_t = x0 + step_out * a_t;

    optimization_problem.evaluate(x_t, ComputeMode{}.set_score().set_jacobian());
    score = optimization_problem(x_t);
    optimization_problem.jacobian(x_t, score_gradient);

    double phi_t = -score;
    double d_phi_t = -(score_gradient.dot(step_out));
    double psi_t = auxilaryFunction_PsiMT(a_t, phi_t, phi_0, d_phi_0, mu);
    double d_psi_t = auxilaryFunction_dPsiMT(d_phi_t, d_phi_0, mu);

    while (!interval_converged && step_iterations < max_step_iterations &&
      !(psi_t <= 0 && d_phi_t <= -nu * d_phi_0))
    {
      if (open_interval) {
        a_t = trialValueSelectionMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
      } else {
        a_t = trialValueSelectionMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
      }

      a_t = std::min(a_t, static_cast<double>(step_max));
      a_t = std::max(a_t, static_cast<double>(step_min));

      x_t = x0 + step_out * a_t;

      optimization_problem.evaluate(x_t, ComputeMode{}.set_score().set_jacobian());
      score = optimization_problem(x_t);
      optimization_problem.jacobian(x_t, score_gradient);

      phi_t -= score;
      d_phi_t -= (score_gradient.dot(step_out));
      psi_t = auxilaryFunction_PsiMT(a_t, phi_t, phi_0, d_phi_0, mu);
      d_psi_t = auxilaryFunction_dPsiMT(d_phi_t, d_phi_0, mu);

      if (open_interval && (psi_t <= 0 && d_psi_t >= 0)) {
        open_interval = false;

        f_l += phi_0 - mu * d_phi_0 * a_l;
        g_l += mu * d_phi_0;

        f_u += phi_0 - mu * d_phi_0 * a_u;
        g_u += mu * d_phi_0;
      }

      if (open_interval) {
        interval_converged =
          updateIntervalMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t) > 0.0;
      } else {
        interval_converged =
          updateIntervalMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t) > 0.0;
      }
      step_iterations++;
    }
    return a_t * step_out;
  }

  ///////////////////////////////////

private:
  double trialValueSelectionMT(
    double a_l, double f_l, double g_l,
    double a_u, double f_u, double g_u,
    double a_t, double f_t, double g_t)
  {
    // Case 1 in Trial Value Selection [More, Thuente 1994]
    if (f_t > f_l) {
      // Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
      // Equation 2.4.52 [Sun, Yuan 2006]
      double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
      double w = std::sqrt(z * z - g_t * g_l);
      // Equation 2.4.56 [Sun, Yuan 2006]
      double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

      // Calculate the minimizer of the quadratic that interpolates f_l, f_t and g_l
      // Equation 2.4.2 [Sun, Yuan 2006]
      double a_q = a_l - 0.5 * (a_l - a_t) * g_l / (g_l - (f_l - f_t) / (a_l - a_t));

      if (std::fabs(a_c - a_l) < std::fabs(a_q - a_l)) {
        return a_c;
      } else {
        return 0.5 * (a_q + a_c);
      }
    } else if (g_t * g_l < 0) {
      // Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
      // Equation 2.4.52 [Sun, Yuan 2006]
      double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
      double w = std::sqrt(z * z - g_t * g_l);
      // Equation 2.4.56 [Sun, Yuan 2006]
      double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

      // Calculate the minimizer of the quadratic that interpolates f_l, g_l and g_t
      // Equation 2.4.5 [Sun, Yuan 2006]
      double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

      if (std::fabs(a_c - a_t) >= std::fabs(a_s - a_t)) {
        return a_c;
      } else {
        return a_s;
      }
    } else if (std::fabs(g_t) <= std::fabs(g_l)) {
      // Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
      // Equation 2.4.52 [Sun, Yuan 2006]
      double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
      double w = std::sqrt(z * z - g_t * g_l);
      double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

      // Calculate the minimizer of the quadratic that interpolates g_l and g_t
      // Equation 2.4.5 [Sun, Yuan 2006]
      double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

      double a_t_next;

      if (std::fabs(a_c - a_t) < std::fabs(a_s - a_t)) {
        a_t_next = a_c;
      } else {
        a_t_next = a_s;
      }

      if (a_t > a_l) {
        return std::min(a_t + 0.66 * (a_u - a_t), a_t_next);
      } else {
        return std::max(a_t + 0.66 * (a_u - a_t), a_t_next);
      }
    } else {
      // Calculate the minimizer of the cubic that interpolates f_u, f_t, g_u and g_t
      // Equation 2.4.52 [Sun, Yuan 2006]
      double z = 3 * (f_t - f_u) / (a_t - a_u) - g_t - g_u;
      double w = std::sqrt(z * z - g_t * g_u);
      // Equation 2.4.56 [Sun, Yuan 2006]
      return a_u + (a_t - a_u) * (w - g_u - z) / (g_t - g_u + 2 * w);
    }
  }

// Copied from ndt.hpp
  double updateIntervalMT(
    double & a_l, double & f_l, double & g_l,
    double & a_u, double & f_u, double & g_u,
    double a_t, double f_t, double g_t)
  {
    // Case U1 in Update Algorithm and Case a in Modified Update Algorithm [More, Thuente 1994]
    if (f_t > f_l) {
      a_u = a_t;
      f_u = f_t;
      g_u = g_t;
      return false;
    } else if (g_t * (a_l - a_t) > 0) {
      a_l = a_t;
      f_l = f_t;
      g_l = g_t;
      return false;
    } else if (g_t * (a_l - a_t) < 0) {
      a_u = a_l;
      f_u = f_l;
      g_u = g_l;

      a_l = a_t;
      f_l = f_t;
      g_l = g_t;
      return false;
    } else {
      return true;
    }
  }

  double auxilaryFunction_PsiMT(double a, double f_a, double f_0, double g_0, double mu)
  {
    return f_a - f_0 - mu * g_0 * a;
  }

  double auxilaryFunction_dPsiMT(double g_a, double g_0, double mu)
  {
    return g_a - mu * g_0;
  }

private:
  StepT m_step_min;
};

}  // namespace optimization
}  // namespace common
}  // namespace autoware


#endif  // OPTIMIZATION__LINE_SEARCH__MORE_THUENTE_LINE_SEARCH_HPP_
