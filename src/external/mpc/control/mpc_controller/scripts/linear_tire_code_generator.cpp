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
#include <acado/acado_code_generation.hpp>

USING_NAMESPACE_ACADO

int main(int argc, char * const argv[])
{
#include <linear_tire.snippet.hpp>
  //
  // Weighting matrices and reference functions: not prespecified
  //

  Function rf;
  Function rfN;

  rf << x << y << yaw << u << ax << delta;
  rfN << x << y << yaw << u;

  BMatrix W = eye<bool>(rf.getDim());
  BMatrix WN = eye<bool>(rfN.getDim());

  //
  // Optimal Control Problem
  //

  // 5 second time horizon
  const int N = 50;  // Number of steps
  const int Ni = 4;  // number of integrators
  const double Ts = 0.1;

  OCP ocp(0, N * Ts, N);

  ocp.subjectTo(f);

  ocp.minimizeLSQ(W, rf);
  ocp.minimizeLSQEndTerm(WN, rfN);

  ocp.subjectTo(0.0 <= u <= 35.0);
  ocp.subjectTo(-3.0 <= v <= 3.0);
  ocp.subjectTo(-0.331 <= delta <= 0.331);
  ocp.subjectTo(-3.0 <= ax <= 3.0);
  ocp.subjectTo(-10.0 <= jx <= 10.0);
  ocp.subjectTo(-0.331 <= delta_dot <= 0.331);

  //
  // Export the code:
  //
  OCPexport mpc(ocp);

  // See https://github.com/cho3/acado/blob/master/acado/utils/acado_types.hpp#L331..L427
  // for all options

  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);

  mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4);
  mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);

  mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
  // mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
  mpc.set(QP_SOLVER, QP_QPOASES);
  mpc.set(MAX_NUM_QP_ITERATIONS, 999);
  mpc.set(HOTSTART_QP, YES);
  mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, NO);
  // mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);
  // mpc.set(QP_SOLVER, QP_FORCES);

  mpc.set(LEVENBERG_MARQUARDT, 1.0e-10);

  mpc.set(GENERATE_TEST_FILE, NO);
  mpc.set(GENERATE_MAKE_FILE, NO);
  mpc.set(GENERATE_MATLAB_INTERFACE, NO);

  // mpc.set(USE_SINGLE_PRECISION, YES);
  // mpc.set(CG_USE_OPENMP, YES);

  if (mpc.exportCode("single_track_dynamics") != SUCCESSFUL_RETURN) {
    exit(EXIT_FAILURE);
  }

  mpc.printDimensionsQP();

  return EXIT_SUCCESS;
}
