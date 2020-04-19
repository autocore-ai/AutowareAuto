// Copyright 2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
#include <common/types.hpp>
#include <chrono>
#include <Eigen/Cholesky>
#include "kalman_filter/srcf_core.hpp"
#include "kalman_filter/esrcf.hpp"
#include "motion_model/constant_velocity.hpp"
#include "motion_model/parameter_estimator.hpp"

using autoware::prediction::kalman_filter::SrcfCore;
using autoware::prediction::kalman_filter::Esrcf;
using autoware::motion::motion_model::ConstantVelocity;
using autoware::motion::motion_model::ParameterEstimator;
using Eigen::Matrix;
using autoware::common::types::float32_t;
const float32_t TOL = 1.0E-6F;

template<typename T, uint64_t H, uint64_t W>
void print(const Matrix<T, H, W> & A)
{
  std::cerr << "\nPrint Matrix:\n\n";
  for (uint64_t i = 0; i < H; ++i) {
    for (uint64_t j = 0; j < W; ++j) {
      std::cerr << A(i, j) << ", \t";
    }
    std::cerr << "\n";
  }
}

// example 5.4, in Kalman Filtering Theory and Practice using Matlab, pg 193-194
TEST(srcf_core, univariate)
{
  Matrix<float32_t, 1, 1> F, H, Q, R, x, z, P, B, G;
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::start();
  const float32_t log2pi = 1.83787706641F;
  EXPECT_FLOAT_EQ(log2pi, logf(3.14159265359F * 2.0F));
  F << 1;
  H << 1;
  Q << 1;  // sqrt, but 1 so whatever
  R << 2;
  x << 1;
  P << sqrtf(10.0F);
  G << 1;  // no mapping from procss noise subspace to state space
  SrcfCore<1, 1> core;
  // temporal update 0->1
  P(0) = F(0) * P(0);
  B(0) = G(0) * Q(0);
  core.right_lower_triangularize_matrices(P, B);
  EXPECT_LT(fabsf(B(0) - 0.0F), TOL);
  EXPECT_LT(fabsf(P(0) - sqrtf(11.0F)), TOL);
  // observation update 1
  z(0) = 2;
  float32_t S = (H(0) * (P(0) * P(0)) * H(0)) + R(0);
  float32_t err = z(0) - (H(0) * x(0));
  float32_t pdf = -0.5F * (log2pi + logf(S) + (err * err) / S);
  EXPECT_LT(fabsf(core.scalar_update(z(0), R(0), H, P, x) - pdf), TOL) << pdf;
  EXPECT_LT(fabsf(x(0) - 24.0F / 13.0F), TOL);
  EXPECT_LT(fabsf(P(0) - sqrtf(22.0 / 13.0)), TOL);
  // temporal update 1->2
  P(0) = F(0) * P(0);
  B(0) = G(0) * Q(0);
  core.right_lower_triangularize_matrices(P, B);
  EXPECT_LT(fabsf(B(0) - 0.0F), TOL);
  EXPECT_LT(fabsf(P(0) - sqrtf(35.0F / 13.0F)), TOL);
  // observation update 2
  z(0) = 3;
  S = (H(0) * (P(0) * P(0)) * H(0)) + R(0);
  err = z(0) - (H(0) * x(0));
  pdf = -0.5F * (log2pi + logf(S) + (err * err) / S);
  EXPECT_LT(fabsf(core.scalar_update(z(0), R(0), H, P, x) - pdf), TOL) << pdf;
  EXPECT_LT(fabsf(x(0) - 153.0F / 61.0F), TOL);
  EXPECT_LT(fabsf(P(0) - sqrtf(70.0F / 61.0F)), TOL);
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::stop();
  // test bad case
  EXPECT_THROW(core.scalar_update(0.0F, 0.0F, H, P, x), std::domain_error);
}

// example 5.5 in Kalman Filtering Theory and Pracice using Matlab, pg 195-196
TEST(srcf_core, multivariate)
{
  Matrix<float32_t, 2, 1> x({1, 2}), z({3, 4});
  Matrix<float32_t, 2, 2> H, C, R;
  H << 0, 2, 3, 0;
  C << 4, 1, 1, 9;
  R << 1, 0, 0, 4;
  Matrix<float32_t, 2, 1> h_row = H.row(0);
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::start();
  // higher TOL to match matlab precision
  const float32_t TOL2 = 1.0E-4F;
  Eigen::LLT<Eigen::Ref<decltype(C)>> llt(C);

  C(0U, 1U) = 0.0F;
  // cholesky from numpy
  EXPECT_LT(fabsf(C(0, 0) - 2), TOL);
  EXPECT_LT(fabsf(C(1, 0) - 0.5F), TOL);
  EXPECT_LT(fabsf(C(1, 1) - 2.95803989F), TOL);
  // do vector update
  SrcfCore<2, 2> core;
  // observation update 1a
  float32_t likelihood = core.scalar_update(z(0), R(0, 0), H.row(0), C, x);
  //// check intermediate values
  // These values are from the worked example in the book
  EXPECT_LT(fabsf(x(0) - (35.0 / 37.0)), TOL2);
  EXPECT_LT(fabsf(x(1) - (56.0 / 37.0)), TOL2);
  // check C, computed manually with scipy
  EXPECT_LT(fabsf(C(0, 0) - (1.97278785)), TOL2);
  EXPECT_LT(fabsf(C(1, 0) - (0.01369992)), TOL2);
  EXPECT_LT(fabsf(C(1, 1) - (0.49300665)), TOL2);
  // observation update 1b
  likelihood += core.scalar_update(z(1), R(1, 1), H.row(1), C, x);
  //// check final values
  // These values are from the worked example in the book
  EXPECT_LT(fabsf(x(0) - (467.0 / 361.0)), TOL2);
  EXPECT_LT(fabsf(x(1) - (2189.0 / 1444.0)), TOL2);
  // check C
  EXPECT_LT(fabsf(C(0, 0) - (0.63157895)), TOL2);
  EXPECT_LT(fabsf(C(1, 0) - (0.00438596)), TOL2);
  EXPECT_LT(fabsf(C(1, 1) - (0.49300665)), TOL2);
  // check likelihood of exact matrix:
  // S = H * P * H' + R = [37, 6; 6, 40]
  // err = z - (H * x) = [-1; 1]
  // P(err | 0, S) = -5.506280400650966 (from scipy)
  EXPECT_LT(fabsf(likelihood - (-5.506280400650966)), 0.01F) << likelihood;
  // Close enough, < 1% relative error
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::stop();
}

// example from Thornton's thesis (see below), pg 28
// This is a case where the conventional kalman filter will fail but the square root will not
// Because the example uses a different factorization paradigm, the answers in thes Thesis
// don't work (they use P = U * U^T, modern standard is P = U^T * U)
/*
s = some big number
e = 1/s = some small number
P = [s^2, 0; 0, s^2]
H = [1, e; 1, 1]
R = [1, 1]
P1 = [
⎡    2        3     ⎤
⎢ 2⋅s       -s      ⎥
⎢──────    ──────   ⎥
⎢ 2         2       ⎥
⎢s  + 2    s  + 2   ⎥
⎢                   ⎥ ~ [2    -s ]
⎢   3     2 ⎛ 2    ⎞⎥   [-s   s^2]
⎢ -s     s ⋅⎝s  + 1⎠⎥
⎢──────  ───────────⎥
⎢ 2          2      ⎥
⎣s  + 2     s  + 2  ⎦
S1 = L1 =
⎡           ________              ⎤
⎢          ╱    2                 ⎥
⎢         ╱    s                  ⎥
⎢  √2⋅   ╱   ──────         0     ⎥
⎢       ╱     2                   ⎥
⎢     ╲╱     s  + 2               ⎥
⎢                                 ⎥
⎢            ________             ⎥ ~ [sqrtf(2)         0             ]
⎢           ╱    2                ⎥   [-sqrf(2)/2 * s   sqrtf(2)/2 * s]
⎢          ╱    s                 ⎥
⎢-√2⋅s⋅   ╱   ──────          ____⎥
⎢        ╱     2             ╱  2 ⎥
⎢      ╲╱     s  + 2    √2⋅╲╱  s  ⎥
⎢─────────────────────  ──────────⎥
⎣          2                2     ⎦

Above were computed exactly using sympy

Similarly, the factors end up being
⎡  _________                       ⎤
⎢╲╱ 2⋅e + 1             0          ⎥
⎢                                  ⎥
⎢                  ________________⎥
⎢                 ╱    2           ⎥
⎢-(3⋅e + 1)      ╱  - e  + 2⋅e + 1 ⎥
⎢───────────    ╱   ────────────── ⎥
⎢  _________  ╲╱       2⋅e + 1     ⎥
⎣╲╱ 2⋅e + 1                        ⎦

for the second update
*/
// TODO(esteve): Test disabled until we fix
// https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/issues/92
TEST(srcf_core, DISABLED_degenerate)
{
  const float32_t eps = 1.0E-6F;
  EXPECT_LT(fabsf((1.0F + (eps * eps)) - 1), TOL);
  const float32_t sigma = 1.0F / eps;
  Matrix<float32_t, 2, 2> H, C;
  H << 1, eps, 1, 1;
  C << sigma * sigma, 0, 0, sigma * sigma;
  Matrix<float32_t, 2, 1> R({1, 1}), x;
  // cholesky on C
  Eigen::LLT<Eigen::Ref<decltype(C)>> llt(C);

  C(0, 1) = 0.0F;
  SrcfCore<2, 2> core;
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::start();
  // update
  float32_t ll = core.scalar_update(0, R(0), H.row(0), C, x);
  EXPECT_LT(fabsf(C(0, 0) - sqrtf(2)), TOL);
  EXPECT_LT(fabsf(C(1, 0) - (-sqrtf(2) * sigma / 2)), TOL);
  EXPECT_LT(fabsf(C(1, 1) - sqrtf(2) * sigma / 2), TOL);
  ll += core.scalar_update(0, R(1), H.row(1), C, x);
  EXPECT_LT(fabsf(C(0, 0) - (1)), TOL);
  EXPECT_LT(fabsf(C(1, 0) - (-1 - 3 * eps)), 0.1F);  // This one is hard to compute exactly
  EXPECT_LT(fabsf(C(1, 1) - (1)), TOL);
  // S = [sig^2, sig^2; sig^2, 2*sig^2]
  // z = err = x = 0
  EXPECT_LT(fabsf(ll - (-29.468897182338893)), 1.0E-5F) << ll;
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::stop();
}

// Example 3.1 in page 66 in NASA JPL paper:
// https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19770005172.pdf
// Modified from reference, since they use U*U^T = P, where convention is U^T * U = P
// test covariance temporal update
TEST(srcf_core, propagation)
{
  Matrix<float32_t, 2, 1> GQ({0, 1});  // B = [0; 1], Q = 1
  const float32_t eps = 1.0E-5F;
  const float32_t sigma = 1.0F / eps;
  // sigma ^2 + 1 ~= sigma ^2
  EXPECT_LT(fabsf((sigma * sigma + 1) - (sigma * sigma)), TOL);
  Matrix<float32_t, 2, 2> C;
  C << sigma, 0, sigma, 1;
  SrcfCore<2, 1> core;
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::start();
  core.right_lower_triangularize_matrices(C, GQ);
  EXPECT_LT(fabsf(C(1, 1) - (sqrtf(2))), eps) << C(0, 0);
  EXPECT_LT(fabsf(C(1, 0) - (sigma)), eps) << C(1, 0);
  EXPECT_LT(fabsf(C(0, 0) - (sigma)), eps) << C(1, 1);
  // B should be zero'd
  EXPECT_LT(fabsf(GQ(0)), eps) << GQ(0);
  EXPECT_LT(fabsf(GQ(1)), eps) << GQ(1);
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::stop();
}

// test other branches of givens rotations
TEST(srcf_core, triangularization)
{
  SrcfCore<3, 1> core;
  Matrix<float32_t, 3, 3> A;
  A <<
    1, 1, 1,
    1, -1, 1,
    1, 1, 0;
  Matrix<float32_t, 3, 3> C(A), D(A), E(A);
  Matrix<float32_t, 3, 1> B;  // zero
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::start();
  core.right_lower_triangularize_matrices(A, B);
  // B should remain 0
  EXPECT_LT(fabsf(B(0)), TOL);
  EXPECT_LT(fabsf(B(1)), TOL);
  EXPECT_LT(fabsf(B(2)), TOL);
  // lower triangle of A should be 0
  EXPECT_LT(fabsf(A(0, 1)), TOL);
  EXPECT_LT(fabsf(A(0, 2)), TOL);
  EXPECT_LT(fabsf(A(1, 2)), TOL);
  // Not NAN
  EXPECT_NE(A(0, 0), NAN);
  EXPECT_NE(A(1, 0), NAN);
  EXPECT_NE(A(2, 0), NAN);
  EXPECT_NE(A(1, 1), NAN);
  EXPECT_NE(A(2, 1), NAN);
  EXPECT_NE(A(2, 2), NAN);
  //// yet another case
  E(0, 0) = 0;
  E(2, 0) = -1;
  core.right_lower_triangularize_matrices(E, B);
  // B should remain 0
  EXPECT_LT(fabsf(B(0)), TOL);
  EXPECT_LT(fabsf(B(1)), TOL);
  EXPECT_LT(fabsf(B(2)), TOL);
  // lower triangle of A should be 0
  EXPECT_LT(fabsf(E(0, 1)), TOL);
  EXPECT_LT(fabsf(E(0, 2)), TOL);
  EXPECT_LT(fabsf(E(1, 2)), TOL);
  // Not NAN
  EXPECT_NE(E(0, 0), NAN);
  EXPECT_NE(E(1, 0), NAN);
  EXPECT_NE(E(2, 0), NAN);
  EXPECT_NE(E(1, 1), NAN);
  EXPECT_NE(E(1, 2), NAN);
  EXPECT_NE(E(2, 2), NAN);
  // another arbitrary case
  B(2) = -1;
  C(2, 1) = -1;
  core.right_lower_triangularize_matrices(C, B);
  EXPECT_LT(fabsf(B(0)), TOL);
  EXPECT_LT(fabsf(B(1)), TOL);
  EXPECT_LT(fabsf(B(2)), TOL);
  // lower triangle of A should be 0
  EXPECT_LT(fabsf(C(0, 1)), TOL);
  EXPECT_LT(fabsf(C(0, 2)), TOL);
  EXPECT_LT(fabsf(C(1, 2)), TOL);
  // Not NAN
  EXPECT_NE(C(0, 0), NAN);
  EXPECT_NE(C(1, 0), NAN);
  EXPECT_NE(C(2, 0), NAN);
  EXPECT_NE(C(1, 1), NAN);
  EXPECT_NE(C(1, 2), NAN);
  EXPECT_NE(C(2, 2), NAN);
  // one more case
  B(0) = -1;
  B(1) = -2;
  B(2) = 1;
  D(0, 0) = 0;
  D(1, 1) = 0;
  core.right_lower_triangularize_matrices(D, B);
  EXPECT_LT(fabsf(B(0)), TOL);
  EXPECT_LT(fabsf(B(1)), TOL);
  EXPECT_LT(fabsf(B(2)), TOL);
  // lower triangle of A should be 0
  EXPECT_LT(fabsf(D(0, 1)), TOL);
  EXPECT_LT(fabsf(D(0, 2)), TOL);
  EXPECT_LT(fabsf(D(1, 2)), TOL);
  // Not NAN
  EXPECT_NE(D(0, 0), NAN);
  EXPECT_NE(D(1, 0), NAN);
  EXPECT_NE(D(2, 0), NAN);
  EXPECT_NE(D(1, 1), NAN);
  EXPECT_NE(D(1, 2), NAN);
  EXPECT_NE(D(2, 2), NAN);
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::stop();
}

// given a correct initial guess and perfect observations, state
// should not change and covariance should shrink to steady state
TEST(esrcf, convergence)
{
  ConstantVelocity model;
  Matrix<float32_t, 4, 1> R({1, 1, 1, 1});
  // identity
  Matrix<float32_t, 4, 4> GQ;
  GQ <<
    0.1F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.1F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.1F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.1F;

  Matrix<float32_t, 4, 1> x({0, 0, 1, -1});  // all 0's
  // cholesky of: (so there is some covariance wrt hidden state
  // 1   0   0.5 0
  // 0   1   0   0.5
  // 0.5 0   1   0
  // 0   0.5 0   1
  Matrix<float32_t, 4, 4> P;
  P <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;
  // prefit and postfit covariance matrices
  Matrix<float32_t, 4, 4> P_m(P), P_p(P), P_last;
  const Matrix<float32_t, 4, 4> H((Matrix<float32_t, 4, 4>() <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1 ).finished());

  Esrcf<4, 4> kf(model, GQ);
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::start();
  kf.reset(x, P);
  float32_t last_ll = -std::numeric_limits<float32_t>::max();
  // microseconds_100 = 0.1s
  std::chrono::nanoseconds microseconds_100(100000000LL);
  for (uint32_t idx = 0; idx < 30; ++idx) {
    x(0) += 0.1;
    x(1) -= 0.1;
    kf.temporal_update(microseconds_100);
    // covariance should grow wrt postfit
    EXPECT_GT(kf.get_covariance()(0, 0), P_p(0, 0));
    EXPECT_GT(kf.get_covariance()(1, 1), P_p(1, 1));
    EXPECT_GT(kf.get_covariance()(2, 2), P_p(2, 2));
    EXPECT_GT(kf.get_covariance()(3, 3), P_p(3, 3));
    // covariance should be smaller than last prefit
    if (idx > 0) {
      EXPECT_LE(kf.get_covariance()(0, 0), P_m(0, 0));
      EXPECT_LE(kf.get_covariance()(1, 1), P_m(1, 1));
      EXPECT_LE(kf.get_covariance()(2, 2), P_m(2, 2));
      EXPECT_LE(kf.get_covariance()(3, 3), P_m(3, 3));
    }
    P_m = kf.get_covariance();
    // state should still be 0
    EXPECT_LT(fabsf(model[0] - x(0)), TOL);
    EXPECT_LT(fabsf(model[1] - x(1)), TOL);
    EXPECT_LT(fabsf(model[2] - x(2)), TOL);
    EXPECT_LT(fabsf(model[3] - x(3)), TOL);
    //// exact observation
    const float32_t ll = kf.observation_update(x, H, R);
    // likelihood should improve
    EXPECT_GT(ll, last_ll);
    last_ll = ll;
    // covariance should shrink
    EXPECT_LE(kf.get_covariance()(0, 0), P_m(0, 0));
    EXPECT_LE(kf.get_covariance()(1, 1), P_m(1, 1));
    EXPECT_LE(kf.get_covariance()(2, 2), P_m(2, 2));
    EXPECT_LE(kf.get_covariance()(3, 3), P_m(3, 3));
    // covariance should also be smaller wrt last postfit
    EXPECT_LE(kf.get_covariance()(0, 0), P_p(0, 0));
    EXPECT_LE(kf.get_covariance()(1, 1), P_p(1, 1));
    EXPECT_LE(kf.get_covariance()(2, 2), P_p(2, 2));
    EXPECT_LE(kf.get_covariance()(3, 3), P_p(3, 3));
    P_last = P_p;
    P_p = kf.get_covariance();
    // state should still be 0
    EXPECT_LT(fabsf(model[0] - x(0)), TOL);
    EXPECT_LT(fabsf(model[1] - x(1)), TOL);
    EXPECT_LT(fabsf(model[2] - x(2)), TOL);
    EXPECT_LT(fabsf(model[3] - x(3)), TOL);
  }
  P_last -= P_p;
  float32_t norm = 0.0F;
  for (uint32_t i = 0; i < P_last.rows(); ++i) {
    for (uint32_t j = 0; j < P_last.cols(); ++j) {
      norm += P_last(i, j) * P_last(i, j);
    }
  }
  EXPECT_LE(sqrtf(norm), 0.0005);
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::stop();
}


// Hidden states should converge to a good value
TEST(esrcf, hidden_state)
{
  ConstantVelocity model;
  Matrix<float32_t, 2, 1> R({0.001F, 0.001F});
  // identity
  Matrix<float32_t, 4, 4> GQ;
  GQ <<
    0.125F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.125F, 0.0F, 0.0F,
    0.5F, 0.0F, 0.1F, 0.0F,
    0.0F, 0.5F, 0.0F, 0.1F
  ;
  const float32_t vx = 1.0F, vy = 1.0F;
  Matrix<float32_t, 4, 1> x({0, 0, 0.0, 0.0});  // all 0's
  // cholesky of: (so there is some covariance wrt hidden state
  // 1   0   0.5 0
  // 0   1   0   0.5
  // 0.5 0   1   0
  // 0   0.5 0   1
  Matrix<float32_t, 4, 4> P;
  P <<
    1.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 10.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 10.0F
  ;
  // prefit and postfit covariance matrices
  Matrix<float32_t, 4, 4> P_m(P), P_p(P);
  const Matrix<float32_t, 4, 4> P0(P);
  const Matrix<float32_t, 2, 4> H((Matrix<float32_t, 2, 4>() <<
    1, 0, 0, 0,
    0, 1, 0, 0).finished());

  Matrix<float32_t, 2, 1> z({0, 0});
  Esrcf<4, 4> kf(model, GQ, x, P);
  EXPECT_NE(model[ConstantVelocity::States::VELOCITY_X], vx);
  EXPECT_NE(model[ConstantVelocity::States::VELOCITY_Y], vy);
  // microseconds_100 = 0.1s
  const std::chrono::nanoseconds microseconds_100(100000000LL);
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::start();
  // velocity error should be shrinking
  float32_t err_u = std::numeric_limits<float32_t>::max();
  float32_t err_v = std::numeric_limits<float32_t>::max();
  float32_t last_ll = -std::numeric_limits<float32_t>::max();
  for (uint32_t idx = 0; idx < 90; ++idx) {
    z(0) += static_cast<float32_t>(microseconds_100.count()) / 1000000000LL * vx;
    z(1) += static_cast<float32_t>(microseconds_100.count()) / 1000000000LL * vy;
    kf.temporal_update(microseconds_100);
    // covariance should grow wrt postfit
    EXPECT_GT(kf.get_covariance()(0, 0), P_p(0, 0)) << idx;
    EXPECT_GT(kf.get_covariance()(1, 1), P_p(1, 1)) << idx;
    // EXPECT_GT(kf.get_covariance()(2, 2), P_p(2, 2)) << idx;
    // EXPECT_GT(kf.get_covariance()(3, 3), P_p(3, 3)) << idx;
    // Analytically, the updated velocity/hidden state covariance is
    // t^2 * (p_v + s^2), where t is the characteristic time step, and s is the characteristic
    // measurement noise. Since the time step is < 1, then variance shrinks
    // Hidden state error should shrink
    float32_t err = fabsf(model[2] - vx);
    EXPECT_LT(err, err_u) << idx;
    err_u = err;
    err = fabsf(model[3] - vy);
    EXPECT_LT(err, err_v) << idx;
    err_v = err;
    if ((err_u < TOL) && (err_v < TOL)) {
      // TODO(ltbj): implement memory_test after the completion of #39
      // osrf_testing_tools_cpp::memory_test::pause();
      std::cout << "Converged at " << idx << "\n";
      // TODO(ltbj): implement memory_test after the completion of #39
      // osrf_testing_tools_cpp::memory_test::resume();
      break;
    }
    P_m = kf.get_covariance();
    //// exact observation
    const float32_t ll = kf.observation_update(z, H, R);
    // likelihood should improve or converge
    EXPECT_GE(ll, last_ll) << idx;
    last_ll = ll;
    // covariance should shrink or converge
    EXPECT_LE(kf.get_covariance()(0, 0), P_m(0, 0)) << idx;
    EXPECT_LE(kf.get_covariance()(1, 1), P_m(1, 1)) << idx;
    EXPECT_LE(kf.get_covariance()(2, 2), P_m(2, 2)) << idx;
    EXPECT_LE(kf.get_covariance()(3, 3), P_m(3, 3)) << idx;
    // postfit covariance should always be smaller than p0
    EXPECT_LT(kf.get_covariance()(0, 0), P0(0, 0)) << idx;
    EXPECT_LT(kf.get_covariance()(1, 1), P0(1, 1)) << idx;
    EXPECT_LT(kf.get_covariance()(2, 2), P0(2, 2)) << idx;
    EXPECT_LT(kf.get_covariance()(3, 3), P0(3, 3)) << idx;
    P_p = kf.get_covariance();
    // state should be very close to observation
    EXPECT_LT(fabsf(model[0] - z(0)), kf.get_covariance()(0, 0)) << idx;
    EXPECT_LT(fabsf(model[1] - z(0)), kf.get_covariance()(1, 1)) << idx;
  }
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::stop();
}

TEST(esrcf, imm_mix)
{
  static const uint32_t dim = 2U;
  Matrix<float32_t, dim, dim> P1, P2, P, C, GQ;
  Matrix<float32_t, dim, 1> x1, x2, x_mix;
  P1 << 2.0F, 2.0F, 2.0F, 4.0F;
  P2 << 2.0F, 1.0F, 1.0F, 2.0F;
  C = P1;
  Eigen::LLT<Eigen::Ref<decltype(C)>> llt(C);

  C(0U, 1U) = 0.0F;
  const float32_t u1 = 0.75F;
  const float32_t u2 = 0.25F;
  x1 = {-3.0F, 5.0F};
  x2 = {5.0F, -3.0F};
  x_mix = {-1.0F, 3.0F};
  ParameterEstimator<dim> model;
  Esrcf<dim, dim> kf(model, GQ, x1, C);
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::start();
  // P1' = u P1 * (x1 - x) (x1 - x)^T
  kf.imm_self_mix(u1, x_mix);
  EXPECT_FLOAT_EQ(model.get_state()(0U), -1.0F);
  EXPECT_FLOAT_EQ(model.get_state()(1U), 3.0F);
  P = P1;
  // dx = {-2, 2} --> dx * dx' = {4, -4; -4, 4}
  P += (Matrix<float32_t, dim, dim>() << 4.0F, -4.0F, -4.0F, 4.0F).finished();
  P *= u1;
  C = P;
  llt.compute(C);
  C(0U, 1U) = 0.0F;
  EXPECT_FLOAT_EQ(kf.get_covariance()(0U, 0U), C(0U, 0U));
  EXPECT_LT(fabsf(kf.get_covariance()(0U, 1U) - C(0U, 1U)), TOL);
  EXPECT_FLOAT_EQ(kf.get_covariance()(1U, 0U), C(1U, 0U));
  EXPECT_FLOAT_EQ(kf.get_covariance()(1U, 1U), C(1U, 1U));
  // Apply update from other model
  C = P2;
  llt.compute(C);

  C(0U, 1U) = 0.0F;
  kf.imm_other_mix(u2, x2, C, dim);
  // inputs should be zero'd after triangularization
  EXPECT_LT(fabsf(C(0U, 0U)), TOL);
  EXPECT_LT(fabsf(C(0U, 1U)), TOL);
  EXPECT_LT(fabsf(C(1U, 0U)), TOL);
  EXPECT_LT(fabsf(C(1U, 1U)), TOL);
  // internal state should remain unchanged
  EXPECT_FLOAT_EQ(model.get_state()(0U), -1.0F);
  EXPECT_FLOAT_EQ(model.get_state()(1U), 3.0F);
  // Compute updated covariance
  // dx2 = {6, -6}
  P2 += (Matrix<float32_t, dim, dim>() << 36.0F, -36.0F, -36.0F, 36.0F).finished();
  P2 *= u2;
  P += P2;
  C = P;
  C(0U, 1U) = 0.0F;
  llt.compute(C);

  EXPECT_FLOAT_EQ(kf.get_covariance()(0U, 0U), C(0U, 0U));
  EXPECT_LT(fabsf(kf.get_covariance()(0U, 1U) - C(0U, 1U)), TOL);
  EXPECT_FLOAT_EQ(kf.get_covariance()(1U, 0U), C(1U, 0U));
  EXPECT_FLOAT_EQ(kf.get_covariance()(1U, 1U), C(1U, 1U));
  // TODO(ltbj): implement memory_test after the completion of #39
  // osrf_testing_tools_cpp::memory_test::stop();
}
