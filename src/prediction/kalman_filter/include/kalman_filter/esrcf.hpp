// Copyright 2018 Apex.AI, Inc.
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

/// \copyright Copyright 2018 Apex.AI, Inc.
/// All rights reserved.
/// \file
/// \brief This file defines a class for extended square root covariance filter
#ifndef KALMAN_FILTER__ESRCF_HPP_
#define KALMAN_FILTER__ESRCF_HPP_

#include <common/types.hpp>
#include <motion_model/motion_model.hpp>
#include <kalman_filter/srcf_core.hpp>

#include <chrono>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace prediction
{
namespace kalman_filter
{

/// \brief This class wraps the carlson-schmidt square root covariance filter
///        with some vector-valued motion model. This class assumes fixed G and Q
///        matrices that live in some other scope, and some fixed, diagonal R matrix
///        also living in some other scope. This class itself is currently not intended
///        to do any factorization on its own.
/// \tparam NumStates dimensionality of state space
/// \tparam ProcessNoiseDim dimensionality of process noise space
template<int32_t NumStates, int32_t ProcessNoiseDim>
class Esrcf
{
  using state_vec_t = typename SrcfCore<NumStates, ProcessNoiseDim>::state_vec_t;
  using square_mat_t = typename SrcfCore<NumStates, ProcessNoiseDim>::square_mat_t;

public:
  /// \brief constructor
  /// \param[inout] model motion model used for state propagation and jacobian calculation
  /// \param[in] GQ_chol_prod reference to cholesky factor of process noise covariance
  explicit Esrcf(
    motion::motion_model::MotionModel<NumStates> & model,
    const Eigen::Matrix<float32_t, NumStates, ProcessNoiseDim> & GQ_chol_prod)
  : m_model_ptr{&model},
    m_is_mat1_covariance{false},
    m_B_mat{GQ_chol_prod},
    m_GQ_factor{GQ_chol_prod} {}

  /// \brief constructor, equivalent of construct(model, GQ, DIAG); reset(x, P);
  /// \param[inout] model motion model used for state propagation and jacobian calculation
  /// \param[in] GQ_chol_prod reference to cholesky factor of process noise covariance
  /// \param[in] x0 initial state
  /// \param[in] P0_chol cholesky factor of initial covariance matrix
  explicit Esrcf(
    motion::motion_model::MotionModel<NumStates> & model,
    const Eigen::Matrix<float32_t, NumStates, ProcessNoiseDim> & GQ_chol_prod,
    const state_vec_t & x0,
    const square_mat_t & P0_chol)
  : Esrcf{model, GQ_chol_prod}
  {
    reset(x0, P0_chol);
  }

  /// \brief Update state externally
  /// \param[in] x0 New state
  void reset(const state_vec_t & x0)
  {
    m_model_ptr->reset(x0);
  }


  /// \brief Initialize state and covariance
  /// \param[in] x0 initial state
  /// \param[in] P0_chol cholesky factor of initial covariance matrix
  void reset(const state_vec_t & x0, const square_mat_t & P0_chol)
  {
    reset(x0);
    m_square_mat2 = P0_chol;
    m_is_mat1_covariance = false;
  }

  /// \brief Get covariance
  /// \return const reference to current covariance matrix
  const square_mat_t & get_covariance() const
  {
    return m_is_mat1_covariance ? m_square_mat1 : m_square_mat2;
  }

  /// \brief Do temporal update: update state estimate, compute jacobian, update covariance
  /// \param[in] dt amount of time to propagate state estimate, and compute associated jacobian for
  void temporal_update(const std::chrono::nanoseconds & dt)
  {
    // alternate which matrix you store F * C in to take advantage of efficient matrix mult
    const square_mat_t & cov_mat = m_is_mat1_covariance ? m_square_mat1 : m_square_mat2;
    square_mat_t & jac_mat = m_is_mat1_covariance ? m_square_mat2 : m_square_mat1;
    // compute jacobian and predict
    m_model_ptr->compute_jacobian_and_predict(jac_mat, dt);
    //// update covariance matrix
    // store jacobian old cov matrix, update F = alpha * F * C
    jac_mat = jac_mat * cov_mat;
    // Do temporal update on F, 0 out B
    m_srcf_core.right_lower_triangularize_matrices(jac_mat, m_B_mat);
    m_is_mat1_covariance = !m_is_mat1_covariance;
    m_B_mat = m_GQ_factor;
  }

  /// \brief Do observation update: update state estimate and covariance. temporal_update() is
  ///        assumed to have been called before this (recently).
  /// \param[in] z observation vector
  /// \param[in] H observation matrix: z = H * x
  /// \param[in] R_diag diagonal of measurement noise covariance matrix
  template<int32_t NumObs>
  float32_t observation_update(
    const Eigen::Matrix<float32_t, NumObs, 1U> & z,
    const Eigen::Matrix<float32_t, NumObs, NumStates> & H,
    const Eigen::Matrix<float32_t, NumObs, 1U> & R_diag)
  {
    square_mat_t & cov_mat = m_is_mat1_covariance ? m_square_mat1 : m_square_mat2;
    // This is ugly, but promotes encapsulation
    m_state_tmp = m_model_ptr->get_state();
    // do sequential update
    float32_t likelihood = 0.0F;
    for (index_t idx = index_t(); idx < NumObs; ++idx) {
      likelihood += m_srcf_core.scalar_update(
        z[idx],
        R_diag[idx],
        H.row(idx),
        cov_mat,
        m_state_tmp);
    }
    // still ugly, but promotes encapsulation
    m_model_ptr->reset(m_state_tmp);
    return likelihood;
  }

  /// \brief Compute first component of mixed covariance for this model:
  ///        P_i' = u_i * (P_i + (x_i - x) * (x_i - x)^T)
  ///        Must be called before imm_other_mix() calls, state and covariance
  ///        are updated to x_mix and the cholesky factor of P_i' as above.
  /// \param[in] self_transition_prob Probability an object following this model
  ///                                 continues to follow this motion model in the next time step
  /// \param[in] x_mix The mixed state vector for this model
  void imm_self_mix(const float32_t self_transition_prob, const state_vec_t & x_mix)
  {
    m_state_tmp = m_model_ptr->get_state();
    m_state_tmp -= x_mix;
    const float32_t sq_prob = sqrtf(self_transition_prob);
    square_mat_t & cov = m_is_mat1_covariance ? m_square_mat1 : m_square_mat2;
    cov *= sq_prob;
    m_state_tmp *= sq_prob;
    m_srcf_core.right_lower_triangularize_matrices(cov, m_state_tmp);
    m_model_ptr->reset(x_mix);
  }

  /// \brief Adds covariance components from another model to this model, computes
  ///        P0' = P0 + u_other * (P_other + (x_other - x_mix) * ( x_other - x_mix)^T)
  ///        This may be called r - 1 times after `imm_self_mix()` has been called, where r is the
  ///        number of models you are mixing.
  /// \param[in] other_transition_prob Probability of an object using this motion model
  ///                                  of switching to the motion model cov_other and x_other
  ///                                  correspond to
  /// \param[in] x_other Pre-mixed state of other motion model and filter
  /// \param[inout] cov_other Cholesky factor of pre-mixed covariance matrix of other motion model
  ///                         and filter
  /// \param[in] rank_other Rank of cov_other, e.g. number of columns to zero out
  void imm_other_mix(
    const float32_t other_transition_prob,
    const state_vec_t & x_other,
    square_mat_t & cov_other,
    const index_t rank_other = NumStates)
  {
    // compute dx = x_other - x_mix
    m_state_tmp = x_other;
    m_state_tmp -= m_model_ptr->get_state();
    // compute sq(u), apply to dx, cov_other
    const float32_t sq_prob = sqrtf(other_transition_prob);
    m_state_tmp *= sq_prob;
    cov_other *= sq_prob;
    square_mat_t & cov = m_is_mat1_covariance ? m_square_mat1 : m_square_mat2;
    // triangularize stacked matrix [C_self / sq(u) * C_other / sq(u) * (x_other - x_mix)]
    for (index_t i = index_t(); i < NumStates; ++i) {
      // For each element in column, starting from the bottom
      m_srcf_core.zero_row(cov, m_state_tmp, i, index_t(), 1U);
      m_srcf_core.zero_row(cov, cov_other, i, index_t(), rank_other);
      // Zero out elements of cov  up to, but not including the diagonal
      m_srcf_core.zero_row(cov, cov, i, i + 1, NumStates);
    }
  }

private:
  SrcfCore<NumStates, ProcessNoiseDim> m_srcf_core;
  motion::motion_model::MotionModel<NumStates> * const m_model_ptr;
  state_vec_t m_state_tmp;
  square_mat_t m_square_mat1;
  square_mat_t m_square_mat2;
  bool8_t m_is_mat1_covariance;
  Eigen::Matrix<float32_t, NumStates, ProcessNoiseDim> m_B_mat;
  Eigen::Matrix<float32_t, NumStates, ProcessNoiseDim> m_GQ_factor;
  static_assert(NumStates > 0U, "must have positive number of states");
  static_assert(ProcessNoiseDim > 0U, "must have positive number of process noise dimensions");
};  // class Esrcf
}  // namespace kalman_filter
}  // namespace prediction
}  // namespace autoware

#endif  // KALMAN_FILTER__ESRCF_HPP_
