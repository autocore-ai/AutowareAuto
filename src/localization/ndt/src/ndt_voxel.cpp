// Copyright 2019 Apex.AI, Inc.
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

#include <ndt/ndt_voxel.hpp>
#include <ndt/utils.hpp>
#include <Eigen/LU>
#include <limits>
#include "common/types.hpp"

using autoware::common::types::bool8_t;

namespace autoware
{
namespace localization
{
namespace ndt
{
constexpr uint32_t DynamicNDTVoxel::NUM_POINT_THRESHOLD;

DynamicNDTVoxel::DynamicNDTVoxel()
{
  // default constructors use uninitialized values.
  m_centroid.setZero();
  m_M2.setZero();
  m_covariance.setZero();
}

void DynamicNDTVoxel::add_observation(const Point & pt)
{
  const auto last_count = m_num_points++;

  const auto last_centroid = m_centroid;
  const auto last_delta = pt - last_centroid;
  m_centroid = last_centroid + (last_delta / m_num_points);

  const auto current_delta = pt - m_centroid;
  m_M2 += last_delta * current_delta.transpose();

  if (usable()) {
    // TODO(yunus.caliskan): Apply numerical stability enhancing steps described in:
    // http://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf, pg 60
    m_covariance = m_M2 / last_count;
  }
}

bool8_t DynamicNDTVoxel::usable() const noexcept
{
  return m_num_points >= NUM_POINT_THRESHOLD;
}

const Eigen::Matrix3d & DynamicNDTVoxel::covariance_() const
{
  if (!usable()) {
    throw std::out_of_range("DynamicNDTVoxel: Cannot get covariance from a "
            "voxel without sufficient number of points");
  }
  return m_covariance;
}

const Eigen::Vector3d & DynamicNDTVoxel::centroid_() const
{
  // Using the overloaded function as the parent function will use the hidden occupancy check
  if (!usable()) {
    throw std::out_of_range("DynamicNDTVoxel: Cannot get centroid from an unoccupied voxel");
  }
  return m_centroid;
}

uint64_t DynamicNDTVoxel::count() const noexcept
{
  return m_num_points;
}

/////////////////////////////////////////////////

StaticNDTVoxel::StaticNDTVoxel()
{
  m_covariance.setZero();
  m_centroid.setZero();
  m_inv_covariance.setZero();
}

StaticNDTVoxel::StaticNDTVoxel(const Point & centroid, const Cov & covariance)
: m_centroid{centroid}, m_covariance{covariance}, m_occupied{true}
{
  if (try_stabilize_covariance(m_covariance)) {
    bool8_t invertible{false};
    m_covariance.computeInverseWithCheck(m_inv_covariance, invertible);
    if (!invertible) {
      m_occupied = false;
      // TODO(yunus.caliskan): Move this to the dynamic voxel in #216
    }
  } else {
    m_occupied = false;
  }
}

const Eigen::Matrix3d & StaticNDTVoxel::covariance_() const
{
  if (!m_occupied) {
    throw std::out_of_range("StaticNDTVoxel: Cannot get covariance from an unoccupied voxel");
  }
  return m_covariance;
}

const Eigen::Vector3d & StaticNDTVoxel::centroid_() const
{
  if (!m_occupied) {
    throw std::out_of_range("StaticNDTVoxel: Cannot get centroid from an unoccupied voxel");
  }
  return m_centroid;
}

const Eigen::Matrix3d & StaticNDTVoxel::inverse_covariance() const
{
  if (!m_occupied) {
    throw std::out_of_range("StaticNDTVoxel: Cannot get inverse covariance. "
            "The voxel is either empty or has singular covariance.");
  }
  return m_inv_covariance;
}

bool8_t StaticNDTVoxel::usable() const noexcept
{
  return m_occupied;
}

}  // namespace ndt
}  // namespace localization
}  // namespace autoware
