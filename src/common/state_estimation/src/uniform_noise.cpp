// Copyright 2021 Apex.AI, Inc.
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

#include <state_estimation/noise_model/uniform_noise.hpp>
#include <state_vector/common_states.hpp>

namespace autoware
{
namespace common
{
namespace state_estimation
{

/// \cond DO_NOT_DOCUMENT

template class STATE_ESTIMATION_PUBLIC UniformNoise<state_vector::ConstAccelerationXY32>;
template class STATE_ESTIMATION_PUBLIC UniformNoise<state_vector::ConstAccelerationXY64>;

template class STATE_ESTIMATION_PUBLIC UniformNoise<state_vector::ConstAccelerationXYZ32>;
template class STATE_ESTIMATION_PUBLIC UniformNoise<state_vector::ConstAccelerationXYZ64>;

template class STATE_ESTIMATION_PUBLIC UniformNoise<state_vector::ConstAccelerationXYYaw32>;
template class STATE_ESTIMATION_PUBLIC UniformNoise<state_vector::ConstAccelerationXYYaw64>;

template class STATE_ESTIMATION_PUBLIC UniformNoise<state_vector::ConstAccelerationXYZYaw32>;
template class STATE_ESTIMATION_PUBLIC UniformNoise<state_vector::ConstAccelerationXYZYaw64>;

template class STATE_ESTIMATION_PUBLIC UniformNoise<state_vector::ConstAccelerationXYZRPY32>;
template class STATE_ESTIMATION_PUBLIC UniformNoise<state_vector::ConstAccelerationXYZRPY64>;

/// \endcond

}  // namespace state_estimation
}  // namespace common
}  // namespace autoware
