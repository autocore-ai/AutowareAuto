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
#include <ndt/ndt_config.hpp>
namespace autoware
{
namespace localization
{
namespace ndt
{

P2DNDTOptimizationConfig::P2DNDTOptimizationConfig(Real outlier_ratio)
: m_outlier_ratio{outlier_ratio} {}

Real P2DNDTOptimizationConfig::outlier_ratio() const noexcept
{
  return m_outlier_ratio;
}

}  // namespace ndt
}  // namespace localization
}  // namespace autoware
