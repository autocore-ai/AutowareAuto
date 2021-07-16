// Copyright 2020 The Autoware Foundation
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

/// \copyright Copyright 2020 Apex.AI, Inc.
/// All rights reserved.

#include <covariance_insertion/covariance_insertion.hpp>

#include <string>
#include <vector>

namespace autoware
{
namespace covariance_insertion
{
CovarianceInsertion::CovarianceInsertion()
{
}

void CovarianceInsertion::insert_covariance(
  const std::string & field,
  const std::vector<common::types::float64_t> & covariance)
{
  m_covariances[field] = covariance;
}

bool CovarianceInsertion::covariances_empty()
{
  return m_covariances.empty();
}

}  // namespace covariance_insertion
}  // namespace autoware
