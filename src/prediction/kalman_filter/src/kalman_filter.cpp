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

/// \copyright Copyright 2018 Apex.AI, Inc.
/// All rights reserved.

#include "kalman_filter/esrcf.hpp"

namespace autoware
{
namespace prediction
{
namespace kalman_filter
{
// This is just to get some static analysis
template class Esrcf<4, 4>;
template class SrcfCore<2, 2>;
template class SrcfCore<1, 1>;
template class SrcfCore<2, 1>;
template class SrcfCore<3, 1>;
}  // namespace kalman_filter
}  // namespace prediction
}  // namespace autoware
