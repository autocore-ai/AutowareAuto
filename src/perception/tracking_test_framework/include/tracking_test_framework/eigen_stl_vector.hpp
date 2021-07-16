// Copyright 2021 The Autoware Foundation
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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file contains the typedef for using fixed size EigenVectors with std::vector due
/// to issues documented here : https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html

#ifndef TRACKING_TEST_FRAMEWORK__EIGEN_STL_VECTOR_HPP_
#define TRACKING_TEST_FRAMEWORK__EIGEN_STL_VECTOR_HPP_

#include <Eigen/StdVector>

#include <vector>

template<typename EigenVectorT>
using EigenStlVector = std::vector<EigenVectorT, Eigen::aligned_allocator<EigenVectorT>>;

#endif  // TRACKING_TEST_FRAMEWORK__EIGEN_STL_VECTOR_HPP_
