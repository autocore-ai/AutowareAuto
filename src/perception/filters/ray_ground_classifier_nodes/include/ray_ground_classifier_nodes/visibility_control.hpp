// Copyright 2017-2018 the Autoware Foundation
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

#ifndef RAY_GROUND_CLASSIFIER_NODES__VISIBILITY_CONTROL_HPP_
#define RAY_GROUND_CLASSIFIER_NODES__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(_MSC_VER) && defined(_WIN64)
  #if defined(RAY_GROUND_CLASSIFIER_NODES_BUILDING_DLL) || \
  defined(RAY_GROUND_CLASSIFIER_NODES_EXPORTS)
    #define RAY_GROUND_CLASSIFIER_NODES_PUBLIC __declspec(dllexport)
    #define RAY_GROUND_CLASSIFIER_NODES_LOCAL
  #else  // defined(RAY_GROUND_CLASSIFIER_NODES_BUILDING_DLL)
// || defined(RAY_GROUND_CLASSIFIER_NODES_EXPORTS)
    #define RAY_GROUND_CLASSIFIER_NODES_PUBLIC __declspec(dllimport)
    #define RAY_GROUND_CLASSIFIER_NODES_LOCAL
  #endif  // defined(RAY_GROUND_CLASSIFIER_NODES_BUILDING_DLL)
// || defined(RAY_GROUND_CLASSIFIER_NODES_EXPORTS)
#elif defined(__GNUC__) && defined(__linux__)
  #define RAY_GROUND_CLASSIFIER_NODES_PUBLIC __attribute__((visibility("default")))
  #define RAY_GROUND_CLASSIFIER_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__GNUC__) && defined(__APPLE__)
  #define RAY_GROUND_CLASSIFIER_NODES_PUBLIC __attribute__((visibility("default")))
  #define RAY_GROUND_CLASSIFIER_NODES_LOCAL __attribute__((visibility("hidden")))
#else  // !(defined(__GNUC__) && defined(__APPLE__))
  #error "Unsupported Build Configuration"
#endif  // _MSC_VER

#endif  // RAY_GROUND_CLASSIFIER_NODES__VISIBILITY_CONTROL_HPP_
