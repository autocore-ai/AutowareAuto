// Copyright 2021 Tier IV, Inc
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

#ifndef OUTLIER_FILTER_NODES__VISIBILITY_CONTROL_HPP_
#define OUTLIER_FILTER_NODES__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(OUTLIER_FILTER_NODES_BUILDING_DLL) || defined(OUTLIER_FILTER_NODES_EXPORTS)
    #define OUTLIER_FILTER_NODES_PUBLIC __declspec(dllexport)
    #define OUTLIER_FILTER_NODES_LOCAL
  #else  // defined(OUTLIER_FILTER_NODES_BUILDING_DLL) || defined(OUTLIER_FILTER_NODES_EXPORTS)
    #define OUTLIER_FILTER_NODES_PUBLIC __declspec(dllimport)
    #define OUTLIER_FILTER_NODES_LOCAL
  #endif  // defined(OUTLIER_FILTER_NODES_BUILDING_DLL) || defined(OUTLIER_FILTER_NODES_EXPORTS)
#elif defined(__linux__)
  #define OUTLIER_FILTER_NODES_PUBLIC __attribute__((visibility("default")))
  #define OUTLIER_FILTER_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define OUTLIER_FILTER_NODES_PUBLIC __attribute__((visibility("default")))
  #define OUTLIER_FILTER_NODES_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // OUTLIER_FILTER_NODES__VISIBILITY_CONTROL_HPP_
