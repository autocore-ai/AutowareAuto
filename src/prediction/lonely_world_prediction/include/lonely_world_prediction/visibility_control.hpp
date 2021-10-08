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

#ifndef LONELY_WORLD_PREDICTION__VISIBILITY_CONTROL_HPP_
#define LONELY_WORLD_PREDICTION__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(LONELY_WORLD_PREDICTION_BUILDING_DLL) || defined(LONELY_WORLD_PREDICTION_EXPORTS)
    #define LONELY_WORLD_PREDICTION_PUBLIC __declspec(dllexport)
    #define LONELY_WORLD_PREDICTION_LOCAL
  #else  // defined(LONELY_WORLD_PREDICTION_BUILDING_DLL) ||
         // defined(LONELY_WORLD_PREDICTION_EXPORTS)
    #define LONELY_WORLD_PREDICTION_PUBLIC __declspec(dllimport)
    #define LONELY_WORLD_PREDICTION_LOCAL
  #endif  // defined(LONELY_WORLD_PREDICTION_BUILDING_DLL) ||
          // defined(LONELY_WORLD_PREDICTION_EXPORTS)
#elif defined(__linux__)
  #define LONELY_WORLD_PREDICTION_PUBLIC __attribute__((visibility("default")))
  #define LONELY_WORLD_PREDICTION_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define LONELY_WORLD_PREDICTION_PUBLIC __attribute__((visibility("default")))
  #define LONELY_WORLD_PREDICTION_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // LONELY_WORLD_PREDICTION__VISIBILITY_CONTROL_HPP_
