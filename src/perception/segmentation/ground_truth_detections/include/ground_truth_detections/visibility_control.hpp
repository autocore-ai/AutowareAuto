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

#ifndef GROUND_TRUTH_DETECTIONS__VISIBILITY_CONTROL_HPP_
#define GROUND_TRUTH_DETECTIONS__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(GROUND_TRUTH_DETECTIONS_BUILDING_DLL) || defined(GROUND_TRUTH_DETECTIONS_EXPORTS)
    #define GROUND_TRUTH_DETECTIONS_PUBLIC __declspec(dllexport)
    #define GROUND_TRUTH_DETECTIONS_LOCAL
  #else
    #define GROUND_TRUTH_DETECTIONS_PUBLIC __declspec(dllimport)
    #define GROUND_TRUTH_DETECTIONS_LOCAL
  #endif
#elif defined(__linux__)
  #define GROUND_TRUTH_DETECTIONS_PUBLIC __attribute__((visibility("default")))
  #define GROUND_TRUTH_DETECTIONS_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define GROUND_TRUTH_DETECTIONS_PUBLIC __attribute__((visibility("default")))
  #define GROUND_TRUTH_DETECTIONS_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // GROUND_TRUTH_DETECTIONS__VISIBILITY_CONTROL_HPP_
