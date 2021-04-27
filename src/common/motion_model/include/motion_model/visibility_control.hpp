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
// Developed by Apex.AI, Inc.

#ifndef MOTION_MODEL__VISIBILITY_CONTROL_HPP_
#define MOTION_MODEL__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(MOTION_MODEL_BUILDING_DLL) || defined(MOTION_MODEL_EXPORTS)
    #define MOTION_MODEL_PUBLIC __declspec(dllexport)
    #define MOTION_MODEL_LOCAL
  #else  // defined(MOTION_MODEL_BUILDING_DLL) || defined(MOTION_MODEL_EXPORTS)
    #define MOTION_MODEL_PUBLIC __declspec(dllimport)
    #define MOTION_MODEL_LOCAL
  #endif  // defined(MOTION_MODEL_BUILDING_DLL) || defined(MOTION_MODEL_EXPORTS)
#elif defined(__linux__)
  #define MOTION_MODEL_PUBLIC __attribute__((visibility("default")))
  #define MOTION_MODEL_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define MOTION_MODEL_PUBLIC __attribute__((visibility("default")))
  #define MOTION_MODEL_LOCAL __attribute__((visibility("hidden")))
#else  // defined(__linux__)
  #error "Unsupported Build Configuration"
#endif  // defined(__WIN32)

#endif  // MOTION_MODEL__VISIBILITY_CONTROL_HPP_
