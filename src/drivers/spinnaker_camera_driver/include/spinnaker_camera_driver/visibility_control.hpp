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

#ifndef SPINNAKER_CAMERA__VISIBILITY_CONTROL_HPP_
#define SPINNAKER_CAMERA__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(SPINNAKER_CAMERA_BUILDING_DLL) || defined(SPINNAKER_CAMERA_EXPORTS)
    #define SPINNAKER_CAMERA_PUBLIC __declspec(dllexport)
    #define SPINNAKER_CAMERA_LOCAL
  #else  // defined(SPINNAKER_CAMERA_BUILDING_DLL) || defined(SPINNAKER_CAMERA_EXPORTS)
    #define SPINNAKER_CAMERA_PUBLIC __declspec(dllimport)
    #define SPINNAKER_CAMERA_LOCAL
  #endif  // defined(SPINNAKER_CAMERA_BUILDING_DLL) || defined(SPINNAKER_CAMERA_EXPORTS)
#elif defined(__linux__)
  #define SPINNAKER_CAMERA_PUBLIC __attribute__((visibility("default")))
  #define SPINNAKER_CAMERA_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define SPINNAKER_CAMERA_PUBLIC __attribute__((visibility("default")))
  #define SPINNAKER_CAMERA_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // SPINNAKER_CAMERA__VISIBILITY_CONTROL_HPP_
