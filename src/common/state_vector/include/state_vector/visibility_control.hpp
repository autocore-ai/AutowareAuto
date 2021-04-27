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
// All rights reserved.

#ifndef STATE_VECTOR__VISIBILITY_CONTROL_HPP_
#define STATE_VECTOR__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(STATE_VECTOR_BUILDING_DLL) || defined(STATE_VECTOR_EXPORTS)
    #define STATE_VECTOR_PUBLIC __declspec(dllexport)
    #define STATE_VECTOR_LOCAL
  #else  // defined(STATE_VECTOR_BUILDING_DLL) || defined(STATE_VECTOR_EXPORTS)
    #define STATE_VECTOR_PUBLIC __declspec(dllimport)
    #define STATE_VECTOR_LOCAL
  #endif  // defined(STATE_VECTOR_BUILDING_DLL) || defined(STATE_VECTOR_EXPORTS)
#elif defined(__linux__)
  #define STATE_VECTOR_PUBLIC __attribute__((visibility("default")))
  #define STATE_VECTOR_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define STATE_VECTOR_PUBLIC __attribute__((visibility("default")))
  #define STATE_VECTOR_LOCAL __attribute__((visibility("hidden")))
#else  // defined(__linux__)
  #error "Unsupported Build Configuration"
#endif  // defined(__WIN32)

#endif  // STATE_VECTOR__VISIBILITY_CONTROL_HPP_
