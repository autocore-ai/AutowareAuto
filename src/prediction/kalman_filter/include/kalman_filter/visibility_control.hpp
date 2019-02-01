// Copyright 2017-2018 Apex.AI, Inc.
// All rights reserved.

#ifndef KALMAN_FILTER__VISIBILITY_CONTROL_HPP_
#define KALMAN_FILTER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(KALMAN_FILTER_BUILDING_DLL) || defined(KALMAN_FILTER_EXPORTS)
    #define KALMAN_FILTER_PUBLIC __declspec(dllexport)
    #define KALMAN_FILTER_LOCAL
  #else  // defined(KALMAN_FILTER_BUILDING_DLL) || defined(KALMAN_FILTER_EXPORTS)
    #define KALMAN_FILTER_PUBLIC __declspec(dllimport)
    #define KALMAN_FILTER_LOCAL
  #endif  // defined(KALMAN_FILTER_BUILDING_DLL) || defined(KALMAN_FILTER_EXPORTS)
#elif defined(__linux__)
  #define KALMAN_FILTER_PUBLIC __attribute__((visibility("default")))
  #define KALMAN_FILTER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define KALMAN_FILTER_PUBLIC __attribute__((visibility("default")))
  #define KALMAN_FILTER_LOCAL __attribute__((visibility("hidden")))
#else  // defined(__linux__)
  #error "Unsupported Build Configuration"
#endif  // defined(__WIN32)

#endif  // KALMAN_FILTER__VISIBILITY_CONTROL_HPP_
