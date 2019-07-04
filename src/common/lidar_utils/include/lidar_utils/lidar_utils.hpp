/// \copyright Copyright 2019 Silexica GmbH, Lichtstr. 25, Cologne, Germany
/// All rights reserved.
/// \file
/// \brief This file provides LIDAR utility functions and types.
#ifndef LIDAR_UTILS__LIDAR_UTILS_HPP_
#define LIDAR_UTILS__LIDAR_UTILS_HPP_

#include "lidar_utils/lidar_types.hpp"
#include <cmath>

namespace autoware
{
namespace common
{
namespace lidar_utils
{

///
/// Approximation was taken from:
/// http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
///
/// |Error = fast_atan2(y, x) - atan2f(y, x)| < 0.00468 rad
///
/// Octants:
///         pi/2
///       ` 3 | 2 /
///        `  |  /
///       4 ` | / 1
///   pi -----+----- 0
///       5 / | ` 8
///        /  |  `
///       / 6 | 7 `
///         3pi/2
///
///
float fast_atan2(float y, float x)
{
  constexpr float scaling_constant = 0.28086f;

  if (x == 0.0f) {
    // Special case atan2(0.0, 0.0) = 0.0
    if (y == 0.0f) {
      return 0.0f;
    }

    // x is zero so we are either at pi/2 for (y > 0) or -pi/2 for (y < 0)
    return ::std::copysign(autoware::common::lidar_utils::PI_2, y);
  }

  // Calculate quotient of y and x
  float div = y / x;

  // Determine in which octants we can be, if |y| is smaller than |x| (|div|<1)
  // then we are either in 1,4,5 or 8 else we are in 2,3,6 or 7.
  if (fabsf(div) < 1.0f) {
    // We are in 1,4,5 or 8

    float atan = div / (1.0f + scaling_constant * div * div);

    // If we are in 4 or 5 we need to add pi or -pi respectively
    if (x < 0.0f) {
      return ::std::copysign(autoware::common::lidar_utils::PI, y) + atan;
    }
    return atan;
  }

  // We are in 2,3,6 or 7
  return ::std::copysign(autoware::common::lidar_utils::PI_2, y) -
         div / (div * div + scaling_constant);
}

}  // namespace lidar_utils
}  // namespace common
}  // namespace autoware

#endif  // LIDAR_UTILS__LIDAR_UTILS_HPP_
