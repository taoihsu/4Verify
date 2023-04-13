// --------------------------------------------------------------------------
/// @file FInterpolate.h
///
/// @brief Contains the coordinate interpolation definitions
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Allan Reinhold Kildeby (allan.kildeby@magna.com)
///
// --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup math
/// @{

#ifndef _MECL_MATH_FINTERPOLATE_H_
#define _MECL_MATH_FINTERPOLATE_H_

#include "mecl/core/Point.h"

namespace mecl
{
namespace math
{

// --------------------------------------------------------------------------
/// @class FInterpolate
/// 
/// @brief Functor class for interpolation schemes
// --------------------------------------------------------------------------
template<typename T>
class FInterpolate
{
public:
  // --------------------------------------------------------------------------
  /// @struct Bilinear
  /// 
  /// @brief Function group for functions related to bilinear interpolation
  // --------------------------------------------------------------------------
  struct Bilinear
  {
    // --------------------------------------------------------------------------
    /// @brief Unsafe bilinear point interpolation
    // --------------------------------------------------------------------------
    static void calcInterpolatedPointsUnsafe_v(const mecl::core::Point2D<T> &i_Pos_rx, 
                                               mecl::core::Matrix<T, 2, 4> &o_Pos_rx,
                                               mecl::core::Vector<T, 4> &o_W_rx);
  };
};

} /* namespace math */
} /* namespace mecl */

#include "FInterpolate.hpp"

#endif /* _MECL_MATH_FINTERPOLATE_H_ */

/// @}
/// @}
