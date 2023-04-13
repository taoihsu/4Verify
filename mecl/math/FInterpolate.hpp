// --------------------------------------------------------------------------
/// @file FInterpolate.hpp
///
/// @brief Contains the functionality to perform coordinate interpolation
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

#ifndef _MECL_MATH_FINTERPOLATE_HPP_
#define _MECL_MATH_FINTERPOLATE_HPP_

#include "FInterpolate.h"

namespace mecl {
namespace math {

// --------------------------------------------------------------------------
/// @brief Unsafe bilinear point interpolation
///
/// Function interpolates 4 nearest neighbors using bilinear interpolation.
/// The function is inherent unsafe since the behavior is undefined for
/// points outside of the image plane.
///
/// @par Example use:
/// @snippet MathTester.cpp FInterpolateBilinear_calcInterpolatedPointsUnsafe_v
///
/// @param[in]    i_Pos_rx  Fractional 2D point in image plane
/// @param[out]   o_Pos_rx  4 nearest neighboring points
/// @param[out]   o_W_rx    Weights for nearest neighboring points
///
/// @return       Nothing
// --------------------------------------------------------------------------
template<typename T>
void FInterpolate<T>::Bilinear::calcInterpolatedPointsUnsafe_v(const mecl::core::Point2D<T> &i_Pos_rx, 
                                                               mecl::core::Matrix<T, 2, 4> &o_Pos_rx,
                                                               mecl::core::Vector<T, 4> &o_W_rx)
{
  // Calculate pixel weights
  T v_Px_f = mecl::math::numeric<T>::floor_x(i_Pos_rx(0));
  T v_Py_f = mecl::math::numeric<T>::floor_x(i_Pos_rx(1));
  T v_Fx_f = i_Pos_rx(0) - v_Px_f;
  T v_Fy_f = i_Pos_rx(1) - v_Py_f;
  T v_Fx1_f = mecl::math::constants<T>::one_x() - v_Fx_f;
  T v_Fy1_f = mecl::math::constants<T>::one_x() - v_Fy_f;

  // Assign weight vector.
  mecl::core::Vector<T, 4>::Config_s  v_WCfg_s = {v_Fx1_f * v_Fy1_f, v_Fx_f * v_Fy1_f, v_Fx1_f * v_Fy_f, v_Fx_f * v_Fy_f};
  o_W_rx.updateConfig_v(v_WCfg_s);

  // Assign point coordinates.
  mecl::core::Matrix<T, 2, 4>::Config_s v_CoordsCfg_s = { v_Px_f, v_Px_f + 1, v_Px_f, v_Px_f + 1, v_Py_f, v_Py_f, v_Py_f + 1, v_Py_f + 1 };
  o_Pos_rx.updateConfig_v(v_CoordsCfg_s);
};


} /* namespace math */
} /* namespace mecl */

#endif /* _MECL_MATH_FINTERPOLATE_HPP_ */

/// @}
/// @}

