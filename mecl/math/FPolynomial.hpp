/// @file FPolynom.hppp
///
/// @brief Contains functionality for polynoms
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
/// @date created 08/31/15
///
// --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup math
/// @{

#ifndef _MECL_MATH_FPOLYNOMIAL_HPP_
#define _MECL_MATH_FPOLYNOMIAL_HPP_

#include "FPolynomial.h"

namespace mecl {
namespace math {

// --------------------------------------------------------------------------
/// @brief Evaluate polynomial 
///
/// Function evaluates polynomial given by parameters \p i_Coeff_ax at
/// position \p i_Val_rx. The entries  of the input array i_Coeff_ax[i] are 
/// the i-th order coefficients of the polynomial function. 
///
/// @remark Function implements Horner scheme 
///
/// @param[in]    i_Coeff_ax[order]  Coefficients of the polynom
/// @param[in]    i_Val_rx           Position to be evalutated
///
/// @return       Value of the given polynomial at input position
// --------------------------------------------------------------------------

template<typename T, uint32_t order>
T FPolynomial<T, order>::eval_x(const T i_Coeff_ax[order+1], const T& i_Val_rx)
{
  const T* v_Coeff_px = &i_Coeff_ax[order];
  const T* const c_StartCoeff_px = &i_Coeff_ax[0];
  T v_Val_x(*v_Coeff_px);

  do {
    v_Val_x *= i_Val_rx;
    v_Val_x += *(--v_Coeff_px);
  } while (v_Coeff_px != c_StartCoeff_px);

  return v_Val_x;
}


} // namespace math
} // namespace mecl

#endif // _MECL_MATH_FPOLYNOMIAL_HPP_

/// @}
/// @}

