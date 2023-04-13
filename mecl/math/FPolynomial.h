/// @file FPolynomial.h
///
/// @brief Contains functionality for polynomials
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

#ifndef MATH_FPOLYNOMIAL_H_
#define MATH_FPOLYNOMIAL_H_

namespace mecl
{
namespace math
{

// --------------------------------------------------------------------------
/// @class FPolynomial
/// 
/// @brief Functor class for polynomials
// --------------------------------------------------------------------------

template<typename T, uint32_t order>
class FPolynomial
{
public:

  /// evaluate polynomial
  static T eval_x(const T i_Coeff_ax[order+1], const T& i_Val_rx);

};

} // namespace math
} // namespace mecl

#include "FPolynomial.hpp"

#endif /* MATH_FPOLYNOM_H_ */

/// @}
/// @}
