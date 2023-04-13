//  --------------------------------------------------------------------------
/// @file Matrix3x3.h
/// @brief Generic matrix storage class (static memory)
///
/// Here may follow a longer description for the template module with examples.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef MECL_CORE_MATRIX3X3_H_
#define MECL_CORE_MATRIX3X3_H_

//! @cond
// PRQA S 1020 1 // macro is used by MKS
#define Matrix3x3_D_VERSION_ID "$Id: Matrix3x3.h 1.2 2019/08/08 15:28:36CEST Michael Becker (MEE_MBECKE) Draft  $"
//! @endcond

// PRQA S 467 EOF // ignore QACPP maintenance waring
// PRQA S 3362    // ignore pre-decrement used as sub-expression warning
// PRQA S 3360    // ignoew pre-increment used as sub-expression warngin

#include "MeclAssert.h"
#include "Matrix.h"
#include "mecl/math/Math.h"



namespace mecl
{
namespace core
{

// --------------------------------------------------------------------------
/// @class Matrix
/// @brief 3x3 matrix data type of Magne Electronics Library
///
/// The 3x3 matrix data type is a one of the primary data types of the Magna 
/// Electronics Library. The template based implementation allows working
/// with 3x3 matrix related data with different base types. The object includes 
/// basic operations like determinant and inverese.
// --------------------------------------------------------------------------
template <typename T>
class Matrix3x3 : public core::Matrix<T, 3, 3>
{
public:

  typedef typename Matrix<T, 3, 3>::Config_s Config_s;  ///< Configuration data set

  // --------------------------------------------------------------------------
  /// @brief Default constructor
  /// The default construtor initializes all cell elements to zero.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix3x3_Constructor1
  ///
  /// @return       Matrix3x3
  // --------------------------------------------------------------------------
  Matrix3x3()
  : Matrix<T, 3, 3>()
  {}

  // --------------------------------------------------------------------------
  /// @brief Constructor with initialization value
  ///
  /// The constructor initializes all cell elements to the input argument \p value.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix3x3_Constructor2
  ///
  /// @param[in] value  Initialization value for all elements in Matrix
  /// @return       Matrix3x3 pre-initialized with specific values
  // --------------------------------------------------------------------------
  Matrix3x3(T i_Value_x)
  : Matrix<T, 3, 3>(i_Value_x)
  {}

  // --------------------------------------------------------------------------
  /// @brief Copy constructor
  ///
  /// The constructor copies the contents of \p i_Matrix_rs into this matrix.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix3x3_Constructor3
  /// @snippet CoreTester.cpp Matrix3x3_Constructor4
  ///
  /// @param[in]    i_Matrix_rx Matrix object from which to copy contents
  /// @return       Matrix pre-initialized with specific content
  // --------------------------------------------------------------------------
  Matrix3x3(const Matrix<T, 3, 3>& i_Matrix_rx)
  : Matrix<T, 3, 3>(i_Matrix_rx)
  {}

  // --------------------------------------------------------------------------
  /// @brief Copy constructor with initialization by configuration
  ///
  /// The constructor copies the contents of \p i_Config_rs into cell elements.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix3x3_Constructor3
  ///
  /// @param[in]    i_Config_rs Configuration to be copied into cell elements in Matrix
  /// @return       Matrix pre-initialized with specific configuration
  // --------------------------------------------------------------------------
  Matrix3x3(const Config_s& i_Config_rs)
  : Matrix<T, 3, 3>(i_Config_rs)
  {}

  // --------------------------------------------------------------------------
  /// @brief Compute determinant of 3x3 matrix
  ///
  /// The function computes the determinant of this matrix
  ///
  /// Exact computation sequence (formula) is:
  ///
  // PRQA S 1051 8 // following lines are doxygen documentation
  /// det(M) = m6 * ( m1 * m5 - m2 * m4) +
  ///               ( m2 * m7 - m1 * m8) * m3 +
  ///               ( m4 * m8 - m5 * m7) * m0
  ///
  /// where
  ///
  ///     m0 m1 m2
  /// M = m3 m4 m5
  ///     m6 m7 m8
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix3x3_Constructor3
  /// @snippet CoreTester.cpp Matrix3x3_det
  ///
  /// @return       Determinant
  // --------------------------------------------------------------------------
 
  // PRQA S 3360 ++ // PRQA S 3362 ++ // to improve readability, we allow post- increment and decrement as sub-expressions
  T det(void) const
  {
    const T* c_Val1_px = & ( (*this) (6) );
    const T* c_Val2_px = & ( (*this) (1) );
    const T* c_Val3_px = c_Val1_px;

    T v_Determinate_x = *c_Val1_px;
    T v_Sum_x = *c_Val2_px * *(--c_Val1_px);
    v_Sum_x -= *(++c_Val2_px) * *(--c_Val1_px);
    v_Determinate_x *= v_Sum_x;
    v_Sum_x  = *c_Val2_px * *(++c_Val3_px);
    v_Sum_x -= *(--c_Val2_px) * *(++c_Val3_px);
    v_Determinate_x += v_Sum_x * *(--c_Val1_px);
    v_Sum_x  = *(++c_Val1_px) * *c_Val3_px;
    v_Sum_x -= *(++c_Val1_px) * *(--c_Val3_px);
    v_Determinate_x += v_Sum_x * *(--c_Val2_px);

    return v_Determinate_x;
  }
  // PRQA S 3360 -- // PRQA S 3362 --

  // --------------------------------------------------------------------------
  /// @brief Determine matrix regularity
  ///
  /// The function determines if the determinant is larger than the smallest allowable 
  /// positive number.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix3x3_Constructor3
  /// @snippet CoreTester.cpp Matrix3x3_isRegular_b_1
  ///
  /// @return       Boolean, True = regular, False = not regular
  // --------------------------------------------------------------------------
  bool_t isRegular_b(void) const
  {
    return math::numeric_limits<T>::epsilon_x() < math::abs_x(this->det());
  }

  // --------------------------------------------------------------------------
  /// @brief Determine matrix regularity
  ///
  /// The function determines if the determinant is larger than the smallest allowable 
  /// positive number and additionally passes back the determinant.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix3x3_Constructor3
  /// @snippet CoreTester.cpp Matrix3x3_isRegular_b_2
  ///
  /// @param[in]    o_Determinate_x   Determinant
  /// @return       Boolean, True = regular, False = not regular
  // --------------------------------------------------------------------------
  bool_t isRegular_b(T& o_Determinate_rx) const
  {
    o_Determinate_rx = this->det();

    return math::numeric_limits<T>::epsilon_x() < math::abs_x(o_Determinate_rx);
  }

  // --------------------------------------------------------------------------
  /// @brief Inverse matrix
  ///
  /// The function computes the inverse matrix if determinant > 0 and otherwise returns zeros();
  /// The coding is an optimized computation sequence for the following (original) implementation
  ///
  // PRQA S 1051 9 // following lines are doxygen documentation
  ///    v_InverseMatrix_o(0,0) = (*this)(1,1) * (*this)(2,2) - (*this)(1,2) * (*this)(2,1);
  ///    v_InverseMatrix_o(0,1) = (*this)(0,2) * (*this)(2,1) - (*this)(0,1) * (*this)(2,2);
  ///    v_InverseMatrix_o(0,2) = (*this)(0,1) * (*this)(1,2) - (*this)(0,2) * (*this)(1,1);
  ///    v_InverseMatrix_o(1,0) = (*this)(1,2) * (*this)(2,0) - (*this)(1,0) * (*this)(2,2);
  ///    v_InverseMatrix_o(1,1) = (*this)(0,0) * (*this)(2,2) - (*this)(0,2) * (*this)(2,0);
  ///    v_InverseMatrix_o(1,2) = (*this)(0,2) * (*this)(1,0) - (*this)(0,0) * (*this)(1,2);
  ///    v_InverseMatrix_o(2,0) = (*this)(1,0) * (*this)(2,1) - (*this)(1,1) * (*this)(2,0);
  ///    v_InverseMatrix_o(2,1) = (*this)(0,1) * (*this)(2,0) - (*this)(0,0) * (*this)(2,1);
  ///    v_InverseMatrix_o(2,2) = (*this)(0,0) * (*this)(1,1) - (*this)(0,1) * (*this)(1,0);
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Matrix3x3_Constructor1
  /// @snippet CoreTester.cpp Matrix3x3_Constructor3
  /// @snippet CoreTester.cpp Matrix3x3_inverse
  ///
  /// @return       Inverse matrix if determinant > 0, otherwise zeros() matrix
  // --------------------------------------------------------------------------

  // PRQA S 3360 ++ // PRQA S 3362 ++ // to improve readability, we allow post- increment and decrement as sub-expressions
  Matrix3x3<T> inverse(void) const
  {
    Matrix3x3<T> v_InverseMatrix_x(math::constants<T>::zero_x());

    T v_Determinate_x(math::constants<T>::zero_x());

    // do we try to compute inverse of a singular matrix?
    AssertFunction(true == this->isRegular_b(v_Determinate_x),"Tried to compute inverse of singular matrix.");

    T* v_InvMat_px = &(v_InverseMatrix_x(0));

    const T* c_Val1_px = &( (*this)(4) );
    const T* c_Val2_px = &( (*this)(8) );
    const T* c_Val3_px = &( (*this)(2) );
    const T* c_Val4_px = &( (*this)(0) );

    *v_InvMat_px      = *c_Val1_px * *c_Val2_px;             // 0 (0,0)
    *v_InvMat_px     -= *(++c_Val1_px) * *(--c_Val2_px);     // 5,7,2,0

    *(++v_InvMat_px)  = *c_Val3_px  * *c_Val2_px;            // 1 (0,1)
    *v_InvMat_px     -= *(--c_Val3_px) * *(++c_Val2_px);     // 5,8,1,0

    *(++v_InvMat_px)  = *c_Val3_px  * *c_Val1_px;            // 2 (0,2)
    *v_InvMat_px     -= *(++c_Val3_px) * *(--c_Val1_px);     // 4,8,2,0

    *(++v_InvMat_px)  = *(++c_Val1_px);                      // 3 (1,0)
    *v_InvMat_px     *= *(++c_Val1_px);
    *v_InvMat_px     -= *(++c_Val3_px) * *c_Val2_px;         // 6,8,3,0

    *(++v_InvMat_px) = *c_Val4_px * *c_Val2_px;              // 4 (1,1)
    *v_InvMat_px    -= *(--c_Val3_px) * *c_Val1_px;          // 6,8,2,0

    *(++v_InvMat_px) = *c_Val3_px;                           // 5 (1,2)
    *v_InvMat_px    *= *(++c_Val3_px);
    *v_InvMat_px    -= *c_Val4_px * *(--c_Val1_px);          // 5,8,3,0

    *(++v_InvMat_px) = *c_Val3_px * *(--c_Val2_px);          // 6 (2,0)
    *v_InvMat_px    -= *(++c_Val3_px) * *(--c_Val2_px);      // 5,6,4,0

    *(++v_InvMat_px) = *(++c_Val4_px) * *c_Val2_px;          // 7 (2,1)
    *v_InvMat_px    -= *(--c_Val4_px) * *(++c_Val2_px);      // 5,7,4,0

    *(++v_InvMat_px) = *c_Val4_px     * *c_Val3_px;          // 8 (2,2)
    *v_InvMat_px    -= *(++c_Val4_px) * *(--c_Val3_px);      // 5,7,3,1

    return v_InverseMatrix_x.div(v_Determinate_x);

  }
  // PRQA S 3360 -- // PRQA S 3362 --
}; // end of template specialization

} // namespace core
} // namespace mecl


#endif // MECL_CORE_MATRIX3X3_H_
/// @}
/// @}
