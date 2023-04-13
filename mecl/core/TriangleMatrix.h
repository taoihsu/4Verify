//  --------------------------------------------------------------------------
/// @file TriangleMatrix.h
/// @brief Triangular matrix storage class
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
/// @date 18.03.2015
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{


#ifndef SRC_CORE_TRIANGLEMATRIX_H_
#define SRC_CORE_TRIANGLEMATRIX_H_

//! @cond 
// PRQA S 1020 1 // macro is used by MKS
#define TriangleMatrix_D_VERSION_ID "$Id: TriangleMatrix.h 1.10 2017/01/26 11:29:53EST Zollner, Helmut (SAI_HeZol) draft  $"
//! @endcond

#include "Matrix.h"

namespace mecl {
namespace core {

// --------------------------------------------------------------------------
/// @brief Upper triangle matrix indexing function
///
/// Function implements indexing scheme for upper triangular matrices. 
///
/// @param[in] i  Row index
/// @param[in] j  Column index
/// @param[in] doAssert  Defines whether or not the function produces an assertion
/// when indexing outside of matrix boundaries is attempted.
/// 
/// @return Index in value array
// --------------------------------------------------------------------------
template <uint32_t n>
static uint32_t calcIndexUpperTriangle_u32(uint32_t i, uint32_t j, bool_t doAssert)
{
  static uint32_t s_Channels_u32 = n*(n+1) / 2 + 1;
  AssertFunction(not(doAssert && (j < i)),"Tried to assign to invalid index.");
  return j < i ? s_Channels_u32 : i * n - (i * (i - 1) ) / 2 + j - i;
}

/// @cond 
// --------------------------------------------------------------------------
// Empty LowerTriangleMatrix class definition to allow compilation of 
// UpperTriangleMatrix without knowing the complete interface.
// --------------------------------------------------------------------------
template <typename T, uint32_t n>
class LowerTriangleMatrix;
/// @endcond

// --------------------------------------------------------------------------
/// @class UpperTriangleMatrix
/// @brief Upper triangle matrix data type
///
/// The upper triangle matrix data type is a specialization of the standard Magna 
/// Electronics Library Matrix class. The triangle matrix classes are utilized in
/// SVD decomposition among others.
// --------------------------------------------------------------------------
template <typename T, uint32_t n>
class UpperTriangleMatrix : public Matrix<T, n, n, e_UpperTriangle>
{

public:
  //! Enumeration type for matrix properties
  enum {
    rows     = n,                       ///< Number of rows
    cols     = n,                       ///< Number of columns
    channels = rows*(cols+1) / 2 + 1,   ///< Number of elements + zero element
    shortdim = n                        ///< Shortest dimension
  };

  typedef typename Matrix<T, n, n>::CalcIndexSig_fp CalcIndexSig_fp;

  // *******************************
  // * construtors and initialization

  // --------------------------------------------------------------------------
  /// @brief Default constructor
  ///
  /// The default construtor initializes all cell elements to zero.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp UpperTriangleMatrix_Constructor1
  ///
  /// @return       UpperTriangleMatrix
  // --------------------------------------------------------------------------
  UpperTriangleMatrix()
  {
    (*this)(channels) = math::constants<T>::zero_x();
  }

  // --------------------------------------------------------------------------
  /// @brief Constructor with initialization value
  ///
  /// The constructor initializes all cell elements to the input argument \p value.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp UpperTriangleMatrix_Constructor2
  ///
  /// @param[in] value  Initialization value for all elements in UpperTriangleMatrix
  /// @return       UpperTriangleMatrix pre-initialized with specific values
  // --------------------------------------------------------------------------
  explicit UpperTriangleMatrix(T value)
  {
    // set zero element
    (*this)(channels) = math::constants<T>::zero_x();
  }

  // --------------------------------------------------------------------------
  /// @brief Virtual destructor
  // --------------------------------------------------------------------------
  virtual ~UpperTriangleMatrix() {};

  // *******************************
  // * methods

  // --------------------------------------------------------------------------
  /// @brief Copy into lower triangular matrix
  ///
  /// The function copies all elements into a lower triangular matrix
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp UpperTriangleMatrix_Constructor2
  /// @snippet CoreTester.cpp UpperTriangleMatrix_toLower
  ///
  /// @return       LowerTriangleMatrix with elements copied from this UpperTriangleMatrix
  // --------------------------------------------------------------------------
  LowerTriangleMatrix<T, n> toLower() const
  {
    LowerTriangleMatrix<T, n> M;
    for(uint32_t i=0; i < n; i++) {
      for(uint32_t j=i; j < n; j++) {
		M(j,i) = (*this)(i,j);
      }
    }
    return M;
  }

};

// --------------------------------------------------------------------------
/// @brief Lower triangle matrix indexing function
///
/// Function implements indexing scheme for lower triangular matrices. 
///
/// @param[in] i  Row index
/// @param[in] j  Column index
/// @param[in] doAssert  Defines whether or not the function produces an assertion
/// when indexing outside of matrix boundaries is attempted.
/// 
/// @return Index in value array
// --------------------------------------------------------------------------
template <uint32_t n>
static uint32_t calcIndexLowerTriangle_u32(uint32_t i, uint32_t j, bool_t doAssert)
{
  static uint32_t s_Channels_u32 = n*(n+1) / 2 + 1;
  AssertFunction(not(doAssert && (i < j)), "Tried to assign to invalid index.");
  return i < j ? s_Channels_u32 : j * n - (j * (j - 1) ) / 2 + i - j;
}

// --------------------------------------------------------------------------
/// @class LowerTriangleMatrix
/// @brief Lower triangle matrix data type
///
/// The lower triangle matrix data type is a specialization of the standard Magna 
/// Electronics Library Matrix class. The triangle matrix classes are utilized in
/// SVD decomposition among others.
// --------------------------------------------------------------------------
template <typename T, uint32_t n>
class LowerTriangleMatrix : public Matrix<T, n, n, e_LowerTriangle>
{

public:
  //! Enumeration type for matrix properties
  enum {
    rows     = n,                       ///< Number of rows
    cols     = n,                       ///< Number of columns
    channels = rows*(cols+1) / 2 + 1,   ///< Number of elements + zero element
    shortdim = n                        ///< Shortest dimension
  };

  // *******************************
  // * construtors and initialization

  // --------------------------------------------------------------------------
  /// @brief Default constructor
  ///
  /// The default construtor initializes all cell elements to zero.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp LowerTriangleMatrix_Constructor1
  ///
  /// @return       LowerTriangleMatrix
  // --------------------------------------------------------------------------
  LowerTriangleMatrix()
  {
    // set zero element
    (*this)(channels) = math::constants<T>::zero_x();
  }

  // --------------------------------------------------------------------------
  /// @brief Constructor with initialization value
  ///
  /// The constructor initializes all cell elements to the input argument \p value.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp LowerTriangleMatrix_Constructor2
  ///
  /// @param[in] value  Initialization value for all elements in LowerTriangleMatrix
  /// @return       LowerTriangleMatrix pre-initialized with specific values
  // --------------------------------------------------------------------------
  explicit LowerTriangleMatrix(T value)
  {
    // set zero element
    (*this)(channels) = math::constants<T>::zero_x();
  }

  // --------------------------------------------------------------------------
  /// @brief Virtual destructor
  // --------------------------------------------------------------------------
  virtual ~LowerTriangleMatrix() {};

  // *******************************
  // * methods

  // --------------------------------------------------------------------------
  /// @brief Copy into upper triangular matrix
  ///
  /// The function copies all elements into a upper triangular matrix
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp LowerTriangleMatrix_Constructor2
  /// @snippet CoreTester.cpp LowerTriangleMatrix_toLower
  ///
  /// @return       UpperTriangleMatrix with elements copied from this LowerTriangleMatrix
  // --------------------------------------------------------------------------
  UpperTriangleMatrix<T, n> toUpper() const
  {
    UpperTriangleMatrix<T, n> M;
    for(uint32_t i=0; i < n; i++) {
      for(uint32_t j=0; j <= i; j++) {
        M(j,i) = (*this)(i,j);
      }
    }
    return M;
  }

};



} // core
} // mecl

#endif /* SRC_CORE_TRIANGLEMATRIX_H_ */
/// @}
/// @}
