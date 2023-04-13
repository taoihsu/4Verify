/*
 * Matrix.hpp
 *
 *  Created on: 06.03.2015
 *      Author: sai_hezol
 */

#ifndef SRC_CORE_MATRIX_HPP_
#define SRC_CORE_MATRIX_HPP_

#include "Matrix.h"

//! @cond
// PRQA S 284 EOF // switch off QACPP "is not a namespace or class", since this is not correctly resolved here by lint checker
// PRQA S 5127 EOF // switch off The stream input/output libary <cstdio> shall not be used (MeclAssert.h)
// PRQA S 621 EOF // linter parsing problems ...
// PRQA S 3708 EOF   // floating point arithmic MUST be used here
// PRQA S 467 EOF // ignore QACPP maintenance waring
// PRQA S 3361 EOF // post-increment is allowed here as sub-expression

//! @endcond

#if defined(_MSC_VER)
#pragma warning(disable : 4351)
#endif


namespace mecl
{
namespace core
{


// --------------------------------------------------------------------------
/// @brief Default constructor
///
/// The default construtor initializes all cell elements to zero.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
///
/// @return       Matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T, m, n, Mtype>::Matrix()
: val()
{
  T* v_Val_px = this->val;
  T const * c_EndVal_px = &this->val[e_Channels];

  do {
    *(v_Val_px++) = math::constants<T>::zero_x();
  } while (v_Val_px != c_EndVal_px);
}


// --------------------------------------------------------------------------
/// @brief Constructor with initialization value
///
/// The constructor initializes all cell elements to the input argument \p value.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor2
///
/// @param[in] value  Initialization value for all elements in Matrix
/// @return       Matrix pre-initialized with specific values
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T, m, n, Mtype>::Matrix (T i_Value_x)
: val()
{
  T* v_Val_px = this->val;
  T const * c_EndVal_px = &this->val[e_Channels];

  do {
   *(v_Val_px++) = i_Value_x;
  } while (v_Val_px != c_EndVal_px);
}


// --------------------------------------------------------------------------
/// @brief Copy constructor with initialization by configuration
///
/// The constructor copies the contents of \p i_Config_rs into cell elements.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor4
///
/// @param[in] i_Config_rs Configuration to be copied into cell elements in Matrix
///
/// @return       Matrix pre-initialized with specific configuration
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T, m, n, Mtype>::Matrix(const Config_s& i_Config_rs)
: val()
{
  AssertFunction( ( static_cast<uint32_t>(e_Channels) * sizeof(T) ) == sizeof(i_Config_rs.cVal_ax),
                  "Input parameter size out of bounds.");

  T* v_Val_px = this->val;
  T const * c_EndVal_px = &this->val[e_Channels];

  const T* v_Config_px = i_Config_rs.cVal_ax;

  do
  {
    *(v_Val_px++) = *(v_Config_px++);
  } while (v_Val_px != c_EndVal_px);

  return;
}


// --------------------------------------------------------------------------
/// @brief Copy matrix contents to configuration struct
///
/// The function copies all cell elements from Matrix into \p o_Config_rs struct.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_copyConfig_v
///
/// @param[out] o_Config_rs Configuration to be overwritten with cell elements from Matrix
/// @return       Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::copyConfig_v(Config_s& o_Config_rs) const
{
  AssertFunction( ( static_cast<uint32_t>(e_Channels) * sizeof(T) ) == sizeof(o_Config_rs.cVal_ax),
                  "Output parameter size out of bounds.");

  const T* c_Val_px = this->val;
  T const * c_EndVal_px = &this->val[e_Channels];

  T* v_Config_px = o_Config_rs.cVal_ax;

  do
  {
    *(v_Config_px++) = *(c_Val_px++);
  } while (c_Val_px != c_EndVal_px);

  return;
}

// --------------------------------------------------------------------------
/// @brief Copy matrix contents to configuration struct of base type T1
///
/// The function copies all cell elements from Matrix into \p o_Config_rs struct.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_copyConfig_v_OtherBaseType
///
/// @param[out] o_Config_rs Configuration to be overwritten with cell elements from Matrix
/// @return       Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
template<typename T1> inline
void Matrix<T, m, n, Mtype>::copyConfig_v(typename Matrix<T1, m, n, Mtype>::Config_s& o_Config_rs) const
{
  this->convert<T1>().copyConfig_v(o_Config_rs);
  return;
}

// --------------------------------------------------------------------------
/// @brief Copy matrix contents to volatile configuration struct
///
/// The function copies all cell elements from Matrix into \p o_Config_rs struct.
/// Using volatile prevents optimization of struct access.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_copyConfigVolatile_v
///
/// @param[out] o_Config_rs Configuration to be overwritten with cell elements from Matrix
/// @return       Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::copyConfigVolatile_v(volatile Config_s& o_Config_rs) const
{
  AssertFunction( (static_cast<uint32_t>(e_Channels) * sizeof(T)) == sizeof(o_Config_rs.cVal_ax),
                "Output parameter size out of bounds.");

  const T* v_Val_px = this->val;
  T const * c_EndVal_px = &this->val[e_Channels];

  volatile T* v_Config_px = o_Config_rs.cVal_ax;

  do
  {
    *(v_Config_px++) = *(v_Val_px++);
  } while (v_Val_px != c_EndVal_px);

  return;
}

// --------------------------------------------------------------------------
/// @brief Copy matrix contents to volatile configuration struct of base type T1
///
/// The function copies all cell elements from Matrix into \p o_Config_rs struct.
/// Using volatile prevents optimization of struct access.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_copyConfigVolatile_v_OtherBaseType
///
/// @param[out] o_Config_rs Configuration to be overwritten with cell elements from Matrix
/// @return       Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
template<typename T1> inline
void Matrix<T, m, n, Mtype>::copyConfigVolatile_v(volatile typename Matrix<T1, m, n, Mtype>::Config_s& o_Config_rs) const
{
  this->convert<T1>().copyConfigVolatile_v(o_Config_rs);
  return;
}

// --------------------------------------------------------------------------
/// @brief Copy configuration struct contents to matrix
///
/// The function copies all elements from \p i_Config_rs struct to cell elements of Matrix.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor4
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_updateConfig_v
///
/// @param[in] i_Config_rs Configuration to be copied into cell elements of Matrix
/// @return       Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::updateConfig_v(const Config_s& i_Config_rs)
{
  AssertFunction( ( static_cast<uint32_t>(e_Channels) * sizeof(T) ) == sizeof(i_Config_rs.cVal_ax),
                    "Input parameter size out of bounds.");
  T* v_Val_px = this->val;
  T const * c_EndVal_px = &this->val[e_Channels];

  const T* v_Config_px = i_Config_rs.cVal_ax;

  do
  {
    *(v_Val_px++) = *(v_Config_px++);
  } while (v_Val_px != c_EndVal_px);

  return;
}

// --------------------------------------------------------------------------
/// @brief Copy configuration struct contents of base type T1 to matrix
///
/// The function copies all elements from \p i_Config_rs struct to cell elements of Matrix.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor4
/// @snippet CoreTester.cpp Matrix_updateConfig_v_OtherBaseType
///
/// @param[in] i_Config_rs Configuration of base type T1 to be copied into cell elements of Matrix
/// @return       Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
template<typename T1> inline
void Matrix<T, m, n, Mtype>::updateConfig_v(const typename Matrix<T1, m, n, Mtype>::Config_s& i_Config_rs)
{
  Matrix<T1, m , n> v_M_x(i_Config_rs);
  *this = v_M_x;
  return;
}

// --------------------------------------------------------------------------
/// @brief Constructor with initialization by array
///
/// The constructor copies the content of \p values into cell elements of Matrix
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor3
///
/// @param[in] values[]   Initialize value organized in array form of smaller or same size
/// @param[in] numValues  Number of values in array
/// @return       Matrix pre-initialized with specific values
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T, m, n, Mtype>::Matrix(const T values[], uint32_t numValues)
: val()
{
  AssertFunction(core::isNotNull_b(values),"Passed over null pointer to array parameter.");

  T* v_Val_px = this->val;
  T const * c_EndVal_px = &this->val[e_Channels];
  const T* v_Values_px = values;
  T const * c_EndValues_px = &values[numValues];

  if (numValues <= e_Channels)
  {
   do {
     *(v_Val_px++) = *(v_Values_px++);
   } while (v_Values_px != c_EndValues_px);

   while (v_Val_px != c_EndVal_px) {
     *(v_Val_px++) = math::constants<T>::zero_x();
   }
  }
  else
  {
   do {
     *(v_Val_px++) = *(v_Values_px++);
   } while (v_Val_px != c_EndVal_px );
  }
}

template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T, m, n, Mtype>::Matrix(const Array<T,e_Channels>& i_Values_rx)
: val()
{
  T* v_Val_px = this->val;
  T const * c_Val_px = &this->val[e_Channels];

  uint32_t i=0;
  do
  {
    *(v_Val_px++) = i_Values_rx[i];
    i++;
  } while (v_Val_px != c_Val_px);

}

// --------------------------------------------------------------------------
/// @brief Return size in bytes of configuration struct (POD)
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_sizeOfConfig_u32
///
/// @return Size in bytes of configuration struct
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
uint32_t Matrix<T,m,n,Mtype>::sizeOfConfig_u32(void)
{
  return sizeof(Config_s);
}

// --------------------------------------------------------------------------
/// @brief Pre-initialized matrix with zero in all cells
///
/// Matrix is neutral element of element-wise matrix addition/substraction.
///
/// @attention Be careful with huge matrixes due to return via stack.
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_zeros
///
/// @return       Matrix pre-initilized with zeros
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T,m,n, Mtype> Matrix<T,m,n, Mtype>::zeros_x()
{
  return Matrix<T,m,n, Mtype>(math::constants<T>::zero_x());
}


// --------------------------------------------------------------------------
/// @brief Set all matrix cells to zeros
///
/// Matrix is set to neutral element of element-wise matrix addition.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_setZeros
///
/// @return       Matrix pre-initialized with ones
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T,m,n, Mtype>::setZeros()
{
  (*this) = Matrix<T,m,n, Mtype>::zeros_x();
}

// --------------------------------------------------------------------------
/// @brief Pre-initialized matrix with ones in all cells
///
/// Matrix is neutral element of element-wise matrix multiplication/division.
///
/// @attention Be careful with huge matrixes due to return via stack.
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_ones
///
/// @return       Matrix pre-initialized with ones
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T,m,n, Mtype> Matrix<T,m,n, Mtype>::ones_x()
{
  return Matrix<T, m, n, Mtype>(math::constants<T>::one_x());
}


// --------------------------------------------------------------------------
/// @brief Set all matrix cells to one
///
/// Matrix is set to neutral element of element-wise matrix multiplication.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_setOnes
///
/// @return       Matrix pre-initialized with ones
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T,m,n, Mtype>::setOnes(void)
{
  (*this) = Matrix<T,m,n, Mtype>::ones_x();
  return;
}

// --------------------------------------------------------------------------
/// @brief Pre-initialized matrix with ones in diagonal line
///
/// Matrix is neutral element of the vector product of matrices.
/// This matrix is also called the identiy matrix I.
///
/// @attention Be careful with huge matrixes due to return via stack.
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
///
/// @return       Matrix pre-initialized with ones in diagonal line
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T,m,n, Mtype> Matrix<T,m,n, Mtype>::eye_x()
{
  Matrix<T,m,n, Mtype> M;
  for(uint32_t i = 0; i < e_Shortdim; i++)
  {
    M(i,i) = math::constants<T>::one_x();
  }
  return M;
}

// --------------------------------------------------------------------------
/// @brief Set matrix cells to ones in diagonal line
///
/// Matrix is set to neutral element of the vector product of matrices.
/// This matrix is also called the identiy matrix I.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_setEye
///
/// @return       Matrix pre-initialized with ones in diagonal line
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::setEye(void)
{
  this->operator=(Matrix<T,m,n, Mtype>::eye_x());
  return;
}

template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T, m, n, Mtype> Matrix<T, m, n, Mtype>::abs(void) const
{
  Matrix<T, m, n, Mtype> v_Mret_x( (*this) );
  T* v_MretVal_px = v_Mret_x.val;
  T* const c_MretValEnd_px =  &v_Mret_x.val[e_Channels];

  do
  {
    if (*v_MretVal_px < math::constants<T>::zero_x() )
    {
      *v_MretVal_px *= math::constants<T>::minusOne_x();
    }
    v_MretVal_px++;
  } while (v_MretVal_px != c_MretValEnd_px);

  return v_Mret_x;
}

// --------------------------------------------------------------------------
/// @brief Unary minus (complement of this matrix)
///
/// Function changes sign of all elements of this matrix
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_minus
///
/// @return       Matrix containing values of this matrix with sign changed
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T, m, n, Mtype> Matrix<T, m, n, Mtype>::minus() const
{
  Matrix<T, m, n, Mtype> v_Mret_x( (*this) );
  T* v_MretVal_px = v_Mret_x.val;
  const T* const c_MRetValEnd_px = &v_Mret_x.val[e_Channels];

  do
  {
    *(v_MretVal_px++) *= math::constants<T>::minusOne_x();
  } while (v_MretVal_px != c_MRetValEnd_px );

  return v_Mret_x;
}

#if 0
// --------------------------------------------------------------------------
/// @brief Add scalar value to matrix
///
/// Function adds scalar input \p value to all cell elements of Matrix
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_addScalar
///
/// @param[in] value  Scalar value to be added to all cell elements of this matrix
/// @return       Matrix with scalar value element-wise added to this matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::add(const T value)
{
  T* v_MVal_px = val;
  T* const c_MValEnd_px = &val[e_Channels];
  do {
    *(v_MVal_px++) += value;
  } while (v_MVal_px != c_MValEnd_px);
}

// --------------------------------------------------------------------------
/// @brief Add two matrices element-wise
///
/// Function adds two matrices by adding values element-wise
///
/// @attention Function assumes matrices of same size.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_ones
/// @snippet CoreTester.cpp Matrix_addMatrix
///
/// @param[in] v  Matrix to be added element-wise to this matrix
/// @return       Matrix with values of \p v element-wise added to this matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::add(const Matrix<T, m, n, Mtype>& i_M_rx)
{
  T* v_MretVal_px = val;
  T* const c_MretValEnd_px = &val[e_Channels];
  const T* v_MVal_px = i_M_rx.val;
  do {
    *(v_MretVal_px++) += *(v_MVal_px++);
  } while (v_MretVal_px != c_MretValEnd_px);
}

// --------------------------------------------------------------------------
/// @brief Subtract scalar value from matrix
///
/// Function subtracts scalar input \p value from all cell elements of Matrix
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_ones
/// @snippet CoreTester.cpp Matrix_subScalar
///
/// @param[in] value  Scalar value to be subtracted from all cell elements of this matrix
/// @return       Matrix with scalar value element-wise subtracted from this matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::sub(const T value)
{
  T* v_MretVal_px = val;
  T* const c_MretValEnd_px = &val[e_Channels];
  do
  {
    *(v_MretVal_px++) -= value;
  } while(v_MretVal_px != c_MretValEnd_px);
}

// --------------------------------------------------------------------------
/// @brief Subtract two matrices element-wise
///
/// Function subtracts two matrices by subtracting values element-wise
///
/// @attention Function assumes matrices of same size.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_ones
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_subMatrix
///
/// @param[in] v  Matrix to be subtracted element-wise to this matrix
/// @return       Matrix with values of \p v element-wise subtracted from this matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::sub(const Matrix<T, m, n, Mtype>& i_M_rx)
{
  T* v_MretVal_px = val;
  const T* const c_MretValEnd_px = &val[e_Channels];
  const T* c_MVal_px = i_M_rx.val;
  do
  {
    *(v_MretVal_px++) -= *(c_MVal_px++);
  } while (v_MretVal_px != c_MretValEnd_px);
}


// --------------------------------------------------------------------------
/// @brief Multiply matrix by scalar value
///
/// Function multiplies all cell elements of Matrix with scalar input \p val
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_ones
/// @snippet CoreTester.cpp Matrix_mulScalar
///
/// @param[in] value  Scalar value to be multiplied to all cell elements of this matrix
/// @return       Matrix with elements multiplied by scalar value
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::mul(const T i_Value_x)
{
  T* v_MretVal_px = val;
  const T* const c_MretValEnd_px = &val[e_Channels];
  do
  {
    *(v_MretVal_px++) *= i_Value_x;
  } while (v_MretVal_px != c_MretValEnd_px);
}


// --------------------------------------------------------------------------
/// @brief Multiply matrices element-wise
///
/// Function multiplies two matrices by multipliying values element-wise
///
/// @attention Function assumes matrices of same size.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_mulMatrix
///
/// @param[in] v  Matrix to be multiplied element-wise to this matrix
/// @return     Matrix with values of \p v element-wise multiplied to this matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::mul(const Matrix<T, m, n, Mtype>& i_M_rx)
{
  T* v_MretVal_px = val;
  const T* const c_MretValEnd_px = &val[e_Channels];
  const T* v_MVal_px = i_M_rx.val;
  do
  {
    *(v_MretVal_px++) *= *(v_MVal_px++);
  } while (v_MretVal_px != c_MretValEnd_px);
}


// --------------------------------------------------------------------------
/// @brief Divide matrix by scalar value
///
/// Function divides all cell elements of Matrix with scalar input \p val
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_ones
/// @snippet CoreTester.cpp Matrix_divScalar
///
/// @param[in] value  Scalar value with which to divide to all cell elements of this matrix
/// @return       Matrix with elements divided by scalar value
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::div(const T i_Value_x)
{
  T* v_MretVal_px = val;
  const T* const c_MretValEnd_px = &val[e_Channels];
  do
  {
    *(v_MretVal_px++) /= i_Value_x;
  } while (v_MretVal_px != c_MretValEnd_px);
}


// --------------------------------------------------------------------------
/// @brief Divide matrices element-wise
///
/// Function divides two matrices by dividing values element-wise
///
/// @attention Function assumes matrices of same size.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_Constructor3
/// @snippet CoreTester.cpp Matrix_ones
/// @snippet CoreTester.cpp Matrix_divMatrix
///
/// @param[in] v  Matrix with which to element-wise divide this matrix
/// @return     Matrix with values of this matrix element-wise divided with \p v
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::div(const Matrix<T, m, n, Mtype>& i_M_rx)
{
 T* v_MretVal_px = val;
 T* const c_MretValEnd_px = &val[e_Channels];
 const T* v_MVal_px = i_M_rx.val;
 do
 {
   *(v_MretVal_px++) /= *(v_MVal_px++);
 } while (v_MretVal_px != c_MretValEnd_px);
}

#endif

// --------------------------------------------------------------------------
/// @brief Scalar sum of matrix multiplication
///
/// Function summarizes element-wise multiplication of this matrix and matrix \p M
///
/// @attention Function assumes matrices of same size.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor3
/// @snippet CoreTester.cpp Matrix_ones
/// @snippet CoreTester.cpp Matrix_scalar
///
/// @param[in] M  Matrix to be multiplied element-wise to this matrix
/// @return     Scalar sum of element-wise multiplication of matrices M and this
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::scalar(const Matrix<T, m, n, Mtype>& i_M_rx) const
{
  const T* c_Val_px = this->val;
  const T* const c_MValEnd_px = &i_M_rx.val[e_Channels];
  const T* c_MVal_px = i_M_rx.val;
  T v_S_x = math::constants<T>::zero_x();

  do
  {
    v_S_x += *(c_Val_px++) * *(c_MVal_px++);
  } while (c_MVal_px != c_MValEnd_px);

  return v_S_x;
}

// --------------------------------------------------------------------------
/// @brief Frobenius norm of a matrix, compatible with vector norm
///
/// The Frobenius norm of a column or row matrix is identical to the euclidean
/// vector norm.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor3
/// @snippet CoreTester.cpp Matrix_norm
///
/// @return    Value of Forbenius norm
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::norm(void) const
{
  return math::algebra<T>::sqrt_x(this->norm2());
}

// --------------------------------------------------------------------------
/// @brief Squared Frobenius norm of a matrix
///
/// The squared Frobenius norm of a column or row matrix is identical to the
/// euclidean squared vector norm.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor3
/// @snippet CoreTester.cpp Matrix_norm
///
/// @return    Value of squared Forbenius norm
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::norm2(void) const
{
  return scalar(*this);
}

// --------------------------------------------------------------------------
/// @brief Get last (m-1, n-1)-th element of this matrix
///
/// @return    Value of last element
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::getW(void) const
{
  return (*this)(m-1,n-1);
}

// --------------------------------------------------------------------------
/// @brief Normalize this matrix to a given factor
///
/// Normalize this matrix such that last (m-1 ,n-1)-th element equals given
/// factor \p i_Factor_rx. Default value for factor is one.
///
/// @param[in] i_Factor_rx  Normalization factor
///
/// @return Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::normalize(const T& i_Factor_rx)
{
  const T c_Factor_x = i_Factor_rx / this->getW();
  T* v_Val_px = this->val;
  T* const c_ValEnd_px = &this->val[e_Channels];
  do
  {
    *(v_Val_px++) *= c_Factor_x;
  } while (v_Val_px != c_ValEnd_px);
  return;
}

// --------------------------------------------------------------------------
/// @brief Get a copy of this matrix normalized to a given factor
///
/// Get a normalized copy of this matrix such that last (m-1, n-1)-th element
/// equals given factor \p i_Factor_rx. Default value for factor is one.
///
/// @param[in] i_Factor_rx Normalization factor
///
/// @return Normalized matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T, m, n, Mtype> Matrix<T, m, n, Mtype>::getNormalized(const T& i_Factor_rx) const
{
  const T c_Factor_x = i_Factor_rx / this->getW();
  return this->mul(c_Factor_x);
}

// --------------------------------------------------------------------------
/// @brief Vector product of matrices
///
/// @attention Be careful with huge matrixes due to return via stack.
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_Constructor4
/// @snippet CoreTester.cpp Matrix_ones
/// @snippet CoreTester.cpp Matrix_mmul1
///
/// @param[in] v  Matrix to multiply with
/// @return       Vector product matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
template<uint32_t m1, uint32_t n1> inline
Matrix<T, m, n1, Mtype> Matrix<T, m, n, Mtype>::mmul(const Matrix<T, m1, n1, Mtype>& v) const
{
  Matrix<T, m, n1, Mtype> M;

  mmul(v, M);

  return M;
}

// --------------------------------------------------------------------------
/// @brief Vector product of matrices
///
/// @attention Assumes number of columns in this matrix is identical to
/// number of rows in input matrix \p v.
/// @attention Function does not reset contents of v1 prior to multiplication.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_Constructor4
/// @snippet CoreTester.cpp Matrix_ones
/// @snippet CoreTester.cpp Matrix_mmul2
///
/// @param[in]  v         Matrix to multiply this matrix with.
/// @param[out] v1        Matrix in which to store vector product of multiplication
/// @return               Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
template<uint32_t m1, uint32_t n1> inline
void Matrix<T, m, n, Mtype>::mmul(const Matrix<T, m1, n1, Mtype>& v, Matrix<T, m, n1, Mtype>& v1) const
{
  for (uint32_t i=0 ; i<m ; ++i)
  {
    for (uint32_t k=0 ; k<n1 ; ++k)
    {
      for (uint32_t j=0 ; j<n ; ++j)
      {
        v1(i,k) += (*this)(i,j)*v(j,k);
      }
    }
  }
  return ;
}

template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
template<uint32_t n1> inline
Matrix<T, m, n1, Mtype> Matrix<T, m, n, Mtype>::mmulFast(const Matrix<T, n, n1, Mtype>& i_Rhs_rx) const
{
  Matrix<T, m, n1> v_Mret_x;
  T* v_Ret_px = &(v_Mret_x(0));

  for (uint32_t i=0; i< this->e_Channels ; i += n)
  {
    const T* const c_LhsRowStart_px = &this->val[i];
    for (uint32_t k=0; k < n1; ++k)
    {
      const T* v_Lhs_px = c_LhsRowStart_px;

      for (uint32_t j=0; j < i_Rhs_rx.e_Channels; j += n1)
      {
        *v_Ret_px += *(v_Lhs_px++) * i_Rhs_rx(j + k);
      };

      ++v_Ret_px;
    }
  }

  return v_Mret_x;

}

// --------------------------------------------------------------------------
/// @brief Sum of all matrix elements
///
/// Returns sum of all elements of this matrix
///
/// @return Sum of all matrix elements

template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::sum(void) const
{
  T v_Sum_x = math::constants<T>::zero_x();
  const T* v_Val_px = this->val;
  const T* const c_ValEnd_px = &this->val[e_Channels];

  do
  {
    v_Sum_x += *(v_Val_px++);

  } while (v_Val_px != c_ValEnd_px );

  return v_Sum_x;
}

// --------------------------------------------------------------------------
/// @brief Mean value of all matrix elements
///
/// Returns mean value of all elements of this matrix
///
/// @return Mean value of all matrix elements
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::mean(void) const
{
  return this->sum() / (n*m);
}
// --------------------------------------------------------------------------
/// @brief Mean value of all matrix elements by given probability matrix
///
/// Returns mean value of all elements of this matrix by given probability
/// matrix \p i_P_rx.
///
/// @param[in] i_P_rx Probability matrix
///
/// @return Mean value of all matrix elements by given probability matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::mean(const Matrix<T, m, n, Mtype>& i_P_rx) const
{
  return ((*this) * i_P_rx).sum() / (n*m);
}

// --------------------------------------------------------------------------
/// @brief Variance (squared standard deviation) of all matrix elements
///
/// Returns variance of all elements of this matrix
///
/// @return Variance of all matrix elements
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::variance(void) const
{
  const T c_Mean_x = this->mean();
  const Matrix<T, m, n, Mtype> v_TempMat_x = (*this) - c_Mean_x;
  return (v_TempMat_x * v_TempMat_x).sum() / (n*m -1);
}

// --------------------------------------------------------------------------
/// @brief Variance of all matrix elements by given probability matrix
///
/// Returns variance of this matrix by given probability matrix \p i_P_rx.
///
/// @param[in] i_P_rx Probability matrix
///
/// @return Variance of all matrix elements
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::variance(const Matrix<T, m, n, Mtype>& i_P_rx) const
{
  const T c_Mean_x = this->mean(i_P_rx);
  const Matrix<T, m, n, Mtype> v_TempMat_x = ( (*this) - c_Mean_x ) * i_P_rx;
  return (v_TempMat_x * v_TempMat_x).sum() / (n*m -1);
}

// --------------------------------------------------------------------------
/// @brief Standard deviation of matrix elements
///
/// Returns standard deviation of elements of this matrix
///
/// @return Standard deviation of matrix elements
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::stdDev(void) const
{
  return math::algebra<T>::sqrt_x( this->variance() );
}

// --------------------------------------------------------------------------
/// @brief Standard deviation of matrix by given probability matrix
///
/// Returns standard deviation of this matrix by given probability matrix
/// \p i_P_rx.
///
/// @return Standard deviation of matrix elements
// --------------------------------------------------------------------------

template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::stdDev(const Matrix<T, m, n, Mtype>& i_P_rx) const
{
  return math::algebra<T>::sqrt_x( this->variance(i_P_rx) );
}

// --------------------------------------------------------------------------
/// @brief Minimum  value of all matrix elements
///
/// Returns minimum value of all vector elements of this matrix together with
/// its row and column indices \p o_RowIndex_ru32 and \p o_ColumnIndex  in matrix
///
/// @param[out] o_RowIndex_ru32 Row index of minimum value in this matrix
/// @param[out] o_ColIndex_ru32 Column index of minimum value in this matrix
///
/// @return Minimum value of all matrix elements

// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::minimum(uint32_t& o_RowIndex_ru32, uint32_t& o_ColumnIndex_ru32) const
{
  const T c_Min_x = this->minimum(o_RowIndex_ru32);
  o_ColumnIndex_ru32 = o_RowIndex_ru32 % n;
  o_RowIndex_ru32 = o_RowIndex_ru32 / n;
  return c_Min_x;
}

// --------------------------------------------------------------------------
/// @brief Minimum  value of all matrix elements
///
/// Returns minimum value of all elements of this matrix together with its
/// index \p o_Index_ru32 in matrix array (vector of rows of this matrix)
///
/// @param[out] o_Index_ru32 Index of minimum value in matrix array
///
/// @return Minimum value of all matrix elements
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::minimum(uint32_t& o_Index_ru32) const
{

  const T* c_Val_px = this->val;
  const T* const c_ValEnd_px = &this->val[e_Channels];
  uint32_t v_Cnt_u32 = 0U;

  T v_Min_x = *(c_Val_px++);
  o_Index_ru32 = 0U;

  do
  {
    if ( *c_Val_px < v_Min_x)
    {
      v_Min_x = *(c_Val_px++);
      ++v_Cnt_u32;
      o_Index_ru32 = v_Cnt_u32;
    }
    else
    {
      ++c_Val_px;
      ++v_Cnt_u32;
    }
  } while (c_Val_px != c_ValEnd_px);

  return v_Min_x;

}

// --------------------------------------------------------------------------
/// @brief Minimum  value of all matrix elements
///
/// Returns minimum value of all vector elements of this matrix
///
/// @return Minimum value of all matrix elements
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::minimum(void) const
{

  const T* c_Val_px = this->val;
  const T* const c_ValEnd_px = &this->val[e_Channels];

  T v_Min_x = *(c_Val_px++);

  do
  {
    if ( *c_Val_px < v_Min_x)
    {
      v_Min_x = *(c_Val_px++);
    }
    else
    {
      ++c_Val_px;
    }
  } while (c_Val_px != c_ValEnd_px);

  return v_Min_x;
}

// --------------------------------------------------------------------------
/// @brief Maximum  value of all matrix elements
///
/// Returns minimum value of all elements of this matrix together with its
/// row and column indices \p o_RowIndex_ru32 and \p o_ColumnIndex  in matrix
///
/// @param[out] o_RowIndex_ru32 Row index of maximum value in this matrix
/// @param[out] o_ColIndex_ru32 Column index of maximum value in this matrix
///
/// @return Maximum value of all matrix elements
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::maximum(uint32_t& o_RowIndex_ru32, uint32_t& o_ColumnIndex_ru32) const
{
  return math::neg_x( this->minus().minimum(o_RowIndex_ru32, o_ColumnIndex_ru32) );;
}

// --------------------------------------------------------------------------
/// @brief Minimum  value of all matrix elements
///
/// Returns minimum value of all elements of this matrix together with its
/// index \p o_Index_ru32 in matrix array (vector of rows of this matrix)
///
/// param[out] o_Index_ru32 Index in matrix array
///
/// @return Minimum value of all matrix elements
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::maximum(uint32_t& o_Index_ru32) const
{
  return math::neg_x<T>( this->minus().minimum(o_Index_ru32) );
}

// --------------------------------------------------------------------------
/// @brief Maximum value of all matrix elements
///
/// Returns maximum value of all vector elements of this matrix
///
/// @return Maximum value of all matrix elements
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T Matrix<T, m, n, Mtype>::maximum(void) const
{
  return math::neg_x<T>( this->minus().minimum() );
}

template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
bool_t Matrix<T, m, n, Mtype>::equals(const Matrix<T, m, n, Mtype>& v_M_rx) const
{
  bool_t v_Equal_b = true;
  uint32_t i = 0;
  uint32_t j = 0;

  while ((true == v_Equal_b) && (i < m))
  {
    while ((true == v_Equal_b) && (j < n))
    {
      v_Equal_b = math::equal_x( (*this)(i,j), v_M_rx(i,j) );
      j++;
    }
    j=0;
    i++;
  }
  return v_Equal_b;
}

// --------------------------------------------------------------------------
/// @brief Extract row of this matrix
///
/// Function returns contents of specific row as 1D matrix.
///
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_row
///
/// @param[in] i          Row number from which to extract cell elements
/// @return               1D Matrix pre-initialized with contents of row \p i from this matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T, 1, n> Matrix<T, m, n, Mtype>::row(uint32_t i) const
{
  AssertFunction( i < m, "Row index out of bounds.");
  Matrix<T, 1, n> v_Mat_x;

  const T* const c_Row_px = &( (*this)(i*n) );
  memcpy( &v_Mat_x(0), c_Row_px, v_Mat_x.sizeOfConfig_u32() );
  return v_Mat_x;

}

// --------------------------------------------------------------------------
/// @brief Set row contents in this matrix
///
/// Function copies the contents of 1D matrix \p i_Mat_x to specific row of this matrix.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_row
/// @snippet CoreTester.cpp Matrix_setRow
///
/// @param[in] i          Row number in which to overwrite cell elements.
/// @param[in] i_Mat_x    1D Matrix from which to copy cell elements.
/// @return               Nothing
// --------------------------------------------------------------------------
//! Set row of matrix
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::setRow(uint32_t i, const Matrix<T,1,n>& i_Mat_x)
{
  AssertFunction( i < m, "Row index out of bounds.");

  T* const v_Row_px = & ( (*this)(i*n) );
  memcpy( v_Row_px, &i_Mat_x(0), i_Mat_x.sizeOfConfig_u32());

  return;
}

// --------------------------------------------------------------------------
/// @brief Set column contents in this matrix
///
/// Function copies the contents of 1D matrix \p i_Mat_x to specific column of this matrix.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_col
/// @snippet CoreTester.cpp Matrix_setCol
///
/// @param[in] j          Column number in which to overwrite cell elements.
/// @param[in] i_Mat_x    1D Matrix from which to copy cell elements.
/// @return               Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::setCol(uint32_t j, const Matrix<T,m,1>& i_Mat_x)
{
  AssertFunction( j < n, "Column index out of bounds.");

  const T* v_MatPtr_px = & ( i_Mat_x(0) );
  for (uint32_t i=j; i < e_Channels; i += n)
  {
    this->val[i] = *(v_MatPtr_px++);
  }

  return;
}

// --------------------------------------------------------------------------
/// @brief Extract column of this matrix
///
/// Function returns contents of specific column as 1D matrix.
///
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_col
///
/// @param[in] j          Column number from which to extract cell elements
/// @return               1D Matrix pre-initialized with contents of column \p j from this matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T, m, 1> Matrix<T, m, n, Mtype>::col(uint32_t j) const
{
  AssertFunction( j < n, "Column index out of bounds.");

  Matrix<T, m, 1> v_Mat_x;
  T* v_MatPtr_px = &v_Mat_x(0);

  for (uint32_t i=j; i < e_Channels; i+=n)
  {
    *(v_MatPtr_px++) = this->val[i];
  }

  return v_Mat_x;

}


// --------------------------------------------------------------------------
/// @brief Extract sub-matrix of this matrix
///
/// Function returns contents of specific subset of this Matrix.
///
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_submat
///
/// @param[in] i_StartRow_u32             Row number at which to start extraction of cell elements
/// @param[in] i_StartColumn_u32          Column number at which to start extraction of cell elements
/// @return                               Matrix pre-initialized with specified sub-contents of this Matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
template<uint32_t m1, uint32_t n1> inline
Matrix<T, m1, n1, Mtype> Matrix<T, m, n, Mtype>::subMatrix(const uint32_t i_StartRow_u32,
                                                           const uint32_t i_StartColumn_u32) const
{
  StaticAssert( (m1 <= m) && (n1 <= n), "subMatrix" )
  AssertFunction( ( (i_StartRow_u32+m1) <= m ) && ( (i_StartColumn_u32+n1) <= n), "Dimensions out of bounds.");

  Matrix<T, m1, n1> v_SubMat_x;

  const uint32_t c_RowSize_u32 = n1*sizeof(T);

  // PRQA S 4107 10 // ignore QACPP maintenance warning
  // PRQA S 1481 10 // multiple variables declared in one line allowed here
  // PRQA S 3348 10 // comma seperator used in a loop expression for a loop expression
  for (uint32_t subIdx_u32 = 0, thisIdx_u32 = i_StartRow_u32 * n + i_StartColumn_u32;
       subIdx_u32 < (n1*m1);
       subIdx_u32 += n1, thisIdx_u32 += n)
  {
    memcpy(&(v_SubMat_x(subIdx_u32)), & ( (*this)(thisIdx_u32) ), c_RowSize_u32 );
  }

  return v_SubMat_x;
}

// --------------------------------------------------------------------------
/// @brief Returns this matrix reshaped with dimensions m1 * n1
///
/// Function returns this matrix with m1 rows and n1 columns.
/// It is also possible to reshape a matrix using the assignment operator. 
/// This in turn casts the right hand Matrix object into the dimensions of 
/// the dimensions of the left hand Matrix.
/// Additionally the matrix copy constructor can be called  with an input matrix
/// of dimensions but same size.
/// This in turn the input matrix values will be casted to the base type specified
/// in the matrix declaration.
/// 
/// @par Example use:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_reshape
/// @snippet CoreTester.cpp Matrix_castReshape
/// @snippet CoreTester.cpp Matrix_c-torReshape
///
/// @return               Reshaped matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
template<uint32_t m1, uint32_t n1> inline
Matrix<T, m1, n1, Mtype> Matrix<T, m, n, Mtype>::reshape(void) const
{
  StaticAssert( (m1 * n1) == (m * n), "Number of elements of matrices is not equal");
  Matrix<T, m1, n1, Mtype> v_Mat_x;

  memcpy(&(v_Mat_x(0)), &( (*this)(0)), this->sizeOfConfig_u32() );

  return v_Mat_x;
}

// --------------------------------------------------------------------------
/// @brief Returns copy of this matrix of base type T1
///
/// Function returns this matrix converted to base type T1.
/// It is also possible to convert a matrix using the assignment operator.
/// This in turn casts the right hand Matrix object to the base type of
/// of the left hand Matrix.
/// Additionally the matrix copy constructor can be called  with an input matrix
/// of different base type.
/// In turn the input matrix values will be casted to the base type specified
/// in the matrix declaration.
///
/// @par Example use:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_convert
/// @snippet CoreTester.cpp Matrix_castConvert
/// @snippet CoreTester.cpp Matrix_c-torConvert
///
/// @return               Converted matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
template<typename T1> inline
Matrix<T1, m, n, Mtype> Matrix<T, m, n, Mtype>::convert(void) const
{
  typename Matrix<T1, m, n, Mtype>::Config_s v_MCfg_s;
  const T*  v_Val_px = this->val;
  const T* const c_ValEnd_px = &(this->val[e_Channels]);
  T1* v_MVal_px = &(v_MCfg_s.cVal_ax[0]);

  do
  {
    *v_MVal_px = static_cast<T1>(*v_Val_px);
    ++v_Val_px;
    ++v_MVal_px;
  } while (c_ValEnd_px != v_Val_px);

  return Matrix<T1, m, n, Mtype>(v_MCfg_s);
}

// --------------------------------------------------------------------------
/// @brief Returns copy of this matrix of base type T1 reshaped with dimensions m1 * n1
///
/// Function returns this matrix converted to base type T1 reshaped with dimensions m1 * n1.
/// It is also possible to convert and reshape a matrix using the assignment operator.
/// This in turn casts the right hand Matrix object to the base type of
/// of the left hand Matrix.
/// Additionally the matrix copy constructor can be called  with an input matrix
/// of different base type.
/// In turn the input matrix values will be casted to the base type and dimensions
/// specified in the matrix declaration.
///
/// @par Example use:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_convertReshape
/// @snippet CoreTester.cpp Matrix_castConvertReshape
/// @snippet CoreTester.cpp Matrix_c-torConvertReshape
///
/// @return               Reshaped and converted matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
template<typename T1, uint32_t m1, uint32_t n1> inline
Matrix<T1, m1, n1, Mtype> Matrix<T, m, n, Mtype>::convertAndReshape(void) const
{
  return this->reshape<m1, n1>().convert<T1>();
}

// --------------------------------------------------------------------------
/// @brief Transpose matrix
///
/// Function returns Matrix with transposed elements of this Matrix.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_Constructor3
/// @snippet CoreTester.cpp Matrix_t
///
/// @return                 Matrix pre-initialized with transposed elements of this Matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T, n, m> Matrix<T, m, n, Mtype>::t() const
{
  Matrix<T, n, m> v_Mat_x;

  for (uint32_t i=0 ; i<m ; i++)
  {
    for (uint32_t k=0 ; k<n ; k++)
    {
      v_Mat_x(k,i) = (*this)(i,k);
    }
  }
  return v_Mat_x;
}

// --------------------------------------------------------------------------
/// @brief Join matrices column-wise (this | input matrix)
///
/// Function returns Matrix joined column-wise with \p i_Mat_x
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_col
/// @snippet CoreTester.cpp Matrix_join2
///
/// @return                 Matrix joined column-wise with \p i_Mat_x
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
template<uint32_t n1> inline
Matrix<T, m, n+n1, Mtype> Matrix<T, m, n, Mtype>::joinCol(const Matrix<T, m, n1, Mtype>& i_Mat_rx) const
{
  Matrix<T, m, n + n1, Mtype> v;
  for (uint32_t j=0; j<n; j++) {
    v.setCol(j, this->col(j));
  };
  for (uint32_t j=n; j<(n+n1); j++) {
    v.setCol(j, i_Mat_rx.col(j-n));
  };
  return v;
}

// --------------------------------------------------------------------------
/// @brief Join matrices row-wise (this , input matrix)
///
/// Function returns Matrix joined row-wise with \p i_Mat_x
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_row
/// @snippet CoreTester.cpp Matrix_join1
///
/// @return                 Matrix joined row-wise with \p i_Mat_x
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> template<uint32_t m1> inline
Matrix<T, m+m1, n, Mtype> Matrix<T, m, n, Mtype>::joinRow(const Matrix<T, m1, n, Mtype>& i_Mat_rx) const
{
  Matrix<T, m+m1, n, Mtype> v;
  for (uint32_t i=0; i<m; i++) {
    v.setRow(i, this->row(i));
  };
  for (uint32_t i=m; i<(m+m1); i++) {
    v.setRow(i, i_Mat_rx.row(i-m));
  };
  return v;
}

// --------------------------------------------------------------------------
/// @brief Indexing operator by internal array index
///
/// Operator allows direct access of cell element by implementing (k) where k
/// is index in value array. Values are returned as non-modifyable constants.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_operator()_3
///
/// @return                 Const cell value reference at specified position
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
const T& Matrix<T, m, n, Mtype>::operator()(uint32_t k) const
{
  AssertFunction(k < static_cast<uint32_t> (e_Channels),"Index out of bounds.");
  // PRQA S 3083 1 // alternative would be copying the value ...
  return const_cast<T&> (this->val[k]);
}

// --------------------------------------------------------------------------
/// @brief Access matrix elements by index of internal array
///
/// Operator allows direct access of cell element by implementing (k) where k
/// is index in value array.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_operator()_4
///
/// @return                 Cell value reference at specified position
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T& Matrix<T, m, n, Mtype>::operator() (uint32_t k)
{
  AssertFunction(k < static_cast<uint32_t> (e_Channels),"Index out of bounds.");
  return this->val[k];
}

// --------------------------------------------------------------------------
/// @brief Indexing operator by row and column
///
/// Operator allows direct access of cell element by implementing (r, c) where r and c is row
/// and column respectively. Values are returned as non-modifyable constants.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_eye
/// @snippet CoreTester.cpp Matrix_operator()_1
///
/// @return                 Const cell value reference at specified position
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
const T& Matrix<T, m, n, Mtype>::operator()(uint32_t i, uint32_t j) const
{
  uint32_t k = MatCalcIndex<Mtype>::calcIndex_u32(i,j,m,n,false);
  AssertFunction(k < static_cast<uint32_t>(e_Channels), "Indices out of bounds.");
  return (*this)(k);
}

// --------------------------------------------------------------------------
/// @brief Indexing operator
///
/// Operator allows direct access of cell element by implementing (r, c) where r and c is row
/// and column respectively.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_Constructor1
/// @snippet CoreTester.cpp Matrix_operator()_2
///
/// @return                 Cell value reference at specified position
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
T& Matrix<T, m, n, Mtype>::operator()(uint32_t i, uint32_t j)
{
  uint32_t k = MatCalcIndex<Mtype>::calcIndex_u32(i,j,m,n,true);
  return (*this)(k);
}


// --------------------------------------------------------------------------
/// @brief Scalar assignment operator
///
/// Operator sets value of all cell elements to specific scalar value.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_operator=_1
///
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::operator= (const T i_Value_x)
{
  T* v_Val_px = this->val;
  T* const v_ValEnd_px = &this->val[e_Channels];

  do
  {
    *(v_Val_px++) = i_Value_x;
  } while (v_Val_px != v_ValEnd_px);

  return;
}

// --------------------------------------------------------------------------
/// @brief Configuration data assignement operator
///
/// Operator sets value of all cell elements to corresponding values from
/// configuration object.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_operator=_3
///
/// @return                 This Matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::operator= (const Config_s& i_Config_s)
{
  this->updateConfig_v((i_Config_s));
  return;
}

// --------------------------------------------------------------------------
/// @brief Matrix assignment operator
///
/// Operator sets value of all cell elements from this Matrix to values of input
/// argument matrix.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_operator=_2
///
/// @return                 This matrix
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
Matrix<T, m, n, Mtype> Matrix<T, m, n, Mtype>::operator= (const Matrix<T, m, n, Mtype>& i_Mat_rx)
{
  T* v_Val_px = this->val;
  const T* c_MatVal_px = &(i_Mat_rx.val[0]);

     memcpy(v_Val_px, c_MatVal_px, this->sizeOfConfig_u32());
  return *this;

}

// --------------------------------------------------------------------------
/// @brief Matrix comparison operator
///
/// Operator compares contents of two matrices element-wise
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_operator==
///
/// @param[in] v            Matrix with which to compare
/// @return                 Boolean, True = equal, False = not equal
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
bool_t Matrix<T, m, n, Mtype>::operator== (const Matrix<T, m, n, Mtype>& i_V_rx) const
{
  bool_t v_Equal_b = true;
  uint32_t i = 0;
  uint32_t j = 0;

  while (true == v_Equal_b && i < m)
  {
    while (true == v_Equal_b && j < n)
    {
      v_Equal_b = math::equal_x( (*this)(i,j), i_V_rx(i,j) );
      j++;
    }
    j=0;
    i++;
  }
  return v_Equal_b;
}

// --------------------------------------------------------------------------
/// @brief Matrix comparison operator
///
/// Operator compares contents of two matrices element-wise
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_operator!=
///
/// @param[in] v            Matrix with which to compare
/// @return                 Boolean, True = not equal, False = equal
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
bool_t Matrix<T, m, n, Mtype>::operator!= (const Matrix<T, m, n, Mtype>& i_V_rx) const
{
  return static_cast<bool_t>(!(*this == i_V_rx));
}
  

// --------------------------------------------------------------------------
/// @brief Print matrix in standard format
///
/// Function prints matrix contents to stdout using tabs as column separators and
/// newline as row separator.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_print1
///
/// @return                 Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::print() const
{
  for (uint32_t i=0 ; i<m ; ++i)
  {
    log_printf ("\n");
    for (uint32_t k=0 ; k<n ; ++k) {
      log_printf ("%g\t", (*this)(i,k));
    }
  }
  log_printf ("\n");
}

// --------------------------------------------------------------------------
/// @brief Print matrix in standard format with heading comments line
///
/// Function prints matrix contents to stdout using tabs as column separators and
/// newline as row separator. In addition, a heading comment line is preprended and
/// separated from the contents with new line.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_print2
///
/// @return                 Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::print(const char* comment) const
{
  log_printf("\n%s\n",comment);
  this->print();
  return;
}

// --------------------------------------------------------------------------
/// @brief Print matrix in special output format
///
/// Function prints matrix contents using formatting specified in \p i_OutputFormat_rs
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_print3
///
/// @param[in] i_OutputFormat_rs Output formatting structure definint pre-/postfixes
/// and row/column separators.
/// @return                 Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype> inline
void Matrix<T, m, n, Mtype>::print(const MatrixOutputFormat_s& i_OutputFormat_rs) const
{
  log_printf ("%s", i_OutputFormat_rs.headingChars_ac);
  for (uint32_t i=0 ; i<m ; i++)
  {

    for (uint32_t j=0 ; j<n-1 ; j++) {
      log_printf ("%f%s", (*this)(i,j), i_OutputFormat_rs.columnSeperatorChars_ac);
    }
    log_printf("%f%s", (*this)(i,n-1), (m-1) != i ? i_OutputFormat_rs.rowSeperatorChars_ac
                                                  : i_OutputFormat_rs.trailingChars_ac);
  }
  return;
}

// --------------------------------------------------------------------------
/// @brief Print matrix in special output format
///
/// Function prints matrix contents using formatting specified in \p i_OutputFormat_rs 
/// In addition, a heading comment line is preprended and separated from the contents with 
/// new line.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_print4
///
/// @param[in] i_OutputFormat_rs Output formatting structure definint pre-/postfixes
/// and row/column separators.
/// @param[in] comment  Additional comment to preprend output 
/// @return             Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
void Matrix<T, m, n, Mtype>::print(const MatrixOutputFormat_s& i_OutputFormat_rs, char* comment) const
{
  log_printf("\n%s", comment);
  this->print(i_OutputFormat_rs);
  return;
}

// --------------------------------------------------------------------------
/// @brief Print matrix in predefined output format style
///
/// Function prints matrix contents using formatting specified by predefines of 
/// enum type MatrixOutputFormatStyle_e (core/MeclTypes.h) in \p i_OutputFormatSytle_e
/// In addition, a heading comment line is preprended and separated from the contents with
/// new line.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_print5
///
/// @param[in] i_OutputFormatStyle_e Output format style
/// @return             Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
void Matrix<T, m, n, Mtype>::print(const MatrixOutputFormatStyle_e i_OutputFormatStyle_e) const
{
  this->print( Matrix<T, m, n, Mtype>::getMatrixOutputFormat(i_OutputFormatStyle_e) );
  return;
}

// --------------------------------------------------------------------------
/// @brief Print matrix in predefined output format style
///
/// Function prints matrix contents using formatting specified by predefines of
/// enum type MatrixOutputFormatStyle_e (see core/MeclTypes.h) in \p i_OutputFormatSytle_e
/// In addition, a heading comment line is preprended and separated from the contents with
/// new line.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Matrix_print6
///
/// @param[in] i_OutputFormatStyle_e Output format style
/// @param[in] comment  Additional comment to preprend output
/// @return             Nothing
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
void Matrix<T, m, n, Mtype>::print(const MatrixOutputFormatStyle_e i_OutputFormatStyle_e, char* comment) const
{
  log_printf("\n%s", comment);
  this->print(i_OutputFormatStyle_e);
  return;
}

// --------------------------------------------------------------------------
/// @brief Get a defined matrix output format
///
/// Function gets matrix format string of type \p i_OutputFormat_e.
/// MatrixOutputFormatStyle_e is defined in MeclTypes.h
/// @param[in] i_OutputFormat_e Matrix format style (type)
///
/// @return Matrix format string
// --------------------------------------------------------------------------
template<typename T, uint32_t m, uint32_t n, MatrixType_e Mtype>
MatrixOutputFormat_s Matrix<T, m, n, Mtype>::getMatrixOutputFormat(MatrixOutputFormatStyle_e i_OutputFormat_e)
{

  MatrixOutputFormat_s v_OutputFormat_s = { "\n\t", "\n\t", "\t", "\n" }; // standard

  switch(i_OutputFormat_e)
  {
    case e_Octave:
    {
      const MatrixOutputFormat_s c_OctaveFormat_s = {
          " = [\n\t",     // heading chars, use comment parameter in Matrix::print() as variable name
          "\n\t",         // row separation chars
          ",\t",          // column separation chars
          "\n];\n"        // trailing chars
      };
      memcpy(&v_OutputFormat_s, &c_OctaveFormat_s, sizeof(MatrixOutputFormat_s));
    }
    break;

    case e_CStruct:
    {
      const MatrixOutputFormat_s c_CStructFormat_s = {
          " = {\n\t",     // heading chars, use comment parameter in Matrix::print() as variable name
          ",\n\t",         // row separation chars
          ",\t",          // column separation chars
          "\n};\n"        // trailing chars
      };
      memcpy(&v_OutputFormat_s, &c_CStructFormat_s, sizeof(MatrixOutputFormat_s));
    }
    break;

    case e_Standard: // fall through
    default:
      break;
  }
  return v_OutputFormat_s;
}

}
}





#endif /* SRC_CORE_MATRIX_HPP_ */
