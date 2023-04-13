//  --------------------------------------------------------------------------
/// @file Matrix.h
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
/// @author Sebastian Pliefke (sebastian.pliefke@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef MECL_CORE_MATRIX_H_
#define MECL_CORE_MATRIX_H_

//! @cond
// PRQA S 1020 1 // macro is used by MKS
#define Matrix_D_VERSION_ID "$Id: Matrix.h 1.2 2019/08/08 15:28:37CEST Michael Becker (MEE_MBECKE) Draft  $"
//! @endcond

//! @cond
// PRQA S 284 EOF // switch off QACPP "is not a namespace or class", since this is not correctly resolved here by lint checker
// PRQA S 5127 EOF // switch off The stream input/output libary <cstdio> shall not be used (MeclAssert.h)
// PRQA S 621 EOF // linter parsing problems ...
// PRQA S 3708 EOF   // floating point arithmic MUST be used here
// PRQA S 467 EOF // ignore QACPP maintenance waring

//! @endcond

#include "Array.h"
#include "MeclAssert.h"
#include "MeclTypes.h"
#include "MatCalcIndex.h"
#include "mecl/math/math.h"

namespace mecl
{
namespace core
{

// --------------------------------------------------------------------------
/// @brief Default indexing function
///
/// Function implements default indexing scheme for standard matrices. 
/// Inheriting may implement different indexing functions and apply to indexing 
/// function pointer, i.e. in sparse and triangular matrices.
///
/// @param[in] i  Row index
/// @param[in] j  Column index
/// @param[in] doAssert  Defines whether or not the function produces an assertion
/// when indexing outside of matrix boundaries is attempted.
/// 
/// @return Index in value array
// --------------------------------------------------------------------------
template<uint32_t m, uint32_t n>
static uint32_t calcIndex_u32(uint32_t i, uint32_t j, bool_t doAssert)
{
  if (doAssert) {
    AssertFunction(i < m && j < n,"Matrix indices out of bounds.");
  }
  return i * n + j;
}

// --------------------------------------------------------------------------
/// @class Matrix
/// @brief Basic matrix data type of Magna Electronics Library
///
/// The matrix data type is a one of the primary data types of the Magna 
/// Electronics Library. The template based implementation allows working
/// with matrix related data with different base types. The object includes 
/// basic mathematical operations like scalar products, transposing etc.
// --------------------------------------------------------------------------
template<typename T, uint32_t m,  uint32_t n, MatrixType_e Mtype = e_Default>
class Matrix
{

public:
  //! Enumeration type for matrix properties
  enum {
    e_Rows     = m,                 ///< Number of rows
    e_Cols     = n,                 ///< Number of columns
    e_Channels = e_Rows*e_Cols,     ///< Number of elements
    e_Shortdim = ((m < n) ? m : n), ///< Shortest dimension
    e_Longdim  = ((m < n) ? n : m)  ///< Longest dimension
  };

  //! @struct Config_s
  //! @brief Configuration data set
  struct Config_s
  {
    T cVal_ax[e_Channels];            ///< Value array
  };                            ///< Configuration data set

  //! define function pointer type (signature) for index calculation
  typedef uint32_t (*CalcIndexSig_fp) (uint32_t i, uint32_t j, bool_t doAssert);

  // *******************************
  // * construtors and initialization

  //! Default constructor
  Matrix(void);

  //! Constructor with initialization value
  explicit Matrix (T i_Value_x);

  //! Constructor with initialization by C array
  explicit Matrix(const T values[], uint32_t numValues);

  //! Constructor with initialization by mecl::core::Array type
  explicit Matrix(const Array<T,e_Channels>& i_Values_rx);

  //! Copy Constructor for Config_s
  explicit Matrix(const Config_s& i_Config_rs);

  //! Copy instance to POD Config_s
  void copyConfig_v(Config_s& o_Config_rs) const;

  //! Copy instance to POD Config_s of different base type
  template<typename T1>
  void copyConfig_v(typename Matrix<T1, m, n, Mtype>::Config_s& o_Config_rs) const;

  //! Copy instance to volatile POD Config_s
  void copyConfigVolatile_v(volatile Config_s& o_Config_rs) const;

  //! Copy instance to volatiole POD Config_s of different base type
  template<typename T1>
  void copyConfigVolatile_v(volatile typename Matrix<T1, m, n, Mtype>::Config_s& o_Config_rs) const;

  //! update instance with Config_s
  void updateConfig_v(const Config_s& i_Config_rs);

  //! update instance with Config_s of different base type
  template<typename T1>
  void updateConfig_v(const typename Matrix<T1, m, n, Mtype>::Config_s& i_Config_rs);

  //! Pre-initilized matrix with zero in all cells
  static Matrix zeros_x();

  //! Pre-initilized matrix with ones in all cells
  static Matrix ones_x();

  //! Pre-initilized matrix with ones in diagonal line
  static Matrix eye_x();

  //! Destructor
  ~Matrix()
  {}

  // *******************************
  // * common operations

  static uint32_t sizeOfConfig_u32(void);

  // *******************************
  // * math operations

  //! Set all elements to zero
  void setZeros(void);

  //! Set all elements to one.
  void setOnes(void);

  //! Set diagonal elements to one.
  void setEye(void);

  //! dot product computed with the default precision
  T scalar(const Matrix<T, m, n, Mtype>& i_M_rx) const;

  //! Frobenius norm of a matrix, compatible with vector norm
  T norm(void) const;

  //! Squared Frobenius norm of a matrix
  T norm2(void) const;

  //! Get (m-1,n-1)-th element of Matrix
   T getW() const;

  //! Normalize matrix such that (m,n)-th element equals a factor
  void normalize(const T& i_Factor_rx = math::constants<T>::one_x() );

  //! Get normalized matrix such that (m-n)-th element equals a factor
  Matrix<T, m, n, Mtype> getNormalized(const T& i_Factor_rx = math::constants<T>::one_x() ) const;

  //! Vector product of matrix multiplication
  template<uint32_t m1, uint32_t n1>
  Matrix<T, m, n1, Mtype> mmul(const Matrix<T, m1, n1, Mtype>& v) const;

  //! Vector product of matrix multiplication (2nd version)
  template<uint32_t m1, uint32_t n1>
  void mmul(const Matrix<T, m1, n1, Mtype>& v, Matrix<T, m, n1, Mtype>& v1) const;

  //! Fast vector product, not suitable for matrix subclasses
  template<uint32_t n1>
  Matrix<T, m, n1, Mtype> mmulFast(const Matrix<T, n, n1, Mtype>& i_Rhs_rx) const;

  //! Compute determinate of largest square (sub)matrix
  T det() const;

  //! Compute inverse matrix of largest square (sub)matrix, assert if determinant is zero
  Matrix<T, m, n, Mtype> inverse() const;

  //! Get major diagonal
  // REMARK: must be implemented inline here since QA-cpp cannot parse function return type correctly
  Matrix<T, Matrix<T, m, n, Mtype>::e_Shortdim, 1> diag() const
  {
    Matrix<T, Matrix<T, m, n, Mtype>::e_Shortdim, 1> v_Diag_x;
    for (uint32_t i=0; i < e_Shortdim; i++) {
      v_Diag_x(i,0) = (*this)(i,i);
    }
    return v_Diag_x;
  }

  // *******************************
  // * nummeric evaluation

  //! Get sum of all matrix entries
  T sum(void) const;

  //! Get maximum value of all matrix entries
  T maximum(void) const;

  //! Get maximum value of all matrix entries with array index
  T maximum(uint32_t& o_Index_ru32) const;

  //! Get maximum value of all matrix entries with row and column indices
  T maximum(uint32_t& o_RowIndex_ru32, uint32_t& o_ColumnIndex_ru32) const;

  //! Get minimum value of all matrix entries
  T minimum(void) const;

  //! Get minimum value of all matrix entries with array index
  T minimum(uint32_t& o_Index_ru32) const;

  //! Get minimum value of all matrix entries with row and column indices
  T minimum(uint32_t& o_RowIndex_ru32, uint32_t& o_ColumnIndex_ru32) const;

 // ********************************
 // * comparision

  //! Compare this matrix to other matrix for exact equality
  bool_t equals(const Matrix<T, m, n, Mtype>& v_M_rx) const;

  //! Compare this matrix to other matrix for approximate equality
  bool_t almostEquals(const Matrix<T, m, n, Mtype>& v_M_rx, const uint16_t i_ULPS_u16 = 4U) const;

  //! Compare this matrix to other matrix for approximate equality and get error
  bool_t almostEquals(const Matrix<T, m, n, Mtype>& v_M_rx,
                                          T& o_Error_rx,
                              const uint16_t i_ULPS_u16 = 4U

                      ) const;



  // *******************************
  // * statistics

  //! Get mean of matrix entries
  T mean(void) const;

  //! Get mean of based on givien probability matrix
  T mean(const Matrix<T, m, n, Mtype>& i_P_rx) const;

  //! Get variance of matrix entries
  T variance(void) const;

  //! Get variance based on given probability matrix
  T variance(const Matrix<T,m,n, Mtype>& i_P_rx) const;

  //! Get standard deviation of matrix entries
  T stdDev(void) const;

  //! Get standard deviation based on given probability matrix
  T stdDev(const Matrix<T, m, n, Mtype>& i_P_rx) const;

  // *******************************
  // * conversions

  //! Extract row of this matrix
  Matrix<T, 1, n> row(uint32_t i) const;

  //! Extract column of this matrix
  Matrix<T, m, 1> col(uint32_t j) const;

  //! Set row of matrix
  void setRow(uint32_t i, const Matrix<T,1,n>& i_Mat_x);

  //! Set column of matrix
  void setCol(uint32_t j, const Matrix<T,m,1>& i_Mat_x);

  //! Extract part of the matrix
  template<uint32_t m1, uint32_t n1>
  Matrix<T, m1, n1, Mtype> subMatrix(const uint32_t i_StartRow_u32=0,
                                     const uint32_t i_StartColumn_u32=0) const;

  //! change the matrix shape
  template<uint32_t m1, uint32_t n1>
  Matrix<T, m1, n1, Mtype> reshape(void) const;

  //! copy and convert this matrix to a different base type T1
  template<typename T1>
  Matrix<T1, m, n, Mtype> convert(void) const;

  //! change matrix shape and convert to a different base type
  template<typename T1, uint32_t m1, uint32_t n1>
  Matrix<T1, m1, n1, Mtype> convertAndReshape(void) const;

  //! Transpose matrix
  Matrix<T, n, m> t() const;

  //! Join matrices column-wise (this | input matrix)
  template<uint32_t n1>
  Matrix<T, m, n+n1, Mtype> joinCol(const Matrix<T, m, n1, Mtype>& i_Mat_rx) const;

  //! Join matrices row-wise (this , input matrix)
  template<uint32_t m1>
  Matrix<T, m+m1, n, Mtype> joinRow(const Matrix<T, m1, n, Mtype>& i_Mat_rx) const;

  //! Get matrix with absolute values of elments of this matrix
  Matrix<T, m, n, Mtype> abs() const;

  //! Unary minus, change sign of matrix elements
  Matrix<T, m, n, Mtype> minus() const;


  // *******************************
  // * operators
  // *******************************

  // -------------------------------
  // * access operators

  //! Access matrix elements by index of internal array as const
  const T& operator() (uint32_t k) const;

  //! Access matrix elements by index of internal array
  T& operator() (uint32_t k);

  //! Access matrix elements by row and column as const
  const T& operator() (uint32_t i, uint32_t j) const;

  //! Access matrix elements by row and column
  T& operator() (uint32_t i, uint32_t j);

  // -------------------------------
  // * comparison operators

  //! Matrix equality operator
  bool_t operator== (const Matrix<T, m, n, Mtype>& i_V_rx) const;
  
  //! Matrix unequality operator
  bool_t operator!= (const Matrix<T, m, n, Mtype>& i_V_rx) const;

  // -------------------------------
  // * assignment operators

  //! Set all elements to specific value
  void operator= (const T i_Value_x);

  //! assignment with configuration struct POD (copy)
  void operator= (const Config_s& i_Config_s);

  //! assignment with matrix (copy)
  Matrix<T, m, n, Mtype> operator= (const Matrix<T, m, n, Mtype>& i_Mat_rx);

  // -------------------------------
  // * cast operators

  //! cast Matrix to different dimensions (reshape and copy)
  template<uint32_t m1, uint32_t n1>
  operator Matrix<T, m1, n1>() const
  {
    return reshape<m1, n1>();
  }

  //! cast Matrix to different base type
  template<typename T1>
  operator Matrix<T1, m, n>() const
  {
    return convert<T1>();
  }

  //! cast Matrix to different dimensions and base type
  template<typename T1, uint32_t m1, uint32_t n1>
  operator Matrix<T1, m1, n1>() const
  {
    return convertAndReshape<T1, m1, n1>();
  }

  // PRQA S 3361 ++ // post-increment is allowed here as sub-expression
  // -------------------------------
  // * arithmetic operators + assignment versions

  //! Operator of addition by scalar + assignment
  void operator+=(const T& i_V_rx)
  {
     T* v_Val_px = val;
	 const T* const c_ValEnd_px = &val[e_Channels];
     do {
       *(v_Val_px++) += i_V_rx;
     } while (v_Val_px != c_ValEnd_px);
  }

  //! Operator of element-wise matrix addition + assignment
  void operator+=(const Matrix<T, m, n, Mtype>& i_M_rx)
  {
    T* v_Val_px = val;
    const T* const c_ValEnd_px = &val[e_Channels];
    const T* c_MVal_px = i_M_rx.val;
    do {
      *(v_Val_px++) += *(c_MVal_px++);
    } while (v_Val_px != c_ValEnd_px);
  }

  //! Operator matrix addition by scalar
  Matrix<T, m, n, Mtype> operator+(const T& i_V_rx) const
  {
    Matrix<T, m, n, Mtype> v_RetM_x (*this);
    v_RetM_x+=i_V_rx;
    return v_RetM_x;
  }
  
  //! Operator of element-wise matrix addition
  Matrix<T, m, n, Mtype> operator+(const Matrix<T, m, n, Mtype>& i_M_rx) const
  {
     Matrix<T, m, n, Mtype> v_RetM_x (*this);
     v_RetM_x+=i_M_rx;
     return v_RetM_x;
  }

  //! Operator of matrix subtraction by scalar + assignment
  void operator-=(const T& i_V_rx)
  {
     T* v_Val_px = val;
	 const T* const c_ValEnd_px = &val[e_Channels];
     do {
       *(v_Val_px++) -= i_V_rx;
     } while (v_Val_px != c_ValEnd_px);
  }

  //! Operator of element-wise matrix subtraction + assignment
  void operator-=(const Matrix<T, m, n, Mtype>& i_M_rx)
  {
    T* v_Val_px = val;
    const T* const c_ValEnd_px = &val[e_Channels];
    const T* c_MVal_px = i_M_rx.val;
    do {
      *(v_Val_px++) -= *(c_MVal_px++);
    } while (v_Val_px != c_ValEnd_px);
  }

  //! Operator of matrix subtraction by scalar
  Matrix<T, m, n, Mtype> operator-(const T& i_V_rx) const
  {
    Matrix<T, m, n, Mtype> v_RetM_x (*this);
    v_RetM_x-=i_V_rx;
    return v_RetM_x;
  }

  //! Operator of element-wise matrix subtraction
  Matrix<T, m, n, Mtype> operator-(const Matrix<T, m, n, Mtype>& i_M_rx) const
  {
    Matrix<T, m, n, Mtype> v_RetM_x (*this);
    v_RetM_x-=i_M_rx;
    return v_RetM_x;
  }

  //! Unary minus operator
  Matrix<T, m, n, Mtype> operator-() const
  {
    return minus();
  }

  //! Operator of multiplication by scalar + assignment
  void operator*=(const T& i_V_rx)
  {
     T* v_Val_px = val;
	 const T* const c_ValEnd_px = &val[e_Channels];
     do {
       *(v_Val_px++) *= i_V_rx;
     } while (v_Val_px != c_ValEnd_px);
  }

  //! Operator of element-wise matrix multiplication + assignment
  void operator *=(const Matrix<T, m, n, Mtype>& i_M_rx)
  {
    T* v_Val_px = val;
    const T* const c_ValEnd_px = &val[e_Channels];
    const T* c_MVal_px = i_M_rx.val;
    do {
      *(v_Val_px++) *= *(c_MVal_px++);
    } while (v_Val_px != c_ValEnd_px);
  }

  //! Operator of matrix multiplication by scalar
  Matrix<T, m, n, Mtype> operator*(const T& i_V_rx) const
  {
    Matrix<T, m, n, Mtype> v_RetM_x (*this);
    v_RetM_x*=i_V_rx;
    return v_RetM_x;
  }

  //! Operator of element-wise matrix multiplication
  Matrix<T, m, n, Mtype> operator*(const Matrix<T, m, n, Mtype>& i_M_rx) const
  {
    Matrix<T, m, n, Mtype> v_RetM_x (*this);
    v_RetM_x*=i_M_rx;
    return v_RetM_x;
  }

  // PRQA S 4222 ++ // implementation of vector product has no assignment variant
  //! Operator of vector product of matrices
  template<uint32_t m1, uint32_t n1>
  Matrix<T, m, n1> operator%(const Matrix<T, m1, n1>& i_M_rx) const
  {
    return mmul(i_M_rx);
  }
  // PRQA S 4222 --

  //! Operator of matrix division by scalar + assignment
  void operator/=(const T& i_V_rx)
  {
     T* v_Val_px = val;
	 const T* const c_ValEnd_px = &val[e_Channels];
     do {
       *(v_Val_px++) /= i_V_rx;
     } while (v_Val_px != c_ValEnd_px);
  }

  //! Operator of element-wise matrix division + assignment
  void operator/=(const Matrix<T, m, n, Mtype>& i_M_rx)
  {
    T* v_Val_px = val;
    const T* const c_ValEnd_px = &val[e_Channels];
    const T* c_MVal_px = i_M_rx.val;
    do {
      *(v_Val_px++) /= *(c_MVal_px++);
    } while (v_Val_px != c_ValEnd_px);
  }

  //! Operator of element-wise matrix division
  Matrix<T, m, n, Mtype> operator/(const Matrix<T, m, n, Mtype>& i_M_rx) const
  {
    Matrix<T, m, n, Mtype> v_RetM_x (*this);
    v_RetM_x/=i_M_rx;
    return v_RetM_x;
  }

  //! Operator of matrix division by scalar
  Matrix<T, m, n, Mtype> operator/(const T& i_V_rx) const
  {
    Matrix<T, m, n, Mtype> v_RetM_x (*this);
    v_RetM_x/=i_V_rx;
    return v_RetM_x;
  }
  // PRQA S 3361 --

  // keep the old interface for now
  //! Add matrix by scalar value
  Matrix<T, m, n, Mtype> add(const T i_Value_x)                    const { return (*this)+i_Value_x; }

  //! Add two matrices element-wise
  Matrix<T, m, n, Mtype> add(const Matrix<T, m, n, Mtype>& i_M_rx) const { return (*this)+i_M_rx;    }

  //! Subtract matrix by scalar value
  Matrix<T, m, n, Mtype> sub(const T i_Value_x)                    const { return (*this)-i_Value_x; }

  //! Subtract two matrices element-wise
  Matrix<T, m, n, Mtype> sub(const Matrix<T, m, n, Mtype>& i_M_rx) const { return (*this)-i_M_rx;    }

  //! Multiply matrix by scalar value
  Matrix<T, m, n, Mtype> mul(const T i_Value_x)                    const { return (*this)*i_Value_x; }

  //! Multiply two matrices element-wise
  Matrix<T, m, n, Mtype> mul(const Matrix<T, m, n, Mtype>& i_M_rx) const { return (*this)*i_M_rx;    }

  //! Divide matrix by scalar value
  Matrix<T, m, n, Mtype> div(const T i_Value_x)                    const { return (*this)/i_Value_x; }

  //! Divide two matrices element-wise
  Matrix<T, m, n, Mtype> div(const Matrix<T, m, n, Mtype>& i_M_rx) const { return (*this)/i_M_rx;    }

  // -------------------------------
  // structure operators

  //! Operator of joining matrices column-wise
  template<uint32_t n1>
  Matrix<T, m, n+n1> operator|(const Matrix<T, m, n1>& i_M_rx) const
  {
    return this->joinCol(i_M_rx);
  }

  //! Operator of joining matrices row-wise
  template<uint32_t m1>
  Matrix<T, m+m1, n> operator&(const Matrix<T, m1, n>& i_M_rx) const
  {
    return this->joinRow(i_M_rx);
  }

  // -------------------------------
  // Output functions

  //! print matrix in standard format
  void print() const;

  //! print matrix in standard format with heading comments line
  void print(const char* comment) const;

  //! print matrix in special output format
  void print(const MatrixOutputFormat_s& i_OutputFormat_rs) const;

  //! print matrix in special output format with heading comments line
  void print(const MatrixOutputFormat_s& i_OutputFormat_rs, char* comment) const;

  //! print matrix in predefined output format style
  void print(const MatrixOutputFormatStyle_e i_OutputFormatStyle_e) const;

  //! print matrix in predefined output format style with heading comments line
  void print(const MatrixOutputFormatStyle_e i_OutputFormatStyle_e, char* comment) const;

  static uint32_t getMaxRow(void)  { return m; } ;

  static uint32_t getMaxCol(void) { return n; } ;

  static MatrixType_e getMatrixType(void) { return Mtype; }

private:

  // get defined matrix output format
  static MatrixOutputFormat_s getMatrixOutputFormat(MatrixOutputFormatStyle_e i_OutputFormat_e);

  T val[e_Channels];  ///< Matrix elements

};

} // namespace core
} // namespace mecl

#include "Matrix.hpp"


#endif // MECL_CORE_MATRIX_H_
/// @}
/// @}
