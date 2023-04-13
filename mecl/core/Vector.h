// --------------------------------------------------------------------------
/// @file Vector.h
/// @brief This is the short description of the template module
///
/// Here may follow a longer description for the template module with examples.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Sebastian Pliefke (sebastian.pliefke@magna.com), Helmut Zollner (helmut.zollner@magna.com)
///
// --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef MECL_CORE_VECTOR_H_
#define MECL_CORE_VECTOR_H_

#include "Matrix.h"

namespace mecl
{
namespace core
{


// --------------------------------------------------------------------------
/// @class Vector
/// @brief Basic vector data type of Magna Electronics Library
///
/// The vector class inherits interface of Matrix class and is
/// defined as a single column matrix of n rows.
// --------------------------------------------------------------------------
template<typename T, uint32_t n>
class Vector : public Matrix<T, n, 1>
{
public:

  typedef typename Matrix<T, n, 1>::Config_s Config_s;  ///< Configuration data set
  
  // --------------------------------------------------------------------------
  /// @brief Default constructor
  ///
  /// Creates empty vector object.
  /// 
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor3
  // --------------------------------------------------------------------------
  Vector(void)
  : Matrix<T, n, 1>()
  {}

  // --------------------------------------------------------------------------
  /// @brief Constructor for scalar value assignment
  ///
  /// Creates vector object and assigns all scalar value to all cell elements.
  /// 
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor4
  ///
  /// @param[in]  i_Value_px Scalar value to be assigned to all elements
  // --------------------------------------------------------------------------
  explicit Vector(const T& i_Value_px)
  : Matrix<T, n, 1>(i_Value_px)
  {}

  // --------------------------------------------------------------------------
  /// @brief Copy constructor with initialization by configuration
  ///
  /// The constructor copies the contents of \p i_Config_px into cell elements.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Vector_Constructor5
  ///
  /// @param[in] i_Config_px Configuration to be copied into cell elements
  // --------------------------------------------------------------------------
  explicit Vector(const Config_s& i_Config_px)
  : Matrix<T, n, 1>(i_Config_px)
  {}

  // --------------------------------------------------------------------------
  /// @brief Constructor for column matrix
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor1
  ///
  /// @param[in] M Column matrix
  // --------------------------------------------------------------------------
  explicit Vector(const Matrix<T, n, 1>& M)
  : Matrix<T, n, 1>(M)
  {}

  // --------------------------------------------------------------------------
  /// @brief Constructor for row matrix
  ///
  /// Vectors are always treated as column matrices. Input 1 x n row matrix is 
  /// transposed into n x 1 column matrix.
  /// 
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor2
  ///
  /// @param[in] M Row matrix
  // --------------------------------------------------------------------------
  explicit Vector(const Matrix<T, 1, n>& M)
  : Matrix<T, n, 1>(M.t())
  {}

  // --------------------------------------------------------------------------
  /// @brief Destructor
  // --------------------------------------------------------------------------
  virtual ~Vector()
  {}

  // --------------------------------------------------------------------------
  /// @brief Get last (homogeneous) coordinate
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor1
  /// @snippet CoreTester.cpp Vector_getW
  ///
  /// @return Last (homogeneous) coordinate
  // --------------------------------------------------------------------------
  T getW() const
  {
    return (*this)(n-1);
  }

  // --------------------------------------------------------------------------
  /// @brief Get subvector of this vector
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor5
  /// @snippet CoreTester.cpp Vector_subVector
  ///
  /// @param[in] i Row (index) number at which to start extraction of cell elements
  /// @return Vector<T,n1>, subvector of size n1 starting at in i
  // --------------------------------------------------------------------------
  template <uint32_t n1>
  const Vector<T, n1> subVector(const uint32_t i = 0) const
  {
    StaticAssert(n1 <= n, "core::Vector<n1) subVector(uint32_t i): Subvector size is bigger than source matrix.");
    AssertFunction(i <= (n-n1), "Subvector index out of bounds.");
    return static_cast<Vector<T, n1> >(core::Matrix<T, n, 1>::template subMatrix<n1,1>(i,0));
  };

  // --------------------------------------------------------------------------
  /// @brief Set last (homogeneous) coordinate
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor1
  /// @snippet CoreTester.cpp Vector_setW
  ///
  /// @param[in] w Last (homogeneous) coordinate
  // --------------------------------------------------------------------------
  void setW(const T& w)
  {
    (*this)(n-1) = w;
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Get normalized vector
  ///
  /// Return normalized vector such that last coordinate is \p i_Factor. Default
  /// \p i_Factor is 1.0f if ommitted.
  /// 
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor5
  /// @snippet CoreTester.cpp Vector_getNormalized
  ///
  /// @param[in] i_Factor_x Normalization factor
  /// @return Copy of normalized vector
  // --------------------------------------------------------------------------
  const Vector getNormalized(const T& i_Factor_x = math::constants<T>::one_x()) const
  {
    const T w = this->getW() * i_Factor_x;
    AssertFunction(math::numeric_limits<T>::epsilon_x() <= math::abs_x(w),
                   "Homogeneous coordinate is 0 (point at infinity?)");
    return Vector(this->div(w));
  }

  // --------------------------------------------------------------------------
  /// @brief Normalize this vector
  ///
  /// Normalize this vector such that last coordinate is \p i_Factor. Default
  /// \p i_Factor is 1.0f if ommitted.
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor5
  /// @snippet CoreTester.cpp Vector_normalize
  ///
  /// @param[in] i_Factor_x Normalization factor
  // --------------------------------------------------------------------------
  void normalize(const T& i_Factor_x = math::constants<T>::one_x())
  {
    const T w = this->getW() * i_Factor_x;
    AssertFunction(math::numeric_limits<T>::epsilon_x() <= math::abs_x(w),
              "Homogeneous coordinate is 0 (point at infinity?)");
    *this = this->div(w);
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Get vector with absolute element values of this vector
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor5
  /// @snippet CoreTester.cpp Vector_abs
  ///
  /// @return Vector with absolute element values of this vector
  // --------------------------------------------------------------------------
  Vector abs() const
  {
    Vector<T,n> v_Vec_x;
    for (uint32_t i=0; i<n; i++)
    {
      v_Vec_x(i) = math::abs_x( (*this)(i) );
    }
    return v_Vec_x;
  }

  // --------------------------------------------------------------------------
  /// @brief Explicit definition of the dot (inner) product for vectors
  ///
  /// This function is an optimization of u.t() % v for u and v being vectors
  /// of same dimensions, since no explicit transposition of u is needed.
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor5
  /// @snippet CoreTester.cpp Vector_dot
  ///
  /// @param[in] i_Vector_rx rhs operand of dot product (this is lhs)
  /// @return Scalar value of inner product
  // --------------------------------------------------------------------------
  T dot(const Vector<T,n>& i_Vector_rx) const
  {
    return this->scalar(i_Vector_rx);
  }

  // --------------------------------------------------------------------------
  /// @brief Explicit definition of the dyadic (outer) product for vectors
  ///
  /// This function is an optimization of u % v.t() for u and v being vectors,
  /// since no explicit transposition of v is needed.
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor5
  /// @snippet CoreTester.cpp Vector_dyadic
  ///
  /// @param[in] i_Vector_rx rhs operand of dot product (this is lhs)
  /// @return Matrix of outer product
  // --------------------------------------------------------------------------
  template<uint32_t n1>
  Matrix<T, n, n1> dyadic(const Vector<T, n1>& i_Vector_rx) const
  {

    Matrix<T, n, n1> v_Matrix_x;

    for (uint32_t i=0; i<n; i++)
    {
      if ( not (math::isZero_b( (*this)(i) ) ) )
      {
        for (uint32_t j=0; j<n1; j++)
        {
          v_Matrix_x(i,j) = (*this)(i) * i_Vector_rx(j);
        }
      }
    }
    return v_Matrix_x;
  }

  // --------------------------------------------------------------------------
  /// @brief Calculate Householder transformation matrix of vector
  ///
  /// The resulting matrix describes a reflection about a (hyper)plane
  /// containing the origin where the plane's normal vector is this vector
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor5
  /// @snippet CoreTester.cpp Vector_calcHouseholder
  ///
  /// @return Householder transformation matrix of this vector
  // --------------------------------------------------------------------------
  Matrix<T, n, n> calcHouseholder(void) const
  {
    Matrix<T, n, n> v_Matrix_x(Matrix<T, n, n>::eye_x());
    const T c_Scalar_x = 2.0F / this->norm2();
    v_Matrix_x -= ( this->dyadic( (*this) ).mul(c_Scalar_x ));
    return v_Matrix_x;
  }

  // --------------------------------------------------------------------------
  /// @brief Set this vector from row matrix
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor1
  /// @snippet CoreTester.cpp Vector_setRow
  ///
  /// @param[in] i_Matrix_rx Row matrix from which values are copied
  // --------------------------------------------------------------------------
  void setRow(const Matrix<T,1,n>& i_Matrix_rx)
  {
    *this = Vector(i_Matrix_rx.t());
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Set this vector from row matrix
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor1
  /// @snippet CoreTester.cpp Vector_setCol
  ///
  /// @param[in] i_Matrix_rx Column matrix from which values are copied
  // --------------------------------------------------------------------------
  void setCol(const Matrix<T,n,1>& i_Matrix_rx)
  {
    *this = Vector(i_Matrix_rx);
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Assign row matrix to this vector
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor1
  /// @snippet CoreTester.cpp Vector_operator=_1
  ///
  /// @return Vector with transposed contents of input matrix argument.
  // --------------------------------------------------------------------------
  Vector operator= (const Matrix<T,1,n>& M)
  {
    this->setRow(M);
    return *this;
  }

  // --------------------------------------------------------------------------
  /// @brief Assign column matrix to this vector
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor1
  /// @snippet CoreTester.cpp Vector_operator=_2
  ///
  /// @return Vector with transposed contents of input matrix argument.
  // --------------------------------------------------------------------------
  Vector operator= (const Matrix<T,n,1>& M)
  {
    this->setCol(M);
    return *this;
  }

  // --------------------------------------------------------------------------
  /// @brief Assign to base type value
  /// Sets all coordinates of this point to assigned value
  ///
  /// @par Example use:
  /// @snippet CoreTester.cpp Vector_Constructor1
  /// @snippet CoreTester.cpp Vector_operator=_3
  ///
  /// @param[in]    v Assigned value
  // --------------------------------------------------------------------------
  Vector operator= (const T& v)
  {
    this->setCol(Matrix<T, n, 1>(v));
    return *this;
  }

};

} // namespace core
} // namespace mecl

#endif // MECL_CORE_VECTOR_H_
/// @}
/// @}
