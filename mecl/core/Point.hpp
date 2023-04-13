//--------------------------------------------------------------------------
/// @file Point.hpp
/// @brief Implementation of class templates for points
///
/// File contains implementation of Point2D<T>, Point3D<T> and Point4D<T>.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Sebastian Pliefke (sebastian.pliefke@magna.com), Helmut Zollner (helmut.zollner@magna.com)
/// @date Created 09.05.2015
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef MECL_CORE_POINT_HPP_
#define MECL_CORE_POINT_HPP_

#include "Point.h"

namespace mecl
{
namespace core
{

// --------------------------------------------------------------------------
//
// Point2D implementation
//
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
/// @brief Default constructor
/// The default constructor initializes all coordinates to zero.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor1
///
/// @return       Point2D
// --------------------------------------------------------------------------
template<typename T>
Point2D<T>::Point2D(void)
: Vector<T, 2>()
{}

// --------------------------------------------------------------------------
/// @brief Constructor with initialization value
///
/// The constructor initializes all coordinates to the input argument \p i_Value_rx.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor2
///
/// @param[in] i_Value_rx Initialization value for coordinates in Point2D
/// @return Point2D pre-initialized with specific coordinates
// --------------------------------------------------------------------------
template<typename T>
Point2D<T>::Point2D(const T& i_Value_rx)
: Vector<T, 2>(i_Value_rx)
{}

// --------------------------------------------------------------------------
/// @brief Constructor with initialization value
///
/// The constructor initializes all coordinates to the input argument \p i_Value2_rx,
/// \p i_Value2_rx.
///
/// @param[in] i_Value1_rx Initialization value for first coordinate
/// @param[in] i_Value2_rx Initialization value for second coordinate
/// @return Point3D pre-initialized with specific coordinates
// --------------------------------------------------------------------------

template<typename T>
Point2D<T>::Point2D(const T& i_Value1_rx, const T& i_Value2_rx)
{
  (*this)(0) = i_Value1_rx;
  (*this)(1) = i_Value2_rx;
  return;
}

// --------------------------------------------------------------------------
/// @brief Copy constructor (column matrix)
///
/// The constructor copies the contents of \p i_Matrix_rx into this point.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor6
///
/// @param[in]    i_Matrix_rx Matrix (column) object from which to copy contents
/// @return       Point2D pre-initialized with specific coordinates
// --------------------------------------------------------------------------
template<typename T>
Point2D<T>::Point2D(const Matrix<T, 2, 1>& i_Matrix_rx)
: Vector<T, 2>(i_Matrix_rx)
{}

// --------------------------------------------------------------------------
/// @brief Copy constructor (row matrix)
///
/// The constructor copies the contents of \p i_Matrix_rx into this point.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor7
///
/// @param[in]    i_Matrix_rx Matrix (row) object from which to copy contents
/// @return       Point2D pre-initialized with specific coordinates
// --------------------------------------------------------------------------
template<typename T>
Point2D<T>::Point2D(const Matrix<T, 1, 2>& i_Matrix_rx)
: Vector<T, 2>(i_Matrix_rx)
{}

// --------------------------------------------------------------------------
/// @brief Copy constructor with initialization by configuration
///
/// The constructor copies the contents of \p i_Config_rs into coordinates.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor3
///
/// @param[in]    i_Config_rs Configuration to be copied into coordinates in Point2D
/// @return       Point2D pre-initialized with specific coordinate configuration
// --------------------------------------------------------------------------
template<typename T>
Point2D<T>::Point2D(const Config_s& i_Config_rs)
: Vector<T, 2>(i_Config_rs)
{}

// --------------------------------------------------------------------------
/// @brief Copy constructor with initialization by polar configuration
///
/// The constructor uses the contents of \p i_Polar_rs to convert into
/// 2D coordinates.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor5
///
/// @param[in]    i_Polar_rs Polar configuration to be converted into coordinates
/// in Point2D
/// @return       Point2D pre-initialized with specific coordinate configuration
// --------------------------------------------------------------------------
template<typename T>
Point2D<T>::Point2D(const Polar_s& i_Polar_rs)
{
  this->setPolar(i_Polar_rs);
}

// --------------------------------------------------------------------------
/// @brief Set X location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_setPosX
///
/// @param[in]    i_Value_rx New X-coordinate to be set
/// @return       void
// --------------------------------------------------------------------------
template<typename T>
void Point2D<T>::setPosX(const T& i_Value_rx)
{
  (*this)(0) = i_Value_rx;
  return;
}

// --------------------------------------------------------------------------
/// @brief Get X location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_getPosX
///
/// @return       Value of X-coordinate
// --------------------------------------------------------------------------
template<typename T>
T Point2D<T>::getPosX(void) const
{
  return (*this)(0);
}

// --------------------------------------------------------------------------
/// @brief Set Y location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_setPosY
///
/// @param[in]    i_Value_rx New Y-coordinate to be set
/// @return       void
// --------------------------------------------------------------------------
template<typename T>
void Point2D<T>::setPosY(const T& i_Value_rx)
{
  (*this)(1) = i_Value_rx;
  return;
}

// --------------------------------------------------------------------------
/// @brief Get Y location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_getPosY
///
/// @return       Value of Y-coordinate
// --------------------------------------------------------------------------
template<typename T>
T Point2D<T>::getPosY(void) const
{
  return (*this)(1);
}

// --------------------------------------------------------------------------
/// @brief Get polar coordinates
///
/// Calculates and returns polar coordinates for cartesian coordinates.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_getPolar
///
/// @param[in]    i_AngleUnit_e Angle unit, either e_Radians or e_Degrees.
/// Default is e_Radians.
/// @return       Polar coordinates
// --------------------------------------------------------------------------
template<typename T>
typename Point2D<T>::Polar_s Point2D<T>::getPolar(AngleUnit_e i_AngleUnit_e) const
{
  AssertFunction( ! math::isAboutZero_b( (*this)(0) )|| ! math::isAboutZero_b( (*this)(1) ),
      "Tried to get polar cooridnates of euclidean origin.");
  Polar_s v_Polar_s;
  const T c_AngleInRads_x = math::trigonometry<T>::atan2_x((*this)(1), (*this)(0));
  v_Polar_s.angleUnit_e = i_AngleUnit_e;
  v_Polar_s.radius_x = this->norm();
  v_Polar_s.phi_x = i_AngleUnit_e == e_Degrees ? math::toDegrees_x(c_AngleInRads_x) : c_AngleInRads_x;
  return v_Polar_s;
}

// --------------------------------------------------------------------------
/// @brief Set polar coordinates
///
/// Calculates and sets cartesian coordinates based on polar coordinates
/// for current point object.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_setPolar
///
/// @param[in]    i_Polar_rs Polar coordinates
// --------------------------------------------------------------------------
template<typename T>
void Point2D<T>::setPolar(const Polar_s& i_Polar_rs)
{
 const T c_AngleInRads_x  =   i_Polar_rs.angleUnit_e == e_Degrees
                            ? math::toRadians_x(i_Polar_rs.phi_x)
                            : i_Polar_rs.phi_x;
 (*this)(0) = i_Polar_rs.radius_x * math::trigonometry<T>::cos_x(c_AngleInRads_x);
 (*this)(1) = i_Polar_rs.radius_x * math::trigonometry<T>::sin_x(c_AngleInRads_x);
 return;
}

// --------------------------------------------------------------------------
/// @brief Project this vector onto input vector \p i_Pos_rx
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor1
/// @snippet CoreTester.cpp Point2D_Constructor2
/// @snippet CoreTester.cpp Point2D_project
///
/// @param[in] i_Pos_rx 2D Vector to project this vector onto
/// @return 2D vector projected to \p i_Pos_rx
// --------------------------------------------------------------------------
template<typename T>
Point2D<T> Point2D<T>::project(const Point2D<T>& i_Pos_rx)
{
  const T c_Factor_x = this->scalar(i_Pos_rx) / this->norm2();
  return Point2D<T>(i_Pos_rx * c_Factor_x);
}

// --------------------------------------------------------------------------
/// @brief Print polar coordinates (radius, phi) of 2D point
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor1
/// @snippet CoreTester.cpp Point2D_setPolar
/// @snippet CoreTester.cpp Point2D_printPolar_1
///
/// @param[in] i_AngleUnit_e Angle (phi) is in radians (e_Radians) or degrees (e_Degrees)
// --------------------------------------------------------------------------
template<typename T>
void Point2D<T>::printPolar(AngleUnit_e i_AngleUnit_e) const
{
  const Polar_s c_Polar_s = this->getPolar(i_AngleUnit_e);
  if (e_Degrees == i_AngleUnit_e) {
    log_printf("Polar Coordinates: Radius: %f, Angle: %f\370\n",
                c_Polar_s.radius_x, c_Polar_s.phi_x);
  } else {
    log_printf("Polar Coordinates: Radius: %f, Angle: %f rad\n",
                c_Polar_s.radius_x, c_Polar_s.phi_x);
  }
  return;
}

// --------------------------------------------------------------------------
/// @brief Print polar coordinates (radius, phi) of 2D point with comment
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor1
/// @snippet CoreTester.cpp Point2D_setPolar
/// @snippet CoreTester.cpp Point2D_printPolar_2
///
/// @param[in] comment Heading comment string (printed with heading and trailing newline)
/// @param[in] i_AngleUnit_e Angle (phi) is in radians (e_Radians) or degrees (e_Degrees)
// --------------------------------------------------------------------------
template<typename T>
void Point2D<T>::printPolar(char* comment, AngleUnit_e i_AngleUnit_e) const
{
  log_printf("\n%s\n", comment);
  this->printPolar(i_AngleUnit_e);
  return;
}

// --------------------------------------------------------------------------
/// @brief Wedge product of 2D vectors is projection of this vector to rhs vector
///
/// @param[in] i_Rhs_rx 2D Point coordinates of vector to project onto
///
/// @return Projection of this vector onto input vector \p i_Rhs_rx
// --------------------------------------------------------------------------

template<typename T>
Point2D<T> Point2D<T>::operator^ (const Point2D<T>& i_Rhs_rx)
{
  return this->project(i_Rhs_rx);
}

// --------------------------------------------------------------------------
/// @brief Assign 2D point to 2x1 matrix (column matrix)
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor1
/// @snippet CoreTester.cpp Point2D_operator=_1
///
/// @param[in] M matrix that is to be assigned to this point
/// @return Pointer to this point
// --------------------------------------------------------------------------
template<typename T>
Point2D<T> Point2D<T>::operator= (const Matrix<T,2,1>& M)
{
  this->setCol(M);
  return *this;
}

// --------------------------------------------------------------------------
/// @brief Assign 2D point to 1x2 matrix (row matrix)
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor1
/// @snippet CoreTester.cpp Point2D_operator=_2
///
/// @param[in] M matrix that is to be assigned to this point
/// @return Pointer to this point
// --------------------------------------------------------------------------
template<typename T>
Point2D<T> Point2D<T>::operator= (const Matrix<T,1,2>& M)
{
  this->setRow(M);
  return *this;
}

// --------------------------------------------------------------------------
/// @brief Assign to polar coordinates
///
/// Calculates and sets cartesian 2D coordinates based assigned polar coordinates
/// for current point object.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor1
/// @snippet CoreTester.cpp Point2D_operator=_3
///
/// @param[in]    i_Polar_rs Polar coordinates
// --------------------------------------------------------------------------
template<typename T>
Point2D<T> Point2D<T>::operator= (const Polar_s& i_Polar_rs)
{
  this->setPolar(i_Polar_rs);
  return *this;
}

// --------------------------------------------------------------------------
/// @brief Assign to base type value
/// 
/// Sets all coordinates of this point to assigned value
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point2D_Constructor1
/// @snippet CoreTester.cpp Point2D_operator=_4
///
/// @param[in]    v Assigned value
// --------------------------------------------------------------------------
template<typename T>
Point2D<T> Point2D<T>::operator= (const T& v)
{
  (*this) = Vector<T,2>(v);
  return *this;
}

template<typename T>
template<typename T1> inline
Point2D<T>::operator Point2D<T1>() const
{
  return Point2D<T1>( static_cast<Matrix<T1, 2, 1> >(*this) ) ;
}

// --------------------------------------------------------------------------
//
// Point3D implementation
//
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
/// @brief Default constructor

/// The default constructor initializes all coordinates to zero.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor1
///
/// @return       Point3D
// --------------------------------------------------------------------------
template<typename T>
Point3D<T>::Point3D(void)
: Vector<T, 3>()
{}

// --------------------------------------------------------------------------
/// @brief Constructor with initialization value
///
/// The constructor initializes all coordinates to the input argument \p i_Value_rx.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor2
///
/// @param[in] i_Value_rx Initialization value for coordinates in Point3D
/// @return Point3D pre-initialized with specific coordinates
// --------------------------------------------------------------------------
template<typename T>
Point3D<T>::Point3D(const T& i_Value_rx)
: Vector<T, 3>(i_Value_rx)
{}

// --------------------------------------------------------------------------
/// @brief Constructor with initialization value
///
/// The constructor initializes all coordinates to the input argument \p i_Value2_rx,
/// \p i_Value2_rx. \p i_Value3_rx
///
///
/// @param[in] i_Value1_rx Initialization value for first coordinate
/// @param[in] i_Value2_rx Initialization value for second coordinate
/// @param[in] i_Value1_rx Initialization value for third coordinate
/// @return Point3D pre-initialized with specific coordinates
// --------------------------------------------------------------------------

template<typename T>
Point3D<T>::Point3D(const T& i_Value1_rx, const T& i_Value2_rx, const T& i_Value3_rx)
{
  (*this)(0) = i_Value1_rx;
  (*this)(1) = i_Value2_rx;
  (*this)(2) = i_Value3_rx;
  return;
}

// --------------------------------------------------------------------------
/// @brief Copy constructor (column Matrix)
///
/// The constructor copies the contents of \p i_Matrix_rx into this point.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor6
///
/// @param[in]    i_Matrix_rx Vector object from which to copy contents
/// @return       Point3D pre-initialized with specific coordinates
// --------------------------------------------------------------------------
template<typename T>
Point3D<T>::Point3D(const Matrix<T, 3, 1>& i_Matrix_rx)
: Vector<T,3>(i_Matrix_rx)
{}

// --------------------------------------------------------------------------
/// @brief Copy constructor (row Matrix)
///
/// The constructor copies the contents of \p i_Matrix_rx into this point.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor7
///
/// @param[in]    i_Matrix_rx Vector object from which to copy contents
/// @return       Point3D pre-initialized with specific coordinates
// --------------------------------------------------------------------------
template<typename T>
Point3D<T>::Point3D(const Matrix<T, 1, 3>& i_Matrix_rx)
: Vector<T,3>(i_Matrix_rx)
{}

// --------------------------------------------------------------------------
/// Copy constructor with initialization by configuration
///
/// The constructor copies the contents of \p i_Config_rs into coordinates.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor3
///
/// @param[in]    i_Config_rs Configuration to be copied into coordinates in Point3D
/// @return       Point3D pre-initialized with specific coordinate configuration
// --------------------------------------------------------------------------
template<typename T>
Point3D<T>::Point3D(const Config_s& i_Config_rs)
: Vector<T, 3>(i_Config_rs)
{}

// --------------------------------------------------------------------------
  /// @brief Copy constructor with initialization by polar coordinates
  ///
  /// The constructor uses the contents of \p i_Polar_rs to convert into
  /// 3D coordinates.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Point3D_Constructor5
  ///
  /// @param[in]    i_Polar_rs Polar configuration to be converted into coordinates
  /// in Point3D
  /// @return       Point3D pre-initialized with specific coordinate configuration
  // --------------------------------------------------------------------------
template<typename T>
Point3D<T>::Point3D(const Polar_s& i_Polar_rs)
{
  this->setPolar(i_Polar_rs);
}

// --------------------------------------------------------------------------
/// @brief Get X location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_getPosX
///
/// @return       Value of X-coordinate
// --------------------------------------------------------------------------
template<typename T>
T Point3D<T>::getPosX(void) const
{
  return (*this)(0);
}

// --------------------------------------------------------------------------
/// @brief Get Y location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_getPosY
///
/// @return       Value of Y-coordinate
// --------------------------------------------------------------------------
template<typename T>
T Point3D<T>::getPosY(void) const
{
  return (*this)(1);
}

// --------------------------------------------------------------------------
/// @brief Get Z location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_getPosZ
///
/// @return       Value of Z-coordinate
// --------------------------------------------------------------------------
template<typename T>
T Point3D<T>::getPosZ(void) const
{
  return (*this)(2);
}

// --------------------------------------------------------------------------
/// @brief Get polar coordinates of this euclidean 3D (projective 2D) point
///
/// Function returns struct defining polar coordinates of this point
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_getPolar
///
/// @param[in] i_AngleUnit_e Angles phi and theta are in radians (e_Radians) or degrees (e_Degrees)
///
/// @return struct Polar_s contains radius, phi, theta (elevation) and angle unit
// --------------------------------------------------------------------------
template<typename T>
typename Point3D<T>::Polar_s Point3D<T>::getPolar(AngleUnit_e i_AngleUnit_e) const
{

  Polar_s v_Polar_s = { 0, 0, 0, i_AngleUnit_e};

  v_Polar_s.radius_x = this->norm();
  AssertFunction( false == math::isZero_b(v_Polar_s.radius_x), "Polar coordinates of origin is undefined.");

  v_Polar_s.phi_x = math::trigonometry<T>::atan2_x((*this)(1), (*this)(0));
  v_Polar_s.theta_x = math::trigonometry<T>::acos_x( (*this)(2) / v_Polar_s.radius_x ) ;

  if (e_Degrees == i_AngleUnit_e) {
    v_Polar_s.phi_x = math::toDegrees_x(v_Polar_s.phi_x);
    v_Polar_s.theta_x = math::toDegrees_x(v_Polar_s.theta_x);
  }

  return v_Polar_s;
}

// --------------------------------------------------------------------------
/// @brief Get elevation angle of given 3D (2D projective) coordinate
///
/// Remark: elevation angle is theta angle of 3D polar coordinates
///
/// param[in] i_AngleUnit_e Angle is in radians (e_Radians) or degrees (e_Degrees)
///
/// @return Elevation angle
// --------------------------------------------------------------------------
template<typename T>
T Point3D<T>::getElevation(AngleUnit_e i_AngleUnit_e) const
{
  const T c_Radius_x = this->norm();
  const T c_Elevation_x =   math::isZero_b(c_Radius_x)
                          ? math::constants<T>::zero_x()
                          : math::trigonometry<T>::acos_x( (*this)(2) / c_Radius_x);

  return e_Degrees == i_AngleUnit_e ? math::toDegrees_x(c_Elevation_x) : c_Elevation_x;
}

// --------------------------------------------------------------------------
/// @brief Get elevation angle of point projected to xz-plane (y=0)
///
/// param[in] i_AngleUnit_e Angle is in radians (e_Radians) or degrees (e_Degrees)
///
/// @return Elevation angle
// --------------------------------------------------------------------------
template<typename T>
T Point3D<T>::getElevationHorizontal(AngleUnit_e i_AngleUnit_e) const
{
  const T c_Radius_x = Point2D<T>( this->getPosX(), this->getPosZ() ).norm();
  const T c_Elevation_x =    math::isZero_b(c_Radius_x)
                           ? math::constants<T>::zero_x()
                           : math::trigonometry<T>::acos_x( this->getPosZ() / c_Radius_x);
  return e_Degrees == i_AngleUnit_e ? math::toDegrees_x(c_Elevation_x) : c_Elevation_x;
}


// --------------------------------------------------------------------------
/// @brief Get elevation angle of point projected to yz-plane (x=0)
///
/// param[in] i_AngleUnit_e Angle is in radians (e_Radians) or degrees (e_Degrees)
///
/// @return Elevation angle
// --------------------------------------------------------------------------
template<typename T>
T Point3D<T>::getElevationVertical(AngleUnit_e i_AngleUnit_e) const
{
  const T c_Radius_x = Point2D<T>( this->getPosY(), this->getPosZ() ).norm();
  const T c_Elevation_x =    math::isZero_b(c_Radius_x)
                           ? math::constants<T>::zero_x()
                           : math::trigonometry<T>::acos_x( this->getPosZ() / c_Radius_x);
  return e_Degrees == i_AngleUnit_e ? math::toDegrees_x(c_Elevation_x) : c_Elevation_x;
}

// -------------------------------------------------------------------------
/// @brief Get point coordinates of this point normalized to cube
///
/// Get point coordinates of this point normalized to cube of side length 2 * \p i_Factor_x
///
/// @param[in] i_Factor_x
///
/// @return Point with Cube coordinates
template<typename T>
Point3D<T> Point3D<T>::getCubeCoords_x(const T i_Factor_x) const
{
  return Point3D<T>( ( (*this) / this->abs().maximum() ) * i_Factor_x ) ;
}

template<typename T>
Point3D<T> Point3D<T>::getCubeCoords_x(uint32_t& o_NormDim_ru32, const T i_Factor_x) const
{
  return Point3D<T>( ( (*this) / this->abs().maximum(o_NormDim_ru32) ) * i_Factor_x ) ;
}

// -------------------------------------------------------------------------
/// @brief Normalize this point to cube
///
/// Normalize this point to cube of side length 2 * \p i_Factor_x
///
/// @param[in] i_Factor_x
///
/// @return void
template<typename T>
void Point3D<T>::normalizeToCube_v(const T i_Factor_x)
{
  *this /= this->abs().maximum() ;
  *this *= i_Factor_x;
  return;
}

// -------------------------------------------------------------------------
// --------------------------------------------------------------------------
/// @brief Set X location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_setPosX
///
/// @param[in]    i_Value_rx New X-coordinate to be set
/// @return       void
// --------------------------------------------------------------------------
template<typename T>
void Point3D<T>::setPosX(const T& i_Value_rx)
{
  (*this)(0) = i_Value_rx;
  return;
}

// --------------------------------------------------------------------------
/// @brief Set Y location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_setPosY
///
/// @param[in]    i_Value_rx New Y-coordinate to be set
/// @return       void
// --------------------------------------------------------------------------
template<typename T>
void Point3D<T>::setPosY(const T& i_Value_rx)
{
  (*this)(1) = i_Value_rx;
  return;
}

// --------------------------------------------------------------------------
/// @brief Set Z location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_setPosZ
///
/// @param[in]    i_Value_rx New Z-coordinate to be set
/// @return       void
// --------------------------------------------------------------------------
template<typename T>
void Point3D<T>::setPosZ(const T& i_Value_rx)
{
  (*this)(2) = i_Value_rx;
  return;
}

// --------------------------------------------------------------------------
/// @brief Set polar coordinates
///
/// Calculates and sets cartesian 3D coordinates based on polar coordinates
/// for current point object.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_setPolar
///
/// @param[in] i_Polar_rs Given polar coordinates given Polar_s POD (radius, phi, theta, AngleUnit_e)
// --------------------------------------------------------------------------
template<typename T>
void Point3D<T>::setPolar(const Polar_s& i_Polar_rs)
{
  const T c_SinPhi_x =   e_Degrees == i_Polar_rs.angleUnit_e
                       ? math::trigonometry<T>::sin_x( math::toRadians_x(i_Polar_rs.phi_x) )
                       : math::trigonometry<T>::sin_x( i_Polar_rs.phi_x);
  const T c_CosPhi_x =   e_Degrees == i_Polar_rs.angleUnit_e
                       ? math::trigonometry<T>::cos_x( math::toRadians_x(i_Polar_rs.phi_x) )
                       : math::trigonometry<T>::cos_x( i_Polar_rs.phi_x);

  const T c_SinTheta_x =   e_Degrees == i_Polar_rs.angleUnit_e
                         ? math::trigonometry<T>::sin_x( math::toRadians_x(i_Polar_rs.theta_x) )
                         : math::trigonometry<T>::sin_x( i_Polar_rs.theta_x);
  const T c_CosTheta_x =   e_Degrees == i_Polar_rs.angleUnit_e
                         ? math::trigonometry<T>::cos_x( math::toRadians_x(i_Polar_rs.theta_x) )
                         : math::trigonometry<T>::cos_x( i_Polar_rs.theta_x);

  (*this)(0) = i_Polar_rs.radius_x * c_SinTheta_x * c_CosPhi_x;
  (*this)(1) = i_Polar_rs.radius_x * c_SinTheta_x * c_SinPhi_x;
  (*this)(2) = i_Polar_rs.radius_x * c_CosTheta_x;
}

// --------------------------------------------------------------------------
/// @brief Calculate cross (wedge) product of two 3D vectors
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor2
/// @snippet CoreTester.cpp Point3D_Constructor3
/// @snippet CoreTester.cpp Point3D_cross
/// 
/// @param[in] i_Pos_x Rhs operand (lhs operand is this vector)
/// @return Point3D representing cross (wedge) product of this point and i_Pos_x
// --------------------------------------------------------------------------
template<typename T>
const Point3D<T> Point3D<T>::cross(const Vector<T,3>& i_Pos_x) const
{
  Point3D<T> c_Pos_x;

  c_Pos_x(0) = (*this)(1) * i_Pos_x(2) - (*this)(2) * i_Pos_x(1);
  c_Pos_x(1) = (*this)(2) * i_Pos_x(0) - (*this)(0) * i_Pos_x(2);
  c_Pos_x(2) = (*this)(0) * i_Pos_x(1) - (*this)(1) * i_Pos_x(0);

  return c_Pos_x;
}

// --------------------------------------------------------------------------
/// @brief Print polar coordinates (radius, phi, theta) of 3D point
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor5
/// @snippet CoreTester.cpp Point3D_printPolar_1
///
/// @param[in] i_AngleUnit_e Angle (phi and theta are in radians (e_Radians) or degrees (e_Degrees)
// --------------------------------------------------------------------------
template<typename T>
void Point3D<T>::printPolar(AngleUnit_e i_AngleUnit_e) const
{
  const Polar_s c_Polar_s = this->getPolar(i_AngleUnit_e);
  if (e_Degrees == i_AngleUnit_e) {
    log_printf("Polar Coordinates: Radius: %f, Phi: %f\370, Theta: %f\171\n",
                c_Polar_s.radius_x, c_Polar_s.phi_x, c_Polar_s.theta_x);
  } else {
    log_printf("Polar Coordinates: Radius: %f, Phi: %f rad, Theta: %f rad\n",
                c_Polar_s.radius_x, c_Polar_s.phi_x, c_Polar_s.theta_x);
  }
  return;
}

// --------------------------------------------------------------------------
/// @brief Print polar coordinates (radius, phi, theta) of 3D point with comment
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor5
/// @snippet CoreTester.cpp Point3D_printPolar_2
///
/// @param[in] comment Heading comment string (printed with heading and trailing newline)
/// @param[in] i_AngleUnit_e Angle phi and theta are in radians (e_Radians) or degrees (e_Degrees)
// --------------------------------------------------------------------------
template<typename T>
void Point3D<T>::printPolar(char * comment, AngleUnit_e i_AngleUnit_e) const
{
  log_printf("\n%s\n", comment);
  this->printPolar(i_AngleUnit_e);
  return;
}

// --------------------------------------------------------------------------
/// @brief Operator of cross (wedge) product of two 3D vectors
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor2
/// @snippet CoreTester.cpp Point3D_Constructor3
/// @snippet CoreTester.cpp Point3D_operator^
///
/// @param[in] i_Pos_x Rhs operand (lhs operand is this vector)
/// @return Point3D representing cross (wedge) product of this point and i_Pos_x
// --------------------------------------------------------------------------
template<typename T>
const Point3D<T> Point3D<T>::operator^ (const Vector<T,3>& i_Pos_x) const
{
  return cross(i_Pos_x);
}

// --------------------------------------------------------------------------
/// @brief Assign point vector to column matrix
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor1
/// @snippet CoreTester.cpp Point3D_operator=_1
///
/// @param[in]    i_Matrix_rx New coordinates as 3x1 matrix
/// @return       void
//  --------------------------------------------------------------------------
template<typename T>
Point3D<T> Point3D<T>::operator= (const Matrix<T,3,1>& i_Matrix_rx)
{
  this->setCol(i_Matrix_rx);
  return *this;
}

// --------------------------------------------------------------------------
/// @brief Assign point vector to row matrix
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor1
/// @snippet CoreTester.cpp Point3D_operator=_2
///
/// @param[in]    i_Matrix_rx New coordinates as 1x3 matrix
/// @return       void
//  --------------------------------------------------------------------------
template<typename T>
Point3D<T> Point3D<T>::operator= (const Matrix<T,1,3>& i_Matrix_rx)
{
  this->setRow(i_Matrix_rx);
  return *this;
}

// --------------------------------------------------------------------------
/// @brief Assign to polar coordinates
///
/// Calculates and sets cartesian 3D coordinates based assigned polar coordinates
/// for current point object.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor3
/// @snippet CoreTester.cpp Point3D_operator=_1
///
/// @param[in]    i_Polar_rs Polar coordinates
// --------------------------------------------------------------------------
template<typename T>
Point3D<T> Point3D<T>::operator= (const Polar_s& i_Polar_rs)
{
  this->setPolar(i_Polar_rs);
  return *this;
}

// --------------------------------------------------------------------------
/// @brief Assign to base type value
/// Sets all coordinates of this point to assigned value
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point3D_Constructor1
/// @snippet CoreTester.cpp Point3D_operator=_4
///
/// @param[in]    v Assigned value
// --------------------------------------------------------------------------
template<typename T>
Point3D<T> Point3D<T>::operator= (const T& v)
{
   (*this) = Vector<T,3>(v);
   return *this;
}

template<typename T>
template<typename T1>
Point3D<T>::operator Point3D<T1>() const
{
  return Point3D<T1>( static_cast<Matrix<T1, 3, 1> >(*this) ) ;
}

// --------------------------------------------------------------------------
//
// Point4D implementation
//
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
/// @brief Default constructor
/// The default constructor initializes all coordinates to zero.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_Constructor1
///
/// @return       Point4D
// --------------------------------------------------------------------------
template<typename T>
Point4D<T>::Point4D(void)
: Vector<T, 4>()
{}

// --------------------------------------------------------------------------
/// @brief Constructor with initialization value
///
/// The constructor initializes all coordinates to the input argument \p i_Value_rx.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_Constructor2
///
/// @param[in] i_Value_rx Initialization value for coordinates in Point4D
/// @return Point4D pre-initialized with specific coordinates
// --------------------------------------------------------------------------
template<typename T>
Point4D<T>::Point4D(const T& i_Value_rx)
: Vector<T, 4>(i_Value_rx)
{}

// --------------------------------------------------------------------------
/// @brief Constructor with initialization value
///
/// The constructor initializes all coordinates to the input argument \p i_Value2_rx,
/// \p i_Value2_rx. \p i_Value3_rx, \p i_Value4_rx
///
///
/// @param[in] i_Value1_rx Initialization value for first coordinate
/// @param[in] i_Value2_rx Initialization value for second coordinate
/// @param[in] i_Value1_rx Initialization value for third coordinate
/// @param[in] i_Value2_rx Initialization value for forth coordinate
/// @return Point4D pre-initialized with specific coordinates
// --------------------------------------------------------------------------
template<typename T>
Point4D<T>::Point4D(const T& i_Value1_rx, const T& i_Value2_rx, const T& i_Value3_rx, const T& i_Value4_rx)
{
  (*this)(0) = i_Value1_rx;
  (*this)(1) = i_Value2_rx;
  (*this)(2) = i_Value3_rx;
  (*this)(3) = i_Value4_rx;
  return;
}

// --------------------------------------------------------------------------
/// @brief Copy constructor (row matrix)
///
/// The constructor copies the contents of \p i_Matrix_rx into this point.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_Constructor5
///
/// @param[in]    i_Matrix_rx Matrix object from which to copy contents
/// @return       Point4D pre-initialized with specific coordinates
// --------------------------------------------------------------------------
template<typename T>
Point4D<T>::Point4D(const Matrix<T, 4, 1>& i_Matrix_rx)
: Vector<T, 4>(i_Matrix_rx)
{}

// --------------------------------------------------------------------------
/// @brief Copy constructor (column matrix)
///
/// The constructor copies the contents of \p i_Point_rx into this point.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_Constructor6
///
/// @param[in]    i_Matrix_rx Matrix object from which to copy contents
/// @return       Point4D pre-initialized with specific coordinates
// --------------------------------------------------------------------------
template<typename T>
Point4D<T>::Point4D(const Matrix<T, 1, 4>& i_Matrix_rx)
: Vector<T, 4>(i_Matrix_rx)
{}

// --------------------------------------------------------------------------
/// @brief Copy constructor with initialization by configuration
///
/// The constructor copies the contents of \p i_Config_rx into coordinates.
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_Constructor3
///
/// @param[in]    i_Config_rs Configuration to be copied into coordinates in Point4D
/// @return       Point4D pre-initialized with specific coordinate configuration
// --------------------------------------------------------------------------
template<typename T>
Point4D<T>::Point4D(const Config_s& i_Config_rs)
: Vector<T, 4>(i_Config_rs)
{}

// --------------------------------------------------------------------------
/// @brief Get X location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_getPosX
///
/// @return       Value of X-coordinate
// --------------------------------------------------------------------------
template<typename T>
T Point4D<T>::getPosX(void) const
{
  return  (*this)(0) ;
}

// --------------------------------------------------------------------------
/// @brief Get Y location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_getPosY
///
/// @return       Value of Y-coordinate
// --------------------------------------------------------------------------
template<typename T>
T Point4D<T>::getPosY(void) const
{
  return  (*this)(1) ;
}

// --------------------------------------------------------------------------
/// @brief Get Z location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_getPosZ
///
/// @return       Value of Z-coordinate
// --------------------------------------------------------------------------
template<typename T>
T Point4D<T>::getPosZ(void) const
{
  return (*this)(2);
}

// --------------------------------------------------------------------------
/// @brief Set X location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_setPosX
///
/// @param[in]    i_Value_rx New X-coordinate to be set
/// @return       void
// --------------------------------------------------------------------------
template<typename T>
void Point4D<T>::setPosX(const T& i_Value_rx)
{
  (*this)(0)  = i_Value_rx;
  return;
}

// --------------------------------------------------------------------------
/// @brief Set Y location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_setPosY
///
/// @param[in]    i_Value_rx New Y-coordinate to be set
/// @return       void
// --------------------------------------------------------------------------
template<typename T>
void Point4D<T>::setPosY(const T& i_Value_rx)
{
  (*this)(1)  = i_Value_rx;
  return;
}

// --------------------------------------------------------------------------
/// @brief Set Z location of point vector
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_setPosZ
///
/// @param[in]    i_Value_rx New Z-coordinate to be set
/// @return       void
// --------------------------------------------------------------------------
template<typename T>
void Point4D<T>::setPosZ(const T& i_Value_rx)
{
  (*this)(2)  = i_Value_rx;
  return;
}

// --------------------------------------------------------------------------
/// @brief Assign point vector to column matrix
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_Constructor1
/// @snippet CoreTester.cpp Point4D_operator=_1
///
/// @param[in]    M New coordinates as 4x1 matrix
/// @return       void
//  --------------------------------------------------------------------------
template<typename T>
Point4D<T> Point4D<T>::operator= (const Matrix<T,4,1>& M)
{
  this->setCol(M);
  return *this;
}

// --------------------------------------------------------------------------
/// @brief Assign point vector to row matrix
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_Constructor1
/// @snippet CoreTester.cpp Point4D_operator=_2
///
/// @param[in]    M New coordinates as 4x1 matrix
/// @return       void
//   --------------------------------------------------------------------------
template<typename T>
Point4D<T> Point4D<T>::operator= (const Matrix<T,1,4>& M)
{
  this->setRow(M);
  return *this;
}

// --------------------------------------------------------------------------
/// @brief Assign to base type value
/// Sets all coordinates of this point to assigned value
///
/// @par Example usage:
/// @snippet CoreTester.cpp Point4D_Constructor1
/// @snippet CoreTester.cpp Point4D_operator=_3
///
/// @param[in]    v Assigned value
// --------------------------------------------------------------------------
template<typename T>
Point4D<T> Point4D<T>::operator= (const T& v)
{
   (*this) = Vector<T,4>(v);
   return *this;
}

template<typename T>
template<typename T1> inline
Point4D<T>::operator Point4D<T1>() const
{
  return Point4D<T1>( static_cast<Matrix<T1, 4, 1> >(*this) ) ;
}

} // namespace core
} // namespace mecl

#endif // MECL_CORE_POINT_HPP_
/// @}
/// @}
