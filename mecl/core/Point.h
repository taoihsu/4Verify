//--------------------------------------------------------------------------
/// @file Point.h
/// @brief Definition of class templates for points
///
/// File contains definition of Point2D<T>, Point3D<T> and Point4D<T>.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Sebastian Pliefke (sebastian.pliefke@magna.com), Helmut Zollner (helmut.zollner@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef MECL_CORE_POINT_H_
#define MECL_CORE_POINT_H_

#include "Vector.h"

namespace mecl
{
namespace core
{

//--------------------------------------------------------------------------
/// @class Point2D
/// @brief Storage class of a 2 dimensional point location
// --------------------------------------------------------------------------

template<typename T>
class Point2D : public Vector<T, 2>
{

public:

  typedef typename Matrix<T, 2, 1>::Config_s Config_s;  ///< Configuration data set

  /// POD struct for representation in polar coordinates
  struct Polar_s
  {
    T radius_x;               //< distance to origin
    T phi_x;                  //< angle to between vector and x-axis
    AngleUnit_e angleUnit_e;  //< defines units of angle (radians, degrees)
  };

  // --------------------------------------------------------------------------
  // constructors + destructor

  /// Default constructor
  Point2D(void);

  /// Constructor with initialization value of base type
  explicit Point2D(const T& i_Value_rx);

  /// Constructor with point coordinates
  explicit Point2D(const T& i_Value1_rx, const T& i_Value2_rx);

  /// Copy constructor (column matrix)
  Point2D(const Matrix<T, 2, 1>& i_Matrix_rx);

  /// Copy constructor  (row matrix)
  Point2D(const Matrix<T, 1, 2>& i_Matrix_rx);

  /// Copy constructor with initialization by configuration
  explicit Point2D(const Config_s& i_Config_rs);

  /// Copy constructor with initialization by polar coordinates
  explicit Point2D(const Polar_s& i_Polar_rs);

  /// Virtual destructor
  virtual ~Point2D(void) {};

  // --------------------------------------------------------------------------
  // getters

  /// Get X location of point vector
  T getPosX(void) const;

  /// Get Y location of point vector
  T getPosY(void) const;

  /// Get polar coordinates
  Polar_s getPolar(AngleUnit_e i_AngleUnit_e = e_Radians) const;

  // --------------------------------------------------------------------------
  // setters

  /// Set X location of point vector
  void setPosX(const T& i_Value_rx);

  /// Set Y location of point vector
  void setPosY(const T& i_Value_rx);

  /// Set polar coordinates
  void setPolar(const Polar_s& i_Polar_rs);

  // --------------------------------------------------------------------------
  // math operations

  /// Project this vector onto input vector
  Point2D project(const Point2D& i_Pos_rx);

  // --------------------------------------------------------------------------
  // output functions

  /// Print polar coordinates (radius, phi) of 2D point
  void printPolar(AngleUnit_e i_AngleUnit_e = e_Radians) const;

  /// Print polar coordinates (radius, phi) of 2D point with comment
  void printPolar(char* comment, AngleUnit_e i_AngleUnit_e = e_Radians) const;

  // --------------------------------------------------------------------------
  // additional arithmetic operators

  /// Wedge product of 2D vectors is projection of lhs vector to rhs vector
  Point2D operator^ (const Point2D& i_Rhs_rx);

  // --------------------------------------------------------------------------
  // additional assignment operators

  /// Assign 2D point to 2x1 matrix (column matrix)
  Point2D operator= (const Matrix<T,2,1>& M);

  /// Assign 2D point to 1x2 matrix (row matrix)
  Point2D operator= (const Matrix<T,1,2>& M);

  /// Assign to polar coordinates
  Point2D operator= (const Polar_s& p);

  /// Assign to base type value
  Point2D operator= (const T& v);

  // --------------------------------------------------------------------------
  // additional cast operators

  /// Cast to 2D Point of different base type
  template<typename T1>
  operator Point2D<T1>() const;

};


//--------------------------------------------------------------------------
/// @class Point3D
/// @brief Storage class of a 3 dimensional point location
//--------------------------------------------------------------------------
template<typename T> class Point3D : public Vector<T, 3>
{

public:

  typedef typename Vector<T, 3>::Config_s Config_s;  ///< Configuration data set

  /// POD struct for transformation to polar (spherical) coordinates
  struct Polar_s
  {
    T radius_x;               //< radius
    T phi_x;                  //< angle between x axis and vector projected to xy plane
    T theta_x;                //< angle between vector and z-axis
    AngleUnit_e angleUnit_e;  //< angle is in radians or degrees
  };

  // --------------------------------------------------------------------------
  // constructors + destructor

  /// Default constructor
  Point3D(void);

  /// Constructor with initialization value
  explicit Point3D(const T& i_Value_rx);

  /// Constructor with point coordinates
  explicit Point3D(const T& i_Value1_rx, const T& i_Value2_rx, const T& i_Value3_rx);

  /// Copy constructor (column matrix)
  explicit Point3D(const Matrix<T, 3, 1>& i_Matrix_rx);

  /// Copy constructor (row matrix)
  explicit Point3D(const Matrix<T, 1, 3>& i_Matrix_rx);

  /// Copy constructor with initialization by configuration
  explicit Point3D(const Config_s& i_Config_rs);

  /// Copy constructor with initialization by polar coordinates
  explicit Point3D(const Polar_s& i_Polar_rs);

  /// Virtual destructor
  virtual ~Point3D(void) {};

  // --------------------------------------------------------------------------
  // getters

  /// Get X location of point vector
  T getPosX(void) const;

  /// Get Y location of point vector
  T getPosY(void) const;

  /// Get Z location of point vector
  T getPosZ(void) const;

  /// Get polar coordinates of this euclidean 3D (projective 2D) point
  Polar_s getPolar(AngleUnit_e i_AngleUnit_e = e_Radians) const;

  /// Get elevation angle (theta of polar coordinate) of this euclidean 3D (projecitve 2D) point
  T getElevation(AngleUnit_e i_AngleUnit_e = e_Radians) const;

  /// Get elevation angle of point projected to xz-plane (y=0)
  T getElevationHorizontal(AngleUnit_e i_AngleUnit_e = e_Radians) const;

  /// Get elevation angle of point projected to yz-plane (x=0)
  T getElevationVertical(AngleUnit_e i_AngleUnit_e = e_Radians) const;

  // Get point coordinates of this point normalized to cube
  Point3D<T> getCubeCoords_x(const T i_Factor_x = math::constants<T>::one_x()) const;

  Point3D<T> getCubeCoords_x(uint32_t& o_NormDim_ru32, const T i_Factor_x = math::constants<T>::one_x()) const;

  // --------------------------------------------------------------------------
  // setters

  //  Normalize this point to cube
  void normalizeToCube_v(const T i_Factor);

  /// Set X location of point vector
  void setPosX(const T& i_Value_rx);

  /// Set Y location of point vector
  void setPosY(const T& i_Value_rx);

  /// Set Z location of point vector
  void setPosZ(const T& i_Value_rx);

  /// Set polar coordinates
  void setPolar(const Polar_s& i_Polar_rs);

  // --------------------------------------------------------------------------
  // math operations

  /// Calculate cross (wedge) product of this and another 3D vector
  const Point3D cross(const Vector<T,3>& i_Pos_rx) const;

  // --------------------------------------------------------------------------
  // output functions

  /// Print polar coordinates (radius, phi, theta) of 3D point
  void printPolar(AngleUnit_e i_AngleUnit_e = e_Degrees) const;

  /// Print polar coordinates (radius, phi, theta) of 3D point with comment
  void printPolar(char * comment, AngleUnit_e i_AngleUnit_e = e_Degrees) const;

  // --------------------------------------------------------------------------
  // operators

  /// Wedge Operator of two 3D vectors is cross product
  const Point3D<T> operator^ (const Vector<T,3>& i_Pos_x) const;

  /// Assign point vector to column matrix
  Point3D operator= (const Matrix<T,3,1>& i_Matrix_rx);

  /// Assign point vector to row matrix
  Point3D operator= (const Matrix<T,1,3>& i_Matrix_rx);

  /// Assign to polar coordinates
  Point3D operator= (const Polar_s& i_Polar_rx);

  /// Sets all coordinates of this point to assigned value
  Point3D operator= (const T& v);

  /// Cast to different base type
  template<typename T1>
  operator Point3D<T1>() const;

};


//--------------------------------------------------------------------------
/// @class Point4D
/// @brief Storage class of a 4 dimensional point location
///
// --------------------------------------------------------------------------
template<typename T> class Point4D : public Vector<T, 4>
{
public:

  typedef typename Vector<T, 4>::Config_s Config_s;  ///< Configuration data set

  // --------------------------------------------------------------------------
  // constructors + destructor

  /// Default constructor
  Point4D(void);

  /// Constructor with initialization value
  explicit Point4D(const T& i_Value_rx);

  /// Constructor with point coordinates
  explicit Point4D(const T& i_Value1_rx, const T& i_Value2_rx, const T& i_Value3_rx, const T& i_Value4_rx);

  /// Copy constructor (row matrix)
  explicit Point4D(const Matrix<T, 4, 1>& i_Matrix_rx);

  /// Copy constructor (column matrix)
  explicit Point4D(const Matrix<T, 1, 4>& i_Matrix_rx);

  /// Copy constructor with initialization by configuration
  explicit Point4D(const Config_s& i_Config_rs);

  /// Virtual destructor
  virtual ~Point4D(void) {};

  // --------------------------------------------------------------------------
  // getters

  /// Get X location of point vector
  T getPosX(void) const;

  /// Get Y location of point vector
  T getPosY(void) const;

  /// Get Z location of point vector
  T getPosZ(void) const;


  // --------------------------------------------------------------------------
  // setters

  /// Set X location of point vector
  void setPosX(const T& i_Value_rx);

  /// Set Y location of point vector
  void setPosY(const T& i_Value_rx);

  /// Set Z location of point vector
  void setPosZ(const T& i_Value_rx);

  // --------------------------------------------------------------------------
  // assignment operators

  /// Assign point vector to column matrix
  Point4D operator= (const Matrix<T,4,1>& M);

  /// Assign point vector to row matrix
  Point4D operator= (const Matrix<T,1,4>& M);

  /// Assign to base type value
  Point4D operator= (const T& v);

  /// Cast to different base type
  template<typename T1>
  operator Point4D<T1>() const;

};

} // namespace core
} // namespace mecl

#include "Point.hpp"

#endif // MECL_CORE_POINT_H_
/// @}
/// @}
