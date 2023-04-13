//  --------------------------------------------------------------------------
/// @file Quaternion.h
/// @brief Used to store and calculate rotations.
///
/// Here may follow a longer description for the template module with examples.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Mark Reichert (Mark.Reichert2@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{


#ifndef MECL_CORE_QUATERNION_H_
#define MECL_CORE_QUATERNION_H_

#include "Projection4D.h"
#include "Matrix3x3.h"
#include "Point.h"

namespace mecl
{

namespace core
{

// --------------------------------------------------------------------------
/// @class Quaternion
/// @brief The Quaternion class implements a 3D rotation and provides converter functions.
// --------------------------------------------------------------------------
template <typename T>
class Quaternion
{
public:
  // --------------------------------------------------------------------------
  //! @struct EulerAngles_s
  //! @brief Euler angles data structure
  // --------------------------------------------------------------------------
  struct EulerAngles_s
  {
    T pitch_x;              ///< image plane rotation around x-axis
    T yaw_x;                ///< image plane rotation around y-axis
    T roll_x;               ///< image plane rotation around z-axis

  // --------------------------------------------------------------------------
  /// @brief Get normalized angle value
  ///
  /// Normalize angle to pitch yaw and roll values within the interval [-pi,+pi] resp. [-180°,+180°] deg
  // --------------------------------------------------------------------------
    void normalize(AngleUnit_e i_AngleUnit_e = e_Radians)
    {
      pitch_x = math::getNormalizedAngle_x(pitch_x, i_AngleUnit_e);
      yaw_x   = math::getNormalizedAngle_x(yaw_x,   i_AngleUnit_e);
      roll_x  = math::getNormalizedAngle_x(roll_x,  i_AngleUnit_e);
    }
  };

  // --------------------------------------------------------------------------
  /// @brief Default constructor
  /// The default construtor creates an identity quaternion
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_Constructor1
  ///
  /// @return       Quaternion
  // --------------------------------------------------------------------------
  Quaternion();

    // --------------------------------------------------------------------------
  /// @brief Constructor with initialization value
  ///
  /// The constructor initializes all cell elements to the input argument \p value.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_Constructor2
  ///
  /// @param[in] i_X  Initialization value for x component
  /// @param[in] i_Y  Initialization value for y component
  /// @param[in] i_Z  Initialization value for z component
  /// @param[in] i_W  Initialization value for w component
  /// @return         Quaternion pre-initialized with specific values
  // --------------------------------------------------------------------------
  explicit Quaternion(const T& i_X, const T& i_Y, const T& i_Z, const T& i_W);

    // --------------------------------------------------------------------------
  /// @brief Copy constructor with initialization by Euler angle configuration
  ///
  /// The constructor initializes cell contents using Euler angle configuration
  /// supplied in \p i_EulerAngles_s.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_Constructor3
  ///
  /// @param[in]    i_EulerAngles_s Euler angle configuration
  /// @return       Quaternion pre-initialized with specific angle configuration
  // --------------------------------------------------------------------------
  explicit Quaternion(const EulerAngles_s& i_EulerAngles_ro);

  // copy constructor and assignment operators
  Quaternion(const Quaternion& i_Q_ro);
  Quaternion& operator = (const Quaternion& i_Q_ro);

  // equality
  bool operator == (const Quaternion& i_Q_ro) const;
  bool operator != (const Quaternion& i_Q_ro) const;

  // unary
  Quaternion operator - (void) const;
  Quaternion operator + (void) const;

  Quaternion& operator -= (const Quaternion& i_Q_ro);
  Quaternion& operator += (const Quaternion& i_Q_ro);
  Quaternion& operator *= (const Quaternion& i_Q_ro);
  Quaternion& operator /= (const Quaternion& i_Q_ro);

  Quaternion& operator -= (const T& i_Scalar);
  Quaternion& operator += (const T& i_Scalar);
  Quaternion& operator *= (const T& i_Scalar);
  Quaternion& operator /= (const T& i_Scalar);

  Quaternion operator - (const Quaternion& i_Q_ro) const;
  Quaternion operator + (const Quaternion& i_Q_ro) const;
  Quaternion operator * (const Quaternion& i_Q_ro) const;
  Quaternion operator / (const Quaternion& i_Q_ro) const;

  Quaternion operator - (const T& i_Scalar) const;
  Quaternion operator + (const T& i_Scalar) const;
  Quaternion operator * (const T& i_Scalar) const;
  Quaternion operator / (const T& i_Scalar) const;

  // cast operators
  operator Point4D<T>(void) const;
  operator Matrix<T, 1, 4>(void) const;

  // get xyzw Components
  inline T getX() const { return xyzw[0]; }
  inline T getY() const { return xyzw[1]; }
  inline T getZ() const { return xyzw[2]; }
  inline T getW() const { return xyzw[3]; }


  // --------------------------------------------------------------------------
  /// @brief Conjugates a quaternion
  ///
  /// The conjugate of a quaternion number is a quaternion with the same magnitudes 
  /// but with the sign of the imaginary parts changed.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_conjugate
  ///
  /// @param[in] i_Q_ro  The Quaternion to conjugate
  ///
  /// @return conjugated Quaternion
  // --------------------------------------------------------------------------
  static Quaternion conjugate(const Quaternion& i_Q_ro);

  // --------------------------------------------------------------------------
  /// @brief computes the inverse of a Quaternion
  ///
  /// Input does not need to be normalized.
  /// Returned Quaternion is normalized if input was normalized
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_inverse
  ///
  /// @param[in] i_Q_ro  Quaternion to invert
  ///
  /// @return inverted Quaternion
  // --------------------------------------------------------------------------
  static Quaternion inverse(const Quaternion& i_Q_ro);

  // --------------------------------------------------------------------------
  /// @brief computes the multiplication of two quaternions (the concatenation of two quaternions)
  ///
  /// Quaternion multiplication is not commutative.
  /// Let q1 be first argument, q2 be second argument: Performs q1*q2. 
  /// That means FIRST rotation q2 is applied, THEN operation q1 is applied
  /// Q1 and q2 do not need to be normalized. Be aware that output is not normalized either 
  /// (output is in fact normalized except for floating point imprecisions, if input was normalized)
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_mul
  ///
  /// @param[in] i_SecondRot_ro  Quaternion to invert
  /// @param[in] i_FirstRot_ro   Quaternion to invert
  ///
  /// @return a quaternion containing the result of the multiplication.
  // --------------------------------------------------------------------------
  static Quaternion mul(const Quaternion& i_SecondRot_ro, const Quaternion& i_FirstRot_ro);

  // --------------------------------------------------------------------------
  /// @brief computes Length of the Quaternion in squared space
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_lengthSquared
  ///
  /// @return The Quaternion's length in squared space
  // --------------------------------------------------------------------------
  T lengthSquared() const;

  // --------------------------------------------------------------------------
  /// @brief computes Length of the Quaternion
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_length
  ///
  /// @return The Quaternion's length
  // --------------------------------------------------------------------------
  T length() const;

  // --------------------------------------------------------------------------
  /// @brief Brings the Quaternion to unit length.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_normalize
  ///
  // --------------------------------------------------------------------------
  void normalize();

  // --------------------------------------------------------------------------
  /// @brief Converts the Quaternion to a 4x4 Matrix
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_toMatrix4x4
  ///
  /// @return A 4x4 Matrix which contains the rotation
  // --------------------------------------------------------------------------
  Projection4D<T> toMatrix4x4() const;

  // --------------------------------------------------------------------------
  /// @brief Converts the Quaternion to a 3x3 Matrix
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_toMatrix3x3
  ///
  /// @return A 3x3 Matrix which contains the rotation
  // --------------------------------------------------------------------------
  Matrix3x3<T> toMatrix3x3() const;

  // --------------------------------------------------------------------------
  /// @brief Calculate Euler angles
  ///
  /// The function calculates Euler angles based on the Quaternion.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_calcEulerAngles
  ///
  /// @return EulerAngles_s normalized Euler angle configuration
  // --------------------------------------------------------------------------
  EulerAngles_s calcEulerAngles_s() const;

  // --------------------------------------------------------------------------
  /// @brief Creates a Quaternion from Euler Angles
  /// 
  /// Creates Rotation around the z-axis, then x-axis, then y-axis:
  /// Clockwise (when axis points towards eye in a left-handed coordinate-system ),
  /// Output Quaternion is normalized
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_fromEulerAngles
  ///
  // --------------------------------------------------------------------------
  void fromEulerAngles(const EulerAngles_s& i_EulerAngles_ro);


  // --------------------------------------------------------------------------
  /// @brief Creates a Quaternion from Angle (in Radians) and axis (needs to be normalized)
  /// 
  /// Rotates around the given axis. If axis points towards observer, 
  /// in a left handed system it is rotated clockwise
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_fromAngleAxis
  ///
  // --------------------------------------------------------------------------
  void fromAngleAxis(T i_Angle, Point3D<T> rotationAxis);

  // --------------------------------------------------------------------------
  /// @brief Creates a Quaternion from a pure rotation 4x4 matrix (no scale and translation)
  /// 
  /// @param[in] i_Mat_ro The input 4x4 matrix to create the quaternion from.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_fromMatrix4x4
  ///
  // --------------------------------------------------------------------------
  void fromMatrix4x4(const Projection4D<T>& i_Mat_ro);

  // --------------------------------------------------------------------------
  /// @brief Creates a Quaternion from a pure rotation 3x3 matrix (no scale and translation)
  /// 
  /// @param[in] i_Mat_ro The input 4x4 matrix to create the quaternion from.
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_fromMatrix3x3
  ///
  // --------------------------------------------------------------------------
  void fromMatrix3x3(const Matrix3x3<T>& i_Mat_ro);


  // --------------------------------------------------------------------------
  /// @brief Creates a Quaternion with zero degree roll
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_preRoll0
  ///
  /// @return A Quaternion with zero degree roll
  // --------------------------------------------------------------------------
  static Quaternion preRoll0(void);

  // --------------------------------------------------------------------------
  /// @brief Creates a Quaternion with 90 degree roll
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_preRoll90
  ///
  /// @return A Quaternion with 90 degree roll
  // --------------------------------------------------------------------------
  static Quaternion preRoll90(void);

  // --------------------------------------------------------------------------
  /// @brief Creates a Quaternion with 180 degree roll
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_preRoll180
  ///
  /// @return A Quaternion with 180 degree roll
  // --------------------------------------------------------------------------
  static Quaternion preRoll180(void);

  // --------------------------------------------------------------------------
  /// @brief Creates a Quaternion with 270 degree roll
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Quaternion_preRoll270
  ///
  /// @return A Quaternion with 270 degree roll
  // --------------------------------------------------------------------------
  static Quaternion preRoll270(void);


  // --------------------------------------------------------------------------
  /// @brief Transform Point3D via Quaternion
  ///
  /// @return The transformed point
  // --------------------------------------------------------------------------
  static Point3D<T> transform(const Point3D<T>& i_Point_ro, const Quaternion<T>& i_Rotation_ro);

  // --------------------------------------------------------------------------
  /// @brief Transform Point3D via Quaternion
  ///
  /// @return The transformed point
  // --------------------------------------------------------------------------
  static Point3D<T> transform(const Point3D<T>& i_Point_ro, const Quaternion<T>& i_Rotation_ro, const Quaternion<T>& inverseRotation);

private:
  T xyzw[4];
};

} // namespace core

} // namespace mecl
#include "Quaternion.hpp"

#endif // MECL_CORE_QUATERNION_H_
