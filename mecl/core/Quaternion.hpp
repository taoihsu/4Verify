//  --------------------------------------------------------------------------
/// @file Quaternion.hpp
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


#ifndef MECL_CORE_QUATERNION_HPP_
#define MECL_CORE_QUATERNION_HPP_

#include "Quaternion.h"

namespace mecl
{

namespace core
{

template <typename T>
Quaternion<T>::Quaternion()
{
  xyzw[0] = math::constants<T>::zero_x();
  xyzw[1] = math::constants<T>::zero_x();
  xyzw[2] = math::constants<T>::zero_x();
  xyzw[3] = math::constants<T>::one_x();
}

template <typename T>
Quaternion<T>::Quaternion(const T& i_X, const T& i_Y, const T& i_Z, const T& i_W)
{
  xyzw[0] = i_X;
  xyzw[1] = i_Y;
  xyzw[2] = i_Z;
  xyzw[3] = i_W;
}


template <typename T>
Quaternion<T>::Quaternion(const EulerAngles_s& i_EulerAngles_ro)
{
  fromEulerAngles(i_EulerAngles_ro);
}

template <typename T>
Quaternion<T>::Quaternion(const Quaternion& i_Q_ro)
{
  *this = i_Q_ro;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator=(const Quaternion<T>& i_Q_ro)
{
  xyzw[0] = i_Q_ro.xyzw[0];
  xyzw[1] = i_Q_ro.xyzw[1];
  xyzw[2] = i_Q_ro.xyzw[2];
  xyzw[3] = i_Q_ro.xyzw[3];

  return *this;
}

template <typename T>
bool Quaternion<T>::operator == (const Quaternion<T>& i_Q_ro) const
{
  return math::equal_x(xyzw[0], i_Q_ro.xyzw[0]) 
      && math::equal_x(xyzw[1], i_Q_ro.xyzw[1]) 
      && math::equal_x(xyzw[2], i_Q_ro.xyzw[2]) 
      && math::equal_x(xyzw[3], i_Q_ro.xyzw[3]);
}

template <typename T>
bool Quaternion<T>::operator != (const Quaternion<T>& i_Q_ro) const
{
  return math::unequal_x(xyzw[0], i_Q_ro.xyzw[0]) 
      || math::unequal_x(xyzw[1], i_Q_ro.xyzw[1]) 
      || math::unequal_x(xyzw[2], i_Q_ro.xyzw[2]) 
      || math::unequal_x(xyzw[3], i_Q_ro.xyzw[3]);
}

template <typename T>
Quaternion<T> Quaternion<T>::operator - (void) const
{
  return Quaternion(-this->xyzw[0],
    -this->xyzw[1],
    -this->xyzw[2],
    -this->xyzw[3]);
}

template <typename T>
Quaternion<T> Quaternion<T>::operator + (void) const
{
  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator -= (const Quaternion<T>& i_Q_ro)
{
  xyzw[0] -= i_Q_ro.xyzw[0];
  xyzw[1] -= i_Q_ro.xyzw[1];
  xyzw[2] -= i_Q_ro.xyzw[2];
  xyzw[3] -= i_Q_ro.xyzw[3];

  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator += (const Quaternion<T>& i_Q_ro)
{
  xyzw[0] += i_Q_ro.xyzw[0];
  xyzw[1] += i_Q_ro.xyzw[1];
  xyzw[2] += i_Q_ro.xyzw[2];
  xyzw[3] += i_Q_ro.xyzw[3];

  return *this;
}


template <typename T>
Quaternion<T>& Quaternion<T>::operator *= (const Quaternion<T>& i_Q_ro)
{
  *this = *this * i_Q_ro;

  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator /= (const Quaternion<T>& i_Q_ro)
{
  *this = *this / i_Q_ro;

  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator -= (const T& i_Scalar)
{
  xyzw[0] -= i_Scalar;
  xyzw[1] -= i_Scalar;
  xyzw[2] -= i_Scalar;
  xyzw[3] -= i_Scalar;

  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator += (const T& i_Scalar)
{
  xyzw[0] += i_Scalar;
  xyzw[1] += i_Scalar;
  xyzw[2] += i_Scalar;
  xyzw[3] += i_Scalar;

  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator *= (const T& i_Scalar)
{
  xyzw[0] *= i_Scalar;
  xyzw[1] *= i_Scalar;
  xyzw[2] *= i_Scalar;
  xyzw[3] *= i_Scalar;

  return *this;
}

template <typename T>
Quaternion<T>& Quaternion<T>::operator /= (const T& i_Scalar)
{
  T v_OneDivScalar = mecl::math::constants<T>::one_x() / i_Scalar;

  xyzw[0] *= v_OneDivScalar;
  xyzw[1] *= v_OneDivScalar;
  xyzw[2] *= v_OneDivScalar;
  xyzw[3] *= v_OneDivScalar;

  return *this;
}

template <typename T>
Quaternion<T> Quaternion<T>::operator - (const Quaternion<T>& i_Q_ro) const
{
  return Quaternion(this->xyzw[0] - i_Q_ro.xyzw[0],
                    this->xyzw[1] - i_Q_ro.xyzw[1],
                    this->xyzw[2] - i_Q_ro.xyzw[2],
                    this->xyzw[3] - i_Q_ro.xyzw[3]);
}

template <typename T>
Quaternion<T> Quaternion<T>::operator + (const Quaternion& i_Q_ro) const
{
  return Quaternion(this->xyzw[0] + i_Q_ro.xyzw[0],
                    this->xyzw[1] + i_Q_ro.xyzw[1],
                    this->xyzw[2] + i_Q_ro.xyzw[2],
                    this->xyzw[3] + i_Q_ro.xyzw[3]);
}

template <typename T>
Quaternion<T> Quaternion<T>::operator * (const Quaternion& i_Q_ro) const
{
  return mul(*this, i_Q_ro);
}

template <typename T>
Quaternion<T> Quaternion<T>::operator / (const Quaternion& i_Q_ro) const
{
  Quaternion v_QuatInverse = inverse(i_Q_ro);

  return mul(*this, v_QuatInverse);
}

template <typename T>
Quaternion<T> Quaternion<T>::operator - (const T& i_Scalar) const
{
  return Quaternion(this->xyzw[0] - i_Scalar,
                    this->xyzw[1] - i_Scalar,
                    this->xyzw[2] - i_Scalar,
                    this->xyzw[3] - i_Scalar);
}


template <typename T>
Quaternion<T> Quaternion<T>::operator + (const T& i_Scalar) const
{
  return Quaternion(this->xyzw[0] + i_Scalar,
                    this->xyzw[1] + i_Scalar,
                    this->xyzw[2] + i_Scalar,
                    this->xyzw[3] + i_Scalar);
}

template <typename T>
Quaternion<T> Quaternion<T>::operator * (const T& i_Scalar) const
{
  return Quaternion(this->xyzw[0] * i_Scalar,
                    this->xyzw[1] * i_Scalar,
                    this->xyzw[2] * i_Scalar,
                    this->xyzw[3] * i_Scalar);
}

template <typename T>
Quaternion<T> Quaternion<T>::operator / (const T& i_Scalar) const
{
  return Quaternion(this->xyzw[0] / i_Scalar,
                    this->xyzw[1] / i_Scalar,
                    this->xyzw[2] / i_Scalar,
                    this->xyzw[3] / i_Scalar);
}


template <typename T>
Quaternion<T>::operator Point4D<T>(void) const
{
  return Point4D<T>(this->xyzw[0], this->xyzw[1],
                    this->xyzw[2], this->xyzw[3]);
}

template <typename T>
Quaternion<T>::operator Matrix<T, 1, 4>(void) const
{
  return Matrix<T, 1, 4>(this->xyzw);
}

template <typename T>
T Quaternion<T>::lengthSquared() const
{
  return xyzw[0] * xyzw[0] +
         xyzw[1] * xyzw[1] +
         xyzw[2] * xyzw[2] +
         xyzw[3] * xyzw[3];
}

template <typename T>
T Quaternion<T>::length() const
{
  return math::algebra<T>::sqrt_x(lengthSquared());
}

template <typename T>
void Quaternion<T>::normalize()
{
  T v_Length = length();

  if (v_Length > math::constants<T>::zero_x())
  {
    T v_LengthInv = math::constants<T>::one_x() / v_Length;

    *this *= v_LengthInv;
  }
}

template <typename T>
Quaternion<T> Quaternion<T>::conjugate(const Quaternion& i_Q_ro)
{
  return Quaternion(-i_Q_ro.xyzw[0],
                    -i_Q_ro.xyzw[1],
                    -i_Q_ro.xyzw[2],
                    +i_Q_ro.xyzw[3]);
}

template <typename T>
Quaternion<T> Quaternion<T>::inverse(const Quaternion&  i_Q_ro)
{
  Quaternion<T> v_Conjugate_o = conjugate(i_Q_ro);

  T lengthSq = v_Conjugate_o.lengthSquared();

  return v_Conjugate_o / lengthSq;
}

template <typename T>
Quaternion<T> Quaternion<T>::mul(const Quaternion& i_SecondRot_ro, const Quaternion& i_FirstRot_ro)
{
  const Quaternion& q1 = i_SecondRot_ro;
  const Quaternion& q2 = i_FirstRot_ro;

  Quaternion q1q2;

  q1q2.xyzw[0] = q1.xyzw[3] * q2.xyzw[0] + q1.xyzw[0] * q2.xyzw[3] + q1.xyzw[1] * q2.xyzw[2] - q1.xyzw[2] * q2.xyzw[1];
  q1q2.xyzw[1] = q1.xyzw[3] * q2.xyzw[1] - q1.xyzw[0] * q2.xyzw[2] + q1.xyzw[1] * q2.xyzw[3] + q1.xyzw[2] * q2.xyzw[0];
  q1q2.xyzw[2] = q1.xyzw[3] * q2.xyzw[2] + q1.xyzw[0] * q2.xyzw[1] - q1.xyzw[1] * q2.xyzw[0] + q1.xyzw[2] * q2.xyzw[3];
  q1q2.xyzw[3] = q1.xyzw[3] * q2.xyzw[3] - q1.xyzw[0] * q2.xyzw[0] - q1.xyzw[1] * q2.xyzw[1] - q1.xyzw[2] * q2.xyzw[2];

  return q1q2;
}

template <typename T>
Projection4D<T> Quaternion<T>::toMatrix4x4() const
{
  Projection4D<T> out;

  out(0,0) = 1 - 2 * xyzw[1] * xyzw[1] - 2 * xyzw[2] * xyzw[2];    
  out(0,1) =     2 * xyzw[0] * xyzw[1] + 2 * xyzw[2] * xyzw[3];    
  out(0,2) =     2 * xyzw[0] * xyzw[2] - 2 * xyzw[1] * xyzw[3];    
  out(0,3) = 0;

  out(1,0) =     2 * xyzw[0] * xyzw[1] - 2 * xyzw[2] * xyzw[3];    
  out(1,1) = 1 - 2 * xyzw[0] * xyzw[0] - 2 * xyzw[2] * xyzw[2];    
  out(1,2) =     2 * xyzw[1] * xyzw[2] + 2 * xyzw[0] * xyzw[3];    
  out(1,3) = 0;

  out(2,0) =     2 * xyzw[0] * xyzw[2] + 2 * xyzw[1] * xyzw[3];    
  out(2,1) =     2 * xyzw[1] * xyzw[2] - 2 * xyzw[0] * xyzw[3];    
  out(2,2) = 1 - 2 * xyzw[0] * xyzw[0] - 2 * xyzw[1] * xyzw[1];    
  out(2,3) = 0;

  out(3,0) = 0;                            
  out(3,1) = 0;                           
  out(3,2) = 0;                            
  out(3,3) = 1;

  return out;
}

template <typename T>
Matrix3x3<T> Quaternion<T>::toMatrix3x3() const
{
  Matrix3x3<T> out;

  out(0,0) = 1 - 2 * xyzw[1] * xyzw[1] - 2 * xyzw[2] * xyzw[2];    
  out(0,1) =     2 * xyzw[0] * xyzw[1] + 2 * xyzw[2] * xyzw[3];    
  out(0,2) =     2 * xyzw[0] * xyzw[2] - 2 * xyzw[1] * xyzw[3];    

  out(1,0) =     2 * xyzw[0] * xyzw[1] - 2 * xyzw[2] * xyzw[3];    
  out(1,1) = 1 - 2 * xyzw[0] * xyzw[0] - 2 * xyzw[2] * xyzw[2];    
  out(1,2) =     2 * xyzw[1] * xyzw[2] + 2 * xyzw[0] * xyzw[3];    

  out(2,0) =     2 * xyzw[0] * xyzw[2] + 2 * xyzw[1] * xyzw[3];    
  out(2,1) =     2 * xyzw[1] * xyzw[2] - 2 * xyzw[0] * xyzw[3];    
  out(2,2) = 1 - 2 * xyzw[0] * xyzw[0] - 2 * xyzw[1] * xyzw[1];    

  return out;
}

template <typename T>
typename Quaternion<T>::EulerAngles_s Quaternion<T>::calcEulerAngles_s() const
{
  EulerAngles_s v_Out_s;

  T sqx = xyzw[0] * xyzw[0];
  T sqy = xyzw[1] * xyzw[1];
  T sqz = xyzw[2] * xyzw[2];
  T sqw = xyzw[3] * xyzw[3];

  // if normalized: unit is one, otherwise is correction factor
  T unit = sqx + sqy + sqz + sqw;

  T test = static_cast<T>(0.449);

  if (xyzw[0] * xyzw[1] + xyzw[2] * xyzw[3] > test * unit)    
  {
    // singularity at north pole
    v_Out_s.yaw_x   = static_cast<T>(2) * math::trigonometry<T>::atan2_x(xyzw[0], xyzw[3]);
    v_Out_s.roll_x  = math::constants<T>::pi_x() / static_cast<T>(2);
    v_Out_s.pitch_x = math::constants<T>::zero_x();
  }
  else if (xyzw[0] * xyzw[1] + xyzw[2] * xyzw[3] < -test * unit)    
  {
    // singularity at south pole
    v_Out_s.yaw_x   = static_cast<T>(-2) * math::trigonometry<T>::atan2_x(xyzw[0], xyzw[3]);
    v_Out_s.roll_x  = -(math::constants<T>::pi_x() / static_cast<T>(2));
    v_Out_s.pitch_x = math::constants<T>::zero_x();
  }
  else
  {
    v_Out_s.yaw_x   = math::trigonometry<T>::atan2_x(static_cast<T>(2) * xyzw[1] * xyzw[3] - static_cast<T>(2) * xyzw[0] * xyzw[2] , sqx - sqy - sqz + sqw);
    v_Out_s.roll_x  = math::trigonometry<T>::asin_x((static_cast<T>(2) * xyzw[0] * xyzw[1] + static_cast<T>(2) * xyzw[2] * xyzw[3]) / unit);
    v_Out_s.pitch_x = math::trigonometry<T>::atan2_x(static_cast<T>(2) * xyzw[0] * xyzw[3] - static_cast<T>(2) * xyzw[1] * xyzw[2] , -sqx + sqy - sqz + sqw);
  }

  v_Out_s.normalize();

  return v_Out_s;
}

template <typename T>
void Quaternion<T>::fromEulerAngles(const EulerAngles_s& i_EulerAngles_ro)
{
  T v_Two = static_cast<T>(2);

  T v_SinHalfX = math::trigonometry<T>::sin_x(i_EulerAngles_ro.pitch_x / v_Two);
  T v_CosHalfX = math::trigonometry<T>::cos_x(i_EulerAngles_ro.pitch_x / v_Two);

  T v_SinHalfY = math::trigonometry<T>::sin_x(i_EulerAngles_ro.yaw_x / v_Two);
  T v_CosHalfY = math::trigonometry<T>::cos_x(i_EulerAngles_ro.yaw_x / v_Two);

  T v_SinHalfZ = math::trigonometry<T>::sin_x(i_EulerAngles_ro.roll_x / v_Two);
  T v_CosHalfZ = math::trigonometry<T>::cos_x(i_EulerAngles_ro.roll_x / v_Two);

  xyzw[0] = v_CosHalfY * v_CosHalfZ * v_SinHalfX + v_SinHalfY * v_SinHalfZ * v_CosHalfX;
  xyzw[1] = v_CosHalfY * v_SinHalfZ * v_SinHalfX + v_SinHalfY * v_CosHalfZ * v_CosHalfX;
  xyzw[2] = v_CosHalfY * v_SinHalfZ * v_CosHalfX - v_SinHalfY * v_CosHalfZ * v_SinHalfX;
  xyzw[3] = v_CosHalfY * v_CosHalfZ * v_CosHalfX - v_SinHalfY * v_SinHalfZ * v_SinHalfX;
}

template <typename T>
void Quaternion<T>::fromAngleAxis(T i_Angle, Point3D<T> rotationAxis)
{
  T v_HalfAngle = i_Angle / static_cast<T>(2);
  T v_SinHalfAngle = math::trigonometry<T>::sin_x(v_HalfAngle);

  xyzw[0] = rotationAxis.getPosX() * v_SinHalfAngle;
  xyzw[1] = rotationAxis.getPosY() * v_SinHalfAngle;
  xyzw[2] = rotationAxis.getPosZ() * v_SinHalfAngle;
  xyzw[3] = math::trigonometry<T>::cos_x(v_HalfAngle);
}


template <typename T>
void Quaternion<T>::fromMatrix4x4(const Projection4D<T>& i_Mat_ro)
{
  T trace = i_Mat_ro(0,0) + i_Mat_ro(1,1) + i_Mat_ro(2,2); 

  T one = math::constants<T>::one_x();

  if (trace > math::constants<T>::zero_x()) 
  {  
    float s = static_cast<T>(0.5) / math::algebra<T>::sqrt_x(trace + one);
    xyzw[0] = (i_Mat_ro(2,1) - i_Mat_ro(1,2) ) * s;
    xyzw[1] = (i_Mat_ro(0,2) - i_Mat_ro(2,0) ) * s;
    xyzw[2] = (i_Mat_ro(1,0) - i_Mat_ro(0,1) ) * s;
    xyzw[3] = static_cast<T>(0.25) / s;
  } 
  else 
  {
    if ( i_Mat_ro(0,0) > i_Mat_ro(1,1) && i_Mat_ro(0,0) > i_Mat_ro(2,2) ) 
    {
      float s = static_cast<T>(2) * math::algebra<T>::sqrt_x(one + i_Mat_ro(0,0) - i_Mat_ro(1,1) - i_Mat_ro(2,2));
      xyzw[0] = static_cast<T>(0.25) * s;
      xyzw[1] = (i_Mat_ro(0,1) + i_Mat_ro(1,0) ) / s;
      xyzw[2] = (i_Mat_ro(0,2) + i_Mat_ro(2,0) ) / s;
      xyzw[3] = (i_Mat_ro(2,1) - i_Mat_ro(1,2) ) / s;
    } 
    else if (i_Mat_ro(1,1) > i_Mat_ro(2,2)) 
    {
      float s = static_cast<T>(2) * math::algebra<T>::sqrt_x(one + i_Mat_ro(1,1) - i_Mat_ro(0,0) - i_Mat_ro(2,2));
      xyzw[0] = (i_Mat_ro(0,1) + i_Mat_ro(1,0) ) / s;
      xyzw[1] = static_cast<T>(0.25) * s;
      xyzw[2] = (i_Mat_ro(1,2) + i_Mat_ro(2,1) ) / s;
      xyzw[3] = (i_Mat_ro(0,2) - i_Mat_ro(2,0) ) / s;
    } 
    else 
    {
      float s = static_cast<T>(2) * math::algebra<T>::sqrt_x(one + i_Mat_ro(2,2) - i_Mat_ro(0,0) - i_Mat_ro(1,1) );
      xyzw[0] = (i_Mat_ro(0,2) + i_Mat_ro(2,0) ) / s;
      xyzw[1] = (i_Mat_ro(1,2) + i_Mat_ro(2,1) ) / s;
      xyzw[2] = static_cast<T>(0.25) * s;
      xyzw[3] = (i_Mat_ro(1,0) - i_Mat_ro(0,1) ) / s;
    }
  }
}


template <typename T>
void Quaternion<T>::fromMatrix3x3(const Matrix3x3<T>& i_Mat_ro)
{
  T trace = i_Mat_ro(0,0) + i_Mat_ro(1,1) + i_Mat_ro(2,2); 

  T one = math::constants<T>::one_x();

  if (trace > math::constants<T>::zero_x()) 
  {  
    float s = static_cast<T>(0.5) / math::algebra<T>::sqrt_x(trace + one);
    xyzw[0] = (i_Mat_ro(2,1) - i_Mat_ro(1,2) ) * s;
    xyzw[1] = (i_Mat_ro(0,2) - i_Mat_ro(2,0) ) * s;
    xyzw[2] = (i_Mat_ro(1,0) - i_Mat_ro(0,1) ) * s;
    xyzw[3] = static_cast<T>(0.25) / s;
  } 
  else 
  {
    if ( i_Mat_ro(0,0) > i_Mat_ro(1,1) && i_Mat_ro(0,0) > i_Mat_ro(2,2) ) 
    {
      float s = static_cast<T>(2) * math::algebra<T>::sqrt_x(one + i_Mat_ro(0,0) - i_Mat_ro(1,1) - i_Mat_ro(2,2));
      xyzw[0] = static_cast<T>(0.25) * s;
      xyzw[1] = (i_Mat_ro(0,1) + i_Mat_ro(1,0) ) / s;
      xyzw[2] = (i_Mat_ro(0,2) + i_Mat_ro(2,0) ) / s;
      xyzw[3] = (i_Mat_ro(2,1) - i_Mat_ro(1,2) ) / s;
    } 
    else if (i_Mat_ro(1,1) > i_Mat_ro(2,2)) 
    {
      float s = static_cast<T>(2) * math::algebra<T>::sqrt_x(one + i_Mat_ro(1,1) - i_Mat_ro(0,0) - i_Mat_ro(2,2));
      xyzw[0] = (i_Mat_ro(0,1) + i_Mat_ro(1,0) ) / s;
      xyzw[1] = static_cast<T>(0.25) * s;
      xyzw[2] = (i_Mat_ro(1,2) + i_Mat_ro(2,1) ) / s;
      xyzw[3] = (i_Mat_ro(0,2) - i_Mat_ro(2,0) ) / s;
    } 
    else 
    {
      float s = static_cast<T>(2) * math::algebra<T>::sqrt_x(one + i_Mat_ro(2,2) - i_Mat_ro(0,0) - i_Mat_ro(1,1) );
      xyzw[0] = (i_Mat_ro(0,2) + i_Mat_ro(2,0) ) / s;
      xyzw[1] = (i_Mat_ro(1,2) + i_Mat_ro(2,1) ) / s;
      xyzw[2] = static_cast<T>(0.25) * s;
      xyzw[3] = (i_Mat_ro(1,0) - i_Mat_ro(0,1) ) / s;
    }
  }
}

template <typename T>
Quaternion<T> Quaternion<T>::preRoll0(void)
{
  static Quaternion v_Identity_o;
  return v_Identity_o;
}

template <typename T>
Quaternion<T> Quaternion<T>::preRoll90(void)
{
  static Quaternion v_Quaternion_o;

  v_Quaternion_o.xyzw[0] = math::constants<T>::zero_x();
  v_Quaternion_o.xyzw[1] = math::constants<T>::zero_x();
  v_Quaternion_o.xyzw[2] = static_cast<T>(0.70710677);
  v_Quaternion_o.xyzw[3] = static_cast<T>(0.70710677);

  return v_Quaternion_o;
}

template <typename T>
Quaternion<T> Quaternion<T>::preRoll180(void)
{
  static Quaternion v_Quaternion_o;

  v_Quaternion_o.xyzw[0] = math::constants<T>::zero_x();
  v_Quaternion_o.xyzw[1] = math::constants<T>::zero_x();
  v_Quaternion_o.xyzw[2] = math::constants<T>::one_x();
  v_Quaternion_o.xyzw[3] = static_cast<T>(-4.3711388e-008);

  return v_Quaternion_o;
}

template <typename T>
Quaternion<T> Quaternion<T>::preRoll270(void)
{
  static Quaternion v_Quaternion_o;

  v_Quaternion_o.xyzw[0] = math::constants<T>::zero_x();
  v_Quaternion_o.xyzw[1] = math::constants<T>::zero_x();
  v_Quaternion_o.xyzw[2] = static_cast<T>( 0.70710677);
  v_Quaternion_o.xyzw[3] = static_cast<T>(-0.70710677);

  return v_Quaternion_o;
}

template <typename T>
Point3D<T> Quaternion<T>::transform(const Point3D<T>& i_Point_ro, const Quaternion<T>& i_Rotation_ro)
{
  return transform(i_Point_ro, i_Rotation_ro, inverse(i_Rotation_ro));
}

template <typename T>
Point3D<T> Quaternion<T>::transform(const Point3D<T>& i_Point_ro, const Quaternion<T>& i_Rotation_ro, const Quaternion<T>& i_InverseRotation_ro)
{
  // q * v * q^-1
  Quaternion<T> vQinv  = mul(Quaternion<T>(i_Point_ro.getPosX(), i_Point_ro.getPosY(), i_Point_ro.getPosZ(), 1.0f), i_InverseRotation_ro);
  Quaternion<T> qVQinv = mul(i_Rotation_ro, vQinv);

  return Point3D<T>(qVQinv.xyzw[0], qVQinv.xyzw[1], qVQinv.xyzw[2]);
}

} // namespace core

} // namespace mecl

#endif
