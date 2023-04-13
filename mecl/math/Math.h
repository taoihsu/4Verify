// --------------------------------------------------------------------------
/// @file Math.h
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
/// @author Sebastian Pliefke (sebastian.pliefke@magna.com)
///
// --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup math
/// @{
/// @brief Magna Electronics Math Library

#ifndef MECL_MATH_MATH_H_
#define MECL_MATH_MATH_H_

//! @cond
// PRQA S 1020 1 // macro is used by MKS
#define Math_D_VERSION_ID "$Id: Math.h 1.30 2016/06/29 06:41:00EDT Zollner, Helmut (SAI_HeZol) draft  $"
//! @endcond

#ifndef __GNUC__
#undef min
#undef max
#include <limits>
#endif
#include <math.h>
#include <stdio.h>

#include "core/MeclTypes.h"

#ifdef _MSC_VER
#pragma warning (disable: 4996) // disable sprintf warnings
#endif

namespace mecl
{
namespace math
{

// --------------------------------------------------------------------------
/// @struct constants
/// @brief Contains numeric constants for type specified in template parameter T.
// --------------------------------------------------------------------------
template <typename T>
struct constants
{
  // --------------------------------------------------------------------------
  /// @brief Number pi (&pi;)
  ///
  /// Returns the type specific value of pi (&pi;)
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_constants_pi_x
  ///
  /// @return Type specific value of pi (&pi;)
  // --------------------------------------------------------------------------
  static const T pi_x(void)
  {
    return static_cast<T>(3.14159265358979323846264338327950);
  }

  // --------------------------------------------------------------------------
  /// @brief One (1)
  ///
  /// Returns the type specific value of one (1)
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_constants_one_x
  ///
  /// @return Type specific value of one (1)
  // --------------------------------------------------------------------------
  static const T one_x(void)
  {
    return static_cast<T>(1);
  }

  // --------------------------------------------------------------------------
  /// @brief Minus one (-1)
  ///
  /// Returns the type specific value of minus one (-1). Different encodings possible 
  /// for e.g. signed integer
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_constants_minusOne_x
  ///
  /// @return Type specific value of minus one (-1)
  // --------------------------------------------------------------------------
  static const T minusOne_x(void)
  {
    return static_cast<T>(-1);
  }

  // --------------------------------------------------------------------------
  /// @brief Zero (0)
  ///
  /// Returns the type specific value of zero (0).
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_constants_zero_x
  ///
  /// @return Type specific value of zero (0)
  // --------------------------------------------------------------------------
  static const T zero_x(void)
  {
    return static_cast<T>(0);
  }
};

// --------------------------------------------------------------------------
/// @brief Number pi (&pi;)
///
/// Returns the type specific value of pi (&pi;)
///
/// @attention This implementation will most likely be removed from the MECL
/// library since it is duplicate functionality of the mecl::math::constants<T>::pi_x()
/// function.
///
/// @par Example use:
/// @snippet MathTester.cpp mecl_math_getPi_x
///
/// @return Type specific value of pi (&pi;)
// --------------------------------------------------------------------------
template<typename T>
static T getPi_x(void)
{
  return constants<T>::pi_x();
}

// --------------------------------------------------------------------------
/// @brief Converts angle from degrees to radians
///
/// @par Example use:
/// @snippet MathTester.cpp mecl_math_toRadians_x
///
/// @param[in] i_Value_rx Angle in radians to be converted into degrees
/// @return Angle in degrees
// --------------------------------------------------------------------------
template<typename T>
static T toRadians_x(const T& i_Value_rx)
{
  return i_Value_rx / 180 * constants<T>::pi_x();
}

// --------------------------------------------------------------------------
/// @brief Converts angle from radians to degrees
///
/// @par Example use:
/// @snippet MathTester.cpp mecl_math_toDegrees_x
///
/// @param[in] i_Value_rx Angle in degrees to be converted into radians
/// @return Angle in radians
// --------------------------------------------------------------------------
template<typename T>
static T toDegrees_x(const T& i_Value_rx)
{
  return i_Value_rx * 180 / constants<T>::pi_x();
}


// --------------------------------------------------------------------------
/// @struct numeric_limits
/// @brief Contains numeric limits for type specified in template parameter T.
// --------------------------------------------------------------------------
template <typename T>
struct numeric_limits
{
  // --------------------------------------------------------------------------
  /// @brief Machine epsilon type specific for template parameter T.
  ///
  /// Returns machine epsilon which is the difference between 1 and the least 
  /// value greater than 1 that is representable.
  ///
  /// @return Epsilon
  // --------------------------------------------------------------------------
  // Tested through float64_t and float32_t type specific implementations.
  static const T epsilon_x(void)
  {
    return static_cast<T>(0);
  }
};

// --------------------------------------------------------------------------
/// @struct numeric_limits
/// @brief Contains numeric limits for type specified in template parameter T.
///
/// Template specialization of numeric limits for float64_t (double precision)
// --------------------------------------------------------------------------
template <>
struct numeric_limits<float64_t>
{
  // --------------------------------------------------------------------------
  /// @brief Machine epsilon specific for float64_t.
  ///
  /// Returns representation of epsilon for float64_t (double precision) type
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_epsilonf64_x
  ///
  /// @return Epsilon
  // --------------------------------------------------------------------------
  static float64_t epsilon_x(void)
  {
#ifdef __GNUC__
    const float64_t c_Epsilon_f64 = __DBL_EPSILON__;
    return c_Epsilon_f64;
#else
    return std::numeric_limits<float64_t>::epsilon();
#endif
  }

  // --------------------------------------------------------------------------
  /// @brief Positive infinity specific for float64_t.
  ///
  /// Returns representation of positive infinity for float64_t (double precision) type
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_infinityf64_x
  ///
  /// @return Infinity
  // --------------------------------------------------------------------------
  static float64_t infinity_x(void)
  {
#ifdef __GNUC__
    return __builtin_huge_val();
#else
    return std::numeric_limits<float64_t>::infinity();
#endif
  }
  // --------------------------------------------------------------------------
  /// @brief Smallest positive finite value of float64_t.
  ///
  /// Returns representation of smallest positive finite value for float32_t (double precision)
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_minf64_x
  ///
  /// @return Smallest positive finite value
  // --------------------------------------------------------------------------
  static float64_t min_x(void)
  {
#ifdef __GNUC__
    const float64_t c_DoubleMin_f64 = __DBL_MIN__;
    return c_DoubleMin_f64;
#else
    return std::numeric_limits<float64_t>::min();
#endif
  }

  // --------------------------------------------------------------------------
  /// @brief Smallest positive subnormal value of float64_t.
  ///
  /// Returns representation of smallest finite positive subnormal value for float32_t (double precision)
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_denormMinf64_x
  ///
  /// @return Smallest finite positive subnormal value
  // --------------------------------------------------------------------------
  static float64_t denormMin_x(void)
  {
#ifdef __GNUC__
    const float64_t c_DoubleDenormMin_f64 = __DBL_DENORM_MIN__;
    return c_DoubleDenormMin_f64;
#else
    return std::numeric_limits<float64_t>::denorm_min();
#endif
  }
  // --------------------------------------------------------------------------
  /// @brief Largest finite value of float64_t.
  ///
  /// Returns representation of largest finite value for float32_t (double precision)
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_maxf64_x
  ///
  /// @return Smallest finite value
  // --------------------------------------------------------------------------
  static float64_t max_x(void)
  {
#ifdef __GNUC__
    const float64_t c_DoubleMax_f64 = __DBL_MAX__;
    return c_DoubleMax_f64;
#else
    return std::numeric_limits<float64_t>::max();
#endif
   }

  // --------------------------------------------------------------------------
  /// @brief Lowest negative finite value of float64_t.
  ///
  /// Returns representation of lowest negative  value for float32_t (double precision)
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_lowest64_x
  ///
  /// @return Lowest finite value
  // --------------------------------------------------------------------------

  static float64_t lowest_x(void)
    {
  #ifdef __GNUC__
      return constants<float64_t>::minusOne_x() * numeric_limits<float64_t>::max_x();
  #else
      return constants<float64_t>::minusOne_x() * std::numeric_limits<float64_t>::max();
  #endif
    }

  // --------------------------------------------------------------------------
  /// @brief Checks whether a value is Not A Number
  ///
  /// Returns A non-zero value (true) if x is a NaN value; and zero (false) otherwise. 
  ///
  /// @return A non-zero value (true) if x is a NaN value; and zero (false) otherwise.
  // --------------------------------------------------------------------------
  static sint32_t isNaN(const float64_t& i_Value_f64)
  {
#ifdef __GNUC__
    return static_cast<sint32_t>( isnan(i_Value_f64) );
#else
    return static_cast<sint32_t>( _isnan(i_Value_f64) );
#endif
  }
};

// --------------------------------------------------------------------------
/// @struct numeric_limits
/// @brief Contains numeric limits for type specified in template parameter T.
///
/// Template specialization of numeric limits for float32_t (single precision)
// --------------------------------------------------------------------------
template <>
struct numeric_limits<float32_t>
{
  // --------------------------------------------------------------------------
  /// @brief Machine epsilon specific for float32_t.
  ///
  /// Returns representation of epsilon for float32_t (single precision) type
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_epsilonf32_x
  ///
  /// @return Epsilon
  // --------------------------------------------------------------------------
  static float32_t epsilon_x(void)
  {
#ifdef __GNUC__
    const float32_t c_Epsilon_f64 = __DBL_EPSILON__;
    return c_Epsilon_f64;
#else
    return std::numeric_limits<float32_t>::epsilon();
#endif
  }

  // --------------------------------------------------------------------------
  /// @brief Positive infinity specific for float32_t.
  ///
  /// Returns representation of positive infinity for float32_t (single precision) type
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_infinityf32_x
  ///
  /// @return Infinity
  // --------------------------------------------------------------------------
  static float32_t infinity_x(void)
  {
#ifdef __GNUC__
    return __builtin_huge_valf();
#else
    return std::numeric_limits<float32_t>::infinity();
#endif
  }

  // --------------------------------------------------------------------------
  /// @brief Smallest positive finite value of float32_t.
  ///
  /// Returns representation of smallest positive finite value for float32_t (single precision)
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_minf32_x
  ///
  /// @return Smallest positive finite value
  // --------------------------------------------------------------------------
  static float32_t min_x(void)
  {
#ifdef __GNUC__
    const float32_t c_Min_f32 = __FLT_MIN__;
    return c_Min_f32;
#else
    return std::numeric_limits<float32_t>::min();
#endif
  }
  // --------------------------------------------------------------------------
  /// @brief Smallest positive subnormal value of float32_t.
  ///
  /// Returns representation of smallest finite positive subnormal value for float32_t (double precision)
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_denormMinf32_x
  ///
  /// @return Smallest finite positive subnormal value
  // --------------------------------------------------------------------------
  static float32_t denormMin_x(void)
  {
#ifdef __GNUC__
    const float32_t c_DenormMin_f32 = __FLT_DENORM_MIN__;
    return c_DenormMin_f32;
#else
    return std::numeric_limits<float32_t>::denorm_min();
#endif
  }
  // --------------------------------------------------------------------------
  /// @brief Largest finite value of float32_t.
  ///
  /// Returns representation of largest finite value for float32_t (single precision)
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_maxf32_x
  ///
  /// @return Smallest finite value
  // --------------------------------------------------------------------------
  static float32_t max_x(void)
  {
#ifdef __GNUC__
    const float32_t c_Max_f32 = __FLT_MAX__;
    return c_Max_f32;
#else
    return std::numeric_limits<float32_t>::max();
#endif
  }

  // --------------------------------------------------------------------------
  /// @brief Lowest negative finite value of float32_t.
  ///
  /// Returns representation of lowest negative  value for float32_t (single precision)
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_lowestf32_x
  ///
  /// @return Smallest finite value
  // --------------------------------------------------------------------------
  static float32_t lowest_x(void)
  {
#ifdef __GNUC__
    return constants<float32_t>::minusOne_x() * numeric_limits<float32_t>::max_x();
#else
    return constants<float32_t>::minusOne_x() * std::numeric_limits<float32_t>::max();
#endif
  }
};

// --------------------------------------------------------------------------
/// @brief Minimum value of two comparable objects.
///
/// Returns the minimum value of two comparable objects. To work operator <(T) is needed.
///
/// @par Example use:
/// @snippet MathTester.cpp mecl_math_min_x
///
/// @param[in] i_Val1_rx Value 1 to be compared
/// @param[in] i_Val2_rx Value 2 to be compared
/// @return Minimum value
// --------------------------------------------------------------------------
template<typename T>
static T min_x(const T& i_Val1_rx, const T& i_Val2_rx)
{
  return (i_Val1_rx < i_Val2_rx) ? i_Val1_rx : i_Val2_rx;
}

// --------------------------------------------------------------------------
/// @brief Maximum value of two comparable objects.
///
/// Returns the maximum value of two comparable objects. To work operator <(T) is needed.
///
/// @par Example use:
/// @snippet MathTester.cpp mecl_math_max_x
///
/// @param[in] i_Val1_rx Value 1 to be compared
/// @param[in] i_Val2_rx Value 2 to be compared
/// @return Maximum value
// --------------------------------------------------------------------------
template<typename T>
static T max_x(const T& i_Val1_rx, const T& i_Val2_rx)
{
  return (i_Val2_rx < i_Val1_rx) ? i_Val1_rx : i_Val2_rx;
}

// --------------------------------------------------------------------------
/// @brief Negative value
///
/// Returns the negative value of a value of type T
///
///
/// @param[in] i_Val_rx Value
/// @return Negative value
// --------------------------------------------------------------------------
template<typename T>
static T neg_x(const T& i_Val_rx)
{
  return constants<T>::minusOne_x() * i_Val_rx;
}

// --------------------------------------------------------------------------
/// @brief Asbolute value
///
/// Returns the absolute value of an object that is comparable to 0.
/// To work both operator<(T, int) and operator-(T) are needed.
///
/// @par Example use:
/// @snippet MathTester.cpp mecl_math_abs_x
///
/// @param[in] i_Val_rx Value comparable to 0
/// @return Absolute value
// --------------------------------------------------------------------------
template<typename T>
static T abs_x(const T& i_Val_rx)
{
  return (constants<T>::zero_x() > i_Val_rx) ? neg_x<T>(i_Val_rx) : i_Val_rx;
}

// --------------------------------------------------------------------------
/// @brief Clamps a value
///
/// Returns a clamped value in Range [i_Min_x, i_Max_x]
///
/// @param[in] i_Min_x minimal desired value
/// @param[in] i_Max_x maximal desired value
/// 
/// @return The clamped value
// --------------------------------------------------------------------------
template<typename T>
T clamp_x(const T& i_Val_rx, const T& i_Min_x, const T& i_Max_x)
{
  return max_x<T>(i_Min_x, min_x<T>(i_Max_x, i_Val_rx));
}

// --------------------------------------------------------------------------
/// @brief Checks if value is about zero
///
/// Equality to zero is not the same as identity to zero (==). If value is
/// slightly bigger (=epsilon_x()) it is still be taken as equal zero.
///
/// @param[i] i_Val_rx Value to be checked
/// @return true, if input value is about zero, otherwise false
// --------------------------------------------------------------------------

template<typename T>
static bool_t isAboutZero_b(const T& i_Val_rx)
{
  return abs_x<T>(i_Val_rx) <= numeric_limits<T>::epsilon_x();
}

// --------------------------------------------------------------------------
/// @brief Checks if value is exactly zero
///
/// Checks if value is exactly the same than zero value of type.
///
/// @param[i] i_Val_rx Value to be checked
/// @return true, if input value is exactly zero, otherwise false
// --------------------------------------------------------------------------

template<typename T>
static bool_t isZero_b(const T& i_Val_rx)
{
  return i_Val_rx == math::constants<T>::zero_x();
}

// --------------------------------------------------------------------------
/// @brief Signum function
///
/// Returns one if test value is greater zero, minus one if its smaller \n
/// and zero if it equals zero
///
/// @param[i] i_Val_rx Value to be checked
/// @return true, if input value is exactly zero, otherwise false
// --------------------------------------------------------------------------
template<typename T>
static T sgn_x(const T& i_Val_rx)
{
  const T c_RetVal_x =
      i_Val_rx > constants<T>::zero_x() ? constants<T>::one_x()
                                        : ( isZero_b(i_Val_rx) ? constants<T>::zero_x()
                                                               : constants<T>::minusOne_x() );
  return c_RetVal_x;
}


// --------------------------------------------------------------------------
/// @brief Test if two value of type T are equal
/// Since values are of floating point type they are consider to be equal
/// Iff absolute value of their difference is smaller or equal epsilon
/// @return Returns true if values are equal
// --------------------------------------------------------------------------

template<typename T>
static bool_t equal_x(const T& i_Val1_rx, const T& i_Val2_rx)
{
  return ( numeric_limits<T>::epsilon_x() >= abs_x( i_Val1_rx - i_Val2_rx ) );
}

// --------------------------------------------------------------------------
/// @brief Test if two value of type T are unequal
/// Since values are of floating point type they are consider to be unequal
/// Iff absolute value of their difference is greater than epsilon
/// @return Returns true if values are unequal
// --------------------------------------------------------------------------

template<typename T>
static bool_t unequal_x(const T& i_Val1_rx, const T& i_Val2_rx)
{
  return not equal_x(i_Val1_rx, i_Val2_rx);
}

// --------------------------------------------------------------------------
/// @struct trigonometry
/// @brief Contains basic trigonometric functions
// --------------------------------------------------------------------------
template <typename T>
struct trigonometry
{
  /// Sine
  static T sin_x(const T& i_Angle_rx);
  
  /// Cosine
  static T cos_x(const T& i_Angle_rx);

  /// Tangent
  static T tan_x(const T& i_Angle_rx);

  /// Inverse sine
  static T asin_x(const T& i_Arc_rx);

  /// Inverse cosine
  static T acos_x(const T& i_Arc_rx);

  /// Inverse tangent
  static T atan2_x(const T& i_Arc1_rx, const T& i_Arc2_rx);
};

// --------------------------------------------------------------------------
/// @brief Get normalized angle value
///
/// Normalize angle to a value within the interval [-pi,+pi] resp. [-180°,+180°] deg
/// 
/// @par Example use:
/// @snippet MathTester.cpp mecl_math_getNormalizedAngle_x
///
/// @param[in] i_Angle_rx Angle value
/// @param[in] i_AngleUnit_e Input and returned angle value is in radians (e_Radians) or degrees (e_Degrees)
///
/// @return Normalized angle value
// --------------------------------------------------------------------------
template<typename T>
static T getNormalizedAngle_x(const T& i_Angle_rx, AngleUnit_e i_AngleUnit_e = e_Radians)
{
  const T c_TwoPi_x = constants<T>::pi_x() + constants<T>::pi_x();
  const T c_InputAngle_x = e_Degrees == i_AngleUnit_e ? toRadians_x(i_Angle_rx) : i_Angle_rx;
  const T c_Quotient_x =   c_InputAngle_x > constants<T>::zero_x()
                         ? floor(c_InputAngle_x / c_TwoPi_x)
                         : ceil(c_InputAngle_x / c_TwoPi_x);
  const T c_Remainder_x = c_InputAngle_x - c_TwoPi_x * c_Quotient_x;
  T v_OutputAngle_x = c_InputAngle_x;
  if ( c_Remainder_x > constants<T>::pi_x() )
  {
    v_OutputAngle_x = c_Remainder_x - c_TwoPi_x;
  }
  else if ( c_Remainder_x < neg_x(constants<T>::pi_x()) )
  {
    v_OutputAngle_x = c_Remainder_x + c_TwoPi_x;
  }
  return e_Degrees == i_AngleUnit_e ? toDegrees_x(v_OutputAngle_x) : v_OutputAngle_x;
}

// --------------------------------------------------------------------------
/// @struct trigonometry
/// @brief Contains basic trigonometric functions.
///
/// Template specialization of trigonometry functions for float64_t (double precision)
// --------------------------------------------------------------------------
template<>
struct trigonometry<float64_t>
{
  // --------------------------------------------------------------------------
  /// @brief Sine
  ///
  /// Returns sine of angle in radians, specific for type float64_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_sinf64_x
  ///
  /// @param[in] i_Angle_rf64 Angle
  /// @return Sine of angle in radians
  // --------------------------------------------------------------------------
  static float64_t sin_x(const float64_t& i_Angle_rf64)
  {
    return sin(i_Angle_rf64);
  }

  // --------------------------------------------------------------------------
  /// @brief Cosine
  ///
  /// Returns cosine of angle in radians, specific for type float64_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_cosf64_x
  ///
  /// @param[in] i_Angle_rf64 Angle
  /// @return Cosine of angle in radians
  // --------------------------------------------------------------------------
  static float64_t cos_x(const float64_t& i_Angle_rf64)
  {
    return cos(i_Angle_rf64);
  }

  // --------------------------------------------------------------------------
  /// @brief Tangent
  ///
  /// Returns tangent of angle in radians, specific for type float64_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_tanf64_x
  ///
  /// @param[in] i_Angle_rf64 Angle
  /// @return Tangent of angle in radians
  // --------------------------------------------------------------------------
  static float64_t tan_x(const float64_t& i_Angle_rf64)
  {
    return tan(i_Angle_rf64);
  }

  // --------------------------------------------------------------------------
  /// @brief Inverse sine
  ///
  /// Returns inverse sine of ratio in radians, specific for type float64_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_asinf64_x
  ///
  /// @param[in] i_Arc_rf64 Ratio
  /// @return Inverse sine of ratio in radians
  // --------------------------------------------------------------------------
  static float64_t asin_x(const float64_t& i_Arc_rf64)
  {
    return asin(i_Arc_rf64);
  }

  // --------------------------------------------------------------------------
  /// @brief Inverse cosine
  ///
  /// Returns inverse cosine of ratio in radians, specific for type float64_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_acosf64_x
  ///
  /// @param[in] i_Arc_rf64 Ratio
  /// @return Inverse cosine of ratio in radians
  // --------------------------------------------------------------------------
  static float64_t acos_x(const float64_t& i_Arc_rf64)
  {
    return acos(i_Arc_rf64);
  }

  // --------------------------------------------------------------------------
  /// @brief Inverse tangent
  ///
  /// Returns four-quadrant inverse tangent in radians, specific for type float64_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_atan2f64_x
  ///
  /// @param[in] i_Arc1_rf64 Y-coordinate 
  /// @param[in] i_Arc2_rf64 X-coordinate
  /// @return Inverse tangent in radians
  // --------------------------------------------------------------------------
  static float64_t atan2_x(const float64_t& i_Arc1_rf64, const float64_t& i_Arc2_rf64)
  {
    return atan2(i_Arc1_rf64, i_Arc2_rf64);
  }
};

// --------------------------------------------------------------------------
/// @struct trigonometry
/// @brief Contains basic trigonometric functions.
///
/// Template specialization of trigonometry functions for float32_t (single precision)
// --------------------------------------------------------------------------
template<>
struct trigonometry<float32_t>
{
  // --------------------------------------------------------------------------
  /// @brief Sine
  ///
  /// Returns sine of angle in radians, specific for type float32_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_sinf32_x
  ///
  /// @param[in] i_Angle_rf32 Angle
  /// @return Sine of angle in radians
  // --------------------------------------------------------------------------
  static float32_t sin_x(const float32_t& i_Angle_rf32)
  {
    return sinf(i_Angle_rf32);
  }

  // --------------------------------------------------------------------------
  /// @brief Cosine
  ///
  /// Returns cosine of angle in radians, specific for type float32_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_cosf32_x
  ///
  /// @param[in] i_Angle_rf32 Angle
  /// @return Cosine of angle in radians
  // --------------------------------------------------------------------------
  static float32_t cos_x(const float32_t& i_Angle_rf32)
  {
    return cosf(i_Angle_rf32);
  }

  // --------------------------------------------------------------------------
  /// @brief Tangent
  ///
  /// Returns tangent of angle in radians, specific for type float32_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_tanf32_x
  ///
  /// @param[in] i_Angle_rf32 Angle
  /// @return Tangent of angle in radians
  // --------------------------------------------------------------------------
  static float32_t tan_x(const float32_t& i_Angle_rf32)
  {
    return tanf(i_Angle_rf32);
  }

  // --------------------------------------------------------------------------
  /// @brief Inverse sine
  ///
  /// Returns inverse sine of ratio in radians, specific for type float32_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_sinf32_x
  /// @snippet MathTester.cpp mecl_math_asinf32_x
  ///
  /// @param[in] i_Arc_rf32 Ratio
  /// @return Inverse sine of ratio in radians
  // --------------------------------------------------------------------------
  static float32_t asin_x(const float32_t& i_Arc_rf32)
  {
    return asinf(i_Arc_rf32);
  }

  // --------------------------------------------------------------------------
  /// @brief Inverse cosine
  ///
  /// Returns inverse cosine of ratio in radians, specific for type float32_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_cosf32_x
  /// @snippet MathTester.cpp mecl_math_acosf32_x
  ///
  /// @param[in] i_Arc_rf32 Ratio
  /// @return Inverse cosine of ratio in radians
  // --------------------------------------------------------------------------
  static float32_t acos_x(const float32_t& i_Arc_rf32)
  {
    return acosf(i_Arc_rf32);
  }

  // --------------------------------------------------------------------------
  /// @brief Inverse tangent
  ///
  /// Returns four-quadrant inverse tangent in radians, specific for type float32_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_sinf32_x
  /// @snippet MathTester.cpp mecl_math_cosf32_x
  /// @snippet MathTester.cpp mecl_math_atan2f32_x
  ///
  /// @param[in] i_Arc1_rf32 Y composant
  /// @param[in] i_Arc2_rf32 X composant
  /// @return Inverse tangent in radians
  // --------------------------------------------------------------------------
  static float32_t atan2_x(const float32_t& i_Arc1_rf32, const float32_t& i_Arc2_rf32)
  {
    return atan2f(i_Arc1_rf32, i_Arc2_rf32);
  }
};

// --------------------------------------------------------------------------
/// @struct algebra
/// @brief Contains basic algebra functions
// --------------------------------------------------------------------------
template<typename T>
struct algebra
{
  static T sqrt_x(const T& i_Val_rx);
};

// --------------------------------------------------------------------------
/// @struct algebra
/// @brief Contains basic algebra functions
///
/// Template specialization of algebra functions for float32_t (single precision)
// --------------------------------------------------------------------------
template<>
struct algebra<float32_t>
{
  // --------------------------------------------------------------------------
  /// @brief Square root
  ///
  /// Returns square root of input argument, specific for type float32_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_algebra_sqrtf32_x
  ///
  /// @param[in] i_Val_rx Value
  /// @return Square root of value
  // --------------------------------------------------------------------------
  static float32_t sqrt_x(const float32_t& i_Val_rx)
  {
    return sqrtf(i_Val_rx);
  }
};

// --------------------------------------------------------------------------
/// @struct algebra
/// @brief Contains basic algebra functions
///
/// Template specialization of algebra functions for float64_t (double precision)
// --------------------------------------------------------------------------
template<>
struct algebra<float64_t>
{
  // --------------------------------------------------------------------------
  /// @brief Square root
  ///
  /// Returns square root of input argument, specific for type float64_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_algebra_sqrtf64_x
  ///
  /// @param[in] i_Val_rx Value
  /// @return Square root of value
  // --------------------------------------------------------------------------
  static float64_t sqrt_x(const float64_t& i_Val_rx)
  {
    return sqrt(i_Val_rx);
  }
};


// --------------------------------------------------------------------------
/// @struct numeric
/// @brief Contains basic numeric functions
// --------------------------------------------------------------------------
template<typename T>
struct numeric
{
  static T floor_x(const T& i_Val_rx);

  static T ceil_x(const T& i_Val_rx);
};

// --------------------------------------------------------------------------
/// @struct numeric
/// @brief Contains basic numeric functions
///
/// Template specialization of numeric function for float32_t (single precision)
// --------------------------------------------------------------------------
template<>
struct numeric<float32_t>
{
  // --------------------------------------------------------------------------
  /// @brief Floor
  ///
  /// Returns nearest lower integer value of input argument, specific for type float32_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_numeric_floorf32_x
  ///
  /// @param[in] i_Val_rx Value
  /// @return Nearest lower integer value
  // --------------------------------------------------------------------------
  static float32_t floor_x(const float32_t& i_Val_rx)
  {
    return floorf(i_Val_rx);
  }

  // --------------------------------------------------------------------------
  /// @brief Ceil
  ///
  /// Returns nearest higher integer value of input argument, specific for type float32_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_numeric_ceilf32_x
  ///
  /// @param[in] i_Val_rx Value
  /// @return Nearest lower integer value
  // --------------------------------------------------------------------------
  static float32_t ceil_x(const float32_t& i_Val_rx)
  {
    return ceilf(i_Val_rx);
  }

  // --------------------------------------------------------------------------
  /// @brief Round
  ///
  /// Returns nearest value of input argument, specific for type float32_t
  ///
  /// @param[in] i_Val_rx the Value to round
  /// @return nearest integer value
  // --------------------------------------------------------------------------
  static float32_t round_x(const float32_t& i_Val_rx)
  {
    float32_t v_ret_X;

    if (i_Val_rx > 0.0F)
    {
      v_ret_X = floor_x(i_Val_rx + 0.5F);
    }
    else
    {
      v_ret_X = ceil_x(i_Val_rx - 0.5F);
    }

    return v_ret_X;
  }
};

// --------------------------------------------------------------------------
/// @struct numeric
/// @brief Contains basic numeric functions
///
/// Template specialization of numeric function for float64_t (double precision)
// --------------------------------------------------------------------------
template<>
struct numeric<float64_t>
{
  // --------------------------------------------------------------------------
  /// @brief Floor
  ///
  /// Returns nearest lower integer value of input argument, specific for type float64_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_numeric_floorf64_x
  ///
  /// @param[in] i_Val_rx Value
  /// @return Nearest lower integer value
  // --------------------------------------------------------------------------
  static float64_t floor_x(const float64_t& i_Val_rx)
  {
    return floor(i_Val_rx);
  }

  // --------------------------------------------------------------------------
  /// @brief Ceil
  ///
  /// Returns nearest higher integer value of input argument, specific for type float64_t
  ///
  /// @par Example use:
  /// @snippet MathTester.cpp mecl_math_numeric_ceilf64_x
  ///
  /// @param[in] i_Val_rx Value
  /// @return Nearest lower integer value
  // --------------------------------------------------------------------------
  static float64_t ceil_x(const float64_t& i_Val_rx)
  {
    return ceil(i_Val_rx);
  }

  // --------------------------------------------------------------------------
  /// @brief Round
  ///
  /// Returns nearest value of input argument, specific for type float64_t
  ///
  /// @param[in] i_Val_rx the Value to round
  /// @return nearest integer value
  // --------------------------------------------------------------------------
  static float64_t round_x(const float64_t& i_Val_rx)
  {
    float64_t v_ret_X;

    if (i_Val_rx > 0.0F)
    {
      v_ret_X = floor_x(i_Val_rx + 0.5);
    }
    else
    {
      v_ret_X = ceil_x(i_Val_rx - 0.5);
    }

    return v_ret_X;
  }

};

// --------------------------------------------------------------------------
/// @struct numeric conversion
/// @brief Contains basic numeric conversion functions
// --------------------------------------------------------------------------
template<typename T>
struct numeric_conversions
{

};

template<>
struct numeric_conversions<float32_t>
{
  // --------------------------------------------------------------------------
  /// @brief Get number of decimals
  ///
  /// Returns the number of digits before decimal point
  ///
  /// @param[in] i_Input_f32 the input value
  /// @param[in] i_Base_f32 the base (e.g. 2, 10, etc)
  ///
  /// @return the number of digits before decimal point
  // --------------------------------------------------------------------------
  static sint32_t getNumDecimals_s32(float32_t i_Input_f32, float32_t i_Base_f32)
  {
    sint32_t v_NumDecimals_s32 = 0;

    float32_t v_AbsInput_f32 = abs_x(i_Input_f32);
    if (not isZero_b(v_AbsInput_f32) )
    {
      v_NumDecimals_s32 = static_cast<sint32_t>(numeric<float32_t>::round_x( logf( v_AbsInput_f32 ) / logf(i_Base_f32) ) );
      v_NumDecimals_s32 += 1;
    }

    return v_NumDecimals_s32;
  }

  // --------------------------------------------------------------------------
  /// @brief retrieve the mantissa and its exponent from float32
  ///
  /// retrieve the mantissa and the exponent from float32
  ///
  /// @param[in] i_Input_f32 the input value
  /// @param[in] i_Base_f32 the base (e.g. 2, 10, etc)
  /// @param[out] o_Exponent_s32 the exponent
  /// @param[out] o_Mantissa_f32 the mantissa
  // --------------------------------------------------------------------------
  static void getMantissaAndExponent_v(float32_t i_Input_f32, float32_t i_Base_f32, sint32_t& o_Exponent_s32, float32_t& o_Mantissa_f32)
  {
    o_Exponent_s32 = getNumDecimals_s32(i_Input_f32, i_Base_f32);
    o_Mantissa_f32 = i_Input_f32 / powf(i_Base_f32, static_cast<float32_t>(o_Exponent_s32));
  }

  // --------------------------------------------------------------------------
  /// @brief retrieve the mantissa and its exponent from a float32 
  ///
  /// retrieve the mantissa and the exponent from a float64 by specifying 
  /// the number of pre-decimal points, the decimal points and the base.
  /// This function will output the pre-Decimal part, the decimal part and
  /// the number of leading zeros after the decimal point.
  ///
  /// @param[in] i_Input_f32 the input value
  /// @param[in] i_NumPDP_s32 the number of pre-decimal points
  /// @param[in] i_NumDP_s32 the number of decimal points
  /// @param[in] i_Base_f32 the base (e.g. 2, 10, etc)
  /// @param[out] o_Exponent_s32 the exponent
  /// @param[out] o_PDP_s64 the pre decimal part containing the sign
  /// @param[out] o_DP_u64 the decimal part (always positive)
  /// @param[out] o_NumZerosDP_s32 the number of leading zeros after the decimal point.
  // --------------------------------------------------------------------------
  static void getMantissaAndExponent_v(float32_t i_Input_f32, sint32_t i_NumPDP_s32, sint32_t i_NumDP_s32, float32_t i_Base_f32, sint32_t& o_Exponent_s32, sint64_t& o_PDP_s64, uint64_t& o_DP_u64, sint32_t& o_NumZerosDP_s32)
  {
    float32_t v_Mantissa_f32;
    getMantissaAndExponent_v(i_Input_f32, i_Base_f32, o_Exponent_s32, v_Mantissa_f32);

    // get the number of pre decimal point positions and the difference between the desired pre decimal point position
    sint32_t v_PDPs_s32 = getNumDecimals_s32(v_Mantissa_f32, i_Base_f32);
    sint32_t v_Diff_s32 = i_NumPDP_s32 - v_PDPs_s32;

    // shift mantissa and exponent
    v_Mantissa_f32 *= powf(i_Base_f32, static_cast<float32_t>(v_Diff_s32));
    o_Exponent_s32 -= v_Diff_s32; 

    // save the pre decimal point and compute the rest
    o_PDP_s64       = static_cast<sint64_t>(v_Mantissa_f32);
    v_Mantissa_f32 -= static_cast<float32_t>(o_PDP_s64);

    // get the number of leading zeros after comma and shift the mantissa
    o_NumZerosDP_s32 = 0;
    uint64_t v_Base_u64 = static_cast<uint64_t>(i_Base_f32);
    for (sint32_t v_i_s32 = 0; v_i_s32 < i_NumDP_s32; ++v_i_s32)
    {
      uint64_t checkZero = static_cast<uint64_t>(v_Mantissa_f32 * static_cast<float32_t>(v_Base_u64) );
      if (checkZero == 0)
      {
        ++o_NumZerosDP_s32;
      }

      v_Base_u64 *= static_cast<uint64_t>(i_Base_f32);
    }

    // shift mantissa and save the decimal point, always set a positive number here, sign is saved in pre decimal point
    v_Mantissa_f32 *= powf(i_Base_f32, static_cast<float32_t>(i_NumDP_s32));

    if (v_Mantissa_f32 < 0.0F)
    {
      v_Mantissa_f32 *= -1.0F;
    }

    o_DP_u64 = static_cast<uint64_t>(numeric<float64_t>::round_x(v_Mantissa_f32));
  }

  // --------------------------------------------------------------------------
  /// @brief retrieve the mantissa and its exponent from a float64 
  ///
  /// retrieve the mantissa and the exponent from a float64 by specifying 
  /// the number of pre-decimal points, the decimal points and the base.
  /// This function will output the pre-Decimal part, the decimal part and
  /// the number of leading zeros after the decimal point.
  ///
  /// @param[in] i_Input_f32 the input value
  /// @param[in] i_NumPDP_s32 the number of pre-decimal points
  /// @param[in] i_NumDP_s32 the number of decimal points
  /// @param[in] i_Base_f32 the base (e.g. 2, 10, etc)
  /// @param[out] o_String_pc pointer to a char array 
  /// (careful, this arrays needs to be big enough) that will contain the 
  /// printf(%f)-like formatted float64 value
  // --------------------------------------------------------------------------
  static void getMantissaAndExponent_v(float32_t i_Input_f32, sint32_t i_NumPDP_s32, sint32_t i_NumDP_s32, float32_t i_Base_f32, char_t* o_String_pc)
  {
    sint32_t v_Exponent_s32;
    sint64_t v_PreDecimalPoint_s64;
    uint64_t v_DecimalPoint_u64;
    sint32_t v_NumZeros_s32;

    getMantissaAndExponent_v(i_Input_f32, i_NumPDP_s32, i_NumDP_s32, i_Base_f32, v_Exponent_s32, v_PreDecimalPoint_s64, v_DecimalPoint_u64, v_NumZeros_s32);

    // add sign
    // if input -> -0.01F -> Result: 0.1 -> cast to sint32_t -> 0. We will lose the sign if the pre decimal point is zero, so add the sign in this special case
    sint32_t v_Count_s32 = 0;
    if (i_Input_f32 < 0.0F && v_PreDecimalPoint_s64 >= 0)
    {
      v_Count_s32 += sprintf(&o_String_pc[v_Count_s32], "-");
    }

    // add the pre decimal part
    v_Count_s32 += sprintf(&o_String_pc[v_Count_s32], "%lld.", v_PreDecimalPoint_s64);

    // print all leading zeros
    sint32_t v_zeros_s32;
    for (v_zeros_s32 = 0; v_zeros_s32 < v_NumZeros_s32; ++v_zeros_s32)
    {
      v_Count_s32 += sprintf(&o_String_pc[v_Count_s32], "0");
    }

    // print the decimal point, but skip this if max number of decimal points is reached
    if (v_zeros_s32 < i_NumDP_s32)
    {
      v_Count_s32 += sprintf(&o_String_pc[v_Count_s32], "%llu", v_DecimalPoint_u64);
    }

    // print the exponent if we have any
    if (v_Exponent_s32 != 0)
    {
      v_Count_s32 += sprintf(&o_String_pc[v_Count_s32], "e%i", v_Exponent_s32);
    }
  }

};

template<>
struct numeric_conversions<float64_t>
{
  // --------------------------------------------------------------------------
  /// @brief Get number of decimals
  ///
  /// Returns the number of digits before decimal point
  ///
  /// @param[in] i_Input_f64 the input value 
  /// @param[in] i_Base_f64 the base (e.g. 2, 10, etc)
  ///
  /// @return the number of digits before decimal point
  // --------------------------------------------------------------------------
  static sint32_t getNumDecimals_s32(float64_t i_Input_f64, float64_t i_Base_f64)
  {
    sint32_t v_NumDecimals_s32 = 0;

    float64_t v_AbsInput_f64 = abs_x(i_Input_f64);
    if (not isZero_b(v_AbsInput_f64) )
    {
      v_NumDecimals_s32 = static_cast<sint32_t>(numeric<float64_t>::round_x( log( v_AbsInput_f64 ) / log(i_Base_f64) ) );
      v_NumDecimals_s32 += 1;
    }

    return v_NumDecimals_s32;
  }

  // --------------------------------------------------------------------------
  /// @brief retrieve the mantissa and its exponent from float64
  ///
  /// retrieve the mantissa and the exponent from float64
  ///
  /// @param[in] i_Input_f64 the input value
  /// @param[in] i_Base_f64 the base (e.g. 2, 10, etc)
  /// @param[out] o_Exponent_s32 the exponent
  /// @param[out] o_Mantissa_f64 the mantissa
  // --------------------------------------------------------------------------
  static void getMantissaAndExponent_v(float64_t i_Input_f64, float64_t i_Base_f64, sint32_t& o_Exponent_s32, float64_t& o_Mantissa_f64)
  {
    o_Exponent_s32 = getNumDecimals_s32(i_Input_f64, i_Base_f64);
    o_Mantissa_f64 = i_Input_f64 / pow(i_Base_f64, static_cast<float64_t> (o_Exponent_s32) );
  }

  // --------------------------------------------------------------------------
  /// @brief retrieve the mantissa and its exponent from a float64 
  ///
  /// retrieve the mantissa and the exponent from a float64 by specifying 
  /// the number of pre-decimal points, the decimal points and the base.
  /// This function will output the pre-Decimal part, the decimal part and
  /// the number of leading zeros after the decimal point.
  ///
  /// @param[in] i_Input_f64 the input value
  /// @param[in] i_NumPDP_s32 the number of pre-decimal points
  /// @param[in] i_NumDP_s32 the number of decimal points
  /// @param[in] i_Base_f64 the base (e.g. 2, 10, etc)
  /// @param[out] o_Exponent_s32 the exponent
  /// @param[out] o_PDP_s64 the pre decimal part containing the sign
  /// @param[out] o_DP_u64 the decimal part (always positive)
  /// @param[out] o_NumZerosDP_s32 the number of leading zeros after the decimal point.
  // --------------------------------------------------------------------------
  static void getMantissaAndExponent_v(float64_t i_Input_f64, sint32_t i_NumPDP_s32, sint32_t i_NumDP_s32, float64_t i_Base_f64, sint32_t& o_Exponent_s32, sint64_t& o_PDP_s64, uint64_t& o_DP_u64, sint32_t& o_NumZerosDP_s32)
  {
    float64_t v_Mantissa_f64;
    getMantissaAndExponent_v(i_Input_f64, i_Base_f64, o_Exponent_s32, v_Mantissa_f64);

    // get the number of pre decimal point positions and the difference between the desired pre decimal point position
    sint32_t v_PDPs_s32 = getNumDecimals_s32(v_Mantissa_f64, i_Base_f64);
    sint32_t v_Diff_s32 = i_NumPDP_s32 - v_PDPs_s32;

    // shift mantissa and exponent
    v_Mantissa_f64 *= pow(i_Base_f64, static_cast<float64_t>(v_Diff_s32));
    o_Exponent_s32 -= v_Diff_s32; 

    // save the pre decimal point and compute the rest
    o_PDP_s64       = static_cast<sint64_t>(v_Mantissa_f64);
    v_Mantissa_f64 -= static_cast<float64_t>(o_PDP_s64);

    // get the number of leading zeros after comma and shift the mantissa
    o_NumZerosDP_s32 = 0;
    uint64_t v_Base_u64 = static_cast<uint64_t>(i_Base_f64);
    for (sint32_t v_i_s32 = 0; v_i_s32 < i_NumDP_s32; ++v_i_s32)
    {
      uint64_t checkZero = static_cast<uint64_t>(v_Mantissa_f64 * static_cast<float64_t> (v_Base_u64) );
      if (checkZero == 0)
      {
        ++o_NumZerosDP_s32;
      }

      v_Base_u64 *= static_cast<uint64_t>(i_Base_f64);
    }

    // shift mantissa and save the decimal point, always set a positive number here, sign is saved in pre decimal point
    v_Mantissa_f64 *= pow(i_Base_f64, static_cast<float64_t>(i_NumDP_s32));

    if (v_Mantissa_f64 < 0.0)
    {
      v_Mantissa_f64 *= -1.0;
    }

    o_DP_u64 = static_cast<uint64_t>(numeric<float64_t>::round_x(v_Mantissa_f64));
  }

  // --------------------------------------------------------------------------
  /// @brief retrieve the mantissa and its exponent from a float64 
  ///
  /// retrieve the mantissa and the exponent from a float64 by specifying 
  /// the number of pre-decimal points, the decimal points and the base.
  /// This function will output the pre-Decimal part, the decimal part and
  /// the number of leading zeros after the decimal point.
  ///
  /// @param[in] i_Input_f64 the input value
  /// @param[in] i_NumPDP_s32 the number of pre-decimal points
  /// @param[in] i_NumDP_s32 the number of decimal points
  /// @param[in] i_Base_f64 the base (e.g. 2, 10, etc)
  /// @param[out] o_String_pc pointer to a char array 
  /// (careful, this arrays needs to be big enough) that will contain the 
  /// printf(%f)-like formatted float64 value
  // --------------------------------------------------------------------------
  static void getMantissaAndExponent_v(float64_t i_Input_f64, sint32_t i_NumPDP_s32, sint32_t i_NumDP_s32, float64_t i_Base_f64, char_t* o_String_pc)
  {
    sint32_t v_Exponent_s32;
    sint64_t v_PreDecimalPoint_s64;
    uint64_t v_DecimalPoint_u64;
    sint32_t v_NumZeros_s32;

    getMantissaAndExponent_v(i_Input_f64, i_NumPDP_s32, i_NumDP_s32, i_Base_f64, v_Exponent_s32, v_PreDecimalPoint_s64, v_DecimalPoint_u64, v_NumZeros_s32);

    // add sign
    // if input -> -0.01F -> Result: 0.1 -> cast to sint32_t -> 0. We will lose the sign if the pre decimal point is zero, so add the sign in this special case
    sint32_t v_Count_s32 = 0;
    if (i_Input_f64 < 0.0 && v_PreDecimalPoint_s64 >= 0)
    {
      v_Count_s32 += sprintf(&o_String_pc[v_Count_s32], "-");
    }

    // add the pre decimal part
    v_Count_s32 += sprintf(&o_String_pc[v_Count_s32], "%lld.", v_PreDecimalPoint_s64);

    // print all leading zeros
    sint32_t v_zeros_s32;
    for (v_zeros_s32 = 0; v_zeros_s32 < v_NumZeros_s32; ++v_zeros_s32)
    {
      v_Count_s32 += sprintf(&o_String_pc[v_Count_s32], "0");
    }

    // print the decimal point, but skip this if max number of decimal points is reached
    if (v_zeros_s32 < i_NumDP_s32)
    {
      v_Count_s32 += sprintf(&o_String_pc[v_Count_s32], "%llu", v_DecimalPoint_u64);
    }

    // print the exponent if we have any
    if (v_Exponent_s32 != 0)
    {
      v_Count_s32 += sprintf(&o_String_pc[v_Count_s32], "e%i", v_Exponent_s32);
    }
  }

};

} // namespace math
} // namespace mecl

#endif // MECL_MATH_MATH_H_
/// @}
/// @}
