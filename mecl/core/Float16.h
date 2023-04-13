//----------------------------------------------------------------------------
/// @file Float16.h
/// @brief Definition and implementation of Float16 class
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com
/// @date 03/03/2015

//----------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{


#ifndef SRC_CORE_FLOAT16_H_
#define SRC_CORE_FLOAT16_H_

#include "MeclTypes.h"

namespace mecl
{
namespace core
{
//--------------------------------------------------------------------------
/// @class Float16
/// @brief A 16-bit floating point number class
///
/// An instance of class Float16 can represent positive and negative numbers whose
///	magnitude is between roughly 6.1e-5 and 6.5e+4 with a relative
///	error of 9.8e-4; numbers smaller than 6.1e-5 can be represented
///	with an absolute error of 6.0e-8.  All integers from -2048 to
///	+2048 can be represented exactly.
///
///---------------------------------------------------------------------------
///
/// Implementation:
///
/// Representation of a float32_t (bit pattern):
///
///	We assume that a float, f, is an IEEE 754 single-precision
///	floating point number, whose bits are arranged as follows:
///
///	    31 (msb)
///	    |
///	    | 30     23
///	    | |      |
///	    | |      | 22                    0 (lsb)
///	    | |      | |                     |
///	    X XXXXXXXX XXXXXXXXXXXXXXXXXXXXXXX
///
///	    s e        m
///
///	S is the sign-bit, e is the exponent and m is the significand.
///
///	If e is between 1 and 254, f is a normalized number:
///
///	            s    e-127
///	    f = (-1)  * 2      * 1.m
///
///	If e is 0, and m is not zero, f is a denormalized number:
///
///	            s    -126
///	    f = (-1)  * 2      * 0.m
///
///	If e and m are both zero, f is zero:
///
///	    f = 0.0
///
///	If e is 255, f is an "infinity" or "not a number" (NAN),
///	depending on whether m is zero or not.
///
///	Examples:
///
///	    0 00000000 00000000000000000000000 = 0.0
///	    0 01111110 00000000000000000000000 = 0.5
///	    0 01111111 00000000000000000000000 = 1.0
///	    0 10000000 00000000000000000000000 = 2.0
///	    0 10000000 10000000000000000000000 = 3.0
///	    1 10000101 11110000010000000000000 = -124.0625
///	    0 11111111 00000000000000000000000 = +infinity
///	    1 11111111 00000000000000000000000 = -infinity
///	    0 11111111 10000000000000000000000 = NAN
///	    1 11111111 11111111111111111111111 = NAN
///
/// Representation of a Float16 (bit pattern):
///
///	Here is the bit-layout for a Float16 number, h:
///
///	    15 (msb)
///	    |
///	    | 14  10
///	    | |   |
///	    | |   | 9        0 (lsb)
///	    | |   | |        |
///	    X XXXXX XXXXXXXXXX
///
///	    s e     m
///
///	S is the sign-bit, e is the exponent and m is the significand.
///
///	If e is between 1 and 30, h is a normalized number:
///
///	            s    e-15
///	    h = (-1)  * 2     * 1.m
///
///	If e is 0, and m is not zero, h is a denormalized number:
///
///	            S    -14
///	    h = (-1)  * 2     * 0.m
///
///	If e and m are both zero, h is zero:
///
///	    h = 0.0
///
///	If e is 31, h is an "infinity" or "not a number" (NAN),
///	depending on whether m is zero or not.
///
///	Examples:
///
///	    0 00000 0000000000 = 0.0
///	    0 01110 0000000000 = 0.5
///	    0 01111 0000000000 = 1.0
///	    0 10000 0000000000 = 2.0
///	    0 10000 1000000000 = 3.0
///	    1 10101 1111000001 = -124.0625
///	    0 11111 0000000000 = +infinity
///	    1 11111 0000000000 = -infinity
///	    0 11111 1000000000 = NAN
///	    1 11111 1111111111 = NAN
///
///
/// Conversions from float to float16_t are not supported yet.
// --------------------------------------------------------------------------

class Float16
{
public:

  /// Default constructor
  Float16(void)
  : val_u16(0U)
  {};

  /// Contstructor with integer 16 base value (bit-pattern)
  explicit Float16(const uint16_t& i_Val_u16)
  : val_u16(i_Val_u16)
  {};

  /// Method Returns internal 16  bit-pattern as integer
  uint16_t toUInt16_u16() const
  {
    return this->val_u16;
  }

  /// Method converts this Float16 instance into a single precision float
  float32_t toFloat_f32() const
  {
    uint32_t v_S_u32 = (this->val_u16 >> 15) & 0x00000001;
    uint32_t v_E_u32 = (this->val_u16 >> 10) & 0x0000001f; // exponent
    uint32_t v_M_u32 = this->val_u16       & 0x000003ff; // mantissa

    if (0 == v_E_u32) {
      if (0 == v_M_u32) { // plus or minus zero
        v_S_u32 <<= 31;
      } else { // exponent is zero
        // denormalize number -- renormalize it
        while ( (v_M_u32 & 0x00000400) != 0x00000000) {
          v_M_u32 <<= 1;
          v_E_u32 -= 1;
        };
        v_E_u32 += 1;
        v_M_u32 &= ~0x00000400;
      }
    } else if (31 == v_E_u32) {
      if (0 == v_M_u32) { // positive or negative infinity
        v_S_u32 <<= 31;
        v_S_u32 |= 0x7f800000;
      } else { // NaN -- preserve sign and significant bits
        v_S_u32 <<= 31;
        v_S_u32 |= 0x7f800000;
        v_S_u32 |= (v_M_u32 << 13);
      }
    } else { // neither NaN nor infinity nor denormalization necessary
      v_E_u32 += (127 - 15);
      v_M_u32 <<= 13;
      // assemble
      v_S_u32 <<=31;
      v_S_u32 |= (v_E_u32 << 23);
      v_S_u32 |= v_M_u32;
    }

    float32_t *v_0_pf32 = reinterpret_cast<float32_t*>(&v_S_u32);
    float32_t v_O_f32 = *( v_0_pf32 );
    return v_O_f32;
  };

  /// Method converts this Float16 instance into a double precision float
  float64_t toDouble_f64() const
  {
    return static_cast<float64_t>(this->toFloat_f32());
  };

  /// Method converts this Float16 instance into a single precision float (second version)
  void toFloat_v(float32_t& o_Val_f32) const
  {
    o_Val_f32 = this->toFloat_f32();
    return;
  };

  /// Method converts this Float16 instance into a double precision float (second version)
  void toDouble_v(float64_t& o_Val_f64) const
  {
    o_Val_f64 = this->toDouble_f64();
  };

  /// operator for casting a Float16 to its internal bit pattern representing a 16 bit integer
  operator uint16_t() const { return this->toUInt16_u16(); } ;

  /// operator for casting a Float16 to a single precision floating point value
  operator float32_t() const { return this->toFloat_f32();};

  /// operator for casting a Float16 to a single precision floating point value
  operator float64_t() const { return this->toDouble_f64(); };

private:
  uint16_t val_u16;
};

} // core
} // mecl


#endif /* SRC_CORE_FLOAT16_H_ */
/// @}
/// @}
