// --------------------------------------------------------------------------
/// @file BitSet.h
/// @brief Easy access to bits on underlying binary data
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Michael Schulte (michael.schulte@magna.com)
///
// --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef MECL_CORE_BITSET_H_
#define MECL_CORE_BITSET_H_

#include "MeclAssert.h"
#include "Helpers.h"

namespace mecl
{
namespace core
{

/// Calculates the BitSet mask given a start and end index.
/// Calculation is done at compile time.
/// Example: c_StartIndex_u32 = 3, c_EndIndex_u32 = 6 : Mask = 1111000 = 0x78
/// @tparam c_StartIndex_u32 start index of the mask - 0 = start at lowest bit
/// @tparam c_EndIndex_u32 end index of the mask - if same as c_StartIndex_u32 the mask is only 1 bit
/// @tparam c_MaxIndex_u32 maximum used c_EndIndex_u32
template <uint32_t c_StartIndex_u32, uint32_t c_EndIndex_u32, uint32_t c_MaxIndex_u32 = c_EndIndex_u32>
struct CalcMask
{
  typedef typename ConditionalType<(c_MaxIndex_u32 > 31U), uint64_t, uint32_t>::type MaskType;
  /// Create mask with N bits set = (2^N)-1 and shift it by the start index value.
  /// N = (end - start) + 1, 2 ^ N = 1 << N
  /// This formula does not work for N = 32 with 32-bit values and N = 64 with 64-bit values, hence specializations below.
  static const MaskType c_Mask_x = static_cast<MaskType>(((static_cast<MaskType>(1ULL) << (c_EndIndex_u32 - c_StartIndex_u32 + 1U)) - 1U) << c_StartIndex_u32);
private:
  /// Check if maxIndex is < 64 - types greater 64 bits are not supported.
  /// Additional check c_EndIndex_u32 < c_MaxIndex_u32 in order to not get a recursive error result by the compiler.
  static const typename ConditionalType<(c_EndIndex_u32 < c_MaxIndex_u32) || (c_MaxIndex_u32 < 64U), bool_t, void>::type c_CheckLower64_b = true;
};

/// Specialization: 32-bits, all bits set.
/// Needed because the shift operation to calculate c_Mask_x here would run out of bounds.
template <>
struct CalcMask<0UL, 31UL, 31UL>
{
  typedef uint32_t MaskType;
  /// Special case for 32 bit: all bits set
  static const MaskType c_Mask_x = 0xFFFFFFFFUL;
};

/// Specialization: 64-bits, all bits set.
/// Needed because the shift operation to calculate c_Mask_x here would run out of bounds.
template <>
struct CalcMask<0UL, 63UL, 63UL>
{
  typedef uint64_t MaskType;
  /// Special case for 64 bit: all bits set
  static const MaskType c_Mask_x = 0xFFFFFFFFFFFFFFFFULL;
};


/// Access to bits with static functions.
/// This is a helper class that can be used without additional memory to hold a reference to the underlying memory.
/// @tparam c_StartIndex_u32 start index of the mask - 0 = start at lowest bit
/// @tparam c_EndIndex_u32 end index of the mask - if same as c_StartIndex_u32 the mask is only 1 bit
template <uint32_t c_StartIndex_u32, uint32_t c_EndIndex_u32 = c_StartIndex_u32>
class BitAccess
{
public:
  /// MaskType is uint32_t for indices up to 31, else uint64_t.
  typedef typename CalcMask<c_StartIndex_u32, c_EndIndex_u32>::MaskType MaskType;

  /// Underlying mask where all bits are set from c_StartIndex_u32 to c_EndIndex_u32.
  static const MaskType c_Mask_x = CalcMask<c_StartIndex_u32, c_EndIndex_u32>::c_Mask_x;

  /// Sets the underlying value.
  /// @param i_Value_x the value to set.
  /// @tparam T the underlying type for the raw bit access
  /// @tparam U the underlying type of the value to set. This makes calling the function with constants a lot easier.
  template <typename T, typename U>
  static void setValue_v(T& o_Byte_rx, const U& i_Value_rx)
  {
    o_Byte_rx = static_cast<T>(
        ((static_cast<T>(i_Value_rx) << c_StartIndex_u32) & c_Mask_x) | (o_Byte_rx & (~c_Mask_x)));
  }

  /// Overloaded setValue_v for bool_t to set all bits to either 1 or 0
  /// @param i_Value_rb if true: all bits are set, if false: all bits are cleared.
  template <typename T>
  static void setValue_v(T& o_Byte_rx, const bool_t& i_Value_rb)
  {
    if (true == i_Value_rb)
    {
      set_v(o_Byte_rx);
    }
    else
    {
      clear_v(o_Byte_rx);
    }
  }

  /// Gets the underlying value
  /// @tparam T the underlying type for the raw bit access
  /// @return the value contained in the bitset
  template <typename T>
  static T getValue_x(const T& i_Byte_rx)
  {
    return static_cast<T>((i_Byte_rx & c_Mask_x) >> c_StartIndex_u32);
  }

  /// Checks if all bits are set
  /// @tparam T the underlying type for the raw bit access
  /// @return true if all bits are set else false.
  template <typename T>
  static bool_t isSet_b(const T& i_Byte_rx)
  {
    return (c_Mask_x == (i_Byte_rx & c_Mask_x));
  }

  /// Checks if a bit is set a specific position
  /// @tparam T the underlying type for the raw bit access
  /// @return true if the specific bit is set else false.
  template <typename T>
  static bool_t isSetAt_b(const T& i_Byte_rx, uint32_t i_Position_u32)
  {
    const MaskType c_MaskBit_x = (1U << (i_Position_u32 + c_StartIndex_u32));
    return c_MaskBit_x == (i_Byte_rx & c_MaskBit_x);
  }

  /// Sets all bits to 1.
  /// @tparam T the underlying type for the raw bit access
  template <typename T>
  static void set_v(T& o_Byte_rx)
  {
    o_Byte_rx = static_cast<T>(o_Byte_rx | c_Mask_x);
  }

  /// Sets a specific bit to 1.
  /// @tparam T the underlying type for the raw bit access
  /// @param i_Position_u32 position to set.
  template <typename T>
  static void setAt_v(T& o_Byte_rx, uint32_t i_Position_u32)
  {
    const MaskType c_MaskBit_x = (1U << (i_Position_u32 + c_StartIndex_u32));
    o_Byte_rx = static_cast<T>(o_Byte_rx | c_MaskBit_x);
  }

  /// Sets all bits to 0.
  /// @tparam T the underlying type for the raw bit access
  template <typename T>
  static void clear_v(T& o_Byte_rx)
  {
    o_Byte_rx = static_cast<T>(o_Byte_rx & (~c_Mask_x));
  }

  /// Sets a specific bit to 0.
  /// @tparam T the underlying type for the raw bit access
  /// @param i_Position_u32 position to clear.
  template <typename T>
  static void clearAt_v(T& o_Byte_rx, uint32_t i_Position_u32)
  {
    const MaskType c_MaskBit_x = (1U << (i_Position_u32 + c_StartIndex_u32));
    o_Byte_rx = static_cast<T>(o_Byte_rx & (~c_MaskBit_x));
  }

  /// Flips (or toggles) all bits.
  /// @tparam T the underlying type for the raw bit access
  template <typename T>
  void flip_v(T& o_Byte_rx)
  {
    o_Byte_rx = static_cast<T>(o_Byte_rx ^ c_Mask_x);
  }

  /// Flips (or toggles) a bit at a specific position.
  /// @param i_Position_u32 position to flip.
  /// @tparam T the underlying type for the raw bit access
  template <typename T>
  void flipAt_v(T& o_Byte_rx, uint32_t i_Position_u32)
  {
    const MaskType c_MaskBit_x = (1U << (i_Position_u32 + c_StartIndex_u32));
    o_Byte_rx = static_cast<T>(o_Byte_rx ^ c_MaskBit_x);
  }

  /// Increments the contents of the underlying bitset by 1. If all bits are set the value is 0 after increment.
  /// @tparam T the underlying type for the raw bit access
  template <typename T>
  static void inc_v(T& o_Byte_rx)
  {
    o_Byte_rx = static_cast<T>((o_Byte_rx + 1U) & c_Mask_x);
  }
};


// PRQA S 1051 15 // comment contains example code
/// This BitSet implementation holds a reference to the underlying raw data and makes access to binary data simpler.
/// @tparam The underlying raw type used to access the raw data
/// @tparam c_StartIndex_u32 start index of the mask - 0 = start at lowest bit
/// @tparam c_EndIndex_u32 end index of the mask - if same as c_StartIndex_u32 the mask is only 1 bit
///
/// @par Example usage:
/// @code{.cpp}
/// uint32_t v_RawItem_u32 = 0U;
/// // create BitSet for bits 16-23 = 00XX0000 (0 = lowest bit, 31 = highest bit)
/// mecl::core::BitSet<uint32_t,16,23> v_BitSet_o(v_RawItem_u32);
/// v_BitSet_o = 0x12UL;
/// EXPECT_EQ(0x00120000, v_RawItem_u32);
/// @endcode
///
template <typename T, uint32_t c_StartIndex_u32, uint32_t c_EndIndex_u32>
class BitSet
{
public:
  /// Mask applied to values in this bitset - all relevant bits set.
  static const typename CalcMask<c_StartIndex_u32, c_EndIndex_u32>::MaskType c_Mask_x
    = BitAccess<c_StartIndex_u32, c_EndIndex_u32>::c_Mask_x;

  /// Number of bits on the underlying bitset
  static const uint32_t c_Length_u32 = c_EndIndex_u32 - c_StartIndex_u32 + 1U;

  /// Constructs a BitSet given access to the raw data item
  /// @param b_Item_rx the underlying raw data
  explicit BitSet(T& b_Item_rx) : rawData_rx(b_Item_rx) {}

  /// Sets the underlying value.
  /// @param i_Value_x the value to set.
  template <typename U>
  void setValue_v(U i_Value_x)
  {
    BitAccess<c_StartIndex_u32, c_EndIndex_u32>::setValue_v(rawData_rx, i_Value_x);
  }

  /// Gets the underlying value
  /// @return the value contained in the bitset
  T getValue_x() const
  {
    return BitAccess<c_StartIndex_u32, c_EndIndex_u32>::getValue_x(rawData_rx);
  }

  /// Checks if all bits are set
  /// @return true if all bits are set else false.
  bool_t isSet_b() const
  {
    return BitAccess<c_StartIndex_u32, c_EndIndex_u32>::isSet_b(rawData_rx);
  }

  /// Checks if a bit is set a specific position
  /// @return true if the specific bit is set else false.
  bool_t isSetAt_b(uint32_t i_Position_u32) const
  {
    return BitAccess<c_StartIndex_u32, c_EndIndex_u32>::isSetAt_b(rawData_rx, i_Position_u32);
  }

  /// Sets all bits to 1.
  void set_v()
  {
    BitAccess<c_StartIndex_u32, c_EndIndex_u32>::set_v(rawData_rx);
  }

  /// Sets a specific bit to 1.
  /// @param i_Position_u32 position to set.
  void setAt_v(uint32_t i_Position_u32)
  {
    BitAccess<c_StartIndex_u32, c_EndIndex_u32>::setAt_v(rawData_rx, i_Position_u32);
  }

  /// Sets all bits to 0.
  void clear_v()
  {
    BitAccess<c_StartIndex_u32, c_EndIndex_u32>::clear_v(rawData_rx);
  }

  /// Sets a specific bit to 0.
  /// @param i_Position_u32 position to clear.
  void clearAt_v(uint32_t i_Position_u32)
  {
    BitAccess<c_StartIndex_u32, c_EndIndex_u32>::clearAt_v(rawData_rx, i_Position_u32);
  }

  /// Flips (or toggles) all bits.
  void flip_v()
  {
    BitAccess<c_StartIndex_u32, c_EndIndex_u32>::flip_v(rawData_rx);
  }

  /// Convenience bit-flip operator.
  /// @return reference to this object.
  BitSet<T, c_StartIndex_u32, c_EndIndex_u32>& operator~()
  {
    flip_v();
    return *this;
  }

  /// Flips (or toggles) a bit at a specific position.
  /// @param i_Position_u32 position to flip.
  void flipAt_v(uint32_t i_Position_u32)
  {
    BitAccess<c_StartIndex_u32, c_EndIndex_u32>::flipAt_v(rawData_rx, i_Position_u32);
  }

  /// Increments the contents of the underlying bitset by 1. If all bits are set the value is 0 after increment.
  void inc_v()
  {
    BitAccess<c_StartIndex_u32, c_EndIndex_u32>::inc_v(rawData_rx);
  }

  /// Convenience prefix operator implementation. Increments the contents of the underlying bitset by 1. If all bits are set the value is 0 after increment.
  /// @return reference to this object.
  BitSet<T, c_StartIndex_u32, c_EndIndex_u32>& operator++()
  {
    inc_v();
    return *this;
  }

  // PRQA S 2639 4 // underlying content is copied as is (binary copy - no new/delete used here)
  /// Operator that sets the contents of the bitset to a specific value.
  /// @param i_Value_x value to set
  template <typename U>
  BitSet<T, c_StartIndex_u32, c_EndIndex_u32>& operator=(const U& i_Value_rx)
  {
    setValue_v(i_Value_rx);
    return *this;
  }

private:
  T& rawData_rx;
};

// PRQA S 1051 10 // comment contains example code
/// Simplified BitSet access with standard types.
///
/// @par Example usage:
/// @code{.cpp}
/// uint32_t v_RawItem_u32 = 0U;
/// // create BitSet for bits 5-9 (0 = lowest bit, 31 = highest bit)
/// mecl::core::Bits<5,9>::Access32 v_BitSet_o(v_RawItem_u32);
/// v_BitSet_o.set_v();
/// EXPECT_EQ(0x000003E0, v_RawItem_u32);
/// @endcode
template <uint32_t c_StartIndex_u32, uint32_t c_EndIndex_u32>
struct Bits
{
  /// Default BitSet implementation using uint64_t for 64-bit access to raw data
  typedef BitSet<uint64_t, c_StartIndex_u32, c_EndIndex_u32> Access64;
  /// Default BitSet implementation using uint32_t for 32-bit access to raw data
  typedef BitSet<uint32_t, c_StartIndex_u32, c_EndIndex_u32> Access32;
  /// Default BitSet implementation using uint16_t for 16-bit access to raw data
  typedef BitSet<uint16_t, c_StartIndex_u32, c_EndIndex_u32> Access16;
  /// Default BitSet implementation using uint8_t for 8-bit access to raw data
  typedef BitSet<uint8_t,  c_StartIndex_u32, c_EndIndex_u32> Access8;
};

} // namespace core
} // namespace mecl

#endif // MECL_CORE_BITSET_H_
/// @}
/// @}
