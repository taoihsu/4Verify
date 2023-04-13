#ifndef MECL_CORE_FIXEDPOINT_H
#define MECL_CORE_FIXEDPOINT_H

#include "mecl/core/MeclTypes.h"
#include "mecl/core/MeclAssert.h"
#include "mecl/core/Helpers.h"

#include <stdio.h> // PRQA S 5188 // use of snprintf here

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

// PRQA S 1051 EOF // comments document calculations - are not source code
namespace mecl
{
namespace core
{

#ifndef FIXEDPOINT_CHECK
#ifdef _DEBUG
const bool_t c_FixedPointCheck_b = true;
#else
const bool_t c_FixedPointCheck_b = false;
#endif
#else
const bool_t c_FixedPointCheck_b = true;
#endif

/// helper template forward declaration

template<bool_t SIGNED, typename T> struct NegValue { static T negValue_x(const T& i_InValue_x) { return i_InValue_x; } }; // negate unsigned stays the same
template<typename T> struct NegValue<true,T> { static T negValue_x(const T& i_InValue_x) { return static_cast<T>((~i_InValue_x)+1U); } }; // negate signed value

template<bool_t SIGNED, typename T, uint32_t BITS> struct MinValue { static const T c_Value_x = 0U; }; // minimum unsigned value is 0
template<typename T, uint32_t BITS> struct MinValue<true,T,BITS> { static const T c_Value_x = -(static_cast<T>(((1ULL << (BITS-1))-1U))); }; // calculate minimum signed value

template<bool_t SIGNED, uint32_t MAXBITS> struct DefaultType;
template<uint32_t BITS8> struct MaxBits;

/// Fixed Point class
/// @tparam T the underlying type used in memory
/// @tparam INTBITS number of bits used in numerator part
/// @tparam FRACBITS number of bits used for fractional part
/// @tparam SIGNED if true, the underlying type is signed, else unsigned.
/// @tparam BITS total number of bits used in underlying type.
/// This is automatically determined by the number of bits used for numerator and fractional part
template <uint32_t INTBITS, uint32_t FRACBITS, bool_t SIGNED=true, uint32_t BITS=INTBITS+FRACBITS>
class FixedPoint
{
public:
  //PRQA S 1034,1020 1 // macro is only used locally (as a short replacement for the rather long template)
  #define FixedPoint_t FixedPoint<INTBITS, FRACBITS, SIGNED, BITS>

  /// Maximum number of bits that could be used with underlying type
  static const uint32_t c_MaxBits_u32 = MaxBits<8U * ((BITS+7U)/8)>::c_Value_u32;
  typedef typename DefaultType<SIGNED, c_MaxBits_u32>::type_t Type_t;
  typedef typename DefaultType<false, c_MaxBits_u32>::type_t UnsignedType_t;

  /// Maximal bit mask for the underlying type (all bits set).
  static const UnsignedType_t c_MaxBitsMask_x = static_cast<UnsignedType_t>((1ULL << c_MaxBits_u32) - 1U);
  /// Length of the used bit for signedness. 1 for signed types, 0 for unsigned types.
  static const uint32_t c_LengthSignedBit_u32 = SIGNED ? 1U : 0U;
  /// Bits that are used for the internal value with sign
  static const UnsignedType_t c_BitsMask_t = static_cast<UnsignedType_t>((1ULL << BITS) - 1U);
  /// Bits that are used for the internal value (without the sign)
  static const UnsignedType_t c_UsedBitsMask_t = static_cast<UnsignedType_t>((1ULL << (BITS - c_LengthSignedBit_u32)) - 1U);
  /// Bits that cannot be used for the internal value (used in checkOverflow_v())
  static const UnsignedType_t c_UnusedBitsMask_t = static_cast<UnsignedType_t>(c_UsedBitsMask_t ^ c_MaxBitsMask_x);

  /// Minimum value for internal representation
  static const Type_t c_MaxInternalValue_t = static_cast<Type_t>(((1ULL << (BITS-c_LengthSignedBit_u32))-1U));
  /// Maximum value for internal representation
  static const Type_t c_MinInternalValue_t = MinValue<SIGNED, Type_t, BITS>::c_Value_x;

  /// Default constructor: set 0
  FixedPoint() : val_x(0)
  {
  }

  static Type_t& adjustSignedness_v(Type_t& o_Value_x)
  {
    if (SIGNED
      && (c_MaxBits_u32 != BITS)
      && (((1 << (BITS-1)) & o_Value_x) != 0) // highest bit is set -> negative number
      && (((c_BitsMask_t ^ c_MaxBitsMask_x) & o_Value_x) == 0)) // all other bits above not set
    {
      // add 1s at beginning
      o_Value_x |= c_UnusedBitsMask_t;
    }
    return o_Value_x;
  }

  // used by operators to construct return type
  explicit FixedPoint(Type_t i_Value_x) : val_x(adjustSignedness_v(i_Value_x))
  {
    if (true == c_FixedPointCheck_b)
    {
      checkOverflow_v();
    }
  }

  explicit FixedPoint(float64_t i_Value_f64) : val_x(convertDouble_x(i_Value_f64))
  {
    if (true == c_FixedPointCheck_b)
    {
      checkFloatOverflow_v(i_Value_f64);
    }
  }


  static bool_t isSigned_b()
  {
    return SIGNED;
  }

  static Type_t convertDouble_x(float64_t i_Value_f64)
  {
    sint32_t v_Fact_i32 = (i_Value_f64 >= 0.0) ? 1 : -1;
    // adding (or subtracting) factor of 0.5/(1<<FRACBITS) rounds the result
    Type_t v_Ret_x = static_cast<Type_t>(static_cast<float64_t>(1ULL << FRACBITS) *
         (i_Value_f64 + (static_cast<float64_t>(v_Fact_i32) * (0.5 / static_cast<float64_t>(1ULL << FRACBITS)))));

    return v_Ret_x;
  }

  UnsignedType_t getBits_x() const
  {
    return static_cast<UnsignedType_t>(val_x) & c_BitsMask_t;
  }

  static FixedPoint_t epsilon_x()
  {
    return FixedPoint_t(1);
  }

  /// Get the value for 1.
  static FixedPoint_t one_x()
  {
    return FixedPoint(static_cast<Type_t>(1 << FRACBITS));
  }

  //PRQA S 2502 2 // generic max implementation used in math.h
  /// Get the negative value. Can only be used for signed types.
  FixedPoint_t neg_x() const
  {
    StaticAssert(SIGNED, "Not allowed to use on unsigned types");
    return FixedPoint(NegValue<SIGNED,Type_t>::negValue_x(val_x));
  }

  /// Get the value for -1. Can only be used for signed types.
  static FixedPoint_t minusOne_x()
  {
    StaticAssert(SIGNED, "Not allowed to use on unsigned types");
    return one_x().neg_x();
  }


  //PRQA S 2502 2 // generic max implementation used in math.h
  /// Get the maximum possible value.
  static FixedPoint_t max_x()
  {
    return FixedPoint_t(c_MaxInternalValue_t);
  }

  //PRQA S 2502 2 // generic max implementation used in math.h
  /// Get the minimum possible value.
  static FixedPoint_t min_x()
  {
    return FixedPoint_t(c_MinInternalValue_t);
  }

  static float64_t maxValue_f64()
  {
    return static_cast<float64_t>(c_MaxInternalValue_t) / static_cast<float64_t>(1ULL << FRACBITS);
  }

  static float64_t minValue_f64()
  {
    return static_cast<float64_t>(c_MinInternalValue_t) / static_cast<float64_t>(1ULL << FRACBITS);
  }

  sint32_t sgn_i32() const
  {
    sint32_t v_Result_i32 = 1;
    if (SIGNED && (0 != (val_x & (1 << (BITS - 1)))))
    {
      v_Result_i32 = -1;
    }
    return v_Result_i32;
  }


  void assertOverflowMsg_v() const
  {
    const Type_t c_Fact_x = static_cast<Type_t>(sgn_i32());
    const uint32_t c_ShiftFactor_u32 = (c_MaxBits_u32 - BITS) + c_LengthSignedBit_u32;
    const Type_t c_RefVal_x = c_Fact_x * static_cast<Type_t>
      ((((static_cast<UnsignedType_t>(c_Fact_x * val_x) << c_ShiftFactor_u32) & c_MaxBitsMask_x) >> c_ShiftFactor_u32));

    char_t v_Msg_ac[80];
    snprintf(&v_Msg_ac[0], sizeof(v_Msg_ac),
        "Fixed-point overflow: %f (0x%x) --> %f (0x%x)\n",
        static_cast<float64_t>(val_x) / static_cast<float64_t>(1ULL << FRACBITS), static_cast<printInt_t>(val_x),
        static_cast<float64_t>(c_RefVal_x) / static_cast<float64_t>(1ULL << FRACBITS), static_cast<printInt_t>(c_RefVal_x));
    AssertMsg(false, &v_Msg_ac[0]);
  }

  /// Checks if the bytes used to render the float value is out of bounds or not.
  /// It also prints a message of the value that could not be rendered.
  void checkOverflow_v() const
  {
    StaticAssert(c_MaxBits_u32 >= BITS, "Max number of bits does not match used number of bits");
    bool_t v_CheckOk_b;
    if (-1 == sgn_i32())
    {
      v_CheckOk_b = (c_UnusedBitsMask_t & static_cast<UnsignedType_t>(NegValue<SIGNED, Type_t>::negValue_x(val_x))) == 0U;
    }
    else
    {
      v_CheckOk_b = (c_UnusedBitsMask_t & static_cast<UnsignedType_t>(val_x)) == 0U;
    }

    if (false == v_CheckOk_b)
    {
      assertOverflowMsg_v();
    }
  }

  void checkFloatOverflow_v(const float64_t& i_Val_rf64) const
  {
    if ( hasOverflow_b(i_Val_rf64) )
    {
      assertOverflowMsg_v();
    }
  }

  static bool_t hasOverflow_b(const float64_t& i_Val_rf64)
  {
    return (    (i_Val_rf64 > (maxValue_f64() + (0.5 / static_cast<float64_t>(1ULL << FRACBITS))))
             || (i_Val_rf64 < (minValue_f64() - (0.5 / static_cast<float64_t>(1ULL << FRACBITS)))));
  }

  // assignment
  FixedPoint_t& operator = (const FixedPoint_t& i_Other_rt)
  {
    val_x = i_Other_rt.val_x;
    return *this;
  }

  void assignRaw_v(const Type_t& i_NewVal_rx)
  {
    this->val_x = i_NewVal_rx;
    if (true == c_FixedPointCheck_b)
    {
      checkOverflow_v();
    }
  }

  FixedPoint_t& operator = (const Type_t& i_NewVal_rx)
  {
    assignRaw_v(i_NewVal_rx);
    return *this;
  }

  FixedPoint_t& operator = (float64_t i_Value_f64)
  {
    val_x = convertDouble_x(i_Value_f64);
    if (true == c_FixedPointCheck_b)
    {
      checkFloatOverflow_v(i_Value_f64);
    }
    return *this;
  }


  // type conversion
  Type_t toType_x() const
  {
    return (val_x + (1 << (FRACBITS - 1))) >> FRACBITS;
  }
  float32_t toFloat_f32() const
  {
    return static_cast<float32_t>(val_x) / static_cast<float32_t>(1 << FRACBITS);
  }
  float64_t toDouble_f64() const
  {
    return static_cast<float64_t>(val_x) / static_cast<float64_t>(1ULL << FRACBITS);
  }

  operator Type_t() const
  {
    return toType_x();
  }
  operator float32_t() const
  {
    return toFloat_f32();
  }
  operator float64_t() const
  {
    return toDouble_f64();
  }


  template <uint32_t NumAlt, uint32_t FracAlt, bool_t SignedAlt, uint32_t BitsAlt>
  FixedPoint<NumAlt, FracAlt, SignedAlt, BitsAlt> convert_x() const
  {
    typedef typename FixedPoint<NumAlt, FracAlt, SignedAlt, BitsAlt>::Type_t TAlt;
    TAlt i_NewVal_x;
    if (FRACBITS <= FracAlt)
    {
      i_NewVal_x = static_cast<TAlt>(val_x << (FracAlt - FRACBITS));
    }
    else
    {
      i_NewVal_x = static_cast<TAlt>(val_x >> (FRACBITS - FracAlt));
    }
    FixedPoint<NumAlt, FracAlt, SignedAlt, BitsAlt> v_Ret_x(i_NewVal_x);
    return v_Ret_x;
  }


  template <uint32_t NumAlt, uint32_t FracAlt>
  FixedPoint<NumAlt, FracAlt> convert_x() const
  {
    typedef typename FixedPoint<NumAlt, FracAlt>::Type_t TAlt;
    TAlt v_NewVal_x;
    if (FRACBITS <= FracAlt)
    {
      v_NewVal_x = val_x << (FracAlt - FRACBITS);
    }
    else
    {
      v_NewVal_x = val_x >> (FRACBITS - FracAlt);
    }
    FixedPoint<NumAlt, FracAlt> v_Ret_x(static_cast<TAlt>(v_NewVal_x));
    return v_Ret_x;
  }


  template <uint32_t NumAlt, uint32_t FracAlt, bool_t SignedAlt, uint32_t BitsAlt>
  void assign_v(const FixedPoint<NumAlt, FracAlt, SignedAlt, BitsAlt>& i_Other_ro)
  {
    Type_t i_NewVal_x;
    if (FRACBITS > FracAlt)
    {
      i_NewVal_x = i_Other_ro.val_x << (FRACBITS - FracAlt);
    }
    else
    {
      i_NewVal_x = i_Other_ro.val_x >> (FracAlt - FRACBITS);
    }
    assignRaw_v(i_NewVal_x);
  }


  friend FixedPoint_t operator + (const FixedPoint_t& i_Lhs_rx, const FixedPoint_t& i_Rhs_rx)
  {
    return FixedPoint_t(i_Lhs_rx.val_x + i_Rhs_rx.val_x);
  }

  friend FixedPoint_t operator - (const FixedPoint_t& i_Lhs_rx, const FixedPoint_t& i_Rhs_rx)
  {
    return FixedPoint_t(i_Lhs_rx.val_x - i_Rhs_rx.val_x);
  }

  friend FixedPoint_t operator * (const FixedPoint_t& i_Lhs_rx, const FixedPoint_t& i_Rhs_rx)
  {
    return FixedPoint_t((i_Lhs_rx.val_x*i_Rhs_rx.val_x + (1<<(FRACBITS-1)) ) >> FRACBITS);
  }

  friend FixedPoint_t operator / (const FixedPoint_t& i_Lhs_rx, const FixedPoint_t& i_Rhs_rx)
  {
    //PRQA S 1051 6 // comment shows how division formula was constructed
    // float32_t af = (float32_t) i_Lhs_rx / (float32_t) (1<<FRACBITS);
    // float32_t bf = (float32_t) i_Rhs_rx / (float32_t) (1<<FRACBITS);
    // float32_t cf = af / bf;
    // int64_t c = (int64_t)(cf * (float32_t) (1<<FRACBITS))
    //           = (  i_Lhs_rx / (1<<FRACBITS) )  / (  i_Rhs_rx / (1<<FRACBITS) ) * (1<<FRACBITS)
    //           = i_Lhs_rx / i_Rhs_rx * (1<<FRACBITS);
    return FixedPoint_t((i_Lhs_rx.val_x << FRACBITS)/i_Rhs_rx.val_x);
  }
                                                                  

  friend FixedPoint_t operator + (const FixedPoint_t& i_Lhs_rx, const Type_t& i_Rhs_rx)
  {
    return FixedPoint_t(i_Lhs_rx.val_x + (i_Rhs_rx << FRACBITS));
  }

  friend FixedPoint_t operator - (const FixedPoint_t& i_Lhs_rx, const Type_t& i_Rhs_rx)
  {
    return FixedPoint_t(i_Lhs_rx.val_x - (i_Rhs_rx << FRACBITS));
  }

  friend FixedPoint_t operator * (const FixedPoint_t& i_Lhs_rx, const Type_t& i_Rhs_rx)
  {
    return FixedPoint_t(i_Lhs_rx.val_x * i_Rhs_rx);
  }

  friend const FixedPoint_t operator / (const FixedPoint_t& i_Lhs_rx, const Type_t& i_Rhs_rx)
  {
    return FixedPoint_t(i_Lhs_rx.val_x / i_Rhs_rx);
  }

  friend const FixedPoint_t operator + (const Type_t& i_Lhs_rx, const FixedPoint_t& i_Rhs_rx)
  {
    return FixedPoint_t((i_Lhs_rx << FRACBITS) + i_Rhs_rx.val_x);
  }

  friend const FixedPoint_t operator - (const Type_t& i_Lhs_rx, const FixedPoint_t& i_Rhs_rx)
  {
    return FixedPoint_t((i_Lhs_rx << FRACBITS) - i_Rhs_rx.val_x);
  }

  friend const FixedPoint_t operator * (const Type_t& i_Lhs_rx, const FixedPoint_t& i_Rhs_rx)
  {
    return FixedPoint_t( i_Lhs_rx * i_Rhs_rx.val_x );
  }

  friend const FixedPoint_t operator / (const Type_t& i_Lhs_rx, const FixedPoint_t& i_Rhs_rx)
  {
    return FixedPoint_t((i_Lhs_rx << (static_cast<UnsignedType_t>(2L)) * FRACBITS) / i_Rhs_rx.val_x);
  }

  friend const FixedPoint_t operator<<(const FixedPoint_t& i_FixedPoint_rt, uint32_t i_ShiftFactor_u32)
  {
    StaticAssert(not SIGNED, "Not allowed to use on signed types");
    FixedPoint_t ret_t(i_FixedPoint_rt.getBits_x() << i_ShiftFactor_u32);
    return ret_t;
  }

  friend const FixedPoint_t operator>>(const FixedPoint_t& i_FixedPoint_rt, uint32_t i_ShiftFactor_u32)
  {
    StaticAssert(not SIGNED, "Not allowed to use on signed types");
    FixedPoint_t ret_t(i_FixedPoint_rt.getBits_x() >> i_ShiftFactor_u32);
    return ret_t;
  }

  //PRQA S 1034,1020 1 // macro is only used locally
#define COMPARE(OP) \
  friend bool operator OP (const FixedPoint_t& i_Lhs_rt, const FixedPoint_t& i_Rhs_rt) { return i_Lhs_rt.val_x OP i_Rhs_rt.val_x; } \
  friend bool operator OP (const Type_t& i_Lhs_rt, const FixedPoint_t&  i_Rhs_rt) { return (i_Lhs_rt << FRACBITS) OP i_Rhs_rt.val_x; } \
  friend bool operator OP (const FixedPoint_t& i_Lhs_rt, const Type_t& i_Rhs_rt) { return i_Lhs_rt.val_x OP (i_Rhs_rt << FRACBITS); }

  COMPARE(< )
  COMPARE(<=)
  COMPARE(> )
  COMPARE(>=)
  COMPARE(==)
  COMPARE(!=)

private:
  Type_t val_x;
};


template<uint32_t BITS8> struct MaxBits {};
template<> struct MaxBits<8>  { static const uint32_t c_Value_u32 = 8UL; };
template<> struct MaxBits<16> { static const uint32_t c_Value_u32 = 16UL; };
template<> struct MaxBits<24> { static const uint32_t c_Value_u32 = 32UL; };
template<> struct MaxBits<32> { static const uint32_t c_Value_u32 = 32UL; };
template<> struct MaxBits<40> { static const uint32_t c_Value_u32 = 64UL; };
template<> struct MaxBits<48> { static const uint32_t c_Value_u32 = 64UL; };
template<> struct MaxBits<56> { static const uint32_t c_Value_u32 = 64UL; };
template<> struct MaxBits<64> { static const uint32_t c_Value_u32 = 64UL; };


template<bool_t SIGNED, uint32_t MAXBITS> struct DefaultType {};
template<> struct DefaultType<true,8>   { typedef sint8_t type_t; };
template<> struct DefaultType<true,16>  { typedef sint16_t type_t; };
template<> struct DefaultType<true,32>  { typedef sint32_t type_t; };
template<> struct DefaultType<true,64>  { typedef sint64_t type_t; };
template<> struct DefaultType<false,8>  { typedef uint8_t type_t; };
template<> struct DefaultType<false,16> { typedef uint16_t type_t; };
template<> struct DefaultType<false,32> { typedef uint32_t type_t; };
template<> struct DefaultType<false,64> { typedef uint64_t type_t; };

template <uint32_t INTBITS, uint32_t FRACBITS>
class UnsignedFixedPoint
{
public:
  typedef FixedPoint<INTBITS, FRACBITS, false, INTBITS+FRACBITS> Type_t;
};

template <uint32_t INTBITS, uint32_t FRACBITS, bool_t SIGNED=true>
class FixedPoint32
{
public:
  typedef FixedPoint<INTBITS, FRACBITS, SIGNED, 32> Type_t;
};

template <uint32_t INTBITS, uint32_t FRACBITS>
class UnsignedFixedPoint32
{
public:
  typedef FixedPoint<INTBITS, FRACBITS, false, 32> Type_t;
};



} // namespace core
} // namespace mecl

#endif // MECL_CORE_FIXEDPOINT_H
