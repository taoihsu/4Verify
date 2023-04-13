// --------------------------------------------------------------------------
/// @file Helpers.h
/// @brief Contains macros and global functions that help when programming.
///
/// In the majority these are helper templates that are used in template definitions.
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

#ifndef HELPERS_H_
#define HELPERS_H_
// PRQA S 1020 ++ // convenience macros for compiling
// for convenience the typedef "type" is always named type
// PRQA S 1507 EOF
// for convenience some of the helper templates are implemented in structs
// PRQA S 2175 EOF
// PRQA S 1070 ++ // allow helper macros here
// PRQA S 2106 EOF // these are short templates that may contain implementation in the class body
// PRQA S 1062, 1063 EOF // something not used on the path - that is normal for template specializations
// PRQA S 1531, 1533 EOF // something not used on the path - that is normal for template specializations
#if __cplusplus <= 199711L
#define OVERRIDE
#else
#define OVERRIDE override
#ifndef __typeof
#define __typeof(type) decltype(type)
#define makeType(i_Var) mecl::ActualType<__typeof(i_Var)>::type
#endif // !__typeof
#endif // c++-11
// PRQA S 1020 -- // PRQA S 1070 --

#include "MeclTypes.h"

// PRQA S 1020 1 // safe macro to count items
#define numitems(Arr) ((sizeof(Arr)) / (sizeof(Arr[0])))

/// General namespace for MAGNA Electronics Common Library
namespace mecl
{
// PRQA S 2502 1 // namespace used throughout the project
namespace core
{
  // Forward declarations required due to Misra Rule 3-3-1
  template<class T> void UnusedParameter( const T& );

// The following templates are only used for meta-programming and not as actual types.
// Hence it should not be necessary to define copy-constructors (1701)
// and assignment operators (1702) especially since none of the templates
// contain mutable data.
// PRQA S 1701 EOF
// PRQA S 1702 EOF

/// Default template where the conditional type yields the first given type (B = true).
template<bool_t B, class T, class F>
struct ConditionalType
{
    typedef T type;
};

/// Template specialization for B=false where the second type is yielded.
template<class T, class F>
struct ConditionalType<false, T, F>
{
    typedef F type;
};

/// Can be used (safely) when a parameter is unused
template<class T> void UnusedParameter( const T& )
{
}

/// Can be used (safely) when a return value is ignored (but always consider checking the return value instead).
template<class T> void IgnoreReturnValue ( const T& )
{
}

// PRQA S 2640 6 // OK to use a static const object here
/// Used to define that a class is derived from a constant value (e.g. true or false).
template<class T, T v>
struct IntegralConstant
{
    static const T value = v;
    typedef T value_type;
    typedef IntegralConstant<T, v> type;
    // PRQA S 2181 1 // this is just the intention using a conversion operator here
    operator const T()
    {
        return v;
    }
};

/// Template to get the type - e.g. of a variable to use in other templates for type checks.
template<typename T>
struct ActualType
{
    typedef T type;
};

/// Common template to get the actual type of a pointer type.
template<typename T>
struct RemovePointer
{
    typedef T type;
};

/// Template specialization for RemovePointer which really removes the '*'-pointer from the type.
template<typename T>
struct RemovePointer<T*>
{
    typedef typename RemovePointer<T>::type type;
};

/// Common template to get the actual type of a reference type.
template<typename T>
struct RemoveReference
{
    typedef T type;
};

/// Template specialization for RemoveReference which really removes the '&'-reference from the type.
template<typename T>
struct RemoveReference<T&>
{
    typedef typename RemoveReference<T>::type type;
};

/// Template to check if a given template parameter is constant
// PRQA S 2116 ++ // no destructor needed for IsIntegral
// PRQA S 2153 ++ // deriving from non-abstract is OK for these templates
// by default the IsConst is derived from false
template<typename T> struct IsConst: public IntegralConstant<bool_t, false> {};
// when T is const, IsConst is derived from true - hence IsConst<const int> yields true
template<typename T> struct IsConst<const T> : public IntegralConstant<bool_t, true> {};
template<typename T> struct IsConst<const T&> : public IntegralConstant<bool_t, true> {};
template<typename T> struct IsConst<const T*> : public IntegralConstant<bool_t, true> {};
template<typename T> struct IsConst<const T*&> : public IntegralConstant<bool_t, true> {};

/// IsPointer is true for any pointer type that is used within a template
template<typename T> struct IsPointer: public IntegralConstant<bool_t, false> {};
template<typename T> struct IsPointer<T*> : public IntegralConstant<bool_t, true> {};
template<typename T> struct IsPointer<const T*> : public IntegralConstant<bool_t, true> {};

/// IsReference is true for any reference type that is used within a template
template<typename T> struct IsReference: public IntegralConstant<bool_t, false> {};
template<typename T> struct IsReference<T&> : public IntegralConstant<bool_t, true> {};
template<typename T> struct IsReference<const T&> : public IntegralConstant<bool_t, true> {};

/// IsIntegral is true for any signed or unsigned integer type
/// This can be used in template specializations specializing on integral types.
template<typename T> struct IsIntegral: public IntegralConstant<bool_t, false> {};
template<> struct IsIntegral<uint64_t> : public IntegralConstant<bool_t, true> {};
template<> struct IsIntegral<uint32_t> : public IntegralConstant<bool_t, true> {};
template<> struct IsIntegral<uint16_t> : public IntegralConstant<bool_t, true> {};
template<> struct IsIntegral<uint8_t> : public IntegralConstant<bool_t, true> {};
template<> struct IsIntegral<sint32_t> : public IntegralConstant<bool_t, true> {};
template<> struct IsIntegral<sint16_t> : public IntegralConstant<bool_t, true> {};
template<> struct IsIntegral<sint8_t> : public IntegralConstant<bool_t, true> {};

/// IsFloatingPoint is true for float32_t and float64_t
/// This can be used in template specializations specializing on floating point types.
template<typename T> struct IsFloatingPoint: public IntegralConstant<bool_t, false> {};
template<> struct IsFloatingPoint<float32_t> : public IntegralConstant<bool_t, true> {};
template<> struct IsFloatingPoint<float64_t> : public IntegralConstant<bool_t, true> {};

/// IsNumeral is true for either integral or floating point types
template<typename T> struct IsNumeral:
        public IntegralConstant<bool_t, IsIntegral<T>::value || IsFloatingPoint<T>::value> {};

/// IsBoolean is true for bool_t
/// This can be used in template specializations specializing on boolean types.
template<typename T> struct IsBoolean: public IntegralConstant<bool_t, false> {};
template<> struct IsBoolean<bool_t>: public IntegralConstant<bool_t, true> {};

/// IsChar is true for char_t
/// This can be used in template specializations specializing on character types.
template<typename T> struct IsChar: public IntegralConstant<bool_t, false> {};
template<> struct IsChar<char_t>: public IntegralConstant<bool_t, true> {};

/// IsPrimitive is true basically for all types that are defined in MeclTypes.h
/// This can be used in template specializations specializing on primitive (non-assembled) types.
template<typename T> struct IsPrimitive:
        public IntegralConstant<bool_t, IsPointer<T>::value
        || IsNumeral<T>::value
        || IsBoolean<T>::value
        || IsChar<T>::value> {};
// PRQA S 2116 -- // PRQA S 2153 --

/// ConstRefType gives a constant reference to non-primitive value types, otherwise the const-made
/// type itself is used.
// PRQA S 1051 ++ // example code
/// @par Example
/// @code{.cpp}
/// struct Point_s { uint32_t x; uint32_t y; }
/// // ConstRefType<Point_s> -> const Point_s &
/// // ConstRefType<uint32_t> -> const uint32_t
/// // ConstRefType<Point_s*> -> const Point_s*
/// @endcode
// PRQA S 1051 -- // example code end
template<typename T> struct ConstRefType
{
  typedef typename ConditionalType<
              IsPrimitive<T>::value ||
              IsPointer<T>::value ||
              IsReference<T>::value, const T, const T&>::type type;
};

/// Set an item to 0 - default implementation (which is empty)
template<typename T, bool_t IsPrimitiveType>
struct InitWithNull
{
    static void init_v(T&)
    {
        // default case: nothing to do
    }
};


/// Specialization for primitive types: initialize with 0
template<typename T>
struct InitWithNull<T,true>
{
    static void init_v(T& i_Item_r)
    {
        // primitive type: set to 0
        i_Item_r = 0;
    }
};

template<typename T>
void setToNull(T& i_Item_r)
{
    InitWithNull<T, IsPrimitive<T>::value>::init_v(i_Item_r);
}

/// Common template to check if two classes are the same.
/// This makes generally only sense in templates that accept two types when specialization is needed when the same type is used.
template<typename, typename>
struct IsSame
{
    static bool const value = false;
};

/// Template specialization delivers true when the type is identical.
template<typename A>
struct IsSame<A, A>
{
    static bool const value = true;
};

/// Determines if a given type is a POD struct or class or a primitive type.
/// A pod type is a type that does not contain virtual methods (such as simple structs).
// PRQA S 1051 ++ // example code
/// @par Example
/// @code{.cpp}
/// StaticAssert(mecl::core::IsSimplePod<MyClass>::value, "Virtual methods not allowed for MyClass!");
/// @endcode
// PRQA S 1051 -- // example code end
template<typename TCheck>
class IsSimplePod
{
    /// Helper template - default implementation for non-primitive types.
    template<typename T, bool IsPrimitiveType>
    struct IsSimplePodImpl
    {
        class _InnerDerived_ : public T
        {
            virtual ~_InnerDerived_()
            {
            }
        };
        /// The value is true when the sizeof the checked type is smaller than the the size of the derived type.
        /// The derived type contains one (more) virtual destructor.
        /// In case the checked type is POD, the virtual table should be empty.
        static const bool simplePod = (sizeof(T) < sizeof(_InnerDerived_));
    };

    /// Helper template - template specialization for primitive types.
    template<typename T>
    struct IsSimplePodImpl<T, true>
    {
        static const bool simplePod = true;
    };

public:
    // PRQA S 1531 1 // PRQA S 1533 1
    static const bool value = IsSimplePodImpl<TCheck, IsPrimitive<TCheck>::value>::simplePod;
};

/// Can be used to (statically) check if a given object does not contain a vtable.
// PRQA S 1051 ++ // example code
/// @par Example
/// @code{.cpp}
/// mecl::core::checkSimple_v(myItem);
/// @endcode
// PRQA S 1051 -- // example code end
template <typename T>
void checkSimple_v(const T& i_CheckItem_o)
{
  StaticAssert(mecl::core::IsSimplePod<T>::value, "Please use only objects without vtable!");
  // parameter is only used for the static check of the parameter type
  mecl::core::UnusedParameter(i_CheckItem_o);
}


// PRQA S 2640 5 // OK to use a static const object here
/// Determines during compile time if a constant number is a power of two
template<uint32_t N> struct IsPowerOfTwo
{
    static const bool_t value = (N != 0) && ((N & (N - 1)) == 0);
};

/// Returns the minimum value of two comparable objects.
/// To get it working operator <(T) is needed.
template<typename T>
T min_x (T val1, T val2)
{
    return (val1 < val2) ? val1 : val2;
}

/// Returns the maximum value of two comparable objects.
/// To get it working operator <(T) is needed.
template<typename T>
T max_x (T val1, T val2)
{
    return (val2 < val1) ? val1 : val2;
}

/// Returns the absolute value of an object that is comparable to 0.
/// To get it working both operator<(T, int) and operator-(T) are needed.
template<typename T>
T abs_x (T val)
{
    return (val < 0) ? -val : val;
}


/// Used to swap bytes due to conversion of little to big-endian and vice versa.
/// This is the general method which has no implementation.
/// Implementation is only done in template specializations.
template<uint32_t size_u32>
void swapBytes_v(void* mem_pv);

/// Swap bytes for 2-byte sized types (i.e. short).
template<>
inline void swapBytes_v<2U>(void* mem_pv)
{
#ifdef PIKEOSCPPENV
  *reinterpret_cast<uint16_t*>(mem_pv) = __builtin_bswap16(*reinterpret_cast<uint16_t*>(mem_pv));
#else
  uint16_t v_Tmp_u16 = *reinterpret_cast<uint16_t*>(mem_pv);
  *reinterpret_cast<uint16_t*>(mem_pv) =
        ((v_Tmp_u16 & 0x00ffU) << 8)
      | ((v_Tmp_u16 & 0xff00U) >> 8);
#endif
}

/// Swap bytes for 2-byte sized types (i.e. int).
template<>
inline void swapBytes_v<4U>(void* mem_pv)
{
#ifdef PIKEOSCPPENV
  *reinterpret_cast<uint32_t*>(mem_pv) = __builtin_bswap32(*reinterpret_cast<uint32_t*>(mem_pv));
#else
  uint32_t v_Tmp_u32 = *reinterpret_cast<uint32_t*>(mem_pv);
  *reinterpret_cast<uint32_t*>(mem_pv) =
        ((v_Tmp_u32 & 0x000000ffUL) << 24)
      | ((v_Tmp_u32 & 0x0000ff00UL) << 8)
      | ((v_Tmp_u32 & 0x00ff0000UL) >> 8)
      | ((v_Tmp_u32 & 0xff000000UL) >> 24);
#endif
}

/// Swap bytes for 8-byte sized types (i.e. long long).
template<>
inline void swapBytes_v<8U>(void* mem_pv)
{
#ifdef PIKEOSCPPENV
  *reinterpret_cast<uint64_t*>(mem_pv) = __builtin_bswap64(*reinterpret_cast<uint64_t*>(mem_pv));
#else
  uint64_t v_Tmp_u64 = *reinterpret_cast<uint64_t*>(mem_pv);
  *reinterpret_cast<uint64_t*>(mem_pv) =
        ((v_Tmp_u64 & 0x00000000000000ffULL) << 56)
      | ((v_Tmp_u64 & 0x000000000000ff00ULL) << 40)
      | ((v_Tmp_u64 & 0x0000000000ff0000ULL) << 24)
      | ((v_Tmp_u64 & 0x00000000ff000000ULL) << 8)
      | ((v_Tmp_u64 & 0x000000ff00000000ULL) >> 8)
      | ((v_Tmp_u64 & 0x0000ff0000000000ULL) >> 24)
      | ((v_Tmp_u64 & 0x00ff000000000000ULL) >> 40)
      | ((v_Tmp_u64 & 0xff00000000000000ULL) >> 56);
#endif
}

/// Swap bytes for a given (primitive) type.
template <typename T>
inline void swapByteOrder_v(T& i_Value_rx)
{
  swapBytes_v<sizeof(T)>(&i_Value_rx);
}

// PRQA S 1051 ++ // example code
/// This can be used for arrays as well as for structs.
/// This only works for structs or parts of struct, where all members have
/// primitives of the same size - for example with uint32_t, int32_t and float32_t mixed.
/// @par Example
/// @code{.cpp}
/// struct MyStruct
/// {
///   float one;
///   float two;
///   float three;
/// };
///
/// MyStruct s = { 1.234F, 5.678F, -44.321F };
/// swapByteOrder_v(s.one); // swap members one by one
/// swapByteOrder_v(s.two);
/// swapByteOrder_v(s.three);
/// swapByteOrderArray_v(&s.one, 3); // swap members in a loop
///
/// printf("%f %f %f\n", s.one, s.two, s.three);
/// @endcode
// PRQA S 1051 -- // example code end
template <typename T>
inline void swapByteOrderArray_v(T* i_Value_px, uint32_t i_Size_u32)
{
  for (uint32_t v_Index_u32 = 0U; v_Index_u32 < i_Size_u32; ++v_Index_u32)
  {
    // PRQA S 3706 1 // Subscript operator is used safely here
    swapByteOrder_v(i_Value_px[v_Index_u32]);
  }
}


/// Copy single bytes from source to destination memory.
/// Allows chaining of copy-operations (the output of the previous copy operation can be used as input for the next).
/// @param[out] o_MemDest_pv destination memory area.
/// @param[in] i_MemSrc_pv source memory area
/// @tparam size_u32 number of bytes to copy
template<uint32_t size_u32>
inline uint8_t* copyBytes_pv(uint8_t* o_MemDest_pv, const uint8_t* i_MemSrc_pv)
{
  // this is only a simple implementation
  // optimization for 4-byte aligned memory is possible here.
  // Also the size as template argument is not exploitet here.
  for (uint32_t i = 0; i < size_u32; ++i)
  {
    *o_MemDest_pv = *i_MemSrc_pv;
    ++o_MemDest_pv;
    ++i_MemSrc_pv;
  }
  return o_MemDest_pv;
}

/// Copy Bytes from a memory area to a value
/// Allows chaining of copy-operations (the output of the previous copy operation can be used as input for the next).
/// @param[out] o_Dest_pv destination pointer.
/// @param[in] i_SrcValue_rx input value.
/// @tparam T type to copy from
/// @return the destination pointer behind the copied data.
template <typename T>
inline void* copyBytesTo_pv(void* o_Dest_pv,
                            const T& i_SrcValue_rx)
{
  // PRQA S 3017 2 // it is OK here to cast from float to uint8 data as it is copied in binary format here
  return copyBytes_pv<sizeof(T)>(reinterpret_cast<uint8_t*>(o_Dest_pv),
      reinterpret_cast<const uint8_t*>(&i_SrcValue_rx));
}

/// Copy Bytes from a memory area to a value.
/// Allows chaining of copy-operations (the output of the previous copy operation can be used as input for the next).
/// @param[out] i_Value_rx destination value
/// @param[in] i_SrcPointer_pv source pointer.
/// @tparam T type to copy to
/// @return the source pointer behind the copied data.
template <typename T>
inline const void* copyBytesFrom_pv(T& o_DestValue_rx,
                                    const void* i_SrcPointer_pv)
{
  // PRQA S 3017 2 // it is OK here to cast from float to uint8 data as it is copied in binary format here
  IgnoreReturnValue(copyBytes_pv<sizeof(T)>(reinterpret_cast<uint8_t*>(&o_DestValue_rx),
      reinterpret_cast<const uint8_t*>(i_SrcPointer_pv)));
  // calculate the return value based on the source pointer
  // PRQA S 3705 1 // OK to use pointer arithmetic here - method shall return memory behind copied area
  return (reinterpret_cast<const uint8_t*>(i_SrcPointer_pv) + sizeof(T));
}

/// Copy Bytes from a memory area to an array value
/// Allows chaining of copy-operations (the output of the previous copy operation can be used as input for the next).
/// @param[out] o_Dest_pv destination pointer.
/// @param[in] i_SrcValue_px pointer to input array.
/// @tparam T type to copy from
/// @return the destination pointer behind the copied data.
template <typename T>
inline void* copyBytesToArray_pv(void* o_Dest_pv,
                                 const T* const i_SrcValue_px,
                                 uint32_t i_Size_u32)
{
  void *memDest_pv = o_Dest_pv;
  for (uint32_t v_Index_u32 = 0U; v_Index_u32 < i_Size_u32; ++v_Index_u32)
  {
    // PRQA S 3706 1 // Subscript operator is used safely here
    memDest_pv = copyBytesTo_pv(memDest_pv, i_SrcValue_px[v_Index_u32]);
  }
  return memDest_pv;
}

/// Copy Bytes from a memory area to a value.
/// Allows chaining of copy-operations (the output of the previous copy operation can be used as input for the next).
/// @param[out] o_Value_px Pointer to destination array.
/// @param[in] i_SrcPointer_pv source pointer.
/// @tparam T type to copy to
/// @return the source pointer behind the copied data.
template <typename T>
inline const void* copyBytesFromArray_pv(T* o_Value_px,
                                         const void* i_SrcPointer_pv,
                                         uint32_t i_Size_u32)
{
  const void *memSrc_pv = i_SrcPointer_pv;
  for (uint32_t v_Index_u32 = 0U; v_Index_u32 < i_Size_u32; ++v_Index_u32)
  {
    // PRQA S 3706 1 // Subscript operator is used safely here
    memSrc_pv = copyBytesFrom_pv(o_Value_px[v_Index_u32], memSrc_pv);
  }
  return memSrc_pv;
}

/// Class to derive from when default copying of elements should be inhibited.
class NonCopyable
{
  protected:
    NonCopyable () {}
    ~NonCopyable () {} /// Protected non-virtual destructor
  private:
    /// Inhibits NonCopyable classes to be used (accidentally) as value parameters
    NonCopyable (const NonCopyable &);
    /// Inhibits NonCopyable class objects to assign to other items
    NonCopyable & operator = (const NonCopyable &);
};

/// Helper template to make a class a "final" class - i.e. a class
/// that cannot be derived from.
template <typename T>
class MakeFinal
{
private:
  friend T;
  ~MakeFinal() {}
};

/// Helper template to make a class a "final" class and non-copyable.
template <typename T>
class MakeFinalNonCopyable
  : public NonCopyable
{
private:
  friend T;
  ~MakeFinalNonCopyable() {}
};


} // namespace core
} // namespace mecl

#endif // HELPERS_H_
