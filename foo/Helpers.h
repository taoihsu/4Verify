/*
 * Helpers.h
 *
 *  Created on: 06.06.2014
 *      Author: MEE_MiSch
 */

#ifndef TSC_HELPERS_H_
#define TSC_HELPERS_H_

#if 0

#include <stdint.h>
#include <mecl/core/MeclTypes.h>

namespace core
{

#ifndef __GNUC__
#ifndef __typeof
#define __typeof(type) decltype(type)
#endif
#endif // ! __GNUC__

// the default template has no body
// - hence object instantiation does not work
template <bool b>
struct StaticAssertTemplateAlt;

// template specialized on true -
// here the initialization works.
template <>
struct StaticAssertTemplateAlt<true>
{
};


// use like: StaticAssert(Size > 0)
// all used values have to be known at compile time (constant expression)
#define StaticAssertAlt(expr) StaticAssertTemplateAlt<(expr)>()

// per default the conditional type yields the first given type (B = true)
template<bool B, class T, class F>
struct ConditionalType { typedef T type; };

// when B=false the second type is yielded.
template<class T, class F>
struct ConditionalType<false, T, F> { typedef F type; };

// Used to define that a class is derived from a constant value (e.g. true or false)
template <class T, T v>
struct IntegralConstant
{
	static const T value = v;
	typedef T value_type;
	typedef IntegralConstant<T,v> type;
	operator const T() { return v; }
};

// Template to get the type - e.g. of a variable
template <typename T>
struct ActualType
{
    typedef T type;
};

#define makeType(i_var) ActualType<__typeof(i_var)>::type

template<typename T>
struct RemovePointer
{
    typedef T type;
};

template<typename T>
struct RemovePointer<T*>
{
    typedef typename RemovePointer<T>::type type;
};


// by default the IsConst is derived from false
template<typename T> struct IsConst          : public IntegralConstant<bool,false> {};
// when T is const, IsConst is derived from true - hence IsConst<const int> yields true
template<typename T> struct IsConst<const T> : public IntegralConstant<bool,true> {};
template<typename T> struct IsConst<const T&> : public IntegralConstant<bool,true> {};
template<typename T> struct IsConst<const T*> : public IntegralConstant<bool,true> {};
template<typename T> struct IsConst<const T*&> : public IntegralConstant<bool,true> {};

template <typename T> struct IsPointer	 	: public IntegralConstant<bool, false> {};
template <typename T> struct IsPointer<T*> 	: public IntegralConstant<bool, true> {};


template <typename T> struct IsIntegral : public IntegralConstant<bool, false> {};
template <> struct IsIntegral<uint32_t> : public IntegralConstant<bool, true> {};
template <> struct IsIntegral<uint16_t> : public IntegralConstant<bool, true> {};
template <> struct IsIntegral<uint8_t> : public IntegralConstant<bool, true> {};
template <> struct IsIntegral<int32_t> : public IntegralConstant<bool, true> {};
template <> struct IsIntegral<int16_t> : public IntegralConstant<bool, true> {};
template <> struct IsIntegral<int8_t> : public IntegralConstant<bool, true> {};

template <typename T> struct IsFloatingPoint : public IntegralConstant<bool, false> {};
template <> struct IsFloatingPoint<float32_t> : public IntegralConstant<bool, true> {};
template <> struct IsFloatingPoint<float64_t> : public IntegralConstant<bool, true> {};

} // namespace core
#endif
#endif /* HELPERS_H_ */
