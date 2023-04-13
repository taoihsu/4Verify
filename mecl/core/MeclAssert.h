//--------------------------------------------------------------------------
/// @file MeclAssert.h
/// @brief Contains the Assert macro to use within the projects.
///
/// The assert implementation should lead to an error log about the
/// location and the condition that occurred.
/// The assert could be deactivated in release builds.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Michael Schulte (michael.schulte@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef MECL_ASSERT_H_
#define MECL_ASSERT_H_

// we need to define some macros in MeclAssert.h which would not work otherwise

#include "MeclTypes.h"
#include "MeclLogging.h"

#ifdef _MSC_VER
#include <crtdefs.h>
#endif

#ifdef _MSC_VER
#define __PRETTY_FUNCTION__ __FUNCSIG__

#else
// PRQA S 1021 1 // use of macro for assert message with function name output
#define __PRETTY_FUNCTION__ __FUNCTION__
#endif

namespace mecl
{
namespace core
{

/// Assertion abstraction
void AssertExt_v(const char_t i_Message_pc[],
               const char_t i_File_pc[],
               const sint32_t i_Line_i32);

/// Test for null pointer
bool_t isNull_b (const void *i_Pointer_p);

/// Test for non-null pointer
bool_t isNotNull_b (const void *i_Pointer_p);

/// Default template for static asserts.
/// Here the initialization works. There is no compile error when the checked condition in template argument is true.
template<bool_t b>
struct StaticAssertTemplate
{
    static void assertSuccess() {}
};

/// Template specialized on false that does not contain an implementation.
/// This yields a compile error if the template argument (i.e. the checked condition) is false.
template<>
struct StaticAssertTemplate<false>;
}
}

// Examples in the comments should be OK
// PRQA S 1051 EOF
// The "stringify" # operator is used here to generate output according to the condition that was checked.
// This is used for debug purposes and makes usage of # necessary
// PRQA S 1038 4 // suppress stringify message for the next 4 lines


/// @brief Assertion macro
/// Asserts that a condition is true. If the condition is false the system may be halted. Function calls assertion abstraction.
#ifndef Assert
// PRQA S 1038 1 // stringify used for convenience/better error output in Assert macro
#define Assert(condition) ((condition) ? static_cast<void>(false) : mecl::core::AssertExt_v(#condition, __FILE__, __LINE__))
#endif

/// Assertion macro with output message
#ifndef AssertMsg
#define AssertMsg(condition,...) \
    if (!(condition)) { \
       log_printf(__VA_ARGS__); \
   } \
   Assert(condition) // enforce semicolon
#endif

/// Assertion macro with function signature and output message
// PRQA S 1598 5 __PRETTY_FUNCTION__ is defined for all plattforms
#ifndef AssertFunction
#define AssertFunction(condition, ...) \
    if (!(condition)) { \
        log_printf("%s: ", __PRETTY_FUNCTION__); \
        vm_cprintf(__VA_ARGS__); \
    } \
    Assert(condition)
#endif

/// Asserts that a pointer is not null. Convenience macro.
#define AssertNotNull(pointer) Assert(mecl::isNotNull_b(pointer))

/// @brief Compile-time assert. 
///
/// This can be used to check constant values or sizes against specific bounds.
/// use like:
///
/// @code{cpp}
/// StaticAssert(Size > 0, "Size must not be 0");
/// @endcode
/// 
/// All used values have to be known at compile time (constant expression)
/// the default template has no body
/// - hence object instantiation does not work
#define StaticAssert(condition, text) \
    mecl::core::StaticAssertTemplate<(condition)>::assertSuccess();

#endif // MECL_ASSERT_H_
/// @}
/// @}
