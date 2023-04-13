//--------------------------------------------------------------------------
/// @file MeclTypes.h
/// @brief Contains the global type definitions for simple types.
///
/// The types defined in this file shall be used for the primitive types
/// instead of using plain int, float or bool_t types.
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

#ifndef MECLTYPES_H_
#define MECLTYPES_H_

// GLOBAL JSF suppressions:

// Turn off JSF Rule 33 to only use <> for including headers
// This really is a mess when working with local includes
// PRQA S 1015 ++

// We need the at-character in Comments and also the dollar character in the version id
// Unfortunately it is not possible to exclude certain characters from
// JSF rule 9 - so all are excluded here
// PRQA S 1095 ++

// Naming rule JSF 50 is not followed
// PRQA S 5050 ++

// Macro is used by MKS
// PRQA S 1020 1
#define MeclTypes_D_VERSION_ID "$Id: MeclTypes.h 1.14 2016/11/08 09:37:53EST Reichert2, Mark (Mark.Reichert2) draft  $"

/// Definition of standard library macro 'NULL'
#ifdef _MSC_VER
#ifdef __cplusplus
#include <cstddef>
#endif
#else
// PRQA S 1014 1 // use std C header here because <cstddef> is not always available
#include "stddef.h"
#endif
#ifndef NULL
#define NULL ((void*)0)
#endif

/// Definition of standard types
#ifdef USE_SYS_TYPES
#include "sys_types.h"

#else
#ifdef USE_STDINT
#include <stdint.h>
// we use the same standard types as in stdint.h, but with additional "s" for signed types
typedef int8_t                sint8_t;
typedef int16_t               sint16_t;
typedef int32_t               sint32_t;

// MISRA-C++ Rule 1-0-1: This declaration is of the non standard type long long
typedef int64_t               sint64_t; // PRQA S 47

#else
// GENERIC_VERSION
// the following typedefs are done in the global namespace because usage
// would be too complicated otherwise and the types will be used anywhere in the code.
// PRQA S 2400 ++

typedef unsigned char          uint8_t;
typedef unsigned short        uint16_t;
#ifdef PIKEOSCPPENV
typedef unsigned long         uint32_t;
#else
typedef unsigned int          uint32_t;
#endif
// MISRA-C++ Rule 1-0-1: This declaration is of the non standard type unsigned long long
typedef unsigned long long    uint64_t; // PRQA S 48


typedef signed char            sint8_t;
typedef signed short          sint16_t;
typedef signed int            sint32_t;

// MISRA-C++ Rule 1-0-1: This declaration is of the non standard type long long
typedef signed long long      sint64_t; // PRQA S 47

#endif
#endif

// MISRA-C++ Rule 0-4-2: Use of floating-point arithmetic shall be documented
// PRQA S 3708 ++
typedef float                float32_t;
typedef double               float64_t;

#ifdef __cplusplus
typedef bool                    bool_t;
#else
// This is for C files that use MeclTypes.h
typedef char                    bool_t;
// PRQA S 5172 2 // Macro definition of cpp keyword '...': we are in C context here
#define true                    (1U)
#define false                   (0U)
#endif

typedef int                     printInt_t; // used for casts in printf calls
typedef char                    char_t;

typedef enum { e_Degrees = 0, e_Radians } AngleUnit_e;

typedef enum { e_RowMajor = 0, e_ColumnMajor } MatrixOrder_e;

typedef enum { e_Default,
               e_UpperTriangle,
               e_LowerTriangle,
               e_Symmetric,
               e_AntiSymmetric,
               e_Sparse
             } MatrixType_e;

typedef enum { e_Standard = 0,  // Matrix standard output format
               e_Octave,        // Octave/Matlab readable output, take comment as lhs operand of assignment
               e_CStruct        // Expression for initializing a Matrix<T>::Config_s POD instance
             } MatrixOutputFormatStyle_e;


//! @struct MatrixOutputFormat_s
//! @brief Datastructure for defining output format of matrix print
typedef struct
{
  char_t headingChars[256];         ///< Characters to print before content
  char_t rowSeperatorChars[16];     ///< Characters to print between rows
  char_t columnSeperatorChars[16];  ///< Characters to print between columns
  char_t trailingChars[256];        ///< Characters to print after contents
} MatrixOutputFormat_s;

// Global macro definitions are needed to replace volatile handling
// PRQA S 1020, 1025 1
#define USE_VOLATILE 0

// PRQA S 2400 --

// this commented code is left here for later reference
// PRQA S 1051 5
//#define ME_TRUE                (true)  // MISRA-C++ Rule 16-2-2: This macro is replaced with a literal, use a constant of type 'const int' instead
//#define ME_FALSE              (false)  // MISRA-C++ Rule 16-2-2: This macro is replaced with a literal, use a constant of type 'const int' instead

//#define NULL                      (0)  // MISRA-C++ Rule 17-0-2: Reuse of name of standard library macro 'NULL'
//#define ME_NULL                   (0)  // MISRA-C++ Rule 4-10-2: The null-pointer-constant is not specified using the NULL macro

#if defined(_MSC_VER)
  // Visual Studio 2015 and older does anot accept alternative operator representations without specifically
  // including the ISO 646 header files.
  //
  // See http://en.cppreference.com/w/cpp/language/operator_alternative
  // See https://connect.microsoft.com/VisualStudio/feedback/details/751912/alternative-tokens-for-logical-operators-c-11
  //
  // This behaviour is NOT compliant with C++98 and newer standards.
  #include <ciso646>
#endif

#endif // MECLTYPES_H_
