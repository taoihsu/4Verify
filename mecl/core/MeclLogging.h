//--------------------------------------------------------------------------
/// @file MeclLogging.h
/// @brief Contains a common definition for logging purposes.
/// This is due to be removed when DLT (data logging tool via CAN) is available.
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Michael Schulte (michael.schulte@magna.com)
///
//  --------------------------------------------------------------------------

#ifndef MECLLOGGING_H_
#define MECLLOGGING_H_

#include "MeclTypes.h"

#ifndef PIKEOSCPPENV
//       COMMON std c++ part
// PRQA S 5188  1 // use library for pikeos
#include <cstdio>
#define vm_cprintf(...) \
     printf(__VA_ARGS__); fflush(stdout)
#define vm_cputs(...) \
     printf(__VA_ARGS__)
#define log_printf(...) vm_cprintf(__VA_ARGS__)

#else
//           PIKEOS part

#ifdef RELEASE_BUILD
// Define this macro to prevent inclusion of std vm_cprintf and vm_cputs
// These functions are removed here
#define VM_CONSOLE_H
#define vm_cprintf(...)
#define vm_cputs(x)
#endif

extern "C"
{
#include <vm.h>
}
#ifndef VM_H // this is actually just for eclipse parser
#define vm_cprintf(...)
#endif

namespace mecl
{
const char_t* getProcName_pc();
}

// PRQA S 1020 EOF // TODO macro for printf will be removed when DLT is available
#define log_printf(...) \
    do \
    { \
        vm_cprintf("%s: ", mecl::getProcName_pc()); \
        vm_cprintf(__VA_ARGS__); \
    } while (false)

#endif

// --------------------------------------------------------------------------
/// @brief provides a printf for floats on the target
///
/// use %f to print a float32 or float64. 
/// use %.4f to print a float and specifying the number of digits after comma.
/// 
/// @par Example usage:
/// @snippet CoreTester.cpp use_vm_cprintfFloat
///
/// @param[in] v_Format_pc the string to format.
/// @param[in] ... the variable arguments
// --------------------------------------------------------------------------
void vm_cprintfFloat(char const* i_Format_pc, ...);

#endif // MECLLOGGING_H_


