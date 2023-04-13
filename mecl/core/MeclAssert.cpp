//--------------------------------------------------------------------------
/// @file MeclAssert.cpp
/// @brief Contains the Assert macro implementation.
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

//#include "stdafx.h"
#include "MeclAssert.h"

// PRQA S 1051 16
#ifdef PIKEOSCPPENV
extern "C"
{
#include <string.h>
// PRQA S 5188 1 // needed for snprintf function
#include <stdio.h>
}
#endif
#ifdef PIKEOSDEBUG
extern "C"
{
/* init_gdbstub(), gdb_breakpoint(). */
#include <vm_debug.h>
#include <p4ext/p4ext_threads.h>
}
#endif

#include <assert.h>

namespace mecl
{
namespace core
{

// --------------------------------------------------------------------------
/// @brief Test for null pointer
///
/// Function tests argument value \p i_Pointer_p against NULL
///
/// @param[in] i_Pointer_p Pointer to be tested
/// @return Boolean, True = null pointer, False = valid non-null pointer
// --------------------------------------------------------------------------
bool_t isNull_b (const void *i_Pointer_p)
{
    return i_Pointer_p == NULL;
}

// --------------------------------------------------------------------------
/// @brief Test for non-null pointer
///
/// Function tests argument value \p i_Pointer_p against NULL. Function
/// is accessed through the AssertNotNull() macro
///
/// @param[in] i_Pointer_p Pointer to be tested
/// @return Boolean, True = valid non-null pointer, False = null pointer
// --------------------------------------------------------------------------
bool_t isNotNull_b (const void *i_Pointer_p)
{
    return i_Pointer_p != NULL;
}

// --------------------------------------------------------------------------
/// @brief Assertion abstraction
///
/// Function implements call redirect to system specific assertion interface. Function
/// is never directly called but accessed through the AssertMsg() and Assert() macros
///
/// @param[in]  i_Message_pc  Assertion message text
/// @param[in]  i_File_pc     File name in which assertion failed
/// @param[in]  i_Line_i32    Line number in which assertion failed
/// @return     void          
// --------------------------------------------------------------------------
void AssertExt_v(const char_t i_Message_pc[],
                 const char_t i_File_pc[],
                 const sint32_t i_Line_i32)
{
#ifdef PIKEOSDEBUG
  gdb_breakpoint();
#endif

#ifdef PIKEOSCPPENV
  static const uint32_t c_AssertMsgLength_u32 = 256U;
  static char_t v_AssertMsg_ac[c_AssertMsgLength_u32];

  snprintf(&v_AssertMsg_ac[0], (sizeof(v_AssertMsg_ac))-1U, "%s: Assert failed: in %s:%d: %s",
           getProcName_pc(), i_File_pc, static_cast<printInt_t>(i_Line_i32), i_Message_pc);
  vm_cputs(&v_AssertMsg_ac[0]);

  // only use a small part of the assertion to send to the system (filename without path + line number):
  char_t* v_MsgPtr_pc = &v_AssertMsg_ac[strnlen(&v_AssertMsg_ac[0], c_AssertMsgLength_u32) - strnlen(i_Message_pc, c_AssertMsgLength_u32)];
  while ((v_AssertMsg_ac != v_MsgPtr_pc) && (*v_MsgPtr_pc != '/'))
  {
    --v_MsgPtr_pc;
  }
  if (v_AssertMsg_ac != v_MsgPtr_pc)
  {
    ++v_MsgPtr_pc; // remove the leading '/' character
  }

  // send application error to health monitor. This should be handled by the App::OsHealthMonitorListener
  // and then by the error manager component
  // PRQA S 0403 8
  vm_e_t v_Ret_t = vm_hm_raise_error(VM_HM_EI_APP_ERROR,
      VM_HM_ERR_MSG_T_CUSTOM, v_MsgPtr_pc, strnlen(v_MsgPtr_pc, c_AssertMsgLength_u32));
  if (VM_E_OK != v_Ret_t)
  {
    // raising the error failed: delegate to "usual" assert
    // PRQA S 1051 2
    __assert(i_Message_pc, i_File_pc, i_Line_i32, getProcName_pc());
  }
#else

  log_printf("ASSERT FAILED %s - %s:%u\n", i_Message_pc, i_File_pc, i_Line_i32);
  assert(false);
#endif
}
}
}
/// @}
/// @}

