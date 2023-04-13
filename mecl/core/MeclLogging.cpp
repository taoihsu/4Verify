//--------------------------------------------------------------------------
/// @file MeclLogging.cpp
/// @brief Contains 
///
/// 
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

#include "MeclLogging.h"
#include "math/Math.h"
#include <ctype.h>          // isdigit
#include <stdlib.h>         // atoi
#include <cstdarg>          // the standard C++ wrapper for stdarg.h.

void vm_cprintfFloat(char const* i_Format_pc, ...)
{
  va_list args;
  va_start(args, i_Format_pc);

  while ('\0' != *i_Format_pc)
  {
    if ('%' == *i_Format_pc)
    {
      ++i_Format_pc;

      // printf uses 6 decimal places ...
      sint32_t v_NumDP_s32 = 6;

      //... if not further specified via %.f
      if ('.' == *i_Format_pc)
      {
        ++i_Format_pc;

        // get all the digits
        const uint8_t c_MaxDigits = 2U;
        char_t	v_Digits_ac[c_MaxDigits];
        uint8_t v_NumDigits_u32 = 0U;
        while (isdigit(*i_Format_pc))
        {
          if (v_NumDigits_u32 < c_MaxDigits)
          {
            v_Digits_ac[v_NumDigits_u32++] = i_Format_pc[0];
          }

          ++i_Format_pc;
        }

        // convert char array to integer
        v_NumDP_s32 = atoi(&v_Digits_ac[0]);
      }

      if ('f' == *i_Format_pc)
      {
        // print float
        float64_t v_Number_f64	= va_arg(args, float64_t);
        sint32_t  v_NumPDP_s32	= mecl::math::numeric_conversions<float64_t>::getNumDecimals_s32(v_Number_f64, 10.0);


        char_t buffer_ac[128];
        if (mecl::math::numeric_limits<float64_t>::isNaN(v_Number_f64) != 0)
        {
          sprintf(&buffer_ac[0], "NAN");
        }
        else if (v_Number_f64 == mecl::math::numeric_limits<float64_t>::infinity_x())
        {
          sprintf(&buffer_ac[0], "INF");
        }
        else
        {
          mecl::math::numeric_conversions<float64_t>::getMantissaAndExponent_v(v_Number_f64, v_NumPDP_s32, v_NumDP_s32, 10.0, &buffer_ac[0]);
        }

        vm_cprintf(buffer_ac);
      }
    }
    else
    {
      vm_cprintf("%c", i_Format_pc[0]);
    }

    i_Format_pc++;
  }

  va_end(args);
}


#ifdef PIKEOSCPPENV

namespace mecl
{
const char_t* getProcName_pc()
{
    static vm_procinfo_t g_procInfo_t;
    static bool_t gotProgName = false;

    if (!gotProgName)
    {
        vm_procinfo(VM_RESPART_MYSELF, VM_PROC_MYSELF, &g_procInfo_t);
    }

    return &g_procInfo_t.appname[5]; // remove Proc_
}
}
#else

#endif
