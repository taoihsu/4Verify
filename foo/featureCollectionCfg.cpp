// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [12-Apr-2013]
// --- Modified by Ehsan Parvizi [29-Jul-2014]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "configuration.h"
#include "featureCollection.h"
// ----------------------------------------------------------------------------
// PRQA S 1070 EOF
// SB using std::string;
using tsc_trace::kCameraStrings;
// ----------------------------------------------------------------------------

//-------------------------------------------------------------------------
bool_t fc::FeatureCollection::loadConfiguration_b()
{
    bool_t v_Ret_b = true;

    tsc_cfg::fc_ConfigStrType v_FcConfigData_t;

    if(!tsc_cfg::LoadModuleConfiguration(tsc_cfg::e_FcModule, reinterpret_cast<void*>(&v_FcConfigData_t)))
    {
        v_Ret_b = false;
    }
    else
    {
        // --- look for tracer ----------------------------------------------------
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
        bool_t iVal = v_FcConfigData_t.tracerInfo.tracerEnable;

        if( iVal && (m_hTracer == 0) )
        {
            m_hTracer = tsc_trace::handleTracerConfig(&v_FcConfigData_t.tracerInfo);
        }
#endif
    }
    // --- configuration successful
    return v_Ret_b;
}

bool_t fc::FeatureCollection::UpdateExternalConfiguration(tscApi::enuCameraID i_CameraID_t)
{
    return implArray_t[i_CameraID_t].UpdateExternalConfiguration(i_CameraID_t);
}

// ---------------------------------- EOF -------------------------------------
