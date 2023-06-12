// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [12-Apr-2013]
// --- Modified by Ehsan Parvizi [24-Jun-2014]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "configuration.h"
#include "kinematicModel.h"
// ----------------------------------------------------------------------------
// SB using std::string;
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
bool_t km::KinematicModel::loadConfiguration_b(void)
{
    bool_t v_Status_b = true;
    tsc_cfg::km_ConfigStrType v_KmConfigData_t;

    if(!tsc_cfg::LoadModuleConfiguration(tsc_cfg::e_KmModule, static_cast<void*> (&v_KmConfigData_t)))
    {
        v_Status_b = false;
    }
    else
    {


    // --- look for tracer ----------------------------------------------------
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)   // PRQA S 1070
    sint32_t iVal = v_KmConfigData_t.tracerInfo.tracerEnable;
    if( (iVal > 0) && (m_hTracer == 0) )
    {
        m_hTracer = tsc_trace::handleTracerConfig(&v_KmConfigData_t.tracerInfo);
    }
#endif


    info_x.putMStrghtMtnDstncThrshMM_v(v_KmConfigData_t.straightMotionDistanceThreshMM_f64);
    UpdateExternalConfiguration(static_cast<tscApi::enuCameraID>(0));
    }

    // --- configuration successful
    return v_Status_b;
}

// ----------------------------------------------------------------------------

bool_t km::KinematicModel::UpdateExternalConfiguration(tscApi::enuCameraID i_CameraID_t)
{
    tscApi::km_ExtConfigStrType v_KmExtConfigData_t;

    /* Fill the structure with the data elements from the Platform Buffer */
    CExtConfig::KM_LoadFromPlatform(i_CameraID_t, &v_KmExtConfigData_t);

    /* Update the parameters in the KM module, while the ImplObjects will be updated through
     * the feature collector update */
    config_x.PutM_TrCrcmfrncPrPls_mm(v_KmExtConfigData_t.tireCircumferencePerPulseMM_f32);

    info_x.putMDstncCG2FrntAxsMM_v(v_KmExtConfigData_t.distanceCoG2FrontAxisMM_f64);

    info_x.putMDstncCG2RrAxsMM_v(v_KmExtConfigData_t.distanceCoG2RearAxisMM_f64);

    return true;
}

// ---------------------------------- EOF -------------------------------------
