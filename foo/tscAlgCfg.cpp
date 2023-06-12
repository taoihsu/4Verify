// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [12-Apr-2013]
// --- Modified by Ehsan Parvizi [14-Aug-2014]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "configuration.h"
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
#include "tscAlg.h"

using tsc::TSCAlg;

bool_t TSCAlg::loadConfiguration_b( void )
{

    bool_t v_RetValue_b = true;
    tsc_cfg::tsc_ConfigStrType v_TscConfig_t;

    if(!tsc_cfg::LoadModuleConfiguration(tsc_cfg::e_TscModule,static_cast<void*>(&v_TscConfig_t)))
    {
        v_RetValue_b =false;
    }
    else
    {
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)    // PRQA S 1070
        bool_t iVal = v_TscConfig_t.tracerInfo.tracerEnable;
        if( iVal && (m_hTracer == 0) )
        {
            m_hTracer = tsc_trace::handleTracerConfig(&v_TscConfig_t.tracerInfo);
        }
#endif

      config_x.SetWorldPtLimit(v_TscConfig_t.sofMConfig_t);



// -- TODO: Need to be separated from TSC and AppCtrl to adjust configuration structures accordingly
      config_x.SetFCRestriction(v_TscConfig_t.maxNumValidFrames_u32, v_TscConfig_t.minNumRawFrames_u32);
    }
    
    // --- configuration successful
    return v_RetValue_b;
}

//-------------------------------------------------------------------------

bool_t TSCAlg::SetCameraSpecificConfig( const tscApi::enuCameraID i_CameraID_t )
{
    bool_t v_RetValue_b = true;
    tsc_cfg::tscCameraCfg_T v_TscCameraConfig_t;
    if(!tsc_cfg::LoadModuleConfiguration(tsc_cfg::e_TscCamera,static_cast<void*>(&v_TscCameraConfig_t)))
    {
        v_RetValue_b =false;
    }
    else
    {
    	cameraConfigs_ax[i_CameraID_t].setFullCalibration(v_TscCameraConfig_t.fullCalibration_ab[i_CameraID_t]);
    }

    return v_RetValue_b;
}
float32_t TSCAlg::TrimMeanPercentage_f32 = 0.0f;
void TSCAlg::setTrimMeanPercentage_v( float32_t percent_f32 )
{
    TSCAlg::TrimMeanPercentage_f32 = percent_f32;
}

bool_t TSCAlg::UpdateExternalConfiguration(tscApi::enuCameraID i_CameraID_t)
{

    /* Update the external configuration of the TSCImpl for this cameraID */
    implArray_t[i_CameraID_t].UpdateExternalConfiguration(i_CameraID_t);

    return true;
}
// ----------------------------------------------------------------------------
