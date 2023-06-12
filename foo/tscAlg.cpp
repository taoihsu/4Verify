// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [04-Jun-2013]
// --- Modified by Ehsan Parvizi [26-Aug-2014]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "tscAlg.h"
#include "tscApi.h"
#include "configuration.h"
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
using tsc::TSCAlg;

// ----------------------------------------------------------------------------
// --- The one and only application object and other globals

TSCAlg::TSCAlg() : Module()
{
}

TSCAlg::~TSCAlg()
{
}

// ----------------------------------------------------------------------------
// --- initialize the plug-in
bool_t TSCAlg::Init( tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po )
{   
    bool_t v_RetValue_b = true;
    if ( !preInit_b(b_TSCCtrlInfo_po) )
    {
        v_RetValue_b = false;
    }
    // --- init OK
    return v_RetValue_b;
}

// ----------------------------------------------------------------------------
bool_t TSCAlg::start_b( tscApi::enuCameraID i_CameraID_t )
{
    bool_t v_RetValue_b = true;
    tsc::TSCAlgImpl* v_Obj_po = getImplObject_pt(i_CameraID_t);
    if( (v_Obj_po == NULL) || (!v_Obj_po->isInitOk_b()) || (!v_Obj_po->start_b()) )
    {
        TRACE_0( m_hTracer, "Failed to start TSC implementation object!" );
        v_RetValue_b = false;
    }
    return v_RetValue_b;
}
// ----------------------------------------------------------------------------
// --- uninitialize the plug-in
void TSCAlg::uninit_v( void )
{
    TRACE_0( m_hTracer, "Uninit called" );

    uninitImplObjects_v();
}

void TSCAlg::reset_v ( void )
{
	resetImplObjects_v();
}

// ----------------------------------------------------------------------------
// --- process frame
bool_t TSCAlg::processFrame_b( void )
{
    bool_t v_RetValue_b = true;
    uint32_t frame = tSCCtrlInfo_px->getMFrmNmbr_u32();
    // --- process this frame
    CREATE_TIMER ( totalTimer );
    START_TIMER( totalTimer );
    TRACE_1( m_hTracer, "Processframe Begin: Frame[%lu]", m_pTSCCtrlInfo->GetM_FrmNmbr() );
    
    if ( !processImplObjects_b() )
    {
        STOP_TIMER( totalTimer );
        TRACE_1( m_hTracer, "Failed to process frame: Frame[%lu]", m_pTSCCtrlInfo->GetM_FrmNmbr() );
        v_RetValue_b = false;
    }

    else
    {
      STOP_TIMER( totalTimer );
      TRACE_2( m_hTracer, "Processframe End: Frame[%lu]. Took [%.0f] ms", m_pTSCCtrlInfo->GetM_FrmNmbr(), GET_ELAPSED_TIME(totalTimer) );
    }
    return v_RetValue_b;
}
//-----------------------------------------------------------------------------
void tsc::TSCConfig::SetWorldPtLimit(tsc_cfg::SofM_ConfigStrType i_Cst_t)
{
  worldPtLimit_t.x_x = i_Cst_t.limitXMM_f64;
  worldPtLimit_t.y_x = i_Cst_t.limitYMM_f64;
  worldPtLimit_t.z_x = i_Cst_t.limitZMM_f64;
  fundMatOutlierThresh_f64 = i_Cst_t.fundMatOutlierThresh_f64;
}

void tsc::TSCConfig::SetFCRestriction(uint32_t i_MaxValidFrame_u32, uint32_t i_MinRawFrame_u32)
{
  fCRestriction_s.maxNumValidFrames_u32 = i_MaxValidFrame_u32;
  fCRestriction_s.minNumRawFrames_u32 = i_MinRawFrame_u32;
}

//-----------------------------------------------------------------------------
bool_t TSCAlg::GetFinalCalibrationResult( tscApi::enuCameraID i_CameraID_t, tscApi::CalibrationParams_s* o_FinalCalibrationResult_ps )
{
    bool_t v_RetValue_b = true;
    if ( implArray_t[i_CameraID_t].isEnabled_b() )
    {
        v_RetValue_b = implArray_t[i_CameraID_t].GetFinalCalibrationResult( o_FinalCalibrationResult_ps );
    }
    else
    {
        // Not enabled
        v_RetValue_b = false;
    }
    return v_RetValue_b;
}

//-----------------------------------------------------------------------------
#ifdef DEBUG_TSC_ALG
bool_t TSCAlg::GetFinalCalibrationResultStdDev( tscApi::enuCameraID cameraID, tscApi::CalibrationParams_s* finalCalibrationResultStdDev )
{
    bool_t retValue = true;
    if ( implArray_t[cameraID].isEnabled_b() )
    {
        retValue = implArray_t[cameraID].GetFinalCalibrationResultStdDev( finalCalibrationResultStdDev );
    }
    else
    {
        // Not enabled
        retValue = false;
    }
    return retValue;
}

#endif

//-----------------------------------------------------------------------------
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)    // PRQA S 1070
bool_t TSCAlg::GetDetailedCalibrationResult( tscApi::enuCameraID cameraID, tsc::DetailedCalibrationResult* detailedCalibrationResult )
{
    bool_t retValue = true;
    if ( implArray_t[cameraID].isEnabled_b() )
    {
        retValue = implArray_t[cameraID].GetDetailedCalibrationResult( detailedCalibrationResult );
    }
    else
    {
        // Not enabled
        retValue = false;
    }
    return retValue;
}
#endif

//-----------------------------------------------------------------------------

