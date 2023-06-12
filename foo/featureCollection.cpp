// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [04-Jun-2013]
// --- Modified by Ehsan Parvizi [26-Aug-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "featureCollection.h"
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// SB using std::string;

fc::FeatureCollection::FeatureCollection():
    Module( )
{
}

fc::FeatureCollection::~FeatureCollection()
{
}

// ----------------------------------------------------------------------------
// --- initialize the plug-in
bool_t fc::FeatureCollection::Init(tscApi::TSCCtrlInfo* b_TscCtrlInfo_po)
{
    bool_t v_Ret_b = true;
    if( !preInit_b(b_TscCtrlInfo_po) )
    {
        v_Ret_b = false;
    }
    else
    {
        kinematicModel_o.Init(b_TscCtrlInfo_po, &km::KinematicModel::getInstance_rt().getModuleInfo_rt(), km::KinematicModel::getInstance_rt().getModuleConfig_pt(), km::KinematicModel::getInstance_rt().getTracer_u64());
        // --- set Kinematic Model obj associated with this Feature Collector
        info_x.putMPKnmtcMdl_v(&kinematicModel_o);
        info_x.getMPKnmtcMdl_po()->ResetMotionVector( true );
    }
    // --- init OK
    return ( v_Ret_b );
}

// ----------------------------------------------------------------------------
// --- uninitialize the plug-in
void fc::FeatureCollection::uninit_v( void )
{
    TRACE_0( m_hTracer, "Uninit called" );
    info_x.putMPKnmtcMdl_v(NULL);
    uninitImplObjects_v();
}

void fc::FeatureCollection::reset_v ( void )
{
	info_x.getMPKnmtcMdl_po()->ResetMotionVector( true );
	resetImplObjects_v();
}

// ----------------------------------------------------------------------------
// --- process frame
bool_t fc::FeatureCollection::processFrame_b( void )
{
    bool_t v_Ret_b = true;
    uint32_t frame = tSCCtrlInfo_px->getMFrmNmbr_u32();
    
    if(!info_x.getMPKnmtcMdl_po()->process_b())
    {
        TRACE_0( m_hTracer, "Failed to process kinematic model object");
        v_Ret_b = false;
    }
    else
    {
        // --- process this frame
        CREATE_TIMER( totalTimer );
        START_TIMER( totalTimer );
        TRACE_1( m_hTracer, "Processframe Begin: Frame[%lu]", m_pTSCCtrlInfo->GetM_FrmNmbr() );
        
        // --- synchronize AppCtrl access from other plugins
        if ( !processImplObjects_b() )
        {
            STOP_TIMER( totalTimer );
            TRACE_1( m_hTracer, "Failed to process frame: Frame[%lu]", m_pTSCCtrlInfo->GetM_FrmNmbr() );
            v_Ret_b = false;
        }
        else
        {
            // reset the kinematic model motion vector before finishing this frame
        	info_x.getMPKnmtcMdl_po()->ResetMotionVector( false );
        
            STOP_TIMER( totalTimer );
            TRACE_2( m_hTracer, "Processframe End: Frame[%lu]. Took [%.0f] ms", m_pTSCCtrlInfo->GetM_FrmNmbr(), GET_ELAPSED_TIME( totalTimer ));
        }
    }
    return v_Ret_b;
}

//-----------------------------------------------------------------------------
