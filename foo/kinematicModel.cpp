// ----------------------------------------------------------------------------
// --- Written by Xai Phan [04-Feb-2013]
// --- Modified by Rathi G. R. [12-Apr-2013] for architectural change of implementation
// --- Modified by Ehsan Parvizi [26-Aug-2014]
// --- Modified by Hany Kashif [26-Sep-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "kinematicModel.h"
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//PRQA S 3708 EOF
using km::KinematicModel;
KinematicModel::KinematicModel()
    : Module()
{
}

KinematicModel::~KinematicModel()
{
}

// ----------------------------------------------------------------------------
// --- initialize the plug-in
bool_t KinematicModel::Init( tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po )
{
    bool_t v_Status_b = true;

    if( !preInit_b( b_TSCCtrlInfo_po ) )
    {
        v_Status_b = false;
    }

    // --- init OK
    return v_Status_b;
}

// ----------------------------------------------------------------------------
void KinematicModel::uninit_v( void )
{
    uninitImplObjects_v();
}

void KinematicModel::reset_v( void )
{
    resetImplObjects_v();
}

// ----------------------------------------------------------------------------
// --- process frame
bool_t KinematicModel::processFrame_b( void )
{
    bool_t v_Status_b = true;
    uint32_t frame = tSCCtrlInfo_px->getMFrmNmbr_u32();
    // --- process this frame
    TRACE_1( m_hTracer, "Processframe Begin: Frame[%lu]", m_pTSCCtrlInfo->GetM_FrmNmbr() );
    // --- First Compute Vehicle Displacement
    computeVehicleDisplacement_v();

    // Process Implementation Objects
    if( !processImplObjects_b() )
    {
        TRACE_1( m_hTracer, "Failed to process frame: Frame[%lu]", m_pTSCCtrlInfo->GetM_FrmNmbr() );
        v_Status_b = false;
    }
    else
    {
        TRACE_1( m_hTracer, "Processframe End: Frame[%lu]", m_pTSCCtrlInfo->GetM_FrmNmbr() );
    }

    return v_Status_b;
}

//-------------------------------------------------------------------------

void KinematicModel::computeVehicleDisplacement_v()
{
    if( isInitialReading_b )
    {
        isInitialReading_b = false;
        TRACE_4( m_hTracer, "ComputeVehicleDisplacement Frame[%lu]: Accumulated [ %lf, %lf, %lf ]",
                 m_pTSCCtrlInfo->GetM_FrmNmbr(), m_info.GetM_AccmltdDsplcmnt_COG()[ 0 ],
                 m_info.GetM_AccmltdDsplcmnt_COG()[ 1 ], m_info.GetM_AccmltdHdng() );
    }
    else
    {
        float64_t v_FrontWheelAngleRad_f64 =  - tSCCtrlInfo_px->getMWhlAngl_f32();
        //float64_t v_FrontWheelAngleRad_f64 =  - tSCCtrlInfo_px->getMHtchAngl_f32(); //JTURK check this
        float64_t v_RearWheelAngleRad_f64 = 0;
        // --- calculate the model
        info_x.putMVhclSlpAnglRd_v( atan( ( info_x.getMDstncCG2FrntAxsMM_f64() *tan( v_RearWheelAngleRad_f64 ) +
                                            info_x.getMDstncCG2RrAxsMM_f64() *tan( v_FrontWheelAngleRad_f64 ) ) /
                                          ( info_x.getMDstncCG2FrntAxsMM_f64() +
                                            info_x.getMDstncCG2RrAxsMM_f64() ) ) );
        // --- Temporarily overriding until a fix is found for wheel stat accuracy problem
        // --- convert the CAN speed from kph to distance in mm, assuming 30 fps
        info_x.putMMnTrvldDstncMM_v( tSCCtrlInfo_px->getMSpd_f32() / 3.6 * 1000.0 / 30.0 );

        // Store the current WheelStat to be used for computation in the next frame

        // --- If Rear gear, reverse the traveled distance
        if( tSCCtrlInfo_px->getMGrDrctn_s32() == tscApi::TSCCtrlInfo::e_GearReverse )
        {
            info_x.putMMnTrvldDstncMM_v( - info_x.getMMnTrvldDstncMM_f64() );
        }

        info_x.putMHdngIncrmnt_v( info_x.getMMnTrvldDstncMM_f64() * \
                                  cos( info_x.getMVhclSlpAnglRd_f64() ) * \
                                  ( tan( v_FrontWheelAngleRad_f64 ) - tan( v_RearWheelAngleRad_f64 ) ) /  \
                                  ( info_x.getMDstncCG2FrntAxsMM_f64() +  \
                                    info_x.getMDstncCG2RrAxsMM_f64() ) );
    }
}
// ----------------------------------------------------------------------------
