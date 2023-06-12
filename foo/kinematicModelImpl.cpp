// ----------------------------------------------------------------------------
// --- Written by Xai Phan [04-Feb-2013]
// --- Modified by Rathi G. R. [12-Apr-2013] for architectural change of implementation
// --- Modified by Ehsan Parvizi [6-May-2013] for design changes
// --- Modified by Ehsan Parvizi [07-Jun-2013] for dual rate execution model
// --- Modified by Rathi G. R. [25-Jun-2013] for Impl object pool re-use
// --- Modified by Ehsan Parvizi [09-Sep-2013] for performance optimization
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2013
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "mathOperations.h"
#include "kinematicModel.h"
// ----------------------------------------------------------------------------
// PRQA S 3708 EOF
using km::KinematicModelImpl;

KinematicModelImpl::KinematicModelImpl() :
    ModuleImpl(),
    isCameraOffsetSet_b(false),
    eadingInRadians_f64(0)
    //m_DisplacementCoG(),
    //m_CameraOffset_mm(),
    //m_CameraRotationMatrix()
{
    // Using initialization lists
  memset( displacementCoG_af64     , 0x0, sizeof( displacementCoG_af64      ) );
  memset( cameraOffsetMM_af64     , 0x0, sizeof( cameraOffsetMM_af64      ) );
  memset( cameraRotationMatrix_af64, 0x0, sizeof( cameraRotationMatrix_af64 ) );
}

bool_t KinematicModelImpl::Init(tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po, km::KinematicModelInfo* b_Info_po, km::KinematicModelConfig* b_Config_po, km::KinematicModelCameraConfig* b_CameraConfig_po, sint64_t i_Tracer_s64, tscApi::enuCameraID i_CameraID_t)
{
    bool_t v_Status_b = true;
    if ( !preInit_b(b_TSCCtrlInfo_po, b_Info_po, b_Config_po, b_CameraConfig_po, i_Tracer_s64, i_CameraID_t) )
    {
        v_Status_b = false;
    }
    else
    {
    initOK_b = true;
    }
    return v_Status_b;
}

//Camera generic initialization. Used particularly by the KM impl pbject for the FC module.
bool_t KinematicModelImpl::Init(tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po, km::KinematicModelInfo* b_Info_po, km::KinematicModelConfig* b_Config_po, sint64_t i_Tracer_s64)
{
    return Init(b_TSCCtrlInfo_po, b_Info_po, b_Config_po, NULL, i_Tracer_s64, tscApi::e_TscFrontCam);   //Front camera ID is just a dummmy argument.
}

bool_t KinematicModelImpl::start_b(void)
{
  return false;
}

bool_t KinematicModelImpl::loadConfiguration_b(void)
{
  return true;
}

// ----------------------------------------------------------------------------
void KinematicModelImpl::SetCameraOffset( float64_t i_XOffsetMM_f64, float64_t i_YOffsetMM_f64, float64_t i_AngleOffsetDeg_f64 )
{
    // Negating the EOL values for translation from Vehicle to Kinematic Coordinate System
    cameraOffsetMM_af64[ 0 ] =  - i_XOffsetMM_f64 + info_px->getMDstncCG2FrntAxsMM_f64();
    cameraOffsetMM_af64[ 1 ] =  - i_YOffsetMM_f64;
    cameraOffsetMM_af64[ 2 ] =  1.0;

    float64_t v_Theta_f64 = tsc_math::Degrees2Radians( i_AngleOffsetDeg_f64 );
    float64_t v_CosTheta_f64 = cos( v_Theta_f64 );
    float64_t v_SinTheta_f64 = sin( v_Theta_f64 );

    float64_t cameraRotationMatrix[ 2][ 2 ] = 
    {
        {
            v_CosTheta_f64, v_SinTheta_f64
        }

        , 
        {
             - v_SinTheta_f64, v_CosTheta_f64
        }
    };

    memcpy( cameraRotationMatrix_af64, cameraRotationMatrix, sizeof( cameraRotationMatrix_af64 ) );

    isCameraOffsetSet_b = true;
}

// ----------------------------------------------------------------------------
void KinematicModelImpl::ResetMotionVector( bool_t i_Accumulate_b )
{
    if ( i_Accumulate_b )
    {
        eadingInRadians_f64 = info_px->getMAccmltdHdng_f64();
        memcpy( displacementCoG_af64, info_px->getMAccmltdDsplcmntCOG_pf64(), sizeof( displacementCoG_af64 ) );
    }
    else
    {
        eadingInRadians_f64 = 0;
        memset( displacementCoG_af64, 0, sizeof( displacementCoG_af64 ) );
    }


    TRACE_4( m_hTracer, "ResetMotionVector Impl[%lu] Frame[%lu]: Main_MV [ %lf, %lf]",
        reinterpret_cast< uint64_t > ( this ), m_pTSCCtrlInfo->GetM_FrmNmbr(), displacementCoG_af64[ 0 ], displacementCoG_af64[ 1 ]);
}

// ----------------------------------------------------------------------------
bool_t KinematicModelImpl::GetMotionVector( float64_t& o_DeltaXMM_rf64, float64_t& o_DeltaYMM_rf64, float64_t& o_HeadingInRadians_rf64 ) const
{
    // --- key frame processing support
    bool_t v_Status_b = true;

    if( !isCameraOffsetSet_b )
    {
        TRACE_0( m_hTracer, "GetMotionVector: Called without camera offset being set");

        v_Status_b = false;
    }
    else
    {

        // asking for the main motion vector

        o_HeadingInRadians_rf64 = eadingInRadians_f64;

    // 3x3 Rotation-Translation Matrix (S1, S2, Psi)
    float64_t transformationMatrix[ 3][ 3 ] = 
    {
        {
            cos( o_HeadingInRadians_rf64 ),  - sin( o_HeadingInRadians_rf64 ), ( displacementCoG_af64[ 0 ] - cameraOffsetMM_af64[ 0 ] )
        }

        , 
        {
            sin( o_HeadingInRadians_rf64 ), cos( o_HeadingInRadians_rf64 ), ( displacementCoG_af64[ 1 ] - cameraOffsetMM_af64[ 1 ] )
        }

        , 
        {
            0, 0, 1.0
        }
    };

    // 3x1 Vector displacement in Kinematic Model Coordinate System
    float64_t v_CameraShiftInKmcs_af64[ 3 ];
    tsc_math::MatrixVectorMultiply( &transformationMatrix[ 0][ 0 ], 3, 3, false, &cameraOffsetMM_af64[ 0 ], &v_CameraShiftInKmcs_af64[ 0 ] );

    // 2x1 Vector displacement in Camera Coordinate System
    float64_t v_CameraDisplacement_af64[ 2 ];
    // We only access the first two elements of cameraShiftInKmcs vector
    tsc_math::MatrixVectorMultiply( &cameraRotationMatrix_af64[ 0][ 0 ], 2, 2, false, &v_CameraShiftInKmcs_af64[ 0 ], &v_CameraDisplacement_af64[ 0 ] );

    o_DeltaXMM_rf64 = v_CameraDisplacement_af64[ 0 ];
    o_DeltaYMM_rf64 = v_CameraDisplacement_af64[ 1 ];

    TRACE_2( m_hTracer, "GetMotionVector: [%lf, %lf]", o_DeltaXMM_rf64, o_DeltaYMM_rf64);

    }
    return v_Status_b;
}

void KinematicModelImpl::updateMotionVector_v(void)
{
    eadingInRadians_f64 += info_px->getMHdngIncrmnt_f64();
    // --- store path traveled w.r.t. the center of gravity (not modified to the camera position)
    displacementCoG_af64[ 0 ] += info_px->getMMnTrvldDstncMM_f64() *cos( eadingInRadians_f64 + info_px->getMVhclSlpAnglRd_f64() );
    displacementCoG_af64[ 1 ] += info_px->getMMnTrvldDstncMM_f64() *sin( eadingInRadians_f64 + info_px->getMVhclSlpAnglRd_f64() );

    // --- key frame processing support
}

// ----------------------------------------------------------------------------
bool_t KinematicModelImpl::process_b(void)
{
    updateMotionVector_v();
    //int frame = tSCCtrlInfo_po->getMFrmNmbr_u32();
    // --- key frame processing support

    TRACE_4( m_hTracer, "Process Impl[%llu] Frame[%lu]: Main_MV [ %lf, %lf]; Key_MV [ %lf, %lf]",
        reinterpret_cast< uint64_t > ( this ), m_pTSCCtrlInfo->GetM_FrmNmbr(), displacementCoG_af64[ 0 ], displacementCoG_af64[ 1 ]);

    // --- process successful
    return true;
}

// ----------------------------------------------------------------------------
bool_t KinematicModelImpl::unInit_b(void)
{
	reset_v();

    // --- initialize the variables to their reset state
    hTracer_s64 = 0;
    tSCCtrlInfo_po = NULL;
    config_px = NULL;
    cameraConfig_px = NULL;
    info_px = NULL;

    initOK_b = false;
    cameraID_t = tscApi::e_TscFrontCam;

    return true;
}

// ----------------------------------------------------------------------------
void KinematicModelImpl::reset_v(void)
{
    enable_b = false;
    isCameraOffsetSet_b = false;

    eadingInRadians_f64 = 0;
    memset( displacementCoG_af64, 0, sizeof( displacementCoG_af64 ) );

    memset( cameraOffsetMM_af64, 0, sizeof( cameraOffsetMM_af64 ) );
    memset( cameraRotationMatrix_af64, 0, sizeof( cameraRotationMatrix_af64 ) );

    availableFrameNumber_u32 = 0;
}

void KinematicModelImpl::cleanupLocalData_v(void)
{
}
// ----------------------------------------------------------------------------

