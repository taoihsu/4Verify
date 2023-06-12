// ----------------------------------------------------------------------------
// --- Written by Hany Kashif
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
// PRQA S 1070 EOF
#include "stdafx.h"
#include "kinematicModel.h"
#include "featureCollection.h"
#include "tscAlg.h"
#include "tscApi.h"
using tsc::TSCAlg;
using km::KinematicModel;

namespace
{
  enum InternalTSCState_e
  {
    e_InternalStateNotReady,
    e_InternalStateRunning,
    e_InternalStateSuspended,
    e_InternalStateTerminated
  };
  InternalTSCState_e internalState;

  // flag to record if TSCAlg has been paused
  // set when TSC_Resume() called
  // clear when processing frame after TSCAlg resumes
  // also clear when TSCAlg is terminated, i.e. TSC_Stop() called
  bool_t wasPaused;

  // flag to check if TSCAlg has ever received ext configs
  // set when all ext configs are updated to local
  // clear when TSC_Init() called
  bool_t extConfiged[tscApi::e_TscNumCam];

  // state if any error occurs out of TSCAlg boundary, i.e.
  // configuration error, backup date error
  tscApi::TSCError_e stateErr[tscApi::e_TscNumCam];

  bool_t IsValidCameraID(tscApi::enuCameraID i_ID_t)
  {
      return ( (i_ID_t >= tscApi::e_TscFrontCam) && (i_ID_t < tscApi::e_TscNumCam) );
  }

  bool_t ResetAllCameras(void)
  {
    KinematicModel::getInstance_rt().reset_v();
    fc::FeatureCollection::getInstance_rt().reset_v();
    TSCAlg::getInstance_rt().reset_v();
    return true;
  }

  bool_t ResetCamera(tscApi::enuCameraID i_CameraID_t)
  {
    KinematicModel::getInstance_rt().getImplObject_pt(i_CameraID_t)->reset_v();
    fc::FeatureCollection::getInstance_rt().getImplObject_pt(i_CameraID_t)->reset_v();
    TSCAlg::getInstance_rt().getImplObject_pt(i_CameraID_t)->reset_v();
        TSCAlg::getInstance_rt().getImplObject_pt( i_CameraID_t )->setTargetCamera( i_CameraID_t );
    return true;
  }

  bool_t cleanup_v(void)
  {
    fc::FeatureCollection::getInstance_rt().cleanup_v();
    return true;
  }

  bool_t RestoreTSCSavedData(tscApi::enuCameraID i_CameraID_t, tscApi::TSCSavedDataInfo_s const * i_TSCSavedData_ps)
  {
    // ValidateSavedData() checks the integritiy of saved data
    // RestoreSavedData() returns true
    return ( fc::FeatureCollection::getInstance_rt().getImplObject_pt(i_CameraID_t)->ValidateSavedData(i_TSCSavedData_ps) &&
             fc::FeatureCollection::getInstance_rt().getImplObject_pt(i_CameraID_t)->restoreSavedData_b() );
  }

}

// ----------------------------------------------------------------------------
bool_t TSC_Init(tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po)
{
    bool_t v_Status_b = false;

	OC_DEBUG_PRINTF("internalState: %d\n",internalState);
    // TSC_Init() is accepted only in NotReady state
    if ( internalState != e_InternalStateNotReady )
    {
        // as TSCAlg only record error state per camera, leave error code as is
    } else if (b_TSCCtrlInfo_po == NULL)
    {
        // check parameter: pTSCCtrlInfo can NOT be NULL
        // as TSCAlg only record error state per camera, leave error code as is
    } else
    {
        // initialize tscApi variables
        wasPaused = false;
        for(uint8_t v_Index_u8 = 0; v_Index_u8 < tscApi::e_TscNumCam; v_Index_u8++)
        {
            extConfiged[v_Index_u8] = false;
            stateErr[v_Index_u8] = tscApi::e_TscErrorNoError;
        }

        // load default ML configurations
        CExtConfig::loadPlatformDefaultConfig();
        OC_DEBUG_PRINTF( "%d, %d, %u, %d, %u\n"
            ,b_TSCCtrlInfo_po->getMCmrFrntOpn_b()
            ,b_TSCCtrlInfo_po->getMFrmNmbr_u32()
            ,b_TSCCtrlInfo_po->getMVdHght_u16()
            ,b_TSCCtrlInfo_po->getMGrDrctn_s32()
            ,b_TSCCtrlInfo_po->getMVdWdth_u16()
        );
        // here no need to check the return value of KM Init and FC Init,
        // as the error code will be set by TSCAlg Init, so check return
        // value from TSCAlg Init
        KinematicModel::getInstance_rt().Init(b_TSCCtrlInfo_po);
        fc::FeatureCollection::getInstance_rt().Init(b_TSCCtrlInfo_po);
        if (!TSCAlg::getInstance_rt().Init(b_TSCCtrlInfo_po))
        {
            for(uint8_t v_Index_u8 = 0; v_Index_u8 < tscApi::e_TscNumCam; v_Index_u8++)
            {
                stateErr[v_Index_u8] = TSCAlg::getInstance_rt().getTSCError_e(static_cast<tscApi::enuCameraID>(v_Index_u8));
                OC_DEBUG_PRINTF("stateErr[%d]: %d\n",v_Index_u8, stateErr[v_Index_u8]);
            }
            KinematicModel::getInstance_rt().uninit_v();
            fc::FeatureCollection::getInstance_rt().uninit_v();
            TSCAlg::getInstance_rt().uninit_v();
        } else {
            internalState = e_InternalStateRunning;
            v_Status_b = true;
        }
    }

    return v_Status_b;
}

// ----------------------------------------------------------------------------
bool_t TSC_Start(tscApi::enuCameraID i_CameraID_t, const tscApi::tscPlatformExtConfigType * i_TSCConfigDataPtr_pt, const tscApi::TSCSavedDataInfo_s * i_TSCSavedData_ps)
{
    bool_t v_Status_b = false;
    TSCAlg::setTrimMeanPercentage_v( i_TSCConfigDataPtr_pt->trimMeanPercentage_f32 );
    km::KinematicModel::getInstance_rt().getModuleInfo_rt().putMStrghtMtnDstncThrshMM_v( i_TSCConfigDataPtr_pt->kinematicModelExternalConfig_t.distanceThreshMM );
    //tsc_cfg::currentPlatformExtConfig->kinematicModelExternalConfig_t.distanceThreshMM = i_TSCConfigDataPtr_pt->kinematicModelExternalConfig_t.distanceThreshMM;
    tsc_cfg::currentPlatformExtConfig->MaxNumValidFrames_i32 = i_TSCConfigDataPtr_pt->MaxNumValidFrames_i32;
    tsc_cfg::currentPlatformExtConfig->MinNoRawFrames_i32 = i_TSCConfigDataPtr_pt->MinNoRawFrames_i32;
#ifdef USE_SVSCM
    OC_DEBUG_PRINTF(( "internalState: %d, camstate: %d, %d, %d, %d, %lu, %d\n"
        ,internalState
        ,TSC_GetState(i_CameraID_t)
        ,i_CameraID_t
        ,(sint32_t)(i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.extrinsicParams.Pitch_deg*10000)
        ,i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.extrinsicParams.flipped
        ,i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.intrinsicParams.downsampleFactor
        ,i_TSCConfigDataPtr_pt->featureColExternalConfig_t.BMALFC_extConfig.rois[0].width
    ));

    OC_DEBUG_PRINTF(( "iS:%d,cS:%d,c:%d,pxSz*10k:%d,ppx:%d,ppy:%d,fl:%d,p:%d,y:%d,r:%d,x:%d,y:%d,z:%d,fl %d,df %lu,roi_wd %d\n"
        ,internalState
        ,TSC_GetState(i_CameraID_t)
        ,i_CameraID_t
        ,static_cast<sint32_t>(i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.intrinsicParams.pixelSize*10000)
        ,static_cast<sint32_t>(i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.intrinsicParams.ppx*10000)
        ,static_cast<sint32_t>(i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.intrinsicParams.ppy*10000)
        ,static_cast<sint32_t>(i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.intrinsicParams.focalLength*10000)
        ,static_cast<sint32_t>(i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.extrinsicParams.Pitch_deg*10000)
        ,static_cast<sint32_t>(i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.extrinsicParams.Yaw_deg*10000)
        ,static_cast<sint32_t>(i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.extrinsicParams.Roll_deg*10000)
        ,static_cast<sint32_t>(i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.orientationParams.deltaX*10000)
        ,static_cast<sint32_t>(i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.orientationParams.deltaY*10000)
        ,static_cast<sint32_t>(i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.extrinsicParams.Z_mm*10000)
        ,i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.extrinsicParams.flipped
        ,i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.intrinsicParams.downsampleFactor
        ,i_TSCConfigDataPtr_pt->featureColExternalConfig_t.BMALFC_extConfig.rois[1].x_x
    ));
#endif
    // check parameters: 
    // cameraID should be in range
    // both TSC_ConfigDataPtr and pTSCSavedData can be NULL
    if ( !IsValidCameraID(i_CameraID_t) )
    {
        // as TSCAlg only record error state per camera, leave error code as is
    } else
    if ( (internalState == e_InternalStateTerminated) || 
         ((internalState == e_InternalStateRunning) && (TSC_GetState(i_CameraID_t) == tscApi::e_TscStateInitOk)) ||
         ((internalState == e_InternalStateRunning) && (TSC_GetState(i_CameraID_t) == tscApi::e_TscStateCalibrationCompleted)) )
    { 
        /* if back from 'Terminated', reset all cameras, otherwise, in 'Running', reset assigned camera */
        if (internalState == e_InternalStateTerminated)
        {
            ResetAllCameras();
            internalState = e_InternalStateRunning;
        }
        else
        {
            ResetCamera(i_CameraID_t);
            OC_DEBUG_PRINTF( ( "ResetCamera\n" ) );
        }

        /* If NULL Pointer, No update to external configuration is required */
        if ( i_TSCConfigDataPtr_pt != NULL)
        {
            /* Check on the provided platform data */
            if ( tsc_cfg::ValidateConfiguration(i_CameraID_t, i_TSCConfigDataPtr_pt) )
            {
                /* Clear the current config */
                if ( tsc_cfg::clearExtPlatformConfig(i_CameraID_t) )
                {
                    /* Copy the new config */
                    tsc_cfg::currentPlatformExtConfig[i_CameraID_t].kinematicModelExternalConfig_t = i_TSCConfigDataPtr_pt->kinematicModelExternalConfig_t;
                    tsc_cfg::currentPlatformExtConfig[i_CameraID_t].cameraModelExternalConfig_t     = i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t;
                    tsc_cfg::currentPlatformExtConfig[i_CameraID_t].featureColExternalConfig_t     = i_TSCConfigDataPtr_pt->featureColExternalConfig_t;
                    // Reload Each Module and Impl Objects Configuration
                    v_Status_b = KinematicModel::getInstance_rt().UpdateExternalConfiguration(i_CameraID_t) && \
                             fc::FeatureCollection::getInstance_rt().UpdateExternalConfiguration(i_CameraID_t) && \
                             TSCAlg::getInstance_rt().UpdateExternalConfiguration(i_CameraID_t);
                }
            }
            else
            {
                // Set CONFIG-ERROR, this is done below by checking the status
            }
        } else 
        {
            // in this case, we should check if ext parameters have ever been configured successfully
            if (extConfiged[i_CameraID_t])
            {
                v_Status_b = true;
            }
        }

        // done configuration, set error code if any
        if (!v_Status_b)
        {
            stateErr[i_CameraID_t] = tscApi::e_TscErrorInvalidParameter;
            OC_DEBUG_PRINTF("stateErr[cameraID] TSCError_InvalidParameter\n");
        }
        else
        {
            // all ext configurations are updated
            extConfiged[i_CameraID_t] = true;
            // check if there is saved Data to restore from
            if (i_TSCSavedData_ps == NULL)
            {
                v_Status_b = TSCAlg::getInstance_rt().start_b(i_CameraID_t);
            }
            else if (!RestoreTSCSavedData(i_CameraID_t, i_TSCSavedData_ps))
            {
                stateErr[i_CameraID_t] = tscApi::e_TscErrorInvalidSavedData;
            }
            else
            {
                v_Status_b = TSCAlg::getInstance_rt().start_b(i_CameraID_t);
            }

        }
    } else
    {
        stateErr[i_CameraID_t] = tscApi::e_TscErrorUnexpectedRequest;
    }

    return v_Status_b;
}

// ----------------------------------------------------------------------------
tscApi::TSCState_e TSC_GetState(tscApi::enuCameraID i_CameraID_t)
{
    tscApi::TSCState_e v_State_t = tscApi::e_TscStateUnknown;
    // check parameter: cameraID should be in range
    if ( !IsValidCameraID(i_CameraID_t) )
    {
        // as TSCAlg only record error state per camera, leave error code as is
    } else if ( internalState == e_InternalStateSuspended )
    {
        v_State_t = tscApi::e_TscStatePaused;
    } else if ( internalState == e_InternalStateTerminated )
    {
        v_State_t = tscApi::e_TscStateTerminated;
    } else if ( internalState == e_InternalStateNotReady )
    {
        v_State_t = tscApi::e_TscStateUnInit;
    } else
    {
        v_State_t = TSCAlg::getInstance_rt().getTSCState_e(i_CameraID_t);
    }
    return v_State_t;
}

// ----------------------------------------------------------------------------
tscApi::TSCError_e TSC_GetError(tscApi::enuCameraID i_CameraID_t)
{
    tscApi::TSCError_e v_Err_e;
    // check parameter: cameraID should be in range
    if ( !IsValidCameraID(i_CameraID_t) )
    {
        v_Err_e = tscApi::e_TscErrorInvalidParameter;
    } else if (stateErr[i_CameraID_t] != tscApi::e_TscErrorNoError)
    {
        v_Err_e = stateErr[i_CameraID_t];
    } else
    {
        v_Err_e = TSCAlg::getInstance_rt().getTSCError_e(i_CameraID_t);
    }
    return v_Err_e;
}

tscApi::TSCCalibManeuver_e TSC_GetCalibManeuver( tscApi::enuCameraID i_CameraID_t )
{
    tscApi::TSCCalibManeuver_e v_CalibManeuver_e;

    // check parameter: cameraID should be in range
    if( !IsValidCameraID( i_CameraID_t ) )
    {
        v_CalibManeuver_e = tscApi::e_TrlrCamraCalibManeuver_FAILED;
    }
    else
    {
        v_CalibManeuver_e = TSCAlg::getInstance_rt().GetCalibManeuver_e( i_CameraID_t );
    }

    return v_CalibManeuver_e;
}





// ----------------------------------------------------------------------------
sint32_t TSC_GetValidFeaturesCount(tscApi::enuCameraID i_CameraID_t)
{
    sint32_t v_Ret_s32 = -1;
    if ( IsValidCameraID(i_CameraID_t) )
    {
        v_Ret_s32 = fc::FeatureCollection::getInstance_rt().GetValidFeaturesCount(i_CameraID_t);
    }
    return v_Ret_s32;
}

// ----------------------------------------------------------------------------
sint32_t TSC_GetIgnoredValidFeaturesCount(tscApi::enuCameraID i_CameraID_t)
{
    sint32_t v_Ret_s32 = -1;
    if ( IsValidCameraID(i_CameraID_t) )
    {
        v_Ret_s32 = fc::FeatureCollection::getInstance_rt().GetIgnoredValidFeaturesCount(i_CameraID_t);
    }
    return v_Ret_s32;
}

// ----------------------------------------------------------------------------
sint32_t TSC_GetInvalidFeaturesCount(tscApi::enuCameraID i_CameraID_t)
{
    sint32_t v_Ret_s32 = -1;
    if ( IsValidCameraID(i_CameraID_t) )
    {
        v_Ret_s32 = fc::FeatureCollection::getInstance_rt().GetInvalidFeaturesCount(i_CameraID_t);
    }
    return v_Ret_s32;
}

// ----------------------------------------------------------------------------
sint32_t TSC_GetUndetectedFeaturesCount(tscApi::enuCameraID i_CameraID_t)
{
    sint32_t v_Ret_s32 = -1;
    if ( IsValidCameraID(i_CameraID_t) )
    {
    	v_Ret_s32 = fc::FeatureCollection::getInstance_rt().GetUndetectedFeaturesCount(i_CameraID_t);
    }
    return v_Ret_s32;
}

// ----------------------------------------------------------------------------
sint32_t TSC_GetSkippedFramesCount(tscApi::enuCameraID i_CameraID_t)
{
    sint32_t v_Ret_s32 = -1;
    if ( IsValidCameraID(i_CameraID_t) )
    {
    	v_Ret_s32 = fc::FeatureCollection::getInstance_rt().GetSkippedFramesCount(i_CameraID_t);
    }
    return v_Ret_s32;
}

// ----------------------------------------------------------------------------
sint32_t TSC_GetProcessedFramesCount(tscApi::enuCameraID i_CameraID_t)
{
    sint32_t v_Ret_s32 = -1;
    if ( IsValidCameraID(i_CameraID_t) )
    {
        v_Ret_s32 = fc::FeatureCollection::getInstance_rt().GetProcessedFramesCount(i_CameraID_t);
    }
    return v_Ret_s32;
}

// ----------------------------------------------------------------------------
const tscApi::DebugOverlay_s* TSC_GetOverlays(tscApi::enuCameraID i_CameraID_t, uint8_t &o_Num_ru8)
{ 
    const tscApi::DebugOverlay_s *c_P_ps = NULL;
    if ( IsValidCameraID(i_CameraID_t) )
    {
        c_P_ps = fc::FeatureCollection::getInstance_rt().GetOverlays(i_CameraID_t, o_Num_ru8);
    }
    return c_P_ps;
}

// ----------------------------------------------------------------------------
bool_t TSC_Uninit(void)
{
    bool_t v_Ret_b = false;
    if (internalState == e_InternalStateTerminated)
    {
        KinematicModel::getInstance_rt().uninit_v();
        fc::FeatureCollection::getInstance_rt().uninit_v();
        TSCAlg::getInstance_rt().uninit_v();
        internalState = e_InternalStateNotReady;
        v_Ret_b = true;
    } else
    {
        // as TSCAlg only record error state per camera, leave error code as is
    }
    return v_Ret_b;
}

// ----------------------------------------------------------------------------
bool_t TSC_ProcessFrame(void)
{
    bool_t v_Ret_b = false;
    if (internalState == e_InternalStateRunning)
    {
        // if we are back from resume, cleanup local data before processing the frame
        if (wasPaused) {
            cleanup_v();
            wasPaused = false;
        }
        // TODO: check frame number here to avoid reprocess the same frame

        // There are function calls that need be called once for all cameras being calibrated.
        // Therefore, ProcessFrame, as of now, does not take cameraID as an argument.
        v_Ret_b = KinematicModel::getInstance_rt().processFrame_b()
              && fc::FeatureCollection::getInstance_rt().processFrame_b()
              &&  TSCAlg::getInstance_rt().processFrame_b();
    } else
    {
        // as TSCAlg only record error state per camera, leave error code as is
    }
    return v_Ret_b;
}

// ----------------------------------------------------------------------------
bool_t TSC_Calibrate(tscApi::enuCameraID i_CameraID_t)
{
    bool_t v_Ret_b = false;
    if ( IsValidCameraID(i_CameraID_t) )
    {
        if ( TSC_GetState(i_CameraID_t) == tscApi::e_TscStateFeatureCollectionCompleted )
        {
            v_Ret_b = TSCAlg::getInstance_rt().Calibrate(i_CameraID_t);
        } else 
        {
            stateErr[i_CameraID_t] = tscApi::e_TscErrorUnexpectedRequest;
        }
    } else
    {
        // as TSCAlg only record error state per camera, leave error code as is
    }
    return v_Ret_b;
}

// ----------------------------------------------------------------------------
bool_t TSC_GetFinalCalibrationResult( tscApi::enuCameraID i_CameraID_t, tscApi::CalibrationParams_s* o_FinalCalibrationResult_ps )
{
    bool_t v_Ret_b = false;
    if ( IsValidCameraID(i_CameraID_t) )
    {
        if ( o_FinalCalibrationResult_ps == NULL )
        {
            stateErr[i_CameraID_t] = tscApi::e_TscErrorInvalidParameter;
        } else
        {
            v_Ret_b = TSCAlg::getInstance_rt().GetFinalCalibrationResult( i_CameraID_t, o_FinalCalibrationResult_ps );
        }
    } else
    {
        // as TSCAlg only record error state per camera, leave error code as is
    }
    return v_Ret_b;
}
    
// ----------------------------------------------------------------------------
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
bool_t TSC_GetDetailedCalibrationResult( tscApi::enuCameraID cameraID, tsc::DetailedCalibrationResult* detailedCalibrationResult )
{
    bool_t ret = false;
    if ( IsValidCameraID(cameraID) )
    {
        if ( detailedCalibrationResult == NULL )
        {
            stateErr[cameraID] = tscApi::e_TscErrorInvalidParameter;
        } else
        {
            ret = TSCAlg::getInstance_rt().GetDetailedCalibrationResult( cameraID, detailedCalibrationResult );
        }
    } else
    {
        // as TSCAlg only record error state per camera, leave error code as is
    }
    return ret;
}

// ----------------------------------------------------------------------------
mecl::core::ArrayList< fc::ValidFeature, tsc_cfg::NUM_VALID_INDICES>& TSC_GetValidFeatures(tscApi::enuCameraID cameraID)
{
    return fc::FeatureCollection::getInstance_rt().GetValidFeatures( cameraID );
}
#endif

// ----------------------------------------------------------------------------

// Those APIs are supposed to be called from a higher-priority job control task

bool_t TSC_Pause(void)
{
    bool_t v_Ret_b = false;
    //Immediately set the flag to suspended, prevents the subsequent TSC_ProcessFrame calls. 
    if(internalState == e_InternalStateRunning)
    {
        internalState = e_InternalStateSuspended;
        v_Ret_b = true;
    } else
    {
        // as TSCAlg only record error state per camera, leave error code as is
    }
    return v_Ret_b;
}

bool_t TSC_Stop(void)
{
    bool_t v_Ret_b = false;
    //Immediately set the flag to suspended, prevents the subsequent Controller::ProcessFrame calls.
    if (internalState != e_InternalStateNotReady)
    {
        wasPaused = false;
        internalState = e_InternalStateTerminated;
        v_Ret_b = true;
    } else
    {
        // as TSCAlg only record error state per camera, leave error code as is
    }
    return v_Ret_b;
}

bool_t TSC_Resume(void)
{
    bool_t v_Ret_b = false;
    //Immediately set the flag to resume
    if(internalState == e_InternalStateSuspended)
    {
        wasPaused = true;
        internalState = e_InternalStateRunning;
        v_Ret_b = true;
    } else
    {
        // as TSCAlg only record error state per camera, leave error code as is
    }
    return v_Ret_b;
}

#ifdef DEBUG_TSC_ALG
// ----------------------------------------------------------------------------
const tscApi::DebugCounters* TSC_GetDebugCounters(tscApi::enuCameraID cameraID)
{
    const tscApi::DebugCounters *p = NULL;
    if ( IsValidCameraID(cameraID) )
    {
        p = fc::FeatureCollection::getInstance_rt().GetDebugCounters(cameraID);
    }
    return p;
}


// ----------------------------------------------------------------------------
bool_t TSC_GetFinalCalibrationResultStdDev( tscApi::enuCameraID cameraID, tscApi::CalibrationParams_s* finalCalibrationResultStdDev )
{
    bool_t ret = false;
    if ( IsValidCameraID(cameraID) )
    {
        if ( finalCalibrationResultStdDev == NULL )
        {
            stateErr[cameraID] = tscApi::e_TscErrorInvalidParameter;
        } else
        {
            ret = TSCAlg::getInstance_rt().GetFinalCalibrationResultStdDev( cameraID, finalCalibrationResultStdDev );
        }
    } else
    {
        // as TSCAlg only record error state per camera, leave error code as is
    }
    return ret;
}
    
#endif

// ----------------------------------------------------------------------------
