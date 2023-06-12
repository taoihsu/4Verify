// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [04-Jun-2013]
// --- Modified by Ehsan Parvizi [26-Aug-2014]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Modified by Dmitri Kelbas [05-Nov-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "mecl/mecl.h"
#include "mathOperations.h"
#include "configuration.h"
#include "featureCollection.h"
#include "featureFilter.h"
#include "featureCollectionImpl.h"
#include "tscAlg.h"
#include <float.h>
//#include "ss.h"
// SB #include <algorithm>
#include <stdlib.h>
// PRQA S 1051 1 // This commented code is uncommented when running on PC.
// #include <iostream>
// ----------------------------------------------------------------------------
// PRQA S 1051 1 // This commented code is uncommented when running on PC.
// using std::string;
using tsc_trace::kCameraStrings;
#define TRIMMEAN
// PRQA S 3708 EOF
// PRQA S 1070 EOF
// ----------------------------------------------------------------------------
namespace tsc
{
TSCAlgImpl::TSCAlgImpl(): 
    ModuleImpl(),
    featureCollectionBeginFrame_u32( 0 ), 
    featureCollectionEndFrame_u32( 0 ), 
    c_IGs_px( NULL ),
    featureCollector_po( NULL ),
    tscState_e (tscApi::e_TscStateUnInit),
    tscError_e (tscApi::e_TscErrorNoError),
    tscCalibManeuver_e( tscApi::e_TrlrCamraCalibManeuver_NULL ),
    calibTime_f32( 0 )
{
}

// ----------------------------------------------------------------------------
TSCAlgImpl::~TSCAlgImpl()
{
        std::string camera;

        switch( cameraID_t )
        {
            case 0:
                camera = "FRONT";
                break;

            case 1:
                camera = "LEFT";
                break;

            case 2:
                camera = "REAR";
                break;

            case 3:
                camera = "RIGHT";
                break;

            default:
                camera = "Unknown";
                break;
}

        // check if it is the camera we ran calibration on before outputting for all cameras.
        if( cameraID_t == ocTargetCamera && !dumpResults )
        {
            std::string featuresFileName = camera + "_ValidFeatureTrack.csv";
            AppCtrl::WriteToFile( featuresFileName, m_validFeaturesStream, m_validFeaturesHeader, false );
            std::string initialGuessFileName = camera + "_InitialGuesses.csv";
            AppCtrl::WriteToFile( initialGuessFileName, m_initialGuessStream, m_initialGuessHeader, false );
            std::string calibrationResultFileName = camera + "_FeatureCollectionFrame.csv";
            AppCtrl::WriteToFile( calibrationResultFileName, m_calibrationResultStream, m_calibrationHeader, true );
            std::string detailedCalibrationFileName = camera + "_DetailedCalibrationResults.csv";
            AppCtrl::WriteToFile( detailedCalibrationFileName, m_detailedCalibrationResultStream, m_detailedCalibrationHeader, true );
        }
    }

//--------------------- INIT -------------------------------------
  bool_t TSCAlgImpl::Init(tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po, tsc::TSCInfo_s* b_Info_ps, tsc::TSCConfig* b_Config_po, tsc::TSCCameraConfig_s* b_CameraConfig_ps, sint64_t i_Tracer_s64, tscApi::enuCameraID i_CameraID_t)
{
    bool_t v_RetValue_b = true;

    if (!preInit_b(b_TSCCtrlInfo_po, b_Info_ps, b_Config_po, b_CameraConfig_ps, i_Tracer_s64, i_CameraID_t))
    {
        tscState_e = tscApi::e_TscStateError;
        tscError_e = tscApi::e_TscErrorInitFail;
        tscCalibManeuver_e = tscApi::e_TrlrCamraCalibManeuver_FAILED;
        v_RetValue_b = false;
    }
    else
    {
#ifdef ENABLE_SFM
      // --- get the SfM
      if( !m_SfM.Init(i_CameraID_t) )
      {
        TRACE_0( m_hTracer, "Failed to Create the SFM object");
        tscState_e = tscApi::e_TscStateError;
        tscError_e = tscApi::e_TscErrorInitFail;
        v_RetValue_b = false;
      }
      else
#endif
      {

        // --- FeatureCollector
        featureCollector_po = fc::FeatureCollection::getInstance_rt().getImplObject_pt(cameraID_t);
        if( (featureCollector_po == NULL) || (!featureCollector_po->isInitOk_b()) )
        {
          TRACE_0( m_hTracer, "Failed to create feature collection object!" );
          tscState_e = tscApi::e_TscStateError;
          tscError_e = tscApi::e_TscErrorInitFail;
          tscCalibManeuver_e = tscApi::e_TrlrCamraCalibManeuver_FAILED;
          v_RetValue_b = false;
        }
        else
        {
          // --- everything fine
          initOK_b = true;
          tscState_e = tscApi::e_TscStateInitOk;
          tscCalibManeuver_e = tscApi::e_TrlrCamraCalibManeuver_DRIVEFWD;
        }
      }
    }
    return v_RetValue_b;
}

// ----------------------------------------------------------------------------
bool_t TSCAlgImpl::unInit_b(void)
{

    TRACE_1( m_hTracer, "[%s] [Uninit]: Uninit called", kCameraStrings[ m_CameraID ].c_str() );

    reset_v();

    TRACE_1( m_hTracer, "[%s] [Uninit]: Almost complete; initialize variables to their reset state", kCameraStrings[ m_CameraID ].c_str() );

    featureCollector_po = NULL;

    tscState_e = tscApi::e_TscStateUnInit;
    tscCalibManeuver_e = tscApi::e_TrlrCamraCalibManeuver_NULL;
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

void TSCAlgImpl::reset_v(void)
{
    c_IGs_px = NULL;

    earlyBreakOnStop_b = false;
    tscState_e = tscApi::e_TscStateInitOk;
    tscCalibManeuver_e = tscApi::e_TrlrCamraCalibManeuver_DRIVEFWD;
    featureCollectionBeginFrame_u32 = 0 ;
    featureCollectionEndFrame_u32 = 0;

    memset( rArry_af32, 0, tsc_cfg::NUM_INITIAL_GUESSES*sizeof(float32_t));
    memset( wArry_af32, 0, tsc_cfg::NUM_INITIAL_GUESSES*sizeof(float32_t));
    initialGuess_o.reset_v();
    iGConfidence_o.reset_v();

    // --- Reset the calibration timer
    availableFrameNumber_u32 = 0;

    enable_b = false;

}

void TSCAlgImpl::cleanupLocalData_v(void)
{
}

// ----------------------------------------------------------------------------
bool_t TSCAlgImpl::start_b(void)
{
    bool_t v_RetValue_b = true;
    enable_v();
    tscState_e = tscApi::e_TscStateInitOk;
    if ( (featureCollector_po == NULL) || (!featureCollector_po->isInitOk_b()) || (!featureCollector_po->start_b()) )
    {
        tscState_e = tscApi::e_TscStateError;
        tscError_e = tscApi::e_TscErrorStartFail;
        tscCalibManeuver_e = tscApi::e_TrlrCamraCalibManeuver_FAILED;
        v_RetValue_b = false;
    }
    else
    {
        tscState_e = tscApi::e_TscStateFeatureCollection;
        tscError_e = tscApi::e_TscErrorNoError;
        tscCalibManeuver_e = tscApi::e_TrlrCamraCalibManeuver_DRIVEFWD;
    }

    return v_RetValue_b;
}

// ----------------------------------------------------------------------------
bool_t TSCAlgImpl::process_b(void)
{
    bool_t v_RetValue_b = true;
    resetErrorType_v();
    if( (!initOK_b) || (!enable_b) )
    {
        // shouldn't be here
        setErrorRecoverable_v(false);
        v_RetValue_b = false;
    }
    
    else
    {
      CREATE_TIMER( timer );

      START_TIMER( timer );

      switch( tscState_e )
      {
        case tscApi::e_TscStateUnInit:
          {
            // should be rejected by TSC_ProcessFrame as unexpected request
            setErrorRecoverable_v(false);
            v_RetValue_b = false;
            break;
          }

        case tscApi::e_TscStateInitOk:
          {
            featureCollectionBeginFrame_u32 = tSCCtrlInfo_po->getMFrmNmbr_u32();
            tscState_e = tscApi::e_TscStateFeatureCollection;
            if( !collectFeatures_b() )
            {
                TRACE_3( m_hTracer, "[%s] [%s]: Failed to collect features in frame [%lu]", 
                    kCameraStrings[ m_CameraID ].c_str(), __func__, m_pTSCCtrlInfo->getMFrmNmbr_u32() );
                // when FeatureCollection in error state 
                v_RetValue_b = false;
            }
            break;
          }

        case tscApi::e_TscStateFeatureCollection:
          {
            if( !collectFeatures_b() )
            {
                TRACE_3( m_hTracer, "[%s] [%s]: Failed to collect features in frame [%lu]", 
                    kCameraStrings[ m_CameraID ].c_str(), __func__, m_pTSCCtrlInfo->getMFrmNmbr_u32() );
                // when FeatureCollection in error state 
                v_RetValue_b = false;
            }
            break;
          }
        
        case tscApi::e_TscStateFeatureCollectionCompleted:
          {
            break;
          }
        case tscApi::e_TscStateCalibration:
          {
            break;
          }
        case tscApi::e_TscStateCalibrationCompleted:
          {
            break;  // Nothing to do.
          }
        
        case tscApi::e_TscStateError:
          {
            setErrorRecoverable_v(false);
            v_RetValue_b = false; // Do nothing. Should not be called.
            break;
          }

        default:
          {
            break;
          }
      }

      if ( v_RetValue_b == true )
      {
        STOP_TIMER( timer );
        TRACE_5( m_hTracer, "[%s] [%s]: Frame [%lu] Processing finished; TSC State [%d]; Elapsed time [%.1f]ms", 
          kCameraStrings[ m_CameraID ].c_str(), __func__, m_pTSCCtrlInfo->getMFrmNmbr_u32(), tscState_e, GET_ELAPSED_TIME( timer ));

        // --- everything worked OK
      }
    }
    if ( !notInError_b() )
    {
        // stop feature collection and tscAlg
        featureCollector_po->disable_v();
        disable_v();
        featureCollectionEndFrame_u32 = tSCCtrlInfo_po->getMFrmNmbr_u32();
        tscError_e = tscApi::e_TscErrorFeatureCollectionError;
        if ( isErrorNonRecoverable_b() )
        {
            tscState_e = tscApi::e_TscStateError;
            tscCalibManeuver_e = tscApi::e_TrlrCamraCalibManeuver_FAILED;
        } else 
        {
            // recoverable
            tscState_e = tscApi::e_TscStateFeatureCollectionCompleted;
            tscCalibManeuver_e = tscApi::e_TrlrCamraCalibManeuver_COMPLETE;
        }
    }
    return v_RetValue_b;
}

// ----------------------------------------------------------------------------
bool_t TSCAlgImpl::collectFeatures_b()
{
    bool_t v_RetValue_b = true;

    if ( featureCollector_po->notInError_b() )
    {
      // PRQA S 3222 ++
      if ( ( featureCollector_po->getDiagnosticData_po()->getNumVldFrms_u32() >= config_px->getFCRestrictionMaxNumValidFrame_u32() ) &&
                //                ( featureCollector_po->getFrameCounter_u32() >= config_px->getFCRestrictionMinNumRawFrame_u32() ) )
                (( featureCollector_po->getDiagnosticData_po()->getNumProcessedFrames_u32() - featureCollector_po->getDiagnosticData_po()->getNumSkippedFrames_u32() ) >= config_px->getFCRestrictionMinNumRawFrame_u32() ) 
                || ( featureCollector_po->getDiagnosticData_po()->getNumVldFrms_u32() >= (config_px->getFCRestrictionMaxNumValidFrame_u32()*2) ))
      // PRQA S 3222 -- 
      {
        // --- Stop feature collection and indicate completion
        featureCollector_po->disable_v();
        tscState_e = tscApi::e_TscStateFeatureCollectionCompleted;
        tscCalibManeuver_e = tscApi::e_TrlrCamraCalibManeuver_COMPLETE;
        featureCollectionEndFrame_u32 = tSCCtrlInfo_po->getMFrmNmbr_u32();
      }
    } else
    {
        setErrorRecoverable_v(!featureCollector_po->isErrorNonRecoverable_b());
        v_RetValue_b = false;
    }
    return v_RetValue_b;
}

// ----------------------------------------------------------------------------
bool_t TSCAlgImpl::isMotionValid_b()
{
    return false;
}

// ----------------------------------------------------------------------------
bool_t TSCAlgImpl::GetFinalCalibrationResult( tscApi::CalibrationParams_s* o_FinalCalibrationResult_ps ) const
{
    o_FinalCalibrationResult_ps->pitchDeg_f64 = initialGuess_o.getPitch_f64();
    o_FinalCalibrationResult_ps->yawDeg_f64 = initialGuess_o.getYaw_f64();
    o_FinalCalibrationResult_ps->rollDeg_f64 = initialGuess_o.getRoll_f64();
    o_FinalCalibrationResult_ps->xMM_f64 = 0.0;
    o_FinalCalibrationResult_ps->yMM_f64 = 0.0;
    o_FinalCalibrationResult_ps->zMM_f64 = 0.0;
    return true;
}

    // ----------------------------------------------------------------------------
//#ifdef DEBUG_TSC_ALG
bool_t TSCAlgImpl::GetFinalCalibrationResultStdDev( tscApi::CalibrationParams_s* finalCalibrationResultStdDev )
{
    finalCalibrationResultStdDev->pitchDeg_f64 = iGConfidence_o.getIGStdDevs_ro().getPitch_f64();
    finalCalibrationResultStdDev->yawDeg_f64 = iGConfidence_o.getIGStdDevs_ro().getYaw_f64();
    finalCalibrationResultStdDev->rollDeg_f64 = iGConfidence_o.getIGStdDevs_ro().getRoll_f64();
    finalCalibrationResultStdDev->xMM_f64 = 0.0;
    finalCalibrationResultStdDev->yMM_f64 = 0.0;
    finalCalibrationResultStdDev->zMM_f64 = 0.0;

    return true;
}
//#endif

// ----------------------------------------------------------------------------
//#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
bool_t TSCAlgImpl::GetDetailedCalibrationResult( tsc::DetailedCalibrationResult* detailedCalibrationResult )
{
    GetFinalCalibrationResult( &(detailedCalibrationResult->finalCalibrationResult) );
    detailedCalibrationResult->featureCollectionBeginFrame = featureCollectionBeginFrame_u32;
    detailedCalibrationResult->featureCollectionEndFrame = featureCollectionEndFrame_u32;
    detailedCalibrationResult->calibrationTime = calibTime_f32;
    detailedCalibrationResult->pIGs = c_IGs_px;
    detailedCalibrationResult->pPerformances = featureCollector_po->GetROIPerformances( 0 );
    detailedCalibrationResult->finalResultConfidenceLevel = iGConfidence_o;

    return true;
}
//#endif

    // ----------------------------------------------------------------------------
    // --- Windows Logging
    bool_t TSCAlgImpl::DumpValidFeatures()
    {
        m_validFeaturesStream.clear();
        m_validFeaturesHeader.clear();
        m_validFeaturesHeader
                << "Feature Idx"
                << ",Track Idx"
                << ",Frame No"
                << ",Warped x"
                << ",Warped y"
                << ",Is Unwarped"
                << ",Unwarped x"
                << ",Unwarped y"
                /// Temp
                << ",Size"
                << ",Angle"
                << ",Response"
                << ",Octave"
                << ",Match Distance"
                /// Temp
                << ",MVdX"
                << ",MVdY"
                << ",MVdPsi"
                << ",Xw"
                << ",Yw"
                << ",Zw"
                << ",Prediction Error"
                << ",FundMat Error"
                << ",ROI Index"
                << ",Xwd"
                << ",Ywd"
                << ",Zwd"
                << "\n";
        unsigned featuresSize = featureCollector_po->getDiagnosticData_po()->getNumValidFeatures_u32();

        for( unsigned featureIdx = 0; featureIdx < featuresSize; ++featureIdx )
        {
            unsigned trackSize = featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx().size_u32();

            for( unsigned trackIdx = 0; trackIdx < trackSize; ++trackIdx )
            {
                m_validFeaturesStream
                        << featureIdx
                        << "," << trackIdx
                        << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getFn_u32()
                        << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getPt_t().x_x
                        << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getPt_t().y_x
                        << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getIsUnWarped_b()
                        << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getUnWarpedPt_x().x_x
                        << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getUnWarpedPt_x().y_x
                        << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getSize_f32()
                        << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getAngle_f32()
                        << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getResponse_f32()
                        << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getOctave_s32();

                if( trackIdx > 0 )
                {
                    m_validFeaturesStream << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getMatchDistance_f32();
                }
                else
                {
                    m_validFeaturesStream << ", ";
                }

                m_validFeaturesStream
                        << ", " << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getMV_o().getX_f64()
                        << ", " << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getMV_o().getY_f64()
                        << ", " << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx()[trackIdx].getMV_o().getPsi_f64();

                if( trackIdx == featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getTrackList_rx().size_u32() - 1 )
                {
                    // Dump the valid feature world pt along the last row of tracklist
                    m_validFeaturesStream
                            << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getWorldPt_t().x_x
                            << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getWorldPt_t().y_x
                            << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getWorldPt_t().z_x
                            //<< "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx]->predictionError
                            // Placeholder 0 for now
                            << "," << 0
                            << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getFundMatError_f64()
                            //<< "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx]->roiIdx
                            // Placeholder 0 for now
                            << "," << 0
                            << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getDesignWorldPt_t().x_x
                            << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getDesignWorldPt_t().y_x
                            << "," << featureCollector_po->getDiagnosticData_po()->getValidFeatures_u32()[featureIdx].getDesignWorldPt_t().z_x;
                }
                else
                {
                    for( size_t i = 0; i < 9; ++i )
                    {
                        m_validFeaturesStream << ",";
                    }
                }

                m_validFeaturesStream << "\n";
            }
        }

        return true;
    }

    bool_t TSCAlgImpl::DumpInitialGuesses()
    {
        m_initialGuessStream.clear();
        m_initialGuessHeader.clear();
        m_initialGuessHeader
                << "IG ID,"
                << "Pitch,"
                << "Yaw,"
                << "Roll\n";

        for( unsigned i = 0; i < c_IGs_px->size_u32(); ++i )
        {
            m_initialGuessStream
                    << i << ","
                    << c_IGs_px->getArray_o()[i].getPitch_f64() << ","
                    << c_IGs_px->getArray_o()[i].getYaw_f64() << ","
                    << c_IGs_px->getArray_o()[i].getRoll_f64() << "\n";
        }

       // camera_model::CameraModelMecl cameraModelMecl;
        return true;
    }

    bool_t TSCAlgImpl::DumpCalibrationResult()
    {
        m_calibrationResultStream.clear();
        m_calibrationHeader.clear();
        m_detailedCalibrationResultStream.clear();
        m_detailedCalibrationHeader.clear();
        m_calibrationHeader
                << ",FC Begin Frame"
                << ",FC End Frame"
                << "\n";
        m_calibrationResultStream
                << featureCollectionBeginFrame_u32 << ", "
                << featureCollectionEndFrame_u32 << "\n";
        return true;
    }
#ifdef TRIMMEAN
    static int32_t partition_i32( float32_t i_array_af32[], int32_t i_low_i32, int32_t i_high_i32 )
    {
        //int pivotIndex = 0;
        // Pick the first element to be the pivot.
        //if( high < array.size() && low < array.size() )
        int32_t o_pivotIndex_i32 = i_low_i32;
        float32_t v_pivot_f32 = i_array_af32[i_low_i32];

        do
        {
            while( i_low_i32 <= i_high_i32 && i_array_af32[i_low_i32] <= v_pivot_f32 )
            {
                i_low_i32++;
            }

            while( i_array_af32[i_high_i32] > v_pivot_f32 )
            {
                i_high_i32--;
            }

            if( i_low_i32 < i_high_i32 )
            {
                int32_t vtemp_i32 = i_array_af32[i_low_i32];
                i_array_af32[i_low_i32] = i_array_af32[i_high_i32];
                i_array_af32[i_high_i32] = vtemp_i32;
            }
        }
        while( i_low_i32 < i_high_i32 );

        int32_t v_temp_i32 = i_array_af32[o_pivotIndex_i32];
        i_array_af32[o_pivotIndex_i32] = i_array_af32[i_high_i32];
        i_array_af32[i_high_i32] = v_temp_i32;
        o_pivotIndex_i32 = i_high_i32;
        return o_pivotIndex_i32;
    }

    static void quickSort_v( float32_t i_array_af32[], int32_t i_first_i32, int32_t i_last_i32 )
    {
        if( i_last_i32 - i_first_i32 >= 1 )
        {
            int32_t v_pivotIndex_i32 = partition_i32( i_array_af32, i_first_i32, i_last_i32 );
            quickSort_v( i_array_af32, i_first_i32, v_pivotIndex_i32 - 1 );
            quickSort_v( i_array_af32, v_pivotIndex_i32 + 1, i_last_i32 );
        }
    }
    inline
    static float32_t mean_f32( float32_t i_array_af32[], int32_t i_low_i32, int32_t i_high_f32 )
    {
        int32_t v_acc_i32 = 0;

        for( int32_t i = i_low_i32; i <= i_high_f32; i++ )
        {
            v_acc_i32 += i_array_af32[i];
        }

        return v_acc_i32 /  static_cast<float32_t>( i_high_f32 - i_low_i32 + 1 );
    }

    float32_t TrimMean_f32( float32_t* i_inputArray_f32, const int32_t i_n_i32, float32_t i_percent_f32 )
    {

        if( i_n_i32 <= 0 )
        {
            // size (n) out of range.
            return 0.0f;
        }

        if( i_percent_f32 < 0 || i_percent_f32 >= 1 )
        {
            // Percent out of range.
            return 0.0f;
        }

        /* TRIMMEAN */
        // Copy inputArray into a local array which we will sort: we don't want to modify the original
        // input array.
        // Use QuickSort algorithm to sort the array.
        quickSort_v( i_inputArray_f32, 0, i_n_i32 - 1 );
        // Calculate the number of elements to exclude and round down to the nearest even number.
        int32_t v_elementsToExclude_i32 = i_n_i32 * i_percent_f32;

        if( v_elementsToExclude_i32 % 2 != 0 )
        {
            v_elementsToExclude_i32--;
        }

        // Using our sorted array, exclude the lowest and highest (elementsToExclude / 2) elements and
        // return the trimmed average.
        int32_t v_low_i32 = v_elementsToExclude_i32 / 2;
        int32_t v_high_i32 = i_n_i32 - ( v_elementsToExclude_i32 / 2 ) - 1;
        return mean_f32( i_inputArray_f32, v_low_i32, v_high_i32 );
    }
#endif
// ----------------------------------------------------------------------------
	bool_t TSCAlgImpl::collectInitialGuessAngles_b()
	{
		bool_t v_RetValue_b = false;
		CREATE_TIMER(timer);
		START_TIMER(timer);
		fc::Pointf v_PitchPairs_at[tsc::arrySz];
		fc::Pointf v_YawPairs_at[tsc::arrySz];
		fc::Pointf v_RollPairs_at[tsc::arrySz];
//		fc::Pointf v_ZPairs_at[tsc::arrySz];

		for (size_t v_Index_t = 0; v_Index_t < c_IGs_px->size_u32(); ++v_Index_t)
		{
			uint32_t v_ID_u32 = v_Index_t + 1;

			v_PitchPairs_at[v_Index_t] = fc::Pointf(static_cast<float32_t> (v_ID_u32), static_cast<float32_t> ((*c_IGs_px)[v_Index_t].getPitch_f64()));
			v_YawPairs_at[v_Index_t] = fc::Pointf(static_cast<float32_t> (v_ID_u32), static_cast<float32_t> ((*c_IGs_px)[v_Index_t].getYaw_f64()));
			v_RollPairs_at[v_Index_t] = fc::Pointf(static_cast<float32_t> (v_ID_u32), static_cast<float32_t> ((*c_IGs_px)[v_Index_t].getRoll_f64()));
//			v_ZPairs_at[v_Index_t] = fc::Pointf(static_cast<float32_t>(v_ID_u32), static_cast<float32_t>((*c_IGs_px)[v_Index_t].getZ_f64()));
		}

		// Find the best representative out of IG responses and update initial Guess
		// In case of small number of responses, take the mean of data
		// Otherwise, apply a robust fitting and take the midpoint
		iGConfidence_o.setNumIGs_v(c_IGs_px->size_u32());
		fc::InitialGuess& v_Means_ro = iGConfidence_o.getIGMeans_ro();
		fc::InitialGuess& v_StdDevs_ro = iGConfidence_o.getIGStdDevs_ro();
		if (TSCAlg::TrimMeanPercentage_f32 == 0.0f)
		{
			if (c_IGs_px->size_u32() < 10)
			{
				float64_t v_Mean_f64;
				float64_t v_StdDev_f64;
				this->meanStdDevEx(v_PitchPairs_at, c_IGs_px->size_u32(), v_Mean_f64, v_StdDev_f64);
				v_Means_ro.PutPitch(v_Mean_f64);
				v_StdDevs_ro.PutPitch(v_StdDev_f64);
				this->meanStdDevEx(v_YawPairs_at, c_IGs_px->size_u32(), v_Mean_f64, v_StdDev_f64);
				v_Means_ro.PutYaw(v_Mean_f64);
				v_StdDevs_ro.PutYaw(v_StdDev_f64);
				this->meanStdDevEx(v_RollPairs_at, c_IGs_px->size_u32(), v_Mean_f64, v_StdDev_f64);
				v_Means_ro.PutRoll(v_Mean_f64);
				v_StdDevs_ro.PutRoll(v_StdDev_f64);
//				this->meanStdDevEx(v_ZPairs_at, c_IGs_px->size_u32(), v_Mean_f64, v_StdDev_f64);
//				v_Means_ro.PutZ(v_Mean_f64);
//				v_StdDevs_ro.PutZ(v_StdDev_f64);
				initialGuess_o.PutPitch(v_Means_ro.getPitch_f64());
				initialGuess_o.PutYaw(v_Means_ro.getYaw_f64());
				initialGuess_o.PutRoll(v_Means_ro.getRoll_f64());
//				initialGuess_o.PutZ(v_Means_ro.getZ_f64());
				v_RetValue_b = true;
			}
			else
			{
				float32_t v_Line_af32[4]; // fitLine() for 2D result size
				if (TSCAlgImpl::fitLineEx(v_PitchPairs_at, c_IGs_px->size_u32(), 0.01, 0.01, v_Line_af32))
				{
					initialGuess_o.PutPitch(v_Line_af32[3]);
					if (TSCAlgImpl::fitLineEx(v_YawPairs_at, c_IGs_px->size_u32(), 0.01, 0.01, v_Line_af32))
					{
						initialGuess_o.PutYaw(v_Line_af32[3]);
						if (TSCAlgImpl::fitLineEx(v_RollPairs_at, c_IGs_px->size_u32(), 0.01, 0.01, v_Line_af32))
						{
							initialGuess_o.PutRoll(v_Line_af32[3]);
                        //                        if( TSCAlgImpl::fitLineEx( v_ZPairs_at, c_IGs_px->size_u32(), 0.01, 0.01, v_Line_af32 ) )
                        //                       {
						//		initialGuess_o.PutZ(v_Line_af32[3]);
								float64_t v_Mean_f64;
								float64_t v_StdDev_f64;
								this->meanStdDevEx(v_PitchPairs_at, c_IGs_px->size_u32(), v_Mean_f64, v_StdDev_f64);
								v_Means_ro.PutPitch(v_Mean_f64);
								v_StdDevs_ro.PutPitch(v_StdDev_f64);
								this->meanStdDevEx(v_YawPairs_at, c_IGs_px->size_u32(), v_Mean_f64, v_StdDev_f64);
								v_Means_ro.PutYaw(v_Mean_f64);
								v_StdDevs_ro.PutYaw(v_StdDev_f64);
								this->meanStdDevEx(v_RollPairs_at, c_IGs_px->size_u32(), v_Mean_f64, v_StdDev_f64);
								v_Means_ro.PutRoll(v_Mean_f64);
								v_StdDevs_ro.PutRoll(v_StdDev_f64);
//								this->meanStdDevEx(v_ZPairs_at, c_IGs_px->size_u32(), v_Mean_f64, v_StdDev_f64);
//								v_Means_ro.PutZ(v_Mean_f64);
//								v_StdDevs_ro.PutZ(v_StdDev_f64);
								v_RetValue_b = true;
							}
						}
					}
				}

		}

		else
		{
			constexpr int iGSize = 500;
			float pitchArr[tsc::arrySz];
			float yawArr[tsc::arrySz];
			float rollArr[tsc::arrySz];
//			float zArr[tsc::arrySz];

			for (size_t i = 0; i < c_IGs_px->size_u32(); i++)
			{
				pitchArr[i] = (v_PitchPairs_at[i].y_x);
				yawArr[i] = (v_YawPairs_at[i].y_x);
				rollArr[i] = (v_RollPairs_at[i].y_x);
//				zArr[i] = (v_ZPairs_at[i].y_x);
			}

			float PitchAvg = TrimMean_f32(pitchArr, c_IGs_px->size_u32(), TSCAlg::TrimMeanPercentage_f32);
			float YawAvg = TrimMean_f32(yawArr, c_IGs_px->size_u32(), TSCAlg::TrimMeanPercentage_f32);
			float RollAvg = TrimMean_f32(rollArr, c_IGs_px->size_u32(), TSCAlg::TrimMeanPercentage_f32);
//			float ZAvg = TrimMean_f32(zArr, c_IGs_px->size_u32(), TSCAlg::TrimMeanPercentage_f32);
			initialGuess_o.PutPitch(PitchAvg);
			initialGuess_o.PutYaw(YawAvg);
			initialGuess_o.PutRoll(RollAvg);
//			initialGuess_o.PutZ(ZAvg);
			v_RetValue_b = true;
		}
	


    STOP_TIMER( timer );
    TRACE_5( m_hTracer, "[%s] [%s]: %s [%u] Initial Guesses. Elapsed time [%.1f]ms", 
             kCameraStrings[ m_CameraID ].c_str(), __func__, earlyBreakOnStop_b ? "Stopped collecting" : "Successfully collected",
             c_IGs_px->size_u32(), GET_ELAPSED_TIME( timer ));

    if ( v_RetValue_b )
    {    
        TRACE_5( m_hTracer, "[%s] [%s]: Final IG Pitch [%.2f], Yaw [%.2f], Roll [%.2f]", 
            kCameraStrings[ m_CameraID ].c_str(), __func__, initialGuess_o.getPitch_f64(), initialGuess_o.getYaw_f64(), initialGuess_o.getRoll_f64() );
        TRACE_5( m_hTracer, "[%s] [%s]: mean(IGs) Pitch [%.2f], Yaw [%.2f], Roll [%.2f]", 
            kCameraStrings[ m_CameraID ].c_str(), __func__, v_Means_ro.getPitch_f64(), v_Means_ro.getYaw_f64(), v_Means_ro.getRoll_f64() );
        TRACE_5( m_hTracer, "[%s] [%s]: stdDev(IGs) Pitch [%.2f], Yaw [%.2f], Roll [%.2f]",
            kCameraStrings[ m_CameraID ].c_str(), __func__, v_StdDevs_ro.getPitch_f64(), v_StdDevs_ro.getYaw_f64(), v_StdDevs_ro.getYaw_f64() );
    } 

    return v_RetValue_b;
}

// ----------------------------------------------------------------------------
bool_t TSCAlgImpl::calibrate_b()
{
    tscState_e = tscApi::e_TscStateCalibration;
    resetErrorType_v();
    CREATE_TIMER( calibTimer );
    START_TIMER( calibTimer );
    TRACE_2( m_hTracer, "[%s] [%s]: started ...", kCameraStrings[ m_CameraID ].c_str(), __func__ );
    TRACE_2( m_hTracer, "[%s] [%s]: Collecting initial guess angles...", kCameraStrings[ m_CameraID ].c_str(), __func__ );
    
    c_IGs_px = &(featureCollector_po->GetInitialGuesses());
        DumpInitialGuesses();

    if ( c_IGs_px == NULL )
    {
      TRACE_2( m_hTracer, "[%s] [%s]: Initial guess could not be collected for this run, null pointer.", 
            kCameraStrings[ m_CameraID ].c_str(), __func__);
      setErrorRecoverable_v(false);
            // Windows Logging
            //DumpValidFeatures();
    } else if ( c_IGs_px->size_u32() < 1 )
    {
      TRACE_2( m_hTracer, "[%s] [%s]: Initial guess could not be collected for this run, empty IG list.", 
            kCameraStrings[ m_CameraID ].c_str(), __func__);
      setErrorRecoverable_v(true);
    }
    else
    {
      if (!collectInitialGuessAngles_b())
      { // force stopped
                // force stopped
                // Windows Logging
                //DumpValidFeatures();
      }
      else
      {
#ifdef ENABLE_SFM
        fc::ValidFeatureCollection *m_pValidFeatureCollection = featureCollector_po->GetValidFeatureCollection();
        if( m_pValidFeatureCollection == NULL || m_pValidFeatureCollection->GetVldFtrs().size_u32() < 1 )
        {
          setErrorRecoverable_v(false);
        }
        else
        {
          TRACE_2( m_hTracer, "[%s] [%s]: Calling SfM...", kCameraStrings[ m_CameraID ].c_str(), __func__ );

          CREATE_TIMER( sfmTimer );
          START_TIMER( sfmTimer );

          if( !m_SfM.process_b( *m_pValidFeatureCollection, m_pTSCCtrlInfo->getMFrmNmbr_u32(), initialGuess_o ) )
          {
            STOP_TIMER( sfmTimer );
            setErrorRecoverable_v(true);
          }
          else
          {
            STOP_TIMER( sfmTimer );
#if defined (DEBUG) && defined (TRACING)
            std::cout << "SofM took [" << GET_ELAPSED_TIME( sfmTimer ) << "] msec. \n";
#endif 
          }
        }
#endif
      }
    }

    if ( notInError_b() )
    {
        tscState_e = tscApi::e_TscStateCalibrationCompleted;
        if ( earlyBreakOnStop_b )
        {
            TRACE_2( m_hTracer, "[%s] [%s]: Terminated By higher priority thread, Failed to compute initial guess.",
                  kCameraStrings[ m_CameraID ].c_str(), __func__);
        } 
        TRACE_2( m_hTracer, "[%s] [%s]: Done Calibration", kCameraStrings[ m_CameraID ].c_str(), __func__);
    } else
    {
        tscError_e = tscApi::e_TscErrorCalibrationError;
        tscCalibManeuver_e = tscApi::e_TrlrCamraCalibManeuver_FAILED;
        if ( isErrorNonRecoverable_b() )
        {
            tscState_e = tscApi::e_TscStateError;
        } else 
        {    
            tscState_e = tscApi::e_TscStateCalibrationCompleted;
        }
        TRACE_3( m_hTracer, "[%s] [%s]: Terminated with ErrorCode [%s]; Failed to compute initial guess.",
                kCameraStrings[ m_CameraID ].c_str(), __func__, tsc_trace::kErrorCodes[tscError_e].c_str());
    }

    STOP_TIMER( calibTimer );
#if defined (DEBUG) && defined (TRACING)
    calibTime_f32 = GET_ELAPSED_TIME( calibTimer );
#endif
    DumpCalibrationResult();
    return notInError_b();
}

//------------- private functions -----------------------------------------
float64_t TSCAlgImpl::CalcDist2DEx( const fc::Pointf (&i_Points_rat)[tsc::arrySz], const sint16_t i_Count_s16, const float32_t (&i_Line_raf32)[4], float32_t (&i_Dist_af32)[tsc_cfg::NUM_INITIAL_GUESSES] )
{
    sint16_t v_Index_s16;
    float32_t v_PX_f32 = i_Line_raf32[2];
    float32_t v_PY_f32 = i_Line_raf32[3];
    float32_t v_NX_f32 = i_Line_raf32[1];
    float32_t v_NY_f32 = -i_Line_raf32[0];
    float64_t v_SumDist_f64 = 0.;

    for( v_Index_s16 = 0; v_Index_s16 < i_Count_s16; v_Index_s16++ )
    {
        float32_t v_X_f32;
        float32_t v_Y_f32;

        v_X_f32 = i_Points_rat[v_Index_s16].x_x - v_PX_f32;
        v_Y_f32 = i_Points_rat[v_Index_s16].y_x - v_PY_f32;

        i_Dist_af32[v_Index_s16] = mecl::math::abs_x<float32_t>( v_NX_f32 * v_X_f32 + v_NY_f32 * v_Y_f32 );
        v_SumDist_f64 += i_Dist_af32[v_Index_s16];
    }

    return v_SumDist_f64;
}

//-------------
bool_t TSCAlgImpl::FitLine2D_wodsEx( fc::Pointf (&i_Points_rat)[tsc::arrySz], sint16_t i_Count_s16, const float32_t (&i_Weights_raf32)[tsc_cfg::NUM_INITIAL_GUESSES], float32_t (&i_Line_af32)[4] )
{
    float64_t v_X_f64 = 0.0;
    float64_t v_Y_f64 = 0.0;
    float64_t v_X2_f64 = 0.0;
    float64_t v_Y2_f64 = 0.0;
    float64_t v_XY_f64 = 0.0;
    float64_t v_W_f64 = 0.0;
    float64_t v_DX2_f64;
    float64_t v_DY2_f64;
    float64_t v_DXY_f64;
    sint16_t v_Index_s16;
    sint16_t v_Count_s16 = i_Count_s16;
    float32_t v_T_f32;

    /* Calculating the average of x and y... */

    if( i_Weights_raf32 == NULL ) /* weights are zeros */
    {
        for( v_Index_s16 = 0; v_Index_s16 < v_Count_s16; v_Index_s16 += 1 )
        {
            v_X_f64 += i_Points_rat[v_Index_s16].x_x;
            v_Y_f64 += i_Points_rat[v_Index_s16].y_x;
            v_X2_f64 += i_Points_rat[v_Index_s16].x_x * i_Points_rat[v_Index_s16].x_x;
            v_Y2_f64 += i_Points_rat[v_Index_s16].y_x * i_Points_rat[v_Index_s16].y_x;
            v_XY_f64 += i_Points_rat[v_Index_s16].x_x * i_Points_rat[v_Index_s16].y_x;
        }
        v_W_f64 = static_cast<float32_t>( v_Count_s16 );
    }
    else /* weights are not zeros */
    {
        for( v_Index_s16 = 0; v_Index_s16 < v_Count_s16; v_Index_s16 += 1 )
        {
            if(mecl::math::abs_x<float32_t>(i_Weights_raf32[v_Index_s16])>=mecl::math::numeric_limits<float32_t>::epsilon_x()) {  // PRQA S 5053 // DK: not zero
                v_X_f64 += i_Weights_raf32[v_Index_s16] * i_Points_rat[v_Index_s16].x_x;
                v_Y_f64 += i_Weights_raf32[v_Index_s16] * i_Points_rat[v_Index_s16].y_x;
                v_X2_f64 += i_Weights_raf32[v_Index_s16] * i_Points_rat[v_Index_s16].x_x * i_Points_rat[v_Index_s16].x_x;
                v_Y2_f64 += i_Weights_raf32[v_Index_s16] * i_Points_rat[v_Index_s16].y_x * i_Points_rat[v_Index_s16].y_x;
                v_XY_f64 += i_Weights_raf32[v_Index_s16] * i_Points_rat[v_Index_s16].x_x * i_Points_rat[v_Index_s16].y_x;
                v_W_f64 += i_Weights_raf32[v_Index_s16];
            }
        }
    }

    v_X_f64 /= v_W_f64;
    v_Y_f64 /= v_W_f64;
    v_X2_f64 /= v_W_f64;
    v_Y2_f64 /= v_W_f64;
    v_XY_f64 /= v_W_f64;

    v_DX2_f64 = v_X2_f64 - v_X_f64 * v_X_f64;
    v_DY2_f64 = v_Y2_f64 - v_Y_f64 * v_Y_f64;
    v_DXY_f64 = v_XY_f64 - v_X_f64 * v_Y_f64;

    v_T_f32 = static_cast<float32_t>( atan2( 2.0 * v_DXY_f64, v_DX2_f64 - v_DY2_f64 ) ) / 2.0;
    i_Line_af32[0] = static_cast<float32_t>( cos( v_T_f32 ) );
    i_Line_af32[1] = static_cast<float32_t>( sin( v_T_f32 ) );

    i_Line_af32[2] = static_cast<float32_t>( v_X_f64 );
    i_Line_af32[3] = static_cast<float32_t>( v_Y_f64 );

    return true;
}

//-------------
void TSCAlgImpl::WeightWelschEx( const float32_t (&i_D_raf32)[tsc_cfg::NUM_INITIAL_GUESSES], sint16_t i_Count_s16, float32_t (&i_W_rf32)[tsc_cfg::NUM_INITIAL_GUESSES] )
{
    sint16_t v_Index_s16;
    const float32_t c_C_f32 = static_cast<float32_t>(1.0) / static_cast<float32_t>(2.9846);

    for( v_Index_s16 = 0; v_Index_s16 < i_Count_s16; v_Index_s16++ )
    {
        i_W_rf32[v_Index_s16] = static_cast<float32_t>( exp( -i_D_raf32[v_Index_s16] * i_D_raf32[v_Index_s16] * c_C_f32 * c_C_f32 ) );
    }
}

//-------------
/* Takes an array of 2D points, type of distance (including user-defined
distance specified by callbacks, fills the array of four floats with line
parameters A, B, C, D, where (A, B) is the normalized direction vector,
(C, D) is the point that belongs to the line. */
bool_t TSCAlgImpl::fitLineEx( fc::Pointf (&i_Points_rt)[tsc::arrySz], sint32_t i_Count_s32, float32_t i_Reps_f32, float32_t i_Aeps_f32, float32_t (&o_Line_rf32)[4] )
{
    float64_t v_EPS_f64 = static_cast<float64_t>(i_Count_s32)*mecl::math::numeric_limits<float32_t>::epsilon_x();  // PRQA S 5053 // <float.h>
    sint32_t v_InnerIndex_s32;
    sint32_t v_J_s32;
    sint32_t v_Index_s32;
    float32_t v_Line_af32[4];
    float32_t v_LinePrev_af32[4];

    float32_t v_RDelta_f32 = (i_Reps_f32 > mecl::math::numeric_limits<float32_t>::epsilon_x()) ? (i_Reps_f32) : static_cast<float32_t>(1.0); // PRQA S 5053
    float32_t v_Adelta_f32 = (i_Aeps_f32 > mecl::math::numeric_limits<float32_t>::epsilon_x()) ? (i_Aeps_f32) : static_cast<float32_t>(0.01); // PRQA S 5053
    float64_t v_MinErr_f64 = DBL_MAX;
    float64_t v_Err_f64 = 0.0;
    
    memset( o_Line_rf32, 0, 4*sizeof(float32_t) );
    memset( rArry_af32, 0, tsc_cfg::NUM_INITIAL_GUESSES*sizeof(float32_t));
    memset( wArry_af32, 0, tsc_cfg::NUM_INITIAL_GUESSES*sizeof(float32_t));

    for( v_Index_s32 = 0; v_Index_s32 < 20; v_Index_s32++ )
    {
        bool_t v_First_b = 1;
        for( v_InnerIndex_s32 = 0; v_InnerIndex_s32 < i_Count_s32; v_InnerIndex_s32++ )
        {
            wArry_af32[v_InnerIndex_s32] = static_cast<float32_t>(0.0); /* weights */
        }

        sint32_t v_LpEnd_s32 = mecl::math::min_x<sint32_t>(i_Count_s32,static_cast<sint32_t>(10));
        v_InnerIndex_s32 = 0;
        while (v_InnerIndex_s32 < v_LpEnd_s32)
        {
            // DK: The function returns a uniformly-distributed random 32-bit unsigned integer and updates the RNG state. 
            // It is similar to the rand() function from the C runtime library, 
            // except that OpenCV functions always generates a 32-bit random number, regardless of the platform.

            /*            
            In    Numerical Recipes in C: The Art of Scientific Computing (William H.
            Press, Brian P. Flannery, Saul A. Teukolsky, William T. Vetterling; New
            York:  Cambridge University Press, 1992 (2nd ed., p. 277)), the follow-
            ing comments are made:
               "If you want to generate a random integer between 1 and 10,    you
               should always do it by using high-order bits, as in
            
                  for example j = 1 + (int) (10.0 * (rand() / (RAND_MAX + 1.0)))
            
               and never by anything resembling
            
                  for example j = 1 + (rand() % 10)
            
               (which uses lower-order bits)." */
            
            //DK: => 'OpenCV is not as random as can be?' j = cvRandInt(&rng) % count; 
            v_J_s32 = static_cast<sint32_t>(static_cast<float32_t>(i_Count_s32) * (static_cast<float32_t>(rand()) / (static_cast<float32_t>(RAND_MAX) + 1.0)));
            
            if( wArry_af32[v_J_s32] < mecl::math::numeric_limits<float32_t>::epsilon_x() ) // PRQA S 5053
            {
                wArry_af32[v_J_s32] = static_cast<float32_t>(1.0); /* weights */
                v_InnerIndex_s32++;
            }
        }

        FitLine2D_wodsEx( i_Points_rt, i_Count_s32, wArry_af32, v_Line_af32 );
        bool_t v_LpBrk_b = false;
        for( v_InnerIndex_s32 = 0; v_InnerIndex_s32 < 30; v_InnerIndex_s32++ )
        {
            float64_t v_SumW_f64 = 0;
            if ( v_LpBrk_b == true )
            {
              break;
            }

            if( v_First_b )
            {
                v_First_b = 0;
            }
            else
            {
                float64_t v_T_f64 = v_Line_af32[0] * v_LinePrev_af32[0] + v_Line_af32[1] * v_LinePrev_af32[1];
                v_T_f64 =  mecl::math::max_x<float64_t>(v_T_f64,-1.0F);
                v_T_f64 = mecl::math::min_x<float64_t>(v_T_f64,1.0F);
                if( mecl::math::abs_x<float64_t>(acos(v_T_f64)) < static_cast<float64_t>(v_Adelta_f32) )
                {
                    float32_t v_X_f32;
                    float32_t v_Y_f32;
                    float32_t v_D_f32;

                    v_X_f32 = mecl::math::abs_x<float32_t>( v_Line_af32[2] - v_LinePrev_af32[2] );
                    v_Y_f32 = mecl::math::abs_x<float32_t>( v_Line_af32[3] - v_LinePrev_af32[3] );

                    v_D_f32 = (v_X_f32 > v_Y_f32) ? v_X_f32 : v_Y_f32;
                    if( v_D_f32 < v_RDelta_f32 )
                    {
                        v_LpBrk_b = true;
                        continue;
                    }
                }
            }
            /* calculate distances */
            v_Err_f64 = CalcDist2DEx( i_Points_rt, i_Count_s32, v_Line_af32, rArry_af32 );
            if( v_Err_f64 < v_EPS_f64 )
            {
                v_LpBrk_b = true;
                continue;
            }

            /* calculate weights */
            WeightWelschEx( rArry_af32, i_Count_s32, wArry_af32 );

            for( v_J_s32 = 0; v_J_s32 < i_Count_s32; v_J_s32++ )
            {
                v_SumW_f64 += wArry_af32[v_J_s32]; /* weights sum */
            }

            if( mecl::math::abs_x<float64_t>(v_SumW_f64) > mecl::math::numeric_limits<float64_t>::epsilon_x() ) // PRQA S 5053
            {
                v_SumW_f64 = 1./v_SumW_f64; 
                for( v_J_s32 = 0; v_J_s32 < i_Count_s32; v_J_s32++ )
                {
                    wArry_af32[v_J_s32] = static_cast<float32_t>(wArry_af32[v_J_s32]*v_SumW_f64); /* divide by sum of weights  */
                }
            }
            else
            {
                for( v_J_s32 = 0; v_J_s32 < i_Count_s32; v_J_s32++ )
                {
                    wArry_af32[v_J_s32] = static_cast<float32_t>(1.0); /* weights */
                }
            }

            /* save the line parameters */
            memcpy( v_LinePrev_af32, v_Line_af32, 4 * sizeof( float32_t ));

            /* Run again... */
            FitLine2D_wodsEx( i_Points_rt, i_Count_s32, wArry_af32, v_Line_af32 );
        }

        // reset lpBrk
        v_LpBrk_b = false;
        if (TSC_GetState(cameraID_t) == tscApi::e_TscStateTerminated)
        {
            earlyBreakOnStop_b = true;
            v_LpBrk_b = true;
        }

        if( v_Err_f64 < v_MinErr_f64 )
        {
            v_MinErr_f64 = v_Err_f64;
            memcpy( o_Line_rf32, v_Line_af32, 4 * sizeof(o_Line_rf32[0]));
            if( v_Err_f64 < v_EPS_f64 )
            {
                v_LpBrk_b = true;
            }
        }
        if ( v_LpBrk_b == true )
        {
            break;
        }
    }

    return (!earlyBreakOnStop_b);
}

// ----------------------------------------------------------------------------
float64_t TSCAlgImpl::meanEx( fc::Pointf (&i_Array_rt)[tsc::arrySz], size_t i_Total_t )
{
    float64_t v_Sum_f64 = 0.0;
    for( size_t v_Index_t = 0; v_Index_t < static_cast<size_t>(i_Total_t); v_Index_t++ )
    {
        v_Sum_f64 += i_Array_rt[v_Index_t].y_x;
    }
    return v_Sum_f64*((i_Total_t != 0) ? (1./static_cast<float64_t>(i_Total_t)) : 0.0);
}

// ----------------------------------------------------------------------------
void TSCAlgImpl::meanStdDevEx( fc::Pointf (&i_Array_rt)[tsc::arrySz], size_t i_Total_t, float64_t &o_Mean_rf64, float64_t &o_StdDev_rf64 )
{
    float64_t v_Sum_f64 = 0.0;
    float64_t v_Sumsq_f64 = 0.0;
    float64_t v_Scale_f64 = (i_Total_t != 0) ? (1./static_cast<float64_t>(i_Total_t)) : 0.0;
    for( size_t v_Index_t = 0; v_Index_t < static_cast<size_t>(i_Total_t); v_Index_t++ )
    {
        v_Sum_f64 += i_Array_rt[v_Index_t].y_x;
        v_Sumsq_f64 += i_Array_rt[v_Index_t].y_x * i_Array_rt[v_Index_t].y_x;
    }
    o_Mean_rf64 =  v_Sum_f64 * v_Scale_f64;
    o_StdDev_rf64 = mecl::math::algebra<float64_t>::sqrt_x( mecl::math::max_x<float64_t>((v_Sumsq_f64 - o_Mean_rf64 * o_Mean_rf64 * static_cast<float64_t>(i_Total_t)) / static_cast<float64_t>(i_Total_t), 0.0) );
}

    void TSCAlgImpl::setTargetCamera( tscApi::enuCameraID targetCamID )
    {
        ocTargetCamera = targetCamID;
        dumpResults = false;
    }
}
//-------------------------------------------------------------------------
