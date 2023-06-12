// ----------------------------------------------------------------------------
// --- Written by Ehsan Parvizi [17-Jul-2014]
// --- Modified by Ehsan Parvizi [22-Aug-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Modified by Dmitri Kelbas [5-Nov-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "kinematicModelCameraImpl.h"
#include "featureCollection.h"
#include "localFeatureCollector.h"
#include "dbgGPIO.h"
#include "armNEON.h"
// ----------------------------------------------------------------------------
// SB #include <algorithm>
// SB #include <sstream>  // stringstream
#include <stdlib.h>
#include <float.h> // FLT_MAX
// ----------------------------------------------------------------------------
// PRQA S 3708 EOF
using lfc::LocalFeatureCollector;
using fc::Point;
using fc::ImageFeature;
using fc::FeatureTrack;
// PRQA S 1051 1 // This commented code is uncommented when running on PC.
// using std::string;
using tsc_trace::kCameraStrings;
using tsc_trace::kROIIds;
// ----------------------------------------------------------------------------
LocalFeatureCollector::LocalFeatureCollector():
    pROI_po( NULL ),
    initOK_b( false ),
    c_Img_pu8( NULL ),
    hTracer_s64( 0 ),
    currFrameNum_u32( 0 ),
    pCameraModel_po( NULL ),
    shouldPickROIs_b( true ),
    nextDetectionFrame_u32( 0 ),
    currTrackFrameSkip_u32( 0 ),
    m_trackNum( 0 )
{
}

// ----------------------------------------------------------------------------
//PRQA S 6200 1 //const not added to input pointer variables, otherwise assignment produces error
bool_t LocalFeatureCollector::Init( tscApi::enuCameraID i_CameraID_t, fc::ROI* i_ROI_po, uint16_t i_VideoWidth_u16, uint16_t i_VideoHeight_u16, const uint8_t* i_Img_pu8, km::KinematicModelCameraImpl* i_KinematicModelCameraImpl_po, fc::DiagnosticData* i_DiagnosticData_po)
{
    bool_t v_Ret_b = true;
    tscCameraTargetID = i_DiagnosticData_po->tscCameraTarget;
    pROI_po = i_ROI_po;
    cameraID_t = i_CameraID_t;
    c_Img_pu8 = i_Img_pu8;
    hTracer_s64 = fc::FeatureCollection::getInstance_rt().getTracer_u64();
    size_t v_Ind_t = static_cast<size_t>(i_CameraID_t);
    pCameraModel_po = fc::FeatureCollection::getInstance_rt().getModuleInfo_rt().getMAddrCmrMdls_po();
    pCameraModel_po += v_Ind_t;
    localTrackList_o.PutRoi(*pROI_po);
    pKinematicModelCameraImpl_po = i_KinematicModelCameraImpl_po;
    pDiagnosticData_po = i_DiagnosticData_po;
    // videoWidth and videoHeight from tscCtrlInfo are used for ROI configuration validation
    frameSize_t.width_s32 = i_VideoWidth_u16;
    frameSize_t.height_s32 = i_VideoHeight_u16;
#ifdef stringstream
// SB
    std::stringstream lfcrect;
    // PRQA S 3840 ++
    lfcrect << tsc_cfg::MODULE_NAME_LOCALFEATURECOLLECTOR << "[" << pROI_po->GetRect().x_x << "," << pROI_po->GetRect().y_x  \
        << "," << pROI_po->GetRect().x_x + pROI_po->GetRect().width_s32 << "," << pROI_po->GetRect().y_x + pROI_po-> GetRect().height_s32 << "]";
    // PRQA S 3840 --
    m_LFC_ID = lfcrect.str();
    TRACE_1( hTracer_s64, "[%s] LFC Object initialized", m_LFC_ID.c_str() );
#endif
    if( !loadConfiguration_b() )
    {
        initOK_b = false;
        TRACE_1( hTracer_s64, "[%s] Init: Failed to load configuration", m_LFC_ID.c_str() );
        v_Ret_b = false;
    }
    else
    // --- collect the needed variables for convenient usage
    if( c_Img_pu8 == NULL )
    {
        initOK_b = false;
        TRACE_1( hTracer_s64, "[%s] Init: Failed to load the image buffer", m_LFC_ID.c_str() );
        v_Ret_b = false;
    }
    else
    {
        initOK_b = true;
    }

    // Windows Logging
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

    if( tscCameraTargetID == cameraID_t )
    {
        m_logFileName = camera + "_" + std::to_string( cameraID_t ) + "_LocalFeatureCollector.csv";
        m_logHeader.clear();
        m_logHeader
                << "Frame, "
                << "Last_X, "
                << "Last_Y, "
                << "Pred_X, "
                << "Pred_Y, "
                << "Next_X, "
                << "Next_Y, "
                << "Delta_X, "
                << "Delta_Y, "
                << "Match_Dist"
                << "\n";
        m_trackFileName = camera + "_LocalTracks.csv";
        m_tracksStream.clear();
        m_tracksHeader.clear();
        m_tracksHeader
                << "FeatureId"
                << ",TrackId"
                << ",FrameNo"
                << ",xw"
                << ",yw"
                << ",Unwarped?"
                << ",xuw"
                << ",yuw"
                /// Temp
                << ",Size"
                << ",Angle"
                << ",Response"
                << ",Octave"
                << ",MatchScore"
                /// Temp
                << ",MVdX"
                << ",MVdY"
                << ",MVdPsi"
                //
                << ",xPrdct"
                << ",yPrdct"
                << ",errPrdct"
                //
                << ",roiId"
                << ",XwDsgn"
                << ",YwDsgn"
                << ",ZwDsgn"
                << "\n";
    }

    return v_Ret_b;
}

//-------------------------------------------------------------------------

LocalFeatureCollector::~LocalFeatureCollector()
{
    // write the collected log stream to file
    if( tscCameraTargetID == cameraID_t )
    {
        AppCtrl::WriteToFile( m_logFileName, m_logStream, m_logHeader, false );
        AppCtrl::WriteToFile( m_trackFileName, m_tracksStream, m_tracksHeader, false );
        resetAllLocalFeatureTracks_b();   // PRQA S 4631
        TRACE_1( hTracer_s64, "[%s] LFC Object destroyed", m_LFC_ID.c_str() ); // PRQA S 4631
    }
}

//-------------------------------------------------------------------------

bool_t LocalFeatureCollector::resetAllLocalFeatureTracks_b()
{
    // --- clear transient data and start fresh next time
    // --- loop through our collection and erase it all
    TRACE_1( hTracer_s64, "[%s] - Resetting all local feature tracks...", m_LFC_ID.c_str() );

    mecl::core::ArrayList < FeatureTrack,tsc_cfg::NUM_TRACKS > ::iterator v_Itr_t = localTrackList_o.getTrcks_rx().rwBegin_o();
    mecl::core::ArrayList < FeatureTrack,tsc_cfg::NUM_TRACKS > ::const_iterator v_Endlp_t = localTrackList_o.getTrcks_rx().end_o();
    for( ; v_Itr_t != v_Endlp_t; ++v_Itr_t )
    {
        FeatureTrack& v_Ft_ro =  * v_Itr_t;

        // release track's kinematic model
        pKinematicModelCameraImpl_po->FreeImplObject( v_Ft_ro.getPKmaticMdl_po() );
        // mark the handle invalid
        v_Ft_ro.PutPKmaticMdl(NULL);
        // --- deleting individual tracks will take care of the embedded trackList object
        // --- which itself is a vector of a simple structure - no special processing needed
    }

    TRACE_2( hTracer_s64, "[%s] - Total [%lu] local tracks erased", m_LFC_ID.c_str(), localTrackList_o.getTrcks_rx().size_u32() );
    localTrackList_o.getTrcks_rx().clear_v();
    nextDetectionFrame_u32 = 0;
    return true;
}

//-------------------------------------------------------------------------

// ----------------------------------------------------------------------------

bool_t LocalFeatureCollector::Process( uint32_t i_CurrFrameNum_u32, float32_t i_Speed_f32, bool_t i_CameraFrontOpen_b, const uint8_t *i_Img_pu8 )
{
    bool_t v_Ret_b = false;
    std::ofstream file;
    //file.open( "test.txt", std::ios_base::app );
    // file << i_CurrFrameNum_u32 << "\n";

    if ( initOK_b )
    {
        // --- store current frame number
        currFrameNum_u32 = i_CurrFrameNum_u32;
        c_Img_pu8 = i_Img_pu8;

        // --- process this LFC
        // --- actual implementation for LFC
        CREATE_TIMER( totalTimer );
        START_TIMER( totalTimer );
        TRACE_3( hTracer_s64, "[%s] [%s]: Initiate processing Frame [%lu]", kCameraStrings[ cameraID_t ].c_str(), m_LFC_ID.c_str(), currFrameNum_u32 );
        SET_GPIO_ON( GPIO_BIT_8 );
        // first remove terminated (and collected) tracks
        CREATE_TIMER( t );
        START_TIMER( t );

        tempTracks_x.clear_v();
        mecl::core::ArrayList < FeatureTrack, tsc_cfg::NUM_TRACKS > ::iterator v_It_t = localTrackList_o.getTrcks_rx().rwBegin_o();
        mecl::core::ArrayList < FeatureTrack, tsc_cfg::NUM_TRACKS > ::const_iterator v_Endlp_t = localTrackList_o.getTrcks_rx().end_o();
        for( ; v_It_t != v_Endlp_t; ++v_It_t )
        {
            FeatureTrack& v_Ft_ro =  * v_It_t;
            if ( !v_Ft_ro.getIsTerm_b() )
            {
                tempTracks_x.pushBack_v(v_Ft_ro);
            }
        }
        localTrackList_o.getTrcks_rx().clear_v();
        mecl::core::ArrayList < FeatureTrack, tsc_cfg::NUM_TRACKS > ::iterator v_It2_t = tempTracks_x.rwBegin_o();
        for( ; v_It2_t != tempTracks_x.end_o(); ++v_It2_t )
        {
            FeatureTrack& v_Ft_ro =  * v_It2_t;
            localTrackList_o.getTrcks_rx().pushBack_v(v_Ft_ro);
        }

        STOP_TIMER( t );

        TRACE_2( hTracer_s64, "[%s] Removal of invalid tracks completed. Elapsed time [%.1f]ms ", m_LFC_ID.c_str(), GET_ELAPSED_TIME( t ) );

        // --- if driving conditions aren't met, skip this frame, terminate all current
        // --- tracks and start fresh with detection next frame
        if( !MeetsDrivingConstraints(i_CameraFrontOpen_b) )
        {
            TRACE_4( hTracer_s64, "[%s] [%s] [%s]: Driving constraints not met for Frame [%lu]. Skipping this Frame",  \
                kCameraStrings[ cameraID_t ].c_str(), m_LFC_ID.c_str(), __func__, currFrameNum_u32 );

            // --- reset all existing LFTs -  
            resetAllLocalFeatureTracks_b();
            pDiagnosticData_po->incSkippedFrames_v();

            STOP_TIMER( totalTimer );
            TRACE_5( hTracer_s64, "[%s] [%s] [%s]: Processed Frame [%lu] in Total [%.1f]ms",  \
            kCameraStrings[ cameraID_t ].c_str(), m_LFC_ID.c_str(), __func__, currFrameNum_u32, GET_ELAPSED_TIME( totalTimer ) );
            m_logStream << "\n";
        }
        else
        {
            // --- Adjust the parameters based on current frame metadata
            {
                // EP TODO: adjust frame skip of detection
                // --- adapt current tracking frame skip rate
                // --- Assumes speed ranges are sorted, otherwise a linked sort is required
                uint32_t v_MaxIdx_u32 = params_o.getspdRngs_rx().size_u32();
                for ( uint32_t v_Idx_u32 = 0; v_Idx_u32 < v_MaxIdx_u32; ++v_Idx_u32 )
                {
                    if ( i_Speed_f32 < static_cast<float32_t>(params_o.getspdRngs_rx()[ v_Idx_u32 ]) )
                    {
                        currTrackFrameSkip_u32 = params_o.gettrckngFrmSkps_rx()[ v_Idx_u32 ];
                        break;
                    }
                }
            }

            // --- Pick ROIs (once in init or every frame?)
            // Pick n (configured) ROI sets from the available list
            if ( shouldPickROIs_b )
            {
                uint32_t v_Col_u32 = 0;
                for ( uint32_t v_Row_u32 = 0; v_Row_u32 < params_o.getNmPckdROIs_u32(); ++v_Row_u32 )
                {
                    uint32_t v_Pick_u32 = v_Row_u32 * params_o.getNmROIsPrPl_u32() + v_Col_u32;
                    pickedROIs_x[v_Row_u32] = v_Pick_u32;
                }
                shouldPickROIs_b = false;
            }

            // --- check the detection skip and detect if it's time to
            bool_t v_ShouldDetect_b = (nextDetectionFrame_u32 == 0) || (i_CurrFrameNum_u32 >= nextDetectionFrame_u32);
            if ( v_ShouldDetect_b )
            {
                START_TIMER( t );
                detectFeatures_b();
                STOP_TIMER( t );

                TRACE_5( hTracer_s64, "[%s] [%s] [%s]: Detection completed in Frame [%lu]. Elapsed time [%.1f]ms ",  \
                    kCameraStrings[ cameraID_t ].c_str(), m_LFC_ID.c_str(), __func__, currFrameNum_u32, GET_ELAPSED_TIME( t ));
                nextDetectionFrame_u32 = i_CurrFrameNum_u32 + params_o.getDtctnFrmSkp_u32() + 1;
            }
            else
            {
                m_logStream << ", ";
            }

            // -- Track current features
            {
                START_TIMER( t );
                trackFeatures_b();
                STOP_TIMER( t );

                TRACE_5( hTracer_s64, "[%s] [%s] [%s]: Tracking completed in Frame [%lu]. Elapsed time [%.1f]ms ",  \
                    kCameraStrings[ cameraID_t ].c_str(), m_LFC_ID.c_str(), __func__, currFrameNum_u32, GET_ELAPSED_TIME( t ));
            }

            // Terminate lost tracks
            terminateLostTracks_v();
            SET_GPIO_OFF( GPIO_BIT_8 );
            STOP_TIMER( totalTimer );
            TRACE_4( hTracer_s64, "[%s] [%s]: Processed Frame [%lu] in Total [%.1f]ms",  \
                kCameraStrings[ cameraID_t ].c_str(), m_LFC_ID.c_str(), currFrameNum_u32, GET_ELAPSED_TIME( totalTimer ));

        }
        v_Ret_b = true;
    }
    return v_Ret_b;
}

// ----------------------------------------------------------------------------
bool_t LocalFeatureCollector::CreateFeatureTrack( uint32_t i_RoiIdx_u32, const tsc_math::ROIRect& i_Patch_rt )
{
    bool_t v_Ret_b = true;
    FeatureTrack v_NewTrack_o;
    v_NewTrack_o.PutPKmaticMdl(pKinematicModelCameraImpl_po->getImplObject_po());
    v_NewTrack_o.getPKmaticMdl_po()->ResetMotionVector( false );

    float64_t v_XOff_f64;
    float64_t v_YOff_f64;
    float64_t v_Deg_f64;
#ifdef USE_SVSCM
    v_XOff_f64 = pCameraModel_po->GetOrientation().GetCmrXOffst();
    v_YOff_f64 = pCameraModel_po->GetOrientation().GetCmrYOffst();
    v_Deg_f64 = pCameraModel_po->GetOrientation().GetCmrPrRll();
#else
    v_XOff_f64 = pCameraModel_po->GetExtrinsics().pos_x.cVal_ax[0];
    v_YOff_f64 = pCameraModel_po->GetExtrinsics().pos_x.cVal_ax[1];
    v_Deg_f64 = pCameraModel_po->getPreRollKmcs_f64();
#endif
    v_NewTrack_o.getPKmaticMdl_po()->SetCameraOffset( v_XOff_f64, v_YOff_f64, v_Deg_f64 );

    // --- use patch in the current image as descriptor
    // --- delete[] is done during cleanup
    uint32_t v_CurrPatchIndex_u32 = static_cast<uint32_t>(i_Patch_rt.x_s32 + i_Patch_rt.y_s32 * frameSize_t.width_s32);
    const uint8_t *c_RefBlock_pu8 = c_Img_pu8;
    c_RefBlock_pu8 += v_CurrPatchIndex_u32;
    ImageROICopy( c_RefBlock_pu8, frameSize_t.width_s32, &v_NewTrack_o.getLstDscrptr_rx()[0], params_o.getPtchSz_t().width_s32, params_o.getPtchSz_t() );

    v_NewTrack_o.PutRoiI(i_RoiIdx_u32);
    v_NewTrack_o.PutTrckFrmSkp(currTrackFrameSkip_u32);
    v_NewTrack_o.PutTrckFrmNum(currFrameNum_u32);
    v_NewTrack_o.PutIsTerm(false);
    v_NewTrack_o.PutIsValid(true);

    ImageFeature v_Feature_o;
    v_Feature_o.PutPt((i_Patch_rt.x_s32 + patchRadius_u32), (i_Patch_rt.y_s32 + patchRadius_u32));
    v_Feature_o.PutFn(currFrameNum_u32);

    float64_t v_TmpX_f64;
    float64_t v_TmpY_f64;
    float64_t v_TmpPsi_f64;
    bool_t v_Result_b = v_NewTrack_o.getPKmaticMdl_po()->GetMotionVector( v_TmpX_f64, v_TmpY_f64, v_TmpPsi_f64 );
    v_Feature_o.getMV_o().PutX(v_TmpX_f64);
    v_Feature_o.getMV_o().PutY(v_TmpY_f64);
    v_Feature_o.getMV_o().PutPsi(v_TmpPsi_f64);
    if( !v_Result_b )
    {
        TRACE_3( hTracer_s64, "[%s] LFC[%s]: CreateFeatureTrack: Failed to get motion vector for Frame [%lu]",  \
            kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), currFrameNum_u32 );

        v_Ret_b = false;
        pDiagnosticData_po->incUndetectedFeatures_v();
    }
    else
    {
        v_NewTrack_o.getTrckLst_rx().pushBack_v( v_Feature_o );
        localTrackList_o.getTrcks_rx().pushBack_v( v_NewTrack_o );
    }
    return v_Ret_b;
}

//-------------------------------------------------------------------------

//-------------------------------------------------------------------------

void LocalFeatureCollector::terminateLostTracks_v()
{
    CREATE_TIMER( t );
    START_TIMER( t );

    // find the terminated tracks
    // and release their kinematic model
    mecl::core::ArrayList < FeatureTrack, tsc_cfg::NUM_TRACKS > ::iterator v_TrackItr_t = localTrackList_o.getTrcks_rx().rwBegin_o();
    mecl::core::ArrayList < FeatureTrack, tsc_cfg::NUM_TRACKS > ::const_iterator v_Endlp_t = localTrackList_o.getTrcks_rx().end_o();
    for( ; v_TrackItr_t != v_Endlp_t; ++v_TrackItr_t )
    {
        FeatureTrack& v_Ft_ro =  * v_TrackItr_t;
        if( !v_Ft_ro.getIsTerm_b() )
        {
            continue;
        }

        // delete the dynamic array

        pKinematicModelCameraImpl_po->FreeImplObject( v_Ft_ro.getPKmaticMdl_po() );
        // After releasing make the handle invalid
        v_Ft_ro.PutPKmaticMdl(NULL);
    }
    STOP_TIMER( t );
    TRACE_2( hTracer_s64, "[%s] TerminateLostTracks completed. Elapsed time [%.1f]ms ", m_LFC_ID.c_str(), GET_ELAPSED_TIME( t ));
}

// ----------------------------------------------------------------------------

bool_t LocalFeatureCollector::detectFeatures_b()
{
    // Given a ROI, Choose a random feature patch and take its centroid as the feature point
    // for the number of patches in each roi, random select a patch

    // --- loop through the roiIdx local ROIs
    uint32_t v_Endlp_u32 = params_o.getNmPckdROIs_u32();
    mecl::core::Point4D<float32_t>leftROICorner; // new code
    mecl::core::Point2D<float32_t>leftROICorner_pix;// new code
    mecl::core::Point2D<uint32_t>leftROICorner_pix_x;// new code
    static uint32_t v_cout_u32 = 0;
	// check size on this loop does it need to be 4?  JJTURK
    for ( uint32_t v_Index_u32 = 0; v_Index_u32 < v_Endlp_u32; ++v_Index_u32)
    {
        uint32_t v_RoiIdx_u32 = pickedROIs_x[ v_Index_u32 ];
        // reference to current ROI
        const tsc_math::ROIRect& c_CurrentROI_rt = params_o.getavlblROIs_rx()[ v_RoiIdx_u32 ];
        leftROICorner(0) = static_cast <float32_t> (c_CurrentROI_rt.x_s32) + pCameraModel_po->GetExtrinsics().pos_x.cVal_ax[0]; // new code starts here
        leftROICorner(1) = static_cast <float32_t> (c_CurrentROI_rt.y_s32) + pCameraModel_po->GetExtrinsics().pos_x.cVal_ax[1];
        leftROICorner(2) = 0.0f;
        leftROICorner(3) = 1.0f;
        pCameraModel_po->ApplyFullProjection(leftROICorner, leftROICorner_pix);
        leftROICorner_pix /= 2.0f;
        leftROICorner_pix_x(0) = static_cast <sint32_t> (leftROICorner_pix(0)) - (c_CurrentROI_rt.width_s32/2);
        leftROICorner_pix_x(1) = static_cast <sint32_t>(leftROICorner_pix(1)) - (c_CurrentROI_rt.height_s32/2);
        sint32_t v_RandWidth_s32 = c_CurrentROI_rt.width_s32 - params_o.getPtchSz_t().width_s32;
        sint32_t v_RandHeight_s32 = c_CurrentROI_rt.height_s32 - params_o.getPtchSz_t().height_s32;

        for ( uint32_t v_InnerIndex_u32 = 0; v_InnerIndex_u32 < params_o.getNmPtchsPrROI_u32(); ++v_InnerIndex_u32 )
        {
            tsc_math::ROIRect v_Patch_t;
            if( params_o.getRndmPtchs_b() )
            {
                //srand(time(NULL));
                v_Patch_t.x_s32 = (rand() % v_RandWidth_s32) + static_cast <sint32_t> (leftROICorner_pix_x(0));
                //srand(time(NULL));
                v_Patch_t.y_s32 = (rand() % v_RandHeight_s32) + static_cast <sint32_t> (leftROICorner_pix_x(1));
            }
            else
            {
                v_Patch_t.x_s32 = static_cast <sint32_t> (leftROICorner_pix_x(0));
                v_Patch_t.y_s32 = static_cast <sint32_t> (leftROICorner_pix_x(1));
            }

            v_Patch_t.width_s32 = params_o.getPtchSz_t().width_s32;
            v_Patch_t.height_s32 = params_o.getPtchSz_t().height_s32;

            CreateFeatureTrack( v_RoiIdx_u32, v_Patch_t );
            if (v_cout_u32 % 100)
            {
                OC_DEBUG_PRINTF( "[LFC] ROI{%u} [CreateFeatureTrack] at (%u, %u) \n", v_RoiIdx_u32, v_Patch_t.x_x, v_Patch_t.y_x);
            }
        }
    }
    v_cout_u32++;
    return true;
}

//-------------------------------------------------------------------------

bool_t LocalFeatureCollector::trackFeatures_b()
{
    bool_t v_Return_b = true;
    // --- loop through the live tracks and track them in current frame if it's time to
    mecl::core::ArrayList < FeatureTrack,tsc_cfg::NUM_TRACKS > ::iterator v_Itr_t = localTrackList_o.getTrcks_rx().rwBegin_o();
    mecl::core::ArrayList < FeatureTrack,tsc_cfg::NUM_TRACKS > ::const_iterator v_Endlp_t = localTrackList_o.getTrcks_rx().end_o();
    for( ; v_Itr_t != v_Endlp_t; ++v_Itr_t )
    {
        FeatureTrack& v_Track_ro = *v_Itr_t;
       // --- track only live feature tracks which their time is this frame
        if ( (v_Track_ro.getIsTerm_b()) || (currFrameNum_u32 < ( v_Track_ro.getTrckFrmNum_u32() + v_Track_ro.getTrckFrmSkp_u32() + 1 )) )
        {
            continue;
        }

        // Predict and create search window
        ImageFeature& v_LastFeature_ro = v_Track_ro.getTrckLst_rx().back_ro();
        fc::MotionVector v_MotionVector_o;
        float64_t v_TmpX_f64;
        float64_t v_TmpY_f64;
        float64_t v_TmpPsi_f64;
        bool_t v_Result_b = v_Track_ro.getPKmaticMdl_po()->GetMotionVector( v_TmpX_f64, v_TmpY_f64, v_TmpPsi_f64 );
        v_MotionVector_o.PutX(v_TmpX_f64); 
        v_MotionVector_o.PutY(v_TmpY_f64); 
        v_MotionVector_o.PutPsi(v_TmpPsi_f64);
        if( !v_Result_b )
        {
            TRACE_4( hTracer_s64, "[%s] LFC[%s] [%s]: Failed to get motion vector for Frame [%lu]",  \
                kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__, currFrameNum_u32 );

            v_Track_ro.PutIsTerm(true);
            continue;
        }

        Point v_Tmp2ndIPt_t;
        fc::Pointd v_TmpUnWarped_x;
        v_Result_b = PredictImageCoordinatesStraight( v_LastFeature_ro.getPt_t(), v_MotionVector_o.getX_f64(),  \
        v_MotionVector_o.getY_f64(), v_Tmp2ndIPt_t, v_TmpUnWarped_x );
        v_LastFeature_ro.Put2ndIPt(v_Tmp2ndIPt_t);
        v_LastFeature_ro.PutUnWarpedPt(v_TmpUnWarped_x);
        if( !v_Result_b )
        {
            TRACE_3( hTracer_s64, "[%s] LFC[%s] [%s]: Failed to PredictImageCoordinates, skipping...",  \
                kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__ );

            v_Track_ro.PutIsTerm(true);
            continue;
        }

        TRACE_4( hTracer_s64, "LFC [TrackFeatures] at (%u, %u) predicted at (%u, %u)", v_LastFeature_ro.getPt_t().x_x, v_LastFeature_ro.getPt_t().y_x,
        		             v_LastFeature_ro.get2ndIPt_t().x_x, v_LastFeature_ro.get2ndIPt_t().y_x);

        // As a bi-product, we got the unwarped point too
        v_LastFeature_ro.PutIsUnWarped(true);

        // --- Do tracking
        Point v_Next_t;
        float32_t v_MinDist_f32;
        v_Result_b = applyBlockMatching_b( v_Track_ro, params_o.getTrckrThrsh1_u32(), params_o.getTrckrThrsh2_u32(), v_Next_t, v_MinDist_f32 );
        // --- Windows LOGGING ---------------------------------------------------
        m_logStream << currFrameNum_u32 << ", ";
        m_logStream << v_LastFeature_ro.getPt_t().x_x << ", ";
        m_logStream << v_LastFeature_ro.getPt_t().y_x << ", ";
        m_logStream << v_LastFeature_ro.get2ndIPt_t().x_x << ", ";
        m_logStream << v_LastFeature_ro.get2ndIPt_t().y_x << ", ";
        m_logStream << v_Next_t.x_x << ", ";
        m_logStream << v_Next_t.y_x << ", ";
        m_logStream << ( v_Next_t.x_x - v_LastFeature_ro.get2ndIPt_t().x_x ) << ", ";
        m_logStream << ( v_Next_t.y_x - v_LastFeature_ro.get2ndIPt_t().y_x ) << ", ";
        m_logStream << v_MinDist_f32 << "\n";

        // ------------------------------------------------------------------------
        if( !v_Result_b )
        {
            TRACE_5( hTracer_s64, "[%s] LFC[%s] [%s]: Unable to find a BMA match for the track, predicted at (%lu,%lu)",  \
                kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__, v_LastFeature_ro.get2ndIPt_t().x_x, v_LastFeature_ro.get2ndIPt_t().y_x );

            // --- if lost, terminate the track
            v_Track_ro.PutIsTerm(true);
            continue;
        }

        // --- update the tracklist w/ new intel
        {
            // --- update descriptor
            uint32_t v_TopLeftIndex_u32 = static_cast< uint32_t >( ( v_Next_t.x_x - patchRadius_u32 ) + ( v_Next_t.y_x - patchRadius_u32 ) * frameSize_t.width_s32 );
            const uint8_t *c_RefBlock_pu8 = c_Img_pu8;
            c_RefBlock_pu8 += v_TopLeftIndex_u32;
            ImageROICopy( c_RefBlock_pu8, frameSize_t.width_s32, &v_Track_ro.getLstDscrptr_rx()[0], params_o.getPtchSz_t().width_s32, params_o.getPtchSz_t() );

            v_Track_ro.PutTrckFrmNum(currFrameNum_u32);

            ImageFeature v_Feature_o;
            v_Feature_o.PutPt((v_Next_t.x_x), (v_Next_t.y_x));
            v_Feature_o.PutFn(currFrameNum_u32);
            v_Feature_o.PutMV(v_MotionVector_o);
            v_Feature_o.PutMatchDist(v_MinDist_f32);

            v_Track_ro.getTrckLst_rx().pushBack_v( v_Feature_o );
            TRACE_5( hTracer_s64, "LFC [TrackFeatures] found a BMA match at (%u, %u) predicted at (%u, %u) with minDist %.3f", v_Next_t.x_x, v_Next_t.y_x,
            		             v_LastFeature_ro.get2ndIPt_t().x_x, v_LastFeature_ro.get2ndIPt_t().y_x, v_MinDist_f32);

            // --- Reset track's motion vector: always between consecutive tracks
            v_Track_ro.getPKmaticMdl_po()->ResetMotionVector( false );
        }

        // if there's enough track length, terminate track
        size_t v_TrackLength_t = v_Track_ro.getTrckLst_rx().size_u32();
        if ( v_TrackLength_t >= params_o.getTrckLngth_u32() )
        {
            // --- Simply sum up the individual feature motion vectors
            // --- (Special case for straight line motion)
            // --- skip the first element's since it's zero anyways
            for ( size_t v_Index_t = 1; v_Index_t < v_TrackLength_t; ++v_Index_t )
            {
                fc::MotionVector &v_MV_ro = v_Track_ro.getTrckLst_rx()[ v_Index_t ].getMVAddress_ro();
                v_Track_ro.PutTotalMVPlus(v_MV_ro);
            }

            v_Track_ro.PutIsTerm(true);
            fc::Pointd v_TempUnwarpedPt_x;
            bool v_InnerResult_b = pCameraModel_po->Unwarp( v_Track_ro.getTrckLst_rx().back_ro().getPt_t(), v_TempUnwarpedPt_x );
            v_Track_ro.getTrckLst_rx().back_ro().PutUnWarpedPt(v_TempUnwarpedPt_x);
            if( !v_InnerResult_b )
            {
                TRACE_5( hTracer_s64, "[%s] [%s] LFC[%s]: Unable to unwarp KeyPoint (%d, %d); Error...",
                         __FUNC__, kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), v_Track_ro.getTrckLst_rx().back_ro().getPt_t().x_x, v_Track_ro.getTrckLst_rx().back_ro().getPt_t().y_x );
                v_Return_b = false;
                break;
            }
            ComputeSfM( v_Track_ro );
            LogTrack( v_Track_ro );

            if(true == v_Return_b)
            {
              continue;
            }
        }
    }

    return v_Return_b;
}

//-------------------------------------------------------------------------

//-------------------------------------------------------------------------

bool_t LocalFeatureCollector::PredictImageCoordinatesStraight(  \
    const Point& i_WarpedPtImg1_rt, float64_t i_DeltaX_f64, float64_t i_DeltaY_f64, Point& o_WarpedPtImg2_rt, fc::Pointd& b_UnwarpedPt1_rx ) const
{
    bool_t v_Ret_b = true;

    // First unwarp the 1st image point
    bool_t v_Result_b = pCameraModel_po->Unwarp( i_WarpedPtImg1_rt, b_UnwarpedPt1_rx );

    if( !v_Result_b )
    {
        TRACE_5( hTracer_s64, "[%s] [%s] LFC[%s]: Unable to unwarp KeyPoint (%d, %d); Skipping...",  \
            __func__, kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), i_WarpedPtImg1_rt.x_x, i_WarpedPtImg1_rt.y_x );
        v_Ret_b = false;
    }
    else if( (cameraID_t < 0) || (cameraID_t >= tsc_cfg::MAX_NUM_CAMERA_CALIB) )
    {
        TRACE_3( hTracer_s64, "[%s] [%s] LFC[%s]: Unable to predict image point. Invalid Camera ID; Skipping...",  \
            __func__, kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str() );
        v_Ret_b = false;
    }
    else
    {
#ifdef USE_SVSCM
            float64_t v_MC_x[3];
            v_MC_x[0] = b_UnwarpedPt1_rx.x_x;
            v_MC_x[1] = b_UnwarpedPt1_rx.y_x;
            v_MC_x[2] = 1.0;

            v_Result_b = tsc_math::MatrixVectorMultiply( reinterpret_cast<float64_t*>(pCameraModel_po->GetIntrinsic().GetKInvPtr()), 3, 3, false, &v_MC_x[ 0 ], &v_MC_x[ 0 ] );
            if( !v_Result_b )
            {
                TRACE_3( hTracer_s64, "[%s] [%s] LFC[%s]: Failed to compute m_c = invK x pt1", __func__, kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str() );
                v_Ret_b = false;
            }
        
            if ( v_Ret_b )
            {
                // transform s1 & m_c to Standard Coordinate System
                float64_t v_S1_f32 = 0.0;
                float64_t temp;
                if ( cameraID_t == tscApi::e_TscFrontCam )
                {
                    v_S1_f32 = i_DeltaY_f64;

                    temp = v_MC_x[ 0 ];
                    v_MC_x[ 0 ] = v_MC_x[ 1 ];
                    v_MC_x[ 1 ] = -temp;
                }
                else if ( cameraID_t == tscApi::e_TscRearCam )
                {
                    v_S1_f32 = -i_DeltaY_f64;

                    temp = v_MC_x[ 0 ];
                    v_MC_x[ 0 ] = - v_MC_x[ 1 ];
                    v_MC_x[ 1 ] = temp;
                }
                else if ( cameraID_t == tscApi::e_TscLeftCam )
                {
                    v_S1_f32 = i_DeltaX_f64;
                    // no transform needed for left camera
                }
                else if ( cameraID_t == tscApi::e_TscRightCam )
                {
                    v_S1_f32 = -i_DeltaX_f64;

                    v_MC_x[ 0 ] = - v_MC_x[ 0 ];
                    v_MC_x[ 1 ] = - v_MC_x[ 1 ];
                }
                else
                {
                }

                float64_t v_R3_x[3];
                float64_t v_Mu_x;
                float64_t v_Theta_f32;
                float64_t v_Lambda_f32;
                float64_t v_MCp_x[ 3 ];
                // PRQA S 3706 ++
                v_R3_x[ 0 ] = pCameraModel_po->GetRstdPtr()[0][2];
                v_R3_x[ 1 ] = pCameraModel_po->GetRstdPtr()[1][2];
                v_R3_x[ 2 ] = pCameraModel_po->GetRstdPtr()[2][2];
                // PRQA S 3706 --

                tsc_math::MatrixVectorMultiply( &v_R3_x[ 0 ], 3, 1, true, &v_MC_x[ 0 ], &v_Mu_x );

                v_Theta_f32 = v_S1_f32 / pCameraModel_po->GetExtrinsic().GetZmm();

                v_Lambda_f32 = 1.0 / ( 1.0 + v_Theta_f32 * v_Mu_x * pCameraModel_po->GetRstdPtr()[ 2 ][ 0 ] );   // PRQA S 3706
                for ( uint8_t i = 0; i < 3; ++i )
                {
                    v_MCp_x[ i ] = v_Lambda_f32 * ( v_MC_x[ i ] + v_Theta_f32 * pCameraModel_po->GetRstdPtr()[ i ][ 0 ] * v_Mu_x );   // PRQA S 3706
                }

                // transform m_cp back to Magna Coordinate System
                if ( cameraID_t == tscApi::e_TscFrontCam )
                {
                    temp = v_MCp_x[ 0 ];
                    v_MCp_x[ 0 ] = - v_MCp_x[ 1 ];
                    v_MCp_x[ 1 ] = temp;
                }
                else if ( cameraID_t == tscApi::e_TscRearCam )
                {
                    temp = v_MCp_x[ 0 ];
                    v_MCp_x[ 0 ] = v_MCp_x[ 1 ];
                    v_MCp_x[ 1 ] = - temp;
                }
                // no transform needed for left camera
                else if ( cameraID_t == tscApi::e_TscRightCam )
                {
                    v_MCp_x[ 0 ] = - v_MCp_x[ 0 ];
                    v_MCp_x[ 1 ] = - v_MCp_x[ 1 ];
                }else
                {
                }

                v_Result_b = tsc_math::MatrixVectorMultiply( reinterpret_cast<float64_t*>(pCameraModel_po->GetIntrinsic().GetKPtr()), 3, 3, false, &v_MCp_x[ 0 ], &v_MCp_x[ 0 ] );
                if( !v_Result_b )
                {
                    TRACE_3( hTracer_s64, "[%s] [%s] LFC[%s]: Failed to compute pt2 = invK x m_cp", __func__, kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str() );
                    v_Ret_b = false;
                }
                else
                {
                    fc::Pointd v_UnwarpedPt2_x( v_MCp_x[ 0 ], v_MCp_x[ 1 ] );
                    // Warp back the 2nd image point
                    pCameraModel_po->Warp( v_UnwarpedPt2_x, o_WarpedPtImg2_rt );
                }
            }
#else
            fc::Pointd v_CamPt1_x;
            pCameraModel_po->Image2Camera( pCameraModel_po->getCameraObj_px(), b_UnwarpedPt1_rx, v_CamPt1_x );

            mecl::core::Point3D< float32_t >::Config_s v_PtCfg_t = {{ static_cast<float32_t>(v_CamPt1_x.x_x), static_cast<float32_t>(v_CamPt1_x.y_x), 1.0 }};
            mecl::core::Point3D< float32_t > v_MC_x( v_PtCfg_t );
            float32_t v_S1_f32 = 0;
            if ( cameraID_t == tscApi::e_TscFrontCam )
            {
                v_S1_f32 = static_cast<float32_t>( -i_DeltaY_f64 );
            }
            else if ( cameraID_t == tscApi::e_TscRearCam )
            {
                v_S1_f32 = static_cast<float32_t>( i_DeltaY_f64 );
            }
            else if ( cameraID_t == tscApi::e_TscLeftCam )
            {
                v_S1_f32 = static_cast<float32_t>( -i_DeltaX_f64 );
            }
            else if ( cameraID_t == tscApi::e_TscRightCam )
            {
                v_S1_f32 = static_cast<float32_t>( i_DeltaX_f64 );
            }
            else
            {
            }

            mecl::core::RotationMatrix<float32_t> v_RVcs_x = pCameraModel_po->getRotationMatrix_x();
            mecl::core::Matrix< float32_t, 3, 1 > v_R3_x = v_RVcs_x.col( 2 );
            mecl::core::Matrix< float32_t, 1, 1 > v_Mu_x = ( v_R3_x.t() ).mmul( v_MC_x );
            float32_t v_Theta_f32 = v_S1_f32 / pCameraModel_po->GetExtrinsics().pos_x.cVal_ax[2];
            float32_t v_Lambda_f32 = 1.0F / ( 1.0 + v_Theta_f32 * v_Mu_x( 0 ) * v_RVcs_x( 2, 0 ) );
            mecl::core::Matrix< float32_t, 3, 1 > v_R1_x = v_RVcs_x.col( 0 );
            mecl::core::Matrix< float32_t, 3, 1 > v_MCp_x = ( v_MC_x + v_R1_x * v_Theta_f32 * v_Mu_x( 0 ) ) * v_Lambda_f32;
            fc::Pointd v_CamPt2_x( v_MCp_x( 0 ), v_MCp_x( 1 ) );  // PRQA S 3223
            fc::Pointd v_UnwarpedPt2_x;
            pCameraModel_po->Camera2Image( pCameraModel_po->getCameraObj_px(), v_CamPt2_x, v_UnwarpedPt2_x );
            // warp back the 2nd image point
            pCameraModel_po->Warp( v_UnwarpedPt2_x, o_WarpedPtImg2_rt );

#endif
    }
    return v_Ret_b;
}

//-------------------------------------------------------------------------

bool_t LocalFeatureCollector::MeetsDrivingConstraints(bool_t i_CameraFrontOpen_b)
{
    bool_t v_Ret_b = true;
    // Get the motion vector of this Frame
    float64_t v_Xoff_f64;
    float64_t v_Yoff_f64;
    float64_t v_Deg_f64;
#ifdef USE_SVSCM
    v_Xoff_f64 = pCameraModel_po->GetOrientation().GetCmrXOffst();
    v_Yoff_f64 = pCameraModel_po->GetOrientation().GetCmrYOffst();
    v_Deg_f64 = pCameraModel_po->GetOrientation().GetCmrPrRll();
#else
    v_Xoff_f64 = pCameraModel_po->GetExtrinsics().pos_x.cVal_ax[0];
    v_Yoff_f64 = pCameraModel_po->GetExtrinsics().pos_x.cVal_ax[1];
    v_Deg_f64 = pCameraModel_po->getPreRollKmcs_f64();
#endif        
    fc::FeatureCollection::getInstance_rt().getModuleInfo_rt().getMPKnmtcMdl_po()->SetCameraOffset( v_Xoff_f64, v_Yoff_f64, v_Deg_f64 );

    float64_t v_TmpX_f64;
    float64_t v_TmpY_f64;
    float64_t v_TmpPsi_f64;
    bool_t v_Result_b = fc::FeatureCollection::getInstance_rt().getModuleInfo_rt().getMPKnmtcMdl_po()->GetMotionVector(
                    v_TmpX_f64, v_TmpY_f64, v_TmpPsi_f64 );
    motionVector_o.PutX(v_TmpX_f64); 
    motionVector_o.PutY(v_TmpY_f64); 
    motionVector_o.PutPsi(v_TmpPsi_f64);
    if( !v_Result_b )
    {
        TRACE_4( hTracer_s64, "[%s] LFC[%s] [%s]: Failed to get motion vector for Frame [%lu]",  \
            kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__, currFrameNum_u32 );

        v_Ret_b = false;
    }
    else
    {
        // we need straight line motion
        float64_t v_LateralMotion_f64;
        if ( (cameraID_t == tscApi::e_TscFrontCam) || (cameraID_t == tscApi::e_TscRearCam) )
        {
            v_LateralMotion_f64 = motionVector_o.getX_f64();
        }
        else // for side cameras
        {
            v_LateralMotion_f64 = motionVector_o.getY_f64();
        }

        if ( mecl::math::abs_x<float64_t>( v_LateralMotion_f64 ) > km::KinematicModel::getInstance_rt().getModuleInfo_rt().getMStrghtMtnDstncThrshMM_f64() )
        {
            TRACE_4( hTracer_s64, "[%s] LFC[%s] [%s]: Lateral motion for Frame [%lu] more than expected, returning...",  \
                kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__, currFrameNum_u32 );
            v_Ret_b = false;
        }
        else if ( ( mecl::math::isAboutZero_b( motionVector_o.getX_f64() ) ) && ( mecl::math::isAboutZero_b( motionVector_o.getY_f64() ) ) )
        {
            TRACE_5( hTracer_s64, "[%s] [%s]: No motion detected for Frame [%lu]: dX=[%.3f] dY=[%.3f], returning...",  \
                kCameraStrings[ cameraID_t ].c_str(), __func__, currFrameNum_u32, motionVector_o.getX_f64(), motionVector_o.getY_f64() );
            v_Ret_b = false;
        }
        else if( ( (cameraID_t == tscApi::e_TscFrontCam) || (cameraID_t == tscApi::e_TscRearCam) ) && !i_CameraFrontOpen_b )
        {
            TRACE_3( hTracer_s64, "[%s] LFC[%s] [%s]: Camera Closed, returning...",  \
                kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__ );

            v_Ret_b = false;
        }
        else
        {
        }

    }
    return v_Ret_b;
}

//-------------------------------------------------------------------------
//PRQA S 6200 1 //const not added to input pointer variables, otherwise assignment produces error
bool_t LocalFeatureCollector::applyBlockMatching_b( FeatureTrack& i_Track_ro, uint32_t i_Thresh1_u32, uint32_t i_Thresh2_u32, Point& o_Next_rt, float32_t& o_MatchedDistance_rf32 ) const
{
    bool_t v_Ret_b = true;
    const ImageFeature& c_Last_ro = i_Track_ro.getTrckLst_rx().back_ro();

    // --- Find pixel matchedDistance from the bottom center of the image
    sint32_t v_X_s32 = frameSize_t.width_s32 / 2 - c_Last_ro.get2ndIPt_t().x_x - 1;
    sint32_t v_Y_s32 = frameSize_t.height_s32 - c_Last_ro.get2ndIPt_t().y_x - 1;
    uint32_t v_Pixdist_u32 = static_cast< uint32_t >( v_X_s32 * v_X_s32 + v_Y_s32 * v_Y_s32 );

    // --- Determine what level of decimation is required
    uint32_t v_Skip_u32;
    if( v_Pixdist_u32 > (i_Thresh1_u32 * i_Thresh1_u32) )
    {
        v_Skip_u32 = 1;
    }
    else if( v_Pixdist_u32 > (i_Thresh2_u32 * i_Thresh2_u32) )
    {
        v_Skip_u32 = 2;
    }
    else
    {
        v_Skip_u32 = 4;
    }

    // predicted point and patch should stay inside image bounds
    if( (c_Last_ro.get2ndIPt_t().x_x < static_cast< sint32_t >( patchRadius_u32 )) ||  \
        (c_Last_ro.get2ndIPt_t().y_x < static_cast< sint32_t >( patchRadius_u32 ))  ||  \
        (c_Last_ro.get2ndIPt_t().x_x >= static_cast< sint32_t >( frameSize_t.width_s32 - v_Skip_u32 * patchRadius_u32 + 1 )) ||  \
        (c_Last_ro.get2ndIPt_t().y_x >= static_cast< sint32_t >( frameSize_t.height_s32 - v_Skip_u32 * patchRadius_u32 + 1 )) )
    {
        TRACE_5( hTracer_s64, "[%s] LFC[%s] [%s]: Predicted patch centered at (%lu,%lu) falls outside image bounds, returning...",  \
            kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__, c_Last_ro.get2ndIPt_t().v_X_s32, c_Last_ro.get2ndIPt_t().v_Y_s32 );
        v_Ret_b =  false;
    }
    else if( v_Skip_u32 == 1 )
    {
        sint32_t search_win_x1 = c_Last_ro.get2ndIPt_t().x_x - patchRadius_u32 - params_o.getSrchRds_u32();
        if( search_win_x1 < 0 )
        {
            search_win_x1 = 0;
        }

        sint32_t search_win_x2 = c_Last_ro.get2ndIPt_t().x_x + patchRadius_u32 + params_o.getSrchRds_u32();
        if( search_win_x2 >= frameSize_t.width_s32 )
        {
            search_win_x2 = frameSize_t.width_s32 - 1;
        }

        sint32_t search_win_y1 = c_Last_ro.get2ndIPt_t().y_x - patchRadius_u32 - params_o.getSrchRds_u32();
        if( search_win_y1 < 0 )
        {
            search_win_y1 = 0;
        }

        sint32_t search_win_y2 = c_Last_ro.get2ndIPt_t().y_x + patchRadius_u32 + params_o.getSrchRds_u32();
        if( search_win_y2 >= frameSize_t.height_s32 )
        {
            search_win_y2 = frameSize_t.height_s32 - 1;
        }

        sint32_t search_win_size_x = search_win_x2 - search_win_x1 + 1;
        sint32_t search_win_size_y = search_win_y2 - search_win_y1 + 1;

        /* Non-decimated case */
        /* Extract the reference block */
        const uint8_t* c_SrcBlock_pu8 = &i_Track_ro.getLstDscrptr_rx()[0];
        uint32_t v_TopLeftIndex_u32 = static_cast< uint32_t >( search_win_x1 + search_win_y1 * frameSize_t.width_s32 );
        const uint8_t* c_SearchBlock_pu8 = c_Img_pu8;
        c_SearchBlock_pu8 += v_TopLeftIndex_u32;

        bool_t v_Result_b;
        if ( params_o.getTrckUsngMAD_b() )
        {
            uint32_t v_Dist_u32;
            v_Result_b = FindBlockSAD( c_SrcBlock_pu8, params_o.getPtchSz_t().width_s32, c_SearchBlock_pu8, frameSize_t.width_s32,  \
                search_win_size_x - params_o.getPtchSz_t().width_s32, search_win_size_y - params_o.getPtchSz_t().width_s32, o_Next_rt, v_Dist_u32 );
            o_MatchedDistance_rf32 = static_cast< float32_t >( v_Dist_u32 );
        }
        else
        {
            v_Result_b = FindBlockCorrelation( c_SrcBlock_pu8, params_o.getPtchSz_t().width_s32, c_SearchBlock_pu8, frameSize_t.width_s32,  \
                search_win_size_x - params_o.getPtchSz_t().width_s32, search_win_size_y - params_o.getPtchSz_t().width_s32, o_Next_rt, o_MatchedDistance_rf32 );
        }

        if( !v_Result_b )
        {
            TRACE_5( hTracer_s64, "[%s] LFC[%s] [%s]: Failed to find a block match predicted at (%lu,%lu), returning...",  \
                kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__, c_Last_ro.get2ndIPt_t().v_X_s32, c_Last_ro.get2ndIPt_t().v_Y_s32 );
            v_Ret_b = false;
        }
        else
        {
            // --- next so far refers to the patch top-left, relative to the search window
            o_Next_rt.x_x += search_win_x1 + patchRadius_u32;
            o_Next_rt.y_x += search_win_y1 + patchRadius_u32;
        }
    }
    else
    {
    }

    return v_Ret_b;
}

//-------------------------------------------------------------------------
// PRQA S 4212 ++
bool_t LocalFeatureCollector::FindBlockSAD(  \
    const uint8_t* restrict i_SrcImg_pu8, sint32_t i_SrcStep_s32,  \
    const uint8_t* restrict i_RefImg_pu8, sint32_t i_RefStep_s32,  \
    sint32_t i_SlideX_s32, sint32_t i_SlideY_s32, Point& o_PT_rt, uint32_t& o_MinDist_ru32 ) const
// PRQA S 4212 --
{
    bool_t v_Found_b = false;
    bool_t v_Err_b = false;
    sint32_t v_X_s32;
    sint32_t v_Y_s32;

    o_MinDist_ru32 = ~0U;
    const uint8_t *c_Src_pu8 = i_SrcImg_pu8;
    BlockMatchAlgoEnum_e v_Algo_e = e_BmaSAD16x16; // TODO: to be set in configuration API

    SET_GPIO_ON( GPIO_BIT_10 );

    // --- x & y: top left position being slided
    for( v_Y_s32 = 0; v_Y_s32 < i_SlideY_s32; ++v_Y_s32 )
    {
        sint32_t yOffset = v_Y_s32 * i_RefStep_s32;
        for( v_X_s32 = 0; v_X_s32 < i_SlideX_s32; ++v_X_s32 )
        {
            uint32_t v_Dist_u32 = 0;
            uint32_t v_TopLeftIndex_u32 = v_X_s32 + yOffset;
            const uint8_t *c_Ref_pu8 = i_RefImg_pu8;
            c_Ref_pu8 += v_TopLeftIndex_u32;

            bool_t v_Result_b = true;
            switch ( i_SrcStep_s32 )
            {
            case 8:
            {
                v_Result_b = tsc_math::GetSAD8x8u8( c_Src_pu8, i_SrcStep_s32, c_Ref_pu8, i_RefStep_s32, &v_Dist_u32 ); // IPPVC_MC_APX_FF
                break;
            }
            case 16:
            {
                switch( v_Algo_e ) {
                case e_BmaSAD16x16:
                {
                   v_Result_b = tsc_math::GetSAD16x16u8( c_Src_pu8, i_SrcStep_s32, c_Ref_pu8, i_RefStep_s32, &v_Dist_u32 ); // IPPVC_MC_APX_FF
                   break;
                }
                case e_BmaSAD16x16NeonOrig:
                {
#ifdef BMA_OPT_BY_ARM_NEON
                  tsc_math_neon::BlockMatch16x16_ARM_NEON_orig(reinterpret_cast<const uint32_t*>(c_Src_pu8), reinterpret_cast<const uint32_t*>(c_Ref_pu8), &v_Dist_u32);
#endif
                  break;
                }
                case e_BmaSAD16x16NeonOpt:
                {
#ifdef BMA_OPT_BY_ARM_NEON
                  tsc_math_neon::BlockMatch16x16_ARM_NEON_opt(reinterpret_cast<const uint32_t*>(c_Src_pu8), reinterpret_cast<const uint32_t*>(c_Ref_pu8), &v_Dist_u32);
#endif
                  break;
                }
                default:
                {
                    TRACE_4( hTracer_s64, "[%s] LFC[%s] [%s]: This code should never be executed with this algo value %d", \
                        kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__, v_Algo_e );
                    v_Err_b = true;
                    break;
                }
                }
                break;
            }
            default:
            {
                TRACE_4( hTracer_s64, "[%s] LFC[%s] [%s]: Block size [%d] is not supported; returning false",  \
                    kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__, i_SrcStep_s32 );
                v_Err_b = true;
                break;
            }
            }

            if ( !v_Result_b )
            {
                TRACE_4( hTracer_s64, "[%s] LFC[%s] [%s]: SAD function returned with Error Number [%d], returning false",  \
                    kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__, v_Result_b );
                v_Err_b = true;
            }
            if( v_Err_b )
            {
                break;
            }
            uint32_t v_CurrentSAD_u32 = static_cast< uint32_t >( v_Dist_u32 );

            if( v_CurrentSAD_u32 < o_MinDist_ru32 )
            {
                o_MinDist_ru32 = v_CurrentSAD_u32;
                o_PT_rt.x_x = v_X_s32;
                o_PT_rt.y_x = v_Y_s32;
                v_Found_b = true;
            }
        }
        if( v_Err_b )
        {
            break;
        }
    }
    SET_GPIO_OFF( GPIO_BIT_10 );
    return v_Found_b;
}

//-------------------------------------------------------------------------
bool_t LocalFeatureCollector::FindBlockCorrelation(  \
    const uint8_t* restrict i_SrcImg_pu8, sint32_t i_SrcStep_s32,  \
    const uint8_t* restrict i_RefImg_pu8, sint32_t i_refStep_s32,  \
    sint32_t i_SlideX_s32, sint32_t i_SlideY_s32, Point& o_PT_rt, float32_t& o_MaxCorr_rf32 ) const
{
    bool_t v_Found_b = false;
    bool_t v_Err_b = false;
    sint32_t v_X_s32;
    sint32_t v_Y_s32;

    SET_GPIO_ON( GPIO_BIT_10 );

    //
    // Use e_BmaCcnl16x16_1Pass to be platform independent
    // Use e_BmaCcnl16x16Neon1PassOrig to get same results on ARM NEON, but ~3,7 times faster
    // Use e_BmaCcnl16x16Neon1PassOpt to  get same results on ARM NEON, but ~4,7 times faster
    //
    // JTURK double check this
    BlockMatchAlgoEnum_e v_Algo_e = e_BmaCcnl16x16OnePass; // TODO: move to configuration.cpp
    //BlockMatchAlgoEnum_e v_Algo_e = e_BmaCcnl16x16;
    o_MaxCorr_rf32 = - mecl::math::numeric_limits<float32_t>::infinity_x();
    const uint8_t *c_Src_pu8 = i_SrcImg_pu8;

    // --- x & y: top left position being slided
    for( v_Y_s32 = 0; v_Y_s32 < i_SlideY_s32; ++v_Y_s32 )
    {
        sint32_t v_YOffset_s32 = v_Y_s32 * i_refStep_s32;
        for( v_X_s32 = 0; v_X_s32 < i_SlideX_s32; ++v_X_s32 )
        {
            float32_t v_CurrentDist_f32 = 0.0;
#ifdef BMA_OPT_BY_ARM_NEON
            float32_t meanSrc = 0.0;
            float32_t meanRef = 0.0;
#endif
            uint32_t v_TopLeftIndex_u32 = v_X_s32 + v_YOffset_s32;
            const uint8_t *c_Ref_pu8 = i_RefImg_pu8;
            c_Ref_pu8 += v_TopLeftIndex_u32;

            bool_t v_Result_b = true;
            switch( v_Algo_e ) {
            case e_BmaCcnl16x16OnePass:
            {
                v_Result_b = tsc_math::CrossCorrValidNormLevel1Pass16x16u8( c_Src_pu8, i_SrcStep_s32, params_o.getPtchSz_t(), c_Ref_pu8, i_refStep_s32, params_o.getPtchSz_t(), &v_CurrentDist_f32 );
                break;
            }
            case e_BmaCcnl16x16Neon1PassOrig:
            {
#ifdef BMA_OPT_BY_ARM_NEON
                tsc_math_neon::CrossCorr16x16_NormLevel_I32_1Pass_ARM_NEON_orig(reinterpret_cast<const uint32_t *>(c_Src_pu8), reinterpret_cast<const uint32_t *>(c_Ref_pu8), &v_CurrentDist_f32 );
#endif
                break;
            }
            case e_BmaCcnl16x16Neon1PassOpt:
            {
#ifdef BMA_OPT_BY_ARM_NEON
              tsc_math_neon::CrossCorr16x16_NormLevel_I32_1Pass_ARM_NEON_opt(reinterpret_cast<const uint32_t *>(c_Src_pu8), reinterpret_cast<const uint32_t *>(c_Ref_pu8), &v_CurrentDist_f32 );
#endif
              break;
            }
            case e_BmaCcnl16x16:
            {
                v_Result_b = tsc_math::CrossCorrValidNormLevel16x16u8( c_Src_pu8, i_SrcStep_s32, params_o.getPtchSz_t(), c_Ref_pu8, i_refStep_s32, params_o.getPtchSz_t(), &v_CurrentDist_f32);
                break;
            }
#ifdef round
// SB
            case BMA_CCNL16x16_NEON_S8_ORIG:
            {
                uint32_t meanSrc_i = 0;
                uint32_t meanRef_i = 0;
                tsc_math_neon::CrossCorr16x16_Mean_F32_ARM_NEON_orig(reinterpret_cast<const uint32_t *>(c_Src_pu8), reinterpret_cast<const uint32_t *>(c_Ref_pu8), &meanSrc, &meanRef, 0.00390625 ); // 1/256
                meanSrc_i = round(meanSrc);
                meanRef_i = round(meanRef);
                tsc_math_neon::CrossCorr16x16_NormLevel_I32_ARM_NEON_orig(reinterpret_cast<const uint32_t *>(c_Src_pu8), reinterpret_cast<const uint32_t *>(c_Ref_pu8), &meanSrc_i, &meanRef_i, &v_CurrentDist_f32 );
                break;
            }
#endif
            case e_BmaCcnl16x16NeonF32Orig:
            {
#ifdef BMA_OPT_BY_ARM_NEON
              tsc_math_neon::CrossCorr16x16_Mean_F32_ARM_NEON_orig(reinterpret_cast<const uint32_t *>(c_Src_pu8), reinterpret_cast<const uint32_t *>(c_Ref_pu8), &meanSrc, &meanRef, 0.00390625); // 1/256
              tsc_math_neon::CrossCorr16x16_NormLevel_Float_ARM_NEON_orig(reinterpret_cast<const uint32_t *>(c_Src_pu8), reinterpret_cast<const uint32_t *>(c_Ref_pu8), &meanSrc, &meanRef, &v_CurrentDist_f32);
#endif
              break;
            }
            case e_BmaCcnl16x16NeonF32Opt:
            {
#ifdef BMA_OPT_BY_ARM_NEON
              tsc_math_neon::CrossCorr16x16_Mean_F32_ARM_NEON_opt(reinterpret_cast<const uint32_t *>(c_Src_pu8), reinterpret_cast<const uint32_t *>(c_Ref_pu8), &meanSrc, &meanRef, 0.00390625); // 1/256
              tsc_math_neon::CrossCorr16x16_NormLevel_Float_ARM_NEON_opt(reinterpret_cast<const uint32_t *>(c_Src_pu8), reinterpret_cast<const uint32_t *>(c_Ref_pu8), &meanSrc, &meanRef, &v_CurrentDist_f32);
#endif
              break;
            }
            case e_BmaCcn16x16NeonS8QuickOrig:
            {
#ifdef BMA_OPT_BY_ARM_NEON
              tsc_math_neon::Correlation16x16Norm_ARM_NEON_orig(reinterpret_cast<const uint32_t *>(c_Src_pu8), reinterpret_cast<const uint32_t *>(c_Ref_pu8), &v_CurrentDist_f32);
#endif
              break;
            }
   	    case e_BmaCcnl16x16NeonS8Opt:
            default:
            {
                v_Err_b = true;
                TRACE_4( hTracer_s64, "[%s] LFC[%s] [%s]: This code should never be executed with this algo value %d", \
                    kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__, v_Algo_e );
                break;
            }
            }

            if ( !v_Result_b )
            {
                TRACE_4( hTracer_s64, "[%s] LFC[%s] [%s]: CrossCorrelation function returned with Error Number [%d], returning false",  \
                    kCameraStrings[ cameraID_t ].c_str(), kROIIds[pROI_po->getID_t()].c_str(), __func__, v_Result_b );
                v_Err_b = true;
                break;
            }

            //int currDst = static_cast<int>( 10000 * v_CurrentDist_f32 );
            //int maxCorr = static_cast<int>( 10000 * o_MaxCorr_rf32 );

            if( v_CurrentDist_f32 > o_MaxCorr_rf32 )
            {
                o_MaxCorr_rf32 = v_CurrentDist_f32;
                o_PT_rt.x_x = v_X_s32;
                o_PT_rt.y_x = v_Y_s32;
                v_Found_b = true;
            }
        }
        if( v_Err_b )
        {
            break;
        }
    }
    SET_GPIO_OFF( GPIO_BIT_10 );

    if( v_Err_b || (o_MaxCorr_rf32 <= 0.3) )
    {
        v_Found_b = false;
    }

    return v_Found_b;
}

// ----------------------------------------------------------------------------

bool_t LocalFeatureCollector::ComputeSfM( FeatureTrack& b_FT_ro ) const
{
  bool_t v_Return_b = true;
  // transform motion vector to VCS
  float32_t v_S1_f32 = 0.0F;
  float32_t v_S2_f32 = 0.0F;

  if( pCameraModel_po->getID_t() == tscApi::e_TscFrontCam )
  {
    v_S1_f32 = static_cast< float32_t >( - b_FT_ro.getTotalMV_o().getY_f64() );
    v_S2_f32 = static_cast< float32_t >( b_FT_ro.getTotalMV_o().getX_f64() );
  }
  else if( pCameraModel_po->getID_t() == tscApi::e_TscRearCam )
  {
    v_S1_f32 = static_cast< float32_t >( b_FT_ro.getTotalMV_o().getY_f64() );
    v_S2_f32 = static_cast< float32_t >( - b_FT_ro.getTotalMV_o().getX_f64() );
  }
  else if( pCameraModel_po->getID_t() == tscApi::e_TscLeftCam )
  {
    v_S1_f32 = static_cast< float32_t >( - b_FT_ro.getTotalMV_o().getX_f64() );
    v_S2_f32 = static_cast< float32_t >( - b_FT_ro.getTotalMV_o().getY_f64() );
  }
  else if( pCameraModel_po->getID_t() == tscApi::e_TscRightCam )
  {
    v_S1_f32 = static_cast< float32_t >( b_FT_ro.getTotalMV_o().getX_f64() );
    v_S2_f32 = static_cast< float32_t >( b_FT_ro.getTotalMV_o().getY_f64() );
  }
  else
  {
    TRACE_2( hTracer_s64, "[%s] [%s]: Failed to transform motion vector to VCS. Invalid Camera ID",
        __FUNC__, kCameraStrings[ cameraID_t ].c_str() );
    v_Return_b = false;
  }

  if (true == v_Return_b)
  {
    mecl::core::Matrix<float32_t, 4U, 4U> v_MotionTransMat_x;
    camera_model::CameraModelMecl::BuildMotionTransformation( v_S1_f32, v_S2_f32, static_cast<float32_t>(b_FT_ro.getTotalMV_o().getPsi_f64()), v_MotionTransMat_x );
    fc::Point3d v_TempDesignWorldPt_t;
    fc::Pointd v_FrontPt_t = b_FT_ro.getTrckLst_rx().front_ro().getUnWarpedPt_x();
    bool v_Result_b = pCameraModel_po->computeWorldPt_b( v_FrontPt_t, b_FT_ro.getTrckLst_rx().back_ro().getUnWarpedPt_x(), v_MotionTransMat_x, v_TempDesignWorldPt_t );
    b_FT_ro.PutDesignWorldPt(v_TempDesignWorldPt_t);

    if( !v_Result_b )
    {
      TRACE_2( hTracer_s64, "[%s] [%s]: Failed to Calculate Corresponding World Point",  \
          fn, m_cameraModelMecl.id.c_str() );
      v_Return_b = false;
    }
  }

  return v_Return_b;
}

// ----------------------------------------------------------------------------
bool_t LocalFeatureCollector::LogTrack( FeatureTrack& ft )
{
    for( unsigned trackIdx = 0; trackIdx < ft.getTrckLst_rx().size_u32(); ++trackIdx )
    {
        m_tracksStream
                << m_trackNum
                << "," << trackIdx
                << "," << ft.getTrckLst_rx()[trackIdx].getFn_u32()
                << "," << ft.getTrckLst_rx()[trackIdx].getPt_t().x_x
                << "," << ft.getTrckLst_rx()[trackIdx].getPt_t().y_x
                << "," << ft.getTrckLst_rx()[trackIdx].getIsUnWarped_b()
                << "," << ft.getTrckLst_rx()[trackIdx].getUnWarpedPt_x().x_x
                << "," << ft.getTrckLst_rx()[trackIdx].getUnWarpedPt_x().y_x
                << "," << ft.getTrckLst_rx()[trackIdx].getSize_f32()
                << "," << ft.getTrckLst_rx()[trackIdx].getAngle_f32()
                << "," << ft.getTrckLst_rx()[trackIdx].getResponse_f32()
                << "," << ft.getTrckLst_rx()[trackIdx].getOctave_s32();

        if( trackIdx > 0 )
        {
            m_tracksStream << "," << ft.getTrckLst_rx()[trackIdx].getMatchDistance_f32();
        }
        else
        {
            m_tracksStream << ",";
        }

        m_tracksStream
                << "," << ft.getTrckLst_rx()[trackIdx].getMV_o().getX_f64()
                << "," << ft.getTrckLst_rx()[trackIdx].getMV_o().getY_f64()
                << "," << ft.getTrckLst_rx()[trackIdx].getMV_o().getPsi_f64();

        if( trackIdx > 0 )
        {
            m_tracksStream
                    << "," << ft.getTrckLst_rx()[trackIdx - 1].get2ndIPt_t().x_x
                    << "," << ft.getTrckLst_rx()[trackIdx - 1].get2ndIPt_t().y_x
                    << "," << ft.getTrckLst_rx()[trackIdx - 1].get2ndIPt_t().distance( ft.getTrckLst_rx()[trackIdx].getPt_t() );
        }
        else
        {
            for( size_t i = 0; i < 3; ++i )
            {
                m_tracksStream << ",";
            }
        }

        if( trackIdx == ft.getTrckLst_rx().size_u32() - 1 )
        {
            // Dump the valid feature world pt along the last row of tracklist
            m_tracksStream
                    << "," << ft.getRoiI_u32()
                    << "," << ft.getDesignWorldPt_t().x_x
                    << "," << ft.getDesignWorldPt_t().y_x
                    << "," << ft.getDesignWorldPt_t().z_x;
        }
        else
        {
            for( size_t i = 0; i < 4; ++i )
            {
                m_tracksStream << ",";
            }
        }

        m_tracksStream << "\n";
    }

    ++m_trackNum;
    return true;
}
// ----------------------------------------------------------------------------
