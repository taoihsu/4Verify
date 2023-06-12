// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [04-Jun-2013]
// --- Modified by Ehsan Parvizi [09-Oct-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
// This commented code is uncommented and used where IOSTREAM is supported.
// PRQA S 1051 1
//#include <iostream>
#include "stdafx.h"
#include "kinematicModel.h"
#include "featureCollectionImpl.h"
#include "featureCollection.h"
#include <stdlib.h>
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// SB using std::string;
using fc::ROI;
using tsc_trace::kCameraStrings;
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
fc::FeatureCollectionImpl::FeatureCollectionImpl() :
    ModuleImpl( ),
    kinematicModelCameraImpl_po( NULL ),
    frameCounter_u32( 0 )
{
    // Using initialization lists
    savedDataInfo_s.data_pu8 = reinterpret_cast<uint8_t*>(&savedData_o);
    savedDataInfo_s.len_u32 = savedData_o.getValidLength_u32();
}

// ----------------------------------------------------------------------------
fc::FeatureCollectionImpl::~FeatureCollectionImpl()
{
    if( tscTargetCamID == cameraID_t )
    {
        AppCtrl::WriteToFile( m_ValidFramesFileName, m_ValidFrames, m_ValidFramesHeader, false );
    }
  // PRQA S 4631 ++
  // PRQA S 4274 ++
  fc::FeatureCollectionImpl::unInit_b();
  // PRQA S 4631 --
  // PRQA S 4274 --
}

// ----------------------------------------------------------------------------
bool_t fc::FeatureCollectionImpl::Init(tscApi::TSCCtrlInfo* b_TscCtrlInfo_po, FeatureCollectionInfo* b_Info_po, FeatureCollectionConfig_s* b_Config_ps, FeatureCollectionCameraConfig_s* b_CameraConfig_ps, sint64_t i_Tracer_s64, tscApi::enuCameraID i_CameraId_t)
{
    bool_t v_Ret_b = true;
    diagnosticData_o.tscCameraTarget = b_TscCtrlInfo_po->tscTargetCamera;
    if ( !preInit_b(b_TscCtrlInfo_po, b_Info_po, b_Config_ps, b_CameraConfig_ps, i_Tracer_s64, i_CameraId_t) )
    {
        v_Ret_b = false;
    }
    else
    {
        kinematicModelCameraImpl_po = km::KinematicModel::getInstance_rt().getImplObject_pt(cameraID_t);
        if( (kinematicModelCameraImpl_po == NULL) || (!kinematicModelCameraImpl_po->isInitOk_b()) )
        {
            TRACE_0( m_hTracer, "Failed to create kinematic model camera object!" );
            v_Ret_b = false;
        }
        else
        {
        // --- now create our LFCs
        for( sint32_t v_RoiIdx_s32 = 0; v_RoiIdx_s32 < tsc_cfg::MAX_ROI_COUNT; ++v_RoiIdx_s32 )
        {
            lfc::LocalFeatureCollector *v_Lfc_po = &lfc_x[ v_RoiIdx_s32 ];
            ff::FeatureFilter *v_FF_po = &ff_x[ v_RoiIdx_s32 ];
            ROI *v_Roi_po = &rois_x[ v_RoiIdx_s32 ];
            if( !v_Lfc_po->Init(cameraID_t, v_Roi_po, b_TscCtrlInfo_po->getMVdWdth_u16(), b_TscCtrlInfo_po->getMVdHght_u16(), b_TscCtrlInfo_po->GetM_Cmrs(cameraID_t), kinematicModelCameraImpl_po, &diagnosticData_o) )
            {
                TRACE_0( m_hTracer, "Failed to Create the LFC object");
                v_Ret_b = false;
#ifdef DEBUG_TSC_ALG
            } else if( !v_FF_po->Init(cameraID_t, v_Roi_po, b_TscCtrlInfo_po->GetM_Cmrs(cameraID_t), &diagnosticData_o, &m_debugCounters) )
#else
            } else if( !v_FF_po->Init(cameraID_t, v_Roi_po, b_TscCtrlInfo_po->GetM_Cmrs(cameraID_t), &diagnosticData_o) )
#endif
            {
                TRACE_0( m_hTracer, "Failed to Create the FF object");
                v_Ret_b = false;
            } else
            {
            }
            srand( 74862 );
            if( !v_Ret_b )
            {
                break;
            }
        }
        if( v_Ret_b )
        {
            TRACE_0( m_hTracer, "LFCs, LFTs, and FFs configured successfully!" );

            // link FCImpl.m_savedDataInfo to TSCCtrlInfo
            b_TscCtrlInfo_po->PutM_PSvdDtInf(cameraID_t, &savedDataInfo_s);
            // set savedData.descriptor.cameraId
            savedData_o.getDescriptor_ro().SetCameraID(cameraID_t);
            // --- everything fine
            initOK_b = true;
        }
        }
    }
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

    if( tSCCtrlInfo_po->tscTargetCamera == cameraID_t )
    {
        m_ValidFramesFileName = camera + "_ValidFrames.csv";
        m_ValidFramesHeader.clear();
        m_ValidFramesHeader
                << "Frame, "
                << "NumValidFrames, "
                << "\n";
    }

    tscTargetCamID = tSCCtrlInfo_po->tscTargetCamera;

    return v_Ret_b;
}

// ----------------------------------------------------------------------------
bool_t fc::FeatureCollectionImpl::start_b(void)
{
    bool_t v_Ret_b = true;
    enable_v();
    if ( (kinematicModelCameraImpl_po == NULL) || (!kinematicModelCameraImpl_po->isInitOk_b()) ||
            (!kinematicModelCameraImpl_po->start_b()) )
    {
        v_Ret_b = false;
    }
    return v_Ret_b;
}

// ----------------------------------------------------------------------------

bool_t fc::FeatureCollectionImpl::unInit_b()
{
    reset_v();

    ff_x.init_v();

    lfc_x.init_v();

    rois_x.clear_v();

    // --- initialize the variables to their reset state
    kinematicModelCameraImpl_po = NULL;
    hTracer_s64 = 0;
    tSCCtrlInfo_po = NULL;
    config_px = NULL;
    cameraConfig_px = NULL;
    info_px = NULL;

    initOK_b = false;
    cameraID_t = static_cast<tscApi::enuCameraID>(0);

    return ( true );
}

void fc::FeatureCollectionImpl::reset_v()
{
    // -- clear localTracks
    resetAllLocalFeatureTracks_b(false);

    // --- clear validfeaturecollection
#ifdef ENABLE_SFM    // PRQA S 1070
    m_validFeatureCollection.GetVldFtrs().clear_v();
#endif

    initialGuesses_x.clear_v();

    // -- clear savedData
    savedData_o.reset_v();
    savedDataInfo_s.len_u32 = savedData_o.getValidLength_u32();

    availableFrameNumber_u32 = 0;
    frameCounter_u32 = 0;
    diagnosticData_o.reset_v();
#ifdef DEBUG_TSC_ALG
    memset(&m_debugCounters, 0, sizeof(tscApi::DebugCounters));
#endif

    srand( 74862 );

    enable_b = false;

}

// ----------------------------------------------------------------------------
bool_t fc::FeatureCollectionImpl::process_b(void)
{
    bool_t v_Ret_b = true;
    resetErrorType_v();
    diagnosticData_o.getOverlays_rx().clear_v();
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
    diagnosticData_o.getValidFeatures_u32().clear_v();
#endif
    if( (!initOK_b) || (!enable_b) )
    {
        setErrorRecoverable_v(false);
        v_Ret_b = false;
    }
    else
    {
        frameCounter_u32++;
        diagnosticData_o.incProcessedFrames_v();
        // --- store the current frame number
        availableFrameNumber_u32 = tSCCtrlInfo_po->getMFrmNmbr_u32();

        // --- call all LFCs to process
        for( uint8_t v_Index_u8 = 0; v_Index_u8 < tsc_cfg::MAX_ROI_COUNT; ++v_Index_u8 )
        {
            lfc::LocalFeatureCollector *v_LFC_po = &lfc_x[v_Index_u8];
            ff::FeatureFilter *v_FF_po = &ff_x[v_Index_u8];

            TRACE_1( m_hTracer, "[LFC #%u] Processing..", v_Index_u8 );

            // first, find valid features from previously terminated tracks
            // and put them into our collection

            if( !CollectValidFeatures( v_LFC_po, v_FF_po ) )
            {
                TRACE_1( m_hTracer, "[LFC #%u] Failed to collect valid features.", v_Index_u8 );
                v_Ret_b = false;
            }

            // Next, process the LFC
            // Let LFC delete all the terminated tracks since we've already taken
            // care of valid terminated tracks.

            if( !v_LFC_po->Process( availableFrameNumber_u32, tSCCtrlInfo_po->getMSpd_f32(), tSCCtrlInfo_po->getMCmrFrntOpn_b(), tSCCtrlInfo_po->GetM_Cmrs(cameraID_t) ) )
            {
                TRACE_1( m_hTracer, "[LFC #%u] Failed to process.", v_Index_u8 );
                setErrorRecoverable_v(false);
                v_Ret_b = false;
            }

            // --- call feature filter
            else if( !v_FF_po->Process( availableFrameNumber_u32, v_LFC_po ))
            {
                TRACE_1( m_hTracer, "[FF #%u] Failed to process.", v_Index_u8 );
                setErrorRecoverable_v(false);
                v_Ret_b = false;
            }else
            {
            }

            if( !v_Ret_b )
            {
                break;
            }
        }
    }
    // --- everything worked OK
    return v_Ret_b;
}

void fc::FeatureCollectionImpl::cleanupLocalData_v(void)
{
    resetAllLocalFeatureTracks_b(false);
}

// ----------------------------------------------------------------------------
bool_t fc::FeatureCollectionImpl::resetAllLocalFeatureTracks_b(bool_t i_Collect_b)
{
    bool_t v_Ret_b = true;
    // ---  do we even have LFCs configured?

    // --- loop through all LFCs and reset the LFTs
    for( sint32_t v_Index_s32 = 0; v_Index_s32 < tsc_cfg::MAX_ROI_COUNT; ++v_Index_s32 )
    {
        lfc::LocalFeatureCollector *v_LFC_po = &lfc_x[ v_Index_s32 ];
        ff::FeatureFilter *v_FF_po = &ff_x[ v_Index_s32 ];
        // first, find valid features from previously terminated tracks
        // and put them into our collection
        if( i_Collect_b )
        {
          if( !CollectValidFeatures( v_LFC_po, v_FF_po) )
          {
            TRACE_1( m_hTracer, "[LFC #%u] Failed to collect valid features.", v_Index_s32);
            v_Ret_b = false;
          }
        }
        if( !v_LFC_po->resetAllLocalFeatureTracks_b() )
        {
            v_Ret_b = false;
        }else
        {
        }

        if( !v_Ret_b )
        {
            break;
        }
    }

    return v_Ret_b;
}

// ----------------------------------------------------------------------------
bool_t fc::FeatureCollectionImpl::CollectValidFeatures( lfc::LocalFeatureCollector *b_LFC_po, ff::FeatureFilter const *i_FF_po )
{
    bool_t v_Ret_b = true;
 
    if( (b_LFC_po == NULL) || (b_LFC_po->getLocalFeatureTrackCollection_po() == NULL) )
    {
        TRACE_3( m_hTracer, "[%s] %s: Frame [%lu] Failed. LocalFeatureTrackCollection cannot be empty",  \
            kCameraStrings[m_CameraID].c_str(), __func__, m_availableFrameNumber );
        setErrorRecoverable_v(false);
        v_Ret_b= false;
    }else
    {

    fc::LocalFeatureTrackCollection& v_LFT_ro = *( b_LFC_po->getLocalFeatureTrackCollection_po() );
    if (v_LFT_ro.getTrcks_rx().size_u32() == 0)
    {
        TRACE_3( m_hTracer, "[%s] %s: Frame [%lu] No local tracks from last frame; skipping...", \
            kCameraStrings[m_CameraID].c_str(), __func__, m_availableFrameNumber );
            int frame = availableFrameNumber_u32;
    }else

    if ( i_FF_po == NULL )
    {
        TRACE_3( m_hTracer, "[%s] %s: Frame [%lu] Failed. FeatureFilter cannot be empty",  \
            kCameraStrings[m_CameraID].c_str(), __func__, m_availableFrameNumber );
        setErrorRecoverable_v(false);
        v_Ret_b = false;
    }else
    {
        CREATE_TIMER( timer );
        START_TIMER( timer );

        const mecl::core::ArrayList< uint32_t, tsc_cfg::NUM_VALID_INDICES >& c_ValidIndices_rt = i_FF_po->GetValidIndices();
        if ( c_ValidIndices_rt.size_u32() == 0 )
        {
            TRACE_3( m_hTracer, "[%s] %s: Frame [%lu] No new valid feature coming from last frame; skipping...",  \
                kCameraStrings[m_CameraID].c_str(), __func__, m_availableFrameNumber );
            v_Ret_b = true;
        }else
        // more than tsc_cfg::NUM_VALID_INDICES
		// check this JJTURK 
#if 1
        if ( (c_ValidIndices_rt.size_u32() > tsc_cfg::NUM_VALID_INDICES) && (c_ValidIndices_rt.size_u32() < 2 ))
        {
            TRACE_4( m_hTracer, "[%s] %s: Frame [%lu] got more valid features than the maximum can be handled %u",  \
                kCameraStrings[m_CameraID].c_str(), __func__, m_availableFrameNumber, c_ValidIndices_rt.size_u32() );
            v_Ret_b = false;
        }
        else
#endif
        {
            // now we should have NUM_VALID_INDICES valid features and maximum NUM_INITIAL_GUESSES_PER_FRAME initial guesses to be 
            // stored in validFeatureCollection, initialGuesses and savedData
            fc::SavedRecord v_SavedRecord_o;

            for( size_t v_Index_t = 0; v_Index_t < c_ValidIndices_rt.size_u32(); ++v_Index_t )
            {
                fc::FeatureTrack& v_FT_ro =  v_LFT_ro.getTrcks_rx()[ c_ValidIndices_rt[ v_Index_t ] ];

                fc::ValidFeature v_ValidFeature_o ( v_FT_ro );

#ifdef ENABLE_SFM    // PRQA S 1070
                m_validFeatureCollection.GetVldFtrs().pushBack_v( v_ValidFeature_o );
#endif

                tscApi::DebugOverlay_s v_DebugOverlay_s;
                v_DebugOverlay_s.startPt_x = v_FT_ro.getTrckLst_rx().front_ro().getPt_t().x_x;
                v_DebugOverlay_s.startPt_y = v_FT_ro.getTrckLst_rx().front_ro().getPt_t().y_x;
                v_DebugOverlay_s.endPt_x = v_FT_ro.getTrckLst_rx().back_ro().getPt_t().x_x;
                v_DebugOverlay_s.endPt_y = v_FT_ro.getTrckLst_rx().back_ro().getPt_t().y_x;
                diagnosticData_o.getOverlays_rx().pushBack_v( v_DebugOverlay_s );
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
                diagnosticData_o.getValidFeatures_u32().pushBack_v( v_ValidFeature_o );
#endif

#ifdef ENABLE_SFM    // PRQA S 1070
                v_SavedRecord_o.AddValidFeature( v_ValidFeature_o );
#endif
            }

            const mecl::core::ArrayList< fc::InitialGuess, tsc_cfg::NUM_INITIAL_GUESSES_PER_FRAME >& c_NewInitialGuesses_rt = i_FF_po->GetInitialGuesses();

            for ( size_t v_Index_t = 0; v_Index_t < c_NewInitialGuesses_rt.size_u32(); ++v_Index_t )
            {
                initialGuesses_x.pushBack_v( c_NewInitialGuesses_rt[ v_Index_t ] );
                v_SavedRecord_o.AddInitialGuess( c_NewInitialGuesses_rt[v_Index_t] );
            }

            diagnosticData_o.addValidFeatures_v(c_ValidIndices_rt.size_u32());
            diagnosticData_o.incNumVldFrms_v();
            
            savedData_o.AddRecord( v_SavedRecord_o );
            savedData_o.getDescriptor_ro().SetRawFrame(frameCounter_u32);
            savedData_o.getDescriptor_ro().SetValidFrame(diagnosticData_o.getNumVldFrms_u32());  // PRQA S 3223
            savedData_o.getDescriptor_ro().SetValidFeatures(diagnosticData_o.getNumValidFeatures_u32());  // PRQA S 3223
            savedDataInfo_s.len_u32 = savedData_o.getValidLength_u32();

            STOP_TIMER( timer );
            TRACE_5( m_hTracer, "[%s] %s: Frame [%lu] Copied valid features into our collection with new size [%u]. Elapsed time [%.1f]ms",  \
                kCameraStrings[m_CameraID].c_str(), __func__, m_availableFrameNumber, diagnosticData_o.getNumValidFeatures_u32(), GET_ELAPSED_TIME( timer ) );
                    m_ValidFrames << availableFrameNumber_u32 << ", ";
                    m_ValidFrames << diagnosticData_o.getNumVldFrms_u32() << "\n";
            }
        }
    }
    return v_Ret_b;
}

// ----------------------------------------------------------------------------
const mecl::core::Array < uint32_t, tsc_cfg::NUM_AVAILABLE_ROIS>* fc::FeatureCollectionImpl::GetROIPerformances( sint32_t lfcNum )
{
    const mecl::core::Array <uint32_t, tsc_cfg::NUM_AVAILABLE_ROIS>* c_Performances_pt = NULL;
    if( lfcNum < tsc_cfg::MAX_ROI_COUNT )
    {
        c_Performances_pt = lfc_x[lfcNum].GetROIPerformances( );
    }

    return c_Performances_pt;
}

// ----------------------------------------------------------------------------
bool_t fc::FeatureCollectionImpl::ValidateSavedData( tscApi::TSCSavedDataInfo_s const * i_TSCSavedData_pt)
{
    bool_t v_Ret_b = false;

    if ((i_TSCSavedData_pt != NULL) && (i_TSCSavedData_pt->data_pu8 != NULL) && (i_TSCSavedData_pt->len_u32 != 0))
    {
        uint32_t v_Len_u32 = i_TSCSavedData_pt->len_u32;
        fc::TSCSavedData* v_Data_po = reinterpret_cast<fc::TSCSavedData*>(i_TSCSavedData_pt->data_pu8);
        fc::Descriptor &v_Descriptor_ro = v_Data_po->getDescriptor_ro();
    
        if ( (v_Descriptor_ro.getVer_u16() == tsc_cfg::TSC_SAVED_DATA_VER) && (v_Descriptor_ro.getCameraID_t() == cameraID_t) )
        {
            uint32_t v_RawFrames_u32 = v_Descriptor_ro.getRawFrames_u32();
            uint32_t v_ValidFrames_u32 = v_Descriptor_ro.getValidFrames_u32();
            if ( v_ValidFrames_u32 <= v_RawFrames_u32 )
            {
                uint32_t v_ExpectedLen_u32 = v_Data_po->getValidLength_u32();
                uint32_t v_ExpectedLen1_u32 = v_ExpectedLen_u32 - v_Data_po->getRecordLength_u32();  // in the case validFrames and len off by 1
                TRACE_3( m_hTracer, "ValidateSavedData() len %u, expected len %u/%u", v_Len_u32 , v_ExpectedLen_u32 , v_ExpectedLen1_u32 );
                if ( (v_Len_u32 == v_ExpectedLen_u32 ) || (v_Len_u32 == v_ExpectedLen1_u32) )
                {
                    v_Ret_b = true;
                }
            }
        }
    }
    if (v_Ret_b)
    {
        memcpy(savedDataInfo_s.data_pu8, i_TSCSavedData_pt->data_pu8, i_TSCSavedData_pt->len_u32);
        savedDataInfo_s.len_u32 = i_TSCSavedData_pt->len_u32;
    }

    return v_Ret_b;
}

// ----------------------------------------------------------------------------
bool_t fc::FeatureCollectionImpl::restoreSavedData_b()
{
    // Adjust the ArrayList size to exactly match the len size
    if (savedDataInfo_s.len_u32 < savedData_o.getValidLength_u32())
    {
        savedData_o.removeLastRecord_b();
        savedData_o.getDescriptor_ro().SetValidFrame( savedData_o.getNumOfRecord_u32() );  // PRQA S 3223
    }

    uint32_t v_LoopEnd_u32 = savedData_o.getNumOfRecord_u32();
    for (uint32_t v_Index_u32 = 0; v_Index_u32 < v_LoopEnd_u32; ++v_Index_u32)
    {
        fc::SavedRecord& v_Srs_ro = savedData_o.getSavedRecord_ro(v_Index_u32);
        // restore validFeatures
#ifdef ENABLE_SFM    // PRQA S 1070
        for (uint16_t v_InnerIndex_u16 = 0; v_InnerIndex_u16 < v_Srs_ro.getNumValidFeatures_u32(); ++v_InnerIndex_u16)
        {
            m_validFeatureCollection.GetVldFtrs().pushBack_v( v_Srs_ro.GetValidFeature(v_InnerIndex_u16) );
        }
#endif

        // restore initialGuesses
        for (uint16_t v_InnerIndex_u16 = 0; v_InnerIndex_u16 < v_Srs_ro.getNumInitialGuesses_s16(); ++v_InnerIndex_u16)
        {
            initialGuesses_x.pushBack_v( v_Srs_ro.getInitialGuess_ro(v_InnerIndex_u16) );
        }
    }
    // update counter
    frameCounter_u32 = savedData_o.getDescriptor_ro().getRawFrames_u32();
    diagnosticData_o.putNumVldFrms_v( savedData_o.getDescriptor_ro().getValidFrames_u32() );
    diagnosticData_o.restoreValidFeatures_v(savedData_o.getDescriptor_ro().getValidFeatures_u32());
    diagnosticData_o.restoreProcessedFrames_v(frameCounter_u32);

    TRACE_1( m_hTracer, "restoreSavedData_b() restored %u record(s)", savedData_o.getNumOfRecord_u32());
    return true;
}
// ----------------------------------------------------------------------------
