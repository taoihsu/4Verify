// ----------------------------------------------------------------------------
// --- Written by Ehsan Parvizi [17-Jul-2014]
// --- Modified by Ehsan Parvizi [09-Oct-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
// --- Config.cpp - Loading configuration method
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "configuration.h"
#include "featureCollection.h"
#include "localFeatureCollector.h"
// ----------------------------------------------------------------------------
// PRQA S 3063 EOF
// PRQA S 1051 1 // This commented code is uncommented when running on PC.
// using std::string;
//-------------------------------------------------------------------------

bool_t lfc::LocalFeatureCollector::loadConfiguration_b()
{
    bool_t v_Ret_b = true;
    tsc_cfg::fc_ConfigStrType v_FcConfigData_t;

    if(!tsc_cfg::LoadModuleConfiguration(tsc_cfg::e_FcModule, reinterpret_cast<void*>(&v_FcConfigData_t)))
    {
        v_Ret_b = false;
    }
    else
    {
        /* Check for the camera ID */
        tsc_cfg::roi_ConfigStrType const * c_CameraConfigStrPtr_pt = &v_FcConfigData_t.fcCameraConfig_at[cameraID_t];

        // --- Only single local feature collector
        tsc_cfg::BMALFC_ConfigStrType const * c_BMALfcConfigStrPtr_pt = &c_CameraConfigStrPtr_pt->bmalfcConfig_t;

        params_o.putNmPckdROIs_v(c_BMALfcConfigStrPtr_pt->numROIs_u32);
        params_o.putNmPtchsPrROI_v(c_BMALfcConfigStrPtr_pt->numPatchesPerROI_u32);

        uint8_t v_Size_u8;
        v_Size_u8 = c_BMALfcConfigStrPtr_pt->patchSize_u32;
        params_o.putPtchSz_v(v_Size_u8, v_Size_u8);

        patchRadius_u32 = static_cast< uint8_t >( v_Size_u8 / 2 );

        params_o.putRndmPtchs_v(c_BMALfcConfigStrPtr_pt->randomPatches_b);

        params_o.putDtctnFrmSkp_v(c_BMALfcConfigStrPtr_pt->frameSkip_u32);

        params_o.putTrckLngth_v(c_BMALfcConfigStrPtr_pt->trackLength_u32);

        params_o.putSrchRds_v(c_BMALfcConfigStrPtr_pt->searchRadius_u32);

        params_o.putTrckThrsh1_v(c_BMALfcConfigStrPtr_pt->thresh1_u32);

        params_o.putTrckThrsh2_v(c_BMALfcConfigStrPtr_pt->thresh2_u32);

        params_o.putTrckUsngMAD_v(c_BMALfcConfigStrPtr_pt->useMAD_b);

        params_o.putNmROIPls_v(c_BMALfcConfigStrPtr_pt->bmalfcROISetConfig_t.numROIPools_u32);

        params_o.putNmROIsPrPl_v(c_BMALfcConfigStrPtr_pt->bmalfcROISetConfig_t.numOfROIsPerPool_u32); //ROIsPerPool ?

        params_o.putCyclThrghROIs_v(c_BMALfcConfigStrPtr_pt->bmalfcROISetConfig_t.cycle_b);
 
       if(!UpdateExternalConfiguration(cameraID_t))
        {
            v_Ret_b = false;
        }
    }
    return v_Ret_b;
}


bool_t lfc::LocalFeatureCollector::UpdateExternalConfiguration(tscApi::enuCameraID i_CameraID_t)
{
    bool_t v_Ret_b = true;

    tsc_cfg::roi_ConfigStrType v_FcCameraConfigStr_t;

    v_Ret_b = CExtConfig::FC_LoadFromPlatform(i_CameraID_t,&v_FcCameraConfigStr_t);

    params_o.getspdRngs_rx().clear_v();

    for(uint8_t v_Idx_u8=0; v_Idx_u8 < tscApi::NUM_SPEED_RANGES; v_Idx_u8++)
    {
        if(v_FcCameraConfigStr_t.bmalfcConfig_t.speedRanges_au32[v_Idx_u8] != 0)
        {
            params_o.getspdRngs_rx().pushBack_v( v_FcCameraConfigStr_t.bmalfcConfig_t.speedRanges_au32[v_Idx_u8] );
        }
        else
        {
            break; // Array Terminator detected
        }
    }

    params_o.gettrckngFrmSkps_rx().clear_v();

    // Only load the frame skips for the corresponding non-zero speed ranges
    { // Restrict scope of Endlp
      uint8_t v_Endlp_u8 = params_o.getspdRngs_rx().size_u32();
      for(uint8_t v_Idx_u8=0; v_Idx_u8 < v_Endlp_u8; v_Idx_u8++)
      {
        if ( v_FcCameraConfigStr_t.bmalfcConfig_t.frameSkips_au32[v_Idx_u8] > tsc_cfg::MAX_FRAME_SKIP )
        {
            TRACE_0( m_hTracer, "LoadConfiguration: Frame Skips exceed limitation." );
            v_Ret_b = false;
        }
        else
        {
            params_o.gettrckngFrmSkps_rx().pushBack_v( v_FcCameraConfigStr_t.bmalfcConfig_t.frameSkips_au32[v_Idx_u8] );
        }
      }
    }

    // Check speed ranges are configured in increasing order
    uint8_t v_MaxIdx_u8 = params_o.getspdRngs_rx().size_u32();
    for(uint8_t v_Idx_u8=1; v_Idx_u8 < v_MaxIdx_u8; v_Idx_u8++)
    {
        uint32_t v_Speed_u32 = params_o.getspdRngs_rx()[v_Idx_u8];
        uint32_t v_PrevSpeed_u32 = params_o.getspdRngs_rx()[v_Idx_u8-1];
        if(v_Speed_u32 < v_PrevSpeed_u32)
        {
            TRACE_2( m_hTracer, "LoadConfiguration: Speed Ranges not in increasing order, (%d, %d)", v_Speed_u32, v_PrevSpeed_u32 );
            v_Ret_b = false;
        }
    }

    size_t v_CurrIndex_t = 0;
    uint32_t v_FoundRoiPools_u32 = 0;

    // Loop through Pool children
    uint32_t v_Endwh_u32 = params_o.getNmROIPls_u32();
    while(v_FoundRoiPools_u32 < v_Endwh_u32)
    {
        // Read in each pool's ROIs
        uint32_t v_FoundRoiPerPool_u32 = 0;
        uint32_t v_Endlp_u32 = params_o.getNmROIsPrPl_u32();
        while( v_FoundRoiPerPool_u32 < v_Endlp_u32 )
        {
            params_o.getavlblROIs_rx()[ v_CurrIndex_t ] = v_FcCameraConfigStr_t.bmalfcConfig_t.bmalfcROISetConfig_t.localROIPoolsConfig_at[v_FoundRoiPools_u32].rect_at[v_FoundRoiPerPool_u32];

            tsc_math::ROIRect& v_Rect_rt = params_o.getavlblROIs_rx()[ v_CurrIndex_t ];
            if( !ValidateROIRect(v_Rect_rt) )
            {
                OC_DEBUG_PRINTF(("Element [ROI] #%u has invalid [Rect] attribute values", ( v_FoundRoiPerPool_u32 + 1 ) ));
                v_Ret_b = false;
            }
            else
            {
                ++v_CurrIndex_t;
                ++v_FoundRoiPerPool_u32;
            }
            if( !v_Ret_b )
            {
                break;
            }
        }

        if( v_Ret_b )
        {
            if( v_FoundRoiPerPool_u32 != params_o.getNmROIsPrPl_u32() )
            {
                OC_DEBUG_PRINTF(("Element [Pool] #%u has unexpected number of ROIs{%d}. Expected{%d}",  \
                                     v_FoundRoiPools_u32 + 1, v_FoundRoiPerPool_u32, params_o.getNmROIsPrPl_u32() ));
                v_Ret_b = false;
            }
            else
            {
                ++v_FoundRoiPools_u32;
            }
        }
        else
        {
            break;
        }
    }

    if( v_FoundRoiPools_u32 != params_o.getNmROIPls_u32() )
    {
        OC_DEBUG_PRINTF(("Element [ROIPools] has unexpected number of Pool{%d}. Expected{%d}", v_FoundRoiPools_u32, params_o.getNmROIPls_u32() ));
        v_Ret_b = false;
    }

    if( params_o.getNmPckdROIs_u32() > params_o.getNmROIPls_u32() )
    {
        OC_DEBUG_PRINTF(( "Invalid config parameter: NumROIs [%u] cannot be greater than total number of available ROI Pools [%u]",  \
                    params_o.getNmPckdROIs_u32(), params_o.getNmROIPls_u32() ));
        v_Ret_b = false;
    }

    return v_Ret_b;
}

bool_t lfc::LocalFeatureCollector::ValidateROIRect(const tsc_math::ROIRect& i_Rect_rt) const
{
    bool_t v_Ret_b = true;
    if ( (frameSize_t.width_s32 == 0) || (frameSize_t.height_s32 == 0) )
    {
        v_Ret_b = false;  // LFC not initialized yet
    }
#if 0
    if ( (i_Rect_rt.x_s32 < 0) || (i_Rect_rt.y_s32 < 0) || (i_Rect_rt.width_s32 <= 0) || (i_Rect_rt.height_s32 <= 0) ||
         ((i_Rect_rt.x_s32 + i_Rect_rt.width_s32) > frameSize_t.width_s32) || ((i_Rect_rt.y_s32 + i_Rect_rt.height_s32) > frameSize_t.height_s32) )
    {
        v_Ret_b = false;
    }
#endif
    return v_Ret_b;
}
//-------------------------------------------------------------------------
