// ----------------------------------------------------------------------------
// --- Written by Ehsan Parvizi [20-Aug-2013]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2013
// ----------------------------------------------------------------------------
// --- FeatureCollectionImplConfig.cpp - Loading configuration method for Impl Object
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "configuration.h"
#include "featureCollection.h"
// ----------------------------------------------------------------------------
// SB using std::string;
//-------------------------------------------------------------------------

bool_t fc::FeatureCollectionImpl::loadConfiguration_b(void)
{
    bool_t v_Ret_b = true;
    // --- load from m_cfg configuration specific to Impl object
    if( !LoadCameraSpecificConfig(cameraID_t) )
    {
        TRACE_2( m_hTracer, "[%s]: Failed to load camera config parameters for [%s]", tsc_cfg::MODULE_NAME_FEATURECOLLECTION, tsc_trace::kCameraStrings[ m_CameraID ].c_str() );
        v_Ret_b = false;
    }

    const bool_t c_RetConfig_b = LoadCameraModelConfig(cameraID_t);
    if(   (true == v_Ret_b)
       && (false == c_RetConfig_b) )
    {
        TRACE_2( m_hTracer, "[%s]: Failed to load cameraModel config parameters for [%s]", tsc_cfg::MODULE_NAME_FEATURECOLLECTION, tsc_trace::kCameraStrings[ m_CameraID ].c_str() );
        v_Ret_b = false;
    }
    return v_Ret_b;

}

//-------------------------------------------------------------------------

bool_t fc::FeatureCollectionImpl::LoadCameraSpecificConfig(const tscApi::enuCameraID i_CameraID_t)
{
    bool_t v_Ret_b = true;

    tsc_cfg::fc_ConfigStrType v_FcConfigData_t;

    if(!tsc_cfg::LoadModuleConfiguration(tsc_cfg::e_FcModule, reinterpret_cast<void*>(&v_FcConfigData_t)))
    {
        v_Ret_b = false;
    }
    else
    {
        tsc_cfg::roi_ConfigStrType const * c_CameraConfigStrPtr_pt = &v_FcConfigData_t.fcCameraConfig_at[i_CameraID_t];

        // step into ROIs
        fc::ROI v_Roi_o;
        v_Roi_o.PutID(c_CameraConfigStrPtr_pt->roiID_u8);
        v_Roi_o.PutRect(c_CameraConfigStrPtr_pt->roiRect_t);

        if( (v_Roi_o.getRect_t().x_s32 < 0) || (v_Roi_o.getRect_t().y_s32 < 0) || (v_Roi_o.getRect_t().width_s32 <= 0) || (v_Roi_o.getRect_t().height_s32 <= 0) )
        {
            TRACE_0( m_hTracer, "Element [ROI] #1 has invalid [Rect] attribute values");
            v_Ret_b = false;
        }
        else
        {
            rois_x.pushBack_v( v_Roi_o );
        }
    }
    return v_Ret_b;
}


bool_t fc::FeatureCollectionImpl::LoadCameraModelConfig(const tscApi::enuCameraID i_CameraID_t)
{
    bool_t v_Ret_b = true;

#ifdef USE_SVSCM
    tscApi::cameraModelConfig_Type cameraModelConfig;
    tscApi::cameraModelConfig_Type * cameraModelConfigPtr = &cameraModelConfig;

    // load external parameters
    if (!CExtConfig::CamModelDesign_LoadFromPlatform(i_CameraID_t, cameraModelConfigPtr))
    {
        v_Ret_b = false;
    }
    else
    {
        camera_model::CameraModel* v_CameraModel_po = &m_pInfo->getMAddrCmrMdls_po()[i_CameraID_t];    // PRQA S 3706
        v_CameraModel_po->PutID(i_CameraID_t);

        if( !v_CameraModel_po->LoadIntrinsicParamConfig( cameraModelConfigPtr ) )
        {
            TRACE_1( m_hTracer, "[LoadIntrisicParamConfig] [%s]: Failed to invert K matrix", tsc_trace::kCameraStrings[i_CameraID_t].c_str() );
            TRACE_1( m_hTracer, "Failed to load Intrinsic CameraModel configuration for [%s]", tsc_trace::kCameraStrings[i_CameraID_t].c_str() );
            v_Ret_b = false;
        } else
        if( !v_CameraModel_po->LoadExtrinsicParamConfig( cameraModelConfigPtr ) )
        {
            TRACE_1( m_hTracer, "Failed to load Extrinsic CameraModel configuration for [%s]", tsc_trace::kCameraStrings[i_CameraID_t].c_str() );
            v_Ret_b = false;
        } else
        if( !v_CameraModel_po->LoadOrientationParamConfig( cameraModelConfigPtr ) )
        {
            TRACE_1( m_hTracer, "Failed to load Orientation CameraModel configuration for [%s]", tsc_trace::kCameraStrings[i_CameraID_t].c_str() );
            v_Ret_b = false;
        } else
        {
            // set the camera model to valid
            v_CameraModel_po->SetValid();
        }
    }
#else
    const tscApi::CameraModelMeclCfg_s * c_CameraModelCfg_pt = CExtConfig::GetPlatformCamModelMeclCfg(i_CameraID_t);

    camera_model::CameraModelMecl* v_CameraModel_po = &info_px->getMAddrCmrMdls_po()[i_CameraID_t];  // PRQA S 3706
    v_CameraModel_po->PutID(i_CameraID_t);
    v_CameraModel_po->LoadConfig( c_CameraModelCfg_pt );
#endif

    return v_Ret_b;
}

bool_t fc::FeatureCollectionImpl::UpdateExternalConfiguration(tscApi::enuCameraID i_CameraID_t)
{
    bool_t v_Status_b = true;

    if( LoadCameraModelConfig(cameraID_t) )
    {
        for( sint32_t v_RoiIdx_s32 = 0; v_RoiIdx_s32 < tsc_cfg::MAX_ROI_COUNT; ++v_RoiIdx_s32 )
        {
            // Update the configuration for the LFCs
            lfc_x[v_RoiIdx_s32].UpdateExternalConfiguration(i_CameraID_t);

            // Update the configuration for the FFs
            ff_x[v_RoiIdx_s32].UpdateExternalConfiguration(i_CameraID_t);
        }
    }
    else
    {
        v_Status_b = false;
    }

    return v_Status_b;
}
