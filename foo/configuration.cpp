// ----------------------------------------------------------------------------
// --- Written by Mahmoud Salem
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "configuration.h"
#include "kinematicModel.h"
#include "featureCollection.h"
#include "tscAlg.h"
// PRQA S 3708 EOF
// PRQA S 1070 EOF
// SB using std::string;

namespace tsc_trace {
// for trace only as of today
/* -------------- Camera Strings ----------------- */
// The order of the strings in the kCameraStrings must match the order
// of cameras in the enuCameraID.
#if defined (DEBUG) && defined (TRACING)
const string kCameraStrings[tsc_cfg::MAX_NUM_CAMERA_CALIB] = {"FrontCamera", "LeftCamera", "RearCamera", "RightCamera"};
const string kTSCStates[tscApi::e_TscStateEnd] = {"Uninit", "InitOK", "Error", "FeatureCollection", "FeatureCollectionCompleted",
                                              "Calibration", "CalibrationCompleted", "Terminated", "Paused", "Unknown"};
const string kErrorCodes[tscApi::e_TscErrorNum] = {"NoError", "InitFail", "StartError", "FeatureCollectionError", "CalibrationError",
                                              "InvalidParameter", "InvalidSavedData", "UnexpectedRequest"};
const string kROIIds[tsc_cfg::MAX_NUM_ROI_ID] = {"Center"};
#else
const uint32_t kCameraStrings[tsc_cfg::MAX_NUM_CAMERA_CALIB] = {0};
const uint32_t kTSCStates[tscApi::e_TscStateEnd] = {0};
const uint32_t kErrorCodes[tscApi::e_TscErrorNum] = {0};
const uint32_t kROIIds[tsc_cfg::MAX_NUM_ROI_ID] = {0};
#endif
}

/* --------------- Kinematic Model Configuration ------------- */
namespace tsc_cfg
{

/* --------------- Kinematic Model Configuration ------------- */
const km_ConfigStrType km_initConfig = {
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
        /* TRACE Info */ { 1, "KM", "Image,Message", "KM.txt" },
#endif
        /* Straight motion Distance Threshold in mm */ DISTANCE_THRESH_MM ,
        /* KM External Config */
        {
            /* TireCircumference Per Pulse in mm */ 24.48F,
            /* Distance COG To Front Axis in mm */ 1550.5F,
            /* Distance COG To Rear Axis in mm */ 1550.5F
        }
};

/* -------------- Camera Model Design ----------------------- */

/*  radialZeroCrossing: Radial distortion zero crossing in pixels
    radialCoeff1,2,3: Radial distortion coefficients
    tangentialCoeff1,2: Tangential distortion coefficients
    affineCoeff1,2: Affine distortion coefficients
    ppx_pixel,ppy_pixel: internal principal point in pixels
    axisX,axisY: camera principal point (for K matrix)
    flipped: whether the camera is flipped horizontally */

const cameraModelDesignType cameraModelDesign_initConfig = {
#ifdef USE_SVSCM
    // ---- Magna cameraModel ----
    {
    /* Front Camera */
    {
        /* Intrinsic Parameters */
        {
            /* pixelSize */ 0.003,
            /* focalLength */ 0.864,
            /* radialZeroCrossing */ 2400.0,
            /* radialCoeff1 */ -11182e-8,
            /* radialCoeff2 */ 0e-8,
            /* radialCoeff3 */ 0e-10,
            /* tangentialCoeff1 */ -89735e-8,
            /* tangentialCoeff2 */ 15339e-8,
            /* affineCoeff1 */ -417581e-8,
            /* affineCoeff2 */ 32921e-8,
            /* ppx_pixel */ 639.3,
            /* ppy_pixel */ 402.1,
            /* origX */ 1280.0,
            /* origY */ 800.0,
            /* axisX */ 639.5,
            /* axisY */ 399.5,
            /* downsampleFactor */ 2
        },
        /* Extrinsic Parameters */
        {
            /* Roll_deg */ 180.0,
            /* Yaw_deg */ 0.0,
            /* Pitch_deg */ 63.0,
            /* X_mm */ 0,
            /* Y_mm */ 0,
            /* Z_mm */ 605.0,
            /* flipped */ 1
        },
        /* Orientation Parameters */
        {
            /* deltaX */ -864.0,
            /* deltaY */ 0,
            /* preRoll_deg */ -90.0
        }
    },
    /* left Camera */
    {
        {
            /* pixelSize */ 0.003,
            /* focalLength */ 0.8814,
            /* radialZeroCrossing */ 2400.0,
            /* radialCoeff1 */ -3616e-8,
            /* radialCoeff2 */ 0e-8,
            /* radialCoeff3 */ 0e-10,
            /* tangentialCoeff1 */ -91179e-8,
            /* tangentialCoeff2 */ 114533e-8,
            /* affineCoeff1 */ -62782e-8,
            /* affineCoeff2 */ 10453e-8,
            /* ppx_pixel */ 637.3,
            /* ppy_pixel */ 404.3,
            /* origX */ 1280.0,
            /* origY */ 800.0,
            /* axisX */ 639.5,
            /* axisY */ 399.5,
            /* downsampleFactor */ 2
        },
        /* Extrinsic Parameters */
        {
            /* Roll_deg */ 0.0,
            /* Yaw_deg */ 0.0,
            /* Pitch_deg */ 12.0,
            /* X_mm */ 0,
            /* Y_mm */ 0,
            /* Z_mm */ 1137.0,
            /* flipped */ 0
        },
        /* Orientation Parameters */
        {
            /* deltaX */ 889.0,
            /* deltaY */ -1037.0,
            /* preRoll_deg */ 0
        }
    },
    /* Rear Camera */
    {
        {
            /* pixelSize */ 0.003,
            /* focalLength */ 0.8794,
            /* radialZeroCrossing */ 2400.0,
            /* radialCoeff1 */ -285e-8,
            /* radialCoeff2 */ 0e-8,
            /* radialCoeff3 */ 0e-10,
            /* tangentialCoeff1 */ 35544e-8,
            /* tangentialCoeff2 */ 82927e-8,
            /* affineCoeff1 */ 45263e-8,
            /* affineCoeff2 */ -80981e-8,
            /* ppx_pixel */ 640.7,
            /* ppy_pixel */ 402.4,
            /* origX */ 1280.0,
            /* origY */ 800.0,
            /* axisX */ 639.5,
            /* axisY */ 399.5,
            /* downsampleFactor */ 2
        },
        /* Extrinsic Parameters */
        {
            /* Roll_deg */ 0.0,
            /* Yaw_deg */ 0.0,
            /* Pitch_deg */ 42.5,
            /* X_mm */ 0,
            /* Y_mm */ 0,
            /* Z_mm */ 1036.0,
            /* flipped */ 0
        },
        /* Orientation Parameters */
        {
            /* deltaX */ 3791.0,
            /* deltaY */ -46.0,
            /* preRoll_deg */ 90.0
        }
    },
    /* Right Camera */
    {
        {
            /* pixelSize */ 0.003,
            /* focalLength */ 0.9063,
            /* radialZeroCrossing */ 2400.0,
            /* radialCoeff1 */ 29703e-8,
            /* radialCoeff2 */ 0e-8,
            /* radialCoeff3 */ 0e-10,
            /* tangentialCoeff1 */ -102833e-8,
            /* tangentialCoeff2 */ 99300e-8,
            /* affineCoeff1 */ -16318e-8,
            /* affineCoeff2 */ 113897e-8,
            /* ppx_pixel */ 638.4,
            /* ppy_pixel */ 403.2,
            /* origX */ 1280.0,
            /* origY */ 800.0,
            /* axisX */ 639.5,
            /* axisY */ 399.5,
            /* downsampleFactor */ 2
        },
        /* Extrinsic Parameters */
        {
            /* Roll_deg */ 180.0,
            /* Yaw_deg */ 0.0,
            /* Pitch_deg */ 12.0,
            /* X_mm */ 0,
            /* Y_mm */ 0,
            /* Z_mm */ 1137.0,
            /* flipped */ 1
        },
        /* Orientation Parameters */
        {
            /* deltaX */ 889.0,
            /* deltaY */ 1037.0,
            /* preRoll_deg */ 180.0
        }
    }
    }, // ---- end Magna cameraModel ----
#else
    { // ---- cameraModel Mecl
      { // front
        reinterpret_cast<mecl::model::Camera<float32_t>*>(CAMERAMODEL_MECL_OBJ_UNDEFINED), // NULL pointer
        0.0
      },
      { // left
        reinterpret_cast<mecl::model::Camera<float32_t>*>(CAMERAMODEL_MECL_OBJ_UNDEFINED), // NULL pointer
        0.0
      },
      { // rear
        reinterpret_cast<mecl::model::Camera<float32_t>*>(CAMERAMODEL_MECL_OBJ_UNDEFINED), // NULL pointer
        0.0
      },
      { // right
        reinterpret_cast<mecl::model::Camera<float32_t>*>(CAMERAMODEL_MECL_OBJ_UNDEFINED), // NULL pointer
        0.0
      }
    } // ---- end cameraModel Mecl ----
#endif
};


/*----------------- TSC Configuration --------------*/

/* Camera Specific Config */
/*
 * Calibrate => 1=yes, 0=no
 * FullCalibration => 1=Angles and Positions, 0=Angles only
 * ValidFeaturesFile => Filename for recording valid feature tracks; if empty, won't log to file
 * CalibrationResultFile => Filename for recording calibration results
 * InitialGuessFile => Filename for recording initial guess angles
*/
/* FeatureCollectionMode Offline: 0=Online from Video, 1=Load features from existing
      * file (ValidFeaturesFile attribute of each camera) */


const tsc_ConfigStrType tsc_initConfig = {
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
    /* TRACE Info */ { 1, "TSC", "Image,Message", "TSC.txt" },
#endif
    /* StructureOfMotion Config */
    {
            SOFM_LIM_X,SOFM_LIM_Y,SOFM_LIM_Z,SOFM_FUNDMATOUTLIER_THRESH
    },
    /* MaxNumValidFrames */ 100,
    /* MinNumRawFrames */ 900
};

const tscCameraCfg_T tscCamera_initConfig = {{ 0, 0, 0, 0 }};
/**************************************************************/
/* Important:                                                  */
/*   Change the maximum size of ROIs or ROIs per set in        */
/*                                           the header file !!*/
/**************************************************************/
/*------------- Feature Collection Configuration -------------*/

/* FrameSkip: possible values {0/1/2} */
/* Tracker configuration:
            TrackLength: possible values {3/4}
            SpeedRanges and FrameSkips: should have the same count */
/* ROI Set Configuration:
    Each Set contains 2 or more ROI elements (same through out the whole set)
    Cycle: Flag to whether cycle ROIs or have a fixed selection */

const fc_ConfigStrType fc_initConfig = {
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
    /* TRACE Info */ { 1, "FeatureCollection", "Image,Message","FC.txt" },
#endif
    {
    /* Front Camera */
    {
        /* ID */ 0, /* Rect */ {170,175,300,165},
        /* BMALFC */
        { /* NumROIs */ 2, /* NumPatchesPerROI */ NUM_PATCHES_PER_ROI, /* PatchSize */ PATCH_SIZE, /* RandomPatches */ USE_RANDOM_PATCHES_VAL,
          /* FrameSkip */ DETECTOR_FRAME_SKIP_VAL, /* TrackLength */ 3, /* SpeedRanges */ {5,10,15,20,25,35}, /* FrameSkips */ {2,1,1,1,0,0},
          /* SearchRadius */ 7, /* Thresh1 */ 2,/* Thresh2 */ 1, /* UseMad */ USE_MAD_VAL,
          /* ROI Sets */
          {
              /* Num of ROIPools*/ 2, /* ROIs per pool*/ 1, /* Cycle */ CYCLE_THROUGH_ROIS_VAL,
              /* ROIPools*/
              {
                  /* Set 1 */ {{/* rect */ {170,230,30,30}}},
                  /* Set 2 */ {{/* rect */ {440,230,30,30}}}
              }
          }
        },
        /* Trajectory Filter */
        { /* MinimumPixelMotion */ 5,/* SlopeDifferenceThresh */ 0.02,
          /* InitialGuess */ /* AngleThresh_deg */ 5.0, /* DeviationPercentage */ 30, /* UseCombinations */ 1 , /* CombinationsDiffThreshold_deg */ 2.0,
          /* useSfMFilter */ 0, /* maxHeightDiff_mm */ 20.0
        }
    },
    /* Left Camera */
    {
        /* ID */ 0, /* Rect */ {200,80,250,150},
        /* BMALFC */
        { /* NumROIs */ 2, /* NumPatchesPerROI */ NUM_PATCHES_PER_ROI, /* PatchSize */ PATCH_SIZE, /* RandomPatches */ USE_RANDOM_PATCHES_VAL,
          /* FrameSkip */ DETECTOR_FRAME_SKIP_VAL, /* TrackLength */ 3, /* SpeedRanges */ {5,10,15,20,25,35}, /* FrameSkips */ {3,2,2,2,0,0},
          /* SearchRadius */ 7, /* Thresh1 */ 2,/* Thresh2 */ 1, /* UseMad */ USE_MAD_VAL,
          /* ROI Sets */
          {
              /* Num of ROIPools*/ 2, /* ROIs per pool*/ 1, /* Cycle */ CYCLE_THROUGH_ROIS_VAL,
              /* ROIPools*/
              {
                  /* Set 1 */ {{/* rect */ {320,70,40,40}}},
                  /* Set 2 */ {{/* rect */ {310,180,40,40}}}
              }
          }
        },
        /* Trajectory Filter */
        { /* MinimumPixelMotion */ 5,/* SlopeDifferenceThresh */ 0.02,
          /* InitialGuess */ /* AngleThresh_deg */ 5.0, /* DeviationPercentage */ 30, /* UseCombinations */ 1 , /* CombinationsDiffThreshold_deg */ 2.0,
          /* useSfMFilter */ 0, /* maxHeightDiff_mm */ 20.0
        }
    },
    /* Rear Camera */
    {
        /* ID */ 0, /* Rect */ {170,125,300,135},
        /* BMALFC */
        { /* NumROIs */ 2, /* NumPatchesPerROI */ NUM_PATCHES_PER_ROI, /* PatchSize */ PATCH_SIZE, /* RandomPatches */ USE_RANDOM_PATCHES_VAL,
          /* FrameSkip */ DETECTOR_FRAME_SKIP_VAL, /* TrackLength */ 3, /* SpeedRanges */ {5,10,15,20,25,35}, /* FrameSkips */ {2,1,1,1,0,0},
          /* SearchRadius */ 7, /* Thresh1 */ 2,/* Thresh2 */ 1, /* UseMad */ USE_MAD_VAL,
          /* ROI Sets */
          {
              /* Num of ROIPools*/ 2, /* ROIs per pool*/ 1, /* Cycle */ CYCLE_THROUGH_ROIS_VAL,
              /* ROIPools*/
              {
                  /* Set 1 */ {{/* rect */ {170,215,30,30}}},
                  /* Set 2 */ {{/* rect */ {440,215,30,30}}}
              }
          }
        },
        /* Trajectory Filter */
        { /* MinimumPixelMotion */ 5,/* SlopeDifferenceThresh */ 0.05,
          /* InitialGuess */ /* AngleThresh_deg */ 5.0, /* DeviationPercentage */ 30, /* UseCombinations */ 1 , /* CombinationsDiffThreshold_deg */ 2.0,
          /* useSfMFilter */ 0, /* maxHeightDiff_mm */ 20.0
        }
    },
    /* Right Camera */
    {
        /* ID */ 0, /* Rect */ {240,100,250,145},
        /* BMALFC */
        { /* NumROIs */ 2, /* NumPatchesPerROI */ NUM_PATCHES_PER_ROI, /* PatchSize */ PATCH_SIZE, /* RandomPatches */ USE_RANDOM_PATCHES_VAL,
          /* FrameSkip */ DETECTOR_FRAME_SKIP_VAL, /* TrackLength */ 3, /* SpeedRanges */ {5,10,15,20,25,35}, /* FrameSkips */ {3,2,2,2,0,0},
          /* SearchRadius */ 7, /* Thresh1 */ 2,/* Thresh2 */ 1, /* UseMad */ USE_MAD_VAL,
          /* ROI Sets */
          {
              /* Num of ROIPools*/ 2, /* ROIs per pool*/ 1, /* Cycle */ CYCLE_THROUGH_ROIS_VAL,
              /* ROIPools*/
              {
                  /* Set 1 */ {{/* rect */ {250,110,40,40}}},
                  /* Set 2 */ {{/* rect */ {240,180,40,40}}}
              }
          }
        },
        /* Trajectory Filter */
        { /* MinimumPixelMotion */ 5,/* SlopeDifferenceThresh */ 0.02,
          /* InitialGuess */ /* AngleThresh_deg */ 5.0, /* DeviationPercentage */ 30, /* UseCombinations */ 1 , /* CombinationsDiffThreshold_deg */ 2.0,
          /* useSfMFilter */ 1, /* maxHeightDiff_mm */ 20.0
        }
    }
    }
};
/* ---- End of Initial Configuration values -----------*/

/* The buffer hold the external configuration values, initially set to the default values then replaced by platform if needed */
tscApi::tscPlatformExtConfigType currentPlatformExtConfig[MAX_NUM_CAMERA_CALIB] = {};


/* API to clear external configuration for a specific camera prior to being set by Platform */
bool_t clearExtPlatformConfig(tscApi::enuCameraID i_CameraID_t)
{
    bool_t v_Status_b = true;

    memset(&currentPlatformExtConfig[i_CameraID_t],0,sizeof(tscApi::tscPlatformExtConfigType));

    return v_Status_b;
}

/* APIs to check if the values set by the platform are valid */
bool_t ValidateConfiguration(tscApi::enuCameraID i_CameraID_t, const tscApi::tscPlatformExtConfigType * i_TSCConfigDataPtr_pt)
{
    bool_t v_Status_b = true;
    
    /* Fill in the validation from the Algo team */
    
    // Check speed ranges are configured in increasing order
    const tscApi::fc_extConfigType * c_FcExtCfg_pt = &(i_TSCConfigDataPtr_pt->featureColExternalConfig_t);
    const uint32_t (&c_SpeedArr_rau32)[tscApi::NUM_SPEED_RANGES] = c_FcExtCfg_pt->bmalfcExtConfig_t.speedRanges_au32;
    const uint32_t (&c_FrameSkip_rau32)[tscApi::NUM_SPEED_RANGES] = c_FcExtCfg_pt->bmalfcExtConfig_t.frameSkips_au32;
    if (c_SpeedArr_rau32[0] == 0)
    {
        v_Status_b = false;
    }
    else
    {
        for(uint8_t v_Index_u8 = 1; v_Index_u8 < tscApi::NUM_SPEED_RANGES; v_Index_u8++)
        {
            if (c_SpeedArr_rau32[v_Index_u8] == 0)
            {
                break;
            }
            if ((c_SpeedArr_rau32[v_Index_u8] < c_SpeedArr_rau32[v_Index_u8-1]) || (c_FrameSkip_rau32[v_Index_u8] > MAX_FRAME_SKIP))
            {
                v_Status_b = false;
            }
        }
    }

    // sanity check for available ROIs, up to MAX_NUM_LOCAL_ROI_POOLS(2) for now, however
    // the ROIs configured are up to EXT_CONFIG_MAX_NUM_ROIS(3)
    const tscApi::ROIRectConfig_T (&c_RoiArr_rat)[tscApi::EXT_CONFIG_MAX_NUM_ROIS] = c_FcExtCfg_pt->bmalfcExtConfig_t.rois_at;
    for (uint8_t v_Index_u8 = 0; v_Index_u8 < tsc_cfg::MAX_NUM_LOCAL_ROI_POOLS; v_Index_u8++)
    {
        tsc_math::ROIRect v_Rect_t;
        v_Rect_t.x_s32 = c_RoiArr_rat[v_Index_u8].x_s32;
        v_Rect_t.y_s32 = c_RoiArr_rat[v_Index_u8].y_s32;
        v_Rect_t.width_s32 = c_RoiArr_rat[v_Index_u8].width_s32;
        v_Rect_t.height_s32 = c_RoiArr_rat[v_Index_u8].height_s32;
        if ( !fc::FeatureCollection::getInstance_rt().getImplObject_pt(i_CameraID_t)->LFCValidateROIRect(v_Rect_t) )
        {
            v_Status_b = false;
        }
    }
    for (uint8_t v_Index_u8 = 1; v_Index_u8 < tscApi::EXT_CONFIG_MAX_NUM_ROIS; v_Index_u8++)
    {
        if ( (c_RoiArr_rat[v_Index_u8].x_s32 == c_RoiArr_rat[v_Index_u8-1].x_s32) && (c_RoiArr_rat[v_Index_u8].y_s32 == c_RoiArr_rat[v_Index_u8-1].y_s32) )
        {
            v_Status_b = false;
        }
    }

#ifndef USE_SVSCM
    // pointer to camera model Mecl instance shall not be NULL
    if ( i_TSCConfigDataPtr_pt->cameraModelExternalConfig_t.camera_px == NULL )
    {
        v_Status_b = false;
    }
#endif
    
    return v_Status_b;
}

/* Function will differ according to module, because of the source of default data in platform structure */
bool_t KM_LoadInitialConfig(km_ConfigStrType * o_ModuleConfigPtr_pt)
{
    bool_t v_Status_b = true;

    /* Load the initial configuration, if enabled Replace the external config from platform layer */
    km_ConfigStrType v_KmConfig_t = km_initConfig;

    /* Load the data retrieved to the argument pointer */
    *o_ModuleConfigPtr_pt = v_KmConfig_t;

    return v_Status_b;
}

bool_t FC_LoadInitialConfig(fc_ConfigStrType * o_ModuleConfigPtr_pt)
{
    bool_t v_Status_b = true;
    /* Load the initial configuration, if enabled Replace the external config from platform layer */
    fc_ConfigStrType v_FcConfig_t = fc_initConfig;

    /* Load the data retrieved to the argument pointer */
    *o_ModuleConfigPtr_pt = v_FcConfig_t;

    return v_Status_b;
}

bool_t CamModelDesign_LoadInitialConfig(cameraModelDesignType * o_ModuleConfigPtr_pt)
{

    bool_t v_Status_b = true;

    /* Load the initial configuration, if enabled Replace the external config from platform layer */
    cameraModelDesignType v_CamModelDesignConfig_t = cameraModelDesign_initConfig;

    /* Load the data retrieved to the argument pointer */
    *o_ModuleConfigPtr_pt = v_CamModelDesignConfig_t;

    return v_Status_b;
}

bool_t TSC_LoadInitialConfig(tsc_ConfigStrType * o_ModuleConfigPtr_pt)
{
    bool_t v_Status_b = true;

    /* Load the initial configuration, if enabled Replace the external config from platform layer */
    tsc_ConfigStrType v_TscConfig_t = tsc_initConfig;

    /* Load the data retrieved to the argument pointer */
    *o_ModuleConfigPtr_pt = v_TscConfig_t;

    return v_Status_b;
}

bool_t TSCCamera_LoadInitialConfig(tscCameraCfg_T * o_ModuleConfigPtr_pt)
{
    tscCameraCfg_T v_TscCameraConfig_t = tscCamera_initConfig;
    *o_ModuleConfigPtr_pt = v_TscCameraConfig_t;
    return true;
}

/* To be used to get the configuration from the platform buffers to the local buffers of the module -- Job_Init() */
bool_t LoadModuleConfiguration(enu_initConfigID i_ModuleID_t, void * b_ModuleConfigPtr_pv)
{
    bool_t v_Status_b = true;

    /*
     * TODO: Perform some checking on module states and buffer data integrity
     *
     */

    /* Call the local functions to format the data according to module need */
    switch(i_ModuleID_t)
    {
    case e_KmModule:
        {
        v_Status_b = tsc_cfg::KM_LoadInitialConfig( reinterpret_cast<km_ConfigStrType*>(b_ModuleConfigPtr_pv) );
        break;
        }
    case e_FcModule:
        {
        v_Status_b = tsc_cfg::FC_LoadInitialConfig( reinterpret_cast<fc_ConfigStrType*>(b_ModuleConfigPtr_pv) );
        break;
        }
    case e_CameraModelDesign:
        {
        v_Status_b = tsc_cfg::CamModelDesign_LoadInitialConfig( reinterpret_cast<cameraModelDesignType*>(b_ModuleConfigPtr_pv) );
        break;
        }
    case e_TscModule:
        {
        v_Status_b = tsc_cfg::TSC_LoadInitialConfig( reinterpret_cast<tsc_ConfigStrType*>(b_ModuleConfigPtr_pv) );
        break;
        }
    case e_TscCamera:
        {
        v_Status_b = tsc_cfg::TSCCamera_LoadInitialConfig( reinterpret_cast<tscCameraCfg_T*>(b_ModuleConfigPtr_pv) );
        break;
        }
    default:
        {
        break;
        }
    }

    return v_Status_b;
}

} // end of namespace tsc_cfg

using tsc_cfg::km_initConfig;
using tsc_cfg::cameraModelDesign_initConfig;
using tsc_cfg::fc_initConfig;
using tsc_cfg::currentPlatformExtConfig;

bool_t CExtConfig::loadPlatformDefaultConfig()
{
    bool_t v_Status_b = true;

    for (uint8_t v_Index_u8 = 0; v_Index_u8 < tsc_cfg::MAX_NUM_CAMERA_CALIB; v_Index_u8++)
    {
        tscApi::enuCameraID v_CameraID_t = static_cast<tscApi::enuCameraID>(v_Index_u8);

        currentPlatformExtConfig[v_CameraID_t].kinematicModelExternalConfig_t = km_initConfig.kmExtConfig_t;
        currentPlatformExtConfig[v_CameraID_t].cameraModelExternalConfig_t = cameraModelDesign_initConfig.cameraModelConfig_at[v_CameraID_t];
        for (uint8_t v_InnerIndex_u8 = 0; v_InnerIndex_u8 < tscApi::NUM_SPEED_RANGES; v_InnerIndex_u8++)
        {
            currentPlatformExtConfig[v_CameraID_t].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[v_InnerIndex_u8] = fc_initConfig.fcCameraConfig_at[v_CameraID_t].bmalfcConfig_t.speedRanges_au32[v_InnerIndex_u8];
            currentPlatformExtConfig[v_CameraID_t].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32[v_InnerIndex_u8] = fc_initConfig.fcCameraConfig_at[v_CameraID_t].bmalfcConfig_t.frameSkips_au32[v_InnerIndex_u8];
        }
        for (uint8_t v_InnerIndex_u8 = 0; v_InnerIndex_u8 < tsc_cfg::MAX_NUM_LOCAL_ROI_POOLS; v_InnerIndex_u8++)
        {
            memcpy(&currentPlatformExtConfig[v_CameraID_t].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[v_InnerIndex_u8],
                   &fc_initConfig.fcCameraConfig_at[v_CameraID_t].bmalfcConfig_t.bmalfcROISetConfig_t.localROIPoolsConfig_at[v_InnerIndex_u8].rect_at[0], sizeof(tsc_math::ROIRect));
        }
        currentPlatformExtConfig[v_CameraID_t].featureColExternalConfig_t.trajectoryFilterConfig_t = fc_initConfig.fcCameraConfig_at[v_CameraID_t].trajectoryFilterConfig_t;
    }

    return v_Status_b;
}

#ifdef USE_SVSCM
bool_t CExtConfig::CamModelDesign_LoadFromPlatform(tscApi::enuCameraID cameraID, tscApi::cameraModelConfig_Type * CamModelConfigPtr)
{
    bool_t v_Status_b = true;

    memcpy(CamModelConfigPtr,&currentPlatformExtConfig[cameraID].cameraModelExternalConfig_t,sizeof(tscApi::cameraModelConfig_Type));

    return v_Status_b;
}
#else
const tscApi::CameraModelMeclCfg_s * CExtConfig::GetPlatformCamModelMeclCfg(tscApi::enuCameraID i_CameraID_t)
{
    return &currentPlatformExtConfig[i_CameraID_t].cameraModelExternalConfig_t;
}
#endif

bool_t CExtConfig::FC_LoadFromPlatform(tscApi::enuCameraID i_CameraID_t, tsc_cfg::roi_ConfigStrType * o_FcROIConfigPtr_pt)
{
    bool_t v_Status_b = true;

    memcpy(&o_FcROIConfigPtr_pt->bmalfcConfig_t.speedRanges_au32, &currentPlatformExtConfig[i_CameraID_t].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32, sizeof(uint32_t)*tscApi::NUM_SPEED_RANGES);
    memcpy(&o_FcROIConfigPtr_pt->bmalfcConfig_t.frameSkips_au32, &currentPlatformExtConfig[i_CameraID_t].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32, sizeof(uint32_t)*tscApi::NUM_SPEED_RANGES);
    for (uint8_t v_Index_u8 = 0; v_Index_u8 < tsc_cfg::MAX_NUM_LOCAL_ROI_POOLS; v_Index_u8++ )
    {
        memcpy(&o_FcROIConfigPtr_pt->bmalfcConfig_t.bmalfcROISetConfig_t.localROIPoolsConfig_at[v_Index_u8].rect_at[0],
               &currentPlatformExtConfig[i_CameraID_t].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[v_Index_u8], sizeof(tsc_math::ROIRect));
    }
    memcpy(&o_FcROIConfigPtr_pt->trajectoryFilterConfig_t,&currentPlatformExtConfig[i_CameraID_t].featureColExternalConfig_t.trajectoryFilterConfig_t,sizeof(tscApi::TrajectoryFilter_ConfigStrType));

    return v_Status_b;
}

bool_t CExtConfig::KM_LoadFromPlatform(tscApi::enuCameraID i_CameraID_t, tscApi::km_ExtConfigStrType * o_KmExtConfigPtr_pt)
{
    bool_t v_Status_b = true;

    memcpy(o_KmExtConfigPtr_pt,&currentPlatformExtConfig[i_CameraID_t].kinematicModelExternalConfig_t,sizeof(tscApi::km_ExtConfigStrType));

    return v_Status_b;
}

namespace tsc_trace {
/* Create from the existed code for more readable LoadConfiguration() */
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
uint64_t handleTracerConfig(const tsc_cfg::tracer_infoType * tracerInfoPtr)
{

// --- create the tracer and start logging
    char_t sLogFile[ _MAX_PATH ];
    strcpy( sLogFile, tracerInfoPtr->sLogFile);
    if( stricmp( sLogFile, "NULL" ) == 0 )
    {
        strcpy( sLogFile, "" );
    }
    char_t sWindow[ 64 ];
    memset( sWindow, 0, sizeof( sWindow ) );

    strcpy ( sWindow, tracerInfoPtr->sWindow);
    if( stricmp( sWindow, "NULL" ) == 0 )
    {
        memset( sWindow, 0, sizeof( sWindow ) );
    }
    bool_t traceMsg = false;
    bool_t traceImg = false;

    char_t sType[ 64 ];
    strcpy ( sType, tracerInfoPtr->sType);
    char_t *pSeparator = ",";
    char_t *pToken = strtok( sType, pSeparator );

    while( pToken )
    {
        if( stricmp( pToken, "Image" ) == 0 )
        {
            traceImg = true;
        }
        else if( stricmp( pToken, "Message" ) == 0 )
        {
            traceMsg = true;
        }
        else
        {
        }

        pToken = strtok( NULL, pSeparator );
    }
    uint64_t hTracer = CreateTracer( sWindow, ( ( strlen( sLogFile ) > 0 ) ? sLogFile : NULL ), traceImg, traceMsg );
    char_t sdate[ 32 ];
    char_t scurrtime[ 32 ];
    TRACE_2( hTracer, "Tracelog started:- [%s %s]", _strdate( sdate ), _strtime( scurrtime ) );

    return hTracer;
}
#endif
}
// ----------------------------------------------------------------------------
