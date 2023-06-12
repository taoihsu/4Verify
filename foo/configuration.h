// ----------------------------------------------------------------------------
// --- Written by Ehsan Parvizi [17-Dec-2013]
// --- Modified by Ehsan Parvizi [26-Aug-2014]
// --- Modified by Hany Kashif [24-Sep-2014]
// --- Modified by Mahmoud Salem [03-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __CONFIGURATION_H_
#define __CONFIGURATION_H_
// ----------------------------------------------------------------------------
// PRQA S 2641 EOF
// PRQA S 2642 EOF
// PRQA S 1070 EOF
#include "tscApi.h"
#include "mathOperations.h"

namespace tsc_cfg
{
const sint8_t MAX_NUM_CAMERA_CALIB = 4;
const sint8_t MAX_NUM_ROI_ID = 1;
const uint32_t TSC_SAVED_DATA_VER = 1;
}
namespace tsc_trace {
#if defined (DEBUG) && defined (TRACING)
extern const std::string kCameraStrings[tsc_cfg::MAX_NUM_CAMERA_CALIB];
extern const std::string kTSCStates[tscApi::e_TscStateEnd];
extern const std::string kErrorCodes[tscApi::e_TscErrorNum]; 
extern const std::string kROIIds[tsc_cfg::MAX_NUM_ROI_ID];
#else
extern const uint32_t kCameraStrings[tsc_cfg::MAX_NUM_CAMERA_CALIB];
extern const uint32_t kTSCStates[tscApi::e_TscStateEnd];
extern const uint32_t kErrorCodes[tscApi::e_TscErrorNum]; 
extern const uint32_t kROIIds[tsc_cfg::MAX_NUM_ROI_ID];
#endif
}

namespace tsc_cfg
{
/* Enum used to load configuration from the const structures */
typedef enum{
    e_KmModule,
    e_CameraModelDesign,
    e_TscModule,
    e_FcModule,
    e_TscCamera
} enu_initConfigID;
/* Defines required for the size of arrays created */
const sint8_t MAX_NUM_LOCAL_ROI_POOLS = 2;
const sint8_t MAX_NUM_ROIS_PER_POOL = 1;

/* Constant values across configuration */
// --- BMALFC Constant Configuration

const sint8_t NUM_PATCHES_PER_ROI = 2;
// --- Kinematic Model Constant Configuration
const sint8_t PATCH_SIZE = 16;

const sint8_t USE_RANDOM_PATCHES_VAL = 1;
// --- TSC SofM Constant Configuration
const sint8_t DETECTOR_FRAME_SKIP_VAL = 0;
const sint8_t USE_MAD_VAL = 0; // 0-CrossCorrelation, 1-SAD
const sint8_t CYCLE_THROUGH_ROIS_VAL = 0;

// --- Kinematic Model Constant Configuration
const float32_t DISTANCE_THRESH_MM = 5.0F;

// --- TSC SofM Constant Configuration
const float32_t SOFM_LIM_X = 100000.0F;
const float32_t SOFM_LIM_Y = 100000.0F;
const float32_t SOFM_LIM_Z = 200000.0F;
const float32_t SOFM_FUNDMATOUTLIER_THRESH  = 1000.02F;

// --- Static limits for arrays and array lists
// Make sure that any configuration change is reflected in these #defines
const sint8_t TRACK_LENGTH = 3;
const sint8_t NUM_FILTERED_INDICES = 4;      //First and last indices

// NUM_SPEED_RANGES = 6, is currently defined in controller/tscApi.h
const sint8_t MAX_FRAME_SKIP = 6;
const sint8_t MAX_ROI_COUNT = 1;		// [OBSOLETE] Max number of ROI configurations for feature collection per camera.
const sint8_t MAX_NUM_PICKED_ROIS = 2;	// Max Number of ROIs can be picked by the LFC from ROIs pool for feature detection
const sint8_t NUM_VALID_FRAMES = 100;
const uint16_t MIN_NUM_RAW_FRAMES = 900;
const sint8_t NUM_AVAILABLE_ROIS = 6;  //m_params.numAvailableROISets * m_params.numROIsPerSet. Max from current config = 4
const sint16_t NUM_DESCRIPTOR_LIST = (PATCH_SIZE*PATCH_SIZE);
const sint16_t NUM_VALID_INDICES = (MAX_NUM_PICKED_ROIS*NUM_PATCHES_PER_ROI);
const sint16_t NUM_VALID_FEATURES = (MIN_NUM_RAW_FRAMES*NUM_VALID_INDICES);
const sint16_t NUM_TOTAL_IMAGE_POINTS = (NUM_VALID_FEATURES)*2;
const sint16_t NUM_INITIAL_GUESSES_PER_FRAME = (NUM_VALID_INDICES*(NUM_VALID_INDICES-1)*TRACK_LENGTH)/2;
//const sint16_t NUM_INITIAL_GUESSES = (NUM_INITIAL_GUESSES_PER_FRAME*MIN_NUM_RAW_FRAMES)/2;
const sint16_t NUM_INITIAL_GUESSES = 10 + ((NUM_INITIAL_GUESSES_PER_FRAME * NUM_VALID_FRAMES*2) / 2);
const sint16_t NUM_TRACKS = ((NUM_VALID_INDICES)*(1+(TRACK_LENGTH-1)*(MAX_FRAME_SKIP+1)));
// TODO: temporary increase the KM Impl Obj ArrayList to see if the assertion still exist
const sint16_t NUM_KM_IMPL_OBJS = NUM_TRACKS*2;

// the default pointer for cameraModel Mecl default configuration
const uint32_t CAMERAMODEL_MECL_OBJ_UNDEFINED = 0xFFFFFFFFU;

/* Module Names */
const char_t MODULE_NAME_KINEMATICMODEL[] = "KinematicModel";
const char_t MODULE_NAME_FEATURECOLLECTION[] = "FeatureCollection";
const char_t MODULE_NAME_LOCALFEATURECOLLECTOR[] = "BMALFC";
const char_t MODULE_NAME_FEATUREFILTER[] = "FCTrajectoryFilter";
const char_t MODULE_NAME_SFM[] = "TSCSfM";
const char_t MODULE_NAME_TSC[] = "TSC";

// ----------------------------------------------------------------------------
/* Structure used to configure the log tracing */
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
typedef struct{
	bool_t tracerEnable;
	char_t sWindow[64];
	char_t sType[64];
	char_t sLogFile[260];
}tracer_infoType;
#endif

//---------------------------------------------------------
/* Structures used to hold configuration for TSC modules */

/* Structure for Kinematic Model Configuration */
typedef struct {
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
    tracer_infoType tracerInfo;
#endif
    float64_t straightMotionDistanceThreshMM_f64;
    tscApi::km_ExtConfigStrType kmExtConfig_t;
} km_ConfigStrType;

/* Structures for TSC Module Configuration */

/* Sub-Structure for SofM Configuration */
typedef struct {
    float64_t limitXMM_f64;
    float64_t limitYMM_f64;
    float64_t limitZMM_f64;
    float64_t fundMatOutlierThresh_f64;
}SofM_ConfigStrType;

/* Structure for TSC Module */
typedef struct {
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
    tracer_infoType tracerInfo;
#endif
    SofM_ConfigStrType sofMConfig_t;
    uint32_t maxNumValidFrames_u32;
    uint32_t minNumRawFrames_u32;
}tsc_ConfigStrType;

typedef struct {
    bool_t fullCalibration_ab[MAX_NUM_CAMERA_CALIB];
}tscCameraCfg_T;

/* Structures for FC Module Configuration */ 
/* Sub-Structure for the localROI configurations */
typedef struct{
    tsc_math::ROIRect rect_at[tsc_cfg::MAX_NUM_ROIS_PER_POOL];
}localROIPools_ConfigStrType;

/* Sub-Structure for LFC ROI set Configuration */
typedef struct{
    uint32_t numROIPools_u32;
    uint32_t numOfROIsPerPool_u32;
    bool_t cycle_b;
    localROIPools_ConfigStrType localROIPoolsConfig_at[tsc_cfg::MAX_NUM_LOCAL_ROI_POOLS];
}BMALFC_ROISet_ConfigStrType;

/* Sub-Structure for LocalFeatureCollector Configuration */
typedef struct{
    uint32_t numROIs_u32;
    uint32_t numPatchesPerROI_u32;
    uint32_t patchSize_u32;
    bool_t randomPatches_b;
    uint32_t frameSkip_u32;
    uint32_t trackLength_u32;
    uint32_t speedRanges_au32[tscApi::NUM_SPEED_RANGES];
    uint32_t frameSkips_au32[tscApi::NUM_SPEED_RANGES];
    uint32_t searchRadius_u32;
    uint32_t thresh1_u32;
    uint32_t thresh2_u32;
    bool_t useMAD_b;
    BMALFC_ROISet_ConfigStrType bmalfcROISetConfig_t;
}BMALFC_ConfigStrType;

/* Sub-Structure for ROI Configuration */
typedef struct{
    uint8_t roiID_u8;
    tsc_math::ROIRect roiRect_t;
    BMALFC_ConfigStrType bmalfcConfig_t;
    tscApi::TrajectoryFilter_ConfigStrType trajectoryFilterConfig_t;
}roi_ConfigStrType;

/* Structure for Feature Collection configuration */
typedef struct {
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
    tracer_infoType tracerInfo;
#endif
    roi_ConfigStrType fcCameraConfig_at[MAX_NUM_CAMERA_CALIB];
}fc_ConfigStrType;

/* Structure for CameraDesign Configuration */
typedef struct {
#ifdef USE_SVSCM
  tscApi::cameraModelConfig_Type cameraModelConfig_at[MAX_NUM_CAMERA_CALIB];
#else
  tscApi::CameraModelMeclCfg_s cameraModelConfig_at[MAX_NUM_CAMERA_CALIB];
#endif
} cameraModelDesignType;

} // End of tsc_cfg namespace
namespace tsc_trace {
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
uint64_t handleTracerConfig(const tsc_cfg::tracer_infoType * tracerInfoPtr);
#endif
}

namespace tsc_cfg 
{
    extern tscApi::tscPlatformExtConfigType currentPlatformExtConfig[MAX_NUM_CAMERA_CALIB];
    bool_t KM_LoadInitialConfig(km_ConfigStrType * o_ModuleConfigPtr_pt);
    bool_t FC_LoadInitialConfig(fc_ConfigStrType * o_ModuleConfigPtr_pt);
    bool_t CamModelDesign_LoadInitialConfig(cameraModelDesignType * o_ModuleConfigPtr_pt);
    bool_t TSC_LoadInitialConfig(tsc_ConfigStrType * o_ModuleConfigPtr_pt);
    bool_t TSCCamera_LoadInitialConfig(tscCameraCfg_T * o_ModuleConfigPtr_pt);
    bool_t ValidateConfiguration(tscApi::enuCameraID i_CameraID_t, const tscApi::tscPlatformExtConfigType * i_TSCConfigDataPtr_pt);
    bool_t clearExtPlatformConfig(tscApi::enuCameraID i_CameraID_t);
    bool_t LoadModuleConfiguration(enu_initConfigID i_ModuleID_t, void * b_ModuleConfigPtr_pv);
}

namespace CExtConfig {

    /* Initially set the external config to hard coded values */
    bool_t loadPlatformDefaultConfig();

    /* Translation layer to transform data from platform buffer to the internal structure format */
    bool_t KM_LoadFromPlatform(tscApi::enuCameraID i_CameraID_t, tscApi::km_ExtConfigStrType * o_KmExtConfigPtr_pt);
    bool_t FC_LoadFromPlatform(tscApi::enuCameraID i_CameraID_t, tsc_cfg::roi_ConfigStrType * o_FcROIConfigPtr_pt);
#ifdef USE_SVSCM
    bool_t CamModelDesign_LoadFromPlatform(tscApi::enuCameraID cameraID, tscApi::cameraModelConfig_Type * CamModelConfigPtr);
#else
    const tscApi::CameraModelMeclCfg_s * GetPlatformCamModelMeclCfg(tscApi::enuCameraID i_CameraID_t);
#endif

}
#endif

//--------------------------------------------------------------------------------------

