// ----------------------------------------------------------------------------
// --- Written by Hany Kashif
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __TSCAPI_H_
#define __TSCAPI_H_
// ----------------------------------------------------------------------------
#include "mecl/mecl.h"
// ----------------------------------------------------------------------------
// PRQA S 1070 EOF // PRQA S 1051 3 //Uncommented when debugging.
//#ifndef DEBUG_TSC_ALG
//#define DEBUG_TSC_ALG
//#endif

// This commented code is uncommented and used when debugging with log_prinfs ON.
// PRQA S 1051 1
//#define OC_LOG_PRINTF_ON

#ifdef OC_LOG_PRINTF_ON
    #define OC_DEBUG_PRINTF(x) log_printf x
#else
//PRQA S 1020 1 //Macro used for debugging
#define OC_DEBUG_PRINTF(x)
#endif

namespace tscApi
{
const sint8_t NUM_SPEED_RANGES = 6;  // --- Maximum Number of Speed Ranges
const uint8_t EXT_CONFIG_MAX_NUM_ROIS = 3;      // --- Maximum Number of ROIS

typedef enum {
    e_TscFrontCam = 0,
    e_TscLeftCam,
    e_TscRearCam,
    e_TscRightCam,
    e_TscNumCam
} enuCameraID;

#ifdef USE_SVSCM
typedef struct{

    struct{
        float64_t pixelSize;
        float64_t focalLength;
        float64_t radialZeroCrossing;
        float64_t radialCoeff1; // --- Radial distortion coefficients
        float64_t radialCoeff2; // --- Radial distortion coefficients
        float64_t radialCoeff3; // --- Radial distortion coefficients
        float64_t tangentialCoeff1; // --- Tangential distortion coefficients
        float64_t tangentialCoeff2; // --- Tangential distortion coefficients
        float64_t affineCoeff1; // --- Affine distortion coefficients
        float64_t affineCoeff2; // --- Affine distortion coefficients
        float64_t ppx; // --- internal principal point
        float64_t ppy; // --- internal principal point
        float64_t origX;
        float64_t origY;
        float64_t axisX;
        float64_t axisY;
        uint32_t downsampleFactor;
    } intrinsicParams;

    struct{
        float64_t rollDeg_f64;  // --- Algo assumption: upside-up, shall be set in 1st or 4th quadrants
                             // ---                  unside-down, shall be set in 2nd or 3rd quadrants 
        float64_t yawDeg_f64;
        float64_t pitchDeg_f64;
        float64_t xMM_f64;
        float64_t yMM_f64;
        float64_t zMM_f64;
        bool_t flipped; // --- should be set to 0, to be removed later 
    } extrinsicParams;

    struct{
        float64_t deltaX;
        float64_t deltaY;
        float64_t preRoll_deg;
    } orientationParams;

} cameraModelConfig_Type;
#else
typedef struct{
    mecl::model::Camera<float32_t> *camera_px;  // pointer to the camera object
    float32_t downSampleFactor_f32;
} CameraModelMeclCfg_s;
#endif

typedef struct{
    uint32_t minPixelMotionThresh_u32;
    float64_t slopeDifferenceThreshold_f64;
    float64_t angleThresholdDegIG_f64;
    uint32_t deviationPercentageIG_u32;
    bool_t useCombinations_b;
    float64_t combinationsDiffThresholdDeg_f64;
    bool_t useSfmFilter_b;
    float64_t maxHeightDiffMm_f64;
}TrajectoryFilter_ConfigStrType;

typedef struct{
    sint32_t x_s32;
    sint32_t y_s32;
    sint32_t width_s32;
    sint32_t height_s32;
} ROIRectConfig_T;

typedef struct{
    uint32_t speedRanges_au32[NUM_SPEED_RANGES]; // --- Array of non-zero speed Ranges (i.e. Zero Value indicate end of Array)
    uint32_t frameSkips_au32[NUM_SPEED_RANGES]; // --- Array of corresponding frame Skips, can't have elements more than non-zero speedRanges
    ROIRectConfig_T rois_at[EXT_CONFIG_MAX_NUM_ROIS];
}BMALFC_extConfigStrType;

typedef struct {
    BMALFC_extConfigStrType bmalfcExtConfig_t;
    TrajectoryFilter_ConfigStrType trajectoryFilterConfig_t;
}fc_extConfigType;

typedef struct {
    float32_t tireCircumferencePerPulseMM_f32;
    float64_t distanceCoG2FrontAxisMM_f64;
    float64_t distanceCoG2RearAxisMM_f64;
    float32_t distanceThreshMM;
} km_ExtConfigStrType;


/* The type to be used with the buffer passed to TSC_Start() */
typedef struct {
    km_ExtConfigStrType kinematicModelExternalConfig_t;
#ifdef USE_SVSCM
    cameraModelConfig_Type cameraModelExternalConfig_t;
#else
    CameraModelMeclCfg_s cameraModelExternalConfig_t;
#endif

    float32_t trimMeanPercentage_f32;
    fc_extConfigType featureColExternalConfig_t;
    int32_t MaxNumValidFrames_i32;
    int32_t MinNoRawFrames_i32;
} tscPlatformExtConfigType;

// ----------------------------------------------------------------------------
struct TSCSavedDataInfo_s
{
    uint8_t* data_pu8;
    uint32_t len_u32;
};

/***** TSC Control Info  ******/
class TSCCtrlInfo
{
public:
    TSCCtrlInfo() :
        speed_f32( 0.0f ),
        wheelAngle_f32( 0.0f ),
        hitchAngle_f32( 0.0f ),
        gearDirection_s32( e_GearNeutral ),
        cameraFrontOpen_b( true ),
        frameNumber_u32( 0 ),
        videoWidth_u16( 0 ),
        videoHeight_u16( 0 )
    {
        for (uint8_t v_Index_u8 = 0; v_Index_u8 < static_cast<uint8_t>(e_TscNumCam); v_Index_u8++)
        {
            c_Cameras_apu8[v_Index_u8] = NULL;
            savedDataInfo_aps[v_Index_u8] = NULL;
        }
    }
        tscApi::enuCameraID tscTargetCamera;

    const uint8_t *GetM_Cmrs(enuCameraID i_Param_t) const { return c_Cameras_apu8[i_Param_t]; }  // PRQA S 4628
    const uint8_t **GetMAddr_Cmrs(enuCameraID i_Param_t) { return &c_Cameras_apu8[i_Param_t]; }
    float32_t getMSpd_f32(void) const { return speed_f32; }
    float32_t getMWhlAngl_f32(void) const { return wheelAngle_f32; }
    float32_t getMHtchAngl_f32( void ) const { return hitchAngle_f32; }
    sint32_t getMGrDrctn_s32(void) const { return gearDirection_s32; }
    bool_t getMCmrFrntOpn_b(void) const { return cameraFrontOpen_b; }
    uint32_t getMFrmNmbr_u32(void) const { return frameNumber_u32; }
    uint16_t getMVdWdth_u16(void) const { return videoWidth_u16; }
    uint16_t getMVdHght_u16(void) const { return videoHeight_u16; }
    TSCSavedDataInfo_s *getMPSvdDtInf_ps(uint8_t i_Param_u8) const { return savedDataInfo_aps[i_Param_u8]; }  // PRQA S 4628

    void PutM_Cmrs(enuCameraID i_ID_t, const uint8_t *i_Param_pu8) { c_Cameras_apu8[i_ID_t] = i_Param_pu8; }
    void PutM_Spd(float32_t i_Param_f32) { speed_f32 = i_Param_f32; }
    void PutM_WhlAngl(float32_t i_Param_f32) { wheelAngle_f32 = i_Param_f32; }
    void PutM_HtchAngl( float32_t i_Param_f32 ) { hitchAngle_f32 = i_Param_f32; }
    void PutM_GrDrctn(sint32_t i_Param_s32) { gearDirection_s32 = i_Param_s32; }
    void PutM_CmrFrntOpn(bool_t i_Param_b) { cameraFrontOpen_b = i_Param_b; }
    void PutM_FrmNmbr(uint32_t i_Param_u32) { frameNumber_u32 = i_Param_u32; }
    void PutM_IncFrmNmbr(uint32_t i_Param_u32) { frameNumber_u32 += i_Param_u32; }
    void PutM_VdWdth(uint16_t i_Param_u16) { videoWidth_u16 = i_Param_u16; }
    void PutM_VdHght(uint16_t i_Param_u16) { videoHeight_u16 = i_Param_u16; }
    //PRQA S 6200 1 //If i_Param_ps is const, compiler would complain for assignment
    void PutM_PSvdDtInf(uint8_t i_ID_u8, TSCSavedDataInfo_s *i_Param_ps) { savedDataInfo_aps[i_ID_u8] = i_Param_ps; }

    enum GearValues_e
    {
        e_GearReverse = -1,
        e_GearNeutral = 0,
        e_GearForward = 1
    };

private:
    // From Video Image Info
    const uint8_t* c_Cameras_apu8[e_TscNumCam];

    float32_t speed_f32;
    float32_t wheelAngle_f32;
    float32_t hitchAngle_f32;
    // From CAN Translation Info
    sint32_t gearDirection_s32;
    bool_t cameraFrontOpen_b;
    
    // From App Ctrl Info
    uint32_t frameNumber_u32;
    uint16_t videoWidth_u16;
    uint16_t videoHeight_u16;

    // pointer to TSC Algo saved data, one per camera
    // TSC algo set these pointers for TSC_Init() caller
    TSCSavedDataInfo_s *savedDataInfo_aps[e_TscNumCam];
};

// ----------------------------------------------------------------------------
/***** TSC States and Errors ******/
enum TSCState_e
{
    e_TscStateUnInit,
    e_TscStateInitOk,
    e_TscStateError,
    e_TscStateFeatureCollection,
    e_TscStateFeatureCollectionCompleted,
    e_TscStateCalibration,
    e_TscStateCalibrationCompleted,
    e_TscStateTerminated,
    e_TscStatePaused,
    e_TscStateUnknown, // only for the case where TSC_GetState() gets called with an unknown cameraID
    e_TscStateEnd
};

enum TSCError_e
{
    e_TscErrorNoError = 0,
    e_TscErrorInitFail,
    e_TscErrorStartFail,
    e_TscErrorFeatureCollectionError,
    e_TscErrorCalibrationError,
    e_TscErrorInvalidParameter,
    e_TscErrorInvalidSavedData,
    e_TscErrorUnexpectedRequest,
    e_TscErrorNum
};

enum TSCCalibManeuver_e
{
	e_TrlrCamraCalibManeuver_NULL = 0,
	e_TrlrCamraCalibManeuver_DRIVEFWD = 1,
	e_TrlrCamraCalibManeuver_TURN = 2,
	e_TrlrCamraCalibManeuver_COMPLETE = 3,
	e_TrlrCamraCalibManeuver_FAILED = 4,
	e_TrlrCamraCalibManeuver_MAX
};

// ----------------------------------------------------------------------------
/***** Calibration Parameters ******/
struct CalibrationParams_s
{
    float64_t yawDeg_f64;
    float64_t pitchDeg_f64;
    float64_t rollDeg_f64;
    float64_t xMM_f64;
    float64_t yMM_f64;
    float64_t zMM_f64;
};

// ----------------------------------------------------------------------------
/***** Debug Overlays ******/
struct DebugOverlay_s
{
    sint32_t startPt_x;
    sint32_t startPt_y;
    sint32_t endPt_x;
    sint32_t endPt_y;
};

#ifdef DEBUG_TSC_ALG
struct DebugCounters
{
    uint32_t invalidTrckLen;
    uint32_t invalidPxMotion;
    uint32_t invalidValidSlope;
    uint32_t ignoredMissingPair;
    uint32_t ignoredIGDeviation;
    uint32_t ignoredIGThreshold;
    uint32_t ignoredIGCombinThreshold;
};
#endif

} // End of namespace tscApi
// ----------------------------------------------------------------------------
/***** TSC Platform Interface ******/
// PRQA S 2000 ++
// PRQA S 2642 ++
bool_t TSC_Init(tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po);
bool_t TSC_Start(tscApi::enuCameraID i_CameraID_t, const tscApi::tscPlatformExtConfigType * i_TSCConfigDataPtr_pt, const tscApi::TSCSavedDataInfo_s * i_TSCSavedData_ps);
bool_t TSC_Uninit(void);
bool_t TSC_Pause(void);
bool_t TSC_Resume(void);
bool_t TSC_ProcessFrame(void);
bool_t TSC_Calibrate(tscApi::enuCameraID i_CameraID_t);
bool_t TSC_Stop(void);
tscApi::TSCState_e TSC_GetState(tscApi::enuCameraID i_CameraID_t);
tscApi::TSCError_e TSC_GetError(tscApi::enuCameraID i_CameraID_t);
tscApi::TSCCalibManeuver_e TSC_GetCalibManeuver( tscApi::enuCameraID i_CameraID_t );
bool_t TSC_GetFinalCalibrationResult( tscApi::enuCameraID i_CameraID_t, tscApi::CalibrationParams_s* o_FinalCalibrationResult_ps );
sint32_t TSC_GetValidFeaturesCount(tscApi::enuCameraID i_CameraID_t);
sint32_t TSC_GetIgnoredValidFeaturesCount(tscApi::enuCameraID i_CameraID_t);
sint32_t TSC_GetInvalidFeaturesCount(tscApi::enuCameraID i_CameraID_t);
sint32_t TSC_GetUndetectedFeaturesCount(tscApi::enuCameraID i_CameraID_t);
sint32_t TSC_GetSkippedFramesCount(tscApi::enuCameraID i_CameraID_t);
sint32_t TSC_GetProcessedFramesCount(tscApi::enuCameraID i_CameraID_t);
const tscApi::DebugOverlay_s* TSC_GetOverlays(tscApi::enuCameraID i_CameraID_t, uint8_t &o_Num_ru8);
#ifdef DEBUG_TSC_ALG
    const tscApi::DebugCounters* TSC_GetDebugCounters( tscApi::enuCameraID cameraID );
    bool_t TSC_GetFinalCalibrationResultStdDev( tscApi::enuCameraID cameraID, tscApi::CalibrationParams* finalCalibrationiResultStdDev );
#endif
// PRQA S 2642 --
// PRQA S 2000 --

#endif

