#ifndef OCDATA_H
#define OCDATA_H


namespace ocdata
{
    enum OcAlgoState_e
    {
        e_OcStateUninit,
        e_OcStateInitOk,
        e_OcStateError,
#if 0
        e_OcStateFeatureCollectionStraight,         /* state after starting Algo */
        e_OcStateFeatureCollectionStraightCompleted,    /* state after min. 100 features and min. 900 frames are collected */
        e_OcStateFeatureCollectionCurved,
        e_OcStateFeatureCollectionCurvedCompleted,
        e_OcStateCalibrationStraight,
        e_OcStateCalibrationStraightCompleted,
        e_OcStateCalibrationCurved,
        e_OcStateCalibrationCurvedCompleted,            /* state after 1 cam is calibrated + e_OC_NoError => values in ME_Hydra2defs_S_OCStatus_t are valid */
#endif
		e_OcStateFeatureCollection,         /* state after starting Algo */
		e_OcStateFeatureCollectionCompleted,    /* state after min. 100 features and min. 900 frames are collected */
		e_OcStateCalibration,
		e_OcStateCalibrationCompleted,
        e_OcStateTerminated,
        e_OcStatePaused,
        e_OcStateUnknown, // only for the case where TSC_GetState() gets called with an unknown cameraID
        e_OcStateEnd
    };

    enum OcErrorCode_e
    {
        e_OcErrCodeNoError = 0,
        e_OcErrCodeInitFail,
        e_OcErrCodeStartFail,
        e_OcErrCodeFeatureCollectionError,
        e_OcErrCodeCalibrationError,
        e_OcErrCodeInvalidConfiguration,
        e_OcErrCodeInvalidSavedData,
        e_OcErrCodeUnexpectedRequest,
        e_OcErrCodeNum
    };

    enum OcDriverPosition_e
    {
        e_DriverPositionLHD = 0,
        e_DriverPositionRHD = 1,
        e_DriverPositionDefault = e_DriverPositionLHD
    };

    struct OcDataToMcu_s
    {
        OcDataToMcu_s()
            : deltaPitch_f32( 0.0 )
            , deltaYaw_f32( 0.0 )
            , deltaRoll_f32( 0.0 )
            , deltaZ_f32( 0.0 )
            , validFeaturesCount_u32( 0U )
            , ignoredFeaturesCount_u32( 0U )
            , invalidFeaturesCount_u32( 0U )
            , ocAlgoState_e( e_OcStateUninit )
            , ocErrorCode_e( e_OcErrCodeNoError )
        {}

        float32_t deltaPitch_f32;
        float32_t deltaYaw_f32;
        float32_t deltaRoll_f32;
        float32_t deltaZ_f32;
        uint32_t  validFeaturesCount_u32;
        uint32_t  ignoredFeaturesCount_u32;
        uint32_t  invalidFeaturesCount_u32;
        OcAlgoState_e    ocAlgoState_e;
        OcErrorCode_e    ocErrorCode_e;

        OcDataToMcu_s& operator=( const OcDataToMcu_s& i_Data_rs )
        {
            if( this != &i_Data_rs )
            {
                deltaPitch_f32 = i_Data_rs.deltaPitch_f32;
                deltaYaw_f32 = i_Data_rs.deltaYaw_f32;
                deltaRoll_f32 = i_Data_rs.deltaRoll_f32;
                deltaZ_f32 = i_Data_rs.deltaZ_f32;
                validFeaturesCount_u32 = i_Data_rs.validFeaturesCount_u32;
                ignoredFeaturesCount_u32 = i_Data_rs.ignoredFeaturesCount_u32;
                invalidFeaturesCount_u32 = i_Data_rs.invalidFeaturesCount_u32;
                ocAlgoState_e = i_Data_rs.ocAlgoState_e;
                ocErrorCode_e = i_Data_rs.ocErrorCode_e;
            }

            return *this;
        }

    private:
        OcDataToMcu_s( const OcDataToMcu_s& i_data_rs );

    };

    enum OcAlgoView640x400Buffer_e
    {
        e_OcAlgoViewBuffer0 = 0,
        e_OcAlgoViewBuffer1
    };

    struct AlgosDebugData_s
    {
        AlgosDebugData_s()
            : frameLastUpdated_u32( 0U )
            , isActivated_b( false )
            , reserved1_u8( 0U )
            , reserved2_u8( 0U )
            , reserved3_u8( 0U )
        {}

        uint32_t frameLastUpdated_u32;
        bool_t isActivated_b;
        uint8_t reserved1_u8;  // for alignment
        uint8_t reserved2_u8;  // for alignment
        uint8_t reserved3_u8;  // for alignment
    };

    struct OcData_s
    {
        OcData_s()
            : deltaPitch_f32( 0.0 )
            , deltaYaw_f32( 0.0 )
            , deltaRoll_f32( 0.0 )
            , deltaX_f32( 0.0 )
            , deltaY_f32( 0.0 )
            , deltaZ_f32( 0.0 )
            , validFeaturesCount_u32( 0U )
            , ignoredFeaturesCount_u32( 0U )
            , invalidFeaturesCount_u32( 0U )
            , ocAlgoState_e( e_OcStateUninit )
            , ocErrorCode_e( e_OcErrCodeNoError )
            , lastRequestedAt_u32( 0U )
            , updatedAt_u32( 0U )
            , mcuCameraId_u8( 0U )
            , algoViewBuffer_e( e_OcAlgoViewBuffer0 )
            , isOcActive_b( false )
        {}

        OcData_s& operator=( const OcData_s& i_Data_rs )
        {
            if( this != &i_Data_rs )
            {
                //copy constructor of ocDataToMcu_s
                deltaPitch_f32 = i_Data_rs.deltaPitch_f32;
                deltaYaw_f32 = i_Data_rs.deltaYaw_f32;
                deltaRoll_f32 = i_Data_rs.deltaRoll_f32;
                deltaZ_f32 = i_Data_rs.deltaZ_f32;
                validFeaturesCount_u32 = i_Data_rs.validFeaturesCount_u32;
                ignoredFeaturesCount_u32 = i_Data_rs.ignoredFeaturesCount_u32;
                invalidFeaturesCount_u32 = i_Data_rs.invalidFeaturesCount_u32;
                ocAlgoState_e = i_Data_rs.ocAlgoState_e;
                ocErrorCode_e = i_Data_rs.ocErrorCode_e;
                //ocDataToMcu_s = i_Data_rs.ocDataToMcu_s;
                lastRequestedAt_u32 = i_Data_rs.lastRequestedAt_u32;
                updatedAt_u32 = i_Data_rs.updatedAt_u32;
                mcuCameraId_u8 = i_Data_rs.mcuCameraId_u8;
                algoViewBuffer_e = i_Data_rs.algoViewBuffer_e;
                isOcActive_b = i_Data_rs.isOcActive_b;
                //debugLayer_s = i_Data_rs.debugLayer_s;
            }

            return *this;
        }

        float32_t deltaPitch_f32;
        float32_t deltaYaw_f32;
        float32_t deltaRoll_f32;
        float32_t deltaZ_f32;
        float32_t deltaX_f32;
        float32_t deltaY_f32;
        uint32_t  validFeaturesCount_u32;
        uint32_t  ignoredFeaturesCount_u32;
        uint32_t  invalidFeaturesCount_u32;
        OcAlgoState_e    ocAlgoState_e;
        OcErrorCode_e    ocErrorCode_e;

        //OcDataToMcu_s ocDataToMcu_s;
        uint32_t lastRequestedAt_u32;
        uint32_t updatedAt_u32;
        uint8_t  mcuCameraId_u8;
        OcAlgoView640x400Buffer_e algoViewBuffer_e;
        bool_t isOcActive_b;
        //AlgosDebugData_s debugLayer_s;

    private:
        OcData_s( const OcData_s& i_data_rs );

    };

    enum AlgoCommand_e
    {
        e_Start = 0,
        e_Stop,
        e_Pause,
        e_Status,
        e_Sync,
        e_GetResult,
        e_Resume,
        e_Set,
        e_Debug,
        e_Unknown                   // this must be the last entry ('VALID_COMMAND' < e_Unknown)
    };


    // ------------------------------------------------------------------------------------------------
    // --- CameraId
    enum CameraId_e
    {
        e_NoCamera = 0,
        e_Front,
        e_Left,
        e_Rear,
        e_Right,
        e_Fifth,
        e_NumCams = e_Fifth
    };


    // ------------------------------------------------------------------------------------------------
    // --- VariantOC
    typedef struct TrajectoryFilterConfigStr_s
    {
        uint32_t  minPixelMotionThresh_u32;
        float64_t slopeDifferenceThreshold_f64;
        float64_t angleThresholdDegIG_f64;
        uint32_t  deviationPercentageIG_u32;
        bool_t    useCombinations_b;
        float64_t combinationsDiffThresholdDeg_f64;
        bool_t    useSfMFilter_b;
        float64_t maxHeightDiffMm_f64;
    } TrajectoryFilterConfigStr_t;
}



#endif // !OCDATA_H

