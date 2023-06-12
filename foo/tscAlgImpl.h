// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [04-Jun-2013]
// --- Modified by Ehsan Parvizi [15-Aug-2014]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __TSCALGIMPL_H_
#define __TSCALGIMPL_H_
// ----------------------------------------------------------------------------
#include "moduleImpl.h"
#include "featureCollection.h"
#include "structureFromMotion.h"
// ----------------------------------------------------------------------------
// This commented code is uncommented and used when debugging with synthetic images.
// PRQA S 1051 3
//#ifndef DEBUG_TSC_ALG
//#define DEBUG_TSC_ALG
//#endif
namespace tsc
{
  struct TSCInfo_s;
  struct TSCCameraConfig_s;
  class TSCConfig;
  const uint16_t arrySz = tsc_cfg::NUM_INITIAL_GUESSES;
//-----------------------------------------------------------------------------
class TSCAlgImpl : public control::ModuleImpl<TSCInfo_s,TSCConfig,TSCCameraConfig_s>
{
    public:
    TSCAlgImpl();
    ~TSCAlgImpl();
    virtual bool_t Init(tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po, tsc::TSCInfo_s* b_Info_ps, tsc::TSCConfig* b_Config_po, tsc::TSCCameraConfig_s* b_CameraConfig_ps, sint64_t i_Tracer_s64, tscApi::enuCameraID i_CameraID_t);
    virtual bool_t unInit_b( void );
    virtual bool_t process_b( void );
    virtual bool_t start_b( void );
    virtual void cleanupLocalData_v( void );
    virtual void reset_v( void );
    
    tscApi::TSCState_e getTSCState_e(void) const { return tscState_e; } 
    tscApi::TSCError_e getTSCError_e(void) const{ return tscError_e; } 
	tscApi::TSCCalibManeuver_e getTSCCalibManeuver_e( void ) const
    {
        return tscCalibManeuver_e;
    }

    bool_t collectInitialGuessAngles_b( void );
    bool_t calibrate_b();
    bool_t UpdateExternalConfiguration(tscApi::enuCameraID i_CameraID_t) const;
    bool_t GetFinalCalibrationResult( tscApi::CalibrationParams_s* o_FinalCalibrationResult_ps ) const;
//#ifdef DEBUG_TSC_ALG
    bool_t GetFinalCalibrationResultStdDev( tscApi::CalibrationParams_s* finalCalibrationResultStdDev );
//#endif
//#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)    // PRQA S 1070
        bool_t GetDetailedCalibrationResult( tsc::DetailedCalibrationResult* detailedCalibrationResult );
        //#endif
//#endif
        // Windows Logging
        bool_t DumpValidFeatures();
        bool_t DumpInitialGuesses();
        bool_t DumpCalibrationResult();
        void setTargetCamera( tscApi::enuCameraID targetCamID );
        struct reprojection
        {
            int x, y;
            float rep;
        };

    private:
    // ---
    virtual bool_t loadConfiguration_b( void );
    uint8_t loadCameraConfig_u8( void );
    bool_t collectFeatures_b( void );
    static bool_t isMotionValid_b( void );
    static float64_t meanEx( fc::Pointf (&i_Array_rt)[tsc::arrySz], size_t i_Total_t ); // -- Replacement of cv::mean()
    bool_t fitLineEx( fc::Pointf (&i_Points_rt)[tsc::arrySz], sint32_t i_Count_s32, float32_t i_Reps_f32, float32_t i_Aeps_f32, float32_t (&o_Line_rf32)[4] ); // -- Replacement of cv::fitLine()
    static void WeightWelschEx( const float32_t (&i_D_raf32)[tsc_cfg::NUM_INITIAL_GUESSES], sint16_t i_Count_s16, float32_t (&i_W_rf32)[tsc_cfg::NUM_INITIAL_GUESSES] );
    static bool_t FitLine2D_wodsEx( fc::Pointf (&i_Points_rat)[tsc::arrySz], sint16_t i_Count_s16, const float32_t (&i_Weights_raf32)[tsc_cfg::NUM_INITIAL_GUESSES], float32_t (&i_Line_af32)[4] );
    static float64_t CalcDist2DEx(  const fc::Pointf (&i_Points_rat)[tsc::arrySz], const sint16_t i_Count_s16, const float32_t (&i_Line_raf32)[4], float32_t (&i_Dist_af32)[tsc_cfg::NUM_INITIAL_GUESSES] );
    static void meanStdDevEx( fc::Pointf (&i_Array_rt)[tsc::arrySz], size_t i_Total_t, float64_t &o_Mean_rf64, float64_t &o_StdDev_rf64 ); // -- Replacement of cv::meanStdDev()

        tscApi::enuCameraID ocTargetCamera;
        bool_t dumpResults;
    bool_t earlyBreakOnStop_b;
    uint32_t featureCollectionBeginFrame_u32;
    uint32_t featureCollectionEndFrame_u32;
    const mecl::core::ArrayList < fc::InitialGuess, tsc_cfg::NUM_INITIAL_GUESSES > *c_IGs_px;
    fc::FeatureCollectionImpl* featureCollector_po;
    fc::InitialGuess initialGuess_o;
    tsc::IGConfidenceLevel iGConfidence_o;
#ifdef ENABLE_SFM    // PRQA S 1070
    StructureFromMotion m_SfM;
#endif
    tscApi::TSCState_e tscState_e;
    tscApi::TSCError_e tscError_e;
	tscApi::TSCCalibManeuver_e tscCalibManeuver_e;
    float32_t calibTime_f32;
    float32_t wArry_af32[tsc_cfg::NUM_INITIAL_GUESSES];
    float32_t rArry_af32[tsc_cfg::NUM_INITIAL_GUESSES];

        // Windows logging
        std::stringstream m_validFeaturesStream;
        std::stringstream m_validFeaturesHeader;

        std::stringstream m_initialGuessStream;
        std::stringstream m_initialGuessHeader;

        std::stringstream m_calibrationResultStream;
        std::stringstream m_calibrationHeader;

        std::stringstream m_detailedCalibrationResultStream;
        std::stringstream m_detailedCalibrationHeader;

};
}

//-----------------------------------------------------------------------------
#endif
