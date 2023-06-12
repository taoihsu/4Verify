// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [04-Jun-2013]
// --- Modified by Ehsan Parvizi [14-Aug-2014]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __TSCALG_H_
#define __TSCALG_H_
#include "module.h"
#include "tscAlgImpl.h"
// ------------------------------------------------------------------------
// --- configuration information
// This commented code is uncommented and used when debugging with synthetic images.
// PRQA S 1051 3
//#ifndef DEBUG_TSC_ALG
//#define DEBUG_TSC_ALG
//#endif
namespace tsc
{
class TSCConfig
{
    public:
      TSCConfig(): 
        worldPtLimit_t(), 
        fundMatOutlierThresh_f64( 0 ), 
        fCRestriction_s( tsc::FeatureCollectionRestriction_s() ) 
      {
      }
      void SetWorldPtLimit(tsc_cfg::SofM_ConfigStrType i_Cst_t);
      void SetFCRestriction(uint32_t i_MaxValidFrame_u32, uint32_t i_MinRawFrame_u32);
      float64_t getWorldPtLimitX_f64(void) const { return worldPtLimit_t.x_x; }
      float64_t getWorldPtLimitY_f64(void) const { return worldPtLimit_t.y_x; }
      float64_t getWorldPtLimitZ_f64(void) const { return worldPtLimit_t.z_x; }
      float64_t getFundMatOutlierThresh_f64(void) const { return fundMatOutlierThresh_f64; }
      uint32_t getFCRestrictionMaxNumValidFrame_u32(void) const { return fCRestriction_s.maxNumValidFrames_u32; }
      uint32_t getFCRestrictionMinNumRawFrame_u32(void) const { return fCRestriction_s.minNumRawFrames_u32; }
    private:
      // --- distance limit for world points
      fc::Point3d worldPtLimit_t;
      // --- fundamental matrix outlier threshold
      float64_t fundMatOutlierThresh_f64;
      tsc::FeatureCollectionRestriction_s fCRestriction_s;
};

struct TSCCameraConfig_s
{
    public:
    TSCCameraConfig_s(): 
        fullCalibration_b( false )
    {
    }
    void setFullCalibration (bool_t i_Value_b) { fullCalibration_b = i_Value_b; }
    private:
    bool_t fullCalibration_b;
};
// ----------------------------------------------------------------------------
// --- global data exposed by this plug-in
  struct TSCInfo_s
  {
    public:
  };

// ----------------------------------------------------------------------------
// --- AppCtrl plug-in
  class TSCAlg : public control::Module<TSCAlg, TSCAlgImpl, TSCInfo_s, TSCConfig, TSCCameraConfig_s>
  {
    // SB friend class Module;
    template <class ModuleType, class ModuleTypeImpl, class ModuleTypeInfo, class ModuleTypeConfig, class ModuleTypeCameraConfig>
    friend class control::Module;

    public:
    TSCAlg();
    ~TSCAlg();
    
    virtual bool_t Init( tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po );
    virtual bool_t processFrame_b( void );
    virtual void uninit_v( void );
    virtual void reset_v( void );
    static float32_t TrimMeanPercentage_f32;
    bool_t UpdateExternalConfiguration(tscApi::enuCameraID i_CameraID_t);
    bool_t start_b(tscApi::enuCameraID i_CameraID_t);
    static void setTrimMeanPercentage_v( float32_t percent_f32 );
    tscApi::TSCState_e getTSCState_e(tscApi::enuCameraID i_CameraID_t) { return implArray_t[i_CameraID_t].getTSCState_e(); }
    tscApi::TSCError_e getTSCError_e(tscApi::enuCameraID i_CameraID_t) { return implArray_t[i_CameraID_t].getTSCError_e(); }
    tscApi::TSCCalibManeuver_e GetCalibManeuver_e( tscApi::enuCameraID i_CameraID_t )
    {
        return implArray_t[i_CameraID_t].getTSCCalibManeuver_e();
    }
	bool_t Calibrate(tscApi::enuCameraID i_CameraID_t) { return implArray_t[i_CameraID_t].calibrate_b(); }
    bool_t GetFinalCalibrationResult( tscApi::enuCameraID i_CameraID_t, tscApi::CalibrationParams_s* o_FinalCalibrationResult_ps );
#ifdef DEBUG_TSC_ALG
    bool_t GetFinalCalibrationResultStdDev( tscApi::enuCameraID cameraID, tscApi::CalibrationParams_s* finalCalibrationResultStdDev );
#endif
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)    // PRQA S 1070
    bool_t GetDetailedCalibrationResult( tscApi::enuCameraID cameraID, tsc::DetailedCalibrationResult* detailedCalibrationResult );
#endif

    private:
    virtual bool_t loadConfiguration_b( void );
    virtual bool_t SetCameraSpecificConfig(const tscApi::enuCameraID i_CameraID_t);   // PRQA S 2128
    bool_t CreateImplObject(tscApi::enuCameraID cameraID);
  };
}

// ----------------------------------------------------------------------------
#endif
