// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [04-Jun-2013]
// --- Modified by Ehsan Parvizi [20-Aug-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __FEATURECOLLECTION_H_
#define __FEATURECOLLECTION_H_
// ----------------------------------------------------------------------------
#include "module.h"
#include "kinematicModel.h"
#include "featureCollectionImpl.h"
// ----------------------------------------------------------------------------
// --- global data exposed by this plug-in
// PRQA S 1051 3 //This code is uncommented for debugging purposes
//#ifndef DEBUG_TSC_ALG
//#define DEBUG_TSC_ALG
//#endif
namespace fc
{
  class FeatureCollectionInfo
  {
    public:
#ifdef USE_SVSCM
      camera_model::CameraModel *getMAddrCmrMdls_po(void) { return &cameraModels_ao[0]; }
#else
      camera_model::CameraModelMecl *getMAddrCmrMdls_po(void) { return &cameraModels_ao[0]; }
#endif
      km::KinematicModelImpl *getMPKnmtcMdl_po(void) const { return pKinematicModel_po; }
      // PRQA S 6200 1 //Input can't be const here, the variable to which it is assigned isn't const.
      void putMPKnmtcMdl_v(km::KinematicModelImpl *i_Param_po) { pKinematicModel_po = i_Param_po; }

    private:
#ifdef USE_SVSCM
      camera_model::CameraModel cameraModels_ao[tsc_cfg::MAX_NUM_CAMERA_CALIB];
#else
      camera_model::CameraModelMecl cameraModels_ao[tsc_cfg::MAX_NUM_CAMERA_CALIB];
#endif
      km::KinematicModelImpl* pKinematicModel_po;
  };

  // ------------------------------------------------------------------------
  // --- configuration information
  struct FeatureCollectionConfig_s
  {
  };
  struct FeatureCollectionCameraConfig_s
  {
  };

  // ----------------------------------------------------------------------------
  // --- AppCtrl plug-in
  class FeatureCollection : public control::Module<FeatureCollection, FeatureCollectionImpl, FeatureCollectionInfo, FeatureCollectionConfig_s, FeatureCollectionCameraConfig_s>
  {
    // SB friend class Module;
    template <class ModuleType, class ModuleTypeImpl, class ModuleTypeInfo, class ModuleTypeConfig, class ModuleTypeCameraConfig>
    friend class control::Module;

    public:
    int selFeaturesSize = 0;
    virtual bool_t Init(tscApi::TSCCtrlInfo* b_TscCtrlInfo_po);
    virtual bool_t processFrame_b(void);
    virtual void uninit_v(void);
    virtual void reset_v(void);

    uint32_t GetValidFeaturesCount(tscApi::enuCameraID i_CameraID_t) { return implArray_t[i_CameraID_t].getDiagnosticData_po()->getNumValidFeatures_u32(); }
    uint32_t GetIgnoredValidFeaturesCount(tscApi::enuCameraID i_CameraID_t) { return implArray_t[i_CameraID_t].getDiagnosticData_po()->getNumIgnoredValidFeatures_u32(); }
    uint32_t GetInvalidFeaturesCount(tscApi::enuCameraID i_CameraID_t) { return implArray_t[i_CameraID_t].getDiagnosticData_po()->getNumInvalidFeatures_u32(); }
    uint32_t GetUndetectedFeaturesCount(tscApi::enuCameraID i_CameraID_t) { return implArray_t[i_CameraID_t].getDiagnosticData_po()->getNumUndetectedFeatures_u32(); }
    uint32_t GetSkippedFramesCount(tscApi::enuCameraID i_CameraID_t) { return implArray_t[i_CameraID_t].getDiagnosticData_po()->getNumSkippedFrames_u32(); }
    uint32_t GetProcessedFramesCount(tscApi::enuCameraID i_CameraID_t) { return implArray_t[i_CameraID_t].getDiagnosticData_po()->getNumProcessedFrames_u32(); }
    const tscApi::DebugOverlay_s* GetOverlays(tscApi::enuCameraID i_CameraID_t, uint8_t &o_Num_ru8) {
      tscApi::DebugOverlay_s *v_RetValue_ps = NULL;
      o_Num_ru8 = implArray_t[i_CameraID_t].getDiagnosticData_po()->getOverlays_rx().size_u32();
      if (o_Num_ru8 > 0){
        v_RetValue_ps =  &implArray_t[i_CameraID_t].getDiagnosticData_po()->getOverlays_rx()[0];
      }
      return v_RetValue_ps;
    }
#ifdef DEBUG_TSC_ALG
    const tscApi::DebugCounters* GetDebugCounters(tscApi::enuCameraID cameraID) { return m_implArray[cameraID].GetDebugCounters(); }
#endif
//#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
    mecl::core::ArrayList< fc::ValidFeature, tsc_cfg::NUM_VALID_INDICES>& GetValidFeatures_u32(tscApi::enuCameraID cameraID) { return implArray_t[cameraID].getDiagnosticData_po()->getValidFeatures_u32(); }
//#endif
    bool_t UpdateExternalConfiguration(tscApi::enuCameraID i_CameraID_t);
    private:
    virtual bool_t loadConfiguration_b(void);

    FeatureCollection();
    ~FeatureCollection();

    km::KinematicModelImpl kinematicModel_o;
  };
}
//-----------------------------------------------------------------------------
#endif
