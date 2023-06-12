// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [04-Jun-2013]
// --- Modified by Ehsan Parvizi [09-Oct-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __FEATURECOLLECTIONIMPL_H_
#define __FEATURECOLLECTIONIMPL_H_
// ----------------------------------------------------------------------------
#include "mecl/mecl.h"
#include "moduleImpl.h"
#include "localFeatureCollector.h"
#include "featureFilter.h"
// ----------------------------------------------------------------------------
// PRQA S 1051 3 //This code is uncommented for debugging purposes
//#ifndef DEBUG_TSC_ALG
//#define DEBUG_TSC_ALG
//#endif
namespace fc
{
class FeatureCollectionInfo;
struct FeatureCollectionConfig_s;
struct FeatureCollectionCameraConfig_s;

// --- feature collection implementation
class FeatureCollectionImpl : public control::ModuleImpl<FeatureCollectionInfo,FeatureCollectionConfig_s,FeatureCollectionCameraConfig_s>
{
    public:
    FeatureCollectionImpl();
    ~FeatureCollectionImpl();

    virtual bool_t Init(tscApi::TSCCtrlInfo* b_TscCtrlInfo_po, FeatureCollectionInfo* b_Info_po, FeatureCollectionConfig_s* b_Config_ps, FeatureCollectionCameraConfig_s* b_CameraConfig_ps, sint64_t i_Tracer_s64, tscApi::enuCameraID i_CameraId_t);
    virtual bool_t unInit_b(void);
    virtual bool_t process_b(void);
    virtual void cleanupLocalData_v(void);
    virtual bool_t start_b(void);
    virtual void reset_v(void);
    bool_t resetAllLocalFeatureTracks_b(bool_t i_Collect_b);
#ifdef ENABLE_SFM    // PRQA S 1070
    fc::ValidFeatureCollection* GetValidFeatureCollection(void) {return &m_validFeatureCollection;}
#endif
    const mecl::core::ArrayList< fc::InitialGuess, tsc_cfg::NUM_INITIAL_GUESSES >& GetInitialGuesses() const { return initialGuesses_x; }
    const mecl::core::Array < uint32_t, tsc_cfg::NUM_AVAILABLE_ROIS>* GetROIPerformances( sint32_t lfcNum );
    bool_t UpdateExternalConfiguration(tscApi::enuCameraID i_CameraID_t);
    uint32_t getFrameCounter_u32(void) const { return frameCounter_u32; }
    bool_t ValidateSavedData(tscApi::TSCSavedDataInfo_s const * i_TSCSavedData_pt);
    bool_t restoreSavedData_b();
    fc::DiagnosticData* getDiagnosticData_po(void) { return &diagnosticData_o; }
    bool_t LFCValidateROIRect(const tsc_math::ROIRect& i_Rect_rt) const { return lfc_x[0].ValidateROIRect(i_Rect_rt); }
#ifdef DEBUG_TSC_ALG
    tscApi::DebugCounters* GetDebugCounters(void) { return &m_debugCounters; }
#endif

    private:
    virtual bool_t loadConfiguration_b(void);
    bool_t LoadCameraSpecificConfig(const tscApi::enuCameraID i_CameraID_t);
    bool_t LoadCameraModelConfig(const tscApi::enuCameraID i_CameraID_t);

    tscApi::enuCameraID tscTargetCamID;
    // --- CollectValidFeatures:
    // --- Find valid features from previously terminated tracks
    // --- and put them into our collection
    // --- also backup them into savedData
    bool_t CollectValidFeatures( lfc::LocalFeatureCollector *b_LFC_po, ff::FeatureFilter const *i_FF_po );

    // --- local feature collectors
    mecl::core::Array<lfc::LocalFeatureCollector, tsc_cfg::MAX_ROI_COUNT> lfc_x;

    // --- feature filter
    mecl::core::Array<ff::FeatureFilter, tsc_cfg::MAX_ROI_COUNT> ff_x;

#ifdef ENABLE_SFM    // PRQA S 1070
    fc::ValidFeatureCollection m_validFeatureCollection;
#endif
    mecl::core::ArrayList<fc::ROI, tsc_cfg::MAX_ROI_COUNT> rois_x;
    mecl::core::ArrayList< fc::InitialGuess, tsc_cfg::NUM_INITIAL_GUESSES > initialGuesses_x;
    fc::DiagnosticData diagnosticData_o;    //ASSUMING A FIXED MAX_ROI_COUNT of 1
    km::KinematicModelCameraImpl* kinematicModelCameraImpl_po;
    fc::TSCSavedData savedData_o;
    tscApi::TSCSavedDataInfo_s savedDataInfo_s;
    uint32_t frameCounter_u32; // move from tscAlg, for store raw frame number in savedData
#ifdef DEBUG_TSC_ALG
    tscApi::DebugCounters m_debugCounters;
#endif

    // Logging Valid Frame Numbers
    std::string m_ValidFramesFileName;
    std::stringstream m_ValidFrames;
    std::stringstream m_ValidFramesHeader;
};
}
//-----------------------------------------------------------------------------
#endif
