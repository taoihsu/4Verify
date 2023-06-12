// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [08-Jul-2013]
// --- Modified by Ehsan Parvizi [20-Aug-2014]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __FF_H_
#define __FF_H_
// ----------------------------------------------------------------------------
#include "featureCollectionStructs.h"
#include "localFeatureCollector.h"
// ----------------------------------------------------------------------------
// --- FeatureFilter
// PRQA S 1051 3 //This code is uncommented for debugging purposes
//#ifndef DEBUG_TSC_ALG
//#define DEBUG_TSC_ALG
//#endif
namespace ff
{
class FeatureFilter
{
    public:
    FeatureFilter( );
    ~FeatureFilter();

#ifdef DEBUG_TSC_ALG
    bool_t Init( tscApi::enuCameraID i_CameraID_t, fc::ROI* i_ROI_po, const uint8_t* i_Img_pu8, fc::DiagnosticData* i_DiagnosticData_po, tscApi::DebugCounters* pDebugCounters);
#else
    bool_t Init( tscApi::enuCameraID i_CameraID_t, fc::ROI* i_ROI_po, const uint8_t* i_Img_pu8, fc::DiagnosticData* i_DiagnosticData_po);
#endif
    bool_t Process( uint32_t i_CurrFrameNum_u32, lfc::LocalFeatureCollector *b_LFC_po );
    const mecl::core::ArrayList< fc::InitialGuess, tsc_cfg::NUM_INITIAL_GUESSES_PER_FRAME >& GetInitialGuesses() const { return initialGuesses_x; }
    const mecl::core::ArrayList< uint32_t, tsc_cfg::NUM_VALID_INDICES >& GetValidIndices() const { return validIndices_x; }
    bool_t UpdateExternalConfiguration(tscApi::enuCameraID i_CameraID_t);

    private:
    // --- CheckMinPixelMotion
    // --- Checks whether the accumulated tracks have
    // --- the minimum required pixel motion
    bool_t loadConfiguration_b(void);

    static bool_t CheckMinPixelMotion(fc::FeatureTrack& b_Ft_ro, uint32_t i_Thresh_u32 );
    // --- UnwarpFeatureTrack
    bool_t UnwarpFeatureTrack( fc::FeatureTrack& b_Ft_ro ) const;
    // --- CheckValidSlope
    bool_t CheckValidSlope( fc::FeatureTrack& b_Ft_ro, bool_t i_UseRatio_b) const;
    // ---
#ifdef string
    void PrintPointPairOnError(std::string val, fc::ImageFeature &imPair1Pt1, fc::ImageFeature &imPair1Pt2, fc::ImageFeature &imPair2Pt1, fc::ImageFeature &imPair2Pt2) const;
#else
// PRQA S 1020, 1034 1 //Debugging code
#define PrintPointPairOnError( ... )
#endif
#ifdef USE_SVSCM
    void ComputeAmatRow(fc::ImageFeature &imPair1Pt1, fc::ImageFeature &imPair1Pt2, float64_t (&AMat)[2][3], uint8_t ind, float64_t (&pairCam)[2][3]) const; 
#else
    void ComputeAmatRow(const fc::ImageFeature &i_ImPair1Pt1_ro, const fc::ImageFeature &i_ImPair1Pt2_ro, float32_t (&o_AMat_rf32)[2][3], uint8_t i_Ind_u8, float32_t (&o_PairCam_rf32)[2][3]) const;
#endif
    template<typename T>
    bool_t ComputeEta(T *o_Eta_px, T (&i_PairCam_rx)[2][3], uint8_t i_Ind_u8, T (&i_RotColumn_rx)[3]);
    template<typename T>
    bool_t ComputeInitialGuessByPairs(const fc::ImageFeature &i_ImPair1Pt1_ro, const fc::ImageFeature &i_ImPair1Pt2_ro, const fc::ImageFeature &i_ImPair2Pt1_ro, const fc::ImageFeature &i_ImPair2Pt2_ro, T i_ThetaCar_x, T v_DirectionalMotion_f32);
    bool_t ComputeInitialGuess( fc::LocalFeatureTrackCollection& b_LocalTrackList_ro );
    bool_t ComputeInitialGuessCombinations( fc::LocalFeatureTrackCollection& b_LocalTrackList_ro );
    void ApplySfMFilter( const mecl::core::ArrayList < fc::FeatureTrack, tsc_cfg::NUM_TRACKS >& i_Tracks_rt );
    template<typename T>
    bool_t SVDcomputeEx(T const (&i_Aarr_ax)[2][3], T (&o_W_ax)[3]);
    template<typename T>
    static void normalizeEx(T const (&i_Src_rax)[3], T (&o_Dst_ax)[3]);
    template<typename T>
    static T normEx(T const (&i_Src_rax)[3]);
    template<typename T>
    static T hypotEx(T i_A_t, T i_B_t);
    template<typename T>
    static sint32_t givensEx(T (&i_A_ax)[2], T (&i_B_ax)[2], sint32_t i_N_s32, T i_C_x, T i_S_x);

    tscApi::enuCameraID cameraID_t;
        tscApi::enuCameraID tscTargetCameraID_t;

    fc::ROI* rpo_po;
    bool_t initOK_b;
    const uint8_t *c_Img_pu8;
    uint64_t hTracer_u64;
    uint32_t minPixelMotionThresh_u32; // --- minimum threshold for pixel motion
    uint32_t currFrameNum_u32;
    // --- implementation specific data members
#ifdef USE_SVSCM
    camera_model::CameraModel *cameraModel_po;
#else
    camera_model::CameraModelMecl *cameraModel_po;
#endif
    uint32_t trackLengthThresh_u32;
    float64_t slopeDifferenceThresh_f64;
    bool_t useSfmFilter_b;    
    float64_t maxHeightDiffMm_f64;
    // --- acceptable threshold for Initial Guess angles
    float32_t angleThreshIGDeg_f32;
    // --- acceptable percentage deviation for Initial Guess theta parameter
    uint32_t deviationPercentageIG_u32;
    mecl::core::ArrayList< uint32_t, tsc_cfg::NUM_VALID_INDICES > validIndices_x;
    mecl::core::ArrayList< fc::InitialGuess, tsc_cfg::NUM_INITIAL_GUESSES_PER_FRAME > initialGuesses_x;
    mecl::core::ArrayList< fc::InitialGuess, tsc_cfg::NUM_INITIAL_GUESSES_PER_FRAME > tempinitialGuesses_x;
        std::stringstream m_initialGuessTimeStream;
    bool_t useInitialGuessCombinations_b;
    // --- threshold for intermediate Initial Guess angles
    float32_t initialGuessDiffThresholdDeg_f32;
    fc::DiagnosticData* diagnosticData_po;
#ifdef DEBUG_TSC_ALG
    tscApi::DebugCounters* m_pDebugCounters;
#endif
        std::string m_logFileName;
        std::stringstream m_logStream;
        std::stringstream m_logHeader;

        mutable uint32_t invalidSlopeCount;
};
}
// ----------------------------------------------------------------------------
#endif
