// ----------------------------------------------------------------------------
// --- Written by Hany Kashif [23-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __TSCALGSTRUCTS_H_
#define __TSCALGSTRUCTS_H_
// ----------------------------------------------------------------------------
#include "mecl/mecl.h"
#include "featureCollectionStructs.h"
// ----------------------------------------------------------------------------
// --- data structures with namespace tsc (feature collector)
// ------------------------------------------------------------------------
namespace tsc
{
    class IGConfidenceLevel
    {
        public:
        uint32_t getNumIGs_u32() const { return numIGs_u32; }
        fc::InitialGuess &getIGMeans_ro() { return means_o; }
        fc::InitialGuess &getIGStdDevs_ro() { return stdDevs_o; }
        void setNumIGs_v(uint32_t i_Num_u32) { numIGs_u32 = i_Num_u32; }
        void reset_v()
        {
            numIGs_u32 = 0;
            means_o.reset_v();
            stdDevs_o.reset_v();
        }

        private:
        uint32_t numIGs_u32;
        fc::InitialGuess means_o;
        fc::InitialGuess stdDevs_o;
    };

//#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)   // PRQA S 1070
    struct DetailedCalibrationResult
    {
        uint32_t featureCollectionBeginFrame;
        uint32_t featureCollectionEndFrame;
        float32_t calibrationTime;
        const mecl::core::ArrayList < fc::InitialGuess, tsc_cfg::NUM_INITIAL_GUESSES > *pIGs;
        const mecl::core::Array < uint32_t, tsc_cfg::NUM_AVAILABLE_ROIS> *pPerformances;
        tscApi::CalibrationParams_s finalCalibrationResult;
        struct IGConfidenceLevel finalResultConfidenceLevel;
    };
//#endif

    struct FeatureCollectionRestriction_s
    {
        uint32_t maxNumValidFrames_u32;
        uint32_t minNumRawFrames_u32;
    };
}
#endif
