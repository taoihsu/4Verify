// ----------------------------------------------------------------------------
// --- Written by Dmitri Kelbas
// --- Modified by Dmitri Kelbas [27-Nov-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __ARM_NEON_H_
#define __ARM_NEON_H_

/*
 *                              -- READ ME FIRST --
 *
 * This file has hand crafted ARM NEON Assembler functions to have a faster platform dependent version of
 * their generic C/C++ implementation. These functions were tested on Xilinx Zynq-702 board.
 *
 * Some functions have multiple implementations with balance between accuracy and execution speed in mind.
 * Some of functions are implemented with suffix _orig with human readable logically ordered assembler
 * instructions. Some are also implemented with suffix _opt with reordered instructions to increase throughput
 * of NEON execution pipelines.
 *
 */

// ----------------------------------------------------------------------------
#include "mecl/core/MeclTypes.h"
// This commented code may be needed when testing with linux?.
// PRQA S 1051 1
// #include <sys/mman.h>
// ----------------------------------------------------------------------------
// This commented code is uncommented and used when NEON is turned on.
// PRQA S 1051 1
// #define BMA_OPT_BY_ARM_NEON

#ifdef BMA_OPT_BY_ARM_NEON
namespace tsc_math_neon
{
    // -- Block matching --
    // Block matching based on Sum-of-Absolute-Differences, readable
    void BlockMatch16x16_ARM_NEON_orig(const uint32_t *source, const uint32_t *patch, uint32_t *sad);
    // Block matching based on Sum-of-Absolute-Differences, optimised
    void BlockMatch16x16_ARM_NEON_opt(const uint32_t *source, const uint32_t *patch, uint32_t *sad);

    // -- Cross Correlation --
    // Cross Correlation function with normalisation (one-pass), readable
    void Correlation16x16Norm_ARM_NEON_orig(const uint32_t *source, const uint32_t *patch, float32_t *sad);
    // Cross Correlation function with normalisation (one-pass), optimised
    // -- not implemented --

    // Cross Correlation function with normalisation and levelling (mean value extraction), (two-pass), readable
    void CrossCorr16x16_Mean_F32_ARM_NEON_orig(const uint32_t *source, const uint32_t *patch, float32_t *meanSrc, float32_t *meanPatch, float32_t recipr256);
    void CrossCorr16x16_NormLevel_I32_ARM_NEON_orig(const uint32_t *source, const uint32_t *patch, uint32_t *meanSrc, uint32_t *meanPatch, float32_t *sad);
    void CrossCorr16x16_NormLevel_Float_ARM_NEON_orig(const uint32_t *source, const uint32_t *patch, float32_t *meanSrc, float32_t *meanPatch, float32_t *sad);
    void CrossCorr16x16_NormLevel_I32_1Pass_ARM_NEON_orig(const uint32_t *source, const uint32_t *patch, float32_t *sad);

    // Cross Correlation function with normalisation and levelling (mean value extraction), (two-pass), optimised
    void CrossCorr16x16_Mean_F32_ARM_NEON_opt(const uint32_t *source, const uint32_t *patch, float32_t *meanSrc, float32_t *meanPatch, float32_t recipr256);
    void CrossCorr16x16_NormLevel_Float_ARM_NEON_opt(const uint32_t *source, const uint32_t *patch, float32_t *meanSrc, float32_t *meanPatch, float32_t *sad);
    void CrossCorr16x16_NormLevel_I32_1Pass_ARM_NEON_opt(const uint32_t *source, const uint32_t *patch, float32_t *sad);
}
#endif
#endif
