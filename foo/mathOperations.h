// ----------------------------------------------------------------------------
// --- Written by Ehsan Parvizi [17-Dec-2013]
// --- Modified by Ehsan Parvizi [26-Aug-2014]
// --- Modified by Hany Kashif [24-Sep-2014]
// --- Modified by Dmitri Kelbas [24-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __MATHOPERATIONS_H_
#define __MATHOPERATIONS_H_
// ----------------------------------------------------------------------------
#define _USE_MATH_DEFINES
#include <math.h>
#include <typeinfo>
#include "mecl/core/MeclTypes.h"

// ----------------------------------------------------------------------------
// Suppress QACPP MISRA warnings regarding use of floating point
// PRQA S 3708 EOF
// PRQA S 2641 EOF
// PRQA S 2642 EOF

namespace tsc_math
{
    typedef struct {
        sint32_t x_s32;
        sint32_t y_s32;
        sint32_t width_s32;
        sint32_t height_s32;
    } ROIRect;

    typedef struct {
        sint32_t width_s32;
        sint32_t height_s32;
    } ROISize;

    bool_t GetSAD16x16u8(const uint8_t* i_Src_pu8, sint32_t i_SrcStep_s32,
            			 const uint8_t* i_Ref_pu8, sint32_t i_RefStep_s32, uint32_t* o_SAD_pu32 );
    bool_t GetSAD8x8u8( const uint8_t* i_SrcCur_pu8, sint32_t i_SrcCurStep_s32,
            			const uint8_t* i_SrcRef_pu8, sint32_t i_SrcRefStep_s32, uint32_t* o_SAD_pu32 );
    float32_t invsqrtf_c( float32_t i_X_f32 );
    float32_t Correlation16x16Norm_ARM_SW( const uint8_t *i_Source_pu8, const uint8_t *i_Patch_pu8 );
    bool_t CrossCorrValidNormLevel16x16u8( const uint8_t* i_Src_pu8, sint32_t i_SrcStep_s32, ROISize i_SrcRoiSize_t,
            							   const uint8_t* i_Tpl_pu8, sint32_t i_TplStep_s32, ROISize i_TplRoiSize_t, float32_t* o_Dst_pf32 );
    bool_t CrossCorrValidNormLevel1Pass16x16u8( const uint8_t* i_Src_pu8, sint32_t i_SrcStep_s32, ROISize i_SrcRoiSize_t,
            									const uint8_t* i_Tpl_pu8, sint32_t i_TplStep_s32, ROISize i_TplRoiSize_t, float32_t* o_Dst_pf32 );
    bool_t CrossCorrValidNorm16x16u8( const uint8_t* i_Src_pu8, sint32_t i_SrcStep_s32, ROISize i_ScRoiSize_t,
            						  const uint8_t* i_Tpl_pu8, sint32_t i_TplStep_s32, ROISize i_TplRoiSize_t, float32_t* o_Dst_pf32 );
    bool_t ImageROICopy( const uint8_t* i_Src_pu8, sint32_t i_SrcStep_s32, uint8_t* o_Dst_pu8, sint32_t i_DstStep_s32, ROISize i_RoiSize_t);
    bool_t magna_MatrixMultiply(const float64_t *i_Src1_pf64, sint32_t i_Src1Height_s32,
            					sint32_t i_Src1Width_s32, const float64_t *i_Src2_pf64, sint32_t i_Src2Width_s32, float64_t *o_Dst_pf64);
    bool_t magna_transpose( const float64_t* i_Src_pf64, float64_t* o_Dest_pf64, sint32_t i_SrcWidth_s32, sint32_t i_SrcHeight_s32 );
    bool_t MatrixInvert( const float64_t (&i_Src_rf64)[3][3], sint32_t i_WidthHeight_s32, float64_t (&o_Dst_rf64)[3][3] );
    bool_t MatrixVectorMultiply( const float64_t *i_Src1_pf64, sint32_t i_Src1Height_s32, sint32_t i_Src1Width_s32, bool_t i_Src1Transposed_b,
    	    					 const float64_t *i_Src2_pf64, float64_t *o_Dst_pf64 );
    bool_t MatrixMultiply( const float64_t *i_Src1_pf64, sint32_t i_Src1Height_s32, sint32_t i_Src1Width_s32, bool_t i_Src1Transposed_b,
    	    			   const float64_t *i_Src2_pf64, sint32_t i_Src2Height_s32, sint32_t i_Src2Width_s32, bool_t i_Src2Transposed_b, float64_t *o_Dst_pf64 );

    //------------------------------------------------------------------
    extern const uint8_t kTimestampSz;
    extern const float64_t epsilon;

    // --- functions related to angle conversions
    inline float64_t Degrees2Radians( float64_t i_Degrees_f64 )
    {
        return i_Degrees_f64 * M_PI / 180.0;
    }

    inline float64_t Radians2Degrees( float64_t i_Radians_f64 )
    {
        return i_Radians_f64 * 180.0 / M_PI;
    }

    template<typename T>
    void toRange(T& b_Angle_rx, T i_Limit_x)
    {
        // PRQA S 4234 1 //If 'angle' and 'limit' are non-Nan and non-Infinity, there is no problem
        while (b_Angle_rx > i_Limit_x)
        {
            b_Angle_rx -= 2.0f * i_Limit_x;
        }
        // PRQA S 4234 1 //If 'angle' and 'limit' are non-Nan and non-Infinity, there is no problem
        while (b_Angle_rx < -i_Limit_x)
        {
            b_Angle_rx += 2.0f * i_Limit_x;
        }
    }
}
#endif
