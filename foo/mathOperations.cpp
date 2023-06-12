// ----------------------------------------------------------------------------
// --- Written by Cindy [31-Oct-2013]
// --- Modified by Dmitri Kelbas [05-Nov-2014]
// --- Modified by Dmitri Kelbas [27-Nov-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include <string.h>  // memcpy
#include "mecl/mecl.h"
#include "mecl/core/MeclAssert.h"
#include "mathOperations.h"
// PRQA S 3708 EOF

namespace tsc_math {

const uint8_t kTimestampSz = 32;
// PRQA S 4636 1 // as long as float64_t is a fundamental arithmetic type, an exception should not be thrown.
const float64_t epsilon = mecl::math::numeric_limits<float64_t>::epsilon_x();

//-----------------------
//pSrcCur       -Pointer to an 8x8 block in the source plane
//srcCurStep    -Distance in bytes between starts of the consecutive lines in the source image
//pSrcRef       -Pointer to an 8x8 block in the reference plane
//srcRefStep    -Distance in bytes between starts of the consecutive lines in the reference image
//pDst          -Pointer to the SAD value
//mcType        -MC type IPPVC_MC_APX
//-----------------------
bool_t GetSAD8x8u8(const uint8_t* i_SrcCur_pu8, sint32_t i_SrcCurStep_s32,
                   const uint8_t* i_SrcRef_pu8, sint32_t i_SrcRefStep_s32, uint32_t* o_SAD_pu32)
{
    uint32_t v_Currsad_u32=0;
    const uint8_t* c_SrcThisLine_pu8;
    const uint8_t* c_RefThisLine_pu8;

    // for every line of 8x8 patch
    for(uint8_t v_Line_u8=0; v_Line_u8 < 8; v_Line_u8++)
    {
        c_SrcThisLine_pu8 = i_SrcCur_pu8;
        c_RefThisLine_pu8 = i_SrcRef_pu8;
        // for every element in current line of patch
        for(uint8_t v_Col_u8=0; v_Col_u8 < 8; v_Col_u8++)
        {
            v_Currsad_u32 += static_cast<uint32_t>(/*std::*/mecl::math::abs_x<float32_t>( static_cast<float32_t>((*c_RefThisLine_pu8) - (*c_SrcThisLine_pu8))));
            c_SrcThisLine_pu8++;
            c_RefThisLine_pu8++;
        }
        i_SrcRef_pu8 += i_SrcRefStep_s32;
        i_SrcCur_pu8 += i_SrcCurStep_s32;
    }
    *o_SAD_pu32 = v_Currsad_u32;
    return true;
}

bool_t GetSAD16x16u8(const uint8_t* i_Src_pu8, sint32_t i_SrcStep_s32,
                     const uint8_t* i_Ref_pu8, sint32_t i_RefStep_s32, uint32_t* o_SAD_pu32)
{
    uint32_t v_Currsad_u32=0;
    const uint8_t* c_SrcThisLine_pu8;
    const uint8_t* c_RefThisLine_pu8;

    // for every line of 8x8 patch
    for(uint8_t v_Line_u8=0; v_Line_u8 < 16; v_Line_u8++)
    {
        c_SrcThisLine_pu8 = i_Src_pu8;
        c_RefThisLine_pu8 = i_Ref_pu8;
        // for every element in current line of patch
        for(uint8_t v_Col_u8=0; v_Col_u8 < 16; v_Col_u8++)
        {
            v_Currsad_u32 += static_cast<uint32_t>(/*std::*/mecl::math::abs_x<float32_t>(static_cast<float32_t>((*c_RefThisLine_pu8) - (*c_SrcThisLine_pu8))));
            c_SrcThisLine_pu8++;
            c_RefThisLine_pu8++;
        }
        i_Ref_pu8 += i_RefStep_s32;
        i_Src_pu8 += i_SrcStep_s32;
    }
    *o_SAD_pu32 = v_Currsad_u32;
    return true;
}

float32_t invsqrtf_c(float32_t i_X_f32) {

  float32_t v_B_f32;
  float32_t v_C_f32;
  float32_t v_F_f32;
  float32_t& v_RF_rf32 = v_F_f32;
  // PRQA S 3017 1 //It's intended to manipulate bits here.
  uint32_t& v_T_ru32 = reinterpret_cast<uint32_t&>(v_RF_rf32);

  //fast invsqrt approx
  v_F_f32 = i_X_f32;
  v_T_ru32 = 0x5F3759DF - (v_T_ru32 >> 1);
  v_C_f32 = i_X_f32 * v_F_f32;
  v_B_f32 = (3.0F - v_C_f32 * v_F_f32) * 0.5;             //VRSQRTS
  v_F_f32 = v_F_f32 * v_B_f32;
  v_C_f32 = i_X_f32 * v_F_f32;
  v_B_f32 = (3.0F - v_C_f32 * v_F_f32) * 0.5;
  v_F_f32 = v_F_f32 * v_B_f32;

  return v_F_f32;
}

inline float32_t Correlation16x16Norm_ARM_SW(
    const uint8_t *i_Source_pu8,
    const uint8_t *i_Patch_pu8)
{
    uint32_t v_Corr_u32 = 0;
    uint32_t v_Sqsum1_u32 = 0;
    uint32_t v_Sqsum2_u32 = 0;
    uint8_t v_X_u8 = 0;
    uint8_t v_Y_u8 = 0; // current position of 16x16 patch on 46x46 image
    float32_t v_Normcorr_f32 = 0.0;
    float32_t v_Invsqroot1_f32;
    float32_t v_Invsqroot2_f32;

    for (v_Y_u8 = 0; v_Y_u8 < 16; v_Y_u8++) {
        for (v_X_u8 = 0; v_X_u8 < 16; v_X_u8++) {
            v_Corr_u32 += (*i_Source_pu8) * (*i_Patch_pu8);
            v_Sqsum1_u32 += (*i_Source_pu8) * (*i_Source_pu8);
            v_Sqsum2_u32 += (*i_Patch_pu8) * (*i_Patch_pu8);
            i_Source_pu8++;
            i_Patch_pu8++;
        }
        i_Source_pu8 += (640 - 16); // it was just incremented 2 lines above, so 640-16=624
    }
    v_Invsqroot1_f32 = invsqrtf_c(static_cast<float32_t>(v_Sqsum1_u32));
    v_Invsqroot2_f32 = invsqrtf_c(static_cast<float32_t>(v_Sqsum2_u32));
    v_Normcorr_f32 = static_cast<float32_t>(v_Corr_u32) * v_Invsqroot1_f32 * v_Invsqroot2_f32;

    return (v_Normcorr_f32);
}

/*
//  Purpose: Computes normalized correlation coefficient between an image and a template.
//           ippiCrossCorr_NormLevel() function allows you to compute the
//           cross-correlation of an image and a template (another image).
//           The cross-correlation values are image similarity measures: the
//           higher cross-correlation at a particular pixel, the more
//           similarity between the template and the image in the neighborhood
//           of the pixel. If IppiSize's of image and template are Wa * Ha and
//           Wb * Hb correspondingly, then the IppiSize of the resulting
//           matrix with normalized cross-correlation coefficients will be in case of 'Valid' suffix:
//              ( Wa - Wb + 1 )*( Ha - Hb + 1 ).
//  Notice:
//           suffix 'R' (ROI) means only scanline alignment (srcStep).
//           The difference from ippiCrossCorr_Norm() functions is the using
//           of Zero Mean image and Template to avoid brightness impact.
//           (Before the calculation of the cross-correlation coefficients,
//           the mean of the image in the region under the feature is subtracted
//           from every image pixel; the same for the template.)
//
//  Parameters:
//      pSrc        Pointer to the source image ROI;
//      srcStep     Step in bytes through the source image buffer;
//      srcRoiSize  Size of the source ROI in pixels;
//      pTpl        Pointer to the template ( feature ) image ROI;
//      tplStep     Step in bytes through the template image buffer;
//      tplRoiSize  Size of the template ROI in pixels;
//      pDst        Pointer to the destination buffer;
//      dstStep     Step in bytes through the destination image buffer;
//      scaleFactor Scale factor value ( integer output data ).
//
//  Returns:
//   ippStsNoErr        OK
//   ippStsNullPtrErr   One of the pointers to pSrc, pDst or pTpl is NULL;
//   ippStsSizeErr      srcRoiSize or tplRoiSize has a field with zero or
//                      negative value,
//                      or srcRoiSize has a field with value smaller than value
//                      of the corresponding field of tplRoiSize;
//   ippStsStepErr      One of the step values is less than or equal to zero;
//   ippStsMemAllocErr  Memory allocation for internal buffers fails.
*/

// Implementation with mean extraction and normalization (to reflect ippi implementation)
bool_t CrossCorrValidNormLevel16x16u8(
        const uint8_t* i_Src_pu8, sint32_t i_SrcStep_s32, ROISize i_SrcRoiSize_t,
        const uint8_t* i_Tpl_pu8, sint32_t i_TplStep_s32, ROISize i_TplRoiSize_t,
        float32_t* o_Dst_pf32 )
{
    const uint8_t *c_Source_pu8 = i_Tpl_pu8;
    const uint8_t *c_Patch_pu8 = i_Src_pu8;
    uint8_t v_X_u8 = 0;
    uint8_t v_Y_u8 = 0; // current position of 16x16 patch on 46x46 image
    float32_t v_Normcorr_f32 = 0.0;
    float32_t v_Invsqroot1_f32=0.0;
    float32_t v_Invsqroot2_f32=0.0;
    float32_t v_MeanSource_f32=0.0;
    float32_t v_MeanPatch_f32=0.0;
    float32_t v_Sqsum1_f32 = 0.0;
    float32_t v_Sqsum2_f32 = 0.0;
    float32_t v_Corr_f32 = 0.0;

    Assert(i_SrcRoiSize_t.height_s32 == 16);
    Assert(i_SrcRoiSize_t.width_s32 == 16);
    Assert(i_TplRoiSize_t.height_s32 == 16);
    Assert(i_TplRoiSize_t.width_s32 == 16);
    Assert(i_SrcStep_s32 == 16);
    Assert(i_TplStep_s32 == 640);

    // compute mean value for template and for source image
    for (v_Y_u8 = 0; v_Y_u8 < 16; v_Y_u8++) {
        for (v_X_u8 = 0; v_X_u8 < 16; v_X_u8++) {
            v_MeanSource_f32 += static_cast<float32_t>(*c_Source_pu8);
            v_MeanPatch_f32  += static_cast<float32_t>(*c_Patch_pu8);
            c_Source_pu8++;
            c_Patch_pu8++;
        }
        c_Source_pu8 += (640 - 16); // it was just incremented 2 lines above, so 640-16=624
    }
    v_MeanSource_f32 /= static_cast<float32_t>(16*16);
    v_MeanPatch_f32  /= static_cast<float32_t>(16*16);

    c_Source_pu8 = i_Tpl_pu8;
    c_Patch_pu8 = i_Src_pu8;

    // compute normalized cross-correlation value between template and image
    for (v_Y_u8 = 0; v_Y_u8 < 16; v_Y_u8++) {
        for (v_X_u8 = 0; v_X_u8 < 16; v_X_u8++) {
            float32_t v_SourceMeanExtr_f32;
            float32_t v_PatchMeanExtr_f32;

            v_SourceMeanExtr_f32 = static_cast<float32_t>(*c_Source_pu8) - v_MeanSource_f32;
            v_PatchMeanExtr_f32 = static_cast<float32_t>(*c_Patch_pu8) - v_MeanPatch_f32;

            v_Corr_f32 += (v_SourceMeanExtr_f32) * (v_PatchMeanExtr_f32);
            v_Sqsum1_f32 += (v_SourceMeanExtr_f32) * (v_SourceMeanExtr_f32);
            v_Sqsum2_f32 += (v_PatchMeanExtr_f32) * (v_PatchMeanExtr_f32);
            c_Source_pu8++;
            c_Patch_pu8++;
        }
        c_Source_pu8 += (640 - 16); // it was just incremented 2 lines above, so 640-16=624
    }
    v_Invsqroot1_f32 = invsqrtf_c(static_cast<float32_t>(static_cast<uint32_t>(v_Sqsum1_f32)));
    v_Invsqroot2_f32 = invsqrtf_c(static_cast<float32_t>(static_cast<uint32_t>(v_Sqsum2_f32)));
    v_Normcorr_f32 = v_Corr_f32 * v_Invsqroot1_f32 * v_Invsqroot2_f32;

    *o_Dst_pf32 = static_cast<float32_t>(v_Normcorr_f32);

    return true;
}

// Implementation with mean extraction and normalisation (to reflect ippi implementation)
// the one - pass approach is used
// "sum(Xm* yi)=Xm*sum(yi)=256*Xm*Ym, same as sum(Ym*xi)=Ym*sum(xi)=256*Xm*Ym", Na
//
bool_t CrossCorrValidNormLevel1Pass16x16u8(
        const uint8_t* i_Src_pu8, sint32_t i_SrcStep_s32, ROISize i_SrcRoiSize_t,
        const uint8_t* i_Tpl_pu8, sint32_t i_TplStep_s32, ROISize i_TplRoiSize_t,
        float32_t* o_Dst_pf32 )
{
    const uint8_t *c_Source_pu8 = i_Tpl_pu8;
    const uint8_t *c_Patch_pu8 = i_Src_pu8;
    uint32_t v_X_u32 = 0;
    uint32_t v_Y_u32 = 0; // current position of 16x16 patch on 46x46 image
    uint32_t v_Sumxy_u32=0;
    uint32_t v_Sumx_u32=0;
    uint32_t v_Sumy_u32=0;
    uint32_t v_Sqsumx_u32 = 0;
    uint32_t v_Sqsumy_u32 = 0;
    uint32_t v_Meanxsq256_u32 = 0;
    uint32_t v_Meanysq256_u32 = 0;
    uint32_t v_Meanxmeany256_u32 = 0;
    sint32_t v_Signedsumxy_s32 = 0;
    sint32_t v_Signedsqsumx_s32 = 0;
    sint32_t v_Signedsqsumy_s32 = 0;

    float32_t v_Normcorr_f32 = 0.0;
    float32_t v_Invsqroot1_f32=0.0;
    float32_t v_Invsqroot2_f32=0.0;

    Assert(i_SrcRoiSize_t.height_s32 == 16);
    Assert(i_SrcRoiSize_t.width_s32 == 16);
    Assert(i_TplRoiSize_t.height_s32 == 16);
    Assert(i_TplRoiSize_t.width_s32 == 16);
    Assert(i_SrcStep_s32 == 16);
    Assert(i_TplStep_s32 == 640);

    // compute normalized cross-correlation value between template and image
    for (v_Y_u32 = 0; v_Y_u32 < 16; v_Y_u32++) {
        for (v_X_u32 = 0; v_X_u32 < 16; v_X_u32++) {
            uint8_t v_SourceVal_u8;
            uint8_t v_PatchVal_u8;

            v_SourceVal_u8 = *c_Source_pu8;
            v_PatchVal_u8 = *c_Patch_pu8;

            v_Sumxy_u32 += (v_SourceVal_u8) * (v_PatchVal_u8);	    // total (8+8+8)bit = 24bit
            v_Sqsumx_u32 += (v_SourceVal_u8) * (v_SourceVal_u8);	// total (8+8+8)bit = 24bit
            v_Sqsumy_u32 += (v_PatchVal_u8) * (v_PatchVal_u8);	    // total (8+8+8)bit = 24bit
            v_Sumx_u32 += v_SourceVal_u8;	    				// total (8+8)bit = 16bit
            v_Sumy_u32 += v_PatchVal_u8;	    				// total (8+8)bit = 16bit
            c_Source_pu8++;
            c_Patch_pu8++;
        }
        c_Source_pu8 += (640 - 16); // it was just incremented 2 lines above, so 640-16=624
    }

    v_Meanxsq256_u32 = (v_Sumx_u32*v_Sumx_u32)>>8; 	// (sumx>>8)*(sumx>>8)<<8 = (sumx*sumx)>>8, total ((16+16)-8)=24bit
    v_Meanysq256_u32 = (v_Sumy_u32*v_Sumy_u32)>>8; 	// (sumy>>8)*(sumy>>8)<<8 = (sumy*sumy)>>8, total ((16+16)-8)=24bit
    v_Meanxmeany256_u32 = (v_Sumx_u32*v_Sumy_u32)>>8; // (sumx>>8)*(sumy>>8)<<8 = (sumx*sumy)>>8, total ((16+16)-8)=24bit

    v_Signedsumxy_s32  = v_Sumxy_u32  - v_Meanxmeany256_u32;
    v_Signedsqsumx_s32 = v_Sqsumx_u32 - v_Meanxsq256_u32;
    v_Signedsqsumy_s32 = v_Sqsumy_u32 - v_Meanysq256_u32;

    if( v_Signedsumxy_s32  < 0 ) { v_Signedsumxy_s32  = -v_Signedsumxy_s32; }
    if( v_Signedsqsumx_s32 < 0 ) { v_Signedsqsumx_s32 = -v_Signedsqsumx_s32; }
    if( v_Signedsqsumy_s32 < 0 ) { v_Signedsqsumy_s32 = -v_Signedsqsumy_s32; }

    v_Invsqroot1_f32 = invsqrtf_c(static_cast<float32_t>(v_Signedsqsumx_s32));
    v_Invsqroot2_f32 = invsqrtf_c(static_cast<float32_t>(v_Signedsqsumy_s32));
    v_Normcorr_f32 = static_cast<float32_t>(v_Signedsumxy_s32) * v_Invsqroot1_f32 * v_Invsqroot2_f32;

    *o_Dst_pf32 = static_cast<float32_t>(v_Normcorr_f32);

    return true;
}


// Implementation with normalization (to speed up execution)
bool_t CrossCorrValidNorm16x16u8(
        const uint8_t* i_Src_pu8, sint32_t i_SrcStep_s32, ROISize i_ScRoiSize_t,
        const uint8_t* i_Tpl_pu8, sint32_t i_TplStep_s32, ROISize i_TplRoiSize_t,
        float32_t* o_Dst_pf32 )
{
    const uint8_t *c_Source_pu8 = i_Tpl_pu8;
    const uint8_t *c_Patch_pu8 = i_Src_pu8;
    uint32_t v_Corr_u32 = 0;
    uint32_t v_Sqsum1_u32 = 0;
    uint32_t v_Sqsum2_u32 = 0;
    uint32_t v_X_u32 = 0;
    uint32_t v_Y_u32 = 0; // current position of 16x16 patch on 46x46 image
    float32_t v_Normcorr_f32 = 0.0;
    float32_t v_Invsqroot1_f32;
    float32_t v_Invsqroot2_f32;

    Assert(i_ScRoiSize_t.height_s32 == 16);
    Assert(i_ScRoiSize_t.width_s32 == 16);
    Assert(i_TplRoiSize_t.height_s32 == 16);
    Assert(i_TplRoiSize_t.width_s32 == 16);
    Assert(i_SrcStep_s32 == 16);
    Assert(i_TplStep_s32 == 640);

    // compute normalized cross-correlation value between template and image
    for (v_Y_u32 = 0; v_Y_u32 < 16; v_Y_u32++) {
        for (v_X_u32 = 0; v_X_u32 < 16; v_X_u32++) {
            v_Corr_u32 += (*c_Source_pu8) * (*c_Patch_pu8);
            v_Sqsum1_u32 += (*c_Source_pu8) * (*c_Source_pu8);
            v_Sqsum2_u32 += (*c_Patch_pu8) * (*c_Patch_pu8);
            c_Source_pu8++;
            c_Patch_pu8++;
        }
        c_Source_pu8 += (640 - 16); // it was just incremented 2 lines above, so 640-16=624
    }
    v_Invsqroot1_f32 = invsqrtf_c(static_cast<float32_t>(v_Sqsum1_u32));
    v_Invsqroot2_f32 = invsqrtf_c(static_cast<float32_t>(v_Sqsum2_u32));
    v_Normcorr_f32 = static_cast<float32_t>(v_Corr_u32) * v_Invsqroot1_f32 * v_Invsqroot2_f32;

    *o_Dst_pf32 = static_cast<float32_t>(v_Normcorr_f32);

    return true;
}

bool_t ImageROICopy(const uint8_t* i_Src_pu8, sint32_t i_SrcStep_s32, uint8_t* o_Dst_pu8, sint32_t i_DstStep_s32, ROISize i_RoiSize_t)
{
    // for every line of ROI
    for(uint16_t v_Line_u16=0; v_Line_u16 < i_RoiSize_t.height_s32; v_Line_u16++)
    {
        // for every byte in current line inside ROI copy it to destination
        memcpy(o_Dst_pu8, i_Src_pu8, i_RoiSize_t.width_s32);
        o_Dst_pu8 += i_DstStep_s32;
        i_Src_pu8 += i_SrcStep_s32;
    }
    return true;
}

// ----------------------------------------------------------------------------
// --- Matrix Operations

bool_t magna_MatrixMultiply(const float64_t *i_Src1_pf64, sint32_t i_Src1Height_s32,
        sint32_t i_Src1Width_s32, const float64_t *i_Src2_pf64, sint32_t i_Src2Width_s32,
        float64_t *o_Dst_pf64) {

    sint32_t v_Index_s32;
    sint32_t v_InnerIndex_s32;
    sint32_t v_IndexK_s32;

    float64_t *v_C_pf64 = o_Dst_pf64;

    // implement C=A*B c(i,j) = sum( a(i,k) * b(k,j) ), k from 0 to src1Width-1
    for (v_Index_s32 = 0; v_Index_s32 < i_Src1Height_s32; v_Index_s32++) {
        for (v_InnerIndex_s32 = 0; v_InnerIndex_s32 < i_Src2Width_s32; v_InnerIndex_s32++) {
            const float64_t *c_A_pf64 = i_Src1_pf64;
            const float64_t *c_B_pf64 = i_Src2_pf64;
            float64_t v_Sum_f64 = 0;

            c_B_pf64 += v_InnerIndex_s32;
            c_A_pf64 += v_Index_s32 * i_Src1Width_s32;
            for (v_IndexK_s32 = 0; v_IndexK_s32 < i_Src1Width_s32; v_IndexK_s32++) {
                v_Sum_f64 += (*c_A_pf64) * (*c_B_pf64);
                c_A_pf64++;
                c_B_pf64 += i_Src2Width_s32;
            }
            (*v_C_pf64) = v_Sum_f64;
            v_C_pf64++;
        }
    }
    return true;
}

// --- magna_transpose (any size and aspect ratio)
 bool_t magna_transpose(const float64_t* i_Src_pf64, float64_t* o_Dest_pf64, sint32_t i_SrcWidth_s32,
        sint32_t i_SrcHeight_s32) {
    bool_t v_Ret_b = true;
    size_t v_Len_t = i_SrcWidth_s32 * i_SrcHeight_s32;   // length in double words
    size_t v_Len1_t = v_Len_t - 1;                 // length in double words

    const float64_t *c_SrcEnd_pf64 = i_Src_pf64;         // length in double words
    c_SrcEnd_pf64 += v_Len_t;
    float64_t *v_DestTmp_pf64 = o_Dest_pf64;
    float64_t *v_DestEnd_pf64 = v_DestTmp_pf64;  // length in double words
    v_DestEnd_pf64 += v_Len_t;

    Assert( i_Src_pf64 != o_Dest_pf64 );  // not support in-place transpose
    if (v_DestTmp_pf64 == NULL) {
        v_Ret_b = false;
    }
    else
    {
        while (v_DestTmp_pf64 < v_DestEnd_pf64) {
            *v_DestTmp_pf64 = *i_Src_pf64;  // dest moves to next row
            v_DestTmp_pf64 ++;
            i_Src_pf64 += i_SrcHeight_s32;   // src moves to next column

            // src wraps around and moves to next column
            if (i_Src_pf64 >= c_SrcEnd_pf64)
            {
                i_Src_pf64 -= v_Len1_t;
            }
        }
    }
    return v_Ret_b;
}

// --- MatrixInvert
bool_t MatrixInvert( const float64_t (&i_Src_rf64)[3][3], sint32_t i_WidthHeight_s32, float64_t (&o_Dst_rf64)[3][3] )
{
    bool_t v_Ret_b = true;

    if (i_WidthHeight_s32 != 3) {
        v_Ret_b = false;    // matrix size is not 3x3
    }
    else
    {
        float64_t v_Determinant_f64;
        v_Determinant_f64 = + i_Src_rf64[0][0]
                      * ( i_Src_rf64[1][1] * i_Src_rf64[2][2] - i_Src_rf64[2][1] * i_Src_rf64[1][2] )
                      - i_Src_rf64[0][1] * ( i_Src_rf64[1][0] * i_Src_rf64[2][2] - i_Src_rf64[1][2] * i_Src_rf64[2][0] )
                      + i_Src_rf64[0][2] * ( i_Src_rf64[1][0] * i_Src_rf64[2][1] - i_Src_rf64[1][1] * i_Src_rf64[2][0] );

        if (mecl::math::abs_x<float64_t>(v_Determinant_f64) < epsilon) // determinant is too close to ZERO
        {
            v_Ret_b = false;
        }
        else
        {

            float64_t v_Invdet_f64 = 1.0 / v_Determinant_f64;

            o_Dst_rf64[0][0] =  ( i_Src_rf64[1][1] * i_Src_rf64[2][2] - i_Src_rf64[2][1] * i_Src_rf64[1][2] ) * v_Invdet_f64;
            o_Dst_rf64[0][1] = -( i_Src_rf64[0][1] * i_Src_rf64[2][2] - i_Src_rf64[0][2] * i_Src_rf64[2][1] ) * v_Invdet_f64;
            o_Dst_rf64[0][2] =  ( i_Src_rf64[0][1] * i_Src_rf64[1][2] - i_Src_rf64[0][2] * i_Src_rf64[1][1] ) * v_Invdet_f64;

            o_Dst_rf64[1][0] = -( i_Src_rf64[1][0] * i_Src_rf64[2][2] - i_Src_rf64[1][2] * i_Src_rf64[2][0] ) * v_Invdet_f64;
            o_Dst_rf64[1][1] =  ( i_Src_rf64[0][0] * i_Src_rf64[2][2] - i_Src_rf64[0][2] * i_Src_rf64[2][0] ) * v_Invdet_f64;
            o_Dst_rf64[1][2] = -( i_Src_rf64[0][0] * i_Src_rf64[1][2] - i_Src_rf64[1][0] * i_Src_rf64[0][2] ) * v_Invdet_f64;

            o_Dst_rf64[2][0] =  ( i_Src_rf64[1][0] * i_Src_rf64[2][1] - i_Src_rf64[2][0] * i_Src_rf64[1][1] ) * v_Invdet_f64;
            o_Dst_rf64[2][1] = -( i_Src_rf64[0][0] * i_Src_rf64[2][1] - i_Src_rf64[2][0] * i_Src_rf64[0][1] ) * v_Invdet_f64;
            o_Dst_rf64[2][2] =  ( i_Src_rf64[0][0] * i_Src_rf64[1][1] - i_Src_rf64[1][0] * i_Src_rf64[0][1] ) * v_Invdet_f64;
        }
    }
    return v_Ret_b;
}

/*
 * Disclaimer: With bundle adjustment disabled, matrix multiply functions
 * are called with matrices of size 4*4 at most. Hence, mallocs in these
 * functions were replaced with arrays created on the functions' stacks.
 * Bundle adjustment calls these functions for huge matrics. This will
 * require statically allocating memory for the required arrays instead.
 * >> Be careful when introducing any new calls to these functions. The
 * size of matrices for which they are being called need to be examined
 * closely.
 */

// --- MatrixVectorMultiply
bool_t MatrixVectorMultiply(
    const float64_t *i_Src1_pf64, sint32_t i_Src1Height_s32, sint32_t i_Src1Width_s32, bool_t i_Src1Transposed_b,
    const float64_t *i_Src2_pf64, float64_t *o_Dst_pf64 )
{
    bool_t v_Ret_b = true;
    float64_t *v_DstTmp_pf64;

    if ((i_Src1Height_s32 > 4) || (i_Src1Width_s32 > 4)) {
        v_Ret_b = false;
    }
    if ((o_Dst_pf64 == i_Src1_pf64) || (o_Dst_pf64 == i_Src2_pf64)) {
        float64_t v_DstTmp_af64[4];
        v_DstTmp_pf64 = v_DstTmp_af64;
    } else {
        v_DstTmp_pf64 = o_Dst_pf64;
    }

    const float64_t* c_A_pf64 = i_Src1_pf64;
    const float64_t* c_B_pf64 = i_Src2_pf64;
    float64_t v_ATrnspArray_af64[4*4];
    float64_t* v_ATrnsp_pf64 = v_ATrnspArray_af64;

    if (i_Src1Transposed_b) {
        // --- dst = src1' x src2
        magna_transpose(c_A_pf64, v_ATrnsp_pf64, i_Src1Width_s32, i_Src1Height_s32);
        magna_MatrixMultiply(v_ATrnsp_pf64, i_Src1Width_s32, i_Src1Height_s32, c_B_pf64,    1, v_DstTmp_pf64);
    } else {
        // --- dst = src1 x src2
        magna_MatrixMultiply(c_A_pf64, i_Src1Height_s32, i_Src1Width_s32, c_B_pf64, 1, v_DstTmp_pf64);
    }

    if ((o_Dst_pf64 == i_Src1_pf64) || (o_Dst_pf64 == i_Src2_pf64)) {
        memcpy(o_Dst_pf64, v_DstTmp_pf64, sizeof(float64_t) * i_Src1Height_s32 * 1);
    }

    return v_Ret_b;
}

// --- MatrixMultiply
 bool_t MatrixMultiply(
    const float64_t *i_Src1_pf64, sint32_t i_Src1Height_s32, sint32_t i_Src1Width_s32, bool_t i_Src1Transposed_b,
    const float64_t *i_Src2_pf64, sint32_t i_Src2Height_s32, sint32_t i_Src2Width_s32, bool_t i_Src2Transposed_b,
    float64_t *o_Dst_pf64 )
{
    bool_t v_Ret_b = true;
    float64_t *v_DstTmp_pf64;

    if ((i_Src1Height_s32 > 4) || (i_Src1Width_s32 > 4) || (i_Src2Height_s32 >4) || (i_Src2Width_s32 > 4)) {
        v_Ret_b = false;
    }
    if ((o_Dst_pf64 == i_Src1_pf64) || (o_Dst_pf64 == i_Src2_pf64)) {
        float64_t v_DstTmp_af64[4*4];
        v_DstTmp_pf64 = v_DstTmp_af64;
    } else {
        v_DstTmp_pf64 = o_Dst_pf64;
    }

    const float64_t* c_A_pf64 = i_Src1_pf64;
    const float64_t* c_B_pf64 = i_Src2_pf64;
    float64_t v_ATrnspArray_af64[4*4];
    float64_t* v_ATrnsp_pf64 = v_ATrnspArray_af64;
    float64_t v_BTrnspArray_af64[4*4];
    float64_t* v_BTrnsp_pf64 = v_BTrnspArray_af64;

    if (i_Src1Transposed_b) {
        if (i_Src2Transposed_b) {
            // --- dst = src1' x src2'
            magna_transpose(c_A_pf64, v_ATrnsp_pf64, i_Src1Width_s32, i_Src1Height_s32);
            magna_transpose(c_B_pf64, v_BTrnsp_pf64, i_Src2Width_s32, i_Src2Height_s32);
            magna_MatrixMultiply(v_ATrnsp_pf64, i_Src1Width_s32, i_Src1Height_s32, v_BTrnsp_pf64, i_Src2Height_s32, v_DstTmp_pf64);
        } else {
            // --- dst = src1' x src2
            magna_transpose(c_A_pf64, v_ATrnsp_pf64, i_Src1Width_s32, i_Src1Height_s32);
            magna_MatrixMultiply(v_ATrnsp_pf64, i_Src1Width_s32, i_Src1Height_s32, c_B_pf64,    i_Src2Width_s32, v_DstTmp_pf64);
        }
    } else {
        if (i_Src2Transposed_b) {
            // --- dst = src1 x src2'
            magna_transpose(c_B_pf64, v_BTrnsp_pf64, i_Src2Width_s32, i_Src2Height_s32);
            magna_MatrixMultiply(c_A_pf64, i_Src1Height_s32, i_Src1Width_s32, v_BTrnsp_pf64,    i_Src2Height_s32, v_DstTmp_pf64);
        } else {
            // --- dst = src1 x src2
            magna_MatrixMultiply(c_A_pf64, i_Src1Height_s32, i_Src1Width_s32, c_B_pf64, i_Src2Width_s32, v_DstTmp_pf64);
        }
    }

    if ((o_Dst_pf64 == i_Src1_pf64) || (o_Dst_pf64 == i_Src2_pf64)) {
        memcpy(o_Dst_pf64, v_DstTmp_pf64, sizeof(float64_t) * i_Src1Height_s32 * i_Src2Width_s32);
    }

    return v_Ret_b;
}
}
