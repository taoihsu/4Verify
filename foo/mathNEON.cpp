/*--------------------------------------------------------------------------
 @file mathNEON.c
 @brief Contains the NEON code for template matching


 --------------------------------------------------------------------------
 @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
 This document in its entirety is CONFIDENTIAL and may not be disclosed,
 disseminated or distributed to parties outside MAGNA Electronics
 without written permission from MAGNA Electronics.

 @author Detlef Hafer (Detlef.Hafer.Extern@magna.com)

 --------------------------------------------------------------------------*/
#include "stdafx.h"

#ifdef __ARM_NEON__

#include "mathNEON.h"


#ifdef WIN32
    #include "NEONvsSSE.h"
#else
    #include "arm_neon.h"
#endif

void CrossBlock_NEON8_v(
    const uint8_t* i_Src_pu8, uint32_t i_SrcStep_u32,
    const uint8_t* i_Tpl_pu8, uint32_t i_TplStep_u32,
    ResultVector_t* v_ResultVector_pt )
{
    uint32x4_t  v_Src_Sumz = vdupq_n_u32( 0 );
    uint32x4_t  v_Tpl_Sumz = vdupq_n_u32( 0 );
    uint32x4_t  v_XY_Sumz  = vdupq_n_u32( 0 );
    uint32x2_t  v_Src_Sums = vdup_n_u32( 0 );
    uint32x2_t  v_Tpl_Sums = vdup_n_u32( 0 );

    for( uint16_t v_Y_u16 = 0; v_Y_u16 < 8U; v_Y_u16++ )
    {
        // v_Src_Sumz !
        uint8x8_t  v_Src_s0 = vld1_u8( i_Src_pu8 );
        // v_Src_Sums += i_Src_pu8
        uint16x4_t c_Src_s1 = vpaddl_u8( v_Src_s0 );
        uint32x2_t c_Src_s2 = vpaddl_u16( c_Src_s1 );
        v_Src_Sums          = vadd_u32( v_Src_Sums, c_Src_s2 );
        // v_Src_Sumz += i_Src_pu8 * i_Src_pu8;
        uint16x8_t v_Src_z0  = vmull_u8( v_Src_s0, v_Src_s0 );
        uint32x4_t v_Src_z1  = vpaddlq_u16( v_Src_z0 );
        v_Src_Sumz           = vaddq_u32( v_Src_Sumz, v_Src_z1 );
        // v_Tpl_Sumz !
        uint8x8_t  v_Tpl_s0 = vld1_u8( i_Tpl_pu8 );
        // v_Tpl_Sums += i_Tpl_pu8
        uint16x4_t v_Tpl_s1 = vpaddl_u8( v_Tpl_s0 );
        uint32x2_t c_Tpl_s2 = vpaddl_u16( v_Tpl_s1 );
        v_Tpl_Sums          = vadd_u32( v_Tpl_Sums, c_Tpl_s2 );
        // v_Tpl_Sumz += i_Tpl_pu8 * i_Tpl_pu8
        uint16x8_t v_Tpl_z0 = vmull_u8( v_Tpl_s0, v_Tpl_s0 );
        uint32x4_t v_Tpl_z1 = vpaddlq_u16( v_Tpl_z0 ); // VPADDL.U16 q0,q0
        v_Tpl_Sumz          = vaddq_u32( v_Tpl_Sumz, v_Tpl_z1 );
        // v_XY_Sumz += i_Src_pu8 * i_Tpl_pu8
        uint16x8_t v_XY_zl1 = vmull_u8( v_Src_s0, v_Tpl_s0 );
        uint32x4_t v_XY_zl2 = vpaddlq_u16( v_XY_zl1 ); // VPADDL.U16 q0,q0
        v_XY_Sumz           = vaddq_u32( v_XY_Sumz, v_XY_zl2 );
        i_Src_pu8 += i_SrcStep_u32;
        i_Tpl_pu8 += i_TplStep_u32;
    }

    uint64x2_t v_Src_Sumz_u1  = vpaddlq_u32( v_Src_Sumz );
    uint32x2_t v_Src_Sumz_u2  = vmovn_u64( v_Src_Sumz_u1 );
    uint64x1_t v_Src_Sumz_u3  = vpaddl_u32( v_Src_Sumz_u2 );
    uint64_t   v_Src_Sumz_u4 = vget_lane_u64( v_Src_Sumz_u3, 0 );
    uint64x2_t v_Tpl_Sumz_u1 = vpaddlq_u32( v_Tpl_Sumz );
    uint32x2_t v_Tpl_Sumz_u2 = vmovn_u64( v_Tpl_Sumz_u1 );
    uint64x1_t v_Tpl_Sumz_u3 = vpaddl_u32( v_Tpl_Sumz_u2 );
    uint64_t   v_Tpl_Sumz_u4 = vget_lane_u64( v_Tpl_Sumz_u3, 0 );
    uint64x2_t v_XY_Sumz_u1 = vpaddlq_u32( v_XY_Sumz );
    uint32x2_t v_XY_Sumz_u2 = vmovn_u64( v_XY_Sumz_u1 );
    uint64x1_t v_XY_Sumz_u3 = vpaddl_u32( v_XY_Sumz_u2 );
    uint64_t   v_XY_Sumz_u4 = vget_lane_u64( v_XY_Sumz_u3, 0 );
    uint64x1_t v_Tpl_Sums_u3 = vpaddl_u32( v_Tpl_Sums );
    uint64_t   v_Tpl_Sums_u4 = vget_lane_u64( v_Tpl_Sums_u3, 0 );
    uint64x1_t v_Src_Sums_u3 = vpaddl_u32( v_Src_Sums );
    uint64_t   v_Src_Sums_u4 = vget_lane_u64( v_Src_Sums_u3, 0 );
    v_ResultVector_pt->v_SumS_u32 = ( uint32_t )v_Src_Sums_u4;
    v_ResultVector_pt->v_SumP_u32 = ( uint32_t )v_Tpl_Sums_u4;
    v_ResultVector_pt->v_SumQ_u32 = ( uint32_t )v_Tpl_Sumz_u4;
    v_ResultVector_pt->v_SumZ_u32 = ( uint32_t )v_Src_Sumz_u4;
    v_ResultVector_pt->v_SumV_u32 = ( uint32_t )v_XY_Sumz_u4;
}



void CrossBlock_NEON16_v(
    const uint8_t* i_Src_pu8, uint32_t i_SrcStep_u32,
    const uint8_t* i_Tpl_pu8, uint32_t i_TplStep_u32,
    ResultVector_t* v_ResultVector_pt )
{
    uint32x4_t  v_Src_Sumz = vdupq_n_u32( 0 );
    uint32x4_t  v_Tpl_Sumz = vdupq_n_u32( 0 );
    uint32x4_t  v_XY_Sumz  = vdupq_n_u32( 0 );
    uint32x4_t  v_Src_Sums = vdupq_n_u32( 0 );
    uint32x4_t  v_Tpl_Sums = vdupq_n_u32( 0 );

    for( uint16_t v_Y_u16 = 0; v_Y_u16 < 16U; v_Y_u16++ )
    {
        // v_Src_Sumz !
        uint8x8_t  v_Src_pl1 = vld1_u8( i_Src_pu8 + 0U );
        uint8x8_t  v_Src_ph1 = vld1_u8( i_Src_pu8 + 8U );
        // v_Src_Sums += i_Src_pu8
        uint16x8_t c_Src_s1 = vaddl_u8( v_Src_pl1, v_Src_ph1 );
        uint32x4_t c_Src_s2 = vpaddlq_u16( c_Src_s1 );
        v_Src_Sums          = vaddq_u32( v_Src_Sums, c_Src_s2 );
        // v_Src_Sumz += i_Src_pu8 * i_Src_pu8;
        uint16x8_t v_Src_zl1 = vmull_u8( v_Src_pl1, v_Src_pl1 );
        uint32x4_t v_Src_zl2 = vpaddlq_u16( v_Src_zl1 );
        uint16x8_t v_Src_zh1 = vmull_u8( v_Src_ph1, v_Src_ph1 );
        uint32x4_t v_Src_zh2 = vpaddlq_u16( v_Src_zh1 );
        uint32x4_t v_Src_z2  = vaddq_u32( v_Src_zl2, v_Src_zh2 );
        v_Src_Sumz           = vaddq_u32( v_Src_Sumz, v_Src_z2 );
        // v_Tpl_Sumz !
        uint8x8_t  v_Tpl_pl1 = vld1_u8( i_Tpl_pu8 + 0U );
        uint8x8_t  v_Tpl_ph1 = vld1_u8( i_Tpl_pu8 + 8U );
        // v_Tpl_Sums += i_Tpl_pu8
        uint16x8_t c_Tpl_s1 = vaddl_u8( v_Tpl_pl1, v_Tpl_ph1 );
        uint32x4_t c_Tpl_s2 = vpaddlq_u16( c_Tpl_s1 );
        v_Tpl_Sums          = vaddq_u32( v_Tpl_Sums, c_Tpl_s2 );
        // v_Tpl_Sumz += i_Tpl_pu8 * i_Tpl_pu8
        uint16x8_t v_Tpl_zl1 = vmull_u8( v_Tpl_pl1, v_Tpl_pl1 );
        uint32x4_t v_Tpl_zl2 = vpaddlq_u16( v_Tpl_zl1 ); // VPADDL.U16 q0,q0
        uint16x8_t v_Tpl_zh1 = vmull_u8( v_Tpl_ph1, v_Tpl_ph1 );
        uint32x4_t v_Tpl_zh2 = vpaddlq_u16( v_Tpl_zh1 ); // VPADDL.U16 q0,q0
        uint32x4_t v_Tpl_z2  = vaddq_u32( v_Tpl_zl2, v_Tpl_zh2 );
        v_Tpl_Sumz           = vaddq_u32( v_Tpl_Sumz, v_Tpl_z2 );
        // v_XY_Sumz += i_Src_pu8 * i_Tpl_pu8
        uint16x8_t v_XY_zl1 = vmull_u8( v_Src_pl1, v_Tpl_pl1 );
        uint32x4_t v_XY_zl2 = vpaddlq_u16( v_XY_zl1 ); // VPADDL.U16 q0,q0
        uint16x8_t v_XY_zh1 = vmull_u8( v_Src_ph1, v_Tpl_ph1 );
        uint32x4_t v_XY_zh2 = vpaddlq_u16( v_XY_zh1 ); // VPADDL.U16 q0,q0
        uint32x4_t v_XY_z2  = vaddq_u32( v_XY_zl2, v_XY_zh2 );
        v_XY_Sumz           = vaddq_u32( v_XY_Sumz, v_XY_z2 );
        i_Src_pu8 += i_SrcStep_u32;
        i_Tpl_pu8 += i_TplStep_u32;
    }

    uint64x2_t v_Src_Sumz_u1  = vpaddlq_u32( v_Src_Sumz );
    uint32x2_t v_Src_Sumz_u2  = vmovn_u64( v_Src_Sumz_u1 );
    uint64x1_t v_Src_Sumz_u3  = vpaddl_u32( v_Src_Sumz_u2 );
    uint64_t   v_Src_Sumz_u4  = vget_lane_u64( v_Src_Sumz_u3, 0 );
    uint64x2_t v_Tpl_Sumz_u1  = vpaddlq_u32( v_Tpl_Sumz );
    uint32x2_t v_Tpl_Sumz_u2  = vmovn_u64( v_Tpl_Sumz_u1 );
    uint64x1_t v_Tpl_Sumz_u3  = vpaddl_u32( v_Tpl_Sumz_u2 );
    uint64_t   v_Tpl_Sumz_u4  = vget_lane_u64( v_Tpl_Sumz_u3, 0 );
    uint64x2_t v_XY_Sumz_u1   = vpaddlq_u32( v_XY_Sumz );
    uint32x2_t v_XY_Sumz_u2   = vmovn_u64( v_XY_Sumz_u1 );
    uint64x1_t v_XY_Sumz_u3   = vpaddl_u32( v_XY_Sumz_u2 );
    uint64_t   v_XY_Sumz_u4   = vget_lane_u64( v_XY_Sumz_u3, 0 );
    uint64x2_t v_Tpl_Sums_u1  = vpaddlq_u32( v_Tpl_Sums );
    uint32x2_t v_Tpl_Sums_u2  = vmovn_u64( v_Tpl_Sums_u1 );
    uint64x1_t v_Tpl_Sums_u3  = vpaddl_u32( v_Tpl_Sums_u2 );
    uint64_t   v_Tpl_Sums_u4  = vget_lane_u64( v_Tpl_Sums_u3, 0 );
    uint64x2_t v_Src_Sums_u1  = vpaddlq_u32( v_Src_Sums );
    uint32x2_t v_Src_Sums_u2  = vmovn_u64( v_Src_Sums_u1 );
    uint64x1_t v_Src_Sums_u3  = vpaddl_u32( v_Src_Sums_u2 );
    uint64_t   v_Src_Sums_u4  = vget_lane_u64( v_Src_Sums_u3, 0 );
    v_ResultVector_pt->v_SumS_u32 = ( uint32_t )v_Src_Sums_u4;
    v_ResultVector_pt->v_SumP_u32 = ( uint32_t )v_Tpl_Sums_u4;
    v_ResultVector_pt->v_SumQ_u32 = ( uint32_t )v_Tpl_Sumz_u4;
    v_ResultVector_pt->v_SumZ_u32 = ( uint32_t )v_Src_Sumz_u4;
    v_ResultVector_pt->v_SumV_u32 = ( uint32_t )v_XY_Sumz_u4;
}


void GetSAD_NEON16_v( const uint8_t* i_Src_pu8, uint32_t i_SrcStep_u32,
                      const uint8_t* i_Tpl_pu8, uint32_t i_TplStep_u32, uint32_t* o_SAD_pu32 )
{
    uint16x4_t v_SumAbs = vdup_n_u16( 0 );

    for( uint16_t v_Line_u16 = 0; v_Line_u16 < 16U; v_Line_u16++ )
    {
        // for every element in current line of patch
        uint8x8_t v_Src_p0      = vld1_u8( i_Src_pu8 + 0U );
        uint8x8_t v_Tpl_p0      = vld1_u8( i_Tpl_pu8 + 0U );
        uint8x8_t v_Diff_p0     = vabd_u8( v_Src_p0, v_Tpl_p0 );
        uint16x4_t v_Add_p0     = vpaddl_u8( v_Diff_p0 );
        uint8x8_t v_Src_p1      = vld1_u8( i_Src_pu8 + 8U );
        uint8x8_t v_Tpl_p1      = vld1_u8( i_Tpl_pu8 + 8U );
        uint8x8_t v_Diff_p1     = vabd_u8( v_Src_p1, v_Tpl_p1 );
        uint16x4_t v_Add_p1     = vpaddl_u8( v_Diff_p1 );
        uint16x4_t v_Add_p2     = vpadd_u16( v_Add_p0, v_Add_p1 );
        v_SumAbs                = vadd_u16( v_SumAbs,  v_Add_p2 );
        i_Src_pu8 += i_SrcStep_u32;
        i_Tpl_pu8 += i_TplStep_u32;
    }

    uint32x2_t v_SumAbs_u1 = vpaddl_u16( v_SumAbs );
    uint64x1_t v_SumAbs_u2 = vpaddl_u32( v_SumAbs_u1 );
    uint64_t   v_SumAbs_u3 = vget_lane_u64( v_SumAbs_u2, 0 );
    *o_SAD_pu32 = ( uint32_t )v_SumAbs_u3;
}


void GetSAD_NEON8_v( const uint8_t* i_Src_pu8, uint32_t i_SrcStep_u32,
                     const uint8_t* i_Tpl_pu8, uint32_t i_TplStep_u32, uint32_t* o_SAD_pu32 )
{
    uint16x4_t v_SumAbs = vdup_n_u16( 0 );

    for( uint16_t v_Line_u16 = 0; v_Line_u16 < 8U; v_Line_u16 += 2U )
    {
        // for every element in current line of patch
        uint8x8_t v_Src_p0   = vld1_u8( i_Src_pu8 + ( v_Line_u16 + 0U ) * i_SrcStep_u32 );
        uint8x8_t v_Tpl_p0   = vld1_u8( i_Tpl_pu8 + ( v_Line_u16 + 0U ) * i_TplStep_u32 );
        uint8x8_t v_Diff_p0  = vabd_u8( v_Src_p0, v_Tpl_p0 );
        uint16x4_t v_Add_p0  = vpaddl_u8( v_Diff_p0 );
        uint8x8_t v_Src_p1   = vld1_u8( i_Src_pu8 + ( v_Line_u16 + 1U ) * i_SrcStep_u32 );
        uint8x8_t v_Tpl_p1   = vld1_u8( i_Tpl_pu8 + ( v_Line_u16 + 1U ) * i_TplStep_u32 );
        uint8x8_t v_Diff_p1  = vabd_u8( v_Src_p1, v_Tpl_p1 );
        uint16x4_t v_Add_p1  = vpaddl_u8( v_Diff_p1 );
        uint16x4_t v_Add_p2  = vpadd_u16( v_Add_p0, v_Add_p1 );
        v_SumAbs             = vadd_u16( v_SumAbs, v_Add_p2 );
    }

    uint32x2_t v_SumAbs_u1 = vpaddl_u16( v_SumAbs );
    uint64x1_t v_SumAbs_u2 = vpaddl_u32( v_SumAbs_u1 );
    uint64_t   v_SumAbs_u3 = vget_lane_u64( v_SumAbs_u2, 0 );
    *o_SAD_pu32 = ( uint32_t )v_SumAbs_u3;
}

#endif

