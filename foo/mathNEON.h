/*--------------------------------------------------------------------------
 @file mathNEON.h
 @brief Contains declarations for template matching based on NEON


 --------------------------------------------------------------------------
 @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
 This document in its entirety is CONFIDENTIAL and may not be disclosed,
 disseminated or distributed to parties outside MAGNA Electronics
 without written permission from MAGNA Electronics.

 @author Detlef Hafer (Detlef.Hafer.Extern@magna.com)

 --------------------------------------------------------------------------*/

#ifndef __MYNEONDEF
#define __MYNEONDEF

#include "mecl/core/MeclTypes.h"

typedef struct ResultVector_s
{
	uint32_t v_SumS_u32;
	uint32_t v_SumP_u32;
	uint32_t v_SumQ_u32;
	uint32_t v_SumZ_u32;
	uint32_t v_SumV_u32;
} ResultVector_t;

#ifdef __cplusplus
extern "C"
{
#endif
void CrossBlock_NEON16_v(
	const uint8_t* i_Src_pu8, uint32_t i_SrcStep_u32,
	const uint8_t* i_Tpl_pu8, uint32_t i_TplStep_u32,
	ResultVector_t* v_ResultVector_ps);

void CrossBlock_NEON8_v(
	const uint8_t* i_Src_pu8, uint32_t i_SrcStep_u32,
	const uint8_t* i_Tpl_pu8, uint32_t i_TplStep_u32,
	ResultVector_t* v_ResultVector_ps);

void GetSAD_NEON16_v(const uint8_t* i_Src_pu8, uint32_t i_SrcStep_u32,
	const uint8_t* i_Tpl_pu8, uint32_t i_TplStep_u32, uint32_t* o_SAD_pu32);

void GetSAD_NEON8_v(const uint8_t* i_Src_pu8, uint32_t i_SrcStep_u32,
	const uint8_t* i_Tpl_pu8, uint32_t i_TplStep_u32, uint32_t* o_SAD_pu32);

#ifdef __cplusplus
};
#endif

#endif

