// ----------------------------------------------------------------------------
// --- Written by Dmitri Kelbas
// --- Modified by Dmitri Kelbas [27-Nov-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include <stdlib.h>
#include "tscApi.h"
#include "mathOperations.h"
#include <string.h>
#include <assert.h>
#include "armNEON.h"

#ifdef BMA_OPT_BY_ARM_NEON
namespace tsc_math_neon {

//-------------------------------------------------------------------------------------------------------------------------
// -- 16x16 with sum abs diff, readable code  --
// PRQA S 4213 1 // Parameter sad is modified in assembler code.
void BlockMatch16x16_ARM_NEON_orig(const uint32_t *source, const uint32_t *patch, uint32_t *sad)
{
#ifdef BMA_OPT_BY_ARM_NEON
    uint32_t pixels = 16*16;

    __asm__ volatile(
	"mov           r4,#0               \n" // initialaize r4 by zeros
	"vdup.u8       q2,r4               \n" // initialaize Q2 by zeros
	"vdup.u8       q3,r4               \n" // initialaize Q3 by zeros

	"vld1.u8       {d0-d1}, [%0]!      \n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source

"BlockMatch16x16_ARM_NEON:             \n" // start loop
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2
	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vqadd.u8      q3, q3, q2          \n" // accumulate saturated abs diff per column into eight u8 registers
	"subs          %3, %3, #16         \n" // subtract 16 from the pixel count
	"bne           BlockMatch16x16_ARM_NEON         \n" // repeat until the row is complete

	"vpaddl.u8     q0, q3              \n" // now we have column-wise sums of SAD in q3, need to sum them together pairwise
	"vpadd.u16     d0, d1              \n" // pairwise addition (from 8 u16 to 4 u16)
	"vpadd.u16     d0, d0              \n" // pairwise addition (from 4 u16 to 2 u16)
	"vpadd.u16     d0, d0              \n" // pairwise addition (from 2 u16 to 1 u16)
	"vst1.u16      {d0[0]}, [%2]       \n" // save result in variable 'sad'

	: "=r"(patch), "=r"(source), "=r"(sad), "=r"(pixels)
	: "0"(patch), "1"(source), "2"(sad), "3"(pixels)
	: "q0", "q1", "q2", "q3", "r4", "cc"
	);
#endif
}

//-------------------------------------------------------------------------------------------------------------------------
// -- 16x16 with sum abs diff, prefetched data, unrolled loop  --
// PRQA S 4213 1 // Parameter sad is modified in assembler code.
void BlockMatch16x16_ARM_NEON_opt(const uint32_t *source, const uint32_t *patch, uint32_t *sad)
{
#ifdef BMA_OPT_BY_ARM_NEON
	__asm__ volatile(
	"mov           r4,#0               \n" // initialize r4 by zeros
	"vdup.u8       q2,r4               \n" // initialize Q2 by zeros
	"vdup.u8       q3,r4               \n" // initialize Q3 by zeros
	"vdup.u8       q4,r4               \n" // initialize Q3 by zeros

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 0)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2 (row 0) (16 values by u8 size)

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 1)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2 (row 1) (16 values by u8 size)

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 2)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2 (row 2)

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 3)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2 (row 3)

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 4)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 5)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 6)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 7)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 8)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 9)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 10)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 11)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 12)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 13)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2 (row 13)

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 14)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2 (row 14)

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch (row 15) - last one
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 0)
	"vabd.u8       q2, q0, q1          \n" // store absolute difference into q2 (row 15) - last one

	"vaddl.u8      q3, d5, d4          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 15) - last one
	"vadd.u16      q4, q4, q3          \n" // accumulate saturated abs diff per 2 adjucent columns into 8 u16 registers (row 15) - last one

	"vpadd.u16     d8, d9              \n" // pairwise addition (from 8 u16 to 4 u16)
	"vpadd.u16     d8, d8              \n" // pairwise addition (from 4 u16 to 2 u16)
	"vpadd.u16     d8, d8              \n" // pairwise addition (from 2 u16 to 1 u16)
	"vst1.u16      {d8[0]}, [%2]       \n" // save result in variable 'sad'
	: "=r"(patch), "=r"(source), "=r"(sad)
	: "0"(patch), "1"(source), "2"(sad)
	: "q0", "q1", "q2", "q3", "q4", "r4", "cc"
	);
#endif
}

//-------------------------------------------------------------------------------------------------------------------------
// -- 16x16 normalized cross correlation --
// PRQA S 4213 1 // Parameter sad is modified in assembler code.
void Correlation16x16Norm_ARM_NEON_orig(const uint32_t *source, const uint32_t *patch, float32_t *sad)
{
#ifdef BMA_OPT_BY_ARM_NEON
	uint32_t pixels = 16*16;

	__asm__ volatile(
	"mov           r4,#0               \n" // initialize r4 by zeros
	"vdup.u8       q2,r4               \n" // initialize Q2 by zeros
	"vdup.u8       q3,r4               \n" // initialize Q3 by zeros
	"vdup.u8       q4,r4               \n" // initialize Q4 by zeros
	"vdup.u8       q5,r4               \n" // initialize Q5 by zeros

	"vdup.u8       q8,r4               \n" // initialize Q8 by zeros - curr sq patch
	"vdup.u8       q9,r4               \n" // initialize Q9 by zeros - curr sq patch
	"vdup.u8       q6,r4               \n" // initialize Q6 by zeros - sum of sqs patch

	"vdup.u8       q10,r4              \n" // initialize Q10 by zeros - curr sq source
	"vdup.u8       q11,r4              \n" // initialize Q11 by zeros - curr sq source
	"vdup.u8       q7,r4               \n" // initialize Q7 by zeros - sum of sqs source

	// ----------------- start loop over 16 rows -----------------------------------------------------
	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640

"Correlation16x16_ARM_NEON:                 \n" // start loop

	"vmull.u8      q2, d0, d2          \n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3          \n" // store (patch * source) into q3

	"vmull.u8      q8, d0, d0          \n" // store squared patch into q8
	"vmull.u8      q9, d1, d1          \n" // store squared patch into q9
	"vmull.u8      q10, d2, d2         \n" // store squared source into q10
	"vmull.u8      q11, d3, d3         \n" // store squared source into q11

	"vld1.u8       {d0-d1}, [%0]       \n" // load 16 pixels from the patch
	"add           %0, %0, #640        \n" // move %0 pointer to patch to next line +640
	"vpadal.u16    q4, q2              \n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3              \n" // pairwise add and accumulate long (crosscorr d1*d3)

	"vpadal.u16    q6, q8              \n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10             \n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q6, q9              \n" // pairwise add and accumulate long for patch (squared patch)
	"vpadal.u16    q7, q11             \n" // pairwise add and accumulate long for source (squared source)
	"vld1.u8       {d2-d3}, [%1]!      \n" // load 16 pixels from the source

	"subs          %3, %3, #16         \n" // subtract 16 from the pixel count
	"bne           Correlation16x16_ARM_NEON         \n" // repeat until the row is complete

										// q0 and q1 are free to reuse
	"vadd.u32      q0, q4, q5          \n" // add pairwise columns' sums of CrCorr in q4 and q5 (4+4) into q0
	"vpadd.u32     d12, d13            \n" // pairwise addition (from 4 u32 to 2 u32) q6
	"vpadd.u32     d0, d1              \n" // pairwise addition (from 4 u32 to 2 u32)
	"vpadd.u32     d14, d15            \n" // pairwise addition (from 4 u32 to 2 u32) q7
	"vpadd.u32     d12, d12            \n" // pairwise addition (from 2 u32 to 1 u32) q6 <-- sum of squares for patch
	"vpadd.u32     d0, d0              \n" // pairwise addition (from 2 u32 to 1 u32)    <-- 16x16 CC, data type u32
	"vpadd.u32     d14, d14            \n" // pairwise addition (from 2 u32 to 1 u32) q7 <-- sum of squares for source

	// --------------- D0 has 16x16 SAD, data type u32 --------------------------------------
	"vcvt.f32.u32  d13, d0             \n" // convert CrossCorr to floating point        <-- 16x16 SAD, data type f32
	// -----------------------------------------------------------------------
	"vcvt.f32.u32   d0, d12            \n" // convert sum of squares to floating point   <-- f32, patch
	"vmov.f32       d1, d0             \n" // d1 = d0
	"vrsqrte.f32    d0, d0             \n" // d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32       d2, d0, d1         \n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0         \n" // d3 = (3 - d0 * d2) / 2
	"vmul.f32       d0, d0, d3         \n" // d0 = d0 * d3
	"vmul.f32       d2, d0, d1         \n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0         \n" // d4 = (3 - d0 * d3) / 2
	"vmul.f32       d8, d0, d3         \n" // d8 = d0 * d4	                               <-- f32, patch
	// -----------------------------------------------------------------------
	"vcvt.f32.u32   d0, d14            \n" // convert sum of squares to floating point    <-- f32, source
	"vmov.f32       d1, d0             \n" // d1 = d0
	"vrsqrte.f32    d0, d0             \n" // d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32       d2, d0, d1         \n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0         \n" // d3 = (3 - d0 * d2) / 2
	"vmul.f32       d0, d0, d3         \n" // d0 = d0 * d3
	"vmul.f32       d2, d0, d1         \n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0         \n" // d4 = (3 - d0 * d3) / 2
	"vmul.f32       d9, d0, d3         \n" // d8 = d0 * d4	                               <-- f32, source
	// -----------------------------------------------------------------------

	"vmul.f32       d0, d13, d8        \n" // multiply CC by inverse square root of sum of squares of patch
	"vmul.f32       d0, d9             \n" // multiply CC by inverse square root of sum of squares of source

	// -----------------------------------------------------------------------
	"vst1.f32      {d0[0]}, [%2]       \n" // save result in variable 'sad'

	: "=r"(patch), "=r"(source), "=r"(sad), "=r"(pixels)
	: "0"(patch), "1"(source), "2"(sad), "3"(pixels)
	: "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "r4", "cc"
	);
#endif
}
//-------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------
// -- 16x16 normalized leveled cross correlation -- -- OPTIMISED
//-------------------------------------------------------------------------------------------------------------------------
// PRQA S 4213 1 // Parameters meanSrc and meanPatch are modified in assembler code.
void CrossCorr16x16_Mean_F32_ARM_NEON_opt(const uint32_t *source, const uint32_t *patch, float32_t *meanSrc, float32_t *meanPatch, float32_t recipr256)
{
#ifdef BMA_OPT_BY_ARM_NEON

	__asm__ volatile(

	// ----------------- START of mean value calculation ------------------------------------------
	// Input: %0 - d0, d1 - pointer to 'patch', a 16x16 ROI in 400x640 array
	//   	  %1 - d2, d3 - pointer to 'source', continueos array of 16x16
	// Used:  q0 - store 'patch' input data
	//        q1 - store 'source' input data
	//        q2 - store d0*d2 (patch * source) into q2
	//        q3 - store d1*d3 (patch * source) into q3
	//        q4 - d8, d9 keep copy of %0, %1
	//        q5 -
	//        q6 - pairwise add and accumulate long (squared patch)
	//        q7 - pairwise add and accumulate long (squared source)
	//        q8 - store sum of patch into q8
	//        q9 - store sum of patch into q9
	//        q10 - store sum of source into q10
	//        q11 - store sum of source into q11
	// Output:d0 - cross correlation sum
	//       d12 - sum of squares of patch
	//       d14 - sum of squares of source

	"mov           r4,#0				\n" // initialize r4 by zeros
	"vdup.u8       q2,r4              	\n" // initialize Q2 by zeros
	"vdup.u8       q3,r4              	\n" // initialize Q3 by zeros
	"vdup.u8       q5,r4              	\n" // initialize Q5 by zeros
	"vdup.u8       q6,r4              	\n" // initialize Q6 by zeros - sum of sqs patch
	"vdup.u8       q7,r4              	\n" // initialize Q7 by zeros - sum of sqs source
	"vdup.u8       q8,r4              	\n" // initialize Q8 by zeros - curr sq patch
	"vdup.u8       q9,r4              	\n" // initialize Q9 by zeros - curr sq patch
	"vdup.u8       q10,r4             	\n" // initialize Q10 by zeros - curr sq source
	"vdup.u8       q11,r4             	\n" // initialize Q11 by zeros - curr sq source
	"vdup.u8       q12,r4             	\n" // initialize Q12 by zeros - mean values d24-patch, d25-source
	"vdup.u8       q13,r4             	\n" // initialize Q13 by zeros - mean values d24-patch, d25-source

	// ----------------- start rolled-out loop over 16 rows ------------------------------------------
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 1 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 2 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 3 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 4 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 5 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 6 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 7 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 8 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 9 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 10 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 11 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 12 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 13 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 14 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 15 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
// -- iteration (row) 16 --
	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16
// -- -- -- -- --
	"vpadd.u16     d8, d4, d5         	\n" // pairwise addition (from 8 u16 to 4 u16)
	"vpadd.u16     d10, d6, d7         	\n" // pairwise addition (from 8 u16 to 4 u16)
	"vpadd.u16     d8, d8           	\n" // pairwise addition (from 4 u16 to 2 u16)
	"vpadd.u16     d10, d10           	\n" // pairwise addition (from 4 u16 to 2 u16)
	"vpaddl.u16    d0, d8 	         	\n" // pairwise addition (from 2 u16 to 1 u32) patch
	"vpaddl.u16    d1, d10  	      	\n" // pairwise addition (from 2 u16 to 1 u32) source

	"vcvt.f32.s32   q0, q0           	\n" // convert sum to floating point   <-- f32, patch

	"mov            r4, %4				\n" // initialize r4 by zeros
	"vmov.f32       d2[0], r4           \n"
	"vmov.f32       d3[0], r4           \n"
	"vmul.f32       q2, q1, q0		    \n" // divide d0 and d1 by 256.0 to get mean values
	// -----------------------------------------------------------------------
	"vst1.f32      {d4[0]}, [%2]      	\n" // save result in variable 'meanSrc'
	"vst1.f32      {d5[0]}, [%3]      	\n" // save result in variable 'meanPatch'

	// ----------------- END of mean value calculation ------------------------------------------

	: "=r"(patch), "=r"(source), "=r"(meanSrc), "=r"(meanPatch), "=r"(recipr256)   // output operand list
	: "0"(patch), "1"(source), "2"(meanSrc), "3"(meanPatch), "4"(recipr256)    	// input operand list
	: "q0", "q1", "q2", "q3", "q4", "q5", "r4", "cc"
	);
#endif
}

//-------------------------------------------------------------------------------------------------------------------------
// -- 16x16 normalized leveled cross correlation
//-------------------------------------------------------------------------------------------------------------------------
// PRQA S 4213 1 // Parameters meanSrc and meanPatch are modified in assembler code.
void CrossCorr16x16_Mean_F32_ARM_NEON_orig(const uint32_t *source, const uint32_t *patch, float32_t *meanSrc, float32_t *meanPatch, float32_t recipr256)
{
#ifdef BMA_OPT_BY_ARM_NEON
	uint32_t pixels = 16*16;

	__asm__ volatile(

	// ----------------- START of mean value calculation ------------------------------------------
	// Input: %0 - d0, d1 - pointer to 'patch', a 16x16 ROI in 400x640 array
	//   	  %1 - d2, d3 - pointer to 'source', continueos array of 16x16
	// Used:  q0 - store 'patch' input data
	//        q1 - store 'source' input data
	//        q2 - store d0*d2 (patch * source) into q2
	//        q3 - store d1*d3 (patch * source) into q3
	//        q4 - d8, d9 keep copy of %0, %1
	//        q5 -
	//        q6 - pairwise add and accumulate long (squared patch)
	//        q7 - pairwise add and accumulate long (squared source)
	//        q8 - store sum of patch into q8
	//        q9 - store sum of patch into q9
	//        q10 - store sum of source into q10
	//        q11 - store sum of source into q11
	// Output:d0 - cross correlation sum
	//       d12 - sum of squares of patch
	//       d14 - sum of squares of source

	"mov           r4,#0				\n" // initialize r4 by zeros
	"vdup.u8       q2,r4              	\n" // initialize Q2 by zeros
	"vdup.u8       q3,r4              	\n" // initialize Q3 by zeros
	"vdup.u8       q5,r4              	\n" // initialize Q5 by zeros
	"vdup.u8       q6,r4              	\n" // initialize Q6 by zeros - sum of sqs patch
	"vdup.u8       q7,r4              	\n" // initialize Q7 by zeros - sum of sqs source
	"vdup.u8       q8,r4              	\n" // initialize Q8 by zeros - curr sq patch
	"vdup.u8       q9,r4              	\n" // initialize Q9 by zeros - curr sq patch
	"vdup.u8       q10,r4             	\n" // initialize Q10 by zeros - curr sq source
	"vdup.u8       q11,r4             	\n" // initialize Q11 by zeros - curr sq source
	"vdup.u8       q12,r4             	\n" // initialize Q12 by zeros - mean values d24-patch, d25-source
	"vdup.u8       q13,r4             	\n" // initialize Q13 by zeros - mean values d24-patch, d25-source

	// ----------------- start rolled-out loop over 16 rows ------------------------------------------
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

"CrossCorr16x16_Mean:        			\n" // start loop

	"vpadal.u8     d4, d0 	         	\n"
	"vpadal.u8     d5, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16
	"vpadal.u8     d6, d2	          	\n"
	"vpadal.u8     d7, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

	"subs          %4, %4, #16        	\n" // subtract 16 from the pixel count
	"bne           CrossCorr16x16_Mean	\n" // repeat until the row is complete

	"vpadd.u16     d8, d4, d5         	\n" // pairwise addition (from 8 u16 to 4 u16)
	"vpadd.u16     d10, d6, d7         	\n" // pairwise addition (from 8 u16 to 4 u16)
	"vpadd.u16     d8, d8           	\n" // pairwise addition (from 4 u16 to 2 u16)
	"vpadd.u16     d10, d10           	\n" // pairwise addition (from 4 u16 to 2 u16)
	"vpaddl.u16    d0, d8 	         	\n" // pairwise addition (from 2 u16 to 1 u32) patch
	"vpaddl.u16    d1, d10  	      	\n" // pairwise addition (from 2 u16 to 1 u32) source

	// now divide it by 256
	//"vrshr.u16     d12, d8, #8        \n" // divide by 16*16=256=2^^8 and round it
	//"vrshr.u16     d13, d10, #8       \n" // divide by 16*16=256=2^^8 and round it
	"vcvt.f32.s32   q0, q0           	\n" // convert sum to floating point   <-- f32, patch

	"mov            r4, %5				\n" // initialize r4 by zeros
	"vmov.f32       d2[0], r4           \n"
	"vmov.f32       d3[0], r4           \n"
	"vmul.f32       q2, q1, q0		    \n" // divide d0 and d1 by 256.0 to get mean values
	// -----------------------------------------------------------------------
	"vst1.f32      {d4[0]}, [%2]      	\n" // save result in variable 'meanSrc'
	"vst1.f32      {d5[0]}, [%3]      	\n" // save result in variable 'meanPatch'

	// ----------------- END of mean value calculation ------------------------------------------

	: "=r"(patch), "=r"(source), "=r"(meanSrc), "=r"(meanPatch), "=r"(pixels), "=r"(recipr256)   // output operand list
	: "0"(patch), "1"(source), "2"(meanSrc), "3"(meanPatch), "4"(pixels), "5"(recipr256)    	// input operand list
	: "q0", "q1", "q2", "q3", "q4", "q5", "r4", "cc"
	);
#endif
}

//-------------------------------------------------------------------------------------------------------------------------
// NOTE: further execution time optimisation:
// More optimisation can be done by:
// - hiding memory access latency time by arithmetic operations on NEON registers (rearrange instructions)
// - unrolling the loop
// - possibly by using cache pre-load instructions "pld"
// PRQA S 4213 1 // Parameter sad is modified in assembler code.
void CrossCorr16x16_NormLevel_I32_1Pass_ARM_NEON_orig(const uint32_t *source, const uint32_t *patch, float32_t *sad)
{
#ifdef BMA_OPT_BY_ARM_NEON
	uint32_t pixels = 16*16;

	__asm__ volatile(

	// ----------------- START of Cross Correlation and squared sums value calculation --------
	// Input: %0 - d0, d1 - pointer to 'patch', a 16x16 ROI in 400x640 array
	//   	  %1 - d2, d3 - pointer to 'source', continueos array of 16x16
	//       d24 - mean value of patch data
	//       d25 - mean value of source data
	// Used:  q0 - store 'patch' input data
	//        q1 - store 'source' input data
	//        q2 - store d0*d2 (patch * source) into q2
	//        q3 - store d1*d3 (patch * source) into q3
	//        q4 - pairwise add and accumulate long (crosscorr d0*d2)
	//        q5 - pairwise add and accumulate long (crosscorr d1*d3)
	//        q6 - pairwise add and accumulate long (squared patch)
	//        q7 - pairwise add and accumulate long (squared source)
	//        q8 - store squared patch into q8
	//        q9 - store squared patch into q9
	//        q10 - store squared source into q10
	//        q11 - store squared source into q11
	//        d24 -
	//        d25 -
	// Output:d0 - cross correlation sum
	//       d12 - sum of squares of patch
	//       d14 - sum of squares of source

	"mov           r4,#0				\n" // initialize r4 by zeros
	"vdup.u8       q2,r4              	\n" // initialize Q2 by zeros
	"vdup.u8       q3,r4              	\n" // initialize Q3 by zeros
	"vdup.u8       q4,r4              	\n" // initialize Q4 by zeros
	"vdup.u8       q5,r4              	\n" // initialize Q5 by zeros

	"vdup.u8       q8,r4              	\n" // initialize Q8 by zeros - curr sq patch
	"vdup.u8       q9,r4              	\n" // initialize Q9 by zeros - curr sq patch
	"vdup.u8       q6,r4              	\n" // initialize Q6 by zeros - sum of sq patch

	"vdup.u8       q10,r4             	\n" // initialize Q10 by zeros - curr sq source
	"vdup.u8       q11,r4             	\n" // initialize Q11 by zeros - curr sq source
	"vdup.u8       q7,r4              	\n" // initialize Q7 by zeros - sum of sq source
	"vdup.u8       q12,r4              	\n" // initialize Q12 by zeros - sum of sq source
	"vdup.u8       q13,r4              	\n" // initialize Q13 by zeros - sum of sq source

	// ----------------- start rolled-out loop over 16 rows ------------------------------------------
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

"Correl16x16NormLevel1Pass:         	\n" // start loop

	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

	"subs          %3, %3, #16        	\n" // subtract 16 from the pixel count
	"bne           Correl16x16NormLevel1Pass	\n" // repeat until the row is complete

	//-- after end of this loop the important data accumulated in registers:
	// Q4,Q5 - cross correlation 2 x 4 x u32
	// Q6 - patch squared 4 x u32
	// Q7 - source squared 4 x u32
	// Q12 - accumulate patch 8 x u16
	// Q13 - accumulate source 8 x u16
	//-----------------------------------------------------------------------------

	// Q0 and Q1 are free to reuse
	"vadd.u32      q0, q4, q5         	\n" // add pairwise columns' sums of CrCorr (Q4+Q5-->Q0)
	"vpadd.u32     d0, d1             	\n" // pairwise addition (from 4 u32 to 2 u32) (Q0-->Q0)

	"vpadd.u32     d12, d13           	\n" // pairwise addition (from 4 u32 to 2 u32) (Q6-->Q6)
	"vpadd.u32     d14, d15           	\n" // pairwise addition (from 4 u32 to 2 u32) (Q7-->Q7)
	"vpadd.u32     d12, d12           	\n" // pairwise addition (from 2 u32 to 1 u32) (Q6-->Q6) <-- sum of squares for patch
	"vpadd.u32     d14, d14           	\n" // pairwise addition (from 2 u32 to 1 u32) (Q7-->Q7) <-- sum of squares for source
	"vpadd.u32     d0, d0             	\n" // pairwise addition (from 2 u32 to 1 u32)           <-- 16x16 CC, data type u32

	"vpadd.u16     d24, d25           	\n" // pairwise addition (from 8 u16 to 4 u16) (Q12-->Q12)
	"vpadd.u16     d24, d24           	\n" // pairwise addition (from 4 u16 to 2 u16) (Q12-->Q12)
	"vpaddl.u16    d24, d24           	\n" // pairwise addition (from 2 u16 to 1 u32) (Q12-->Q12)

	"vpadd.u16     d26, d27           	\n" // pairwise addition (from 8 u16 to 4 u16) (Q13-->Q13)
	"vpadd.u16     d26, d26           	\n" // pairwise addition (from 4 u16 to 2 u16) (Q13-->Q13)
	"vpaddl.u16    d26, d26           	\n" // pairwise addition (from 2 u16 to 1 u32) (Q13-->Q13)

	//-- the important data accumulated in registers:
	// Q0[0] - cross correlation u32
	// Q6[0] - patch squared u32
	// Q7[0] - source squared u32
	// D24[0] - accumulate patch u32
	// D26[0] - accumulate source u32
	//-----------------------------------------------------------------------------

	"vmul.u32		d25, d24, d24		\n" // (sumx>>8)*(sumx>>8)<<8 = (sumx*sumx)>>8, total ((16+16)-8)=24bit
	"vshr.u32		d25, #8				\n"
	"vmul.u32		d27, d26, d26		\n" // (sumy>>8)*(sumy>>8)<<8 = (sumy*sumy)>>8, total ((16+16)-8)=24bit
	"vshr.u32		d27, #8				\n"
	"vmul.u32		d22, d24, d26		\n" // (sumy>>8)*(sumy>>8)<<8 = (sumy*sumy)>>8, total ((16+16)-8)=24bit
	"vshr.u32		d22, #8				\n"
	// -- new data are stored in registers:
	// D25[0] u32 - mean patch squared * 256
	// D27[0] u32 - mean source squared * 256
	// D22[0] u32 - mean patch * mean source * 256

	"vsub.s32       d4, d0,  d22		\n" // signed sum x*y       = sum x*y - mean x * mean y * 256;
	"vsub.s32       d5, d12, d25		\n" // signed squared sum x = squared sum x - mean x squared * 256;
	"vsub.s32       d6, d14, d27		\n" // signed squared sum y = squared sum y - mean y squared * 256;

	"vabs.s32       d4, d4				\n" //if( signedsumxy  < 0 ) then signedsumxy  = -signedsumxy
	"vabs.s32       d5, d5				\n" //if( signedsqsumx < 0 ) then signedsqsumx = -signedsqsumx
	"vabs.s32       d6, d6				\n" //if( signedsqsumy < 0 ) then signedsqsumy = -signedsqsumy

	// --------------- D0 has 16x16 SAD, data type u32 --------------------------------------
	"vcvt.f32.s32   d7, d4            	\n" // convert CrossCorr to floating point        <-- 16x16 SAD, data type f32
	// -----------------------------------------------------------------------
	"vcvt.f32.s32   d0, d5           	\n" // convert sum of squares to floating point   <-- f32, patch
	"vmov.f32       d1, d0            	\n" // d1 = d0
	"vrsqrte.f32    d0, d0            	\n" // d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d3 = (3 - d0 * d2) / 2
	"vmul.f32       d0, d0, d3        	\n" // d0 = d0 * d3
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d4 = (3 - d0 * d3) / 2
	"vmul.f32       d8, d0, d3        	\n" // d8 = d0 * d4	                               <-- f32, patch
	// -----------------------------------------------------------------------
	"vcvt.f32.s32   d0, d6           	\n" // convert sum of squares to floating point    <-- f32, source
	"vmov.f32       d1, d0            	\n" // d1 = d0
	"vrsqrte.f32    d0, d0            	\n" // d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d3 = (3 - d0 * d2) / 2
	"vmul.f32       d0, d0, d3        	\n" // d0 = d0 * d3
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d4 = (3 - d0 * d3) / 2
	"vmul.f32       d9, d0, d3         	\n" // d8 = d0 * d4	                               <-- f32, source
	// -----------------------------------------------------------------------

	"vmul.f32       d0, d7, d8       	\n" // multiply CC by inverse square root of sum of squares of patch
	"vmul.f32       d0, d9            	\n" // multiply CC by inverse square root of sum of squares of source

	// -----------------------------------------------------------------------
	"vst1.f32      {d0[0]}, [%2]      	\n" // save result in variable 'sad'

	: "=r"(patch), "=r"(source), "=r"(sad), "=r"(pixels)
	: "0"(patch),   "1"(source),  "2"(sad),  "3"(pixels)
	: "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "q12", "q13", "r4", "cc"
	);
#endif
}

//-------------------------------------------------------------------------------------------------------------------------
// PRQA S 4213 1 // Parameter sad is modified in assembler code.
void CrossCorr16x16_NormLevel_I32_1Pass_ARM_NEON_opt(const uint32_t *source, const uint32_t *patch, float32_t *sad)
{
#ifdef BMA_OPT_BY_ARM_NEON

	__asm__ volatile(

	// ----------------- START of Cross Correlation and squared sums value calculation --------
	// Input: %0 - d0, d1 - pointer to 'patch', a 16x16 ROI in 400x640 array
	//   	  %1 - d2, d3 - pointer to 'source', continueos array of 16x16
	//       d24 - mean value of patch data
	//       d25 - mean value of source data
	// Used:  q0 - store 'patch' input data
	//        q1 - store 'source' input data
	//        q2 - store d0*d2 (patch * source) into q2
	//        q3 - store d1*d3 (patch * source) into q3
	//        q4 - pairwise add and accumulate long (crosscorr d0*d2)
	//        q5 - pairwise add and accumulate long (crosscorr d1*d3)
	//        q6 - pairwise add and accumulate long (squared patch)
	//        q7 - pairwise add and accumulate long (squared source)
	//        q8 - store squared patch into q8
	//        q9 - store squared patch into q9
	//        q10 - store squared source into q10
	//        q11 - store squared source into q11
	//        d24 -
	//        d25 -
	// Output:d0 - cross correlation sum
	//       d12 - sum of squares of patch
	//       d14 - sum of squares of source

	"mov           r4,#0				\n" // initialize r4 by zeros
	"vdup.u8       q2,r4              	\n" // initialize Q2 by zeros
	"vdup.u8       q3,r4              	\n" // initialize Q3 by zeros
	"vdup.u8       q4,r4              	\n" // initialize Q4 by zeros
	"vdup.u8       q5,r4              	\n" // initialize Q5 by zeros

	"vdup.u8       q8,r4              	\n" // initialize Q8 by zeros - curr sq patch
	"vdup.u8       q9,r4              	\n" // initialize Q9 by zeros - curr sq patch
	"vdup.u8       q6,r4              	\n" // initialize Q6 by zeros - sum of sq patch

	"vdup.u8       q10,r4             	\n" // initialize Q10 by zeros - curr sq source
	"vdup.u8       q11,r4             	\n" // initialize Q11 by zeros - curr sq source
	"vdup.u8       q7,r4              	\n" // initialize Q7 by zeros - sum of sq source
	"vdup.u8       q12,r4              	\n" // initialize Q12 by zeros - sum of sq source
	"vdup.u8       q13,r4              	\n" // initialize Q13 by zeros - sum of sq source

	// ----------------- start rolled-out loop over 16 rows ------------------------------------------
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

			//-- iteration 1
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

			//-- iteration 2
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 3
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 4
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 5
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 6
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 7
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 8
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 9
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 10
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 11
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 12
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 13
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 14
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 15
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
			//-- iteration 16
	//-- pairwise multiplication between 16 x u8 source and patch generates 16 x u16 (Q0,Q1 -> Q3,Q2)
	"vmull.u8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.u8      q3, d1, d3         	\n" // store (patch * source) into q3

	//-- pairwise multiplication between 16 x u8 patch squared generates 16 x u16 (Q0 -> Q9,Q8)
	"vmull.u8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.u8      q9, d1, d1         	\n" // store squared patch into q9

	//-- pairwise multiplication between 16 x u8 source squared generates 16 x u16 (Q1 -> Q11,Q10)
	"vmull.u8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.u8      q11, d3, d3        	\n" // store squared source into q11

	//-- pairwise accumulate patch from 16 x u8 to 8 x u16 (Q0 -> Q12)
	"vpadal.u8     d24, d0 	         	\n"
	"vpadal.u8     d25, d1	          	\n" // pairwise add and accumulate long to sum of patch from 16xu8 to 8xu16

	//-- pairwise accumulate source from 8 x u8 to 4 x u16 (Q1 -> Q13)
	"vpadal.u8     d26, d2	          	\n"
	"vpadal.u8     d27, d3	          	\n" // pairwise add and accumulate long to sum of source from 16xu8 to 8xu16

	// moved here to hide latency
	"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)

	// pairwise accumulate cross correlation 8 x u16 to 4 x u32, (Q2 -> Q4), (Q3 -> Q5)
	"vpadal.u16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.u16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	// pairwise accumulate patch squared 8 x u16 to 4 x u32, (Q9,Q8 -> Q6)
	//"vpadal.u16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.u16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)

	// pairwise accumulate source squared 8 x u16 to 4 x u32, (Q11,Q10 -> Q7)
	//"vpadal.u16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.u16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)

	//-- after end of this loop the important data accumulated in registers:
	// Q4,Q5 - cross correlation 2 x 4 x u32
	// Q6 - patch squared 4 x u32
	// Q7 - source squared 4 x u32
	// Q12 - accumulate patch 8 x u16
	// Q13 - accumulate source 8 x u16
	//-----------------------------------------------------------------------------

	// Q0 and Q1 are free to reuse
	"vadd.u32      q0, q4, q5         	\n" // add pairwise columns' sums of CrCorr (Q4+Q5-->Q0)
	"vpadd.u32     d12, d13           	\n" // pairwise addition (from 4 u32 to 2 u32) (Q6-->Q6)
	"vpadd.u32     d14, d15           	\n" // pairwise addition (from 4 u32 to 2 u32) (Q7-->Q7)
	"vpadd.u32     d0, d1             	\n" // pairwise addition (from 4 u32 to 2 u32) (Q0-->Q0)
	"vpadd.u16     d24, d25           	\n" // pairwise addition (from 8 u16 to 4 u16) (Q12-->Q12)
	"vpadd.u16     d26, d27           	\n" // pairwise addition (from 8 u16 to 4 u16) (Q13-->Q13)

	"vpadd.u32     d12, d12           	\n" // pairwise addition (from 2 u32 to 1 u32) (Q6-->Q6) <-- sum of squares for patch
	"vpadd.u32     d14, d14           	\n" // pairwise addition (from 2 u32 to 1 u32) (Q7-->Q7) <-- sum of squares for source
	"vpadd.u32     d0, d0             	\n" // pairwise addition (from 2 u32 to 1 u32)           <-- 16x16 CC, data type u32

	"vpadd.u16     d24, d24           	\n" // pairwise addition (from 4 u16 to 2 u16) (Q12-->Q12)
	"vpadd.u16     d26, d26           	\n" // pairwise addition (from 4 u16 to 2 u16) (Q13-->Q13)
	"vpaddl.u16    d24, d24           	\n" // pairwise addition (from 2 u16 to 1 u32) (Q12-->Q12)
	"vpaddl.u16    d26, d26           	\n" // pairwise addition (from 2 u16 to 1 u32) (Q13-->Q13)

	//-- the important data accumulated in registers:
	// Q0[0] - cross correlation u32
	// Q6[0] - patch squared u32
	// Q7[0] - source squared u32
	// D24[0] - accumulate patch u32
	// D26[0] - accumulate source u32
	//-----------------------------------------------------------------------------

	"vmul.u32		d25, d24, d24		\n" // (sumx>>8)*(sumx>>8)<<8 = (sumx*sumx)>>8, total ((16+16)-8)=24bit
	"vmul.u32		d27, d26, d26		\n" // (sumy>>8)*(sumy>>8)<<8 = (sumy*sumy)>>8, total ((16+16)-8)=24bit

	"vmul.u32		d22, d24, d26		\n" // (sumx>>8)*(sumy>>8)<<8 = (sumx*sumy)>>8, total ((16+16)-8)=24bit
	"vtrn.u32       d25, d27            \n" // d25[0] - Xmean^^2, d25[1] - Ymean^^2
	"vtrn.u32       d12, d14            \n" // d12[0] - squared sum x, d12[1] - squared sum y

	"vshr.u32		d22, #8				\n"
	"vshr.u32		d25, #8				\n"
	// -- new data are stored in registers:
	// D25[0] u32 - mean patch squared * 256
	// D25[1] u32 - mean source squared * 256
	// D22[0] u32 - mean patch * mean source * 256

	"vsub.s32       d4, d0,  d22		\n" // signed sum x*y       = sum x*y - mean x * mean y * 256;
	"vsub.s32       d5, d12, d25		\n" // d5[0]: signed squared sum x = squared sum x - mean x squared * 256;
											// d5[1]: signed squared sum y = squared sum y - mean y squared * 256;

	"vabs.s32       d4, d4				\n" //if( signedsumxy  < 0 ) then signedsumxy  = -signedsumxy
	"vabs.s32       d5, d5				\n" //if( signedsqsumx < 0 ) then signedsqsumx = -signedsqsumx
											//if( signedsqsumy < 0 ) then signedsqsumy = -signedsqsumy

	// --------------- D0 has 16x16 SAD, data type u32 --------------------------------------
	"vcvt.f32.s32   d7, d4            	\n" // convert CrossCorr to floating point        <-- 16x16 SAD, data type f32
	// -----------------------------------------------------------------------

	// -----------------------------------------------------------------------
	"vcvt.f32.s32   d0, d5           	\n" // convert sum of squares to floating point   <-- f32, patch
	"vmov.f32       d1, d0            	\n" // d1 = d0
	"vrsqrte.f32    d0, d0            	\n" // d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d3 = (3 - d0 * d2) / 2
	"vmul.f32       d0, d0, d3        	\n" // d0 = d0 * d3
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d4 = (3 - d0 * d3) / 2
	"vmul.f32       d8, d0, d3        	\n" // d8 = d0 * d4	                               <-- f32, patch

	// -----------------------------------------------------------------------

	"vmul.f32       d0, d7, d8       	\n" // multiply CC by inverse square root of sum of squares of patch
	"vtrn.f32       d8, d7              \n"
	"vmul.f32       d0, d7        		\n" // multiply CC by inverse square root of sum of squares of source

	// -----------------------------------------------------------------------
	"vst1.f32      {d0[0]}, [%2]      	\n" // save result in variable 'sad'

	: "=r"(patch), "=r"(source), "=r"(sad)
	: "0"(patch),   "1"(source),  "2"(sad)
	: "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "q12", "q13", "r4", "cc"
	);
#endif
}


//-------------------------------------------------------------------------------------------------------------------------
// PRQA S 4213 1 // Parameter sad is modified in assembler code.
void CrossCorr16x16_NormLevel_I32_ARM_NEON_orig(const uint32_t *source, const uint32_t *patch, uint32_t *meanSrc, uint32_t *meanPatch, float32_t *sad)
{
#ifdef BMA_OPT_BY_ARM_NEON
	uint32_t pixels = 16*16;

	__asm__ volatile(

	"vld1.u8       {d24}, [%2]			\n" // copy 'meanSrc' to d24
	"vld1.u8       {d26}, [%3]			\n" // copy 'meanPatch' to d26

	// now duplicate it
	"vdup.u8       q12, d24[0]          \n" // duplicate first byte to all 8 bytes -- order is important
	"vdup.u8       q13, d26[0]          \n" // duplicate first byte to all 8 bytes -- order is important

	// ----------------- END of mean value preparation ------------------------------------------

	// ----------------- START of Cross Correlation and squared sums value calculation --------
	// Input: %0 - d0, d1 - pointer to 'patch', a 16x16 ROI in 400x640 array
	//   	  %1 - d2, d3 - pointer to 'source', continueos array of 16x16
	//       d24 - mean value of patch data
	//       d25 - mean value of source data
	// Used:  q0 - store 'patch' input data
	//        q1 - store 'source' input data
	//        q2 - store d0*d2 (patch * source) into q2
	//        q3 - store d1*d3 (patch * source) into q3
	//        q4 - pairwise add and accumulate long (crosscorr d0*d2)
	//        q5 - pairwise add and accumulate long (crosscorr d1*d3)
	//        q6 - pairwise add and accumulate long (squared patch)
	//        q7 - pairwise add and accumulate long (squared source)
	//        q8 - store squared patch into q8
	//        q9 - store squared patch into q9
	//        q10 - store squared source into q10
	//        q11 - store squared source into q11
	//        d24 - mean value u16 of patch[]
	//        d25 - mean value u16 of source[]
	// Output:d0 - cross correlation sum
	//       d12 - sum of squares of patch
	//       d14 - sum of squares of source

	"mov           r4,#0				\n" // initialize r4 by zeros
	"vdup.u8       q2,r4              	\n" // initialize Q2 by zeros
	"vdup.u8       q3,r4              	\n" // initialize Q3 by zeros
	"vdup.u8       q4,r4              	\n" // initialize Q4 by zeros
	"vdup.u8       q5,r4              	\n" // initialize Q5 by zeros

	"vdup.u8       q8,r4              	\n" // initialize Q8 by zeros - curr sq patch
	"vdup.u8       q9,r4              	\n" // initialize Q9 by zeros - curr sq patch
	"vdup.u8       q6,r4              	\n" // initialize Q6 by zeros - sum of sq patch

	"vdup.u8       q10,r4             	\n" // initialize Q10 by zeros - curr sq source
	"vdup.u8       q11,r4             	\n" // initialize Q11 by zeros - curr sq source
	"vdup.u8       q7,r4              	\n" // initialize Q7 by zeros - sum of sq source
	// Note, q12 and q13 are filled out with 'meanSrc' and 'meanPatch'

	// ----------------- start rolled-out loop over 16 rows ------------------------------------------
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

"Correlation16x16NormLevel:         	\n" // start loop
	"vsub.i8       q0, q12              \n"	// subtract mean value from patch data
	"vsub.i8       q1, q13              \n"	// subtract mean value from source data

	"vmull.s8      q2, d0, d2         	\n" // store (patch * source) into q2
	"vmull.s8      q3, d1, d3         	\n" // store (patch * source) into q3

	"vmull.s8      q8, d0, d0         	\n" // store squared patch into q8
	"vmull.s8      q9, d1, d1         	\n" // store squared patch into q9
	"vmull.s8      q10, d2, d2        	\n" // store squared source into q10
	"vmull.s8      q11, d3, d3        	\n" // store squared source into q11

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vpadal.s16    q4, q2             	\n" // pairwise add and accumulate long (crosscorr d0*d2)
	"vpadal.s16    q5, q3             	\n" // pairwise add and accumulate long (crosscorr d1*d3)

	"vpadal.s16    q6, q8             	\n" // pairwise add and accumulate long (squared patch)
	"vpadal.s16    q7, q10            	\n" // pairwise add and accumulate long (squared source)
	"vpadal.s16    q6, q9             	\n" // pairwise add and accumulate long for patch (squared patch)
	"vpadal.s16    q7, q11            	\n" // pairwise add and accumulate long for source (squared source)
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

	"subs          %5, %5, #16        	\n" // subtract 16 from the pixel count
	"bne           Correlation16x16NormLevel	\n" // repeat until the row is complete

		// q0 and q1 are free to reuse
	"vadd.s32      q0, q4, q5         	\n" // add pairwise columns' sums of CrCorr in q4 and q5 (4+4) into q0
	"vpadd.s32     d12, d13           	\n" // pairwise addition (from 4 u32 to 2 u32) q6
	"vpadd.s32     d0, d1             	\n" // pairwise addition (from 4 u32 to 2 u32)
	"vpadd.s32     d14, d15           	\n" // pairwise addition (from 4 u32 to 2 u32) q7
	"vpadd.s32     d12, d12           	\n" // pairwise addition (from 2 u32 to 1 u32) q6 <-- sum of squares for patch
	"vpadd.s32     d0, d0             	\n" // pairwise addition (from 2 u32 to 1 u32)    <-- 16x16 CC, data type u32
	"vpadd.s32     d14, d14           	\n" // pairwise addition (from 2 u32 to 1 u32) q7 <-- sum of squares for source
	// ----------------- END of Cross Correlation and squared sums value calculation --------

	// --------------- D0 has 16x16 SAD, data type u32 --------------------------------------
	"vcvt.f32.s32  d13, d0            	\n" // convert CrossCorr to floating point        <-- 16x16 SAD, data type f32
	// -----------------------------------------------------------------------
	"vcvt.f32.s32   d0, d12           	\n" // convert sum of squares to floating point   <-- f32, patch
	"vmov.f32       d1, d0            	\n" // d1 = d0
	"vrsqrte.f32    d0, d0            	\n" // d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d3 = (3 - d0 * d2) / 2
	"vmul.f32       d0, d0, d3        	\n" // d0 = d0 * d3
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d4 = (3 - d0 * d3) / 2
	"vmul.f32       d8, d0, d3        	\n" // d8 = d0 * d4	                               <-- f32, patch
	// -----------------------------------------------------------------------
	"vcvt.f32.s32   d0, d14           	\n" // convert sum of squares to floating point    <-- f32, source
	"vmov.f32       d1, d0            	\n" // d1 = d0
	"vrsqrte.f32    d0, d0            	\n" // d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d3 = (3 - d0 * d2) / 2
	"vmul.f32       d0, d0, d3        	\n" // d0 = d0 * d3
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d4 = (3 - d0 * d3) / 2
	"vmul.f32       d9, d0, d3         	\n" // d8 = d0 * d4	                               <-- f32, source
	// -----------------------------------------------------------------------

	"vmul.f32       d0, d13, d8       	\n" // multiply CC by inverse square root of sum of squares of patch
	"vmul.f32       d0, d9            	\n" // multiply CC by inverse square root of sum of squares of source

	// -----------------------------------------------------------------------
	"vst1.f32      {d0[0]}, [%4]      	\n" // save result in variable 'sad'

	: "=r"(patch), "=r"(source), "=r"(meanSrc), "=r"(meanPatch), "=r"(sad), "=r"(pixels)
	: "0"(patch),   "1"(source),  "2"(meanSrc),  "3"(meanPatch),  "4"(sad),  "5"(pixels)
	: "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "q12", "q13", "r4", "cc"
	);
#endif
}

//-------------------------------------------------------------------------------------------------------------------------
// PRQA S 4213 1 // Parameter sad is modified in assembler code.
void CrossCorr16x16_NormLevel_Float_ARM_NEON_orig(const uint32_t *source, const uint32_t *patch, float32_t *meanSrc, float32_t *meanPatch, float32_t *sad)
{
#ifdef BMA_OPT_BY_ARM_NEON
	uint32_t pixels = 16*16;

	__asm__ volatile(
	// ----------------- START of mean value preparation ------------------------------------------
	"vld1.f32       {d24}, [%2]			\n" // copy 'meanSrc' to d24
	"vld1.f32       {d26}, [%3]			\n" // copy 'meanPatch' to d26
	// now duplicate it
	"vdup.32		q12, d24[0]         \n" // duplicate first <float> to all 4 <float>
	"vdup.32		q13, d26[0]         \n" // duplicate first <float> to all 4 <float>
	// ----------------- END of mean value preparation ------------------------------------------

	// ----------------- START of Cross Correlation and squared sums value calculation --------
	// Input: %0  - d0, d1 - pointer to 'patch', a 16x16 ROI in 400x640 array
	//   	  %1  - d2, d3 - pointer to 'source', continueos array of 16x16
	//       d24  - mean value of patch data
	//       d25  - mean value of source data
	// Used:  q0  - store 'patch' input data
	//        q1  - store 'source' input data
	//        q2  - store d0*d2 (patch * source) into q2
	//        q3  - store d1*d3 (patch * source) into q3
	//        q4  - pairwise add and accumulate long (crosscorr d0*d2)
	//        q5  - pairwise add and accumulate long (crosscorr d1*d3)
	//        q6  - pairwise add and accumulate long (squared patch)
	//        q7  - pairwise add and accumulate long (squared source)
	//        q8  - store squared patch into q8
	//        q9  - store squared patch into q9
	//        q10 - store squared source into q10
	//        q11 - store squared source into q11
	//        d24 - mean value u16 of patch[]
	//        d25 - mean value u16 of source[]
	// Output:d0  - cross correlation sum
	//        d12 - sum of squares of patch
	//        d14 - sum of squares of source

	"mov           r4,#0				\n" // initialize r4 by zeros TODO: is float 0.0 is 4 zero bytes?
	"vdup.u8       q2,r4              	\n" // initialize Q2 by zeros
	"vdup.u8       q3,r4              	\n" // initialize Q3 by zeros
	"vdup.u8       q4,r4              	\n" // initialize Q4 by zeros
	"vdup.u8       q5,r4              	\n" // initialize Q5 by zeros

	"vdup.u8       q8,r4              	\n" // initialize Q8 by zeros - curr sq patch
	"vdup.u8       q9,r4              	\n" // initialize Q9 by zeros - curr sq patch
	"vdup.u8       q6,r4              	\n" // initialize Q6 by zeros - sum of sq patch

	"vdup.u8       q10,r4             	\n" // initialize Q10 by zeros - curr sq source
	"vdup.u8       q11,r4             	\n" // initialize Q11 by zeros - curr sq source
	"vdup.u8       q7,r4              	\n" // initialize Q7 by zeros - sum of sq source
	// Note, q12 and q13 are filled out with <float> 'meanSrc' and 'meanPatch' respectively

	// ----------------- start rolled-out loop over 16 rows ------------------------------------------
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

"Correlation16x16NormLevelF32:         	\n" // start loop
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch

	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch

	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch

	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch

	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

	"subs          %5, %5, #16        	\n" // subtract 16 from the pixel count
	"bne           Correlation16x16NormLevelF32	\n" // repeat until the row is complete

	"vpadd.f32     d14, d14		       	\n" // pairwise addition (from 2 f32 to 1 f32) q7
	"vpadd.f32     d16, d16		       	\n" // pairwise addition (from 2 f32 to 1 f32) q8
	"vpadd.f32     d18, d18			  	\n" // pairwise addition (from 2 f32 to 1 f32) q9
	// ----------------- END of Cross Correlation and squared sums value calculation --------

	// q0 and q1 are free to reuse
	//

	// -----------------------------------------------------------------------
	//"vcvt.f32.s32   d0, d16           	\n" // convert sum of squares to floating point   <-- f32, patch
	"vmov.f32       d0, d16            	\n"
	"vmov.f32       d1, d0            	\n" // d1 = d0
	"vrsqrte.f32    d0, d0            	\n" // d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d3 = (3 - d0 * d2) / 2
	"vmul.f32       d0, d0, d3        	\n" // d0 = d0 * d3
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d4 = (3 - d0 * d3) / 2
	"vmul.f32       d8, d0, d3        	\n" // d8 = d0 * d4	                               <-- f32, patch
	// -----------------------------------------------------------------------
	//"vcvt.f32.s32   d0, d18           	\n" // convert sum of squares to floating point    <-- f32, source
	"vmov.f32       d0, d18            	\n"
	"vmov.f32       d1, d0            	\n" // d1 = d0
	"vrsqrte.f32    d0, d0            	\n" // d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d3 = (3 - d0 * d2) / 2
	"vmul.f32       d0, d0, d3        	\n" // d0 = d0 * d3
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d4 = (3 - d0 * d3) / 2
	"vmul.f32       d9, d0, d3         	\n" // d8 = d0 * d4	                               <-- f32, source
	// -----------------------------------------------------------------------

	"vmul.f32       d0, d14, d8       	\n" // multiply CC by inverse square root of sum of squares of patch
	"vmul.f32       d0, d9            	\n" // multiply CC by inverse square root of sum of squares of source

	// -----------------------------------------------------------------------
	"vst1.f32      {d0[0]}, [%4]      	\n" // save result in variable 'sad'

	: "=r"(patch), "=r"(source), "=r"(meanSrc), "=r"(meanPatch), "=r"(sad), "=r"(pixels)
	: "0"(patch),   "1"(source),  "2"(meanSrc),  "3"(meanPatch),  "4"(sad),  "5"(pixels)
	: "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "q12", "q13", "r4", "cc"
	);
#endif
}

//-------------------------------------------------------------------------------------------------------------------------
// PRQA S 4213 1 // Parameter sad is modified in assembler code.
void CrossCorr16x16_NormLevel_Float_ARM_NEON_opt(const uint32_t *source, const uint32_t *patch, float32_t *meanSrc, float32_t *meanPatch, float32_t *sad)
{
#ifdef BMA_OPT_BY_ARM_NEON
	uint32_t pixels = 16*16;

	__asm__ volatile(
	// ----------------- START of mean value preparation ------------------------------------------
	"vld1.f32       {d24}, [%2]			\n" // copy 'meanSrc' to d24
	"vld1.f32       {d26}, [%3]			\n" // copy 'meanPatch' to d26
	// now duplicate it
	"vdup.32		q12, d24[0]         \n" // duplicate first <float> to all 4 <float>
	"vdup.32		q13, d26[0]         \n" // duplicate first <float> to all 4 <float>
	// ----------------- END of mean value preparation ------------------------------------------

	// ----------------- START of Cross Correlation and squared sums value calculation --------
	// Input: %0  - d0, d1 - pointer to 'patch', a 16x16 ROI in 400x640 array
	//   	  %1  - d2, d3 - pointer to 'source', continueos array of 16x16
	//       d24  - mean value of patch data
	//       d25  - mean value of source data
	// Used:  q0  - store 'patch' input data
	//        q1  - store 'source' input data
	//        q2  - store d0*d2 (patch * source) into q2
	//        q3  - store d1*d3 (patch * source) into q3
	//        q4  - pairwise add and accumulate long (crosscorr d0*d2)
	//        q5  - pairwise add and accumulate long (crosscorr d1*d3)
	//        q6  - pairwise add and accumulate long (squared patch)
	//        q7  - pairwise add and accumulate long (squared source)
	//        q8  - store squared patch into q8
	//        q9  - store squared patch into q9
	//        q10 - store squared source into q10
	//        q11 - store squared source into q11
	//        d24 - mean value u16 of patch[]
	//        d25 - mean value u16 of source[]
	// Output:d0  - cross correlation sum
	//        d12 - sum of squares of patch
	//        d14 - sum of squares of source

	"mov           r4,#0				\n" // initialize r4 by zeros TODO: is float 0.0 is 4 zero bytes?
	"vdup.u8       q2,r4              	\n" // initialize Q2 by zeros
	"vdup.u8       q3,r4              	\n" // initialize Q3 by zeros
	"vdup.u8       q4,r4              	\n" // initialize Q4 by zeros
	"vdup.u8       q5,r4              	\n" // initialize Q5 by zeros

	"vdup.u8       q8,r4              	\n" // initialize Q8 by zeros - curr sq patch
	"vdup.u8       q9,r4              	\n" // initialize Q9 by zeros - curr sq patch
	"vdup.u8       q6,r4              	\n" // initialize Q6 by zeros - sum of sq patch

	"vdup.u8       q10,r4             	\n" // initialize Q10 by zeros - curr sq source
	"vdup.u8       q11,r4             	\n" // initialize Q11 by zeros - curr sq source
	"vdup.u8       q7,r4              	\n" // initialize Q7 by zeros - sum of sq source
	// Note, q12 and q13 are filled out with <float> 'meanSrc' and 'meanPatch' respectively

	// ----------------- start rolled-out loop over 16 rows ------------------------------------------
	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640

//"Correlation16x16NormLevelF32Opt:         	\n" // start loop
// -- iteration (row) 1 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 2 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 3 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 4 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 5 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 6 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 7 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 8 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 9 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 10 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 11 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 12 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 13 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 14 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 15 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

// -- iteration (row) 16 --
	// process first 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// second 4 bytes
	"vmovl.u8   	q10, d0           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d2           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// third 4 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d20           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d22           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

	// last 4 out of 16 bytes
	"vmovl.u8   	q10, d1           	\n" // convert pixels    u8 --> u16, patch
	"vmovl.u8   	q11, d3           	\n" // convert pixels    u8 --> u16, source
	"vmovl.u16   	q10, d21           	\n" // convert pixels    u16 --> u32, patch
	"vmovl.u16   	q11, d23           	\n" // convert pixels    u16 --> u32, source

	"vcvt.f32.u32   q2, q10           	\n" // convert pixels to floating point   u8 --> f32, patch
	"vcvt.f32.u32   q3, q11           	\n" // convert pixels to floating point   u8 --> f32, source

	"vsub.f32      q2, q12              \n"	// subtract mean value from patch data
	"vsub.f32      q3, q13              \n"	// subtract mean value from source data

	"vmul.f32      q5, q2, q2         	\n" // store squared patch into q5
	"vmul.f32      q4, q3, q2         	\n" // store (patch * source) into q4
	"vmul.f32      q6, q3, q3         	\n" // store squared source into q6

	"vadd.f32     d16, d16, d10	   		\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d8		   	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d12      	\n" // pairwise add and accumulate (squared source)
	"vadd.f32     d16, d16, d11 	   	\n" // pairwise add and accumulate (squared patch)
	"vadd.f32     d14, d14, d9        	\n" // pairwise add and accumulate (crosscorr)
	"vadd.f32     d18, d18, d13     	\n" // pairwise add and accumulate (squared source)

//	"vld1.u8       {d0-d1}, [%0]      	\n" // load 16 pixels from the patch
//	"add           %0, %0, #640       	\n" // move %0 pointer to patch to next line +640
//	"vld1.u8       {d2-d3}, [%1]!     	\n" // load 16 pixels from the source

//	"subs          %5, %5, #16        	\n" // subtract 16 from the pixel count
//	"bne           Correlation16x16NormLevelF32Opt	\n" // repeat until the row is complete

	"vpadd.f32     d14, d14		       	\n" // pairwise addition (from 2 f32 to 1 f32) q7
	"vpadd.f32     d16, d16		       	\n" // pairwise addition (from 2 f32 to 1 f32) q8
	"vpadd.f32     d18, d18			  	\n" // pairwise addition (from 2 f32 to 1 f32) q9
	// ----------------- END of Cross Correlation and squared sums value calculation --------

	// q0 and q1 are free to reuse
	//

	// -----------------------------------------------------------------------
	"vmov.f32       d0, d16            	\n"
	"vmov.f32       d1, d0            	\n" // d1 = d0
	"vrsqrte.f32    d0, d0            	\n" // d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d3 = (3 - d0 * d2) / 2
	"vmul.f32       d0, d0, d3        	\n" // d0 = d0 * d3
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d4 = (3 - d0 * d3) / 2
	"vmul.f32       d8, d0, d3        	\n" // d8 = d0 * d4	                               <-- f32, patch
	// -----------------------------------------------------------------------
	"vmov.f32       d0, d18            	\n"
	"vmov.f32       d1, d0            	\n" // d1 = d0
	"vrsqrte.f32    d0, d0            	\n" // d0 = ~ 1.0 / sqrt(d0)
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d3 = (3 - d0 * d2) / 2
	"vmul.f32       d0, d0, d3        	\n" // d0 = d0 * d3
	"vmul.f32       d2, d0, d1        	\n" // d2 = d0 * d1
	"vrsqrts.f32    d3, d2, d0        	\n" // d4 = (3 - d0 * d3) / 2
	"vmul.f32       d9, d0, d3         	\n" // d8 = d0 * d4	                               <-- f32, source
	// -----------------------------------------------------------------------

	"vmul.f32       d0, d14, d8       	\n" // multiply CC by inverse square root of sum of squares of patch
	"vmul.f32       d0, d9            	\n" // multiply CC by inverse square root of sum of squares of source

	// -----------------------------------------------------------------------
	"vst1.f32      {d0[0]}, [%4]      	\n" // save result in variable 'sad'

	: "=r"(patch), "=r"(source), "=r"(meanSrc), "=r"(meanPatch), "=r"(sad), "=r"(pixels)
	: "0"(patch),   "1"(source),  "2"(meanSrc),  "3"(meanPatch),  "4"(sad),  "5"(pixels)
	: "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "q12", "q13", "r4", "cc"
	);
#endif
}
// -----------------------------------------------------------------------
} // end of name space [tsc_math_neon]
// -----------------------------------------------------------------------
#endif
