/*
 * Checksum.h
 *
 *  Created on: 09.08.2016
 *      Author: sai_hezol
 */

#ifndef MECL_MATH_CHECKSUM_H_
#define MECL_MATH_CHECKSUM_H_

template <typename CheckSum_T, typename Input_T>
class CheckSum
{
  static CheckSum_T fletcher_x( const Input_T* i_Data_px, const uint32_t i_Size_u32);
};

template<>
uint32_t CheckSum<uint32_t, uint16_t>::fletcher_x(const uint16_t *i_Data_px,  uint32_t  i_Len_u32 )
{
  uint32_t v_Sum1_u32 = 0xffff;
  uint32_t v_Sum2_u32 = 0xffff;

  while (i_Len_u32)
  {
    uint32_t v_Len_u32 = i_Len_u32 > 360 ? 360 : i_Len_u32;
    i_Len_u32 -= v_Len_u32;
    do
    {
      v_Sum1_u32 += *i_Data_px++;
      v_Sum2_u32 += v_Sum1_u32;
    } while (--v_Len_u32);
    v_Sum1_u32 = (v_Sum1_u32 & 0xffff) + (v_Sum1_u32 >> 16);
    v_Sum2_u32 = (v_Sum2_u32 & 0xffff) + (v_Sum2_u32 >> 16);
  }
  /* Second reduction step to reduce sums to 16 bits */
  v_Sum1_u32 = (v_Sum1_u32 & 0xffff) + (v_Sum1_u32 >> 16);
  v_Sum2_u32 = (v_Sum2_u32 & 0xffff) + (v_Sum2_u32 >> 16);
  return v_Sum2_u32 << 16 | v_Sum1_u32;
}

template<>
uint16_t CheckSum<uint16_t, uint8_t>::fletcher_x(const uint8_t *i_Data_px,  uint32_t  i_Len_u32 )
{
  uint16_t v_Sum1_u16 = 0xff;
  uint16_t v_Sum2_u16 = 0xff;

  while (i_Len_u32)
  {
    uint32_t v_Len_u32 = i_Len_u32 > 21 ? 21 : i_Len_u32;
    i_Len_u32 -= v_Len_u32;
    do {
      v_Sum1_u16 += *i_Data_px++;
      v_Sum2_u16 += v_Sum1_u16;
    } while (--v_Len_u32);
    v_Sum1_u16 = (v_Sum1_u16 & 0xff) + (v_Sum1_u16 >> 8);
    v_Sum2_u16 = (v_Sum2_u16 & 0xff) + (v_Sum2_u16 >> 8);
  }
  /* Second reduction step to reduce sums to 8 bits */
  v_Sum1_u16 = (v_Sum1_u16 & 0xff) + (v_Sum1_u16 >> 8);
  v_Sum2_u16 = (v_Sum2_u16 & 0xff) + (v_Sum2_u16 >> 8);
  return v_Sum2_u16 << 8 | v_Sum1_u16;
}



#endif /* MECL_MATH_CHECKSUM_H_ */
