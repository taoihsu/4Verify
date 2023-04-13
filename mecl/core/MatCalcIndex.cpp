/*
 * MatCalcIndex.cpp
 *
 *  Created on: 05.09.2016
 *      Author: sai_hezol
 */

#include "MatCalcIndex.h"

namespace mecl {
namespace core {

template<>
uint32_t MatCalcIndex<e_Default>::calcIndex_u32(const uint32_t i,
                                                const uint32_t j,
                                                const uint32_t m,
                                                const uint32_t n,
                                                bool_t doAssert)
{
  if (doAssert)
  {
    AssertFunction(i < m && j < n,"Matrix indices out of bounds.");
  }
  return i * n + j;
};

template<>
uint32_t MatCalcIndex<e_UpperTriangle>::calcIndex_u32(const uint32_t i,
                                                      const uint32_t j,
                                                      const uint32_t m,
                                                      const uint32_t n,
                                                      bool_t doAssert)
{
  static uint32_t s_Channels_u32 = n*(n+1) / 2 + 1;
  AssertFunction(not(doAssert && (j < i) && (i < m) && (j < n) ),"Tried to assign to invalid index.");
  return j < i ? s_Channels_u32 : i * n - (i * (i - 1) ) / 2 + j - i;
};

template<>
uint32_t MatCalcIndex<e_LowerTriangle>::calcIndex_u32(const uint32_t i,
                                                      const uint32_t j,
                                                      const uint32_t m,
                                                      const uint32_t n,
                                                      bool_t doAssert)
{
  static uint32_t s_Channels_u32 = n*(n+1) / 2 + 1;
  AssertFunction(not(doAssert && (i < j) && (i < m) && (j < n)), "Tried to assign to invalid index.");
  return i < j ? s_Channels_u32 : j * n - (j * (j - 1) ) / 2 + i - j;
};

}
}
