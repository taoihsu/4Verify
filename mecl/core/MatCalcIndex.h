/*
 * MatCalcIndex.h
 *
 *  Created on: 05.09.2016
 *      Author: sai_hezol
 */

#ifndef CORE_MATCALCINDEX_H_
#define CORE_MATCALCINDEX_H_

#include "MeclAssert.h"

namespace mecl {
namespace core {

template<MatrixType_e MType>
class MatCalcIndex
{
public:
  static uint32_t calcIndex_u32(const uint32_t i,
                                const uint32_t j,
                                const uint32_t max_i,
                                const uint32_t max_j,
                                bool_t doAssert = false);
};

}
}

#endif /* CORE_MATCALCINDEX_H_ */
