//--------------------------------------------------------------------------
/// @file core.h
/// @brief This is the main header file for all mecl core classes
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Sebastian Pliefke (sebastian.pliefke@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{
/// @brief Magna Electronics Common Library

#ifndef MECL_CORE_CORE_H_
#define MECL_CORE_CORE_H_

/// @brief includes defining base classes for mecl models (namespace mecl::core)
#include "Matrix.h"
#include "Matrix3x3.h"
#include "Projection4D.h"
#include "Point.h"
#include "RotationMatrix.h"
#include "Quaternion.h"
#include "Singleton.h"
#include "TriangleMatrix.h"
#include "Vector.h"
/// @}

/// @brief includes integrated from old core lib (namespace mecl)
#include "Array.h"
#include "ArrayBase.h"
#include "ArrayList.h"
#include "ArrayN.h"
#include "BitStream.h"
#include "Collection.h"
#include "Helpers.h"
#include "MeclAssert.h"
#include "MeclLogging.h"
#include "MeclTypes.h"
#include "MemArray.h"
#include "MemArrayList.h"
#include "MultiArrayAccess.h"

/// @brief includes defining additional new classes (not used in camera model)
#include "Float16.h"

#endif // MECL_CORE_CORE_H_

/// @}
