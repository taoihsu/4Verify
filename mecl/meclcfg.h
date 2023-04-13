/* ===========================================================================
 * MAGNA Electronics - C O N F I D E N T I A L
 *  This document in its entirety is CONFIDENTIAL and may not be disclosed,
 *  disseminated or distributed to parties outside MAGNA Electronics
 *  without written permission from MAGNA Electronics.
 * ===========================================================================
 * SHORT:   meclcfg
 * DESIGN:  <tbd>
 * DESCRIPTION: Type abstractions for mecl objects.
 * ======================================================================== */

#ifndef _MECLCFG_H_
#define _MECLCFG_H_

#define meclcfg_VERSION_ID_H "$Id: meclcfg.h 1.3 2015/10/28 11:52:51EDT Kildeby, Allan (SAI_AlKil) draft  $"

#include "mecl/model/Model.h"

/* +++ Public Type definitions, Enumerations, Defines +++++++++++++++++++++ */

namespace mecl
{
  // MECL specific type definitions.
  typedef mecl::model::NoLens<float32_t> NoLens_t;
  typedef mecl::model::LensRadial<float32_t, 6U, 6U> LensRadial_t;
  typedef mecl::model::LensCylinder<float32_t> LensCylinder_t;
  typedef mecl::model::Sensor<float32_t> Sensor_t;
  typedef mecl::model::Pinhole<float32_t> Pinhole_t;
  typedef mecl::model::Camera<float32_t> Camera_t;
  typedef mecl::model::SingleView<float32_t> SingleView_t;
  typedef mecl::core::Point2D<float32_t> Point2D_t;
  typedef mecl::core::Point3D<float32_t> Point3D_t;
  typedef mecl::core::Point4D<float32_t> Point4D_t;
  typedef mecl::core::RotationMatrix<float32_t> RotationMatrix_t;
}

/* +++ Public Variables, Constants ++++++++++++++++++++++++++++++++++++++++ */

/* +++ Public Macros ++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

/* +++ Public Function Prototypes +++++++++++++++++++++++++++++++++++++++++ */

/* === End Of Header File ================================================= */

#endif // _MECLCFG_H_
