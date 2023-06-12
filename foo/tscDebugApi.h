// ----------------------------------------------------------------------------
// --- Written by Hany Kashif
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __TSCDEBUGAPI_H_
#define __TSCDEBUGAPI_H_
// ----------------------------------------------------------------------------
#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)   // PRQA S 1070
#include "tscApi.h"
#include "tscAlgStructs.h"
// ----------------------------------------------------------------------------

bool_t TSC_GetDetailedCalibrationResult( tscApi::enuCameraID cameraID, tsc::DetailedCalibrationResult* detailedCalibrationResult );
mecl::core::ArrayList< fc::ValidFeature, tsc_cfg::NUM_VALID_INDICES>& TSC_GetValidFeatures(tscApi::enuCameraID cameraID);
#endif

#endif

