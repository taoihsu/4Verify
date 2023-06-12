// ----------------------------------------------------------------------------
// --- Written by Ehsan Parvizi [19-Aug-2013]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2013
// ----------------------------------------------------------------------------
// --- TSCImplConfig.cpp - Loading configuration method for Impl Object
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "tscAlgImpl.h"
// ----------------------------------------------------------------------------
bool_t tsc::TSCAlgImpl::loadConfiguration_b()
{
    return true;
}

// PRQA S 4327 ++
// PRQA S 4212 ++
bool_t tsc::TSCAlgImpl::UpdateExternalConfiguration(tscApi::enuCameraID i_CameraID_t) const
{
    bool_t v_Status_b = true;
    mecl::core::UnusedParameter(i_CameraID_t); //To suppress warning
    return v_Status_b;
}
// PRQA S 4327 --
// PRQA S 4212 --
