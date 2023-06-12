// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [12-Apr-2013]
// --- Modified by Ehsan Parvizi [6-May-2013] for design changes
// --- Modified by Ehsan Parvizi [07-Jun-2013] for dual rate execution model
// --- Modified by Rathi G. R. [25-Jun-2013] for Impl object pool re-use
// --- Modified by Ehsan Parvizi [09-Sep-2013] for performance optimization
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2013
// ----------------------------------------------------------------------------
#ifndef __KINEMATICMODELIMPL_H_
#define __KINEMATICMODELIMPL_H_
// ----------------------------------------------------------------------------
#include "moduleImpl.h"
namespace km
{
class KinematicModelInfo;
class KinematicModelConfig;
class KinematicModelCameraConfig;

//-----------------------------------------------------------------------------
class KinematicModelImpl : public control::ModuleImpl<KinematicModelInfo,KinematicModelConfig,KinematicModelCameraConfig>
{
    public:
    KinematicModelImpl();
 
    virtual bool_t process_b(void);
    virtual bool_t Init(tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po, km::KinematicModelInfo* b_Info_po, km::KinematicModelConfig* b_Config_po, km::KinematicModelCameraConfig* b_CameraConfig_po, sint64_t i_Tracer_s64, tscApi::enuCameraID i_CameraID_t);
    bool_t Init(tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po, km::KinematicModelInfo* b_Info_po, km::KinematicModelConfig* b_Config_po, sint64_t i_Tracer_s64);
    virtual bool_t unInit_b(void);
    virtual void cleanupLocalData_v(void);
    virtual bool_t start_b(void);  // Not used
    virtual void reset_v(void);
    void updateMotionVector_v();
    void SetCameraOffset(float64_t i_XOffsetMM_f64, float64_t i_YOffsetMM_f64, float64_t i_AngleOffsetDeg_f64);
    void ResetMotionVector( bool_t i_Accumulate_b );
    bool_t GetMotionVector( float64_t& o_DeltaXMM_rf64, float64_t& o_DeltaYMM_rf64, float64_t& o_HeadingInRadians_rf64) const;

    private:
    virtual bool_t loadConfiguration_b(void);

    bool_t  isCameraOffsetSet_b;
    float64_t eadingInRadians_f64;
    float64_t displacementCoG_af64[2];
    float64_t cameraOffsetMM_af64[3];
    float64_t cameraRotationMatrix_af64[2][2];
};
}
//-----------------------------------------------------------------------------
#endif
