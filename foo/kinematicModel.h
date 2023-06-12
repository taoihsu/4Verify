// ----------------------------------------------------------------------------
// --- Written by James Turk [28-Mar-2013]
// --- Modified by Rathi G. R. [12-Apr-2013] for architectural change of implementation
// --- Modified by Ehsan Parvizi [24-Jun-2014]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __KINEMATICMODEL__H_
#define __KINEMATICMODEL__H_
// ----------------------------------------------------------------------------
#define _USE_MATH_DEFINES
// ----------------------------------------------------------------------------
#include <math.h>
#include "tscApi.h"
#include "kinematicModelCameraImpl.h"
#include "module.h"
// ----------------------------------------------------------------------------
namespace km
{
class KinematicModelConfig
{
  public:

    KinematicModelConfig() : tireCircumferencePerPulseMM_f32(0)
    {
    }
    void PutM_TrCrcmfrncPrPls_mm(float32_t i_Param_f32) { tireCircumferencePerPulseMM_f32 = i_Param_f32; }

  private:
    float32_t tireCircumferencePerPulseMM_f32;
};

class KinematicModelCameraConfig
{
};
// ----------------------------------------------------------------------------
// --- global data exposed by this plug-in
class KinematicModelInfo
{
  public:
    KinematicModelInfo() :
        vehicleSlipAngleRad_f64(0),
        distanceCoG2FrontAxisMM_f64(0),
        distanceCoG2RearAxisMM_f64(0),
        headingIncrement_f64(0),
        meanTraveledDistanceMM_f64(0),
        accumulatedHeading_f64(0),
        accumulatedDisplacementCOG_af64(),
        straightMotionDistanceThreshMM_f64(0)
    {
    }

    float64_t getMVhclSlpAnglRd_f64(void) const { return vehicleSlipAngleRad_f64; }
    float64_t getMDstncCG2FrntAxsMM_f64(void) const { return distanceCoG2FrontAxisMM_f64; }
    float64_t getMDstncCG2RrAxsMM_f64(void) const { return distanceCoG2RearAxisMM_f64; }
    float64_t getMHdngIncrmnt_f64(void) const { return headingIncrement_f64; }
    float64_t getMMnTrvldDstncMM_f64(void) const { return meanTraveledDistanceMM_f64; }
    float64_t getMAccmltdHdng_f64(void) const { return accumulatedHeading_f64; }
    float64_t *getMAccmltdDsplcmntCOG_pf64(void) { return &accumulatedDisplacementCOG_af64[0]; }
    float64_t getMStrghtMtnDstncThrshMM_f64(void) const { return straightMotionDistanceThreshMM_f64; }
    
    void putMVhclSlpAnglRd_v(float64_t i_Param_f64) { vehicleSlipAngleRad_f64 = i_Param_f64; }
    void putMDstncCG2FrntAxsMM_v(float64_t i_Param_f64) { distanceCoG2FrontAxisMM_f64 = i_Param_f64; }
    void putMDstncCG2RrAxsMM_v(float64_t i_Param_f64) { distanceCoG2RearAxisMM_f64 = i_Param_f64; }
    void putMHdngIncrmnt_v(float64_t i_Param_f64) { headingIncrement_f64 = i_Param_f64; }
    void putMMnTrvldDstncMM_v(float64_t i_Param_f64) { meanTraveledDistanceMM_f64 = i_Param_f64; }
    void putMStrghtMtnDstncThrshMM_v(float64_t i_Param_f64) { straightMotionDistanceThreshMM_f64 = i_Param_f64; }

  private:
    float64_t vehicleSlipAngleRad_f64;
    float64_t distanceCoG2FrontAxisMM_f64;
    float64_t distanceCoG2RearAxisMM_f64;
    float64_t headingIncrement_f64;
    float64_t meanTraveledDistanceMM_f64;
    float64_t accumulatedHeading_f64;
    float64_t accumulatedDisplacementCOG_af64[2];
    float64_t straightMotionDistanceThreshMM_f64;
};

class KinematicModel : public control::Module<KinematicModel, KinematicModelCameraImpl, KinematicModelInfo, KinematicModelConfig, KinematicModelCameraConfig>
{
    // SB friend class Module;
    template <class ModuleType, class ModuleTypeImpl, class ModuleTypeInfo, class ModuleTypeConfig, class ModuleTypeCameraConfig>
    friend class control::Module;

    public:
    virtual bool_t Init(tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po);
    virtual bool_t processFrame_b(void);
    virtual void uninit_v(void);
    virtual void reset_v(void);
    bool_t UpdateExternalConfiguration(tscApi::enuCameraID i_CameraID_t);

    private:
    virtual bool_t loadConfiguration_b(void);
    KinematicModel();
    ~KinematicModel();
    void computeVehicleDisplacement_v(void);
};
}
//-----------------------------------------------------------------------------
#endif
