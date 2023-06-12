// ----------------------------------------------------------------------------
// --- Written by Hany Kashif [20-Nov-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __KINEMATICMODELCAMERAIMPL_H_
#define __KINEMATICMODELCAMERAIMPL_H_

#include "mecl/mecl.h"
#include "moduleImpl.h"
#include "kinematicModelImpl.h"
#include "configuration.h"
namespace km
{
class KinematicModelInfo;
class KinematicModelConfig;
class KinematicModelCameraConfig;

//-----------------------------------------------------------------------------
class KinematicModelCameraImpl : public control::ModuleImpl<KinematicModelInfo,KinematicModelConfig,KinematicModelCameraConfig>
{
    public:
    KinematicModelCameraImpl();
 
    virtual bool_t process_b(void);
    virtual bool_t Init(tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po, km::KinematicModelInfo* b_Info_po, km::KinematicModelConfig* b_Config_po, km::KinematicModelCameraConfig* b_CameraConfig_po, sint64_t i_Tracer_s64, tscApi::enuCameraID i_CameraID_t);
    virtual bool_t unInit_b(void);
    virtual void cleanupLocalData_v(void);
    virtual bool_t start_b(void);
    virtual void reset_v(void);
   
    KinematicModelImpl* getImplObject_po(void);
    void FreeImplObject(KinematicModelImpl* b_Pobj_po);
    
    private:
    virtual bool_t loadConfiguration_b(void);

    bool_t processImplObjects_b(void);
    KinematicModelImpl* getFirstFreeObjectfromCollection_po(void);
    uint16_t getFreeObjectCountfromCollection_u16(void);
    KinematicModelImpl* createImplObject_po(void);

    mecl::core::ArrayList<KinematicModelImpl, tsc_cfg::NUM_KM_IMPL_OBJS> implObjs_x;
    typedef mecl::core::ArrayList<KinematicModelImpl, tsc_cfg::NUM_KM_IMPL_OBJS>::iterator ImplObjsItr;
    typedef mecl::core::ArrayList<KinematicModelImpl, tsc_cfg::NUM_KM_IMPL_OBJS>::const_iterator ConstImplObjsItr;
};
}
//-----------------------------------------------------------------------------
#endif
