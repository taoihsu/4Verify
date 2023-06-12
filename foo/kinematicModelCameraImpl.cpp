// ----------------------------------------------------------------------------
// --- Written by Hany Kashif [20-Nov-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2013
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "kinematicModel.h"
// ----------------------------------------------------------------------------
using km::KinematicModelCameraImpl;
KinematicModelCameraImpl::KinematicModelCameraImpl():
ModuleImpl()
{
    // Using initialization lists
}

bool_t KinematicModelCameraImpl::loadConfiguration_b(void)
{
  return true;
}

bool_t KinematicModelCameraImpl::Init(tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po, km::KinematicModelInfo* b_Info_po, km::KinematicModelConfig* b_Config_po, km::KinematicModelCameraConfig* b_CameraConfig_po, sint64_t i_Tracer_s64, tscApi::enuCameraID i_CameraID_t)
{
    bool_t v_Status_b = true;
    if ( !preInit_b(b_TSCCtrlInfo_po, b_Info_po, b_Config_po, b_CameraConfig_po, i_Tracer_s64, i_CameraID_t) )
    {
        v_Status_b = false;
    }
    
    else
    {    
    initOK_b = true;
    }
    return v_Status_b;
}

void KinematicModelCameraImpl::cleanupLocalData_v(void)
{
}

// ----------------------------------------------------------------------------
bool_t KinematicModelCameraImpl::start_b(void)
{
    enable_v();
    return true;
}

// ----------------------------------------------------------------------------
bool_t KinematicModelCameraImpl::process_b(void)
{
    bool_t v_Status_b = true;
    if ( !processImplObjects_b() )
    {
        TRACE_0( m_hTracer, "Failed to process implementation objects");
        v_Status_b = false;
    }    
    // --- process successful
    return v_Status_b;
}

// ----------------------------------------------------------------------------
bool_t KinematicModelCameraImpl::processImplObjects_b(void)
{
    bool_t v_Status_b = true;
    for( ImplObjsItr v_Itr_t = implObjs_x.rwBegin_o(); v_Itr_t != implObjs_x.end_o(); ++v_Itr_t )
    {
        if(!(*v_Itr_t).isEnabled_b())
        {
            continue;
        }
        if(!(*v_Itr_t).process_b())
        {
            TRACE_0( m_hTracer, "Failed to process frame");
            v_Status_b = false;
            break;
        }
    }
    return v_Status_b;
}

// ----------------------------------------------------------------------------
bool_t KinematicModelCameraImpl::unInit_b(void)
{
	reset_v();

    // --- initialize the variables to their reset state
    hTracer_s64 = 0;
    tSCCtrlInfo_po = NULL;
    config_px = NULL;
    cameraConfig_px = NULL;
    info_px = NULL;

    initOK_b = false;

    cameraID_t = tscApi::e_TscFrontCam;

    return true;
}

// ----------------------------------------------------------------------------
void KinematicModelCameraImpl::reset_v(void)
{
    implObjs_x.clear_v();

    availableFrameNumber_u32 = 0;

    enable_b = false;
}

// ----------------------------------------------------------------------------
uint16_t KinematicModelCameraImpl::getFreeObjectCountfromCollection_u16( void )
{
    uint16_t v_FreeObjects_u16 = 0;
    for( ImplObjsItr v_Itr_t = implObjs_x.rwBegin_o(); v_Itr_t != implObjs_x.end_o(); ++v_Itr_t )
    {
        if( !(*v_Itr_t).isEnabled_b() )
        {
            v_FreeObjects_u16++;
        }
    }
    return v_FreeObjects_u16;
}
// ----------------------------------------------------------------------------
km::KinematicModelImpl* KinematicModelCameraImpl::getFirstFreeObjectfromCollection_po(void)
{
    KinematicModelImpl* v_Pobj_po = NULL;
    for( ImplObjsItr v_Itr_t = implObjs_x.rwBegin_o(); v_Itr_t != implObjs_x.end_o(); ++v_Itr_t )
    {
        if(!(*v_Itr_t).isEnabled_b())
        {
            v_Pobj_po = reinterpret_cast<KinematicModelImpl*>(&(*v_Itr_t));
            break;
        }
    }
    return v_Pobj_po;
}
// ----------------------------------------------------------------------------
km::KinematicModelImpl* KinematicModelCameraImpl::createImplObject_po(void)
{
    KinematicModelImpl* v_Pobj_po = &implObjs_x.addItem_ro();
    return v_Pobj_po;
}
// ----------------------------------------------------------------------------
void KinematicModelCameraImpl::FreeImplObject(km::KinematicModelImpl* b_Pobj_po)   // PRQA S 4212
{   
    //Recycle object
    if(b_Pobj_po != NULL)
    {
        b_Pobj_po->disable_v();
        TRACE_3(m_hTracer, "Impl Object added to the free pool...Total Impl objects in Collection = %lu, free = %lu, Max = ", implObjs_x.size_u32(), getFreeObjectCountfromCollection_u16(), tsc_cfg::NUM_KM_IMPL_OBJS);
    }
}
// ----------------------------------------------------------------------------
km::KinematicModelImpl* KinematicModelCameraImpl::getImplObject_po(void)
{
    TRACE_3(m_hTracer, "Total Impl objects in Collection = %lu, free = %lu, Max = %lu", implObjs_x.size_u32(), getFreeObjectCountfromCollection_u16(), tsc_cfg::NUM_KM_IMPL_OBJS);
    KinematicModelImpl* v_Pobj_po = getFirstFreeObjectfromCollection_po();
    if( v_Pobj_po != NULL )
    {
        // reset the object before starting re-use
        TRACE_0(m_hTracer, "Found a free object from the collection - reusing...");
        v_Pobj_po->reset_v();
    }
    else
    {
        // create the impl object
        TRACE_0(m_hTracer, "No free object available in the collection - creating a new object...");
        v_Pobj_po = createImplObject_po();
        v_Pobj_po->Init(tSCCtrlInfo_po, info_px, config_px, cameraConfig_px, hTracer_s64, cameraID_t);
    }
    v_Pobj_po->enable_v();
    return v_Pobj_po;
}

// ----------------------------------------------------------------------------

