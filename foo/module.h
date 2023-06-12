// ----------------------------------------------------------------------------
// --- Written by Hany Kashif
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __MODULE_H_
#define __MODULE_H_
// ----------------------------------------------------------------------------
#include "mecl/mecl.h"
#include "tscApi.h"
#include "configuration.h"
#include "tracing.h"
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
namespace control
{
template <class ModuleType, class ModuleTypeImpl, class ModuleTypeInfo, class ModuleTypeConfig, class ModuleTypeCameraConfig>
class Module
{
public:
  virtual bool_t Init(tscApi::TSCCtrlInfo* pTSCCtrlInfo) = 0;
  virtual bool_t processFrame_b(void) = 0;
  virtual void uninit_v(void) = 0;
  virtual void reset_v(void)= 0;

  ModuleTypeInfo& getModuleInfo_rt(void)       {return info_x;}
  ModuleTypeConfig* getModuleConfig_pt(void)   {return &config_x;}
  ModuleTypeCameraConfig* getModuleCameraConfig_pt(tscApi::enuCameraID cameraID)   {return &cameraConfigs_ax[cameraID];}
  uint64_t getTracer_u64(void) const            {return hTracer_s64;}

  static ModuleType& getInstance_rt()
  {
    static ModuleType instance;// PRQA S 4633 // destructor is empty
    return instance;
  }

  ModuleTypeImpl* getImplObject_pt(tscApi::enuCameraID i_CameraID_t)
  {
    return &implArray_t[i_CameraID_t];
  }

  void cleanup_v( void )
  {
    TRACE_0(hTracer_s64, "Pause was called, clean up local data");
    // Pause all imp objects
    for(ImplArrayItr v_Itr_t = implArray_t.rwBegin_o(); v_Itr_t != implArray_t.end_o(); ++v_Itr_t )
    {
      if ( (*v_Itr_t).isEnabled_b() )
      {
        (*v_Itr_t).cleanupLocalData_v();
      }
    }
  }


protected:
  virtual bool_t loadConfiguration_b(void) = 0;
  // PRQA S 2127, 2131 1 //Making it non-pure inline virtual function is intentional
  virtual bool_t SetCameraSpecificConfig(const tscApi::enuCameraID i_CameraID_t)
  {
    mecl::core::UnusedParameter(i_CameraID_t);
    return true;
  }

  Module() :
    hTracer_s64(0),
    tSCCtrlInfo_px(NULL),
    config_x(),
    info_x(),
    isInitialReading_b(false)
  {
  }

  ~Module()
  {
  }

  bool_t processImplObjects_b(void)
  {
    bool_t v_Ret_b = true;
    for(ImplArrayItr v_Itr_t = implArray_t.rwBegin_o(); v_Itr_t != implArray_t.end_o(); ++v_Itr_t )
    {
      if(!(*v_Itr_t).isEnabled_b())
      {
        continue;
      }
      if(!(*v_Itr_t).process_b())
      {
        TRACE_0( hTracer_s64, "Failed to process frame");
        v_Ret_b = false;
      }
    }
    return v_Ret_b;
  }

  void uninitImplObjects_v(void)
  {
    // Uninit all objects
    for(ImplArrayItr v_Itr_t = implArray_t.rwBegin_o(); v_Itr_t != implArray_t.end_o(); ++v_Itr_t )
    {
      (*v_Itr_t).unInit_b();
    }
    implArray_t.init_v();
  }

  void resetImplObjects_v(void)
  {
    // Uninit all objects
    for(ImplArrayItr v_Itr_t = implArray_t.rwBegin_o(); v_Itr_t != implArray_t.end_o(); ++v_Itr_t )
    {
      (*v_Itr_t).reset_v();
    }
    implArray_t.init_v();
  }
  //PRQA S 6200 1 //input parameter can't be made const here.
  bool_t preInit_b( tscApi::TSCCtrlInfo* i_TSCCtrlInfo_po )
  {
    bool_t v_Ret_b = true;

    tSCCtrlInfo_px = i_TSCCtrlInfo_po;
    // load the configuration
    if(!loadConfiguration_b())
    {
      TRACE_0( hTracer_s64, "Failed to load configuration...");
      v_Ret_b = false;
    }

    if(true == v_Ret_b)
    {
      // were we initialized OK?
      TRACE_1(hTracer_s64, "Init called with (CTSCCtrlInfo *) %08X", reinterpret_cast <uint64_t> (tSCCtrlInfo_px));

      for (uint8_t v_Index_u8 = 0; v_Index_u8 < tsc_cfg::MAX_NUM_CAMERA_CALIB; v_Index_u8++)
      {
        tscApi::enuCameraID v_CameraID_t = static_cast<tscApi::enuCameraID>(v_Index_u8);
                    if( !initCamera_b( v_CameraID_t, i_TSCCtrlInfo_po ) )
        {
          v_Ret_b = false;
          break;
        }
      }
    }

    return v_Ret_b;
  }

        bool_t initCamera_b( tscApi::enuCameraID i_CameraID_t, tscApi::TSCCtrlInfo* i_TSCCtrlInfo_po )
  {
    bool_t v_Ret_b = true;
    if ( !SetCameraSpecificConfig( i_CameraID_t ))
    {
      TRACE_1( hTracer_s64, "Failed to set camera specific config for [%s]", tsc_trace::kCameraStrings[i_CameraID_t].c_str() );
      v_Ret_b = false;
    }

    if (true == v_Ret_b)
    {
      if( !implArray_t[i_CameraID_t].Init(tSCCtrlInfo_px, &info_x, &config_x, &cameraConfigs_ax[i_CameraID_t], hTracer_s64, i_CameraID_t) )
      {
        TRACE_1( hTracer_s64, "Failed to initialize camera [%s]", tsc_trace::kCameraStrings[i_CameraID_t].c_str() );
        v_Ret_b = false;
      }
    }
    return v_Ret_b;
  }

  typedef typename mecl::core::Array<ModuleTypeImpl, tsc_cfg::MAX_NUM_CAMERA_CALIB> ImplArray;
  typedef typename ImplArray::iterator ImplArrayItr;
  //PRQA S 2101 7 //protected members needed here
  sint64_t hTracer_s64;
  tscApi::TSCCtrlInfo *tSCCtrlInfo_px;
  ModuleTypeConfig config_x;
  ModuleTypeInfo info_x;
  ImplArray implArray_t;
  ModuleTypeCameraConfig cameraConfigs_ax[tsc_cfg::MAX_NUM_CAMERA_CALIB];
  bool_t isInitialReading_b;
};
}

#endif

