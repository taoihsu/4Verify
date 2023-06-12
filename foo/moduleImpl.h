// ----------------------------------------------------------------------------
// --- Written by Hany Kashif
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __MODULEIMPL_H_
#define __MODULEIMPL_H_
// ----------------------------------------------------------------------------
#include "tscApi.h"
#include "tracing.h"
// ----------------------------------------------------------------------------

namespace control
{
    typedef enum {
        e_ErrTypeNoError = 0,
        e_ErrTypeNonRecoverable,
        e_ErrTypeRecoverable
    } enuErrType;
template <class ModuleTypeInfo, class ModuleTypeConfig, class ModuleTypeCameraConfig>
class ModuleImpl
{
    public:
    virtual bool_t process_b(void) = 0;
    virtual bool_t Init(tscApi::TSCCtrlInfo* pTSCCtrlInfo, ModuleTypeInfo* pInfo, ModuleTypeConfig* pConfig, ModuleTypeCameraConfig* pCameraConfig, sint64_t tracer, tscApi::enuCameraID cameraID) = 0;
    virtual bool_t unInit_b(void) = 0;
    virtual void cleanupLocalData_v(void) = 0;
    virtual bool_t start_b(void) = 0;
    virtual void reset_v(void) = 0;
    
    void enable_v(void)   {enable_b = true;}
    void disable_v(void) {enable_b = false;}
    bool_t isEnabled_b(void) const     {return enable_b;}
    bool_t isInitOk_b(void) const   {return initOK_b;}
    bool_t notInError_b(void) const { return (errorType_t == e_ErrTypeNoError); }
    bool_t isErrorNonRecoverable_b(void) const { return (errorType_t == e_ErrTypeNonRecoverable); }
    void setErrorRecoverable_v(bool_t i_Recoverable_b) {
        errorType_t = i_Recoverable_b ? e_ErrTypeRecoverable : e_ErrTypeNonRecoverable;
    }
    void resetErrorType_v(void) { errorType_t = e_ErrTypeNoError; }
    

    ModuleImpl() :
        hTracer_s64(0),
        tSCCtrlInfo_po(NULL),
        config_px(NULL),
        cameraConfig_px(NULL),
        info_px(NULL),
        availableFrameNumber_u32(0),
        initOK_b(false),
        enable_b(false),
        errorType_t(e_ErrTypeNoError)
    {
    }

    protected:
    virtual bool_t loadConfiguration_b(void) = 0;

    bool_t preInit_b(tscApi::TSCCtrlInfo* b_TSCCtrlInfo_po, ModuleTypeInfo* b_Info_px, ModuleTypeConfig* b_Config_px, ModuleTypeCameraConfig* b_CameraConfig_px, sint64_t i_Tracer_s64, tscApi::enuCameraID i_CameraID_t)
    {
        bool_t v_Ret_b = true;
        if( initOK_b )
        {
            TRACE_0( hTracer_s64, "Init() can't be called twice!!!" );
            v_Ret_b = false ;
        }
        
        else
        {
            hTracer_s64 = i_Tracer_s64;
            tSCCtrlInfo_po = b_TSCCtrlInfo_po;
            info_px = b_Info_px;
            config_px = b_Config_px;
            cameraConfig_px = b_CameraConfig_px;
            cameraID_t = i_CameraID_t;

            if( config_px == NULL )
            {
                v_Ret_b = false ;
            }
            else if( !loadConfiguration_b() )
            {
                TRACE_0( hTracer_s64, "Failed to load configuration..." );
                initOK_b = false;
                v_Ret_b = false;
            }
            else
            {
            }
        }
        return v_Ret_b;
    }
    //PRQA S 2101 10 //protected members needed here
    sint64_t hTracer_s64;
    tscApi::TSCCtrlInfo *tSCCtrlInfo_po;
    ModuleTypeConfig *config_px;
    ModuleTypeCameraConfig *cameraConfig_px;
    ModuleTypeInfo *info_px;
    uint32_t availableFrameNumber_u32;
    bool_t initOK_b;
    bool_t enable_b;
    tscApi::enuCameraID cameraID_t;
    enuErrType errorType_t;
};
}

#endif

