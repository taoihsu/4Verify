// ----------------------------------------------------------------------------
// --- written by Joseph Braun [23-MAR-2021]
// --- Copyright (c) Magna Electronics - Brampton 2021
// ----------------------------------------------------------------------------
#ifndef ONLINECALIBRATIONIMPL_H
#define ONLINECALIBRATIONIMPL_H
// ----------------------------------------------------------------------------
// --- never include stdafx.h in the include file, since the path determined by the
// --- compiler when this file is included in a different project may conflict
// --- with the project types (especially with MFC and Non-MFC projects)
// --- #include "stdafx.h"
// ----------------------------------------------------------------------------


#include <DataCollectAgent.h>
#include "OC.h"
#include "OCIntf.h"
#include "tscApi.h"
#include "PluginImplAPI.h"
#include <FrameStats.h>
#include <FrameProcessStatus.h>
#include <ThreadProcessStatus.h>
#include "VideoImageIntf.h"
#include "OCStructs.h"
#include "OCData.h"


namespace oc
{
    class IDataProviderOC;
}

namespace container
{
    class IJob;
}


class COnlineCalibrationImpl : public I_PluginImpl
{
public:

    // --- initialiser
    COnlineCalibrationImpl();
    ~COnlineCalibrationImpl();

    // ------------------------------------------------------------------------
    // --- plug-in functions --------------------------------------------------
    bool Suspend();
    bool Resume();
    bool Reset();

    // Not Used
    bool Init( Camera_ID id, ImplState go2StateImplState );
    bool Uninit( void );
    bool Process( int sysEvent );

    void Reconfigure( void );
    virtual bool GetDcaDisplayPanelInfo( int index, void* pArg );
    virtual const char* GetDcaDisplayPanelName( int index );
    virtual int GetNumDcaDisplayPanels();
    bool GetDcaImageAndGraphs( void* pArg, unsigned int& numGraphs, void* pGraphPool, const unsigned int maxGraphNum );

    // --- hold the needed pointers and variables (received from the plug-in)
    bool m_InitOK; // --- default to false
    bool m_Enable; // --- plugin is enabled. default to false
    CAppCtrlInfo* m_pAppCtrl;
    CPlugInInfo* m_pAppCtrlPluginInfo;

    // --- which frame last processed?
    unsigned long m_processedFrameNum;


    class DataCollectAgent* m_pDataCollectAgent;

    COnlineCalibrationAPI m_objAPI;
    COnlineCalibrationInfo m_objIfo;

    CCANTranslationInfo* m_pObjCANTranslation;

    std::map<Camera_ID, CameraProfile*> m_pCameraInfoMap;
    std::map<Camera_ID, CVideoImageInfo::VideoImageProfile*> m_pVidImageProfileMap;

    float32_t m_YawRateTimeStamp;

    ocdata::OcDataToMcu_s initMCUData;

    ocdata::OcData_s* initOCData;

    oc::IDataProviderOC* m_IDataProvider;
    container::IJob* m_IJob;
    int m_AlgoState;

    bool m_Logging;
    //char m_ShortName[10];
    tscApi::enuCameraID m_OCTargetCamera;
    Camera_ID m_CamID;
    const char* getShortName();
};




#endif // !ONLINECALIBRATIONIMPL_H
