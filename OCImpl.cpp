// ----------------------------------------------------------------------------
// --- written by Joseph Braun [23-MAR-2021]
// --- Copyright (c) Magna Electronics - Brampton 2021
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "OCImpl.h"
#include "OC.h"
#include "Configuration/OCCfg.h"
#include "DataCollectAgent.h"
#include "DataShare.h"
#include "version.h"
#include "DataProviderOC.h" // TODO: rework DataProviderOC
#include "oc\JobOC.h" // TODO: Still in progress

// ----------------------------------------------------------------------------
extern class COnlineCalibration* g__pPlugin;
extern class CAppCtrlInfo* g__pAppCtrl;
extern class CCANTranslationInfo* g__pObjCANTranslation;

COnlineCalibrationImpl::COnlineCalibrationImpl() :
    m_InitOK( false ),
    m_Enable( false ),
    m_pAppCtrl( NULL ),
    m_pAppCtrlPluginInfo( NULL ),
    m_processedFrameNum( 0 )
{
    // Using initialization lists
    m_pDataCollectAgent = NULL;
}

COnlineCalibrationImpl::~COnlineCalibrationImpl( void )
{
    Uninit();
}

//------------------------------------------------------------------------
bool COnlineCalibrationImpl::Init( Camera_ID id, ImplState go2State )
{
    // --- do not allow multiple init
    if( m_InitOK )
    {
        TRACE_0( m_hTracer, "Init() can't be called twice!!!" );
        return ( false );
    }

    for( vidImageProfileItr itr = m_pVidImageProfileMap.begin(); itr != m_pVidImageProfileMap.end(); itr++ )
    {
        //TODO:
        // - fill in m_output data --> not sure what this is used for yet
        // - fill in DataShare and DataCollectAgent info --> not sure how to set this up yet
        // - config data ? if this is needed
        // itr->first == Camera_ID, itr->second == CVideoImageInfo::VideoImageProfile*
        do_CameraParam* pCamInfo = static_cast<do_CameraParam*>( itr->second->pCamera );
        Camera_ID camOrientation = pCamInfo->camera_param.getOrientation();
        m_pCameraInfoMap[camOrientation] = new CameraProfile;
        m_pCameraInfoMap[camOrientation]->m_aCamIfo = ( *pCamInfo );
        m_pCameraInfoMap[camOrientation]->m_camOrientation = camOrientation;
        m_pCameraInfoMap[camOrientation]->m_pIncomingImagePtr = itr->second->pImagePtr;
    }

    m_Logging = g__pPlugin->m_LogResults;
    initOCData = new ocdata::OcData_s;
    m_IDataProvider = new oc::DataProviderOC(
        initOCData,
        g__pAppCtrl,
        m_pVidImageProfileMap[Front],
        m_pVidImageProfileMap[Left],
        m_pVidImageProfileMap[Rear],
        m_pVidImageProfileMap[Right],
        g__pObjCANTranslation,
        m_pCameraInfoMap[Front]->m_pIncomingImagePtr,
        m_pCameraInfoMap[Left]->m_pIncomingImagePtr,
        m_pCameraInfoMap[Rear]->m_pIncomingImagePtr,
        m_pCameraInfoMap[Right]->m_pIncomingImagePtr,
        m_Logging,
        m_pAppCtrl->m_FrameNumber,
        m_pCameraInfoMap[Front]->m_aCamIfo,
        m_pCameraInfoMap[Left]->m_aCamIfo,
        m_pCameraInfoMap[Rear]->m_aCamIfo,
        m_pCameraInfoMap[Right]->m_aCamIfo,
        OCcfg::configData.OCTargetCamera,
        ( unsigned long )this
    );
    m_IJob = new oc::JobOC( *m_IDataProvider );

    switch( OCcfg::configData.OCTargetCamera )
    {
        case tscApi::e_TscFrontCam:
            m_CamID = Front;
            break;

        case tscApi::e_TscRearCam:
            m_CamID = Rear;
            break;

        case tscApi::e_TscRightCam:
            m_CamID = Right;
            break;

        case tscApi::e_TscLeftCam:
            m_CamID = Left;
            break;
    };

    strcpy_s( m_CameraName, sizeof( m_CameraName ), m_pVidImageProfileMap[m_CamID]->CameraName );

    strcpy_s( m_CameraImageName, sizeof( m_CameraImageName ), m_pVidImageProfileMap[m_CamID]->DisplayName );

    sprintf_s( m_DcaViewName, sizeof( m_DcaViewName ), "%s[%s]", m_pAppCtrlPluginInfo->m_PlugInShortName, m_pVidImageProfileMap[m_CamID]->ShortName );

    SetImplName( m_pAppCtrlPluginInfo->m_PlugInShortName, m_pVidImageProfileMap[m_CamID]->ShortName );

    m_IJob->init_v();

    m_InitOK = true;

    m_Enable = true;

    m_Fps.Reset();

    m_ImplState = IMS_Data;

    if( m_pDataCollectAgent )
    {
        m_pDataCollectAgent->RegisterOCImpl( this );
    }

    m_FrProc.SetIgnoreThisFrame();
    return true;
}

//------------------------------------------------------------------------
bool COnlineCalibrationImpl::Uninit()
{
    if( m_IDataProvider )
    {
        delete m_IDataProvider;
        m_IDataProvider = NULL;
    }

    if( m_IJob )
    {
        delete m_IJob;
        m_IJob = NULL;
    }

    m_pDataCollectAgent = NULL;
    // --- initialize the variables to their reset state
    m_InitOK = false;
    m_Enable = false;
    m_hTracer = 0;
    return true;
}

// ------------------------------------------------------------------------------
void COnlineCalibrationImpl::Reconfigure( void )
{
    m_IJob->init_v();
}

// -----------------------------------------------------------
bool COnlineCalibrationImpl::Reset()
{
    //--- Reset skip frame
    m_FrProc.SetResetPending();
    m_FrProc.ClearResetPending();
    m_FrProc.SetFrameProcessInProgress();
    m_IJob->init_v();
    m_FrProc.SetIgnoreThisFrame();
    return true;
}

// -----------------------------------------------------------
bool COnlineCalibrationImpl::Process( int sysEvent )
{
    static const char fn[] = "Process";
    m_FrProc.SetFrameProcessInProgress();
    m_Fps.NewFrame();
    //TRACE_3(m_hTracer, "%s:%s[%s] Processing Fr[%u]", fn, m_pAppCtrlPluginInfo->m_PlugInName, m_ShortName, m_pAppCtrl->m_FrameNumber);
    // --- store the current frame number
    m_processedFrameNum = m_pAppCtrl->m_FrameNumber;

    if( !m_InitOK )
    {
        m_FrProc.SetIgnoreThisFrame();
        m_Fps.EndFrame();
        return false;
    }

    m_pVidImageProfileMap[m_CamID]->ImageReady( m_pAppCtrl->m_FrameNumber );

    if( m_pDataCollectAgent )
    {
        AgentPanelFeature* pFeature = m_pDataCollectAgent->FindPanelFromList( m_CameraImageName );

        if( pFeature )
        {
            DS_Panel_UpdateImage( pFeature->hImpl, pFeature->PanelIndex, m_pVidImageProfileMap[m_CamID]->pImagePtr );
        }
    }

    switch( ( SystemEvent )sysEvent )
    {
        case SE_Move2Standby:
        {
            switch( m_ImplState )
            {
                case IMS_Inactive:
                    m_ImplState = IMS_Standby;
                    break;

                case IMS_Data:
                    Reset();
                    m_ImplState = IMS_Standby;
                    break;

                case IMS_Standby:
                case IMS_Error:
                case IMS_Uninstalled:
                default:
                    break;
            };

            m_FrProc.SetFrameProcessCompleted();

            m_Fps.EndFrame();

            return ( true );
        }
        break;

        case SE_Move2Operational:
            if( m_ImplState == IMS_Standby )
            {
                m_ImplState = IMS_Data;
                break;
            }

        default:
            if( m_ImplState != IMS_Data )
            {
                DataShare_DebugMsgUpdate( ( unsigned long )this );
                m_FrProc.SetFrameProcessCompleted();
                m_Fps.EndFrame();
                return ( true );
            }

            break;
    }

    //------------------------------------- PROCESSING HAPPENS HERE ---------------------
    m_IJob->start_v();
    m_IJob->execute_v();
    m_IJob->end_v();
    m_AlgoState = m_IDataProvider->getOutData()->ocAlgoState_e;

    if( m_pDataCollectAgent )
    {
        m_pDataCollectAgent->Process( this );
    }

    DataShare_DebugMsgUpdate( ( unsigned long )this );
    m_FrProc.SetFrameProcessCompleted();
    m_Fps.EndFrame();
    return true;
}
