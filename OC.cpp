// ----------------------------------------------------------------------------
// --- written by Joseph Braun [12-MAR-2021]
// --- Copyright (c) Magna Electronics - Brampton 2021
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "OC.h"
#include "OCImpl.h"
#include "../Configuration/OCCfg.h"
#include "PluginImplAPI.h"
#include "DataCollectAgent.h"
#include <version.h>
#include "Camera.h"
//-----------------------------------------------------------------------------
#ifdef _DEBUG
    #define new DEBUG_NEW
#endif
// ----------------------------------------------------------------------------
using namespace AppCtrl;
using std::string;
// ----------------------------------------------------------------------------
#pragma warning( push )
#pragma warning( disable : 4351 )
// --- The one and only application object and other globals
CWinApp theApp;
Ipp64u g__ipp_cpuid = 0;
class COnlineCalibration* g__pPlugin = NULL;
static COnlineCalibrationAPI* g__pPluginAPI = NULL;
static COnlineCalibrationInfo* g__pPluginInfo = NULL;

// ----------------------------------------------------------------------------
class CAppCtrlInfo* g__pAppCtrl = NULL;
class CVideoImageInfo* g__pObjVideoImage = NULL;
class CCANTranslationInfo* g__pObjCANTranslation = NULL;
class CKinematicModelAPI* g__pKinematicModelAPI = NULL;
class DataCollectAgent* g__pDataCollectAgent = NULL;

// ----------------------------------------------------------------------------
static T_DataLocker g__Lock;

typedef std::vector< COnlineCalibrationImpl* > vecPluginImpl;
typedef vecPluginImpl::iterator vecPluginImplItr;

// Plug-in Implementation vector
static vecPluginImpl g__vecImpl;

// ----------------------------------------------------------------------------
static COnlineCalibrationImpl* GetObjectfromCollection( unsigned long handle )
{
    for( vecPluginImplItr itr = g__vecImpl.begin(); itr != g__vecImpl.end(); ++itr )
    {
        if( ( unsigned long )( *itr ) == handle )
        {
            return  *itr;
        }
    }

    return NULL;
}

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
BOOL APIENTRY DllMain( HANDLE hModule, DWORD ul_reason_for_call, LPVOID lpReserved )
{
    ( void* )hModule;
    ( void* )lpReserved;

    // --- dll entry point
    switch( ul_reason_for_call )
    {
        case DLL_PROCESS_ATTACH:
        {
            // --- initialize MFC and print and error on failure
            if( !AfxWinInit( ::GetModuleHandle( NULL ), NULL, ::GetCommandLine(), 0 ) )
            {
                ErrorMsg( __FILE__, __LINE__, AppCtrlPlugInOnlineCalibration, "MFC initialization failed" );
                return FALSE;
            }

            // --- initialize libraries
            // --- initialize IPP
            IppStatus ipp_init_status = ippInit();

            if( ipp_init_status != ippStsNoErr )
            {
                // --- ipp init failed
                // --- "Failed to Initialize Libraries or Platform not Supported"
                // --- we may be running on non-intel platforms
                // --- ErrorMsg( __FILE__, __LINE__, AppCtrlPlugInPedestrianDetection, "Failed to initialize IPP" );
                // --- return false;
            }

            // --- check if IPP is supported on this platform
            Ipp64u u64FeaturesMask = ippCPUID_GETINFO_A;
            Ipp32u u32CpuidInfoRegs[] =
            {
                1, 0, 0, 0
            }
            ;
            IppStatus ipp_status = ippGetCpuFeatures( &u64FeaturesMask, u32CpuidInfoRegs );

            if( ipp_status != ippStsNoErr )
            {
                // --- ipp init failed
                // --- "Failed to Initialize Libraries or Platform not Supported"
                ErrorMsg( __FILE__, __LINE__, AppCtrlPlugInOnlineCalibration, "Failed to initialize IPP" );
                return ( false );
            }

            g__ipp_cpuid = u64FeaturesMask & 0x1ff;
            // --- initialize fastgraph
            fg_vbinit();
            // --- create our plug-in objects
            g__pPlugin = new COnlineCalibration;
            g__pPluginAPI = new COnlineCalibrationAPI;
            g__pPluginInfo = new COnlineCalibrationInfo;
            g__pDataCollectAgent = new DataCollectAgent;
        }
        break;

        case DLL_THREAD_ATTACH:
            break;

        case DLL_THREAD_DETACH:
            break;

        case DLL_PROCESS_DETACH:
        {
            // --- cleanups, if necessary
            for( vecPluginImplItr itr = g__vecImpl.begin(); itr != g__vecImpl.end(); ++itr )
            {
                if( *itr )
                {
                    delete  *itr;
                }

                *itr = NULL;
            }

            g__vecImpl.clear();
            // --- release all global objects
            delete g__pPlugin;
            g__pPlugin = NULL;
            delete g__pPluginAPI;
            g__pPluginAPI = NULL;
            delete g__pPluginInfo;
            g__pPluginInfo = NULL;
            delete g__pDataCollectAgent;
            g__pDataCollectAgent = NULL;
            // --- cleanup fastgraph
            fg_vbfin();
        }
        break;
    }

    return TRUE;
}

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
extern "C" __declspec( dllexport )I_AppCtrlAPI* GetAppCtrlPlugInIface( void )
{
    return g__pPlugin;
}

// ----------------------------------------------------------------------------

COnlineCalibration::COnlineCalibration() :
    m_InitOK( false ),
    m_LogResults( false )
{}


// -----------------------------------------------------------------------------
// --- initialize the plug-in
bool COnlineCalibration::Init( CAppCtrlInfo* pAppCtrlIfo, unsigned pluginIndex )
{
    m_pAppCtrl = pAppCtrlIfo;
    g__pAppCtrl = pAppCtrlIfo;
    CPlugInInfo& plugin = pAppCtrlIfo->m_aPlugIn[pluginIndex];
    m_pAppCtrlPluginInfo = &pAppCtrlIfo->m_aPlugIn[pluginIndex];
    plugin.Init( AppCtrlPlugInOnlineCalibration, "OC", pAppCtrlIfo->m_ApplicationIndex, this, g__pPluginAPI, g__pPluginInfo );
    strcpy_s( m_CfgBaseName, sizeof( m_CfgBaseName ), plugin.m_CfgBaseName );

    if( g__pPluginInfo == NULL )
    {
        if( pAppCtrlIfo->m_PopupErrorMessage )
        {
            ErrorMsg( __FILE__, __LINE__, plugin.m_PlugInName, "Cannot have NULL PluginInfo object" );
        }

        m_InitOK = false;
    }

    if( m_pAppCtrl->m_VIDIfo.m_Width < 100 || m_pAppCtrl->m_VIDIfo.m_Height < 100 )
    {
        ErrorMsg( __FILE__, __LINE__, plugin.m_PlugInName, "There is no valid image or image dimension is invalid" );
        m_InitOK = false;
        pAppCtrlIfo->m_req = Req_Quit;
        return ( false );
    }

    if( !LoadConfiguration() )
    {
        if( pAppCtrlIfo->m_PopupErrorMessage )
        {
            ErrorMsg( __FILE__, __LINE__, plugin.m_PlugInName, "Failed to load configuration" );
        }

        pAppCtrlIfo->m_req = Req_Quit;
        return ( false );
    }
    else
    {
        if( !LoadOCConfiguration() )
        {
            pAppCtrlIfo->m_req = Req_Quit;
            return false;
        }

        m_InitOK = true;
    }

    OCcfg::setPluginNames( plugin.m_PlugInName, plugin.m_PlugInShortName );
    OCcfg::SetPluginImgSetting( plugin.m_ImageWidth, plugin.m_ImageHeight, m_pAppCtrl->m_VIDIfo.m_ImgType, plugin.m_ImageDistortionType );

    if( g__pDataCollectAgent )
    {
        g__pDataCollectAgent->m_pAppCtrl = pAppCtrlIfo;
        g__pDataCollectAgent->m_OverlaySettings = &pAppCtrlIfo->Overlays[0];

        if( !g__pDataCollectAgent->Initialize( plugin.m_PlugInName, m_pAppCtrl->m_ApplicationIndex, plugin.UserInputAllowed ) )
        {
            if( pAppCtrlIfo->m_PopupErrorMessage )
            {
                ErrorMsg( __FILE__, __LINE__, plugin.m_PlugInName, "Cannot initialize DataCollectAgent class" );
            }

            return false;
        }
    }

    // --- were we initialized OK?
    TRACE_1( m_hTracer, "Init called with (CAppCtrlInfo *) %08X", reinterpret_cast < unsigned long >( pAppCtrlIfo ) );
    // --- check the dependencies of previously loaded plug-ins
    g__pObjVideoImage = static_cast <CVideoImageInfo*>( m_pAppCtrl->GetPluginInfoObject( AppCtrlPlugInVideoImage ) );
    g__pObjCANTranslation = static_cast <CCANTranslationInfo*>( m_pAppCtrl->GetPluginInfoObject( AppCtrlPlugInCANTranslation ) );
    g__pKinematicModelAPI = static_cast <CKinematicModelAPI*>( m_pAppCtrl->GetPluginAPI( AppCtrlPluginKinematicModel ) );

    if( g__pObjVideoImage == NULL )
    {
        if( pAppCtrlIfo->m_PopupErrorMessage )
        {
            ErrorMsg( __FILE__, __LINE__, plugin.m_PlugInName, "Can't find required plug-in [%s]", AppCtrlPlugInVideoImage );
        }

        pAppCtrlIfo->m_req = Req_Quit;
        return ( false );
    }

    if( g__pObjCANTranslation == NULL )
    {
        if( pAppCtrlIfo->m_PopupErrorMessage )
        {
            ErrorMsg( __FILE__, __LINE__, plugin.m_PlugInName, "Can't find required plug-in [%s]", AppCtrlPlugInCANTranslation );
        }

        pAppCtrlIfo->m_req = Req_Quit;
        return ( false );
    }

    if( g__pKinematicModelAPI == NULL )
    {
        if( pAppCtrlIfo->m_PopupErrorMessage )
        {
            ErrorMsg( __FILE__, __LINE__, plugin.m_PlugInName, "Can't find required plug-in [%s]", AppCtrlPluginKinematicModel );
        }

        pAppCtrlIfo->m_req = Req_Quit;
        return ( false );
    }

    if( plugin.m_ViewString.size() == 0 )
    {
        if( pAppCtrlIfo->m_PopupErrorMessage )
        {
            ErrorMsg( __FILE__, __LINE__, plugin.m_PlugInName, "Views not assigned to [%s]", plugin.m_PlugInName );
        }

        pAppCtrlIfo->m_req = Req_Quit;
        return ( false );
    }

    // --- load the configuration
    COnlineCalibrationImpl* pOCImpl = ( COnlineCalibrationImpl* )g__pPluginAPI->CreateImplObject();

    for( unsigned int i = 0; i < plugin.m_ViewString.size(); i++ )
    {
        char* CameraName = plugin.m_ViewString[i];
        CVideoImageInfo::VideoImageProfile* pProfile = g__pObjVideoImage->GetImageProfilePtr( plugin.m_ImageDistortionType, plugin.m_ImageWidth, plugin.m_ImageHeight, CameraName );

        if( pProfile == NULL )
        {
            continue;
        }

        do_CameraParam* pcamInfo = static_cast<do_CameraParam*>( pProfile->pCamera );
        Camera_ID camOrientation = pcamInfo->camera_param.getOrientation();
        pOCImpl->m_pVidImageProfileMap[camOrientation] = pProfile;
    }

    pOCImpl->m_hTracer = m_hTracer;
    pOCImpl->m_pAppCtrl = m_pAppCtrl;
    pOCImpl->m_pAppCtrlPluginInfo = m_pAppCtrlPluginInfo;
    pOCImpl->m_pObjCANTranslation = g__pObjCANTranslation;
    pOCImpl->m_pDataCollectAgent = g__pDataCollectAgent;

    if( !pOCImpl->Init( OCcfg::configData.ImplTargetCamera,  I_PluginImpl::IMS_Data ) )
    {
        string msgFormat = "[Init]: Failed to initialize Impl object for %s";
        TRACE_1( m_hTracer, msgFormat.c_str(), m_pAppCtrlPluginInfo->m_PlugInName );
        ErrorMsg( __FILE__, __LINE__, plugin.m_PlugInName, msgFormat.c_str(), m_pAppCtrlPluginInfo->m_PlugInName );
        pAppCtrlIfo->m_req = Req_Quit;
        return false;
    }

    pOCImpl->m_FrProc.SetMinSkipFrame( m_pAppCtrlPluginInfo->m_NumSkipFrames );

    if( g__vecImpl.size() == 0 )
    {
        string msgFormat = "There is no impl class for %s";
        TRACE_1( m_hTracer, msgFormat.c_str(), m_pAppCtrlPluginInfo->m_PlugInName );
        ErrorMsg( __FILE__, __LINE__, m_pAppCtrlPluginInfo->m_PlugInName, msgFormat.c_str(), m_pAppCtrlPluginInfo->m_PlugInName );
        pAppCtrlIfo->m_req = Req_Quit;
        return false;
    }

    //TODO: write these DCA functions
    if( g__pDataCollectAgent )
    {
        g__pDataCollectAgent->CreateDisplayPanels();
        g__pDataCollectAgent->CreateDisplayViews( m_pAppCtrlPluginInfo->m_OwnsDisplayControl );
    }

    ReportPluginVersionInfo( pAppCtrlIfo->m_aPlugIn[pluginIndex] );
    // --- init OK
    return true;
}

// --------------------------------------------------
void
COnlineCalibration::ReportPluginVersionInfo( CPlugInInfo& PlugIn )
{
    strcpy_s( PlugIn.m_VersionStr, sizeof( PlugIn.m_VersionStr ), VER_FILE_VERSION_STR );
    PlugIn.m_pDataCollectAgent = ( void* )g__pDataCollectAgent;

    if( g__vecImpl.size() > 0 )
    {
        strcpy_s( PlugIn.m_ComponentList, sizeof( PlugIn.m_ComponentList ), g__vecImpl[0]->m_CameraName );
    }

    for( unsigned int i = 1; i < g__vecImpl.size(); i++ )
    {
        strcat_s( PlugIn.m_ComponentList, sizeof( PlugIn.m_ComponentList ), "," );
        strcat_s( PlugIn.m_ComponentList, sizeof( PlugIn.m_ComponentList ), g__vecImpl[i]->m_CameraName );
    }

    AppCtrl::ExtractDateString( VER_DATE_STR, PlugIn.m_DateStr );
}

// ----------------------------------------------------------------------------

// --- uninitialize the Plug-In
void COnlineCalibration::Uninit( void )
{
    // --- were we initialized OK?
    TRACE_0( m_hTracer, "Uninit called" );

    if( !m_InitOK )
    {
        TRACE_0( m_hTracer, "Can't uninitialize without proper initial configuration" );
        return;
    }

    m_pAppCtrlPluginInfo->SetUnlockThread();

    if( m_pAppCtrlPluginInfo->m_Threaded )
    {
        for( vecPluginImplItr itr = g__vecImpl.begin(); itr != g__vecImpl.end(); ++itr )
        {
            I_PluginImpl& impl = **itr;
            impl.m_ThProc.m_Cmd = ThreadProcessStatus::Cmd_Quit;
            // --- wait for the thread to stop
            WaitForSingleObject( impl.m_ThProc.m_hHandle, INFINITE );
            CloseHandle( impl.m_ThProc.m_hHandle );
        }
    }
}

// ----------------------------------------------------------------------------
// --- reset the plug-in
void COnlineCalibration::Reset( void )
{
    // --- were we initialized OK?
    TRACE_0( m_hTracer, "Reset called" );

    if( !m_InitOK )
    {
        TRACE_0( m_hTracer, "Can't reset without proper initial configuration" );
        return;
    }

    // synchronize AppCtrl access from other plugins
    T_WriterLock lock( g__Lock );

    // --- Reset every impl object
    for( vecPluginImplItr itr = g__vecImpl.begin(); itr != g__vecImpl.end(); ++itr )
    {
        COnlineCalibrationImpl& impl = **itr;

        if( m_pAppCtrlPluginInfo->m_Threaded )
        {
            if( impl.m_ThProc.m_hHandle == NULL )
            {
                continue;
            }
            else
            {
                impl.m_FrProc.SetResetPending();
            }
        }
        else
        {
            if( !impl.Reset() )
            {
                string msgFormat = "Failed to reset";
                TRACE_0( m_hTracer, msgFormat.c_str() );

                if( m_pAppCtrl->m_PopupErrorMessage )
                {
                    ErrorMsg( __FILE__, __LINE__, AppCtrlPlugInOnlineCalibration, msgFormat.c_str() );
                }

                m_pAppCtrl->m_req = Req_Quit;
                break;
            }
        }
    }

    // --- nothing to do for us
    ( void )0;
}

// ----------------------------------------------------------------------------

// --- reload the configuration
void COnlineCalibration::Reconfigure( void )
{
    // --- were we initialized OK?
    TRACE_0( m_hTracer, "Reconfigure called" );

    if( !m_InitOK )
    {
        TRACE_0( m_hTracer, "Can't re-configure without proper initial configuration" );
        return;
    }

    // --- load the configuration
    if( !LoadOCConfiguration() )
    {
        ErrorMsg( __FILE__, __LINE__, m_pAppCtrlPluginInfo->m_PlugInName, "Failed to load configuration" );
        m_pAppCtrl->m_req = Req_Quit;
        m_InitOK = false;
    }

    for( vecPluginImplItr itr = g__vecImpl.begin(); itr != g__vecImpl.end(); ++itr )
    {
        ( ( COnlineCalibrationImpl* )( *itr ) )->Reconfigure();
    }
}

// ----------------------------------------------------------------------------
// --- process frame
void COnlineCalibration::Processframe( SystemEvent sysEvent )
{
    static const char fn[] = "Processframe";

    // --- were we initialized OK?
    if( !m_InitOK )
    {
        TRACE_0( m_hTracer, "Can't process frame without proper initial configuration" );
        return;
    }

    //unsigned long currFrameNum = m_pAppCtrl->m_FrameNumber;
    // --- process this frame
    Timer totalTimer;
    totalTimer.Start();

    if( sysEvent == SE_Move2Operational )
    {
        m_pAppCtrlPluginInfo->SetUnlockThread();
    }

    for( vecPluginImplItr itr = g__vecImpl.begin(); itr != g__vecImpl.end(); ++itr )
    {
        I_PluginImpl& impl = **itr;

        if( m_pAppCtrlPluginInfo->m_Threaded )
        {
            ThreadProcessStatus& ThProc = impl.m_ThProc;

            if( sysEvent != SE_NoEvent )
            {
                impl.m_ThProc.m_SystemEvent = sysEvent;
                impl.m_FrProc.CancelSkipFrame();
            }

            if( ThProc.m_hHandle == NULL )
            {
                DWORD_PTR threadMask = m_pAppCtrlPluginInfo->m_ThreadAffinityMask & m_pAppCtrl->m_ProcessAffinityMask;
                ThProc.Setup( m_pAppCtrlPluginInfo->m_ThreadMutexName, &impl, m_pAppCtrlPluginInfo->m_ThreadPriority, threadMask );
            }
            else if( impl.m_FrProc.IsResetPending() )
            {
                impl.m_FrProc.ClearNewFrameAvailable();
            }
            else
            {
                if( impl.m_FrProc.IsFrameProcessCompleted() )
                {
                    impl.m_FrProc.SetNewFrameAvailable( m_pAppCtrl->m_FrameNumber );
                }
            }

            // --- check if the processor thread is terminated
            if( WaitForSingleObject( impl.m_ThProc.m_hHandle, 0 ) == WAIT_OBJECT_0 )
            {
                DWORD exitCode;
                GetExitCodeThread( impl.m_ThProc.m_hHandle, &exitCode );
                CloseHandle( ThProc.m_hHandle );

                // in case of failure
                if( exitCode != ThreadProcessStatus::ErrCode_Success )
                {
                    string msgFormat = "[%s]: ProcessorThread [%u] terminated with error [%lu]";
                    TRACE_3( m_hTracer, msgFormat.c_str(), fn, ThProc.m_ThreadID, exitCode );

                    if( m_pAppCtrl->m_PopupErrorMessage )
                    {
                        ErrorMsg( __FILE__, __LINE__, m_pAppCtrlPluginInfo->m_PlugInName, msgFormat.c_str(), fn, ThProc.m_ThreadID, exitCode );
                    }

                    m_pAppCtrl->m_req = Req_Quit;
                    break;
                }
            }
        }
        else
        {
            impl.m_FrProc.SetNewFrameAvailable( m_pAppCtrl->m_FrameNumber );

            if( impl.m_FrProc.IsThisASkippedFrame() )
            {
                continue;
            }

            if( !( impl.Process( sysEvent ) ) )
            {
                string msgFormat = "Failed to process frame";
                TRACE_0( m_hTracer, msgFormat.c_str() );

                if( m_pAppCtrl->m_PopupErrorMessage )
                {
                    ErrorMsg( __FILE__, __LINE__, m_pAppCtrlPluginInfo->m_PlugInName, msgFormat.c_str() );
                }

                m_pAppCtrl->m_req = Req_Quit;
                break;
            }
        }
    }

    totalTimer.Stop();

    if( sysEvent == SE_Move2Standby )
    {
        m_pAppCtrlPluginInfo->SetLockThread();
    }

    //TRACE_2( m_hTracer, "Processframe End: Frame[%lu]. Took [%.0f] ms", currFrameNum, totalTimer.GetElapsedTime() );
}

// ----------------------------------------------------------------------------
bool COnlineCalibration::LoadOCConfiguration( void )
{
    if( g__pAppCtrl->IsLogResultPlugin( AppCtrlPlugInOnlineCalibration ) )
    {
        m_LogResults = true;
    }

    if( !OCcfg::loadConfiguration( m_configData, m_hTracer, m_pAppCtrlPluginInfo->m_PlugInName, m_pAppCtrl->m_CameraProfile ) )
    {
        return false;
    }

    m_InitOK = true;
    return ( m_InitOK );
}

//-----------------------------------------------------------------------------
// --- plugin API
//-----------------------------------------------------------------------------
unsigned long COnlineCalibrationAPI::CreateImplObject()
{
    // synchronize AppCtrl access from other plugins
    T_WriterLock lock( g__Lock );
    // --- create the impl object
    COnlineCalibrationImpl* pobj = new COnlineCalibrationImpl;
    g__vecImpl.push_back( pobj );
    // --- return the handle to our object
    return ( unsigned long )pobj;
}
//-----------------------------------------------------------------------------
void COnlineCalibrationAPI::ReleaseImplObject( unsigned long objHandle )
{
    // synchronize AppCtrl access from other plugins
    T_WriterLock lock( g__Lock );

    // --- release the implementation object from the collection
    for( vecPluginImplItr itr = g__vecImpl.begin(); itr != g__vecImpl.end(); ++itr )
    {
        if( ( unsigned long )( *itr ) == objHandle )
        {
            delete *itr;
            g__vecImpl.erase( itr );
            return;
        }
    }
}
//-----------------------------------------------------------------------------
bool COnlineCalibrationAPI::IsValidImplObject( unsigned long objHandle )
{
    // synchronize AppCtrl access from other plugins
    T_WriterLock lock( g__Lock );
    // --- check to see if this is a valid impl object
    COnlineCalibrationImpl* pobj = GetObjectfromCollection( objHandle );
    return pobj != NULL;
}
// ---------------------------------------------------------------------
bool COnlineCalibrationAPI::SetLock()
{
    // --- lock the entire data structure
    g__Lock.AquireWriteLock();
    TRACE_0( g__pPlugin->m_hTracer, "Setting the global lock..." );
    return true;
}
// ----------------------------------------------------------------------------
bool COnlineCalibrationAPI::ReleaseLock()
{
    // --- release the entire data structure
    g__Lock.ReleaseLock();
    TRACE_0( g__pPlugin->m_hTracer, "Releasing the global lock..." );
    return true;
}
// ----------------------------------------------------------------------------
void* COnlineCalibrationAPI::GetDataCollectAgent()
{
    return g__pDataCollectAgent;
}