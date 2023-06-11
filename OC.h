// ----------------------------------------------------------------------------
// --- written by Joseph Braun [12-MAR-2021]
// --- Copyright (c) Magna Electronics - Brampton 2021
// ----------------------------------------------------------------------------
#ifndef ONLINECALIBRATION_H
#define ONLINECALIBRATION_H

#include <vector>
#include "OCIntf.h"
#include <FrameProcessStatus.h>

// ----------------------------------------------------------------------------
// --- global data exposed by this plug-in
class COnlineCalibrationInfo : public I_AppCtrlPlugInInfo
{
public:
    COnlineCalibrationInfo()
    {
    }
    ~COnlineCalibrationInfo()
    {
    }
};

// ----------------------------------------------------------------------------
// --- AppCtrl plug-in
class COnlineCalibration : public I_AppCtrlAPI
{
public:

    // --- Plug-in implementation ---------------------------------------------
    COnlineCalibration();
    ~COnlineCalibration()
    {
        ;
    }


    // --- plug-in functions --------------------------------------------------
    // --- initialize the plug-in
    // --- plug-ins are expected to find their slot in CPlugInInfo and fill the pointers
    virtual bool Init( CAppCtrlInfo* pAppCtrlIfo, unsigned PluginIndex );

    // --- uninitialize the plug-in
    virtual void Uninit( void );

    // --- reset the plug-in
    virtual void Reset( void );

    // --- reload the configuration
    virtual void Reconfigure( void );

    // --- process frame
    virtual void Processframe( SystemEvent sysEvent );

    // --- implementation functions
    bool LoadOCConfiguration();

    // --- configuration
    int m_NumSkipFrame;

    void ReportPluginVersionInfo( CPlugInInfo& PlugIn );

    // --- plug-in is initialized
    bool m_InitOK;

    bool m_LogResults;

private:

};



#endif // !ONLINECALIBRATION_H