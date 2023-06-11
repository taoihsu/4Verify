// ----------------------------------------------------------------------------
// --- written by Joseph Braun [23-MAR-2021]
// --- Copyright (c) Magna Electronics - Brampton 2021
// ----------------------------------------------------------------------------

#include "stdafx.h"
#include "OCImpl.h"
#include "DataCollectAgent.h"
#include "DataCollect_const.h"
#include "Graph2Xml.h"
#include "T_DataLock.h"

static T_DataLocker g__OCAgent_Lock;

DataCollectAgent::DataCollectAgent( void )
{
    m_pAppCtrl = NULL;
    m_Menu.numItems = 0;
    m_Menu.item[0].enabled = true;
    Assert( m_Menu.numItems <= sizeof( m_Menu.item ) / sizeof( m_Menu.item[0] ) );
}

DataCollectAgent::~DataCollectAgent( void )
{
}

// ----------------------------------------------------------------------------
void
DataCollectAgent::RegisterOCImpl( class COnlineCalibrationImpl* pOCImpl )
{
    T_WriterLock lock( g__OCAgent_Lock );

    if( pOCImpl == NULL )
    {
        return;
    }

    m_ImplList.push_back( pOCImpl );
    AddPluginImplName( pOCImpl->m_PluginImplName );
}

// ----------------------------------------------------------------------------

void DataCollectAgent::ProcessUserKey( UserKeyIDs UserKey )
{
    ( void )UserKey;
    return;
}

// ----------------------------------------------------------------------------------------
void DataCollectAgent::CreateDisplayPanels()
{
    T_WriterLock lock( g__OCAgent_Lock );
    Assert( m_Panels.size() == 0 );

    if( m_Panels.size() > 0 )
    {
        return;
    }

    for( unsigned int i = 0; i < m_ImplList.size(); i++ )
    {
        I_PluginImpl* pImpl = ( I_PluginImpl* )m_ImplList[i];

        if( pImpl == NULL )
        {
            continue;
        }

        int numDisplayPanels = pImpl->GetNumDcaDisplayPanels();

        if( numDisplayPanels <= 0 )
        {
            continue;
        }

        for( int panelIndex = 0; panelIndex < numDisplayPanels; panelIndex++ )
        {
            AgentPanelFeature* pFeature = new AgentPanelFeature( this );

            if( !pImpl->GetDcaDisplayPanelInfo( panelIndex, pFeature ) )
            {
                delete pFeature;
                pFeature = NULL;
                continue;
            }

            pFeature->DataSz = 0;
            pFeature->pAgentClass = this;
            pFeature->hImpl = ( unsigned long )m_ImplList[i];
            Assert( strlen( pFeature->source ) > 0 );
            Assert( pFeature->imgWidth > 0 );
            Assert( pFeature->imgHeight > 0 );
            Assert( pFeature->ImageSz > 0 );
            bool hasbeenRegistered = false;
            int DsPanelIndex = DS_Panel_Add( ( unsigned long )pImpl, pFeature->source, pFeature->ImageSz, hasbeenRegistered, pFeature );
            assert( DsPanelIndex >= 0 );
            pFeature->PanelIndex = DsPanelIndex;
            m_Panels.push_back( pFeature );
        }
    }
}

// ---------------------------------------------------------------------
void DataCollectAgent::CreateDisplayViews( bool IsDisplayOwner )
{
    T_WriterLock lock( g__OCAgent_Lock );
    Assert( m_Panels.size() > 0 );

    if( m_Panels.size() == 0 )
    {
        return;
    }

    if( m_Views.size() > 0 )
    {
        return;
    }

    int numPanelsPerImpl = m_ImplList[0]->GetNumDcaDisplayPanels();
    int numImpl = m_ImplList.size();
    AgentViewFeature* pViewL = NULL;
    char viewLName[40];
    bool viewLTransposed = false;
    memset( viewLName, 0, sizeof( viewLName ) );

    if( numImpl > 1 && numPanelsPerImpl < 5 )
    {
        pViewL = new AgentViewFeature();
        sprintf_s( viewLName, sizeof( viewLName ), "%s[", m_PluginName );

        if( numImpl > numPanelsPerImpl )
        {
            viewLTransposed = true;
        }

        for( int i = 0; i < numImpl * numPanelsPerImpl; i++ )
        {
            pViewL->m_PanelList.push_back( NULL );
        }
    }

    for( unsigned int implIndex = 0; implIndex < m_ImplList.size(); implIndex++ )
    {
        COnlineCalibrationImpl* pImpl = ( COnlineCalibrationImpl* )m_ImplList[implIndex];

        if( pImpl == NULL )
        {
            continue;
        }

        if( pViewL != NULL )
        {
            if( implIndex > 0 )
            {
                strcat( viewLName, "+" );
            }

            strcat( viewLName, pImpl->getShortName() );
        }

        int numPanels = pImpl->GetNumDcaDisplayPanels();

        if( numPanels == 0 )
        {
            Assert( strlen( "numPanels == 0" ) == 0 );
            continue;
        }

        AgentViewFeature* pView = new AgentViewFeature();

        for( int PanelIndex = 0; PanelIndex < numPanels; PanelIndex++ )
        {
            char* pPanelName = ( char* )pImpl->GetDcaDisplayPanelName( PanelIndex );

            if( pPanelName == NULL || strlen( pPanelName ) == 0 )
            {
                break;
            }

            AgentPanelFeature* pPanel = FindPanelFromList( pPanelName );

            if( pPanel )
            {
                pView->m_PanelList.push_back( pPanel );

                if( pViewL )
                {
                    if( !viewLTransposed )
                    {
                        pViewL->m_PanelList[implIndex * numPanels + PanelIndex] = pPanel;
                    }
                    else
                    {
                        pViewL->m_PanelList[PanelIndex * m_ImplList.size() + implIndex] = pPanel;
                    }
                }
            }
        }

        pView->DebugTextEnabled = true;
        pView->NumRows = 1;

        if( pView->GetPanelCount() > 2 )
        {
            pView->NumRows = 2;
        }

        sprintf_s( pView->m_HotKeyStr, sizeof( pView->m_HotKeyStr ), "CTRL,%c", 'a' + m_Views.size() );
        strcpy_s( pView->DisplayName, sizeof( pView->DisplayName ), pImpl->m_DcaViewName );
        m_Views.push_back( pView );
    }

    if( pViewL )
    {
        pViewL->NumRows = numImpl;

        if( viewLTransposed )
        {
            pViewL->NumRows = numPanelsPerImpl;

            if( numPanelsPerImpl == 1 && pViewL->GetPanelCount() > 3 )
            {
                pViewL->NumRows = 2;
            }
        }

        sprintf_s( pViewL->m_HotKeyStr, sizeof( pViewL->m_HotKeyStr ), "CTRL,%c", 'a' + m_Views.size() );
        strcat( viewLName, "]" );
        strcpy_s( pViewL->DisplayName, sizeof( pViewL->DisplayName ), viewLName );
        m_Views.insert( m_Views.begin(), pViewL );
    }

    if( IsDisplayOwner )
    {
        CreatePluginOwnedViews();
    }
}

bool DataCollectAgent::Process( void* pImpl )
{
    class COnlineCalibrationImpl* pOcImpl = ( COnlineCalibrationImpl* )pImpl;
    T_WriterLock lock( g__OCAgent_Lock );

    if( pImpl == NULL )
    {
        return false;
    }

    unsigned int GraphSz = sizeof( m_GraphPool ) / sizeof( m_GraphPool[0] );
    unsigned int GraphNum = 0;

    for( unsigned int i = 0; i < m_Panels.size(); i++ )
    {
        if( m_Panels[i] == NULL )
        {
            continue;
        }

        if( !DS_Panel_GetStatus( m_Panels[i]->PanelIndex ) )
        {
            continue;
        }

        AgentPanelFeature* pFeature = m_Panels[i];

        if( stricmp( pFeature->source, pOcImpl->m_CameraImageName ) == 0 )
        {
            GraphNum = 0;
            pOcImpl->GetDcaImageAndGraphs( pFeature, GraphNum, m_GraphPool, GraphSz );
            ConvertGraphsList2Xml( pFeature, m_GraphPool, GraphNum );
        }
    }

    return true;
}