// ----------------------------------------------------------------------------
// --- written by Joseph Braun [23-MAR-2021]
// --- Copyright (c) Magna Electronics - Brampton 2021
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "OCImpl.h"
#include <time.h>
// ----------------------------------------------------------------------------

bool COnlineCalibrationImpl::GetDcaDisplayPanelInfo( int index, void* pArg )
{
    if( pArg == NULL )
    {
        return false;
    }

    AgentPanelFeature* pPanel = static_cast <AgentPanelFeature*>( pArg );

    if( index != 0 || strlen( m_CameraImageName ) == 0 )
    {
        return false;
    }

    strcpy_s( pPanel->source, sizeof( pPanel->source ), m_CameraImageName );
    pPanel->imgWidth = m_pVidImageProfileMap[m_CamID]->Width;
    pPanel->imgHeight = m_pVidImageProfileMap[m_CamID]->Height;
    pPanel->ImageSz = pPanel->imgWidth * pPanel->imgHeight;

    switch( g__pAppCtrl->m_VIDIfo.m_ImgType )
    {
        case IMG_RGB:
            pPanel->imgType = ImageType_YUV;
            pPanel->ImageSz = pPanel->imgWidth * pPanel->imgHeight * 3;
            break;

        case IMG_GRAYSCALE:
            pPanel->imgType = ImageType_GrayScale;
            break;

        default:
            Assert( strlen( "Image type not supported" ) == 0 );
            return false;
            break;
    }

    return true;
}

const char* COnlineCalibrationImpl::GetDcaDisplayPanelName( int index )
{
    if( index == 0 && strlen( m_CameraImageName ) > 0 )
    {
        return m_CameraImageName;
    }

    return NULL;
}

int COnlineCalibrationImpl::GetNumDcaDisplayPanels()
{
    return 1;
}

const char* COnlineCalibrationImpl::getShortName()
{
    if( m_pVidImageProfileMap[m_CamID] )
    {
        return m_pVidImageProfileMap[m_CamID]->ShortName;
    }

    return "Unkown";
}

bool COnlineCalibrationImpl::GetDcaImageAndGraphs( void* pArg, unsigned int& numGraphs, void* pGraphPool, const unsigned int maxGraphNum )
{
    AgentPanelFeature* pFeature = static_cast <AgentPanelFeature*>( pArg );

    if( pFeature == NULL )
    {
        return false;
    }

    if( stricmp( pFeature->source, m_CameraImageName ) != 0 )
    {
        return false;
    }

    if( !g__pDataCollectAgent->OverlayIDEnabled( Overlay_CamView_OC ) )
    {
        return false;
    }

    std::string StateStr;

    switch( m_AlgoState )
    {
        case tscApi::e_TscStateUnInit:
            StateStr = std::string( "OC State : UnInit" );
            break;

        case tscApi::e_TscStateInitOk:
            StateStr = std::string( "OC State : Init Ok" );
            break;

        case tscApi::e_TscStateFeatureCollection:
            StateStr = std::string( "OC State : Feature Collection" );
            break;

        case tscApi::e_TscStateFeatureCollectionCompleted:
            StateStr = std::string( "OC State : Features Collected" );
            break;


        case tscApi::e_TscStateCalibration:
            StateStr = std::string( "OC State : Calibrating" );
            break;

        case tscApi::e_TscStateCalibrationCompleted:
            StateStr = std::string( "OC State : Calibrated" );
            break;


        case tscApi::e_TscStateTerminated:
            StateStr = std::string( "OC State : Terminated" );
            break;

        case tscApi::e_TscStatePaused:
            StateStr = std::string( "OC State : Calibration Paused" );
            break;

        case tscApi::e_TscStateUnknown:
            StateStr = std::string( "OC State : Unknown" );
            break;

        case tscApi::e_TscStateEnd:
            StateStr = std::string( "OC State : End" );
            break;
    };

    {
        int stateLength = StateStr.length();
        int x_offset = 0;

        if( stateLength % 2 == 0 )
        {
            x_offset = stateLength / 2;
        }
        else
        {
            x_offset = ( stateLength + 1 ) / 2;
        }

        GraphStruct* pPool = static_cast<GraphStruct*>( pGraphPool );
        int lineNumber = 0;
        COLORREF textColour = RGB( 0x10, 0xfe, 0x9f );
        unsigned int textlen = 0;
        char txtmsg[100];
        strcpy( txtmsg, StateStr.c_str() );
        char* pTextStr = txtmsg;
        unsigned int y_gap = 12;
        GraphStruct* pGraph = &pPool[numGraphs];
        pGraph->data.text.x = ( uint32_t )( 640 / 2 - 12 * ( x_offset + 1 ) );
        pGraph->data.text.y = ( uint32_t )( 400 / 4 ) + y_gap * lineNumber;
        pGraph->type = GraphType_Text;
        pGraph->thickness = 1;
        pGraph->height = 35;
        pGraph->colour = textColour;
        strcpy( pGraph->data.text.content, pTextStr );
        numGraphs++;
        tscApi::enuCameraID tsc_CamID = tscApi::e_TscFrontCam;

        switch( m_CamID )
        {
            case Front:
                tsc_CamID = tscApi::e_TscFrontCam;
                break;

            case Rear:
                tsc_CamID = tscApi::e_TscRearCam;
                break;

            case Left:
                tsc_CamID = tscApi::e_TscLeftCam;
                break;

            case Right:
                tsc_CamID = tscApi::e_TscRightCam;
                break;
        }

        uint8_t num_overlays;
        const tscApi::DebugOverlay_s* TSC_overlays = TSC_GetOverlays( tsc_CamID, num_overlays );

        //const tscApi::DebugOverlay_s* TSC_GetOverlays(tscApi::enuCameraID i_CameraID_t, uint8_t& o_Num_ru8)
        for( int i = 0; i < num_overlays; i++ )
        {
            tscApi::DebugOverlay_s curr_overlays = TSC_overlays[i];
            GraphStruct* pGraphCircle = &pPool[numGraphs];
            pGraphCircle->type = GraphType_CircleLine;
            pGraphCircle->colour = RGB_GREEN;
            pGraphCircle->data.circle.centreX = ( uint32_t )curr_overlays.startPt_x;
            pGraphCircle->data.circle.centreY = ( uint32_t )curr_overlays.startPt_y;
            pGraphCircle->data.circle.radius = ( uint32_t )( 10 );
            pGraphCircle->thickness = 4;
            numGraphs++;
            pGraphCircle = &pPool[numGraphs];
            pGraphCircle->type = GraphType_CircleLine;
            pGraphCircle->colour = RGB_RED;
            pGraphCircle->data.circle.centreX = ( uint32_t )curr_overlays.endPt_x;
            pGraphCircle->data.circle.centreY = ( uint32_t )curr_overlays.endPt_y;
            pGraphCircle->data.circle.radius = ( uint32_t )( 10 );
            pGraphCircle->thickness = 4;
            numGraphs++;
//            GraphStruct* pGraph = &pPool[numGraphs];
            pGraph->type = GraphType_Contour;
            pGraph->colour = RGB_PINK;
            pGraph->thickness = 1;
            pGraph->data.points.numPoints = 2;
            pGraph->data.points.x[0] = ( int32_t )curr_overlays.startPt_x;
            pGraph->data.points.y[0] = ( int32_t )curr_overlays.startPt_y;
            pGraph->data.points.x[1] = ( int32_t )curr_overlays.endPt_x;
            pGraph->data.points.y[1] = ( int32_t )curr_overlays.endPt_y;
            numGraphs++;
        }
    }

    return ( numGraphs > 0 );
}

// ---------------------------------- EOF -------------------------------------