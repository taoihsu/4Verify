//--------------------------------------------------------------------------
/// @file JobOC.cpp
/// @brief Contains
///
///
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Muzammil Rasool (muzammil.rasool@magna.com)
///
//  --------------------------------------------------------------------------

#include "stdafx.h"

#include "mecl/core/meclassert.h"
#include "mecl/core/mecllogging.h"
#include "meclcfg.h"
#include "JobOC.h"

namespace oc
{
#ifdef OC_GRAB_ALGO_VIEW_IMAGES
static const uint32_t V_FramesToRead_u32 = 0;
static const uint32_t V_StartFrame_u32 = 300;
static const uint32_t V_NumCameras_u32 = 1;
uint8_t V_AlgoViewBuff_au8[V_FramesToRead_u32][640*400*V_NumCameras_u32];
uint8_t *V_Addresses_pau8[V_FramesToRead_u32];
uint32_t V_FrameIdB4_au32[V_FramesToRead_u32];
uint32_t V_FrameIdAfter_au32[V_FramesToRead_u32];
#endif

    //#define OC_HARDCODED_INPUT_IMAGE
#ifdef OC_HARDCODED_INPUT_IMAGE
    uint8_t buffCount = 0;
    const uint8_t OcSyntheticImageFront_au8 [2][400U * 640U] =
    {
        //PRQA S 5026 1 //The include file contains test image data for debugging.
        //#include "front_test_img.hex"
#include "test-image.hex"
};
#endif

#ifdef USE_SVSCM
const float64_t V_PreRoll_af32[tscApi::e_TscNumCam] = {-90.0F, 0.0F, 90.0F, 180.0F};
#endif


    JobOC::JobOC( IDataProviderOC& i_dataProvider_ro ) : JobBaseAlgo<IDataProviderOC>( i_dataProvider_ro ),
        //Logging_Enabled( dataProvider_ro.getLoggingEnabled() ),
        state_e( e_UnInit ),
        ocFrameCounter_u32( 0U ),
        isAlgorithmStarted_b( false ),
		isAlgorithmFinished_b(false),
		
        isNewCommand_b( false ),
        algoCommand_e( ocdata::e_Unknown ),
        c_OcExtrinsics_ps( NULL ),
        algoViewBufferToRead_e( ocdata::e_OcAlgoViewBuffer0 ),
        algoViewConfiguredCounter_u32( 0U ),
        prevValidFeatureCount_u32( 0U ),
        abortFrameCounter_u32( 0U ),
#ifdef OC_GRAB_ALGO_VIEW_IMAGES
        testCounter_u32( 0U ),
#endif
        m_aFrontCameraInfo( dataProvider_ro.getFrontDoCameraParam() ),
        m_aLeftCameraInfo( dataProvider_ro.getLeftDoCameraParam() ),
        m_aRearCameraInfo( dataProvider_ro.getRearDoCameraParam() ),
        m_aRightCameraInfo( dataProvider_ro.getRightDoCameraParam() ),
        mcuCamIdToOcAlgo_e( ocdata::e_Front )
{
  for(uint32_t v_Index_u32 = 0U; v_Index_u32 < static_cast<uint32_t>(tscApi::e_TscNumCam);v_Index_u32++)
  {
    tscIntermediateData_as[v_Index_u32].data_pu8 = NULL;
    tscIntermediateData_as[v_Index_u32].len_u32 = 0U;
    tscCtrlInfo_o.PutM_PSvdDtInf( static_cast <uint8_t> (v_Index_u32), &tscIntermediateData_as[v_Index_u32] );
  }
  memset(&tscStartConfiguration_as, 0, sizeof(tscStartConfiguration_as));
  memset(&tscErrorCode_ae, 0, sizeof(tscErrorCode_ae));
  memset(&tscAlgoState_ae, 0, sizeof(tscAlgoState_ae));
  memset(&tscCalibrationResults_as, 0, sizeof(tscCalibrationResults_as));

  tscCtrlInfo_o.PutM_Spd(0.0F);
  tscCtrlInfo_o.PutM_WhlAngl(0.0F);
  tscCtrlInfo_o.PutM_HtchAngl(0.0F);
  tscCtrlInfo_o.PutM_GrDrctn(tscApi::TSCCtrlInfo::e_GearForward);
  tscCtrlInfo_o.PutM_CmrFrntOpn(true);
  tscCtrlInfo_o.PutM_FrmNmbr(0U);
  tscCtrlInfo_o.PutM_VdWdth(c_ImageWidth_u32);
  tscCtrlInfo_o.PutM_VdHght(c_ImageHeight_u32);
  tscCtrlInfo_o.PutM_Cmrs(tscApi::e_TscFrontCam, NULL);
  tscCtrlInfo_o.PutM_Cmrs(tscApi::e_TscLeftCam, NULL);
  tscCtrlInfo_o.PutM_Cmrs(tscApi::e_TscRearCam, NULL);
  tscCtrlInfo_o.PutM_Cmrs(tscApi::e_TscRightCam, NULL);

  OC_DEBUG_PRINTF( "*JobOcConstructor* %d, %d, %u, %d, %u\n"
      ,tscCtrlInfo_o.getMCmrFrntOpn_b()
      ,tscCtrlInfo_o.getMFrmNmbr_u32()
      ,tscCtrlInfo_o.getMVdHght_u16()
      ,tscCtrlInfo_o.getMGrDrctn_s32()
      ,tscCtrlInfo_o.getMVdWdth_u16()
  );
}

JobOC::~JobOC()
{
}

    void JobOC::reset_v()
    {
    }

void JobOC::init_v()
{
#ifdef DUMP_EMB_IP
        FILE* fp;
        fp = fopen( "CAN.txt", "w" );
        fclose( fp );
#endif
  bool_t v_IsAlgoInitSuccess_b = false;
  OC_DEBUG_PRINTF( ( "*init_v* Before init, state: %d\n", TSC_GetState( tscApi::e_TscFrontCam ) ) );
#ifdef OC_HARDCODED_INPUT_IMAGE
  tscCtrlInfo_o.PutM_Spd( 0.0F );
  tscCtrlInfo_o.PutM_WhlAngl( 0.0F );
  tscCtrlInfo_0.PutM_HtchAngl_f32(0.0F);
  tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscFrontCam, &OcSyntheticImageFront_au8[buffCount][0] );
  tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscLeftCam, &OcSyntheticImageFront_au8[buffCount][0] );
  tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscRearCam, &OcSyntheticImageFront_au8[buffCount][0] );
  tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscRightCam, &OcSyntheticImageFront_au8[buffCount][0] );
#else
  tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscFrontCam, get640x400AlgoView_pu8( ocdata::e_Front ) );
  tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscLeftCam, get640x400AlgoView_pu8( ocdata::e_Left ) );
  tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscRearCam, get640x400AlgoView_pu8( ocdata::e_Rear ) );
  tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscRightCam, get640x400AlgoView_pu8( ocdata::e_Right ) );
#endif
        tscCtrlInfo_o.tscTargetCamera = dataProvider_ro.getCameraID_e();
  v_IsAlgoInitSuccess_b = TSC_Init(&tscCtrlInfo_o);
  if(false == v_IsAlgoInitSuccess_b)
  {
    dataProvider_ro.getOutData()->ocAlgoState_e = ocdata::e_OcStateError;
    dataProvider_ro.getOutData()->ocErrorCode_e = ocdata::e_OcErrCodeInitFail;
    //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
  }
  else
  {
    dataProvider_ro.getOutData()->ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>(TSC_GetState(tscApi::e_TscFrontCam));
    dataProvider_ro.getOutData()->ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>(TSC_GetError(tscApi::e_TscFrontCam));
    //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(tscApi::e_TscFrontCam));
    state_e = e_OcInit;
  }

  // Not sure if this is converted correctly
  //oc::LogCtxOC::registerCtxId_v();
  //const uint32_t c_FrameId_u32 = dataInProvider_ro.getDataSystem()->psFrameId_u32;
  const uint32_t c_FrameId_u32 = dataProvider_ro.getFrameNum_u32();
  //dataOutProvider_ro.getOutData()->updatedAt_u32 = c_FrameId_u32;
  dataProvider_ro.getOutData()->updatedAt_u32 = c_FrameId_u32;
  OC_DEBUG_PRINTF( ( "*init_v* v_IsAlgoInitSuccess_b %d, state %d, error %d\n", v_IsAlgoInitSuccess_b, dataOutProvider_ro.getOutData()->ocAlgoState_e, dataOutProvider_ro.getOutData()->ocErrorCode_e ) );
}

bool_t JobOC::hasNext_b() const
{
  bool_t v_Ret_t = false;
  if (e_OcReadyToExecute == state_e)
  {
    v_Ret_t = true;
  }
  return v_Ret_t;
}

void JobOC::start_v()
{
  // Don't bother checking AlgoType - we are always running OC
  // Check if algorithm has started - if so, we will skip this function
  // There are no "commands" coming in, so we will only start algorithm once and let it run
  if( !isAlgorithmStarted_b && !isAlgorithmFinished_b )
  {
#ifdef OC_HARDCODED_INPUT_IMAGE
    mcuCamIdToOcAlgo_e = ocdata::e_Front;
#else
    //uint8_t v_McuCamId_u8 = dataInProvider_ro.getDataMcu()->algoControl_s.MCUCameraID_u8;
    //prjcontainer::e_Rear //This MCU Cam ID is used for OC/TSC algorithm
#endif
    tscApi::enuCameraID v_OcAlgoCamId_e = dataProvider_ro.getCameraID_e();

    switch( v_OcAlgoCamId_e )
    {
      case tscApi::e_TscFrontCam:
        default:
        mcuCamIdToOcAlgo_e = ocdata::e_Front;
        break;

      case tscApi::e_TscRearCam:
        mcuCamIdToOcAlgo_e = ocdata::e_Rear;
        break;

      case tscApi::e_TscLeftCam:
        mcuCamIdToOcAlgo_e = ocdata::e_Left;
        break;

      case tscApi::e_TscRightCam:
        mcuCamIdToOcAlgo_e = ocdata::e_Right;
        break;
    }

            isNewCommand_b = true;
            // For now, hardcode algoCommand to be Start
            // Since this value is supposed to come from MCU,
            // there should be some analog to this in AppCtrl
            algoCommand_e = ocdata::e_Start;
            uint32_t v_FrameId_u32 = dataProvider_ro.getFrameNum_u32();
            dataProvider_ro.getOutData()->lastRequestedAt_u32 = v_FrameId_u32;

      switch(algoCommand_e)
      {
                case ocdata::e_Start:
        {
          //memset( &dataProvider_ro.getOutData()->ocDataToMcu_s, 0, sizeof( ocdata::OcDataToMcu_s ) );

          //If algorithm is already running, it has to be stopped. So that new OC run can be started.
          if(true == isAlgorithmStarted_b)
          {
            bool_t v_IsStopSuccess_b = false;
            v_IsStopSuccess_b = TSC_Stop();
            if(false == v_IsStopSuccess_b)
            {
              dataProvider_ro.getOutData()->ocAlgoState_e = ocdata::e_OcStateError;
              dataProvider_ro.getOutData()->ocErrorCode_e = ocdata::e_OcErrCodeUnexpectedRequest;
              //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
            }
            else
            {
              dataProvider_ro.getOutData()->ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
              dataProvider_ro.getOutData()->ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
              //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
            }

            OC_DEBUG_PRINTF( ( "*start_v* OC running already, new start command comes. Current run stopped. \n" ) );
          }
          else
          {
            dataProvider_ro.getOutData()->ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
            dataProvider_ro.getOutData()->ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
            //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>( TSC_GetCalibManeuver( v_OcAlgoCamId_e ) );

          }

          dataProvider_ro.getOutData()->mcuCameraId_u8 = mcuCamIdToOcAlgo_e;
          //Initialize tscStartConfiguration_as
          setOcKinematicModelConfiguration_v();
          setOcCamModelConfiguration_v();
          setOcFeatureCollectionConfiguration_v();

#ifdef OC_LOG_PRINTF_ON
          bool_t v_IsAlgoStartSuccess_b = false;
#endif
                    if( ocdata::e_OcStateUninit != dataProvider_ro.getOutData()->ocAlgoState_e )
          {
#ifndef OC_LOG_PRINTF_ON
          bool_t v_IsAlgoStartSuccess_b = false;
#endif
            //TODO Intermediate/saved data
            v_IsAlgoStartSuccess_b = TSC_Start(v_OcAlgoCamId_e, &tscStartConfiguration_as[v_OcAlgoCamId_e], NULL);
#ifdef OC_GRAB_ALGO_VIEW_IMAGES
            testCounter_u32 = 0;
#endif
            if(false == v_IsAlgoStartSuccess_b)
            {
              dataProvider_ro.getOutData()->ocAlgoState_e = ocdata::e_OcStateError;
              dataProvider_ro.getOutData()->ocErrorCode_e = ocdata::e_OcErrCodeStartFail;
              //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
            }
            else
            {
              dataProvider_ro.getOutData()->ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
              dataProvider_ro.getOutData()->ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
              //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
            }
              v_FrameId_u32 = dataProvider_ro.getFrameNum_u32();
              dataProvider_ro.getOutData()->updatedAt_u32 = v_FrameId_u32;
          }
          OC_DEBUG_PRINTF( ( "*start_v* v_IsAlgoStartSuccess_b %d, state %d, error %d\n", v_IsAlgoStartSuccess_b, dataOutProvider_ro.getOutData()->ocAlgoState_e, dataOutProvider_ro.getOutData()->ocErrorCode_e ) );
          ocFrameCounter_u32 = 0U;
          isAlgorithmStarted_b = true;
          OC_DEBUG_PRINTF( ( "JobOC::execute_v = %d, started\n", static_cast<printInt_t>( v_Time_o.getTimeNs_u64() / 1000000 ) ) );

          break;
        }
        case ocdata::e_Stop:
        {
          bool_t v_IsStopSuccess_b = false;
          OC_DEBUG_PRINTF( ( "JobOC::execute_v = %d, stopped\n", static_cast<printInt_t>( v_Time_o.getTimeNs_u64() / 1000000 ) ) );
          v_IsStopSuccess_b = TSC_Stop();
          if(false == v_IsStopSuccess_b)
          {
            dataProvider_ro.getOutData()->ocAlgoState_e = ocdata::e_OcStateError;
            dataProvider_ro.getOutData()->ocErrorCode_e = ocdata::e_OcErrCodeCalibrationError;
            //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
          }
          else
          {
            dataProvider_ro.getOutData()->ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
            dataProvider_ro.getOutData()->ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
            //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
          }

          dataProvider_ro.getOutData()->validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
          dataProvider_ro.getOutData()->ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
          dataProvider_ro.getOutData()->invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );
          OC_DEBUG_PRINTF( ( "*StopCommand* algoCam: %d, Machinestate: %d, stateToMcu: %d, errorToMcu: %d, driveMan: %d, validToMcu: %lu, ignoredToMcu: %lu, invalidToMcu: %lu, skiped: %d, procsd: %d, state: %d, error: %d\n"
              ,v_OcAlgoCamId_e
              ,tscAlgoState_ae[v_OcAlgoCamId_e]
              , dataOutProvider_ro.getOutData()->ocAlgoState_e
              , dataOutProvider_ro.getOutData()->ocErrorCode_e
              //,dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e
              , dataOutProvider_ro.getOutData()->validFeaturesCount_u32
              , dataOutProvider_ro.getOutData()->ignoredFeaturesCount_u32
              , dataOutProvider_ro.getOutData()->invalidFeaturesCount_u32
              ,TSC_GetSkippedFramesCount(v_OcAlgoCamId_e)
              ,TSC_GetProcessedFramesCount(v_OcAlgoCamId_e)
              ,TSC_GetState(v_OcAlgoCamId_e)
              ,TSC_GetError(v_OcAlgoCamId_e)
          ) );

          isAlgorithmStarted_b = false;
          break;
        }

        case ocdata::e_Pause:
        {
          OC_DEBUG_PRINTF( ( "JobOC::execute_v = %d, paused\n", static_cast<printInt_t>( v_Time_o.getTimeNs_u64() / 1000000 ) ) );
          bool_t v_IsPauseSuccess_b = false;
          v_IsPauseSuccess_b = TSC_Pause();

          if( false == v_IsPauseSuccess_b )
          {
            dataProvider_ro.getOutData()->ocAlgoState_e = ocdata::e_OcStateError;
            dataProvider_ro.getOutData()->ocErrorCode_e = ocdata::e_OcErrCodeCalibrationError;
            //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
          }
          else
          {
            dataProvider_ro.getOutData()->ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
            dataProvider_ro.getOutData()->ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
            //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
          }

          dataProvider_ro.getOutData()->validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
          dataProvider_ro.getOutData()->ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
          dataProvider_ro.getOutData()->invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );

          isAlgorithmStarted_b = false;
          break;
        }
        case ocdata::e_Status:
        {
          break;
        }
        case ocdata::e_Sync:
        {
          OC_DEBUG_PRINTF( ( "JobOC::execute_v = %d, sync\n", static_cast<printInt_t>( v_Time_o.getTimeNs_u64() / 1000000 ) ) );

          break;
        }
        case ocdata::e_GetResult:
        {
          break;
        }
        case ocdata::e_Resume:
        {
          OC_DEBUG_PRINTF( ( "JobOC::execute_v = %d, resumed\n", static_cast<printInt_t>( v_Time_o.getTimeNs_u64() / 1000000 ) ) );
          bool_t v_IsResumeSuccess_b = false;
          v_IsResumeSuccess_b = TSC_Resume();
          if(false == v_IsResumeSuccess_b)
          {
            dataProvider_ro.getOutData()->ocAlgoState_e = ocdata::e_OcStateError;
            dataProvider_ro.getOutData()->ocErrorCode_e = ocdata::e_OcErrCodeUnexpectedRequest;
            //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
          }
          else
          {
            dataProvider_ro.getOutData()->ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
            dataProvider_ro.getOutData()->ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
            //dataOutProvider_ro.getOutData()->ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
          }

          dataProvider_ro.getOutData()->validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
          dataProvider_ro.getOutData()->ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
          dataProvider_ro.getOutData()->invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );

          isAlgorithmStarted_b = true;
          break;
        }
        case ocdata::e_Debug:
        {
          OC_DEBUG_PRINTF( ( "start_v = %lu, debug, val: %d\n", v_Time_o.getTimeMs_u32(), dataInProvider_ro.getDataMcu()->algoControl_s.algMData_s.algoDebugView_e ) );
          break;
        }
        default:
        {
          break;
        }
      }
    }

  state_e = e_OcReadyToExecute;
}

    // -----------------------------------------------------------------------------------------------------
    // For now, I'm not going to worry about Debug view
    // This would need major refactoring to work on Windows

    //void JobOC::deactivateDebugView_v() const
    //{
    //    ocdata::AdapterAlgoDebugData::ReturnValue_e v_RV_e = prjcontainer::AdapterAlgoDebugData::e_ReturnValueERROR;
    //    // initialize the adapter to debug data storage
    //    // todo move to initialization method
    //    prjcontainer::AdapterAlgoDebugData v_AdapterDebug_o( &dataOutProvider_ro.getOutData()->debugLayer_s );
    //    // deactivate debug view
    //    v_RV_e = v_AdapterDebug_o.deactivate_e();
    //    AssertMsg( v_RV_e == prjcontainer::AdapterAlgoDebugData::e_ReturnValueSUCCESS, "failed to deactivate CCF debug view" );
    //}

    // fixme this is only the test version to demonstrate access to debug view
    //void JobOC::createDebugView_v( const uint8_t* i_AlgoView_pu8, const uint8_t i_OcAlgoCamId_u8 )
    //{
    //    // do not update debug view for every frame to reduce memory bandwidth
    //    const uint32_t c_UpdateInterval_u32 = 5U;
    //    static uint32_t v_Cnt_u32 = c_UpdateInterval_u32;
    //    --v_Cnt_u32;

    //    if( v_Cnt_u32 == 0U )
    //    {
    //        v_Cnt_u32 = c_UpdateInterval_u32; // reset counter to update interval
    //        prjcontainer::AdapterAlgoDebugData::ReturnValue_e v_RV_e = prjcontainer::AdapterAlgoDebugData::e_ReturnValueERROR;
    //        // setup algo debug overlay description (interpreted by overlay render engine)
    //        shmdata::AlgoDebugImageDscr_s v_ImageDescr_s;
    //        v_ImageDescr_s.imgWidth_u32  = c_ImageWidth_u32;  // src width of debug view
    //        v_ImageDescr_s.imgHeight_u32 = c_ImageHeight_u32;  // src height of debug view
    //        v_ImageDescr_s.dstPosX_u32   = 0U;   // vertical position of debug view
    //        v_ImageDescr_s.dstPosY_u32   = 0U;   // vertical position of debug view
    //        v_ImageDescr_s.dstWidth_u32  = c_ImageWidth_u32;  // dest width of debug view
    //        v_ImageDescr_s.dstHeight_u32 = c_ImageHeight_u32;  // dest height of debug view
    //        // initialize the adapter to debug data storage
    //        // todo move to initialization methode
    //        prjcontainer::AdapterAlgoDebugData v_AdapterDebug_o( &dataOutProvider_ro.getOutData()->debugLayer_s );
    //        v_AdapterDebug_o.lock_v();
    //        // clear inventory
    //        v_RV_e = v_AdapterDebug_o.clear_e();
    //        Assert( v_RV_e == prjcontainer::AdapterAlgoDebugData::e_ReturnValueSUCCESS );
    //        // add algo view
    //        v_RV_e = v_AdapterDebug_o.addImage_e( v_ImageDescr_s, i_AlgoView_pu8 );
    //        AssertMsg( v_RV_e == prjcontainer::AdapterAlgoDebugData::e_ReturnValueSUCCESS, "failed to add debug image" );
    //        // draw a rect
    //        shmdata::AlgoDebugRectDscr_s v_RectDescr_s;
    //        v_RectDescr_s.posTopLeftX_u32 = tscStartConfiguration_as[i_OcAlgoCamId_u8].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].x_s32 - 2;
    //        v_RectDescr_s.posTopLeftY_u32 = tscStartConfiguration_as[i_OcAlgoCamId_u8].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].y_s32 - 2;
    //        v_RectDescr_s.width_u32 = tscStartConfiguration_as[i_OcAlgoCamId_u8].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].width_s32 + 4;
    //        v_RectDescr_s.height_u32 = tscStartConfiguration_as[i_OcAlgoCamId_u8].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].height_s32 + 4;
    //        v_RectDescr_s.colorStrokeRGBA_u32 = 0xFF0000FFU;
    //        v_RectDescr_s.colorFillRGBA_u32   = 0x00000000U;
    //        v_RectDescr_s.strokeWidth_u32 = 1U;
    //        v_RV_e = v_AdapterDebug_o.addRect_e( v_RectDescr_s );
    //        AssertMsg( v_RV_e == prjcontainer::AdapterAlgoDebugData::e_ReturnValueSUCCESS, "failed to add debug rect 1" );
    //        // draw a second rect
    //        v_RectDescr_s.posTopLeftX_u32 = tscStartConfiguration_as[i_OcAlgoCamId_u8].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].x_s32 - 2;
    //        v_RectDescr_s.posTopLeftY_u32 = tscStartConfiguration_as[i_OcAlgoCamId_u8].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].y_s32 - 2;
    //        v_RectDescr_s.width_u32 = tscStartConfiguration_as[i_OcAlgoCamId_u8].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].width_s32 + 4;
    //        v_RectDescr_s.height_u32 = tscStartConfiguration_as[i_OcAlgoCamId_u8].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].height_s32 + 4;
    //        v_RectDescr_s.colorStrokeRGBA_u32 = 0x00FF00FFU;
    //        v_RectDescr_s.colorFillRGBA_u32   = 0x00000000U;
    //        v_RectDescr_s.strokeWidth_u32 = 1U;
    //        v_RV_e = v_AdapterDebug_o.addRect_e( v_RectDescr_s );
    //        AssertMsg( v_RV_e == prjcontainer::AdapterAlgoDebugData::e_ReturnValueSUCCESS, "failed to add debug rect 2" );
    //        // activate inventory for visualization of debug overlays
    //        v_AdapterDebug_o.activate_e( dataInProvider_ro.getDataSystem()->psFrameId_u32 );
    //        v_AdapterDebug_o.unlock_v();
    //    }
    //}
    // -----------------------------------------------------------------------------------------------------


void JobOC::execute_v()
{
  //remove( "slopeThreshold.txt" );
  //remove( "pixelThreshold.txt" );
  ocdata::OcData_s& v_OcData_rs = *dataProvider_ro.getOutData();

  tscApi::enuCameraID v_OcAlgoCamId_e = getOcAlgoCamId_e(mcuCamIdToOcAlgo_e);

  tscAlgoState_ae[v_OcAlgoCamId_e] = TSC_GetState(v_OcAlgoCamId_e);
  tscErrorCode_ae[v_OcAlgoCamId_e] = TSC_GetError(v_OcAlgoCamId_e);
#ifdef DUMP_EMB_IP

        if( dataProvider_ro.getFrameNum_u32() < 2000 )
        {
            FILE* fp;
            fp = fopen( "CAN.txt", "a" );
            fprintf( fp, "{ %f, %f },\n", dataProvider_ro.getSpeed_f32(), dataProvider_ro.getWheelAngle_f32() );
            fclose( fp );
        }
#endif
  if(true == isAlgorithmStarted_b)
  {
    ocFrameCounter_u32++;

    if(tscApi::e_TscErrorNoError == tscErrorCode_ae[v_OcAlgoCamId_e])
    {

      switch(tscAlgoState_ae[v_OcAlgoCamId_e])
      {

        case tscApi::e_TscStateUnInit:
        {
          OC_DEBUG_PRINTF( ( "*Statemachine_uninit* Shouldn't come here\n" ) );
          bool_t v_IsAlgoInitSuccess_b = false;
#ifdef OC_HARDCODED_INPUT_IMAGE
          tscCtrlInfo_o.PutM_Spd( 0.0F );
          tscCtrlInfo_o.PutM_WhlAngl( 0.0F );
          tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscFrontCam, &OcSyntheticImageFront_au8[buffCount][0] );
          tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscLeftCam, &OcSyntheticImageFront_au8[buffCount][0] );
          tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscRearCam, &OcSyntheticImageFront_au8[buffCount][0] );
          tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscRightCam, &OcSyntheticImageFront_au8[buffCount][0] );
#else
          tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscFrontCam, get640x400AlgoView_pu8( ocdata::e_Front ) );
          tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscLeftCam, get640x400AlgoView_pu8( ocdata::e_Left ) );
          tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscRearCam, get640x400AlgoView_pu8( ocdata::e_Rear ) );
          tscCtrlInfo_o.PutM_Cmrs( tscApi::e_TscRightCam, get640x400AlgoView_pu8( ocdata::e_Right ) );
#endif
          v_IsAlgoInitSuccess_b = TSC_Init(&tscCtrlInfo_o);
          if(false == v_IsAlgoInitSuccess_b)
          {
            v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
            v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeInitFail;
            //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
            isAlgorithmStarted_b = false;
          }
          else
          {
            v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
            v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
            //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
          }

            OC_DEBUG_PRINTF( ( "*Statemachine_uninit* v_IsAlgoInitSuccess_b %d, state %d, error %d\n", v_IsAlgoInitSuccess_b, v_OcData_rs.ocAlgoState_e, v_OcData_rs.ocErrorCode_e ) );
          break;
        }
        case tscApi::e_TscStateInitOk:
        {
          //memset( &v_OcData_rs.ocDataToMcu_s, 0, sizeof( ocdata::OcDataToMcu_s ) );

          // If algorithm is already running, it has to be stopped. So that new OC run can be started.
          if(true == isAlgorithmStarted_b)
          {
            bool_t v_IsStopSuccess_b = false;
            v_IsStopSuccess_b = TSC_Stop();
            if(false == v_IsStopSuccess_b)
            {
              v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
              v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeUnexpectedRequest;
              //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
            }
            else
            {
              v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
              v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
              //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
            }

            OC_DEBUG_PRINTF( ( "*Statemachine_initok* OC running already, new start command comes. Current run stopped. \n" ) );
          }
          else
          {
            v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
            v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
            //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
          }

          bool_t v_IsAlgoStartSuccess_b = false;

          //Initialize tscStartConfiguration_as
          setOcKinematicModelConfiguration_v();
          setOcCamModelConfiguration_v();
          setOcFeatureCollectionConfiguration_v();

          //TODO Intermediate/saved data
          v_IsAlgoStartSuccess_b = TSC_Start(v_OcAlgoCamId_e, &tscStartConfiguration_as[v_OcAlgoCamId_e], NULL);
          if(false == v_IsAlgoStartSuccess_b)
          {
            v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
            v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeStartFail;
            //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
            isAlgorithmStarted_b = false;
          }
          else
          {
            v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
            v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
            //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
          }

          OC_DEBUG_PRINTF( ( "*Statemachine_initok* v_IsAlgoStartSuccess_b %d, state %d, error %d\n", v_IsAlgoStartSuccess_b, v_OcData_rs.ocAlgoState_e, v_OcData_rs.ocErrorCode_e ) );
          OC_DEBUG_PRINTF( ( "*Statemachine_initok* algoCam: %d, Machinestate: %d, stateToMcu: %d, errorToMcu: %d, DriveMan %d, validToMcu: %lu, ignoredToMcu: %lu, invalidToMcu: %lu, skiped: %d, procsd: %d, state: %d, error: %d\n"
              ,v_OcAlgoCamId_e
              ,tscAlgoState_ae[v_OcAlgoCamId_e]
              , v_OcData_rs.ocAlgoState_e
              , v_OcData_rs.ocErrorCode_e
              //,v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e
              , v_OcData_rs.validFeaturesCount_u32
              , v_OcData_rs.ignoredFeaturesCount_u32
              , v_OcData_rs.invalidFeaturesCount_u32
              ,TSC_GetSkippedFramesCount(v_OcAlgoCamId_e)
              ,TSC_GetProcessedFramesCount(v_OcAlgoCamId_e)
              ,TSC_GetState(v_OcAlgoCamId_e)
              ,TSC_GetError(v_OcAlgoCamId_e)
          ) );


          break;
        }

        case tscApi::e_TscStateError:
        {
          bool_t v_IsStopSuccess_b = false;
          v_IsStopSuccess_b = TSC_Stop();
          if(false == v_IsStopSuccess_b)
          {
            v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
            v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeFeatureCollectionError;
            //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
          }
          else
          {
            v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
            v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
            //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
          }

          v_OcData_rs.validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
          v_OcData_rs.ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
          v_OcData_rs.invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );
          OC_DEBUG_PRINTF( ( "*Statemachine_error* algoCam: %d, Machinestate: %d, stateToMcu: %d, errorToMcu: %d, driveMan %d, validToMcu: %lu, ignoredToMcu: %lu, invalidToMcu: %lu, skiped: %d, procsd: %d, state: %d, error: %d\n"
              ,v_OcAlgoCamId_e
              ,tscAlgoState_ae[v_OcAlgoCamId_e]
                                           , v_OcData_rs.ocAlgoState_e
                                           , v_OcData_rs.ocErrorCode_e
                                           //,v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e
                                           , v_OcData_rs.validFeaturesCount_u32
                                           , v_OcData_rs.ignoredFeaturesCount_u32
                                           , v_OcData_rs.invalidFeaturesCount_u32
              ,TSC_GetSkippedFramesCount(v_OcAlgoCamId_e)
              ,TSC_GetProcessedFramesCount(v_OcAlgoCamId_e)
              ,TSC_GetState(v_OcAlgoCamId_e)
              ,TSC_GetError(v_OcAlgoCamId_e)
                                         ) );
          isAlgorithmStarted_b = false;

          break;
        }
        case tscApi::e_TscStateFeatureCollection:
        {
          {
            bool_t v_IsAlgoFcSuccess_b = false;
#ifdef OC_HARDCODED_INPUT_IMAGE
            tscCtrlInfo_o.PutM_Cmrs( v_OcAlgoCamId_e, &OcSyntheticImageFront_au8[buffCount][0] );
            tscCtrlInfo_o.PutM_FrmNmbr( dataProvider_ro.getFrameNum_u32() );
            const uint8_t* c_AlgoViewBuf_pu8 = OcSyntheticImageFront_au8[buffCount];

            if( 0 == buffCount )
            {
              tscCtrlInfo_o.PutM_Spd( 6.29687500F );
              tscCtrlInfo_o.PutM_WhlAngl( mecl::math::toRadians_x( -0.0235990081F ) );
              buffCount = 1;
            }
            else if( 1 == buffCount )
            {
              tscCtrlInfo_o.PutM_Spd( 6.34375000F );
              tscCtrlInfo_o.PutM_WhlAngl( mecl::math::toRadians_x( -0.0235990081F ) );
              buffCount = 0;
            }

#else
            tscCtrlInfo_o.PutM_Spd( dataProvider_ro.getSpeed_f32() );
            tscCtrlInfo_o.PutM_WhlAngl( dataProvider_ro.getWheelAngle_f32() );
            //JTURK fix the trailer hitch for PC
            //tscCtrlInfo_o.PutM_HtchAngl(mecl::math::toRadians_x(dataInProvider_ro.getDataMcu()->trailerState_s.trlrAidAn3Actl_u16));
            const uint8_t* c_AlgoViewBuf_pu8 = get640x400AlgoView_pu8( static_cast<ocdata::CameraId_e>( v_OcData_rs.mcuCameraId_u8 ) );
            tscCtrlInfo_o.PutM_Cmrs(v_OcAlgoCamId_e, c_AlgoViewBuf_pu8);
            // PRQA S 1051 1 // This commented code is uncommented and used when debugging.
            //tscCtrlInfo_o.PutM_FrmNmbr(1 + tscCtrlInfo_o.GetM_FrmNmbr());
            tscCtrlInfo_o.PutM_FrmNmbr( dataProvider_ro.getFrameNum_u32() );

            if(1 > ocFrameCounter_u32)
            {
              OC_DEBUG_PRINTF( ( "*Statemachine_featurecollection* speed: %d, Angle: %d, TrailAng %d, frameId: %lu, Buf: %p\n"
                  ,static_cast<sint32_t>(tscCtrlInfo_o.getMSpd_f32()*10000)
                  ,static_cast<sint32_t>(tscCtrlInfo_o.getMWhlAngl_f32()*10000)
				  ,static_cast<sint32_t>(tscCtrlInfo_o.getMHtchAngl_f32() * 10000)
                  ,dataInProvider_ro.getDataSystem()->psFrameId_u32
                  ,c_AlgoViewBuf_pu8
              ));
            }

                //&& (true == isAlgoViewConfigured_b)
#endif

            if(6 > ocFrameCounter_u32)
            {
              OC_DEBUG_PRINTF( ( "*Statemachine_featurecollection* speed*10000: %d, Angle*10000: %d, TrailAngle*1000: %d,  frameId: %lu, Buf: %p\n"
                  ,static_cast<sint32_t>(tscCtrlInfo_o.getMSpd_f32()*10000)
                  ,static_cast<sint32_t>(tscCtrlInfo_o.getMWhlAngl_f32()*10000)
				  ,static_cast<sint32_t>(tscCtrlInfo_o.getMHtchAngl_f32() * 10000)
                  ,dataInProvider_ro.getDataSystem()->psFrameId_u32
                  ,c_AlgoViewBuf_pu8
              ) );
            }

#ifdef OC_GRAB_ALGO_VIEW_IMAGES
            if(    (testCounter_u32 >= V_StartFrame_u32)
                && (testCounter_u32 < V_FramesToRead_u32+V_StartFrame_u32))
            {
              v_IsAlgoFcSuccess_b = true;
              V_FrameIdB4_au32[testCounter_u32-V_StartFrame_u32] = dataInProvider_ro.getDataSystem()->psFrameId_u32;
              V_Addresses_pau8[testCounter_u32-V_StartFrame_u32] = (uint8_t *)c_AlgoViewBuf_pu8;
              for(uint32_t v_CamIndx_u32 = 0; v_CamIndx_u32 < V_NumCameras_u32; v_CamIndx_u32++)
              {
                memcpy(&V_AlgoViewBuff_au8[testCounter_u32-V_StartFrame_u32][v_CamIndx_u32*totalPixels_u32], c_AlgoViewBuf_pu8,totalPixels_u32);
              }
              V_FrameIdAfter_au32[testCounter_u32-V_StartFrame_u32]= dataInProvider_ro.getDataSystem()->psFrameId_u32;

            }
            else if(    (V_FramesToRead_u32 > 0)
                && (testCounter_u32 == V_FramesToRead_u32+V_StartFrame_u32))
            {
              v_IsAlgoFcSuccess_b = true;
              for(uint32_t v_Index_u32 = 0U; v_Index_u32 < V_FramesToRead_u32; v_Index_u32++)
              {
                vm_cprintf("V_Addresses_pau8: %p, V_FrameIdB4_au32: %lu, V_FrameIdB4_au32: %lu\n", V_Addresses_pau8[v_Index_u32], V_FrameIdB4_au32[v_Index_u32], V_FrameIdAfter_au32[v_Index_u32]);
              }
              vm_cprintf("\n\n");

              for(uint32_t v_Index_u32 = 0U; v_Index_u32 < V_FramesToRead_u32; v_Index_u32++)
              {
                for (uint32_t i = 0x0U; i < (totalPixels_u32*V_NumCameras_u32 + (c_ImageWidth_u32*0)); ++i)
                {
                  vm_cprintf("%d,", (int)V_AlgoViewBuff_au8[v_Index_u32][i]);
                }
                vm_cprintf("\n\n");
              }
              vm_cprintf("\n\n");
            }
            else
#endif
            {
              v_IsAlgoFcSuccess_b = TSC_ProcessFrame();
#ifdef DEBUG_TSC_ALG
              const tscApi::DebugCounters* v_DebugCounter_ps = TSC_GetDebugCounters(v_OcAlgoCamId_e);
              if(NULL != v_DebugCounter_ps)
              {
                vm_cprintf( "ignIGComb:%lu,ignIGDev:%lu,ignIGThr:%lu,ignMisPr:%lu,invPxMtn:%lu,invTrcL:%lu,invVSl:%lu,spd:%d,angl:%d,trlAngle:%d\n"
                    ,v_DebugCounter_ps->ignoredIGCombinThreshold
                    ,v_DebugCounter_ps->ignoredIGDeviation
                    ,v_DebugCounter_ps->ignoredIGThreshold
                    ,v_DebugCounter_ps->ignoredMissingPair
                    ,v_DebugCounter_ps->invalidPxMotion
                    ,v_DebugCounter_ps->invalidTrckLen
                    ,v_DebugCounter_ps->invalidValidSlope
                    ,static_cast<sint32_t>(tscCtrlInfo_o.getMSpd_f32()*10000)
                    ,static_cast<sint32_t>(mecl::math::toDegrees_x(tscCtrlInfo_o.getMWhlAngl_f32())*10000)
					,static_cast<sint32_t>(tscCtrlInfo_o.getMHtchAngl_f32() * 10000)
                );
              }
#endif
            }
#ifdef OC_GRAB_ALGO_VIEW_IMAGES
            testCounter_u32++;
#endif
            //OC_DEBUG_PRINTF(("t: %u, f: %u, t: %u\n",(static_cast<uint32_t>(v_time_o.getTimeNs_u64() / 1000)-v_TempTime_u32),dataInProvider_ro.getDataSystem()->psFrameId_u32,v_TempTime_u32));
            if(false == v_IsAlgoFcSuccess_b)
            {
              v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
              v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeFeatureCollectionError;
              //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;

              bool_t v_IsStopSuccess_b = false;
              v_IsStopSuccess_b = TSC_Stop();
              if(false == v_IsStopSuccess_b)
              {
                v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
                v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeFeatureCollectionError;
                //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
              }
              else
              {
                v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
                v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
                //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
              }

              isAlgorithmStarted_b = false;
            }
            else
            {
              v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
              v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
              //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
            }

            // fixme to be refactored as part of algo manager integration
            // create debug view for OC only if no other debug screen is activated
                            /*if( shmdata::e_AlgoDebugViewOc == dataInProvider_ro.getDataMcu()->algoControl_s.algMData_s.algoDebugView_e )
                            {
                                createDebugView_v( c_AlgoViewBuf_pu8, static_cast<uint8_t>( v_OcAlgoCamId_e ) );
                            }
                            else if( shmdata::e_AlgoDebugViewNone == dataInProvider_ro.getDataMcu()->algoControl_s.algMData_s.algoDebugView_e )
                            {
                                deactivateDebugView_v();
                            }
                            else
                            {
                            }*/
            }
            v_OcData_rs.validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
            v_OcData_rs.ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
            v_OcData_rs.invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );

          break;
        }
        case tscApi::e_TscStateFeatureCollectionCompleted:
        {
          bool_t v_IsAlgoCalibSuccess_b = false;
          //OC_DEBUG_PRINTF(("t: %u, f: %u\n",static_cast<uint32_t>((v_time_o.getTimeNs_u64() / 1000)),dataInProvider_ro.getDataSystem()->psFrameId_u32));
          //TODO: low prio-thread??
          v_IsAlgoCalibSuccess_b = TSC_Calibrate(v_OcAlgoCamId_e);

          //OC_DEBUG_PRINTF(("t: %u, f: %u\n",(static_cast<uint32_t>(v_time_o.getTimeNs_u64() / 1000)-v_TempTime_u32),dataInProvider_ro.getDataSystem()->psFrameId_u32));
          if(false == v_IsAlgoCalibSuccess_b)
          {
                            v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
                            v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeCalibrationError;
                            //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
            bool_t v_IsStopSuccess_b = false;
            v_IsStopSuccess_b = TSC_Stop();
            if(false == v_IsStopSuccess_b)
            {
                                v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
                                v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeFeatureCollectionError;
                                //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
            }
            else
            {
                                v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
                                v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
                                //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
            }
            isAlgorithmStarted_b = false;
          }
          else
          {
                            v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
                            v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
                            //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
          }

                        v_OcData_rs.validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
                        v_OcData_rs.ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
                        v_OcData_rs.invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );
          break;
        }
        case tscApi::e_TscStateCalibration:
        {
                        v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
                        v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );

                        v_OcData_rs.validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
                        v_OcData_rs.ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
                        v_OcData_rs.invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );
          break;
        }
        case tscApi::e_TscStateCalibrationCompleted:
        {
          bool_t v_IsAlgoFinalResultSuccess_b = false;
          v_IsAlgoFinalResultSuccess_b = TSC_GetFinalCalibrationResult(v_OcAlgoCamId_e, &tscCalibrationResults_as[v_OcAlgoCamId_e]);

#ifdef DEBUG_TSC_ALG
          bool_t v_IsStdDevSuccess_b = false;
          tscApi::CalibrationParams finalCalibrationiResultStdDev;

          v_IsStdDevSuccess_b = TSC_GetFinalCalibrationResultStdDev(v_OcAlgoCamId_e, &finalCalibrationiResultStdDev);

          if(false == v_IsStdDevSuccess_b)
          {
            vm_cprintf("StdDev func returned false\n");
          }
          else
          {
            vm_cprintf("stdev*10k:p:%d,y:%d,r:%d,x:%d,y:%d,z:%d\n"
                ,static_cast<sint32_t>(finalCalibrationiResultStdDev.pitchDeg_f64*10000)
                ,static_cast<sint32_t>(finalCalibrationiResultStdDev.yawDeg_f64*10000)
                ,static_cast<sint32_t>(finalCalibrationiResultStdDev.rollDeg_f64*10000)
                ,static_cast<sint32_t>(finalCalibrationiResultStdDev.xMM_f64*10000)
                ,static_cast<sint32_t>(finalCalibrationiResultStdDev.yMM_f64*10000)
                ,static_cast<sint32_t>(finalCalibrationiResultStdDev.zMM_f64*10000)
            );
          }
#endif

          if(false == v_IsAlgoFinalResultSuccess_b)
          {
                            v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
                            v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeCalibrationError;
                            //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
            bool_t v_IsStopSuccess_b = false;
            v_IsStopSuccess_b = TSC_Stop();
            if(false == v_IsStopSuccess_b)
            {
                                v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
                                v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeFeatureCollectionError;
                                //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
            }
            else
            {
                                v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
                                v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
                                //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
            }
          }
          else
          {
                            v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
                            v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
                            //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
          }

          if(    (mecl::math::abs_x<float64_t>(tscCalibrationResults_as[v_OcAlgoCamId_e].pitchDeg_f64) < mecl::math::numeric_limits<float32_t>::epsilon_x())
              || (NULL == c_OcExtrinsics_ps))
          {
                            v_OcData_rs.deltaPitch_f32 = 0.0F;
          }
          else
          {
                            v_OcData_rs.deltaPitch_f32 = static_cast<float32_t>( tscCalibrationResults_as[v_OcAlgoCamId_e].pitchDeg_f64 )
                                                                              - mecl::math::toDegrees_x(c_OcExtrinsics_ps->eulerAngles_s.pitch_x);
          }

          if(    (mecl::math::abs_x<float64_t>(tscCalibrationResults_as[v_OcAlgoCamId_e].yawDeg_f64) < mecl::math::numeric_limits<float32_t>::epsilon_x())
              || (NULL == c_OcExtrinsics_ps))
          {
                            v_OcData_rs.deltaYaw_f32 = 0.0F;
          }
          else
          {
                            v_OcData_rs.deltaYaw_f32 = static_cast<float32_t>( tscCalibrationResults_as[v_OcAlgoCamId_e].yawDeg_f64 )
                                                                            - mecl::math::toDegrees_x(c_OcExtrinsics_ps->eulerAngles_s.yaw_x);
          }

          if(    (mecl::math::abs_x<float64_t>(tscCalibrationResults_as[v_OcAlgoCamId_e].rollDeg_f64) < mecl::math::numeric_limits<float32_t>::epsilon_x())
              || (NULL == c_OcExtrinsics_ps))
          {
                            v_OcData_rs.deltaRoll_f32 = 0.0F;
          }
          else
          {
                            v_OcData_rs.deltaRoll_f32 = static_cast<float32_t>( tscCalibrationResults_as[v_OcAlgoCamId_e].rollDeg_f64 )
                                                                             - mecl::math::toDegrees_x(c_OcExtrinsics_ps->eulerAngles_s.roll_x);

                            if( v_OcData_rs.deltaRoll_f32 > 180.0F )
            {
                                v_OcData_rs.deltaRoll_f32 -= 360.0F;
            }
                            else if( v_OcData_rs.deltaRoll_f32 < -180.0F )
            {
                                v_OcData_rs.deltaRoll_f32 += 360.0F;
            }
            else
            {
              //Do nothing
            }
          }
                        if( ( mecl::math::abs_x<float64_t>( tscCalibrationResults_as[v_OcAlgoCamId_e].zMM_f64 ) < mecl::math::numeric_limits<float32_t>::epsilon_x() )
                            || ( NULL == c_OcExtrinsics_ps ) )
                        {
                            v_OcData_rs.deltaZ_f32 = 0.0F;
                        }
                        else
                        {
                            v_OcData_rs.deltaZ_f32 = static_cast<float32_t>( tscCalibrationResults_as[v_OcAlgoCamId_e].zMM_f64 )
                                                     - c_OcExtrinsics_ps->pos_x.cVal_ax[2] ;
                        }

                        if( tscCalibrationResults_as[v_OcAlgoCamId_e].xMM_f64 != 0 )
                        {
                            v_OcData_rs.deltaX_f32 = static_cast<float32_t>( tscCalibrationResults_as[v_OcAlgoCamId_e].xMM_f64 )
                                                     - c_OcExtrinsics_ps->pos_x.cVal_ax[0];
                        }
                        else
                        {
                            v_OcData_rs.deltaX_f32 = 0;
                        }

                        if( tscCalibrationResults_as[v_OcAlgoCamId_e].yMM_f64 != 0 )
                        {
                            v_OcData_rs.deltaY_f32 = static_cast<float32_t>( tscCalibrationResults_as[v_OcAlgoCamId_e].yMM_f64 )
                                                     - c_OcExtrinsics_ps->pos_x.cVal_ax[1];
                        }
                        else
                        {
                            v_OcData_rs.deltaY_f32 = 0;
                        }

                        v_OcData_rs.validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
                        v_OcData_rs.ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
                        v_OcData_rs.invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );
                        OC_DEBUG_PRINTF( ( "*Statemachine_calibrationcompleted* algoCam: %d, Machinestate: %d, stateToMcu: %d, errorToMcu: %d, driveMan %d, validToMcu: %lu, ignoredToMcu: %lu, invalidToMcu: %lu, skiped: %d, procsd: %d, state: %d, error: %d\n"
              ,v_OcAlgoCamId_e
              ,tscAlgoState_ae[v_OcAlgoCamId_e]
                                           , v_OcData_rs.ocAlgoState_e
                                           , v_OcData_rs.ocErrorCode_e
                                           //,v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e
                                           , v_OcData_rs.validFeaturesCount_u32
                                           , v_OcData_rs.ignoredFeaturesCount_u32
                                           , v_OcData_rs.invalidFeaturesCount_u32
              ,TSC_GetSkippedFramesCount(v_OcAlgoCamId_e)
              ,TSC_GetProcessedFramesCount(v_OcAlgoCamId_e)
              ,TSC_GetState(v_OcAlgoCamId_e)
              ,TSC_GetError(v_OcAlgoCamId_e)
                                         ) );

                        OC_DEBUG_PRINTF( ( "*Statemachine_calibrationcompleted* Pitch*10000: %d, Yaw*10000: %d, Roll*10000: %d, X*10000: %d, Y*10000: %d, Z*10000: %d\n"
              ,static_cast<sint32_t>(10000*tscCalibrationResults_as[v_OcAlgoCamId_e].pitchDeg_f64)
              ,static_cast<sint32_t>(10000*tscCalibrationResults_as[v_OcAlgoCamId_e].yawDeg_f64)
              ,static_cast<sint32_t>(10000*tscCalibrationResults_as[v_OcAlgoCamId_e].rollDeg_f64)
              ,static_cast<sint32_t>(10000*tscCalibrationResults_as[v_OcAlgoCamId_e].xMM_f64)
              ,static_cast<sint32_t>(10000*tscCalibrationResults_as[v_OcAlgoCamId_e].yMM_f64)
              ,static_cast<sint32_t>(10000*tscCalibrationResults_as[v_OcAlgoCamId_e].zMM_f64)
                                         ) );

                        OC_DEBUG_PRINTF( ( "*Statemachine_calibrationcompleted* deltaPitch*10000: %d, deltaYaw*10000: %d, deltaRoll*10000: %d, X*10000: %d, Y*10000: %d, Z*10000: %d\n"
                                           , static_cast<sint32_t>( 10000 * v_OcData_rs.deltaPitch_f32 )
                                           , static_cast<sint32_t>( 10000 * v_OcData_rs.deltaYaw_f32 )
                                           , static_cast<sint32_t>( 10000 * v_OcData_rs.deltaRoll_f32 )
             // ,static_cast<sint32_t>(10000 * v_OcData_rs.ocDataToMcu_s.camPosX_f32)
			 // ,static_cast<sint32_t>(10000 * v_OcData_rs.ocDataToMcu_s.camPosY_f32)
                                           , static_cast<sint32_t>( 10000 * v_OcData_rs.deltaZ_f32 )
                                         ) );

          isAlgorithmStarted_b = false;
                        isAlgorithmFinished_b = true;
                        // --------------------------------------------------------------------------------------
                        // Dumping to CSV file
                        DumpCalibrationResults( v_OcAlgoCamId_e, v_OcData_rs );
#if 0

                        if( v_OcAlgoCamId_e == tscApi::e_TscFrontCam )
                        {
                            DeleteFile( "straightMotion//FRONT_NEW_CalibrationResult.csv" );
                            CopyFile( "FRONT_CalibrationResult.csv", "straightMotion//FRONT_NEW_CalibrationResult.csv", true );
        }
                        else if( v_OcAlgoCamId_e == tscApi::e_TscRearCam )
                        {
                            DeleteFile( "straightMotion//REAR_NEW_CalibrationResult.csv" );
                            CopyFile( "REAR_CalibrationResult.csv", "straightMotion//REAR_NEW_CalibrationResult.csv", true );
                        }
                        else if( v_OcAlgoCamId_e == tscApi::e_TscLeftCam )
                        {
                            DeleteFile( "straightMotion//LEFT_NEW_CalibrationResult.csv" );
                            CopyFile( "LEFT_CalibrationResult.csv", "straightMotion//LEFT_NEW_CalibrationResult.csv", true );
                        }
                        else
                        {
                            DeleteFile( "straightMotion//RIGHT_NEW_CalibrationResult.csv" );
                            CopyFile( "RIGHT_CalibrationResult.csv", "straightMotion//RIGHT_NEW_CalibrationResult.csv", true );
                        }

#endif
                        // ------------------------------------------------------------------------------------------
                    }
        break;

        case tscApi::e_TscStateTerminated:
        {
                        v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
                        v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
                        //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));

                        v_OcData_rs.validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
                        v_OcData_rs.ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
                        v_OcData_rs.invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );
          isAlgorithmStarted_b = false;
          break;
        }
        case tscApi::e_TscStatePaused:
        {
                        v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
                        v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
                        //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
                        v_OcData_rs.validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
                        v_OcData_rs.ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
                        v_OcData_rs.invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );
          isAlgorithmStarted_b = false;
          break;
        }
        default:
        {
                        v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
                        v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
                        //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
                        v_OcData_rs.validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
                        v_OcData_rs.ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
                        v_OcData_rs.invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );
          break;
        }
      }

      //OC_DEBUG_PRINTF(("current Machinestate:%d\n",tscAlgoState_ae[v_OcAlgoCamId_e]));

                static ocdata::OcAlgoState_e v_PrevState_e;
                static ocdata::OcErrorCode_e v_PrevError_e;

      if(    ((ocFrameCounter_u32%100) == 0)
                    || ( v_PrevState_e != v_OcData_rs.ocAlgoState_e )
                    || ( v_PrevError_e != v_OcData_rs.ocErrorCode_e )
      )
      {
                    v_PrevState_e = v_OcData_rs.ocAlgoState_e;
                    v_PrevError_e = v_OcData_rs.ocErrorCode_e;
                    OC_DEBUG_PRINTF( ( "*execute_v:IsAlgoStarted* algoCam: %d, Machinestate: %d, state: %d, error: %d, driveMan: %d, valid: %lu, ignored: %lu, invalid: %lu, undetected: %d, skiped: %d, procsd: %d, state: %d, error: %d\n"
            ,v_OcAlgoCamId_e
            ,tscAlgoState_ae[v_OcAlgoCamId_e]
                                       , v_OcData_rs.ocAlgoState_e
                                       , v_OcData_rs.ocErrorCode_e
                                       //,v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e
                                       , v_OcData_rs.validFeaturesCount_u32
                                       , v_OcData_rs.ignoredFeaturesCount_u32
                                       , v_OcData_rs.invalidFeaturesCount_u32
            ,TSC_GetUndetectedFeaturesCount(v_OcAlgoCamId_e)
            ,TSC_GetSkippedFramesCount(v_OcAlgoCamId_e)
            ,TSC_GetProcessedFramesCount(v_OcAlgoCamId_e)
            ,TSC_GetState(v_OcAlgoCamId_e)
            ,TSC_GetError(v_OcAlgoCamId_e)
                                     ) );
                }

                //Abort strategy
                sint32_t v_ProcessedFramesCount_s32 = TSC_GetProcessedFramesCount( v_OcAlgoCamId_e );

      if (v_ProcessedFramesCount_s32 >= 0)
      {
        const uint32_t c_ProcessedFramesCount_u32 = static_cast<uint32_t>(v_ProcessedFramesCount_s32);
        //If processed frames are already more than minimum abort frame count limit
        if(    (c_MinAbortFrameCntLimit_u32 < c_ProcessedFramesCount_u32)
                        && ( ocdata::e_OcStateFeatureCollectionCompleted != v_OcData_rs.ocAlgoState_e )
                        && ( ocdata::e_OcStateCalibration != v_OcData_rs.ocAlgoState_e )
                        && ( ocdata::e_OcStateCalibrationCompleted != v_OcData_rs.ocAlgoState_e )
        )
        {
          abortFrameCounter_u32++;

          if(c_FrameWindow_u32 == abortFrameCounter_u32)
          {
            abortFrameCounter_u32 = 0U;
                            if( c_MinValidFeaturesInWindow_u32 > ( v_OcData_rs.validFeaturesCount_u32 - prevValidFeatureCount_u32 ) )
            {
              bool_t v_IsStopSuccess_b = false;
              v_IsStopSuccess_b = TSC_Stop();
              if(false == v_IsStopSuccess_b)
              {
                                    v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
                                    v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeFeatureCollectionError;
                                    //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
              }
              else
              {
                                    v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
                                    v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
                                    //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
              }

                                v_OcData_rs.validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
                                v_OcData_rs.ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
                                v_OcData_rs.invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );

                                OC_DEBUG_PRINTF( ( "*execute_v:abortHandling* algoCam: %d, Machinestate: %d, stateToMcu: %d, errorToMcu: %d, validToMcu: %lu, ignoredToMcu: %lu, invalidToMcu: %lu, skiped: %d, procsd: %d, state: %d, error: %d\n"
                  ,v_OcAlgoCamId_e
                  ,tscAlgoState_ae[v_OcAlgoCamId_e]
                                                   , v_OcData_rs.ocAlgoState_e
                                                   , v_OcData_rs.ocErrorCode_e
                                                   //,v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e
                                                   , v_OcData_rs.validFeaturesCount_u32
                                                   , v_OcData_rs.ignoredFeaturesCount_u32
                                                   , v_OcData_rs.invalidFeaturesCount_u32
                  ,TSC_GetSkippedFramesCount(v_OcAlgoCamId_e)
                  ,TSC_GetProcessedFramesCount(v_OcAlgoCamId_e)
                  ,TSC_GetState(v_OcAlgoCamId_e)
                  ,TSC_GetError(v_OcAlgoCamId_e)
                                                 ) );

              isAlgorithmStarted_b = false;
            }
            else
            {
                                prevValidFeatureCount_u32 = v_OcData_rs.validFeaturesCount_u32;
            }
          }
        }
        else
        {
                        prevValidFeatureCount_u32 = v_OcData_rs.validFeaturesCount_u32;
          abortFrameCounter_u32 = 0;
        }
      }
      else
      {
        bool_t v_IsStopSuccess_b = false;
        v_IsStopSuccess_b = TSC_Stop();
        if(false == v_IsStopSuccess_b)
        {
                        v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
                        v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeFeatureCollectionError;
                        //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;

        }
        else
        {
                        v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
                        v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
                        //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
        }
                    v_OcData_rs.validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
                    v_OcData_rs.ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
                    v_OcData_rs.invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );

                    OC_DEBUG_PRINTF( ( "*execute_v:AbortProcessFramesCountElseCase* algoCam: %d, Machinestate: %d, stateToMcu: %d, errorToMcu: %d, validToMcu: %lu, ignoredToMcu: %lu, invalidToMcu: %lu, skiped: %d, procsd: %d, state: %d, error: %d\n"
            ,v_OcAlgoCamId_e
            ,tscAlgoState_ae[v_OcAlgoCamId_e]
                                       , v_OcData_rs.ocAlgoState_e
                                       , v_OcData_rs.ocErrorCode_e
                                       //,v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e
                                       , v_OcData_rs.validFeaturesCount_u32
                                       , v_OcData_rs.ignoredFeaturesCount_u32
                                       , v_OcData_rs.invalidFeaturesCount_u32
            ,TSC_GetSkippedFramesCount(v_OcAlgoCamId_e)
            ,TSC_GetProcessedFramesCount(v_OcAlgoCamId_e)
            ,TSC_GetState(v_OcAlgoCamId_e)
            ,TSC_GetError(v_OcAlgoCamId_e)
                                     ) );
        isAlgorithmStarted_b = false;
      }
    }
    else
    {
      bool_t v_IsStopSuccess_b = false;
      v_IsStopSuccess_b = TSC_Stop();
      if(false == v_IsStopSuccess_b)
      {
                    v_OcData_rs.ocAlgoState_e = ocdata::e_OcStateError;
                    v_OcData_rs.ocErrorCode_e = ocdata::e_OcErrCodeFeatureCollectionError;
                    //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = shmdata::aZynqM_e_TrlrCamraCalibManeuver_FAILED;
      }
      else
      {
                    v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( TSC_GetState( v_OcAlgoCamId_e ) );
                    v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
                    //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
      }
                v_OcData_rs.validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
                v_OcData_rs.ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
                v_OcData_rs.invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );

                OC_DEBUG_PRINTF( ( "*execute_v:NoErrorElseCase* algoCam: %d, Machinestate: %d, stateToMcu: %d, errorToMcu: %d, driveMan: %d, validToMcu: %lu, ignoredToMcu: %lu, invalidToMcu: %lu, skiped: %d, procsd: %d, state: %d, error: %d\n"
          ,v_OcAlgoCamId_e
          ,tscAlgoState_ae[v_OcAlgoCamId_e]
                                   , v_OcData_rs.ocAlgoState_e
                                   , v_OcData_rs.ocErrorCode_e
                                   //,v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e
                                   , v_OcData_rs.validFeaturesCount_u32
                                   , v_OcData_rs.ignoredFeaturesCount_u32
                                   , v_OcData_rs.invalidFeaturesCount_u32
          ,TSC_GetSkippedFramesCount(v_OcAlgoCamId_e)
          ,TSC_GetProcessedFramesCount(v_OcAlgoCamId_e)
          ,TSC_GetState(v_OcAlgoCamId_e)
          ,TSC_GetError(v_OcAlgoCamId_e)
                                 ) );
      isAlgorithmStarted_b = false;
    }
  }
  else
  {
    //Do nothing
  }

  if(true == isNewCommand_b)
  {
    switch(algoCommand_e)
    {
                case ocdata::e_Start:
      {
        break;
      }
                case ocdata::e_Stop:
      {
        break;
      }

                case ocdata::e_Pause:
      {
        break;
      }

                case ocdata::e_Status:
      {
        //      OC_DEBUG_PRINTF(("JobOC::execute_v = %d, status\n", static_cast<printInt_t>(v_time_o.getTimeNs_u64() / 1000000)));

                    if( ( ocdata::e_OcStateError != v_OcData_rs.ocAlgoState_e )
                        && ( ocdata::e_OcErrCodeNoError == v_OcData_rs.ocErrorCode_e ) )
        {
                        v_OcData_rs.ocAlgoState_e = static_cast<ocdata::OcAlgoState_e>( tscAlgoState_ae[v_OcAlgoCamId_e] );
                        v_OcData_rs.ocErrorCode_e = static_cast<ocdata::OcErrorCode_e>( TSC_GetError( v_OcAlgoCamId_e ) );
                        //v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e = static_cast<shmdata::TrlrCamraCalibManeuver_Stat_e>(TSC_GetCalibManeuver(v_OcAlgoCamId_e));
        }

                    v_OcData_rs.validFeaturesCount_u32 = TSC_GetValidFeaturesCount( v_OcAlgoCamId_e );
                    v_OcData_rs.ignoredFeaturesCount_u32 = TSC_GetIgnoredValidFeaturesCount( v_OcAlgoCamId_e );
                    v_OcData_rs.invalidFeaturesCount_u32 = TSC_GetInvalidFeaturesCount( v_OcAlgoCamId_e );

                    OC_DEBUG_PRINTF( ( "*StatusCommand* algoCam: %d, Machinestate: %d, stateToMcu: %d, errorToMcu: %d, DriveMan: %d, validToMcu: %lu, ignoredToMcu: %lu, invalidToMcu: %lu, skiped: %d, procsd: %d, state: %d, error: %d\n"
            ,v_OcAlgoCamId_e
            ,tscAlgoState_ae[v_OcAlgoCamId_e]
                                       , v_OcData_rs.ocAlgoState_e
                                       , v_OcData_rs.ocErrorCode_e
                                       //,v_OcData_rs.ocDataToMcu_s.ocCalibManeuver_e
                                       , v_OcData_rs.validFeaturesCount_u32
                                       , v_OcData_rs.ignoredFeaturesCount_u32
                                       , v_OcData_rs.invalidFeaturesCount_u32
            ,TSC_GetSkippedFramesCount(v_OcAlgoCamId_e)
            ,TSC_GetProcessedFramesCount(v_OcAlgoCamId_e)
            ,TSC_GetState(v_OcAlgoCamId_e)
            ,TSC_GetError(v_OcAlgoCamId_e)
                                     ) );

        break;
      }
                case ocdata::e_Sync:
      {
        break;
      }
//      case shmdata::e_Set:
//      {
//        break;
//      }
                case ocdata::e_Resume:
      {
                    //configureFpgaForAlgoView_v( isAlgorithmStarted_b );
        break;
      }
                case ocdata::e_Debug:
      {
                    OC_DEBUG_PRINTF( ( "execute_v = %lu, debug, val: %d\n", v_Time_o.getTimeMs_u32(), dataInProvider_ro.getDataMcu()->algoControl_s.algMData_s.algoDebugView_e ) );
        break;
      }
      default:
      {
                    OC_DEBUG_PRINTF( ( "JobOC::execute_v = %lu, unknown command from MCU\n", v_Time_o.getTimeMs_u32() ) );
        break;
      }
    }
    isNewCommand_b = false;
  }

  v_OcData_rs.isOcActive_b = isAlgorithmStarted_b;
        v_OcData_rs.updatedAt_u32 = dataProvider_ro.getFrameNum_u32();
  //    OC_DEBUG_PRINTF(("execute_v finish = %d, lastreq_at = %u, reqat: %u\n", static_cast<printInt_t>(v_time_o.getTimeNs_u64() / 1000000), v_OcData_rs.lastRequestedAt_u32, dataInProvider_ro.getDataMcu()->algoControl_s.cntrl_s.requestedAt_u32));
  state_e = e_DoNothing;
}

void JobOC::end_v()
{
}

    ocdata::OcDriverPosition_e JobOC::getDriverPosition_e( void ) const
{
        // For now, hardcode this value since there is no
        // MCU to get variant data from (haha)
        return ocdata::e_DriverPositionLHD;
        //return static_cast<ocdata::OcDriverPosition_e>( dataInProvider_ro.getDataMcu()->variantCoding_s.data_s.configBits_s.driverPosition_b );
}


    // Getting Fisheye view from Camera
    const uint8_t* JobOC::get640x400AlgoView_pu8( ocdata::CameraId_e CamID ) const
{
        const uint8_t* c_AlgoViewBufferAddress_pu8;
        //uint8_t v_WindowsCamID_u8 = getWindowsCamID( CamID );
        //uint8_t v_FpgaCamId_u8 = prjcontainer::AdapterCameraReal::getCameraConnectorIdFromMcuId_u8( i_McuCamId_e, dataInProvider_ro );

        // Only need one switch case
        // No two buffers like embedded
        switch( CamID )
  {
    case 1:
    {
                c_AlgoViewBufferAddress_pu8 = dataProvider_ro.getFrontInputImage();
      break;
    }
    case 2:
    {
                c_AlgoViewBufferAddress_pu8 = dataProvider_ro.getLeftInputImage();
      break;
    }
    case 3:
    {
                c_AlgoViewBufferAddress_pu8 = dataProvider_ro.getRearInputImage();
                break;
            }

            case 4:
            {
                c_AlgoViewBufferAddress_pu8 = dataProvider_ro.getRightInputImage();
      break;
    }
    default:
    {
                c_AlgoViewBufferAddress_pu8 = dataProvider_ro.getFrontInputImage();
      break;
    }
  }
  return c_AlgoViewBufferAddress_pu8;
}


    // Doesn't make sense to have a function for configuring FPGA
    // For now, ignore this function entirely and remove all references/calls

    //    void JobOC::configureFpgaForAlgoView_v( bool_t i_EnableAlgoView_b )
    //    {
    //        osal::Time v_Time_o;
    //        prjcontainer::SharedMem v_SharedMem_o;
    //#ifdef OC_LOG_PRINTF_ON
    //        uint32_t* v_ControlReg_pu32 = v_SharedMem_o.accessRw_pu32( prjcontainer::e_AxiPlRegister );
    //#endif
    //        uint32_t* v_ModSelectionReg0_pu32 = v_SharedMem_o.accessRw_pu32( prjcontainer::e_AxiAlgoViewSelection0 );
    //        uint32_t* v_ModSelectionReg1_pu32 = v_SharedMem_o.accessRw_pu32( prjcontainer::e_AxiAlgoViewSelection1 );
    //
    //        if( true == i_EnableAlgoView_b )
    //        {
    //            switch( algoViewBufferToRead_e )
    //            {
    //                case shmdata::e_OcAlgoViewBuffer0:
    //                {
    //                    //so that next time, AlgoView Buffer 1 is read by OC, which is currently being written by FPGA.
    //                    algoViewBufferToRead_e = shmdata::e_OcAlgoViewBuffer1;
    //                    //Configuring registers so that FPGA writes in buffer0 in next frame, and OC reads buffer 1 in that frame.
    //                    dataOutProvider_ro.getOutData()->algoViewBuffer_e = shmdata::e_OcAlgoViewBuffer0;
    //#if (PIKEOS_VERSION < 40) // Hydra2
    //                    // PRQA S 3706 1 //Using subscript operator here is intended.
    //                    v_ModSelectionReg0_pu32[0] = 0x01U; //Full Resolution (1280x800) AlgoView turned off. Only 640x400 AlgoView turned on.
    //#else   // Hydra2e
    //                    // Bit 4   : Enable QuadView (640x400 for each CamID)
    //                    // Bit 1..2: CamID
    //                    // Bit 0   : Enable FullView (1280x800 for selected CamID)
    //                    v_ModSelectionReg0_pu32[0] = ( 1U << 4 ); //Full Resolution (1280x800) AlgoView turned off. Only 640x400 AlgoView turned on.
    //                    v_ModSelectionReg0_pu32[1] = ( 1U << 4 ); //Config FPGA to write Algoview to Buffer1
    //#endif
    //                    algoViewConfiguredCounter_u32++;
    //                    break;
    //                }
    //
    //                case shmdata::e_OcAlgoViewBuffer1:
    //                {
    //                    //so that next time, AlgoView Buffer 0 is read by OC, which is currently being written by FPGA.
    //                    algoViewBufferToRead_e = shmdata::e_OcAlgoViewBuffer0;
    //                    //Configuring registers so that FPGA writes in buffer1 in next frame, and OC reads buffer 0 in that frame.
    //                    dataOutProvider_ro.getOutData()->algoViewBuffer_e = shmdata::e_OcAlgoViewBuffer1;
    //#if (PIKEOS_VERSION < 40) // Hydra2
    //                    // PRQA S 3706 1 //Using subscript operator here is intended.
    //                    v_ModSelectionReg1_pu32[0] = 0x01U; //Full Resolution (1280x800) AlgoView turned off. Only 640x400 AlgoView turned on.
    //#else   // Hydra2e
    //                    // Bit 4   : Enable QuadView (640x400 for each CamID)
    //                    // Bit 1..2: CamID
    //                    // Bit 0   : Enable FullView (1280x800 for selected CamID)
    //                    v_ModSelectionReg0_pu32[0] = ( 1U << 4 ); //Full Resolution (1280x800) AlgoView turned off. Only 640x400 AlgoView turned on.
    //                    v_ModSelectionReg0_pu32[1] = 0U;        //Config FPGA to write Algoview to Buffer0
    //#endif
    //                    algoViewConfiguredCounter_u32++;
    //                    break;
    //                }
    //
    //                default:
    //                {
    //                    break;
    //                }
    //            }
    //        }
    //        else
    //        {
    //            //Turn off 640x400 AlgoView.
    //            // PRQA S 3706 2 //Using subscript operator here is intended.
    //            v_ModSelectionReg0_pu32[0] &= ~( 0x01 );
    //            v_ModSelectionReg1_pu32[0] &= ~( 0x01 );
    //            algoViewConfiguredCounter_u32 = 0U;
    //            OC_DEBUG_PRINTF( ( "*ConfigFpga* buf2read %d, frameID: %lu, algoFrame: %lu, FPGA registers %#lx %#lx %#lx\n"
    //                               , algoViewBufferToRead_e
    //                               , dataInProvider_ro.getDataSystem()->psFrameId_u32
    //                               , tscCtrlInfo_o.getMFrmNmbr_u32()
    //                               , v_ControlReg_pu32[( ( 0x100 / 4 ) )]
    //                               , v_ModSelectionReg0_pu32[0]
    //                               , v_ModSelectionReg1_pu32[0]
    //                             ) );
    //        }
    //
    //        if( 10 > ocFrameCounter_u32 )
    //        {
    //            OC_DEBUG_PRINTF( ( "*ConfigFpga* buf2read %d, frameID: %lu, algoFrame: %lu, FPGA registers %#lx %#lx %#lx\n"
    //                               , algoViewBufferToRead_e
    //                               , dataInProvider_ro.getDataSystem()->psFrameId_u32
    //                               , tscCtrlInfo_o.getMFrmNmbr_u32()
    //                               , v_ControlReg_pu32[( ( 0x100 / 4 ) )]
    //                               , v_ModSelectionReg0_pu32[0]
    //                               , v_ModSelectionReg1_pu32[0]
    //                             ) );
    //        }
    //    }


    tscApi::enuCameraID JobOC::getOcAlgoCamId_e( ocdata::CameraId_e i_McuCamId_e )
{
  tscApi::enuCameraID v_OcAlgoCamId_e;

  switch(i_McuCamId_e)
  {
            case ocdata::e_Front:
    {
      v_OcAlgoCamId_e = tscApi::e_TscFrontCam;
      break;
    }
            case ocdata::e_Left:
    {
      v_OcAlgoCamId_e = tscApi::e_TscLeftCam;
      break;
    }
            case ocdata::e_Rear:
    {
      v_OcAlgoCamId_e = tscApi::e_TscRearCam;
      break;
    }
            case ocdata::e_Right:
    {
      v_OcAlgoCamId_e = tscApi::e_TscRightCam;
      break;
    }
    default:
    {
      v_OcAlgoCamId_e = tscApi::e_TscFrontCam;
      break;
    }
  }

  return v_OcAlgoCamId_e;
}

    // New function for converting OC Camera ID to Windows Camera ID (Camera.h)
    // TODO: remove ocdata::CameraId_e and make all camera IDs reference
    // to the Windows Camera.h enum
    Camera_ID JobOC::getWindowsCamID( ocdata::CameraId_e OCCamID )
    {
        Camera_ID v_WindowsCamID_e;

        switch( OCCamID )
        {
            case 1:
            {
                v_WindowsCamID_e = Front;
                break;
            }

            case 2:
            {
                v_WindowsCamID_e = Left;
                break;
            }

            case 3:
            {
                v_WindowsCamID_e = Rear;
                break;
            }

            case 4:
            {
                v_WindowsCamID_e = Right;
                break;
            }

            default:
            {
                v_WindowsCamID_e = Front;
                break;
            }
        }

        return v_WindowsCamID_e;
    }

void JobOC::setOcKinematicModelConfiguration_v(void)
{
  for(uint32_t v_Index_u32 = 0U; v_Index_u32 < static_cast<uint32_t>(tscApi::e_TscNumCam);v_Index_u32++)
  {
#if 1
            // Leave hardcoded for now since I'm not sure how to get KinematicModel config from DataProvider
            tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.tireCircumferencePerPulseMM_f32 = 1.0F;
            /*tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.distanceCoG2FrontAxisMM_f64     = 1550.5;
            tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.distanceCoG2RearAxisMM_f64      = 1550.5;*/
            /*tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.distanceCoG2FrontAxisMM_f64 = 1835.97;
            tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.distanceCoG2RearAxisMM_f64 = 1835.97;*/
            tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.distanceCoG2FrontAxisMM_f64 = 1435;
            tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.distanceCoG2RearAxisMM_f64 = 1435;
            tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.distanceThreshMM = OCcfg::configData.kmDistThresh_f32;
            tscStartConfiguration_as[v_Index_u32].MaxNumValidFrames_i32 = OCcfg::configData.MaxNumValidFrames_i32;
            tscStartConfiguration_as[v_Index_u32].MinNoRawFrames_i32 = OCcfg::configData.MinNoRawFrames_i32;
#else
    // PRQA S 3706 3 //Using subscript operator here is intended.
    tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.tireCircumferencePerPulseMM_f32 = dataInProvider_ro.getDataVarM()->tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_s.tireCircumferencePerPulseMm_f32;
    tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.distanceCoG2FrontAxisMM_f64     = dataInProvider_ro.getDataVarM()->tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_s.distanceCoG2FrontAxisMm_f64;
    tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.distanceCoG2RearAxisMM_f64      = dataInProvider_ro.getDataVarM()->tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_s.distanceCoG2RearAxisMm_f64;
            OC_DEBUG_PRINTF( ( "c%lu,t*10k%d,f*10k%d,r*10k%d\n"
        ,v_Index_u32
        ,static_cast<sint32_t>(tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.tireCircumferencePerPulseMM_f32*10000)
        ,static_cast<sint32_t>(tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.distanceCoG2FrontAxisMM_f64*10000)
        ,static_cast<sint32_t>(tscStartConfiguration_as[v_Index_u32].kinematicModelExternalConfig_t.distanceCoG2RearAxisMM_f64*10000)
        ) );
#endif
  }
}


    // No MCU or Real Cameras, need to replace with Windows DataProvider somehow
    void JobOC::setCamParametersOneCam_v( ocdata::CameraId_e camID )
    {
        tscApi::enuCameraID v_OcAlgoCamId_e = getOcAlgoCamId_e( camID );
        //uint8_t v_ConnectorCamId_u8 = prjcontainer::AdapterCameraReal::getCameraConnectorIdFromMcuId_u8( i_McuCamId_e, dataProvider_ro );
#if defined(OC_LOG_PRINTF_ON) || defined(USE_SVSCM)
  mecl::Sensor_t* v_OcSensor_po = NULL;
  mecl::LensRadial_t::Config_s v_OcLensRadialCfg_s;
  mecl::LensRadial_t* v_OcLensRadial_po = NULL;
  const mecl::Pinhole_t::Intrinsic_s* v_OcIntrinsic_ps = NULL;
#endif

  mecl::Camera_t* v_OcCamera_po = NULL;
        //#ifdef OC_HARDCODED_INPUT_IMAGE // Use same camera data as live version
#if 0

  switch(i_McuCamId_e)
  {
            case ocdata::e_Front:
    {
      v_OcCamera_po = &adapterCameraRealFront_o.accessCamera_rt(v_ConnectorCamId_u8, prjcontainer::AdapterCameraReal::e_ExtrDesign, prjcontainer::AdapterCameraReal::e_IntrDefault, prjcontainer::AdapterCameraReal::e_LensDefault, false);
      break;
    }
            case ocdata::e_Left:
    {
      v_OcCamera_po = &adapterCameraRealLeft_o.accessCamera_rt(v_ConnectorCamId_u8, prjcontainer::AdapterCameraReal::e_ExtrDesign, prjcontainer::AdapterCameraReal::e_IntrDefault, prjcontainer::AdapterCameraReal::e_LensDefault, false);
      break;
    }
            case ocdata::e_Rear:
    {
      v_OcCamera_po = &adapterCameraRealRear_o.accessCamera_rt(v_ConnectorCamId_u8, prjcontainer::AdapterCameraReal::e_ExtrDesign, prjcontainer::AdapterCameraReal::e_IntrDefault, prjcontainer::AdapterCameraReal::e_LensDefault, false);
      break;
    }
            case ocdata::e_Right:
    {
      v_OcCamera_po = &adapterCameraRealRight_o.accessCamera_rt(v_ConnectorCamId_u8, prjcontainer::AdapterCameraReal::e_ExtrDesign, prjcontainer::AdapterCameraReal::e_IntrDefault, prjcontainer::AdapterCameraReal::e_LensDefault, false);
      break;
    }
    default:
    {
      v_OcCamera_po = &adapterCameraRealFront_o.accessCamera_rt(v_ConnectorCamId_u8, prjcontainer::AdapterCameraReal::e_ExtrDesign, prjcontainer::AdapterCameraReal::e_IntrDefault, prjcontainer::AdapterCameraReal::e_LensDefault, false);
      break;
    }
  }
#else
        switch( camID )
  {
            case ocdata::e_Front:
    {
                //v_OcCamera_po = &adapterCameraRealFront_o.accessCamera_rt( v_ConnectorCamId_u8, prjcontainer::AdapterCameraReal::e_ExtrDesign, prjcontainer::AdapterCameraReal::e_IntrCurrent, prjcontainer::AdapterCameraReal::e_LensCurrent, true );
                v_OcCamera_po = &dataProvider_ro.getFrontDoCameraParam().camera_param.m_realCam;
      break;
    }
            case ocdata::e_Left:
    {
                //v_OcCamera_po = &adapterCameraRealLeft_o.accessCamera_rt( v_ConnectorCamId_u8, prjcontainer::AdapterCameraReal::e_ExtrDesign, prjcontainer::AdapterCameraReal::e_IntrCurrent, prjcontainer::AdapterCameraReal::e_LensCurrent, true );
                v_OcCamera_po = &dataProvider_ro.getLeftDoCameraParam().camera_param.m_realCam;
      break;
    }
            case ocdata::e_Rear:
    {
                //v_OcCamera_po = &adapterCameraRealRear_o.accessCamera_rt( v_ConnectorCamId_u8, prjcontainer::AdapterCameraReal::e_ExtrDesign, prjcontainer::AdapterCameraReal::e_IntrCurrent, prjcontainer::AdapterCameraReal::e_LensCurrent, true );
                v_OcCamera_po = &dataProvider_ro.getRearDoCameraParam().camera_param.m_realCam;
      break;
    }
            case ocdata::e_Right:
    {
                //v_OcCamera_po = &adapterCameraRealRight_o.accessCamera_rt( v_ConnectorCamId_u8, prjcontainer::AdapterCameraReal::e_ExtrDesign, prjcontainer::AdapterCameraReal::e_IntrCurrent, prjcontainer::AdapterCameraReal::e_LensCurrent, true );
                v_OcCamera_po = &dataProvider_ro.getRightDoCameraParam().camera_param.m_realCam;
      break;
    }
    default:
    {
                //v_OcCamera_po = &adapterCameraRealFront_o.accessCamera_rt( v_ConnectorCamId_u8, prjcontainer::AdapterCameraReal::e_ExtrDesign, prjcontainer::AdapterCameraReal::e_IntrCurrent, prjcontainer::AdapterCameraReal::e_LensCurrent, true );
                v_OcCamera_po = &dataProvider_ro.getFrontDoCameraParam().camera_param.m_realCam;
      break;
    }
  }
#endif

  if(NULL != v_OcCamera_po)
  {
    // PRQA S 3077 3 // This downcast is necessary here.
    mecl::Pinhole_t& v_PinHole_ro = dynamic_cast<mecl::Pinhole_t&>(v_OcCamera_po->getImager_rx());
    v_PinHole_ro.init_v();

    c_OcExtrinsics_ps = &v_PinHole_ro.getExtrinsic_rs();

    //OC_DEBUG_PRINTF(("After Extrinsics:x*10k:%d, y*10k:%d, z*10k:%d, pre_r:%d, rot_type: %d\n", static_cast<sint32_t>(v_NotConstOcExtrinsics_s.pos_x.cVal_ax[0]*10000.0F), static_cast<sint32_t>(v_NotConstOcExtrinsics_s.pos_x.cVal_ax[1]*10000.0F), static_cast<sint32_t>(v_NotConstOcExtrinsics_s.pos_x.cVal_ax[2]*10000.0F), v_NotConstOcExtrinsics_s.preRoll_e, v_NotConstOcExtrinsics_s.rotationType_e));



#if defined(OC_LOG_PRINTF_ON) || defined(USE_SVSCM)
    // PRQA S 3077 2 // This downcast is necessary here.
    v_OcSensor_po = &dynamic_cast<mecl::Sensor_t&>(v_OcCamera_po->getSensor_rx());
    v_OcLensRadial_po = &dynamic_cast<mecl::LensRadial_t&>(v_OcCamera_po->getLens_rx());
    v_OcLensRadial_po->copyConfig_v(v_OcLensRadialCfg_s);
    v_OcIntrinsic_ps = &v_PinHole_ro.getIntrinsic_rs();
#endif
  }

        OC_DEBUG_PRINTF( ( "extr_ps: %p, Cam_ro: %p\n", c_OcExtrinsics_ps, v_OcCamera_po ) );

  if(NULL != c_OcExtrinsics_ps)
  {
            OC_DEBUG_PRINTF( ( "p*10K:%d, y*10K:%d, r*10k:%d\n", static_cast<sint32_t>( mecl::math::toDegrees_x( c_OcExtrinsics_ps->eulerAngles_s.pitch_x * 10000.0F ) ), static_cast<sint32_t>( mecl::math::toDegrees_x( c_OcExtrinsics_ps->eulerAngles_s.yaw_x * 10000.0F ) ), static_cast<sint32_t>( mecl::math::toDegrees_x( c_OcExtrinsics_ps->eulerAngles_s.roll_x * 10000.0F ) ) ) );
            OC_DEBUG_PRINTF( ( "x*10k:%d, y*10k:%d, z*10k:%d, pre_r:%d, rot_type: %d\n", static_cast<sint32_t>( c_OcExtrinsics_ps->pos_x.cVal_ax[0] * 10000.0F ), static_cast<sint32_t>( c_OcExtrinsics_ps->pos_x.cVal_ax[1] * 10000.0F ), static_cast<sint32_t>( c_OcExtrinsics_ps->pos_x.cVal_ax[2] * 10000.0F ), c_OcExtrinsics_ps->preRoll_e, c_OcExtrinsics_ps->rotationType_e ) );

  }

#ifdef OC_LOG_PRINTF_ON

  if(NULL != v_OcIntrinsic_ps)
  {
            OC_DEBUG_PRINTF( ( "fX*10k:%d,fY*10k:%d,pppCfgX*10k:%d,pppCfgY*10k:%d\n"
        ,static_cast<sint32_t>(v_OcIntrinsic_ps->foclX_x*10000.0F)
        ,static_cast<sint32_t>(v_OcIntrinsic_ps->foclY_x*10000.0F)
        ,static_cast<sint32_t>(v_OcIntrinsic_ps->pppCfg_x.cVal_ax[0]*10000.0F)
        ,static_cast<sint32_t>(v_OcIntrinsic_ps->pppCfg_x.cVal_ax[1]*10000.0F)
                             ) );
  }

  if(NULL != v_OcSensor_po)
  {
            OC_DEBUG_PRINTF( ( "ppx*10k:%d,ppy*10k:%d\n", static_cast<sint32_t>( v_OcSensor_po->getPpp_rx().getPosX() * 10000.0F ), static_cast<sint32_t>( v_OcSensor_po->getPpp_rx().getPosY() * 10000.0F ) ) );
  }

  if (NULL != v_OcLensRadial_po)
  {
            OC_DEBUG_PRINTF( ( "W2I[0]*100K: %d, W2I[1]*100K: %d, W2I[2]*100K: %d, W2I[3]*100K: %d, W2I[4]*100K: %d, W2I[5]*100K: %d\n",
        static_cast<sint32_t>(v_OcLensRadialCfg_s.world2image_x[0] * 100000.0F),
        static_cast<sint32_t>(v_OcLensRadialCfg_s.world2image_x[1] * 100000.0F),
        static_cast<sint32_t>(v_OcLensRadialCfg_s.world2image_x[2] * 100000.0F),
        static_cast<sint32_t>(v_OcLensRadialCfg_s.world2image_x[3] * 100000.0F),
        static_cast<sint32_t>(v_OcLensRadialCfg_s.world2image_x[4] * 100000.0F),
        static_cast<sint32_t>(v_OcLensRadialCfg_s.world2image_x[5] * 100000.0F)
                             ) );
            OC_DEBUG_PRINTF( ( "I2W[0]*100K: %d, I2W[1]*100K: %d, I2W[2]*100K: %d, I2W[3]*100K: %d, I2W[4]*100K: %d, I2W[5]*100K: %d\n, ",
        static_cast<sint32_t>(v_OcLensRadialCfg_s.image2world_x[0] * 100000.0F),
        static_cast<sint32_t>(v_OcLensRadialCfg_s.image2world_x[1] * 100000.0F),
        static_cast<sint32_t>(v_OcLensRadialCfg_s.image2world_x[2] * 100000.0F),
        static_cast<sint32_t>(v_OcLensRadialCfg_s.image2world_x[3] * 100000.0F),
        static_cast<sint32_t>(v_OcLensRadialCfg_s.image2world_x[4] * 100000.0F),
        static_cast<sint32_t>(v_OcLensRadialCfg_s.image2world_x[5] * 100000.0F)
                             ) );
  }
#endif


#ifdef USE_SVSCM
  if (NULL != v_OcIntrinsic_ps)
  {
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.focalLength = static_cast<float64_t>(v_OcIntrinsic_ps->foclX_x);
  }

  if (NULL != v_OcSensor_po)
  {
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.pixelSize   = static_cast<float64_t>(v_OcSensor_po->getPsz_rx().getPosX());

    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.ppx         = static_cast<float64_t>(v_OcSensor_po->getPpp_rx().getPosX());
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.ppy         = static_cast<float64_t>(v_OcSensor_po->getPpp_rx().getPosY());
#if 0
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.pixelSize   = static_cast<float64_t>(v_OcSensorCfg_ps->pszCfg_x.cVal_ax[0]);

    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.ppx         = static_cast<float64_t>(v_OcSensorCfg_ps->pppCfg_x.cVal_ax[0]);// + 0.5F //TODO: 0.5F added for SVS model?
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.ppy         = static_cast<float64_t>(v_OcSensorCfg_ps->pppCfg_x.cVal_ax[1]);// + 0.5F //TODO: 0.5F added for SVS model?
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.axisX       = static_cast<float64_t>(v_OcSensorCfg_ps->pppCfg_x.cVal_ax[0]);
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.axisY       = static_cast<float64_t>(v_OcSensorCfg_ps->pppCfg_x.cVal_ax[1]);
#endif
    OC_DEBUG_PRINTF(("camId:%d, pixelSize*10000: %d, focalLength*10000: %d, ppx*10000: %d, ppy*10000: %d\n, ",
        v_OcAlgoCamId_e,
        static_cast<sint32_t>(tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.pixelSize * 10000),
        static_cast<sint32_t>(tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.focalLength * 10000),
        static_cast<sint32_t>(tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.ppx * 10000),
        static_cast<sint32_t>(tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.ppy * 10000)));
  }

  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.radialZeroCrossing = 2400.0;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.radialCoeff1 = 0.0;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.radialCoeff2 = 0.0;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.radialCoeff3 = 0.0;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.tangentialCoeff1 = 0.0;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.tangentialCoeff2 = 0.0;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.affineCoeff1 = 0.0;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.affineCoeff2 = 0.0;

  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.origX = 1280.0;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.origY = 800.0;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.axisX = 639.5;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.axisY = 399.5;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.intrinsicParams.downsampleFactor = 2;

  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.extrinsicParams.X_mm = 0.0;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.extrinsicParams.Y_mm = 0.0;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.extrinsicParams.flipped = false;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.orientationParams.preRoll_deg = V_PreRoll_af32[v_OcAlgoCamId_e];

  if (NULL != c_OcExtrinsics_ps)
  {
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.orientationParams.deltaX  = static_cast<float64_t>(c_OcExtrinsics_ps->pos_x.cVal_ax[0]);
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.orientationParams.deltaY  = static_cast<float64_t>(c_OcExtrinsics_ps->pos_x.cVal_ax[1]);
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.extrinsicParams.Z_mm      = static_cast<float64_t>(c_OcExtrinsics_ps->pos_x.cVal_ax[2]);
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.extrinsicParams.Pitch_deg = static_cast<float64_t>(mecl::math::toDegrees_x(c_OcExtrinsics_ps->eulerAngles_s.pitch_x));
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.extrinsicParams.Yaw_deg   = static_cast<float64_t>(mecl::math::toDegrees_x(c_OcExtrinsics_ps->eulerAngles_s.yaw_x));
    tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.extrinsicParams.Roll_deg  = static_cast<float64_t>(mecl::math::toDegrees_x(c_OcExtrinsics_ps->eulerAngles_s.roll_x));

    OC_DEBUG_PRINTF(("pitch*10000: %d, yaw*10000: %d, roll*10000: %d, PreRoll*10000:%d\n, ",
        static_cast<sint32_t>(tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.extrinsicParams.Pitch_deg * 10000),
        static_cast<sint32_t>(tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.extrinsicParams.Yaw_deg * 10000),
        static_cast<sint32_t>(tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.extrinsicParams.Roll_deg * 10000),
        static_cast<sint32_t>(tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.orientationParams.preRoll_deg * 10000)
    ));
    OC_DEBUG_PRINTF(("deltaX*10000: %d, deltaY*10000: %d, z*10000: %d\n, ",
        static_cast<sint32_t>(tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.orientationParams.deltaX * 10000),
        static_cast<sint32_t>(tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.orientationParams.deltaY * 10000),
        static_cast<sint32_t>(tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.extrinsicParams.Z_mm * 10000)
    ));
  }
  if (NULL != v_OcLensRadial_po)
  {
  }
#else
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.camera_px = v_OcCamera_po;
  tscStartConfiguration_as[v_OcAlgoCamId_e].cameraModelExternalConfig_t.downSampleFactor_f32 = 2.0F;
#endif

}

void JobOC::setOcCamModelConfiguration_v(void)
{
        setCamParametersOneCam_v( ocdata::e_Front );
        setCamParametersOneCam_v( ocdata::e_Left );
        setCamParametersOneCam_v( ocdata::e_Rear );
        setCamParametersOneCam_v( ocdata::e_Right );
  setCamParametersOneCam_v(mcuCamIdToOcAlgo_e);
}

void JobOC::setOcFeatureCollectionConfiguration_v(void)
{
#ifndef QNX_ENVIRONMENT

        for( int CamID = 0; CamID < tscApi::e_TscNumCam; CamID++ )
        {
            // Trajectory Filter
            tscStartConfiguration_as[CamID].featureColExternalConfig_t.trajectoryFilterConfig_t.minPixelMotionThresh_u32 = OCcfg::configData.CalibrationConfigs[CamID].m_TrajectoryFilter.minPixelMotionThresh;
            tscStartConfiguration_as[CamID].featureColExternalConfig_t.trajectoryFilterConfig_t.slopeDifferenceThreshold_f64 = OCcfg::configData.CalibrationConfigs[CamID].m_TrajectoryFilter.slopeDifferenceThreshold;
            tscStartConfiguration_as[CamID].featureColExternalConfig_t.trajectoryFilterConfig_t.useSfmFilter_b = OCcfg::configData.CalibrationConfigs[CamID].m_TrajectoryFilter.useSfmFilter;
            tscStartConfiguration_as[CamID].featureColExternalConfig_t.trajectoryFilterConfig_t.maxHeightDiffMm_f64 = OCcfg::configData.CalibrationConfigs[CamID].m_TrajectoryFilter.maxHeightDiff;
            tscStartConfiguration_as[CamID].featureColExternalConfig_t.trajectoryFilterConfig_t.angleThresholdDegIG_f64 = OCcfg::configData.CalibrationConfigs[CamID].m_TrajectoryFilter.angleThresholdDegIG;
            tscStartConfiguration_as[CamID].featureColExternalConfig_t.trajectoryFilterConfig_t.deviationPercentageIG_u32 = OCcfg::configData.CalibrationConfigs[CamID].m_TrajectoryFilter.deviationPercentageIG;
            tscStartConfiguration_as[CamID].featureColExternalConfig_t.trajectoryFilterConfig_t.useCombinations_b = OCcfg::configData.CalibrationConfigs[CamID].m_TrajectoryFilter.useCombinations;
            tscStartConfiguration_as[CamID].featureColExternalConfig_t.trajectoryFilterConfig_t.combinationsDiffThresholdDeg_f64 = OCcfg::configData.CalibrationConfigs[CamID].m_TrajectoryFilter.combinationsDiffThreshold;
            tscStartConfiguration_as[CamID].trimMeanPercentage_f32 = OCcfg::configData.trimMeanPercentage_f32;

            // BMALFC
            for( int ind = 0; ind < tscApi::NUM_SPEED_RANGES; ind++ )
            {
                tscStartConfiguration_as[CamID].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[ind] = OCcfg::configData.CalibrationConfigs[CamID].m_BundleConfiguration.speedRanges[ind];
                tscStartConfiguration_as[CamID].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32[ind] = OCcfg::configData.CalibrationConfigs[CamID].m_BundleConfiguration.frameSkips[ind];
            }

            for( int ROI_ind = 0; ROI_ind < OCcfg::configData.CalibrationConfigs[CamID].m_BundleConfiguration.ROIs_Init; ROI_ind++ )
            {
                tscStartConfiguration_as[CamID].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[ROI_ind].x_s32 = OCcfg::configData.CalibrationConfigs[CamID].m_BundleConfiguration.ROIs[ROI_ind].x_s32;
                tscStartConfiguration_as[CamID].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[ROI_ind].y_s32 = OCcfg::configData.CalibrationConfigs[CamID].m_BundleConfiguration.ROIs[ROI_ind].y_s32;
                tscStartConfiguration_as[CamID].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[ROI_ind].width_s32 = OCcfg::configData.CalibrationConfigs[CamID].m_BundleConfiguration.ROIs[ROI_ind].width_s32;
                tscStartConfiguration_as[CamID].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[ROI_ind].height_s32 = OCcfg::configData.CalibrationConfigs[CamID].m_BundleConfiguration.ROIs[ROI_ind].height_s32;
            }
        }

#else
        // FRONT CAM
        // Trajectory Filter
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.trajectoryFilterConfig_t.minPixelMotionThresh_u32         = 5;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.trajectoryFilterConfig_t.slopeDifferenceThreshold_f64     = 0.08;
        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.trajectoryFilterConfig_t.useSfmFilter_b                      = false;
        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.trajectoryFilterConfig_t.maxHeightDiffMm_f64                 = 20.0;
        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.trajectoryFilterConfig_t.angleThresholdDegIG_f64             = 5.0;
        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.trajectoryFilterConfig_t.deviationPercentageIG_u32           = 20;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.trajectoryFilterConfig_t.useCombinations_b              = true;
        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.trajectoryFilterConfig_t.combinationsDiffThresholdDeg_f64    = 2.0;
        // BMALFC
        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[0] = 3;
        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[1] = 7;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[2] = 15;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[3] = 20;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[4] = 25;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[5] = 35;
        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [0] = 4;
        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [1] = 2;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [2] = 1;
        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [3] = 0;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [4] = 0;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [5] = 0;

        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].x_s32 = 200;
        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].y_s32 = 265;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].width_s32  = 30;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].height_s32 = 30;

        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].x_s32 = 440;
        tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].y_s32 = 265;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].width_s32  = 30;
  tscStartConfiguration_as[tscApi::e_TscFrontCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].height_s32 = 30;

  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.trajectoryFilterConfig_t.minPixelMotionThresh_u32          = 5; /* default 5    */
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.trajectoryFilterConfig_t.slopeDifferenceThreshold_f64     = 0.05; /* default 0.02 */
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.trajectoryFilterConfig_t.useSfmFilter_b                   = false;
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.trajectoryFilterConfig_t.maxHeightDiffMm_f64              = 20.0;
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.trajectoryFilterConfig_t.angleThresholdDegIG_f64          = 5;
  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.trajectoryFilterConfig_t.deviationPercentageIG_u32         = 20;
  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.trajectoryFilterConfig_t.useCombinations_b               = true;
  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.trajectoryFilterConfig_t.combinationsDiffThresholdDeg_f64  = 2;
        // BMALFC
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[0] = 6;
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[1] = 12;
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[2] = 17;
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[3] = 22;
  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[4] = 25;
  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[5] = 35;
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [0] = 4;
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [1] = 3;
  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [2] = 2;
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [3] = 1;
  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [4] = 0;
  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [5] = 0;

        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].x_s32 = 410;
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].y_s32 = 210;
  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].width_s32  = 40;
  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].height_s32 = 40;

        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].x_s32 = 410;
        tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].y_s32 = 165;
  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].width_s32  = 40;
  tscStartConfiguration_as[tscApi::e_TscLeftCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].height_s32 = 40;

  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.trajectoryFilterConfig_t.minPixelMotionThresh_u32          = 5; /* default 5    */
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.trajectoryFilterConfig_t.slopeDifferenceThreshold_f64      = 0.05; /* default 0.05 */
        tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.trajectoryFilterConfig_t.useSfmFilter_b                   = false;
        tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.trajectoryFilterConfig_t.maxHeightDiffMm_f64              = 20.0;
        tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.trajectoryFilterConfig_t.angleThresholdDegIG_f64          = 5;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.trajectoryFilterConfig_t.deviationPercentageIG_u32         = 20;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.trajectoryFilterConfig_t.useCombinations_b               = true;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.trajectoryFilterConfig_t.combinationsDiffThresholdDeg_f64  = 2;
        // BMALFC
        tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[0] = 6;
        tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[1] = 11;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[2] = 15;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[3] = 20;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[4] = 25;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[5] = 35;
        tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [0] = 4;
        tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [1] = 3;
        tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [2] = 2;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [3] = 1;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [4] = 0;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [5] = 0;

        tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].x_s32 = 200;
        tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].y_s32 = 280;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].width_s32  = 30;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].height_s32 = 30;

        tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].x_s32 = 450;
        tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].y_s32 = 280;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].width_s32  = 30;
  tscStartConfiguration_as[tscApi::e_TscRearCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].height_s32 = 30;

  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.trajectoryFilterConfig_t.minPixelMotionThresh_u32          = 5; /* default 5    */
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.trajectoryFilterConfig_t.slopeDifferenceThreshold_f64        = 0.05; /* default 0.05 */
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.trajectoryFilterConfig_t.useSfmFilter_b                      = false;
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.trajectoryFilterConfig_t.maxHeightDiffMm_f64                 = 20.0;
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.trajectoryFilterConfig_t.angleThresholdDegIG_f64             = 5;
  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.trajectoryFilterConfig_t.deviationPercentageIG_u32         = 20;
  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.trajectoryFilterConfig_t.useCombinations_b               = true;
  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.trajectoryFilterConfig_t.combinationsDiffThresholdDeg_f64  = 2;
        // BMALFC
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[0] = 6;
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[1] = 12;
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[2] = 17;
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[3] = 22;
  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[4] = 25;
  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.speedRanges_au32[5] = 35;
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [0] = 4;
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [1] = 3;
  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [2] = 2;
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [3] = 1;
  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [4] = 0;
  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.frameSkips_au32 [5] = 0;

        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].x_s32 = 300;
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].y_s32 = 230;
  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].width_s32  = 40;
  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[0].height_s32 = 40;

        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].x_s32 = 300;
        tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].y_s32 = 170;
  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].width_s32  = 40;
  tscStartConfiguration_as[tscApi::e_TscRightCam].featureColExternalConfig_t.bmalfcExtConfig_t.rois_at[1].height_s32 = 40;
#endif
#if 0
        //for( uint32_t v_Index_u32 = 0U; v_Index_u32 < static_cast<uint32_t>( tscApi::e_TscNumCam ); v_Index_u32++ )
        //{
        //    // PRQA S 3706 1 //Using subscript operator here is intended.
        //    const shmdata::TrajectoryFilterConfigStr_t& c_VarMTrajectoryFilterConfig_rs = dataInProvider_ro.getDataVarM()->tscStartConfiguration_as[v_Index_u32].featureColExternalConfig_s.trajectoryFilterConfig_s;
        //    tscApi::TrajectoryFilter_ConfigStrType& v_TscTrajectoryFilterConfig_rs = tscStartConfiguration_as[v_Index_u32].featureColExternalConfig_t.trajectoryFilterConfig_t;
        //    v_TscTrajectoryFilterConfig_rs.minPixelMotionThresh_u32          = c_VarMTrajectoryFilterConfig_rs.minPixelMotionThresh_u32;
        //    v_TscTrajectoryFilterConfig_rs.slopeDifferenceThreshold_f64      = c_VarMTrajectoryFilterConfig_rs.slopeDifferenceThreshold_f64;
        //    v_TscTrajectoryFilterConfig_rs.angleThresholdDegIG_f64          = c_VarMTrajectoryFilterConfig_rs.angleThresholdDegIG_f64;
        //    v_TscTrajectoryFilterConfig_rs.deviationPercentageIG_u32         = c_VarMTrajectoryFilterConfig_rs.deviationPercentageIG_u32;
        //    v_TscTrajectoryFilterConfig_rs.useCombinations_b               = c_VarMTrajectoryFilterConfig_rs.useCombinations_b;
        //    v_TscTrajectoryFilterConfig_rs.combinationsDiffThresholdDeg_f64  = c_VarMTrajectoryFilterConfig_rs.combinationsDiffThresholdDeg_f64;
        //    v_TscTrajectoryFilterConfig_rs.useSfmFilter_b                  = c_VarMTrajectoryFilterConfig_rs.useSfMFilter_b;
        //    v_TscTrajectoryFilterConfig_rs.maxHeightDiffMm_f64              = c_VarMTrajectoryFilterConfig_rs.maxHeightDiffMm_f64;
        //    // PRQA S 3706 1 //Using subscript operator here is intended.
        //    const shmdata::BmalFcExtConfigStr_t& c_VarMBmalfcExtConfig_rs = dataInProvider_ro.getDataVarM()->tscStartConfiguration_as[v_Index_u32].featureColExternalConfig_s.bmalfcExtConfig_s;
        //    tscApi::BMALFC_extConfigStrType& v_TscBmalfcExtConfig_rs = tscStartConfiguration_as[v_Index_u32].featureColExternalConfig_t.bmalfcExtConfig_t;
        //    v_TscBmalfcExtConfig_rs.speedRanges_au32[0] = c_VarMBmalfcExtConfig_rs.speedRanges_au32[0];
        //    v_TscBmalfcExtConfig_rs.speedRanges_au32[1] = c_VarMBmalfcExtConfig_rs.speedRanges_au32[1];
        //    v_TscBmalfcExtConfig_rs.speedRanges_au32[2] = c_VarMBmalfcExtConfig_rs.speedRanges_au32[2];
        //    v_TscBmalfcExtConfig_rs.speedRanges_au32[3] = c_VarMBmalfcExtConfig_rs.speedRanges_au32[3];
        //    v_TscBmalfcExtConfig_rs.speedRanges_au32[4] = c_VarMBmalfcExtConfig_rs.speedRanges_au32[4];
        //    v_TscBmalfcExtConfig_rs.speedRanges_au32[5] = c_VarMBmalfcExtConfig_rs.speedRanges_au32[5];
        //    v_TscBmalfcExtConfig_rs.frameSkips_au32 [0] = c_VarMBmalfcExtConfig_rs.frameSkips_au32[0];
        //    v_TscBmalfcExtConfig_rs.frameSkips_au32 [1] = c_VarMBmalfcExtConfig_rs.frameSkips_au32[1];
        //    v_TscBmalfcExtConfig_rs.frameSkips_au32 [2] = c_VarMBmalfcExtConfig_rs.frameSkips_au32[2];
        //    v_TscBmalfcExtConfig_rs.frameSkips_au32 [3] = c_VarMBmalfcExtConfig_rs.frameSkips_au32[3];
        //    v_TscBmalfcExtConfig_rs.frameSkips_au32 [4] = c_VarMBmalfcExtConfig_rs.frameSkips_au32[4];
        //    v_TscBmalfcExtConfig_rs.frameSkips_au32 [5] = c_VarMBmalfcExtConfig_rs.frameSkips_au32[5];
        //    v_TscBmalfcExtConfig_rs.rois_at[0].x_s32      = c_VarMBmalfcExtConfig_rs.rois_as[0].x_s32;
        //    v_TscBmalfcExtConfig_rs.rois_at[0].y_s32      = c_VarMBmalfcExtConfig_rs.rois_as[0].y_s32;
        //    v_TscBmalfcExtConfig_rs.rois_at[0].width_s32  = c_VarMBmalfcExtConfig_rs.rois_as[0].width_s32;
        //    v_TscBmalfcExtConfig_rs.rois_at[0].height_s32 = c_VarMBmalfcExtConfig_rs.rois_as[0].height_s32;
        //    v_TscBmalfcExtConfig_rs.rois_at[1].x_s32      = c_VarMBmalfcExtConfig_rs.rois_as[1].x_s32;
        //    v_TscBmalfcExtConfig_rs.rois_at[1].y_s32      = c_VarMBmalfcExtConfig_rs.rois_as[1].y_s32;
        //    v_TscBmalfcExtConfig_rs.rois_at[1].width_s32  = c_VarMBmalfcExtConfig_rs.rois_as[1].width_s32;
        //    v_TscBmalfcExtConfig_rs.rois_at[1].height_s32 = c_VarMBmalfcExtConfig_rs.rois_as[1].height_s32;
    //float32_t test_var = 0.06f;
//    OC_DEBUG_PRINTF("c%d,m%d,d%d,u%d,s*10k%d,a*10k%d,c*10k%d,sfm%d,h%d,s0:%d,s1:%d,s2:%d,s3:%d,s4:%d,s5:%d, %d, %f\n"
//        ,v_Index_u32
//        ,v_TscTrajectoryFilterConfig_rs.minPixelMotionThresh_u32
//        ,v_TscTrajectoryFilterConfig_rs.deviationPercentageIG_u32
//        ,v_TscTrajectoryFilterConfig_rs.useCombinations_b
//        ,static_cast<sint32_t>(v_TscTrajectoryFilterConfig_rs.slopeDifferenceThreshold_f64*10000)
//        ,static_cast<sint32_t>(v_TscTrajectoryFilterConfig_rs.angleThresholdDegIG_f64*10000)
//        ,static_cast<sint32_t>(v_TscTrajectoryFilterConfig_rs.combinationsDiffThresholdDeg_f64*10000)
//        ,v_TscTrajectoryFilterConfig_rs.useSfmFilter_b
//        ,static_cast<sint32_t>(v_TscTrajectoryFilterConfig_rs.maxHeightDiffMm_f64*10000)
//        ,v_TscBmalfcExtConfig_rs.speedRanges_au32[0]
//        ,v_TscBmalfcExtConfig_rs.speedRanges_au32[1]
//        ,v_TscBmalfcExtConfig_rs.speedRanges_au32[2]
//        ,v_TscBmalfcExtConfig_rs.speedRanges_au32[3]
//        ,v_TscBmalfcExtConfig_rs.speedRanges_au32[4]
//        ,v_TscBmalfcExtConfig_rs.speedRanges_au32[5]
//		,static_cast<sint32_t>(tscStartConfiguration_as[v_Index_u32].trimMeanPercentage * 100)
//		, test_var
//    );

        //    OC_DEBUG_PRINTF( ( "f0:%lu,f1:%lu,f2:%lu,f3:%lu,f4:%lu,f5:%lu,r0:%d,r1:%d,r2:%d,r3:%d,r4:%d,r5:%d,r6:%d,r7:%d\n"
        //                       , v_TscBmalfcExtConfig_rs.frameSkips_au32 [0]
        //                       , v_TscBmalfcExtConfig_rs.frameSkips_au32 [1]
        //                       , v_TscBmalfcExtConfig_rs.frameSkips_au32 [2]
        //                       , v_TscBmalfcExtConfig_rs.frameSkips_au32 [3]
        //                       , v_TscBmalfcExtConfig_rs.frameSkips_au32 [4]
        //                       , v_TscBmalfcExtConfig_rs.frameSkips_au32 [5]
        //                       , v_TscBmalfcExtConfig_rs.rois_at[0].x_s32
        //                       , v_TscBmalfcExtConfig_rs.rois_at[0].y_s32
        //                       , v_TscBmalfcExtConfig_rs.rois_at[0].width_s32
        //                       , v_TscBmalfcExtConfig_rs.rois_at[0].height_s32
        //                       , v_TscBmalfcExtConfig_rs.rois_at[1].x_s32
        //                       , v_TscBmalfcExtConfig_rs.rois_at[1].y_s32
        //                       , v_TscBmalfcExtConfig_rs.rois_at[1].width_s32
        //                       , v_TscBmalfcExtConfig_rs.rois_at[1].height_s32
        //                     ) );*/
        //}
#endif
}

void JobOC::dump_v()
{
}

    int JobOC::vid_save_bmp( char* input_filename, char* img, int h, int w )
    {
        FILE* f;
        //int filesize = 54 + 3*w*h;  //w is your image width, h is image height, both int
        int filesize = 54 + w * h; //w is your image width, h is image height, both int
        int i;
        unsigned int image_offset = 54 + 4 * 256; /* offset is file header + Dib header + colormap */
        unsigned char bmpfileheader[14] = { 'B', 'M', 0, 0, 0, 0, 0, 0, 0, 0, 54, 0, 0, 0 };
        unsigned char bmpinfoheader[40] = { 40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 8, 0 };
        unsigned char bmppad[3] = { 0, 0, 0 };
        bmpfileheader[2] = ( unsigned char )( filesize );
        bmpfileheader[3] = ( unsigned char )( filesize >> 8 );
        bmpfileheader[4] = ( unsigned char )( filesize >> 16 );
        bmpfileheader[5] = ( unsigned char )( filesize >> 24 );
        bmpfileheader[0xA] = ( unsigned char )( image_offset >> 0 ); /* set offset of image */
        bmpfileheader[0xB] = ( unsigned char )( image_offset >> 8 );
        bmpfileheader[0xC] = ( unsigned char )( image_offset >> 16 );
        bmpfileheader[0xD] = ( unsigned char )( image_offset >> 24 );
        bmpinfoheader[4] = ( unsigned char )( w );
        bmpinfoheader[5] = ( unsigned char )( w >> 8 );
        bmpinfoheader[6] = ( unsigned char )( w >> 16 );
        bmpinfoheader[7] = ( unsigned char )( w >> 24 );
        bmpinfoheader[8] = ( unsigned char )( h );
        bmpinfoheader[9] = ( unsigned char )( h >> 8 );
        bmpinfoheader[10] = ( unsigned char )( h >> 16 );
        bmpinfoheader[11] = ( unsigned char )( h >> 24 );
        f = fopen( input_filename, "wb" );
        fwrite( bmpfileheader, 1, 14, f );
        fwrite( bmpinfoheader, 1, 40, f );
        // write out 256 entry grayscale
        unsigned char bgr_[4];

        for( i = 0; i < 256; i++ )
        {
            bgr_[0] = ( char )i;
            bgr_[1] = ( char )i;
            bgr_[2] = ( char )i;
            bgr_[3] = 0;
            fwrite( bgr_, 1, 4, f );
        }

        for( i = 0; i < h; i++ )
        {
            fwrite( img + ( w * ( h - i - 1 ) ), 1, w, f );
            fwrite( bmppad, 1, ( 4 - ( w ) % 4 ) % 4, f );
            //    fwrite(img+(w*(h-i-1)*3),3,w,f);
            //    fwrite(bmppad,1,(4-(w*3)%4)%4,f);
        }

        fclose( f );
        return( 0 );
    }

    void JobOC::vid_save_txt( const char* input_filename, char* img, int h, int w )
    {
        // Save image as 8 bit hex map
        FILE* fp;
        fp = fopen( input_filename, "w" );
        int k = 0;

        for( int i = 0; i < h; i++ )
        {
            for( int j = 0; j < w; j++ )
            {
                fprintf( fp, "0x%hhx,", img[k] );
                k++;
            }

            fprintf( fp, "\n" );
        }
    }

    void JobOC::DumpCalibrationResults( tscApi::enuCameraID v_OcAlgoCamId_e, ocdata::OcData_s& v_OcData_rs )
    {
        m_AlgoStateHeader.clear();
        m_AlgoStateStream.clear();
        m_ResultHeader.clear();
        m_ResultStream.clear();
        m_AlgoStateHeader
                << "Camera ID"
                << ",TSC Algo State"
                << ",State to MCU"
                << ",OC Error Code"
                << ",Valid Features"
                << ",Ignored Features"
                << ",Invalid Features"
                << ",Skipped Frames"
                << ",Processed Frames"
                << ",State"
                << ",Error"
                << "\n";
        m_AlgoStateStream
                << v_OcAlgoCamId_e << ","
                << tscAlgoState_ae[v_OcAlgoCamId_e] << ","
                << v_OcData_rs.ocAlgoState_e << ","
                << v_OcData_rs.ocErrorCode_e << ","
                << v_OcData_rs.validFeaturesCount_u32 << ","
                << v_OcData_rs.ignoredFeaturesCount_u32 << ","
                << v_OcData_rs.invalidFeaturesCount_u32 << ","
                << TSC_GetSkippedFramesCount( v_OcAlgoCamId_e ) << ","
                << TSC_GetProcessedFramesCount( v_OcAlgoCamId_e ) << ","
                << TSC_GetState( v_OcAlgoCamId_e ) << ","
                << TSC_GetError( v_OcAlgoCamId_e ) << "\n";
        m_ResultHeader
                << "Pitch"
                << ",Yaw"
                << ",Roll"
                << ",X"
                << ",Y"
                << ",Z"
                << ",Delta Pitch"
                << ",Delta Yaw"
                << ",Delta Roll"
                << ",Delta X"
                << ",Delta Y"
                << ",Delta Z"
                << "\n";
        m_ResultStream
        /*<< static_cast<sint32_t>( 1000 * tscCalibrationResults_as[v_OcAlgoCamId_e].pitchDeg_f64 ) << ","
        << static_cast<sint32_t>( 1000 * tscCalibrationResults_as[v_OcAlgoCamId_e].yawDeg_f64 ) << ","
        << static_cast<sint32_t>( 1000 * tscCalibrationResults_as[v_OcAlgoCamId_e].rollDeg_f64 ) << ","
        << static_cast<sint32_t>( 1000 * tscCalibrationResults_as[v_OcAlgoCamId_e].xMM_f64 ) << ","
        << static_cast<sint32_t>( 1000 * tscCalibrationResults_as[v_OcAlgoCamId_e].yMM_f64 ) << ","
        << static_cast<sint32_t>( 1000 * tscCalibrationResults_as[v_OcAlgoCamId_e].zMM_f64 ) << ","
        << static_cast<sint32_t>( 1000 * v_OcData_rs.deltaPitch_f32 ) << ","
        << static_cast<sint32_t>( 1000 * v_OcData_rs.deltaYaw_f32 ) << ","
        << static_cast<sint32_t>( 1000 * v_OcData_rs.deltaRoll_f32 ) << "\n";*/
                << tscCalibrationResults_as[v_OcAlgoCamId_e].pitchDeg_f64 << ","
                << tscCalibrationResults_as[v_OcAlgoCamId_e].yawDeg_f64 << ","
                << tscCalibrationResults_as[v_OcAlgoCamId_e].rollDeg_f64 << ","
                << tscCalibrationResults_as[v_OcAlgoCamId_e].xMM_f64 << ","
                << tscCalibrationResults_as[v_OcAlgoCamId_e].yMM_f64 << ","
                << tscCalibrationResults_as[v_OcAlgoCamId_e].zMM_f64 << ","
                << v_OcData_rs.deltaPitch_f32 << ","
                << v_OcData_rs.deltaYaw_f32 << ","
                << v_OcData_rs.deltaRoll_f32 << ","
                << v_OcData_rs.deltaX_f32 << ","
                << v_OcData_rs.deltaY_f32 << ","
                << v_OcData_rs.deltaZ_f32 << "\n";

        switch( v_OcAlgoCamId_e )
        {
            case 0: //Front
                AppCtrl::WriteToFile( "FRONT_AlgoStateResult.csv", m_AlgoStateStream, m_AlgoStateHeader, true );
                AppCtrl::WriteToFile( "FRONT_CalibrationResult.csv", m_ResultStream, m_ResultHeader, true );
                break;

            case 1: //Left
                AppCtrl::WriteToFile( "LEFT_AlgoStateResult.csv", m_AlgoStateStream, m_AlgoStateHeader, true );
                AppCtrl::WriteToFile( "LEFT_CalibrationResult.csv", m_ResultStream, m_ResultHeader, true );
                break;

            case 2: //Rear
                AppCtrl::WriteToFile( "REAR_AlgoStateResult.csv", m_AlgoStateStream, m_AlgoStateHeader, true );
                AppCtrl::WriteToFile( "REAR_CalibrationResult.csv", m_ResultStream, m_ResultHeader, true );
                break;

            case 3: //Right
                AppCtrl::WriteToFile( "RIGHT_AlgoStateResult.csv", m_AlgoStateStream, m_AlgoStateHeader, true );
                AppCtrl::WriteToFile( "RIGHT_CalibrationResult.csv", m_ResultStream, m_ResultHeader, true );
                break;

            default:
                AppCtrl::WriteToFile( "AlgoStateResult.csv", m_AlgoStateStream, m_AlgoStateHeader, true );
                AppCtrl::WriteToFile( "CalibrationResult.csv", m_ResultStream, m_ResultHeader, true );
                break;
        }
    }

} /* namespace oc */

