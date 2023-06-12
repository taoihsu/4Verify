//--------------------------------------------------------------------------
/// @file JobOC.h
/// @brief Contains the definition of OC = Online Calibration
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

#ifndef JOBOC_H
#define JOBOC_H

#include "core/MeclTypes.h"
#include "tscApi.h"
#include "IDataProviderOC.h"
#include "JobBaseAlgo.h"
#include "../OC/Configuration/OCCfg.h"
#include "OCData.h"
#include <string>

//"E:\work\AppCtrl_32b_Mazda7GL_GO\OC\Configuration\OCCfg.h"

//#define OC_HARDCODED_INPUT_IMAGE

class DataCollectAgent;

namespace oc
{
#ifdef OC_HARDCODED_INPUT_IMAGE
extern const uint8_t OcSyntheticImageFront_au8 [400U*640U];
#endif

    class JobOC : public container::JobBaseAlgo< IDataProviderOC >
{
public:
  enum State_e
  {
    e_UnInit = 0,
    e_OcInit,
    e_OcReadyToExecute,
    e_DoNothing
  };
  JobOC( IDataProviderOC& i_dataProvider_ro );
  virtual ~JobOC();

  void init_v( void ) override;
  bool_t hasNext_b( void ) const override;
  void start_v( void ) override;
  void execute_v( void ) override;
  void end_v( void ) override;
  void dump_v( void ) override;

  void reset_v( void ) override;
private:
        bool Logging_Enabled;
        std::stringstream m_AlgoStateHeader;
        std::stringstream m_AlgoStateStream;
        std::stringstream m_ResultHeader;
        std::stringstream m_ResultStream;

        ocdata::OcDriverPosition_e getDriverPosition_e( void ) const;
        const uint8_t* get640x400AlgoView_pu8( ocdata::CameraId_e CamID ) const;
        //void configureFpgaForAlgoView_v( bool_t i_EnableAlgoView_b );
        static tscApi::enuCameraID getOcAlgoCamId_e( ocdata::CameraId_e i_McuCamId_e );
        static Camera_ID getWindowsCamID( ocdata::CameraId_e OCCamID );
  void setOcKinematicModelConfiguration_v(void);
  void setOcCamModelConfiguration_v(void);
  void setOcFeatureCollectionConfiguration_v(void);

        // need to refactor for Windows
        void setCamParametersOneCam_v( ocdata::CameraId_e i_McuCamId_e );

        // Dump image pointer to bmp
        int vid_save_bmp( char* input_filename, char* img, int h, int w );

        // Dump image pointer to txt
        void vid_save_txt( const char* input_filename, char* img, int h, int w );

        // Dump Calibration results
        void DumpCalibrationResults( tscApi::enuCameraID v_OcAlgoCamId_e, ocdata::OcData_s& v_OcData_rs );

        // ignored for now
        //void createDebugView_v( const uint8_t* i_AlgoView_pu8, const uint8_t i_OcAlgoCamId_u8 );
        //void deactivateDebugView_v() const;

  State_e state_e;
  uint32_t ocFrameCounter_u32;
  bool_t isAlgorithmStarted_b;
        bool_t isAlgorithmFinished_b= false;
  bool_t isNewCommand_b;
        ocdata::AlgoCommand_e algoCommand_e;
        const mecl::model::Pinhole<float32_t>::Extrinsic_s* c_OcExtrinsics_ps;

        ocdata::OcAlgoView640x400Buffer_e algoViewBufferToRead_e;
        uint32_t algoViewConfiguredCounter_u32;
  uint32_t prevValidFeatureCount_u32;
  uint32_t abortFrameCounter_u32;
        float32_t trimMeanPercentage_f32;
        /* prjcontainer::AdapterCameraReal adapterCameraRealFront_o;
         prjcontainer::AdapterCameraReal adapterCameraRealLeft_o;
         prjcontainer::AdapterCameraReal adapterCameraRealRear_o;
         prjcontainer::AdapterCameraReal adapterCameraRealRight_o;*/
        // This may not be a proper analog, we'll see
        do_CameraParam& m_aFrontCameraInfo;
        do_CameraParam& m_aLeftCameraInfo;
        do_CameraParam& m_aRearCameraInfo;
        do_CameraParam& m_aRightCameraInfo;
        ocdata::CameraId_e mcuCamIdToOcAlgo_e;

  tscApi::TSCCtrlInfo tscCtrlInfo_o;
  tscApi::TSCSavedDataInfo_s  tscIntermediateData_as[tscApi::e_TscNumCam];
  tscApi::tscPlatformExtConfigType tscStartConfiguration_as[tscApi::e_TscNumCam];
  tscApi::TSCError_e tscErrorCode_ae[tscApi::e_TscNumCam];
  tscApi::TSCState_e tscAlgoState_ae[tscApi::e_TscNumCam];
  tscApi::CalibrationParams_s tscCalibrationResults_as[tscApi::e_TscNumCam];

  static const uint32_t c_ImageWidth_u32  = 640U;
  static const uint32_t c_ImageHeight_u32 = 400U;
  static const uint32_t c_TotalPixels_u32 = c_ImageWidth_u32*c_ImageHeight_u32;
  static const uint32_t c_MinAbortFrameCntLimit_u32 = 2700U; //90 seconds, 30 Hz => 90*30 = 2700
  static const uint32_t c_FrameWindow_u32 = 600U; //In 600 consecutive frames, if 'minValidFeaturesInWindow_u32' valid features are not found, we abort.
  static const uint32_t c_MinValidFeaturesInWindow_u32 = ( c_FrameWindow_u32 / 60U );


  static const uint8_t c_DriverPositionConfigBitMask_u8 = 0x01U;
};

} //namespace oc

#endif /* JOBOC_H_ */
