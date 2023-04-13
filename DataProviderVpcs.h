//--------------------------------------------------------------------------
/// @file DataProviderVpcs.h
/// @brief Contains
///
/// DataProvider is an interface between Vpcs module and the shared memory.
/// It implements the pure virtual interface definition to adapt the
/// module/component to the framework by specifying where the data comes form
/// or goes to. This means, this file contains all the interfaces required
/// to access input and output data.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Cameron
///
//  --------------------------------------------------------------------------
#ifndef DATAPROVIDERVpcs_H_
#define DATAPROVIDERVpcs_H_

#include "IDataProviderVpcs.h"
#include "Config.h"
//#define PREWARPED
//needed for file Loading on Windows
#ifdef WINDOWS
#include <opencv2/opencv.hpp>
#elif defined(PIKEOS)
#include "prjcontainer/IDataProvider.h"
#include "shmdata/VpcsSegment.h"
#include "VpcsTypes.h"
#include <stdio.h>
#endif

namespace vpcs
{
  class DataProviderVpcs : public IDataProviderVpcs
  {
  public:
    explicit DataProviderVpcs();
    virtual ~DataProviderVpcs();

    virtual void clearVpcsDataToMcu_v();
    virtual void setMcuCameraId_v(uint8_t i_McuCameraId_u8);
#ifdef PIKEOS
    virtual CameraId_t getRequestedCameraId_e();
#endif
    virtual E_CameraId_t getVpcsAlgoCameraId_e();
    virtual void setLastUpdated_v();
    virtual void setVpcsActive(bool_t i_VpcsActive_rb);
    virtual void setLastRequestedAt_v(uint32_t i_LastRequestedAt_u32);
    virtual void setVpcsErrorCode_v(const VpcsErrorCode_t i_VpcsErrorCode_e);
    virtual uint32_t getLastRequestedAt_u32() const;
    virtual uint8_t getAlgoCommand_u8();
    virtual uint8_t getAlgoType_u8();
    virtual uint32_t getRequestedAt_u32() const;

#ifdef PIKEOS
    virtual uint8_t getCameraConnectorIdFromMcuId_u8(const CameraId_t& i_CameraId_rt);
    virtual const mecl::Pinhole_t::Extrinsic_s* accessCameraExtrinsicDesign_ps(uint32_t i_McuCamId_u32) const;
    virtual void configureFpgaForInputImage_v(const bool_t& i_EnableAlgoView_rb, const CameraId_t& i_McuCamId_rt);
#endif
    virtual mecl::model::Camera<float32_t>& getCameraInfo_rt(const E_CameraId_t& i_CameraId_rt, char *dirname);

    virtual ResponseState_t getResponseState_e();

    virtual uint32_t getFrameId_u32();
	
#ifdef PIKEOS
    virtual uint8_t* accessVpcsSrTemplate_pu8();
    virtual uint8_t* accessVpcsSlTemplate_pu8();
    virtual uint8_t* accessVpcsRlsTemplate_pu8();
#endif

    virtual const uint8_t* getInputImage_pu8(char *dirname, FILE *fp0) const;

    virtual void setAlgoState_v( const VpcsAlgoState_t i_VpcsAlgoState_e);
    virtual VpcsConfig_s getConfig_s();
    virtual void setDeltaPitch_v( float32_t i_DeltaPitch_f32 );
    virtual void setDeltaRoll_v( float32_t i_DeltaRoll_f32 );
    virtual void setDeltaYaw_v( float32_t i_DeltaYaw_f32 );
    virtual void setDeltaX_v( float32_t i_DeltaX_f32 );
    virtual void setDeltaY_v( float32_t i_DeltaY_f32 );
    virtual void setDeltaZ_v( float32_t i_DeltaZ_f32 );
    virtual float32_t getDeltaPitch_f32();
    virtual float32_t getDeltaRoll_f32();
    virtual float32_t getDeltaYaw_f32();
    virtual float32_t getDeltaX_f32();
    virtual float32_t getDeltaY_f32();
    virtual float32_t getDeltaZ_f32();


	

	//datastructure to monitor what's being sent to the MCU on windows
#ifdef WINDOWS
  struct windowsFakeMcuData {
    uint32_t frameID;

    uint32_t LastRequestedAt;
    uint32_t RequestedAt;
    uint32_t LastUpdatedAt;
    ResponseState_t responseState;
    bool_t vpcsActive;
    VpcsErrorCode_t vpcsErrorcode;

    aParaMgr_S_Calibration_t calibData;
    AlgoCommand_t algoCommand;
    VpcsAlgoState_t algoState;
};
    windowsFakeMcuData wfmd;
#endif


  private:

    uint8_t imageBuffer[c_ImageWidth_u32*c_ImageHeight_u32];

  };

} // namespace Vpcs

#endif // DATAPROVIDERVpcs_H_

