//--------------------------------------------------------------------------
/// @file DataProviderVpcs.cpp
/// @brief Contains
///
/// DataProvider is an interface between Vpcs module and the shared memory.
/// This file contains all the implementation required to access input and
/// output data.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Roman Berngardt(Roman.Berngardt@magna.com)
///         Cameron Taylor
//  --------------------------------------------------------------------------


#include "DataProviderVpcs.h"
#include "./src/Vpcs_types.h"
//#include "World2ImageTypes.h"
#include "Config.h"
#include "SensorCfg.h"
//#define DBG_PRINT 1
#include <stdio.h>

#define AUTOMATE
#define REGULAR
//#define AH
//#define Sailauf
//#define FiveDoor
//#define CHINA
////#define THREEDNA2500
//#define BRAMPTON
//#define B0926
//#define BB0927
//#define CLUTTER_AH
//#define CLUTTER_AH_ZERO
//#define P558
//#define TEAMS1102_AH1
//#define TEAMS0107_Brampton
//#define BPT20210107
//#define AH_EOL_I9_3D_0109
//#define Pre_I8_Test_01_09_2020
//#define AH_3line_0122
//#define AH_WheelBase175_0210
//#define AH_WheelBase160_0210
//#define TEAMS1102_AH

//#define CLUTTER_AH_ONE
//#define AH_THIRTYSEPT
//#define AH_SECONDOCT
//#define OLD_ARM
//#define DSC
//#define FIVEDNA
//
//define Shorter_wheel

//TODO: A lot of this code is identical between DataProviderOc and DataProviderVpcs. A common base class should be introduced.

namespace vpcs
{
	const wchar_t *GetWC(const char *c);
	int ReadConfiguration(const wchar_t* szFileName);
	const wchar_t *GetWC(const char *c)
	{
		const size_t cSize = strlen(c) + 1;
		wchar_t* wc = new wchar_t[cSize]; // PRQA S 5118 // WINDOWS code
		mbstowcs(wc, c, cSize);
		return wc;
	}
	
	int ReadConfiguration(const wchar_t* szFileName)
	{
		char_t buffer[1024];
		char_t szSymbol[256];
		char_t szEqual[256];
		char_t szValue[256];

		FILE *in = NULL;

		errno_t v_Result_t = _wfopen_s(&in, szFileName, L"rt");
		if (v_Result_t == 0)
		{
			memset(&v_IntrinsicCfg_s, 0, sizeof(v_IntrinsicCfg_s));
			memset(&v_ExtrinsicCfg_s, 0, sizeof(v_ExtrinsicCfg_s));
			while (!feof(in))
			{
				if (!fgets(buffer, sizeof(buffer), in))
				{
					break;
				}
				int erg = sscanf(buffer, "%s%s%s", szSymbol, szEqual, szValue);
				printf("%s %s %sF\n", szSymbol, szEqual, szValue);
				//fprintf(fp0,"%s %s %sF;\n", szSymbol, szEqual, szValue);
				if (erg >= 3)
				{
					trim(szSymbol);
					trim(szValue);
					for (int i = 0; i < NSymbols; i++)
					{
						if (!strcmp(szSymbol, c_ReadInfo_s[i].szField))
						{
							ScanAttribute(szValue, c_ReadInfo_s[i]);
							break;
						}
					}
				}
			}
			fclose(in);
		}
		return 0;
	}
DataProviderVpcs::DataProviderVpcs()
: IDataProviderVpcs()
{
}

DataProviderVpcs::~DataProviderVpcs()
{
  // nothing to do here
}

void DataProviderVpcs::clearVpcsDataToMcu_v() {
#ifdef PIKEOS
		memset(&getOutData().vpcsDataToMcu_s, 0, sizeof(shmdata::VpcsDataToMcu_s));
#elif defined(WINDOWS)
	memset(&wfmd, 0, sizeof(windowsFakeMcuData));
	
#endif	
	return;
};

void DataProviderVpcs::setMcuCameraId_v(uint8_t i_McuCameraId_u8)
{
#ifdef PIKEOS
	getOutData().mcuCameraId_u8 = i_McuCameraId_u8;
#elif defined(WINDOWS)
	wfmd.calibData.camId_e = (E_CameraId_t)i_McuCameraId_u8;
#endif
}

#ifdef PIKEOS
CameraId_t DataProviderVpcs::getRequestedCameraId_e()
{
  return static_cast<CameraId_t>(dataInProvider_ro.getDataMcu()->algoControl_s.MCUCameraID_u8);
}
#endif

E_CameraId_t DataProviderVpcs::getVpcsAlgoCameraId_e()
{
#ifdef PIKEOS
  CameraId_t v_RequestedCameraId_t = getRequestedCameraId_e();
  this->setMcuCameraId_v(static_cast<uint8_t>(v_RequestedCameraId_t));
  E_CameraId_t v_Result_e = e_FrontCamAlgo;
  switch (v_RequestedCameraId_t)
  {
    case prjcontainer::e_Front: { v_Result_e = e_FrontCamAlgo; break; }
    case prjcontainer::e_Left:  { v_Result_e = e_LeftCamAlgo;  break; }
    case prjcontainer::e_Rear:  { v_Result_e = e_RearCamAlgo;  break; }
    case prjcontainer::e_Right: { v_Result_e = e_RightCamAlgo; break; }
    default: { break; }
  }
  return v_Result_e;
#elif defined(WINDOWS)
	return wfmd.calibData.camId_e;
#endif
}

#ifdef PIKEOS
uint8_t DataProviderVpcs::getCameraConnectorIdFromMcuId_u8(const CameraId_t& i_CameraId_rt)
{
  return prjcontainer::AdapterCameraReal::getCameraConnectorIdFromMcuId_u8(i_CameraId_rt, dataInProvider_ro);
}

const mecl::Pinhole_t::Extrinsic_s* DataProviderVpcs::accessCameraExtrinsicDesign_ps(uint32_t i_McuCamId_u32) const
{
  prjcontainer::AdapterCameraReal v_AdapterCameraReal_o(dataInProvider_ro);
  return v_AdapterCameraReal_o.accessCameraExtrinsic_ps(static_cast<prjcontainer::CameraId_e>(i_McuCamId_u32), prjcontainer::AdapterCameraReal::e_ExtrDesign);
}


void DataProviderVpcs::configureFpgaForInputImage_v(const bool_t& i_EnableAlgoView_rb, const CameraId_t& i_McuCamId_rt)
{
  if(true == i_EnableAlgoView_rb)
  {
    getOutData().requestedFullResolutionCameraConnectorID_u8 = getCameraConnectorIdFromMcuId_u8(i_McuCamId_rt);
  }
}
#endif

mecl::model::Camera<float32_t>& DataProviderVpcs::getCameraInfo_rt(const E_CameraId_t& i_CameraId_rt, char *dirname)
{

#ifdef USER_REAL_CAMERA_PICTURES
    return adapterCameraReal_o.accessCamera_rt(i_CameraId_t, prjcontainer::AdapterCameraReal::e_ExtrDesign, prjcontainer::AdapterCameraReal::e_IntrCurrent, prjcontainer::AdapterCameraReal::e_LensCurrent, true);
#else
   /*uint32_t  IntrWidth = 1200;
   uint32_t  IntrHeight = 800;*/
   v_IntrinsicCfg_s.IntrWidth = static_cast<float32_t>(vpcs::c_ImageWidth_u32);
   v_IntrinsicCfg_s.IntrHeight = static_cast<float32_t>(vpcs::c_ImageHeight_u32);

   v_IntrinsicCfg_s.IntrPpX  = 0.0F;
   v_IntrinsicCfg_s.IntrPpY =  0.0F;
   v_IntrinsicCfg_s.IntrPolyImage2World_0 = 0.0F;
   v_IntrinsicCfg_s.IntrPolyImage2World_1 = 0.0F;
   v_IntrinsicCfg_s.IntrPolyImage2World_2 = 0.0F;
   v_IntrinsicCfg_s.IntrPolyImage2World_3 = 0.0F;
   v_IntrinsicCfg_s.IntrPolyImage2World_4 = 0.0F;
   v_IntrinsicCfg_s.IntrPolyImage2World_5 = 0.0F;
   v_IntrinsicCfg_s.IntrPolyWorld2Image_0 = 0.0F;
   v_IntrinsicCfg_s.IntrPolyWorld2Image_1 = 0.0F;
   v_IntrinsicCfg_s.IntrPolyWorld2Image_2 = 0.0F;
   v_IntrinsicCfg_s.IntrPolyWorld2Image_3 = 0.0F;
   v_IntrinsicCfg_s.IntrPolyWorld2Image_4 = 0.0F;
   v_IntrinsicCfg_s.IntrPolyWorld2Image_5 = 0.0F;
   v_IntrinsicCfg_s.IntrPixelSizeX = 0.003f;
   v_IntrinsicCfg_s.IntrPixelSizeY = 0.003f;
   v_IntrinsicCfg_s.IntrFieldOfView = 3.490658f;
   v_IntrinsicCfg_s.IntrFocalLength = 1.0f;
   v_ExtrinsicCfg_s.Camera_X = -930.37;
   v_ExtrinsicCfg_s.Camera_Y = 0;
   v_ExtrinsicCfg_s.Camera_Z = 976.21;
   v_ExtrinsicCfg_s.Camera_Roll = 0;
   v_ExtrinsicCfg_s.Camera_Pitch = 75.07;
   v_ExtrinsicCfg_s.Camera_Yaw = 0;
   float32_t IntrPixelSizeX = 0.0F;
   float32_t IntrPixelSizeY = 0.0F;
   float32_t IntrFieldOfView = 0.0F;
   mecl::core::Point3D<float32_t>::Config_s v_Cfg_s = { 0.0f, 0.0f, 0.0f };
   mecl::model::PreRoll_e preRoll_e = mecl::model::e_PreRoll0;
   mecl::model::RotationType_e  v_rotationType_e = mecl::model::e_WorldToCamera;

   mecl::core::RotationMatrix<float32_t>::EulerAngles_s v_EulerAngles_s = { 0.0f, 0.0f, 0.0f };



   mecl::model::Camera<float32_t> Camera;
   //char filename[4][30] = { "EEPROM.txt", "EEPROM.txt", "EEPROM.txt", "EEPROM.txt" };
   char filename[250] = { "EEPROM.txt"};
   char tempFn[250];

   strcpy(tempFn, ".\\Images\\");
   strcat(tempFn, dirname);

   E_CameraId_t cam_id = i_CameraId_rt;
   
  // strcpy(filename[0], ".\\Images\\");
  //// strcat(filename[0],  dirname);
   if (cam_id == 0)
   {
	   strcat(tempFn, "\\EEPROM_Front.txt");
	   //strcpy(filename[cam_id], tempFn);
	   strcpy(filename, tempFn);
   }
   //strcpy(filename[0], ".\\Config\\EEPROM_Front.txt");
   //strcpy(filename[1], "EEPROM_Left(TOW).txt");
   //strcpy(filename[1], ".\\Images\\");
   ////strcat(filename[1], dirname);
   else if (cam_id == 1)
   {
	   strcat(tempFn, "\\EEPROM_Left.txt");
	   //strcpy(filename[cam_id], tempFn);
	   strcpy(filename, tempFn);
   }
   //strcat(filename[1], "\\EEPROM_Left.txt");
   //strcpy(filename[1], ".\\Config\\EEPROM_Left.txt");
   //strcpy(filename[2], ".\\Images\\");
   ////strcat(filename[2], dirname);
   else if (cam_id == 2)
   {
	   strcat(tempFn, "\\EEPROM_Rear.txt");
	   //strcpy(filename[cam_id], tempFn);
	   strcpy(filename, tempFn);
   }
   //strcat(filename[2], "\\EEPROM_Rear.txt");
   //strcpy(filename[2], ".\\Config\\EEPROM_Rear.txt");
   ////strcpy(filename[3], "EEPROM_Right(TOW).txt");
   //strcpy(filename[3], ".\\Images\\");
   ////strcat(filename[3], dirname);
   else if (cam_id == 3)
   {
	   strcat(tempFn, "\\EEPROM_Right.txt");
	   //strcpy(filename[cam_id], tempFn);
	   strcpy(filename, tempFn);
   }
   //strcat(filename[3], "\\EEPROM_Right.txt");
   //strcpy(filename[3], ".\\Config\\EEPROM_Right.txt");
   

   //for (uint32_t cam_id = 0; cam_id < 4; cam_id++) {
   
	   //wchar_t const* szFileName = GetWC(filename[cam_id]);
   wchar_t const* szFileName = GetWC(filename);
	   ReadConfiguration(szFileName);
	   delete szFileName; // PRQA S 5118 // WINDOWS code

   Camera = SensorCfg::Sensorcfg::Create_Camera(v_IntrinsicCfg_s, v_ExtrinsicCfg_s, cam_id);
  // const mecl::model::LensRadial<float32_t>::Config_s c_LensCfg_s =

   const mecl::model::LensRadial<float32_t>::Config_s c_lensCfg_s =
   {
      {  v_IntrinsicCfg_s.IntrPolyWorld2Image_0,
         v_IntrinsicCfg_s.IntrPolyWorld2Image_1,
         v_IntrinsicCfg_s.IntrPolyWorld2Image_2,
         v_IntrinsicCfg_s.IntrPolyWorld2Image_3,
         v_IntrinsicCfg_s.IntrPolyWorld2Image_4,
         v_IntrinsicCfg_s.IntrPolyWorld2Image_5 },

      { v_IntrinsicCfg_s.IntrPolyImage2World_0,
        v_IntrinsicCfg_s.IntrPolyImage2World_1,
        v_IntrinsicCfg_s.IntrPolyImage2World_2,
        v_IntrinsicCfg_s.IntrPolyImage2World_3,
        v_IntrinsicCfg_s.IntrPolyImage2World_4,
        v_IntrinsicCfg_s.IntrPolyImage2World_5 
		},
	   mecl::math::toDegrees_x(3.490658e+00 / 2.0F)
      //mecl::math::toDegrees_x(IntrFieldOfView / 2.0F)
   };

   // Lens
  // static mecl::model::LensRadial<float32_t> c_Lens_as;
   //mecl::model::LensRadial<float32_t> c_Lens(c_lensCfg_s);
   static mecl::model::LensRadial<float32_t> c_mLens_as[4];
   c_mLens_as[cam_id].updateConfig_v(c_lensCfg_s);
   //c_Lens_as = c_Lens;
   //c_Lens_as = c_mLens_as[cam_id];
   // Sensor


// Pinhole
const mecl::core::RotationMatrix<float32_t>::EulerAngles_s c_EulerAngles_s = { mecl::math::toRadians_x(v_ExtrinsicCfg_s.Camera_Pitch), mecl::math::toRadians_x(v_ExtrinsicCfg_s.Camera_Yaw), mecl::math::toRadians_x(v_ExtrinsicCfg_s.Camera_Roll) };
const mecl::core::Point3D<float32_t>::Config_s c_Cfg_s = { { v_ExtrinsicCfg_s.Camera_X, v_ExtrinsicCfg_s.Camera_Y, v_ExtrinsicCfg_s.Camera_Z } };
// preroll: preroll90?? i_IntrinsicCfg_rs.preroll does not exist!!
// Use cameraID instead:

mecl::model::PreRoll_e camera = mecl::model::e_PreRoll90;
if (cam_id == 0) { // front
	camera = mecl::model::e_PreRoll90;
}
else if (cam_id == 1) { // left
	camera = mecl::model::e_PreRoll180;
}
else if (cam_id == 2) { // rear
	camera = mecl::model::e_PreRoll270;
}
else if (cam_id == 3) { // right
	camera = mecl::model::e_PreRoll0;
}
else
{
	//Do Nothing
}
static mecl::model::Pinhole<float32_t> c_Pinhole_s;
static mecl::model::Sensor<float32_t> c_Sensor_s; 
const mecl::model::Pinhole<float32_t>::Extrinsic_s c_Extrinsic_s = { c_Cfg_s, c_EulerAngles_s, camera, mecl::model::e_WorldToCamera };
const mecl::model::Pinhole<float32_t>::Intrinsic_s c_Intrinsic_s = { v_IntrinsicCfg_s.IntrFocalLength, v_IntrinsicCfg_s.IntrFocalLength,{ { 0.0f, 0.0f } } };  // Values for K-matrix (focal length in x and y and principal point in camera coordinates) 
const mecl::model::Pinhole<float32_t>::Config_s c_PinholeCfg_s = { c_Extrinsic_s, c_Intrinsic_s };
//static mecl::model::Pinhole<float32_t> 
c_Pinhole_s.updateConfig_v(c_PinholeCfg_s);

// Sensor
const mecl::core::Point2D<float32_t>::Config_s c_PszPxCfg_s = { { v_IntrinsicCfg_s.IntrPixelSizeX, v_IntrinsicCfg_s.IntrPixelSizeY } };
const mecl::core::Point2D<float32_t>::Config_s c_PppPxCfg_s = { { v_IntrinsicCfg_s.IntrPpX, v_IntrinsicCfg_s.IntrPpY } };
const  mecl::model::Sensor<float32_t>::Config_s c_sensorCfg_s = { c_PppPxCfg_s, c_PszPxCfg_s, mecl::model::e_UpperLeft,{ { 0.0F, 0.0F } } ,{ { v_IntrinsicCfg_s.IntrWidth - 1.0F, v_IntrinsicCfg_s.IntrHeight - 1.0F } } };
c_Sensor_s.updateConfig_v(c_sensorCfg_s);


//mecl::model::Camera<float32_t> Camera(c_Pinhole_s, c_mLens_as[i_CameraId_rt], c_Sensor_s);
return mecl::model::Camera<float32_t> (c_Pinhole_s, c_mLens_as[i_CameraId_rt], c_Sensor_s);
//return camera_o;
#endif

		
		//}
}

void DataProviderVpcs::setAlgoState_v( const VpcsAlgoState_t i_VpcsAlgoState_e)
{
#ifdef PIKEOS
	getOutData().vpcsDataToMcu_s.vpcsAlgoState_e = i_VpcsAlgoState_e;
#elif defined(WINDOWS)
	wfmd.algoState = i_VpcsAlgoState_e;
#endif
	return;
}

void DataProviderVpcs::setLastUpdated_v()
{
#ifdef PIKEOS
	getOutData().updatedAt_u32 = dataInProvider_ro.getDataSystem()->psFrameId_u32;
#elif defined(WINDOWS)
	wfmd.frameID += 1;
	wfmd.LastUpdatedAt = wfmd.frameID;
#endif
}

void DataProviderVpcs::setVpcsActive(bool_t i_VpcsActive_rb)
{
#ifdef PIKEOS
	getOutData().isVpcsActive_b = i_VpcsActive_b;
#elif defined(WINDOWS)
	wfmd.vpcsActive = i_VpcsActive_rb;
#endif
}

void DataProviderVpcs::setLastRequestedAt_v(uint32_t i_LastRequestedAt_u32)
{
#ifdef PIKEOS
	getOutData().lastRequestedAt_u32 = i_LastRequestedAt_u32;
#elif defined(WINDOWS)
	wfmd.LastRequestedAt = i_LastRequestedAt_u32;
#endif
}

uint32_t DataProviderVpcs::getLastRequestedAt_u32() const
{
#ifdef PIKEOS
	return getOutData().lastRequestedAt_u32;
#elif defined(WINDOWS)
	return wfmd.LastRequestedAt;
#endif
}

uint32_t DataProviderVpcs::getRequestedAt_u32() const
{
#ifdef PIKEOS
	return dataInProvider_ro.getDataMcu()->algoControl_s.cntrl_s.requestedAt_u32;
#elif defined(WINDOWS)
	return wfmd.RequestedAt;
#endif
}

ResponseState_t DataProviderVpcs::getResponseState_e()
{
	return shmdata::e_ResponsePositive; //return dataInProvider_ro.getDataMcu()->algoControl_s.cntrl_s.responseState_e;
}

void DataProviderVpcs::setVpcsErrorCode_v(const VpcsErrorCode_t i_VpcsErrorCode_e)
{
#ifdef PIKEOS
	getOutData().vpcsDataToMcu_s.vpcsErrorCode_e = i_VpcsErrorCode_e;
#elif defined(WINDOWS)
	wfmd.vpcsErrorcode = i_VpcsErrorCode_e;
	return;
#endif
}

uint8_t DataProviderVpcs::getAlgoCommand_u8()
{
#ifdef PIKEOS
	return dataInProvider_ro.getDataMcu()->algoControl_s.AlgoCommand_u8;
#elif defined(WINDOWS)
	return (uint8_t)wfmd.algoCommand;
#endif
}

uint8_t DataProviderVpcs::getAlgoType_u8()
{
#ifdef PIKEOS
	return dataInProvider_ro.getDataMcu()->algoControl_s.AlgoType_u8;
#else
	return 0;
#endif
}

uint32_t DataProviderVpcs::getFrameId_u32()
{
#ifdef PIKEOS
	return dataInProvider_ro.getDataSystem()->psFrameId_u32;
#elif defined(WINDOWS)
	return wfmd.frameID;  
#endif
}

VpcsConfig_s DataProviderVpcs::getConfig_s() {
	VpcsConfig_s i_vpcsConfig = { c_LineLength_u32, c_ModelSize_u32 };
	 return i_vpcsConfig;

};

void DataProviderVpcs::setDeltaPitch_v( float32_t i_DeltaPitch_f32 )
{
#ifdef PIKEOS
    getOutData().vpcsDataToMcu_s.deltaPitch_f32 = i_DeltaPitch_f32;
#elif defined(WINDOWS)
	wfmd.calibData.camPitch_f32 = i_DeltaPitch_f32;
#endif
}

void DataProviderVpcs::setDeltaRoll_v( float32_t i_DeltaRoll_f32 )
{
#ifdef PIKEOS
    getOutData().vpcsDataToMcu_s.deltaRoll_f32 = i_DeltaRoll_f32;
#elif defined(WINDOWS)
	wfmd.calibData.camRoll_f32 = i_DeltaRoll_f32;
#endif
}

void DataProviderVpcs::setDeltaYaw_v( float32_t i_DeltaYaw_f32 )
{
#ifdef PIKEOS
    getOutData().vpcsDataToMcu_s.deltaYaw_f32 = i_DeltaYaw_f32;
#elif defined(WINDOWS)
	wfmd.calibData.camYaw_f32 = i_DeltaYaw_f32;
#endif
}

void DataProviderVpcs::setDeltaX_v( float32_t i_DeltaX_f32 )
{
#ifdef PIKEOS
    getOutData().vpcsDataToMcu_s.deltaX_f32 = i_DeltaX_f32;
#elif defined(WINDOWS)
	wfmd.calibData.camX_f32 = i_DeltaX_f32;
#endif
}

void DataProviderVpcs::setDeltaY_v( float32_t i_DeltaY_f32 )
{
#ifdef PIKEOS
    getOutData().vpcsDataToMcu_s.deltaY_f32 = i_DeltaY_f32;
#elif defined(WINDOWS)
	wfmd.calibData.camY_f32 = i_DeltaY_f32;
#endif
}

void DataProviderVpcs::setDeltaZ_v( float32_t i_DeltaZ_f32 )
{
#ifdef PIKEOS
    getOutData().vpcsDataToMcu_s.deltaZ_f32 = i_DeltaZ_f32;
#elif defined(WINDOWS)
	wfmd.calibData.camZ_f32 = i_DeltaZ_f32;
#endif
}

float32_t DataProviderVpcs::getDeltaPitch_f32()
{
#ifdef PIKEOS
	return getOutData().vpcsDataToMcu_s.deltaPitch_f32;
#elif defined(WINDOWS)
	return wfmd.calibData.camPitch_f32;
#endif
}

float32_t DataProviderVpcs::getDeltaRoll_f32()
{
#ifdef PIKEOS
	return getOutData().vpcsDataToMcu_s.deltaRoll_f32;
#elif defined(WINDOWS)
	return wfmd.calibData.camRoll_f32;
#endif
}

float32_t DataProviderVpcs::getDeltaYaw_f32()
{
#ifdef PIKEOS
  return getOutData().vpcsDataToMcu_s.deltaYaw_f32;
#elif defined(WINDOWS)
	return wfmd.calibData.camYaw_f32;
#endif
}

float32_t DataProviderVpcs::getDeltaX_f32()
{
#ifdef PIKEOS
	return getOutData().vpcsDataToMcu_s.deltaX_f32;
#elif defined(WINDOWS)
	return wfmd.calibData.camX_f32;
#endif
}

float32_t DataProviderVpcs::getDeltaY_f32()
{
#ifdef PIKEOS
	return getOutData().vpcsDataToMcu_s.deltaY_f32;
#elif defined(WINDOWS)
	return wfmd.calibData.camY_f32;
#endif
}

float32_t DataProviderVpcs::getDeltaZ_f32()
{
#ifdef PIKEOS
	return getOutData().vpcsDataToMcu_s.deltaZ_f32;
#elif defined(WINDOWS)
	return wfmd.calibData.camZ_f32;
#endif
}

const uint8_t* DataProviderVpcs::getInputImage_pu8(char* dirname, FILE *fp0) const
{
#ifdef WINDOWS

	char strInputFile[250];

	
	//strcpy(folderName, "TestRun_x_05");
	if (wfmd.calibData.camId_e == e_FrontCamAlgo) {
		strcpy(strInputFile, ".\\Images\\");
		//strcpy(strInputFile, "..\\EOL_P558_VPCS\\Images\\");
		strcat(strInputFile, dirname);
#ifdef REGULAR
		strcat(strInputFile, "\\Front.raw");
#else
		strcat(strInputFile, "\\mem_dump_front_cam_dumped.raw");
#endif

		log_printf("---Front CAM---\n");
		fprintf(fp0, "---Front CAM---\n");
	}
	else if (wfmd.calibData.camId_e == e_LeftCamAlgo) {
		strcpy(strInputFile, ".\\Images\\");
		//strcpy(strInputFile, "..\\EOL_P558_VPCS\\Images\\");
		strcat(strInputFile, dirname);
#ifdef REGULAR
		strcat(strInputFile, "\\Left.raw");
#else
		strcat(strInputFile, "\\mem_dump_left_cam_dumped.raw");
#endif

		log_printf("\n---Left CAM---\n");
		fprintf(fp0, "---Left CAM---\n");
	}
	else if (wfmd.calibData.camId_e == e_RearCamAlgo) {
		strcpy(strInputFile, ".\\Images\\");
		//strcpy(strInputFile, "..\\EOL_P558_VPCS\\Images\\");
		strcat(strInputFile, dirname);
#ifdef REGULAR
		strcat(strInputFile, "\\Rear.raw");
#else
		strcat(strInputFile, "\\mem_dump_rear_cam_dumped.raw");
#endif

		log_printf("---Rear CAM---\n");
		fprintf(fp0, "---Rear CAM---\n");
	}
	else if (wfmd.calibData.camId_e == e_RightCamAlgo) {
		strcpy(strInputFile, ".\\Images\\");
		//strcpy(strInputFile, "..\\EOL_P558_VPCS\\Images\\");
		strcat(strInputFile, dirname);
#ifdef REGULAR
		strcat(strInputFile, "\\Right.raw");
#else
		strcat(strInputFile, "\\mem_dump_right_cam_dumped.raw");
#endif

		log_printf("\n---Right CAM---\n");
		fprintf(fp0, "---Right CAM---\n");
	}


	FILE* pInput = NULL;

	pInput = fopen(strInputFile, "rb");
	if (pInput)
	{
			//read in a row of pixels
			fread((void*)imageBuffer, 1, vpcs::c_ImageSize_u32, pInput);
		
	}
	fclose(pInput);

	return imageBuffer;

#elif defined(PIKEOS)
  return dataInProvider_ro.getDataAlgM()->c_AlgoViewFullRes_pu8;
#endif
}

} // namespace Vpcs