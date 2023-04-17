//--------------------------------------------------------------------------
/// @file JobAlgoVpcs.cpp
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
/// @author Cameron Taylor 
/// @modified by David Hsu 04/01/2020
//  --------------------------------------------------------------------------
#include "JobAlgoVpcs.h"
#include "mecl/core/RotationMatrix.h"
#include<ctime>
#include "string.h"
#include <iostream>
//#define HIST_EQ
#define EXECLUSION

//#define INITIALROI
#ifdef VPCS3D
#include "MD_VarStore.h"
#endif

//#define USE_LOCAL_CAMERA_PICTURES
#define DBG_LOG_RESULTS


namespace vpcs
{
	static void Initial_ROI(boolean_T b[c_ImageSize_u32], E_CameraId_t cameraID_e, mecl::Recti myROI)
	{
		memset(b, 1, c_ImageSize_u32);
		int Ax, Ay, Bx, By;
		int A, B, C, dir;
		int Bot_front = 600 * vpcs::c_ImageWidth_u32;
		int Bot_rear = 700 * vpcs::c_ImageWidth_u32;
		for (int r = 0; r < vpcs::c_ImageSize_u32; r++) {

			int height = vpcs::c_ImageWidth_u32;
			int front_row = 360;


			int right_left_botrow = 600;
			int Px = r % vpcs::c_ImageWidth_u32;
			int Py = r / vpcs::c_ImageWidth_u32;
			switch (cameraID_e)
			{
			case 0:
			{
				//int rear_row = 390;
				bool rowBelow = (r > ((422) * vpcs::c_ImageWidth_u32) - 1);
				//if (rowBelow && !((fabs(Px - 640.0F) < 150.0F) && (Py > rear_row + 100)))
				if (rowBelow)
					b[r] = 1;
				else
					b[r] = 0;

				Ax = 170;
				Ay = 430;
				Bx = 15;
				By = 515;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool left_rowBelow = (dir < 0);
				if ((left_rowBelow))
					b[r] = 0;

				Ax = 321;
				Ay = 430;
				Bx = 220;
				By = 515;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);

				rowBelow = (dir > 0);
				/*if ((rowBelow) )
				b[r] = 0;*/


				Ax = 940;
				Ay = 430;
				Bx = 1022;
				By = 515;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool rowAbove = (dir > 0);


				if ((rowBelow) && (rowAbove))
					b[r] = 0;

				Ax = 1088;
				Ay = 430;
				Bx = 1225;
				By = 515;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool right_rowBelow = (dir < 0);
				if ((right_rowBelow))
					b[r] = 0;
				if (r > Bot_front)
					b[r] = 0;
			}
			break;
			case 1:
			{
				Ax = 55;
				Ay = 360;
				Bx = 45;
				By = 570;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool left_rowBelow = (dir < 0);
				if ((left_rowBelow))
					b[r] = 0;
				//bool rowBelow = (r > ((right_left_toprow - 1)*height) - 1);
				Ax = 1130;
				Ay = 355;
				Bx = 55;
				By = 360;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);

				bool rowBelow = (dir < 0);
				if ((rowBelow))
					b[r] = 0;

				Ax = 1130;
				Ay = 355;
				Bx = 915;
				By = 550;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool rowAbove = (dir > 0);


				if ((rowBelow) && (rowAbove))
					b[r] = 0;
				
				Ax = 915;
				Ay = 550;
				Bx = 45;
				By = 570;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);

				bool bot_rowBelow = (dir > 0);
				if ((bot_rowBelow))
					b[r] = 0;
				/*bool rowAbove = (r < ((620 - 1)*1280) - 1);
				if ( (rowAbove))
				b[r] = 1;
				else
				b[r] = 0;*/

			}
			break;
			case 2:
			{
				//int rear_row = 330;
				bool rowBelow = (r > ((360) * vpcs::c_ImageWidth_u32) - 1);
				//if (rowBelow && !((fabs(Px - 640.0F) < 150.0F) && (Py > rear_row + 100)))
				if (rowBelow)
					b[r] = 1;
				else
					b[r] = 0;

				Ax = 260;
				Ay = 365;
				Bx = 30;
				By = 515;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool left_rowBelow = (dir < 0);
				if ((left_rowBelow))
					b[r] = 0;

				Ax = 400;
				Ay = 365;
				Bx = 257;
				By = 515;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);

				rowBelow = (dir > 0);
				/*if ((rowBelow) )
				b[r] = 0;*/


				
				Ax = 872;
				Ay = 365;
				
				Bx = 1017;
				By = 515;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool rowAbove = (dir > 0);


				if ((rowBelow) && (rowAbove))
					b[r] = 0;

				Ax = 1021;
				Ay = 365;
				Bx = 1247;
				By = 515;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool right_rowBelow = (dir < 0);
				if ((right_rowBelow))
					b[r] = 0;

				if (r > Bot_rear)
					b[r] = 0;
			}

			break;
			case 3:
			{
				Ax = 55;
				Ay = 357;
				Bx = 1237;
				By = 360;
				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool left_rowBelow = (dir < 0);
				if ((left_rowBelow))
					b[r] = 0;
				Ax = 1130;
				Ay = 360;
				Bx = 55;
				By = 357;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);

				bool rowBelow = (dir < 0);
				if ((rowBelow))
					b[r] = 0;

				Ax = 1130;
				Ay = 355;
				Bx = 915;
				By = 555;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool rowAbove = (dir > 0);


				if ((rowBelow) && (rowAbove))
					b[r] = 0;

				Ax = 915;
				Ay = 555;
				Bx = 45;
				By = 555;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);

				bool bot_rowBelow = (dir > 0);
				if ((bot_rowBelow))
					b[r] = 0;
			}
			break;
			}
		}
	}


#ifdef PIKEOS
JobAlgoVpcs::JobAlgoVpcs(IDataProviderVpcs& b_DPVpcs_ro,
                         logging::ILogSender& b_LogSender_ro)
    : dataProvider_ro(b_DPVpcs_ro),
      intrPpx_f32(0.0F),
      intrPpy_f32(0.0F),
      cameraID_t(e_FrontCamAlgo),
      log_ro(b_LogSender_ro),
      jobAlgoVpcsState_e(e_VpcsJobAlgoUnInitialized)
{

}
#else
JobAlgoVpcs::JobAlgoVpcs(IDataProviderVpcs& b_DPVpcs_ro)
	: dataProvider_ro(b_DPVpcs_ro),
	intrPpx_f32(0.0F),
	intrPpy_f32(0.0F),
	cameraID_t(e_FrontCamAlgo),
	isImageFlipped_b(false),
	jobAlgoVpcsState_e(e_VpcsJobAlgoUnInitialized)
{

}
#endif
JobAlgoVpcs::~JobAlgoVpcs()
{
}


/************************************************************************** Init/Start BEGIN **********************************************************************/
void JobAlgoVpcs::init_v()
{
  mConfig_s = dataProvider_ro.getConfig_s();

#if 0
  sr = dataProvider_ro.accessVpcsSrTemplate_pu8();
  sl = dataProvider_ro.accessVpcsSlTemplate_pu8();
  RLs = dataProvider_ro.accessVpcsRlsTemplate_pu8();
#else
  
#ifdef DBG_PRINT
  vm_cprintf("VPCS_DBG: Template data is initialized! \r\n");
#endif
#endif
}

bool_t JobAlgoVpcs::hasNext_b() const
{
	bool_t v_Ret_b = false;
	if (e_VpcsJobAlgoJobFinished != jobAlgoVpcsState_e)
	{
		v_Ret_b = true;
	}
	return v_Ret_b;
}

void JobAlgoVpcs::start_v(const uint8_t* i_InputImage_pu8,
		uint8_t* b_WarpedImage_pu8, char *dirnmae, FILE *fp0)
{
	v_FLAG_t = shmdata::e_EOL_CALIBRATION_IN_PROCESS;
	c_InImage_pu8 = i_InputImage_pu8;
	cameraID_t = dataProvider_ro.getVpcsAlgoCameraId_e();
	mecl::Recti roi;
	if (cameraID_t == e_FrontCamAlgo)
	{
		
		roi.x = 70; roi.y = 355;
		roi.width = 1100; roi.height = 260;
	}
	if (cameraID_t == e_LeftCamAlgo)
	{
		//front_rear = 0;
		roi.x = 300; roi.y = 215;
		roi.width = 980; roi.height = 420;
	}
	if (cameraID_t == e_RearCamAlgo)
	{
		//front_rear = 1;
		roi.x = 140; roi.y = 370;
		roi.width = 960; roi.height = 310;
	}
	if (cameraID_t == e_RightCamAlgo)
	{
		//front_rear = 0;
		roi.x = 300; roi.y = 215;
		roi.width = 980; roi.height = 360;
	}
#ifdef OPENCV_OUT	
	//#if 0

	cv::Mat Binariz(vpcs::c_ImageHeight_u32, vpcs::c_ImageWidth_u32, CV_8UC1);
#endif
//#ifdef INITIALROI
#if 0
	boolean_T Initial_Mask[vpcs::vpcs::c_ImageSize_u32];
	Initial_ROI(Initial_Mask, cameraID_t, roi);

#ifdef OPENCV_OUT
	for (int row = 0; row < vpcs::vpcs::c_ImageHeight_u32; row++)
	{
		for (int col = 0; col < vpcs::vpcs::c_ImageWidth_u32; col++)

		{
			
			Binariz.at<uint8_t>(row, col) = Initial_Mask[col + row * vpcs::c_ImageWidth_u32] * 255;
		}
	}
	cv::imshow("MaskOUT", Binariz);
	cv::waitKey(0);
#endif
#endif
//#ifdef BINARIZ
#if 0
	float32_t threshold = 100;
	uint8_t c_MY_InImage_pu8[c_ImageSize_u32];// image pointer
	memcpy(c_MY_InImage_pu8, i_InputImage_pu8, c_ImageSize_u32);
	for (int row = 0; row < vpcs::c_ImageHeight_u32; row++)
	{
		for (int col = 0; col < vpcs::c_ImageWidth_u32; col++)

		{
			//if ((col > roi.x) && (col < (roi.x + roi.width)) &&
			//	(row > roi.y) && (row < (roi.y + roi.height)))
			if (Mask_output[col + row * vpcs::c_ImageWidth_u32] == 1)
				c_MY_InImage_pu8[col + row * vpcs::c_ImageWidth_u32] = i_InputImage_pu8[col + row * vpcs::c_ImageWidth_u32];
			else
				c_MY_InImage_pu8[col + row * vpcs::c_ImageWidth_u32] = 0;

			int value = i_InputImage_pu8[col + row * vpcs::c_ImageWidth_u32];
			if (value < (threshold*0.56)
				//if (value < (threshold*0.8)
				&&
				(Mask_output[col + row * vpcs::c_ImageWidth_u32] == 1))
				
				/*((col > roi.x) && (col < (roi.x + roi.width)) &&
				(row > roi.y) && (row < (roi.y + roi.height))))*/
				c_MY_InImage_pu8[col + row * vpcs::c_ImageWidth_u32] = 0;
				
			else
				c_MY_InImage_pu8[col + row * vpcs::c_ImageWidth_u32] = c_MY_InImage_pu8[col + row * vpcs::c_ImageWidth_u32]*0.5;
			
#ifdef OPENCV_OUT
			Binariz.at<uint8_t>(row, col) = c_MY_InImage_pu8[col + row * vpcs::c_ImageWidth_u32];
#endif
		}
	}
//#ifdef OPENCV_OUT
#if 1
	cv::imshow("Binariz", Binariz);
	cv::waitKey(0);
#endif
		
#endif	
	mCameraObject_o = dataProvider_ro.getCameraInfo_rt(cameraID_t, dirnmae);
#ifndef PIKEOS
	mecl::model::Pinhole<float32_t> &v_PinHole = dynamic_cast<mecl::model::Pinhole<float32_t>&>(mCameraObject_o.getImager_rx());
	const mecl::model::Pinhole<float32_t>::Extrinsic_s c_CamExtrinsic_ps = v_PinHole.getExtrinsic_rs();
	float32_t c_CamExtrRoll_f32 =
	c_CamExtrinsic_ps.eulerAngles_s.roll_x;
#endif
#ifdef PIKEOS
	const float32_t c_CamExtrRoll_f32 =
		dataProvider_ro.accessCameraExtrinsicDesign_ps(dataProvider_ro.getRequestedCameraId_e())->eulerAngles_s.roll_x;
#endif
	isImageFlipped_b = (mecl::math::abs_x(c_CamExtrRoll_f32) > 2.1F);
	//isImageFlipped_b = true;

	mecl::model::LensRadial<float32_t>::Config_s v_config_o;
#ifdef DBG_PRINT
	mecl::model::LensRadial<float32_t> *v_RadialLens_px = reinterpret_cast<mecl::model::LensRadial<float32_t> *>(&(mCameraObject_o.getLens_rx()));
	if (0 == v_RadialLens_px)
	{
		log_printf("VPCS_DBG: Camera Lens config LENS NULL: \r\n");
		fprintf(fp0,"VPCS_DBG: Camera Lens config LENS NULL: \r\n");
	}
	else
	{
		v_RadialLens_px->copyConfig_v(v_config_o);
		log_printf("VPCS_DBG: Camera Lens config: \r\n");
		vm_cprintfFloat("VPCS_DBG:    elevationMaxCfg_x = %f\r\n", v_config_o.elevationMaxCfg_x);
		vm_cprintfFloat("VPCS_DBG:    image2world_x = [%f,%f,%f,%f,%f,%f]\r\n", v_config_o.image2world_x[0], v_config_o.image2world_x[1], v_config_o.image2world_x[2], v_config_o.image2world_x[3], v_config_o.image2world_x[4], v_config_o.image2world_x[5]);
		vm_cprintfFloat("VPCS_DBG:    world2image_x = [%f,%f,%f,%f,%f,%f]\r\n", v_config_o.world2image_x[0], v_config_o.world2image_x[1], v_config_o.world2image_x[2], v_config_o.world2image_x[3], v_config_o.world2image_x[4], v_config_o.world2image_x[5]);
		fprintf(fp0, "VPCS_DBG: Camera Lens config: \r\n");
		fprintf(fp0, "VPCS_DBG:    elevationMaxCfg_x = %f\r\n", v_config_o.elevationMaxCfg_x);
		fprintf(fp0, "VPCS_DBG:    image2world_x = [%f,%f,%f,%f,%f,%f]\r\n", v_config_o.image2world_x[0], v_config_o.image2world_x[1], v_config_o.image2world_x[2], v_config_o.image2world_x[3], v_config_o.image2world_x[4], v_config_o.image2world_x[5]);
		fprintf(fp0, "VPCS_DBG:    world2image_x = [%f,%f,%f,%f,%f,%f]\r\n", v_config_o.world2image_x[0], v_config_o.world2image_x[1], v_config_o.world2image_x[2], v_config_o.world2image_x[3], v_config_o.world2image_x[4], v_config_o.world2image_x[5]);
	}
#endif

#ifdef PREWARPED
	memcpy(dstMecl_au8, c_InImage_pu8, c_ImageSize_u32);
#else
	//isImageFlipped_b = true;
	SensorCfg::SensorCfg_Image2World_u16(mCameraObject_o, &dstMecl_au8[0], c_InImage_pu8, isImageFlipped_b);
#ifdef OPENCV_OUT
#ifdef DISPLAYARRAY
	DisplayArray2D a(c_ImageWidth_u32, c_ImageHeight_u32, dstMecl_au8);
	a.display();
#endif	
	cv::Mat ncv_img(vpcs::c_ImageHeight_u32, vpcs::c_ImageWidth_u32, CV_8UC1, dstMecl_au8);
	std::string CamNum = std::to_string(cameraID_t);
	std::string ImageFile1 = "Images\\Flatten\\516W428\\Flaten" + CamNum +  ".png";
    //cv::imwrite(ImageFile1, ncv_img);
	imshow("flatten img", ncv_img);
	cv::waitKey(0);
#endif
#endif
	// Save Distortion paramters MD_MOD
	MD_Data NewData;
	MD_DataC = NewData; // Clear the old variables
	for (int i=0; i<6; i++)
	{
		MD_DataC.Dist_im2world[i] = v_config_o.image2world_x[i];
		MD_DataC.Dist_world2im[i] = v_config_o.world2image_x[i];
	}
	mecl::core::Matrix3x3<float32_t> K = mCameraObject_o.getImager_rx().getKMatrix_x(); // K matrix is different from my K matrix
	float32_t mmPerPixX = mCameraObject_o.getSensor_rx().getPsz_rx().getPosX(); // mm/pix
	float32_t mmPerPixY = mCameraObject_o.getSensor_rx().getPsz_rx().getPosY(); // mm/pix
	MD_DataC.CamFC[0] = K(0,0)/mmPerPixX; // FoCal length x
	MD_DataC.CamFC[1] = K(1,1)/mmPerPixY; // FoCal length y
	MD_DataC.CamPP[0] = mCameraObject_o.getSensor_rx().getPpp_rx().getPosX(); // Principal point x
	MD_DataC.CamPP[1] = mCameraObject_o.getSensor_rx().getPpp_rx().getPosY(); // Principal point x
	// MD_MOD PP Change start
	//if (dataProvider_ro.getVpcsAlgoCameraId_e() == e_RightCamAlgo)
		if(isImageFlipped_b)
	{
		MD_DataC.CamPP[0] = vpcs::c_ImageWidth_u32-1 - MD_DataC.CamPP[0];
		MD_DataC.CamPP[1] = vpcs::c_ImageHeight_u32-1 - MD_DataC.CamPP[1];
	}
	// MD_MOD PP Change end
	// Save Distortion paramters MD_MOD

	memcpy(b_WarpedImage_pu8, dstMecl_au8, c_ImageSize_u32);

#ifdef DBG_PRINT
	vm_cprintf("VPCS_DBG: Warped picture for CameraID=%d is saved. \r\n", cameraID_t);
	fprintf(fp0,"VPCS_DBG: Warped picture for CameraID=%d is saved. \r\n", cameraID_t);
	
#endif
	jobAlgoVpcsState_e = e_VpcsJobAlgoRunStage1;
} // End of JobAlgoVpcs::start_v()
/************************************************************************** Init/Start END **********************************************************************/


void PrintR__(mecl::core::Matrix<float32_t, 3, 3> &R)
{
	for (int i = 0; i<3; i++)
	{
		vm_cprintf("\n     ");
		for (int j = 0; j<3; j++)
		{
			vm_cprintf("%f, \n", R(i, j));
		}
	}
	vm_cprintf("\n");
}
void FILE_PrintR__(mecl::core::Matrix<float32_t, 3, 3> &R, FILE *fp0)
{
	for (int i = 0; i<3; i++)
	{
		fprintf(fp0, "\n     ");
		for (int j = 0; j<3; j++)
		{
			fprintf(fp0, "%f, \n", R(i, j));
		}
	}
	fprintf(fp0, "\n");
}





/************************************************************************** Main execution of extrinsic calibration **********************************************************************/
void JobAlgoVpcs::execute_v(char *dirname, FILE *fp0)
{
	
  switch (jobAlgoVpcsState_e)
  {
    case e_VpcsJobAlgoRunStage1:
    {
      cameraID_t = dataProvider_ro.getVpcsAlgoCameraId_e();
      intrPpx_f32 = mCameraObject_o.getSensor_rx().getPpp_rx().getPosX();
      intrPpy_f32 = mCameraObject_o.getSensor_rx().getPpp_rx().getPosY();

      if (cameraID_t == e_FrontCamAlgo)
      {
        mConfig_s.lineLength = c_LineLengthFrontRear_u32;
		MD_DataC.CamNum = 0;
      }

      if (cameraID_t == e_RearCamAlgo)
      {
        mConfig_s.lineLength = c_LineLengthFrontRear_u32;
		MD_DataC.CamNum = 2;
      }
      if (cameraID_t == e_LeftCamAlgo)
      {
        mConfig_s.lineLength = c_LineLengthLeftRight_u32;
		MD_DataC.CamNum = 1;
      }
      if (cameraID_t == e_RightCamAlgo)
      {
        mConfig_s.lineLength = c_LineLengthLeftRight_u32;
		MD_DataC.CamNum = 3;
      }
//


#ifdef OPENCV_OUT
	  //FILE *fp4;
	  //char filename4[100];
	  //sprintf(filename4, "His.txt");
	  //fp4 = fopen(filename4, "w");
	  //int k4 = 0;
	  ///*for (int i = 0; i < 256; i++)
		 // fprintf(fp4, "%d,", histogram[i]);
	  //fprintf(fp4, "\n");*/
	  //for (int i = 0; i < 800; i++)
	  //{
	  //for (int j = 0; j < 1280; j++)
	  //{
	  //fprintf(fp4, "%d,", output_image[k4]);
	  //k4++;
	  //}
	  //fprintf(fp4, "\n");
	  //}
	  //fclose(fp4);
	  IplImage * grayImgU1;
	  grayImgU1 = cvCreateImage(cvSize(c_ImageWidth_u32, c_ImageHeight_u32), IPL_DEPTH_8U, 1);
	  for (int i = 0; i < c_ImageSize_u32; i++) {
		  grayImgU1->imageData[i] = static_cast<uint8_t>(dstMecl_au8[i]);
	  }
	  
	 /* cvShowImage("his", grayImgU1);
	  cvWaitKey(1);*/
#endif

      vpExtractor_o.setCameraID(cameraID_t);
      vanpoints_s = vpExtractor_o.findVanishingPoints(&dstMecl_au8[0], mConfig_s.lineLength, mConfig_s.modelSize);

#ifdef EXECLUSION
		for (int i = 0; i < vanpoints_s.lineNum; ++i) {
			if ((cameraID_t == e_FrontCamAlgo)
					|| (cameraID_t == e_RearCamAlgo)) {
				if (vanpoints_s.clusters[1][i] == 1) {
					if (fabs(vanpoints_s.lines_as[i].theta_f32) > 0.3496) {
						vanpoints_s.clusters[1][i] = vanpoints_s.clusters[0][i];
					}
				}
			}
		}
#endif
#ifdef OPENCV_OUT
	  IplImage * grayImgU;
	  grayImgU = cvCreateImage(cvSize(c_ImageWidth_u32, c_ImageHeight_u32), IPL_DEPTH_8U, 1);
	  memcpy(grayImgU->imageData, dstMecl_au8, c_ImageWidth_u32 * c_ImageHeight_u32);
	  cv::Mat showline = cv::cvarrToMat(grayImgU);
	  cv::Mat _c3;
	  cv::cvtColor(showline, _c3, CV_GRAY2BGR);
	  IplImage cc3 = _c3;

	  for (int i = 0; i < vanpoints_s.lineNum; ++i)
	  {
		  CvScalar color;
			if(vanpoints_s.clusters[0][i]==1)																					 
			  color = CV_RGB(0, 0, 255);
			  if (vanpoints_s.clusters[1][i] == 1)
			  //if (m_clusters[1][i] == 1)
			  color = CV_RGB(0, 255, 0);
		  float32_t x1 = vanpoints_s.lines_as[i].x1_af32[0];
		  float32_t y1 = vanpoints_s.lines_as[i].x1_af32[1];
		  float32_t x2 = vanpoints_s.lines_as[i].x2_af32[0];
		  float32_t y2 = vanpoints_s.lines_as[i].x2_af32[1];
		  cvLine(&cc3, cvPoint((sint32_t)x1, (sint32_t)y1), cvPoint((sint32_t)x2, (sint32_t)y2), color, 2);
		 // float32_t theta = atan2((y2 - y1), (x2 - x1));
		  /*if (i == 5)
			  printf("i%d theta %f vanpoints_s.lines_as[i]. %f\n", i, theta, vanpoints_s.lines_as[i].theta_f32);

		  printf("^^^^^i %d theta %f vanpoints_s.clusters[0][i] %d vanpoints_s.clusters[1][i] %d\n", i, theta, vanpoints_s.clusters[0][i], vanpoints_s.clusters[1][i]);*/
	  }
#ifdef SHOW_IMAGE
	  /*cvShowImage("myROIS lines", &cc3);
	  cvWaitKey(0);*/
#endif
	  /*cv::Mat overlay = cv::cvarrToMat(&cc3);

	  cv::Mat overlay_out;
	  cv::cvtColor(Mask, overlay_out, CV_GRAY2BGR);*/

#endif
		#ifdef OPENCV_OUT
			cv::Mat ncv(vpcs::c_ImageHeight_u32, vpcs::c_ImageWidth_u32, CV_8UC1, dstMecl_au8);
  
			cv::Mat c3;
			cv::cvtColor(ncv, c3, CV_GRAY2BGR);

			IplImage iplimg = c3;
			cvSaveImage("Input.bmp", &iplimg);
			sint32_t Returned;
			Returned= vpExtractor_o.superposeLines(&iplimg, vanpoints_s.lines_as, vanpoints_s.lineNum, vanpoints_s.clusters, vanpoints_s.vanPoints_as, mConfig_s.lineLength);
			#endif

			#ifdef DBG_PRINT
			for (uint8_t i = 0; i < c_ClusterNum_u32; i++)
			{
			vm_cprintf("VDG_DBG: VP %d is: \r\n", i);
			vm_cprintfFloat("VDG_DBG:  %f %f \r\n ", vanpoints_s.vanPoints_as[i].x_f32, vanpoints_s.vanPoints_as[i].y_f32);
			fprintf(fp0, "VDG_DBG: VP %d is: \r\n", i);
			fprintf(fp0, "VDG_DBG:  %f %f \r\n ", vanpoints_s.vanPoints_as[i].x_f32, vanpoints_s.vanPoints_as[i].y_f32);

			}
		#endif


		/**************************** MD_MOD extrinsic calibration algorithm BEGIN ******************************************/
		
		// Read in info in concise format
		// Read in original camera extrinsics
		mCameraObject_o = dataProvider_ro.getCameraInfo_rt(cameraID_t, dirname);
		mecl::model::Pinhole<float32_t> &v_PinHole = dynamic_cast<mecl::model::Pinhole<float32_t>&>(mCameraObject_o.getImager_rx());
		const mecl::model::Pinhole<float32_t>::Extrinsic_s *c_CamExtrinsic_ps = &v_PinHole.getExtrinsic_rs();
v_PinHole.init_v();
mecl::core::Matrix<float32_t,3,4> v_m = v_PinHole.getProjectionMatrix_rx();
#ifdef PIKEOS
log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Projection Matrix = \n");
log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"%f %f %f %f\n",v_m(0,0),v_m(0,1),v_m(0,2),v_m(0,3) );
log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"%f %f %f %f\n",v_m(1,0),v_m(1,1),v_m(1,2),v_m(1,3) );
log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"%f %f %f %f\n",v_m(2,0),v_m(2,1),v_m(2,2),v_m(2,3) );
#endif
		MD_DataC.Orig_xyz_CamPos(0) = c_CamExtrinsic_ps->pos_x.cVal_ax[0];
		MD_DataC.Orig_xyz_CamPos(1) = c_CamExtrinsic_ps->pos_x.cVal_ax[1];
		MD_DataC.Orig_xyz_CamPos(2) = c_CamExtrinsic_ps->pos_x.cVal_ax[2];
		MD_DataC.Orig_pyr_CamPos(0) = c_CamExtrinsic_ps->eulerAngles_s.pitch_x;
		MD_DataC.Orig_pyr_CamPos(1) = c_CamExtrinsic_ps->eulerAngles_s.yaw_x;
		MD_DataC.Orig_pyr_CamPos(2) = c_CamExtrinsic_ps->eulerAngles_s.roll_x;
		// MATLAB_MATCH delete next 2 lines:
//if (isImageFlipped_b)
//{
//	MD_DataC.Orig_pyr_CamPos(2) -= M_PI;
//}
		MD_DataC.numEndPts_s32 = 0;
		MD_DataC.numLinLab_s32 = 0;
		vm_cprintf("initial: MD_DataC.numEndPts_s32 %f, MD_DataC.numLinLab_s32 %f  \n", MD_DataC.numEndPts_s32, MD_DataC.numLinLab_s32);
		fprintf(fp0,"initial: MD_DataC.numEndPts_s32 %f, MD_DataC.numLinLab_s32 %f  \n", MD_DataC.numEndPts_s32, MD_DataC.numLinLab_s32);
		// Read in lines used in vanishing points
		for (int i=0; i<vanpoints_s.lineNum; i++)
		{
			MD_DataC.Lines_EndPts[MD_DataC.numEndPts_s32][0] = vanpoints_s.lines_as[i].x1_af32[0];
			MD_DataC.Lines_EndPts[MD_DataC.numEndPts_s32][1] = vanpoints_s.lines_as[i].x2_af32[0];
			MD_DataC.Lines_EndPts[MD_DataC.numEndPts_s32][2] = vanpoints_s.lines_as[i].x1_af32[1];
			MD_DataC.Lines_EndPts[MD_DataC.numEndPts_s32][3] = vanpoints_s.lines_as[i].x2_af32[1];
			MD_DataC.numEndPts_s32++;
			if (vanpoints_s.clusters[0][i] == 1)
				MD_DataC.Lines_Lab[MD_DataC.numLinLab_s32] = 0;
			else if (vanpoints_s.clusters[1][i] == 1)
				MD_DataC.Lines_Lab[MD_DataC.numLinLab_s32] = 1;
			else if (vanpoints_s.clusters[2][i] == 1)
				MD_DataC.Lines_Lab[MD_DataC.numLinLab_s32] = 2;
			else
				MD_DataC.Lines_Lab[MD_DataC.numLinLab_s32] = -1;
			MD_DataC.numLinLab_s32++;
		}

		for (int i = 0; i < vanpoints_s.lineNum; i++)
		{
			vm_cprintf("vanpoints_s.clusters[0][i%d] %d: \n", (sint32_t)i, vanpoints_s.clusters[0][i]);
			vm_cprintf("Line %f: Label: %f, Endpoints: [x1=%f, x2=%f, y1=%f, y2=%f]\n", (float32_t)i, (float32_t)MD_DataC.Lines_Lab[i], MD_DataC.Lines_EndPts[i][0], MD_DataC.Lines_EndPts[i][1], MD_DataC.Lines_EndPts[i][2], MD_DataC.Lines_EndPts[i][3]);
			fprintf(fp0,"vanpoints_s.clusters[0][i%d] %d: \n", (sint32_t)i, vanpoints_s.clusters[0][i]);
			fprintf(fp0,"Line %f: Label: %f, Endpoints: [x1=%f, x2=%f, y1=%f, y2=%f]\n", (float32_t)i, (float32_t)MD_DataC.Lines_Lab[i], MD_DataC.Lines_EndPts[i][0], MD_DataC.Lines_EndPts[i][1], MD_DataC.Lines_EndPts[i][2], MD_DataC.Lines_EndPts[i][3]);

		}
		// Read in undistored image
		for (uint32_t i=0; i<vpcs::c_ImageSize_u32; i++)
		{
			MD_DataC.ImUD[i] = dstMecl_au8[i]; // Undistorted image
		}

		// Initialize other extrinsic rotations
		vm_cprintf("R_A_C0: BB4"); PrintR__(MD_DataC.R_A_C0);
		fprintf(fp0,"R_A_C0: BB4"); FILE_PrintR__(MD_DataC.R_A_C0,fp0);
		MD_DataC.ExtrinsicsMD_C_A();
		ExRefine ECalib;
		// Get rotation matrix from original roll, pitch, yaw estimate
		//MD_DataC.Orig_RAdjust = ECalib.pyr_to_R3(MD_DataC.Orig_pyr_CamPos);
		//MD_DataC.Orig_R_A_C = MD_DataC.R_A_C0.mmul(MD_DataC.Orig_RAdjust);
		vm_cprintf("R_A_C0: B4"); PrintR__(MD_DataC.R_A_C0);
		fprintf(fp0,"R_A_C0: B4"); FILE_PrintR__(MD_DataC.R_A_C0,fp0);
		// MATLAB_MATCH add following line:
		MD_DataC.Orig_R_A_C = ECalib.get_R_C_A(MD_DataC.Orig_pyr_CamPos, MD_DataC.CamNum, fp0);

	
		CalibTLinesC.CamNum = MD_DataC.CamNum;
		vm_cprintf("Cam %d Orig_R_A_C:", CalibTLinesC.CamNum); PrintR__(MD_DataC.Orig_R_A_C.t());
		fprintf(fp0,"Cam %d Orig_R_A_C:", CalibTLinesC.CamNum); FILE_PrintR__(MD_DataC.Orig_R_A_C.t(),fp0);
		/*vm_cprintf("Cam 1 Orig_R_A_C:"); JobPrintR__(Stitch_DataC[1].Orig_R_A_C.t());
		vm_cprintf("Cam 2 Orig_R_A_C:"); JobPrintR__(Stitch_DataC[2].Orig_R_A_C.t());
		vm_cprintf("Cam 3 Orig_R_A_C:"); JobPrintR__(Stitch_DataC[3].Orig_R_A_C.t());*/
		vm_cprintf("Cam %d Orig_RAdjust:", CalibTLinesC.CamNum);  PrintR__(MD_DataC.Orig_RAdjust);
		fprintf(fp0,"Cam %d Orig_RAdjust:", CalibTLinesC.CamNum);  FILE_PrintR__(MD_DataC.Orig_RAdjust,fp0);
		/*vm_cprintf("Cam 1 Orig_RAdjust:"); JobPrintR__(Stitch_DataC[1].Orig_RAdjust);
		vm_cprintf("Cam 2 Orig_RAdjust:"); JobPrintR__(Stitch_DataC[2].Orig_RAdjust);
		vm_cprintf("Cam 3 Orig_RAdjust:"); JobPrintR__(Stitch_DataC[3].Orig_RAdjust);*/
		vm_cprintf("R_A_C0:"); PrintR__(MD_DataC.R_A_C0);
		fprintf(fp0,"R_A_C0:"); FILE_PrintR__(MD_DataC.R_A_C0,fp0);
		// Refine extrinsic calibration: MAIN ALGORITHMS
		ECalib.VP3D_Extrinsics(MD_DataC, fp0);

		float64_t TotEr;
		mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> Matches[2];
		mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> MatchesAll[2]; // MD_MOD_OPTIM_ALL_LINES this line
		if (cameraID_t == e_FrontCamAlgo)
		{
			// MD_MOD_3LINES start
			const int SepDistSize = 4;
			float32_t SepDist[SepDistSize] = { 2000, 2250, 2400, 2500};
			//float32_t SepDist[SepDistSize] = { 2000, 2250, 2425, 2500};
			/*const int SepDistSize = 3;
			float32_t SepDist[SepDistSize] = { 2000, 2250, 2400};*/
			/*const int SepDistSize = 1;
			float32_t SepDist[SepDistSize] = { 2400 };*/
			//const int SepDistSize = 5;
			//float32_t SepDist[SepDistSize] = {2000, 2250, 2400, 2500, 2750}; // SET THIS VARIABLE TO TRY DIFFERENT SEPARATION DISTANCES
			//float32_t SepDist[SepDistSize] = { 2000, 2250, 2400, 2500, 2718 };
			//float32_t SepDist[1] = { 2718 };
			
			float64_t BestScore = 1e10;
			uint32_t sIdx = 0;
			uint32_t nLIdx = 0;
#ifdef vm_cprintf
			float64_t TotScore[SepDistSize][2]; // Just for printing error
			float64_t TotErs[SepDistSize][2]; // Just for printing error
			uint32_t nMatchs[SepDistSize][2]; // Just for printing
#endif
			for (int s = 0; s < SepDistSize; s++)
			{
				//for (int L = 0; L < 2; L++)
				int L = 0;
				{
					TargetLines CalibTLines_;
					NUM_LINES num_Lines;
					if (L == 0)
						num_Lines = NUM_LINES::TWO_LINES;
					else
						num_Lines = NUM_LINES::THREE_LINES;
					float32_t SepDist_ = SepDist[s];
					CalibTLines_.SetSepDist(SepDist_, num_Lines);
					CalibTLines_.CamNum = MD_DataC.CamNum;
					ECalib.MatchLines(MD_DataC, CalibTLines_, false, Matches, MatchesAll, fp0); // MD_MOD_OPTIM_ALL_LINES this line
					ECalib.RefineExtrinsics(MD_DataC, CalibTLines_, MatchesAll, TotEr, true, fp0); // MD_MOD_2021_03_02
					// MD_MOD_2021_03_01 start
					// Redo matching until no new lines can be added
					for (int i=0; i<4; i++)
					{
						mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> Matches_[2];
						mecl::core::Matrix<float32_t,3,1> TOrigSave = MD_DataC.Orig_xyz_CamPos;
						MD_DataC.Orig_xyz_CamPos = MD_DataC.Refined_xyz_CamPos;
						ECalib.MatchLines(MD_DataC, CalibTLines_, true, Matches_, MatchesAll, fp0);
						uint32_t nMatchOld_ = 0;
						for (int m=0; m<CalibTLines_.LinesLen[0]; m++)
						{
							if (Matches_[0][m] > -1)
								nMatchOld_++;
						}
						uint32_t nMatchNew_ = 0;
						for (int m=0; m<CalibTLines_.LinesLen[0]; m++)
						{
							if (Matches_[0][m] > -1)
								nMatchNew_++;
						}
						MD_DataC.Orig_xyz_CamPos = TOrigSave;
						if (nMatchNew_ <= nMatchOld_)
							break;
						ECalib.RefineExtrinsics(MD_DataC, CalibTLines_, MatchesAll, TotEr, true, fp0);
						nMatchOld_ = nMatchNew_;
						Matches[0].clear_v();
						Matches[1].clear_v();
						Matches[0].copy_v(Matches_[0], MD_DataC.numLinLab_s32);
						Matches[1].copy_v(Matches_[1], MD_DataC.numLinLab_s32);
					}
					// MD_MOD_2021_03_01 end
					uint32_t nMatch = 0;
					for (int m = 0; m < CalibTLines_.LinesLen[0]; m++)
					{
						if (Matches[0][m] > -1)
							nMatch++;
					}
					for (int m = 0; m < CalibTLines_.LinesLen[1]; m++)
					{
						if (Matches[1][m] > -1)
							nMatch++;
					}
					TotEr = TotEr / float64_t(nMatch); // We really want an average error per line
					float64_t Tot_Score = TotEr / std::pow(5.0, float64_t(nMatch + 1)); // Lower score is better <= MD_MOD_2021_03_01: weight # lines matches as hugely important - each extra line improves the score by 5x
					if (Tot_Score < BestScore)
					{
						BestScore = Tot_Score;
						sIdx = s;
						nLIdx = L;
					}
					TotScore[s][L] = Tot_Score;
					TotErs[s][L] = TotEr;
					nMatchs[s][L] = nMatch;
				}
			}

			NUM_LINES numLines;
			nLIdx = 0;
			if (nLIdx == 0)
				numLines = NUM_LINES::TWO_LINES;
			else
				numLines = NUM_LINES::THREE_LINES;

			predSepDis_f32 = SepDist[sIdx];

			CalibTLinesC.SetSepDist(predSepDis_f32, numLines);
			CalibTLinesC.CamNum = MD_DataC.CamNum;
#ifdef DBG_PRINT
			for (int s = 0; s < SepDistSize; s++)
			{
				vm_cprintf("2Line: %f SepDist => %f error with %f matches = %f score\n", float32_t(SepDist[s]), float32_t(TotErs[s][0]), float32_t(nMatchs[s][0]), float32_t(TotScore[s][0]));
			    fprintf(fp0, "2Line: %f SepDist => %f error with %f matches = %f score\n", float32_t(SepDist[s]), float32_t(TotErs[s][0]), float32_t(nMatchs[s][0]), float32_t(TotScore[s][0]));
		    }
			/*for (int s = 0; s < SepDistSize; s++)
			{
				vm_cprintf("3Line: %f SepDist => %f error with %f matches = %f score\n", float32_t(SepDist[s]), float32_t(TotErs[s][1]), float32_t(nMatchs[s][1]), float32_t(TotScore[s][1]));
				fprintf(fp0, "3Line: %f SepDist => %f error with %f matches = %f score\n", float32_t(SepDist[s]), float32_t(TotErs[s][1]), float32_t(nMatchs[s][1]), float32_t(TotScore[s][1]));
			}*/
#endif
			// MD_MOD_3LINES end
		}
		//CalibTLinesC.SetSepDist(2400, NUM_LINES::THREE_LINES);
		//CalibTLinesC.CamNum = MD_DataC.CamNum;

		// Main Line matching step
		ECalib.MatchLines(MD_DataC, CalibTLinesC, false, Matches, MatchesAll, fp0); // MD_MOD_OPTIM_ALL_LINES this line
		// MD_MOD_02_27 start 
		uint32_t nMatchOld = 0;
		for (int m=0; m<CalibTLinesC.LinesLen[0]; m++)
		{
			if (Matches[0][m] > -1)
				nMatchOld++;
		}
		// MD_MOD_02_27 end

		// Throw error if target is blocked
#ifdef vm_cprintf
		bool TargetBlocked = false;
#endif
		uint32_t nMatchesXLeft = 0;
		uint32_t nMatchesXRight = 0;
		uint32_t nMatchesY = 0;
		for (int m=0; m<CalibTLinesC.LinesLen[0]; m++) // xLine matches
		{
			if (Matches[0][m] > -1)
			{
				if (m < CalibTLinesC.LinesLen[0]/2)
					nMatchesXLeft++;
				else
					nMatchesXRight++;
			}
		}
		for (int m=0; m<CalibTLinesC.LinesLen[1]; m++) // yLine matches
		{
 			if (Matches[1][m] > -1)
				nMatchesY++;
		}
		switch (cameraID_t)
		{
			case e_FrontCamAlgo :				
				//if (nMatchesXLeft < 2 || nMatchesY < 0)
				if (nMatchesXLeft < 2 )
				{
					TargetBlocked = true;
					v_FLAG_t = shmdata::TARGET_NOT_FOUND_LEFT;
				}
				if (nMatchesXRight < 2 )					
				//if (nMatchesXRight < 2 || nMatchesY < 0)
				{
					TargetBlocked = true;
					v_FLAG_t = shmdata::TARGET_NOT_FOUND_RIGHT;
				}			
				break;
			case e_LeftCamAlgo :
				if (nMatchesXLeft < 2 )					
				//if (nMatchesXLeft < 2 || nMatchesY < 0)
				{
					TargetBlocked = true;
					v_FLAG_t = shmdata::TARGET_NOT_FOUND_LEFT;
				}
				break;
			case e_RearCamAlgo :
			    //if (nMatchesXLeft < 2 || nMatchesY < 0)
				if (nMatchesXLeft < 2 )					
				{
					TargetBlocked = true;
					v_FLAG_t = shmdata::TARGET_NOT_FOUND_LEFT;
				}
				//if (nMatchesXRight < 2 || nMatchesY < 0)
				if (nMatchesXRight < 2)					
				{
					TargetBlocked = true;
					v_FLAG_t = shmdata::TARGET_NOT_FOUND_RIGHT;
				}
				break;
			case e_RightCamAlgo :				
				if ( nMatchesXRight < 2)
				//if (nMatchesXRight < 2 ||  nMatchesY < 0)
				{
					TargetBlocked = true;
					v_FLAG_t = shmdata::TARGET_NOT_FOUND_RIGHT;
				}
				break;
		}
#ifdef DBG_PRINT
		fprintf(fp0, "\n camid %d  nMatchesXLeft %d nMatchesXRight %d  nMatchesY %d!\n", cameraID_t, nMatchesXLeft, nMatchesXRight, nMatchesY);
		if (TargetBlocked) 
		{
			vm_cprintf("\n\nError! Target blocked!\n\n");
			fprintf(fp0,"\n\nError! Target blocked!\n\n");
		}
#endif
		// Main calibration step
		ECalib.RefineExtrinsics(MD_DataC, CalibTLinesC, MatchesAll, TotEr, true, fp0); // MD_MOD_OPTIM_ALL_LINES this line


		// MD_MOD_02_27 start
		// Redo matching until no new lines can be added
		for (int i=0; i<4; i++)
		{
			mecl::core::Matrix<float32_t,3,1> TOrigSave = MD_DataC.Orig_xyz_CamPos;
			MD_DataC.Orig_xyz_CamPos = MD_DataC.Refined_xyz_CamPos;
			ECalib.MatchLines(MD_DataC, CalibTLinesC, true, Matches, MatchesAll, fp0); // MD_MOD_OPTIM_ALL_LINES this line
			uint32_t nMatchNew = 0;
			for (int m=0; m<CalibTLinesC.LinesLen[0]; m++)
			{
				if (Matches[0][m] > -1)
					nMatchNew++;
			}
			MD_DataC.Orig_xyz_CamPos = TOrigSave;
			if (nMatchNew <= nMatchOld)
				break;
			ECalib.RefineExtrinsics(MD_DataC, CalibTLinesC, MatchesAll, TotEr, true, fp0); // MD_MOD_OPTIM_ALL_LINES this line
			nMatchOld = nMatchNew;
		}
		// MD_MOD_02_27 end
		


		// Record roll, pitch, yaw
		//MD_DataC.VP3D_RAdjust = (MD_DataC.R_A_C0.t()).mmul(MD_DataC.VP3D_R_A_C);
		//MD_DataC.VP3D_pyr_CamPos = ECalib.R3_to_pyr(MD_DataC.VP3D_RAdjust);
		//MD_DataC.Refined_RAdjust = (MD_DataC.R_A_C0.t()).mmul(MD_DataC.Refined_R_A_C);
		//MD_DataC.Refined_pyr_CamPos = ECalib.R3_to_pyr(MD_DataC.Refined_RAdjust);

		// MATLAB_MATCH add following 2 lines:
//#ifndef MYPYR
#if MYPYR == false
		MD_DataC.VP3D_pyr_CamPos = ECalib.get_pyr(MD_DataC.VP3D_R_A_C, MD_DataC.CamNum,fp0);
		MD_DataC.Refined_pyr_CamPos = ECalib.get_pyr(MD_DataC.Refined_R_A_C, MD_DataC.CamNum,fp0);
#else
		double pitch, yaw, roll;
		MD_DataC.VP3D_pyr_CamPos = ECalib.get_pyr(MD_DataC.VP3D_R_A_C, pitch, yaw, roll, MD_DataC.CamNum, fp0);
		MD_DataC.Refined_pyr_CamPos = ECalib.get_pyr(MD_DataC.Refined_R_A_C, pitch, yaw, roll, MD_DataC.CamNum, fp0);
#endif
			



#ifdef PIKEOS
		PrintData();
#else
		PrintData(MD_DataC, fp0);
		//PrintData(MD_DataC);
#endif

		/**************************** MD_MOD extrinsic calibration algorithm END ******************************************/
	  





#ifdef PIKEOS
      jobAlgoVpcsState_e = e_VpcsJobAlgoRunStage2;
    }
      break;

    case e_VpcsJobAlgoRunStage2:
    {
#endif
      if (v_FLAG_t == shmdata::e_EOL_CALIBRATION_IN_PROCESS)
      {
#ifdef VPCS2D
        mecl::Point3f v_R_at[3];
        mecl::Point3f v_RR_at[3];
        for (uint16_t v_I_u16 = 0; v_I_u16 < c_ClusterNum_u32; v_I_u16++)
        {
          // PRQA S 3270 1 // to avoiding division by 0 we have to test == 0
          if (vanpoints_s.vanPoints_as[v_I_u16].z_f32 == 0)
          {
            vanpoints_s.vanPoints_as[v_I_u16].z_f32 = mecl::math::numeric_limits<float32_t>::epsilon_x();
          }
          v_R_at[v_I_u16].x_x = vanpoints_s.vanPoints_as[v_I_u16].x_f32 / vanpoints_s.vanPoints_as[v_I_u16].z_f32;
          v_R_at[v_I_u16].y_x = vanpoints_s.vanPoints_as[v_I_u16].y_f32 / vanpoints_s.vanPoints_as[v_I_u16].z_f32;
          v_R_at[v_I_u16].z_x = 1.0F;
        }

        v_R_at[2].x_x = v_R_at[0].y_x * v_R_at[1].z_x - v_R_at[0].z_x * v_R_at[1].y_x;
        v_R_at[2].y_x = v_R_at[0].z_x * v_R_at[1].x_x - v_R_at[0].x_x * v_R_at[1].z_x;
        v_R_at[2].z_x = v_R_at[0].x_x * v_R_at[1].y_x - v_R_at[0].y_x * v_R_at[1].x_x;

        mecl::Point2f v_Zenith_t;
        mecl::Point2f v_Horrizon_t;
        mecl::Point2f v_Extension_t;
        float32_t v_Meanx_f32 = 0;
        float32_t v_Meany_f32 = 0;
        sint32_t v_Cnt_s32 = 0;
        float32_t v_AvgTheta_f32 = 0;
		mecl::core::Point2D<float32_t>::Config_s c_PppPxCfg_s;
        if ((cameraID_t == e_RightCamAlgo) || (cameraID_t == e_LeftCamAlgo))
        {
          bool_t v_NoHorizontalEdgeFnd_b = true;
          for (sint32_t i = 0; i < vanpoints_s.lineNum; ++i)
          {
            if (mecl::math::abs_x<float32_t>(vanpoints_s.lines_as[i].theta_f32) < 0.139626F) // 8 degree
            {
              v_AvgTheta_f32 += vanpoints_s.lines_as[i].theta_f32;
              v_Meanx_f32    += vanpoints_s.lines_as[i].mean_af32[0];
              v_Meany_f32    += vanpoints_s.lines_as[i].mean_af32[1];
              v_Cnt_s32 += 1;
              v_NoHorizontalEdgeFnd_b = false;
            }
          }
          if (v_NoHorizontalEdgeFnd_b)
          {
#ifdef DBG_PRINT
            vm_cprintf("No Horizontal edge is found \n");
#endif
            v_FLAG_t = shmdata::e_Vpcs_TARGET_FEATURE_MISSING;
          }
          v_AvgTheta_f32 /= static_cast<float32_t>(v_Cnt_s32);
          v_Meanx_f32    /= static_cast<float32_t>(v_Cnt_s32);
          v_Meany_f32    /= static_cast<float32_t>(v_Cnt_s32);
		  
		  // Principal point c_PppPxCfg_s

		  //mecl::core::Point2D<float32_t>::Config_s c_PppPxCfg_s;
		  if ((mecl::math::abs_x<float64_t>(
			  mecl::math::abs_x<float64_t>(mecl::math::trigonometry<float32_t>::atan2_x(
				  v_R_at[0].y_x - 400.0F,
				  v_R_at[0].x_x - 640.0F
				  // Principal point and pixel size, see above.
                  /*v_R_at[0].y_x - c_PppPxCfg_s.cVal_ax[1],
                  v_R_at[0].x_x - c_PppPxCfg_s.cVal_ax[0]*/
			  ))
                  - M_PI / 2.0F)
               < 0.5F))
          {
            v_Zenith_t.x_x = v_R_at[0].x_x;
            v_Zenith_t.y_x = v_R_at[0].y_x;
            //if (v_R_at[1].x_x < 0)
			if (mecl::math::abs_x(v_R_at[1].x_x) < 2000.0F)
            {
              v_Horrizon_t.x_x = v_R_at[1].x_x;
              v_Horrizon_t.y_x = v_R_at[1].y_x;
//			  float64_t angle_compensate_x = mecl::math::trigonometry<float32_t>::atan2_x(
//				  (c_PppPxCfg_s.cVal_ax[1] - 640), (c_PppPxCfg_s.cVal_ax[0] - 400));
////#ifdef DBG_PRINT
//#if 1
//			  vm_cprintf("VPCS_DBG: angle_compensate_x = %f\n", angle_compensate_x*180.0/M_PI);
//#endif
//              v_Extension_t = mlineEnd_toRight(v_AvgTheta_f32 - angle_compensate_x, v_Horrizon_t);
			  v_Extension_t = mlineEnd_toRight(v_AvgTheta_f32, v_Horrizon_t);
              v_R_at[1].x_x = v_Extension_t.x_x;
              v_R_at[1].y_x = v_Extension_t.y_x;
            }
            v_RR_at[0] = v_R_at[1];
            v_RR_at[1] = v_R_at[0];
            v_RR_at[2] = v_R_at[2];
          }
          else
          {
            v_Zenith_t.x_x = v_R_at[1].x_x;
            v_Zenith_t.y_x = v_R_at[1].y_x;
            //if (v_R_at[0].x_x < 0)
			if (mecl::math::abs_x(v_R_at[0].x_x) < 4000.0F)
            {
              v_Horrizon_t.x_x = v_R_at[0].x_x;
              v_Horrizon_t.y_x = v_R_at[0].y_x;
              v_Extension_t = mlineEnd_toRight(v_AvgTheta_f32, v_Horrizon_t);
              v_R_at[0].x_x = v_Extension_t.x_x;
              v_R_at[0].y_x = v_Extension_t.y_x;
            }
            v_RR_at[0] = v_R_at[0];
            v_RR_at[1] = v_R_at[1];
            v_RR_at[2] = v_R_at[2];
          }
        }
        if ((cameraID_t == e_FrontCamAlgo) || (cameraID_t == e_RearCamAlgo))
        {
          sint32_t longlineIdx = 0;
          for (sint32_t i = 0; i < vanpoints_s.lineNum; ++i)
          {
            // PRQA S 3270 4 // to avoiding division by 0 we have to test == 0
            if ((mecl::math::abs_x<float32_t>(vanpoints_s.lines_as[i].theta_f32) < 0.139626F)
             && (vanpoints_s.lines_as[i].mean_af32[0] != 0)
             && (vanpoints_s.lines_as[i].mean_af32[1] != 0)
             && (vanpoints_s.lines_as[i].r_f32 != 0))
            // 8 degree
            {
#ifdef DBG_PRINT

				vm_cprintf("VPCS_DBG: vanpoints_s.lines_as[%d].theta_f32 = %f length %f \n",i, vanpoints_s.lines_as[i].theta_f32*180.0 / M_PI, vanpoints_s.lines_as[i].r_f32);
#endif
              v_AvgTheta_f32 += vanpoints_s.lines_as[i].theta_f32;
              v_Cnt_s32 += 1;
            }
          }
          v_Meanx_f32 = vanpoints_s.lines_as[longlineIdx].mean_af32[0];
          v_Meany_f32 = vanpoints_s.lines_as[longlineIdx].mean_af32[1];
          v_AvgTheta_f32 /= static_cast<float32_t>(v_Cnt_s32);
#ifdef DBG_PRINT
          vm_cprintfFloat("VPCS_DBG: meanx = %f meany = %f \r\n", v_Meanx_f32, v_Meany_f32);
#endif

#ifdef DBG_PRINT
          vm_cprintfFloat("VPCS_DBG: r[0]=(%f, %f, %f)\n", v_R_at[0].x_x, v_R_at[0].y_x, v_R_at[0].z_x);
          vm_cprintfFloat("VPCS_DBG: r[1]=(%f, %f, %f)\n", v_R_at[1].x_x, v_R_at[1].y_x, v_R_at[1].z_x);
          vm_cprintfFloat("VPCS_DBG: r[2]=(%f, %f, %f)\n", v_R_at[2].x_x, v_R_at[2].y_x, v_R_at[2].z_x);
#endif
          if ((mecl::math::abs_x(v_R_at[0].y_x) < 220.0F) && (mecl::math::abs_x(v_R_at[0].x_x - 640.0F) < 200.0F)) //test if r[0] is zenith vp
          {
            v_Zenith_t.x_x = v_R_at[0].x_x;
            v_Zenith_t.y_x = v_R_at[0].y_x;

            if (mecl::math::abs_x(v_R_at[1].x_x) < 4000.0F)
            {
              v_Horrizon_t.x_x = v_R_at[1].x_x;
              v_Horrizon_t.y_x = v_R_at[1].y_x;

              v_Extension_t = mlineEnd_toRight(v_AvgTheta_f32, v_Horrizon_t);
              v_R_at[1].x_x = v_Extension_t.x_x;
              v_R_at[1].y_x = v_Extension_t.y_x;
            }
			//if (cameraID_t == e_FrontCamAlgo)
			//{
			//	//v_R_at[1].x_x = 8407.9;
			//	v_R_at[1].x_x = 32407.9;
			//	//v_R_at[1].y_x = 353.5;
			//}
			//if (cameraID_t == e_RearCamAlgo)
			//{
			//	v_R_at[1].x_x = 15544.217773;
			//	v_R_at[1].y_x = 105.901825;
			//}
            v_RR_at[0] = v_R_at[1];
            v_RR_at[1] = v_R_at[0];
            v_RR_at[2] = v_R_at[2];
          }
          else
          {
            if (mecl::math::abs_x(v_R_at[0].x_x) < 4000.0F)
            {
              v_Zenith_t.x_x = v_R_at[1].x_x;
              v_Zenith_t.y_x = v_R_at[1].y_x;
              v_Horrizon_t.x_x = v_R_at[0].x_x;
              v_Horrizon_t.y_x = v_R_at[0].y_x;
              v_Extension_t = mlineEnd_toRight(v_AvgTheta_f32, v_Horrizon_t);
              v_R_at[0].x_x = v_Extension_t.x_x;
              v_R_at[0].y_x = v_Extension_t.y_x;
            }
			  //if (cameraID_t == e_FrontCamAlgo)
			  //{
				 // //v_R_at[0].x_x = 8407.9;
				 // v_R_at[0].x_x = 12407.9;
				 // v_R_at[0].y_x = 353.5;
			  //}
			  //if (cameraID_t == e_RearCamAlgo)
			  //{
				 // v_R_at[0].x_x = 15544.217773;
				 // v_R_at[0].y_x = 105.901825;
				 // 
			  //}
            v_RR_at[0] = v_R_at[0];
            v_RR_at[1] = v_R_at[1];
            v_RR_at[2] = v_R_at[2];
          }
        }
        //start orientation estimate
#ifdef DBG_PRINT
        for (uint8_t i = 0; i < c_ClusterNum_u32; i++)
        {
          vm_cprintf("VPCS_DBG: adjusted VP %d is: \r\n", i);
          vm_cprintfFloat("VPCS_DBG:  %f %f %f \r\n ", v_RR_at[i].x_x, v_RR_at[i].y_x, v_RR_at[i].z_x);
        }
#endif
        mecl::core::RotationMatrix<float32_t>::EulerAngles_s v_EulerAngles_s;
        v_RotMatrix_o = RotfromVP(&v_RR_at[0], camWorldPosition_x, isImageFlipped_b,
            &v_EulerAngles_s, mCameraObject_o);

		if (((v_R_at[0].x_x) < -4000.0F) || ((v_R_at[1].x_x) < -4000.0F))
		{
			alphaX_f32 = M_PI - v_EulerAngles_s.pitch_x;
			betaY_f32 = -v_EulerAngles_s.yaw_x;
			gammaZ_f32 = v_EulerAngles_s.roll_x - M_PI;
		}
		else
		{
        alphaX_f32 = v_EulerAngles_s.pitch_x;
        betaY_f32 = v_EulerAngles_s.yaw_x;
        gammaZ_f32 = v_EulerAngles_s.roll_x;
		}

#ifdef DBG_LOG_RESULTS
        vm_cprintfFloat("VPCS_DBG: alphaX_f32 = %f betaY_f32 = %f gammZ_f32 = %f \r\n",
            mecl::math::toDegrees_x(alphaX_f32), mecl::math::toDegrees_x(betaY_f32),
            mecl::math::toDegrees_x(gammaZ_f32));
#endif
#endif
#ifdef VPCS3D

		vpcs::DataProviderVpcs i_dataProviderVpcs;
		i_dataProviderVpcs.wfmd.frameID = 0; //No setter, straight from MCU
		i_dataProviderVpcs.setLastRequestedAt_v(0);

#ifdef PIKEOS
		vpcs::JobAlgoVpcs i_jobVpcs(i_dataProviderVpcs, *b_LogSender_ro);
#else
		vpcs::JobAlgoVpcs i_jobVpcs(i_dataProviderVpcs);
#endif

		i_dataProviderVpcs.wfmd.responseState = shmdata::ResponseState_e::e_ResponsePositive; //No setter, straight from MCU

		i_jobVpcs.init_v();

		mecl::core::ArrayList<MD_Data, 4> DatAll;
		MD_Data MD_Data;

		// Front cam (0)
		i_dataProviderVpcs.setMcuCameraId_v(vpcs::e_FrontCamAlgo);
		i_dataProviderVpcs.wfmd.frameID += 1;
		i_dataProviderVpcs.wfmd.RequestedAt = i_dataProviderVpcs.wfmd.frameID;
		i_dataProviderVpcs.wfmd.algoCommand = shmdata::AlgoCommand_e::e_Start;
		const uint8_t *inImage = i_dataProviderVpcs.getInputImage_pu8();
		uint8_t i_WarpedImage_pu8[vpcs::c_ImageSize_u32];
		i_jobVpcs.start_v(inImage, i_WarpedImage_pu8);
		i_jobVpcs.execute_v();
		i_jobVpcs.end_v();
		MD_Data.StoreVars_MD(DatAll, i_jobVpcs, inImage);

		// Left cam (1)
		i_dataProviderVpcs.setMcuCameraId_v(vpcs::e_LeftCamAlgo);
		i_dataProviderVpcs.wfmd.frameID += 1;
		i_dataProviderVpcs.wfmd.RequestedAt = i_dataProviderVpcs.wfmd.frameID;
		i_dataProviderVpcs.wfmd.algoCommand = shmdata::AlgoCommand_e::e_Start;
		inImage = i_dataProviderVpcs.getInputImage_pu8();
		i_jobVpcs.start_v(inImage, i_WarpedImage_pu8);
		i_jobVpcs.execute_v();
		i_jobVpcs.end_v();
		MD_Data.StoreVars_MD(DatAll, i_jobVpcs, inImage);

		// rerun new frame since the EOL setup this is based on can only run either start or execute+end each frame
		i_dataProviderVpcs.wfmd.frameID += 1;

		// Rear cam (2)
		i_dataProviderVpcs.setMcuCameraId_v(vpcs::e_RearCamAlgo);
		i_dataProviderVpcs.wfmd.frameID += 1;
		i_dataProviderVpcs.wfmd.RequestedAt = i_dataProviderVpcs.wfmd.frameID;
		i_dataProviderVpcs.wfmd.algoCommand = shmdata::AlgoCommand_e::e_Start;
		inImage = i_dataProviderVpcs.getInputImage_pu8();
		i_jobVpcs.start_v(inImage, i_WarpedImage_pu8);
		i_jobVpcs.execute_v();
		i_jobVpcs.end_v();
		MD_Data.StoreVars_MD(DatAll, i_jobVpcs, inImage);

		// Right cam (3)
		i_dataProviderVpcs.setMcuCameraId_v(vpcs::e_RightCamAlgo);
		i_dataProviderVpcs.wfmd.frameID += 1;
		i_dataProviderVpcs.wfmd.RequestedAt = i_dataProviderVpcs.wfmd.frameID;
		i_dataProviderVpcs.wfmd.algoCommand = shmdata::AlgoCommand_e::e_Start;
		inImage = i_dataProviderVpcs.getInputImage_pu8();
		i_jobVpcs.start_v(inImage, i_WarpedImage_pu8);
		i_jobVpcs.execute_v();
		i_jobVpcs.end_v();
		MD_Data.StoreVars_MD(DatAll, i_jobVpcs, inImage);
		DatAll[3].Orig_pyr_CamPos(2) -= M_PI;



#endif
      } // if in progrocess
      jobAlgoVpcsState_e = e_VpcsJobAlgoJobFinished;
    }
    break;

    case e_VpcsJobAlgoJobFinished:
    {
      // Nothing to do yet
    }
    break;

    default:
    {
      AssertMsg(false, "Unexpected VPCS JobAlgo FSM state [%d] \r\n", static_cast<uint32_t>(jobAlgoVpcsState_e));
    }
    break;
  }
}




/************************************************************* Print out data ******************************************************************/
#ifdef PIKEOS
void JobAlgoVpcs::PrintData()
{
	float32_t Rad2Deg = 180.0 / M_PI;
	float32_t x,y,z;
	// Intrinsics
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"\n\ncamera ID %d\n",cameraID_t
	);
	log_printf("\n\nIntrinsic calibration specs:\n");
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"\n\nIntrinsic calibration specs:\n"
	);
	vm_cprintfFloat("FC = [%f, %f] \n", MD_DataC.CamFC[0], MD_DataC.CamFC[1]);
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"FC = [%f, %f] \n", MD_DataC.CamFC[0], MD_DataC.CamFC[1]
	);
	vm_cprintfFloat("PP = [%f, %f] \n", MD_DataC.CamPP[0], MD_DataC.CamPP[1]);
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"PP = [%f, %f] \n", MD_DataC.CamPP[0], MD_DataC.CamPP[1]
	);
	vm_cprintfFloat("Dist_I2W = [%f, %f, %f, %f, %f, %f] \n", MD_DataC.Dist_im2world[0], MD_DataC.Dist_im2world[1],
			MD_DataC.Dist_im2world[2], MD_DataC.Dist_im2world[3],MD_DataC.Dist_im2world[4], MD_DataC.Dist_im2world[5]);
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"Dist_I2W = [%f, %f, %f, %f, %f, %f] \n", MD_DataC.Dist_im2world[0], MD_DataC.Dist_im2world[1],
			MD_DataC.Dist_im2world[2], MD_DataC.Dist_im2world[3],MD_DataC.Dist_im2world[4], MD_DataC.Dist_im2world[5]
	);
	vm_cprintfFloat("Dist_W2I = [%f, %f, %f, %f, %f, %f] \n", MD_DataC.Dist_world2im[0], MD_DataC.Dist_world2im[1],
			MD_DataC.Dist_world2im[2], MD_DataC.Dist_world2im[3], MD_DataC.Dist_world2im[4], MD_DataC.Dist_world2im[5]);
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"Dist_W2I = [%f, %f, %f, %f, %f, %f] \n", MD_DataC.Dist_world2im[0], MD_DataC.Dist_world2im[1],
			MD_DataC.Dist_world2im[2], MD_DataC.Dist_world2im[3], MD_DataC.Dist_world2im[4], MD_DataC.Dist_world2im[5]
	);
	// Extrinsics
	log_printf("\nExtrinsics calibration summary:\n");
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			""
	);
	vm_cprintfFloat("Extrinsic fit Orig  [x, y, z] = [%f, %f, %f] \n",MD_DataC.Orig_xyz_CamPos(0), MD_DataC.Orig_xyz_CamPos(1),MD_DataC.Orig_xyz_CamPos(2));
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"Extrinsic fit Orig  [x, y, z] = [%f, %f, %f] \n",MD_DataC.Orig_xyz_CamPos(0), MD_DataC.Orig_xyz_CamPos(1),MD_DataC.Orig_xyz_CamPos(2)
	);
	vm_cprintfFloat("Extrinsic fit Final [x, y, z] = [%f, %f, %f] \n", MD_DataC.Refined_xyz_CamPos(0), MD_DataC.Refined_xyz_CamPos(1), MD_DataC.Refined_xyz_CamPos(2));
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"Extrinsic fit Final [x, y, z] = [%f, %f, %f] \n", MD_DataC.Refined_xyz_CamPos(0), MD_DataC.Refined_xyz_CamPos(1), MD_DataC.Refined_xyz_CamPos(2)
	);
	x = MD_DataC.Refined_xyz_CamPos(0)-MD_DataC.Orig_xyz_CamPos(0);
	y = MD_DataC.Refined_xyz_CamPos(1)-MD_DataC.Orig_xyz_CamPos(1);
	z = MD_DataC.Refined_xyz_CamPos(2)-MD_DataC.Orig_xyz_CamPos(2);
	vm_cprintfFloat("Extrinsic fit Final - Orig [x, y, z] = [%f, %f, %f] \n", x,y,z);
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"Extrinsic fit Final - Orig [x, y, z] = [%f, %f, %f] \n", x,y,z
	);
	x = MD_DataC.Orig_pyr_CamPos(0)*Rad2Deg;
	y = MD_DataC.Orig_pyr_CamPos(1)*Rad2Deg;
	z = MD_DataC.Orig_pyr_CamPos(2)*Rad2Deg;
	vm_cprintfFloat("Orig [Pitch, Yaw, Roll] = [%f, %f, %f] \n", x,y,z);
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"Orig [Pitch, Yaw, Roll] = [%f, %f, %f] \n", x,y,z
	);
	x = MD_DataC.VP3D_pyr_CamPos(0)*Rad2Deg;
	y = MD_DataC.VP3D_pyr_CamPos(1)*Rad2Deg;
	z = MD_DataC.VP3D_pyr_CamPos(2)*Rad2Deg;
	vm_cprintfFloat("VP_3D [Pitch, Yaw, Roll] = [%f, %f, %f] \n", x,y,z);
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"VP_3D [Pitch, Yaw, Roll] = [%f, %f, %f] \n", x,y,z
	);
	x = MD_DataC.Refined_pyr_CamPos(0)*Rad2Deg;
	y = MD_DataC.Refined_pyr_CamPos(1)*Rad2Deg;
	z = MD_DataC.Refined_pyr_CamPos(2)*Rad2Deg;
	vm_cprintfFloat("Final [Pitch, Yaw, Roll] = [%f, %f, %f] \n", x,y,z);
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"Final [Pitch, Yaw, Roll] = [%f, %f, %f] \n", x,y,z
	);
	x = (MD_DataC.Refined_pyr_CamPos(0) - MD_DataC.Orig_pyr_CamPos(0)) * Rad2Deg;
	y = (MD_DataC.Refined_pyr_CamPos(1) - MD_DataC.Orig_pyr_CamPos(1)) * Rad2Deg;
	z = (MD_DataC.Refined_pyr_CamPos(2) - MD_DataC.Orig_pyr_CamPos(2)) * Rad2Deg;
	vm_cprintfFloat("Final - Orig [Pitch, Yaw, Roll] = [%f, %f, %f] \n\n\n", x,y,z);
	log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"Final - Orig [Pitch, Yaw, Roll] = [%f, %f, %f] \n\n\n", x,y,z
	);
}
#else
void JobAlgoVpcs::PrintData(MD_Data &MD_DataC, FILE *fp0)
//void JobAlgoVpcs::PrintData(MD_Data &MD_DataC)
{
	float32_t Rad2Deg = 180.0 / M_PI;
	float32_t x,y,z;
	// Intrinsics
	log_printf("\n\nIntrinsic calibration specs:\n");
	vm_cprintfFloat("FC = [%f, %f] \n", MD_DataC.CamFC[0], MD_DataC.CamFC[1]);
	vm_cprintfFloat("PP = [%f, %f] \n", MD_DataC.CamPP[0], MD_DataC.CamPP[1]);
	vm_cprintfFloat("Dist_I2W = [%f, %f, %f, %f, %f, %f] \n", MD_DataC.Dist_im2world[0], MD_DataC.Dist_im2world[1],		
			MD_DataC.Dist_im2world[2], MD_DataC.Dist_im2world[3],MD_DataC.Dist_im2world[4], MD_DataC.Dist_im2world[5]);
	fprintf(fp0,"\n\nIntrinsic calibration specs:\n");
	fprintf(fp0, "FC = [%f, %f] \n", MD_DataC.CamFC[0], MD_DataC.CamFC[1]);
	fprintf(fp0, "PP = [%f, %f] \n", MD_DataC.CamPP[0], MD_DataC.CamPP[1]);
	fprintf(fp0, "Dist_I2W = [%f, %f, %f, %f, %f, %f] \n", MD_DataC.Dist_im2world[0], MD_DataC.Dist_im2world[1],
	MD_DataC.Dist_im2world[2], MD_DataC.Dist_im2world[3], MD_DataC.Dist_im2world[4], MD_DataC.Dist_im2world[5]);
	vm_cprintfFloat("Dist_W2I = [%f, %f, %f, %f, %f, %f] \n", MD_DataC.Dist_world2im[0], MD_DataC.Dist_world2im[1],		
			MD_DataC.Dist_world2im[2], MD_DataC.Dist_world2im[3], MD_DataC.Dist_world2im[4], MD_DataC.Dist_world2im[5]);
	fprintf(fp0, "Dist_W2I = [%f, %f, %f, %f, %f, %f] \n", MD_DataC.Dist_world2im[0], MD_DataC.Dist_world2im[1],
	MD_DataC.Dist_world2im[2], MD_DataC.Dist_world2im[3], MD_DataC.Dist_world2im[4], MD_DataC.Dist_world2im[5]);
	
		
	
		

	// Extrinsics
 	log_printf("\nExtrinsics calibration summary:\n");
	vm_cprintfFloat("Extrinsic fit Orig  [x, y, z] = [%f, %f, %f] \n",MD_DataC.Orig_xyz_CamPos(0), MD_DataC.Orig_xyz_CamPos(1),MD_DataC.Orig_xyz_CamPos(2));
	vm_cprintfFloat("Extrinsic fit Final [x, y, z] = [%f, %f, %f] \n", MD_DataC.Refined_xyz_CamPos(0), MD_DataC.Refined_xyz_CamPos(1), MD_DataC.Refined_xyz_CamPos(2));
	fprintf(fp0, "\nExtrinsics calibration summary:\n");
	fprintf(fp0, "Extrinsic fit Orig  [x, y, z] = [%f, %f, %f] \n", MD_DataC.Orig_xyz_CamPos(0), MD_DataC.Orig_xyz_CamPos(1), MD_DataC.Orig_xyz_CamPos(2));
	fprintf(fp0, "Extrinsic fit Final [x, y, z] = [%f, %f, %f] \n", MD_DataC.Refined_xyz_CamPos(0), MD_DataC.Refined_xyz_CamPos(1), MD_DataC.Refined_xyz_CamPos(2));

	x = MD_DataC.Refined_xyz_CamPos(0)-MD_DataC.Orig_xyz_CamPos(0);
	y = MD_DataC.Refined_xyz_CamPos(1)-MD_DataC.Orig_xyz_CamPos(1);
	z = MD_DataC.Refined_xyz_CamPos(2)-MD_DataC.Orig_xyz_CamPos(2);
	vm_cprintfFloat("Extrinsic fit Final - Orig [x, y, z] = [%f, %f, %f] \n", x,y,z);
	fprintf(fp0, "Extrinsic fit Final - Orig [x, y, z] = [%f, %f, %f] \n", x, y, z);
	x = MD_DataC.Orig_pyr_CamPos(0)*Rad2Deg;
	y = MD_DataC.Orig_pyr_CamPos(1)*Rad2Deg;
	z = MD_DataC.Orig_pyr_CamPos(2)*Rad2Deg;
	vm_cprintfFloat("Orig [Pitch, Yaw, Roll] = [%f, %f, %f] \n", x,y,z);
	fprintf(fp0, "Orig [Pitch, Yaw, Roll] = [%f, %f, %f] \n", x, y, z);
	x = MD_DataC.VP3D_pyr_CamPos(0)*Rad2Deg;
	y = MD_DataC.VP3D_pyr_CamPos(1)*Rad2Deg;
	z = MD_DataC.VP3D_pyr_CamPos(2)*Rad2Deg;
	vm_cprintfFloat("VP_3D [Pitch, Yaw, Roll] = [%f, %f, %f] \n", x,y,z);
	fprintf(fp0, "VP_3D [Pitch, Yaw, Roll] = [%f, %f, %f] \n", x, y, z);
	x = MD_DataC.Refined_pyr_CamPos(0)*Rad2Deg;
	y = MD_DataC.Refined_pyr_CamPos(1)*Rad2Deg;
	z = MD_DataC.Refined_pyr_CamPos(2)*Rad2Deg;
	vm_cprintfFloat("Final [Pitch, Yaw, Roll] = [%f, %f, %f] \n", x,y,z);
	fprintf(fp0, "Final [Pitch, Yaw, Roll] = [%f, %f, %f] \n", x, y, z);
	x = (MD_DataC.Refined_pyr_CamPos(0) - MD_DataC.Orig_pyr_CamPos(0)) * Rad2Deg;
	y = (MD_DataC.Refined_pyr_CamPos(1) - MD_DataC.Orig_pyr_CamPos(1)) * Rad2Deg;
	z = (MD_DataC.Refined_pyr_CamPos(2) - MD_DataC.Orig_pyr_CamPos(2)) * Rad2Deg;
  	vm_cprintfFloat("Final - Orig [Pitch, Yaw, Roll] = [%f, %f, %f] \n\n\n", x,y,z);
	fprintf(fp0,"Final - Orig [Pitch, Yaw, Roll] = [%f, %f, %f] \n\n\n", x, y, z);
}
#endif



void JobAlgoVpcs::end_v(char *dirname, FILE *fp0)
{
#ifdef PIKEOS
  const mecl::Pinhole_t::Extrinsic_s* c_CamExtrinsic_ps = dataProvider_ro.accessCameraExtrinsicDesign_ps(
      dataProvider_ro.getRequestedCameraId_e());
#else
  mCameraObject_o = dataProvider_ro.getCameraInfo_rt(cameraID_t, dirname);
  mecl::model::Pinhole<float32_t> &v_PinHole = dynamic_cast<mecl::model::Pinhole<float32_t>&>(mCameraObject_o.getImager_rx());
  const mecl::model::Pinhole<float32_t>::Extrinsic_s *c_CamExtrinsic_ps = &v_PinHole.getExtrinsic_rs();
#endif

  const float32_t c_CamExtrX_f32 = c_CamExtrinsic_ps->pos_x.cVal_ax[0];
  const float32_t c_CamExtrY_f32 = c_CamExtrinsic_ps->pos_x.cVal_ax[1];
  const float32_t c_CamExtrZ_f32 = c_CamExtrinsic_ps->pos_x.cVal_ax[2];
  const float32_t c_DeltaX_f32 = MD_DataC.Refined_xyz_CamPos(0) - c_CamExtrX_f32;
  const float32_t c_DeltaY_f32 = MD_DataC.Refined_xyz_CamPos(1) - c_CamExtrY_f32;
  const float32_t c_DeltaZ_f32 = MD_DataC.Refined_xyz_CamPos(2) - c_CamExtrZ_f32;

  /*if ((v_FLAG_t == shmdata::TARGET_NOT_FOUND_INIT)
   || (v_FLAG_t == shmdata::TARGET_IDENTIFICATION_ERROR_LEFT)*/ 
  if ((v_FLAG_t == shmdata::TARGET_IDENTIFICATION_ERROR_LEFT)
   || (v_FLAG_t == shmdata::TARGET_IDENTIFICATION_ERROR_RIGHT)
   || (v_FLAG_t == shmdata::TARGET_NOT_FOUND_LEFT)
   || (v_FLAG_t == shmdata::TARGET_NOT_FOUND_RIGHT)
   || (v_FLAG_t == shmdata::ABORTED)
   )
  {
    dataProvider_ro.setVpcsErrorCode_v(v_FLAG_t);
#ifdef DBG_LOG_RESULTS
    vm_cprintf("VPCS_ERROR: VpcsErrorCode = %d\n", static_cast<uint32_t>(v_FLAG_t));
	fprintf(fp0,"VPCS_ERROR: VpcsErrorCode = %d\n", static_cast<uint32_t>(v_FLAG_t));
#endif
#ifdef PIKEOS
    log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32, "VPCS_ERROR: VpcsErrorCode = %d",
        static_cast<uint32_t>(v_FLAG_t));
#endif
  }

  if (v_FLAG_t == shmdata::e_EOL_CALIBRATION_IN_PROCESS)
  {
    mecl::core::RotationMatrix<float32_t>::EulerAngles_s v_EA_s;

    // SAI_RoBer: We get here the result values in Radians!
    const float32_t c_CamExtrPitch_f32 = c_CamExtrinsic_ps->eulerAngles_s.pitch_x;
    const float32_t c_CamExtrYaw_f32 = c_CamExtrinsic_ps->eulerAngles_s.yaw_x;
    const float32_t c_CamExtrRoll_f32 = c_CamExtrinsic_ps->eulerAngles_s.roll_x;

		/*v_EA_s.pitch_x = (MD_DataC.Refined_pyr_CamPos(0)) - c_CamExtrPitch_f32;
		if (MD_DataC.CamNum == 0 || MD_DataC.CamNum == 2)
			v_EA_s.yaw_x   = -1.0f*((MD_DataC.Refined_pyr_CamPos(2))) - c_CamExtrYaw_f32;
		else
			v_EA_s.yaw_x   = ((MD_DataC.Refined_pyr_CamPos(2))) - c_CamExtrYaw_f32;
		if (MD_DataC.CamNum == 1 || MD_DataC.CamNum == 3)
			v_EA_s.roll_x  = -1.0f*((MD_DataC.Refined_pyr_CamPos(1))) - c_CamExtrRoll_f32;
		else
			v_EA_s.roll_x  = ((MD_DataC.Refined_pyr_CamPos(1))) - c_CamExtrRoll_f32;
*/
	if (mecl::math::abs_x(c_CamExtrRoll_f32) > 2.1F)MD_DataC.Refined_pyr_CamPos(0) = M_PI - MD_DataC.Refined_pyr_CamPos(0);
	v_EA_s.pitch_x = MD_DataC.Refined_pyr_CamPos(0) - MD_DataC.Orig_pyr_CamPos(0);
	v_EA_s.yaw_x = MD_DataC.Refined_pyr_CamPos(1) - MD_DataC.Orig_pyr_CamPos(1);
	v_EA_s.roll_x = MD_DataC.Refined_pyr_CamPos(2) - MD_DataC.Orig_pyr_CamPos(2);


    mecl::core::RotationMatrix<float32_t>::normalizeEA_v(v_EA_s);

    // Convert to degrees
    v_EA_s.pitch_x = mecl::math::toDegrees_x(v_EA_s.pitch_x);
    v_EA_s.yaw_x   = mecl::math::toDegrees_x(v_EA_s.yaw_x);
    v_EA_s.roll_x  = mecl::math::toDegrees_x(v_EA_s.roll_x);
#ifdef PIKEOS
		log_ro.logError_v(logging::LogCtx::c_LogCtxDefault_u32,
							"Output [Pitch, Yaw, Roll] = [%f, %f, %f] \n\n\n", v_EA_s.pitch_x,v_EA_s.yaw_x,v_EA_s.roll_x
					);
#endif
	// The right roll angle default is 180 which is weird...
	if (v_EA_s.roll_x > 90)
		v_EA_s.roll_x -= 180.0;
	else if (v_EA_s.roll_x < -90)
		v_EA_s.roll_x += 180.0;



#ifdef DBG_LOG_RESULTS
    log_printf("Calculating results is done.\r\n");
    /*vm_cprintfFloat("VPCS_DBG:   CamExtrinsics [Pitch, Yaw, Roll] = [%f, %f, %f] \r\n", c_CamExtrPitch_f32,
        c_CamExtrYaw_f32, c_CamExtrRoll_f32);
    vm_cprintfFloat("VPCS_DBG:   CamExtrinsics [X, Y, Z] = [%f, %f, %f] \r\n", c_CamExtrX_f32, c_CamExtrY_f32,
        c_CamExtrZ_f32);
    vm_cprintfFloat("VPCS_DBG:   Delta values[DPitch, DYaw, DRoll] = [%f, %f, %f] \r\n", v_EA_s.pitch_x, v_EA_s.yaw_x,
        v_EA_s.roll_x);
    vm_cprintfFloat("VPCS_DBG:   Delta values[X, Y, Z] = [%f, %f, %f] \r\n", c_DeltaX_f32, c_DeltaY_f32, c_DeltaZ_f32);*/
#endif
    // set results in DataProvider
    dataProvider_ro.setDeltaPitch_v(v_EA_s.pitch_x);
    dataProvider_ro.setDeltaYaw_v(v_EA_s.yaw_x);
    dataProvider_ro.setDeltaRoll_v(v_EA_s.roll_x);
    //dataProvider_ro.setDeltaX_v(c_DeltaX_f32);
	if (cameraID_t == vpcs::e_RightCamAlgo || cameraID_t == vpcs::e_LeftCamAlgo)

		dataProvider_ro.setDeltaX_v(c_DeltaX_f32);

	else

		dataProvider_ro.setDeltaX_v(0);
    dataProvider_ro.setDeltaY_v(c_DeltaY_f32);
    dataProvider_ro.setDeltaZ_v(c_DeltaZ_f32);

    v_FLAG_t = shmdata::e_EOL_CALIBRATION_SUCCESS;
#ifdef DBG_PRINT
     vm_cprintf("VPCS_DBG: VpcsErrorCode = %d\n", static_cast<uint32_t>(v_FLAG_t));
	 fprintf(fp0, "VPCS_DBG: VpcsErrorCode = %d\n", static_cast<uint32_t>(v_FLAG_t));
#endif
     dataProvider_ro.setVpcsErrorCode_v(v_FLAG_t);
#ifdef PIKEOS
    log_ro.logInfo_v(logging::LogCtx::c_LogCtxDefault_u32, "[VPCS_INFO]: Calculating results for the camera % is done.",
        dataProvider_ro.getRequestedCameraId_e());
    log_ro.logInfo_v(logging::LogCtx::c_LogCtxDefault_u32,
        "[VPCS_INFO]: v_CamExtrinsics [Pitch, Yaw, Roll] = [%f, %f, %f]", c_CamExtrPitch_f32, c_CamExtrYaw_f32,
        c_CamExtrRoll_f32);
    log_ro.logInfo_v(logging::LogCtx::c_LogCtxDefault_u32, "[VPCS_INFO]: v_CamExtrinsics[X, Y, Z] = [%f, %f, %f]",
        c_CamExtrX_f32, c_CamExtrY_f32, c_CamExtrZ_f32);
    log_ro.logInfo_v(logging::LogCtx::c_LogCtxDefault_u32,
        "[VPCS_INFO]: Delta values[DPitch, DYaw, DRoll] = [%f, %f, %f]", v_EA_s.pitch_x, v_EA_s.yaw_x, v_EA_s.roll_x);
    log_ro.logInfo_v(logging::LogCtx::c_LogCtxDefault_u32, "[VPCS_INFO]: Delta values[X, Y, Z] = [%f, %f, %f]",
        c_DeltaX_f32, c_DeltaY_f32, c_DeltaZ_f32);
#endif
  }

}

  
} //namespace vpcs
