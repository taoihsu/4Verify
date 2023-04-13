//--------------------------------------------------------------------------
/// @file SensorCfg.cpp
/// @brief methods to distort and undistort pixels
///
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Detlef Hafer (Detlef.Hafer.Extern@magna.com)
///
//  --------------------------------------------------------------------------

#include "SensorCfg.h"
#include "mecl/mecl.h"

// PRQA S 3706 EOF // applying subscript operators to pointer values cannot be avoided here

namespace SensorCfg
{
	static mecl::model::LensRadial<float32_t> c_Lens_as[4];
	/*Eol::DataProviderEol i_dataProviderEol;
	Eol::JobAlgoEol i_jobEol(i_dataProviderEol);
	Stitch_Data Sensor_Stitch_DataC[4];*/
	static mecl::model::Pinhole<float32_t> c_Pinhole_s[4];
	static mecl::model::Sensor<float32_t> c_Sensor_s[4];
	// Front, left rear, right
	mecl::model::Camera<float32_t> Sensorcfg::Create_Camera(const W2Image::SetupIntrinsicCfg_s& i_IntrinsicCfg_rs, const W2Image::SetupExtrinsicCfg_s& i_ExtrinsicCfg_rs, const uint8_t cameraID)
	{
		const mecl::model::LensRadial<float32_t>::Config_s c_lensCfg_s =
		{
			{ i_IntrinsicCfg_rs.IntrPolyWorld2Image_0,
			i_IntrinsicCfg_rs.IntrPolyWorld2Image_1,
			i_IntrinsicCfg_rs.IntrPolyWorld2Image_2,
			i_IntrinsicCfg_rs.IntrPolyWorld2Image_3,
			i_IntrinsicCfg_rs.IntrPolyWorld2Image_4,
			i_IntrinsicCfg_rs.IntrPolyWorld2Image_5
			},
			{ i_IntrinsicCfg_rs.IntrPolyImage2World_0,
			i_IntrinsicCfg_rs.IntrPolyImage2World_1,
			i_IntrinsicCfg_rs.IntrPolyImage2World_2,
			i_IntrinsicCfg_rs.IntrPolyImage2World_3,
			i_IntrinsicCfg_rs.IntrPolyImage2World_4,
			i_IntrinsicCfg_rs.IntrPolyImage2World_5
			},
			mecl::math::toDegrees_x(i_IntrinsicCfg_rs.IntrFieldOfView / 2.0F)
		};
		// Lens
		c_Lens_as[cameraID].updateConfig_v(c_lensCfg_s);

		

		// --------------------
		// Pinhole
		const mecl::core::RotationMatrix<float32_t>::EulerAngles_s c_EulerAngles_s = { mecl::math::toRadians_x(i_ExtrinsicCfg_rs.Camera_Pitch), mecl::math::toRadians_x(i_ExtrinsicCfg_rs.Camera_Yaw), mecl::math::toRadians_x(i_ExtrinsicCfg_rs.Camera_Roll) };
		const mecl::core::Point3D<float32_t>::Config_s c_Cfg_s = { { i_ExtrinsicCfg_rs.Camera_X, i_ExtrinsicCfg_rs.Camera_Y, i_ExtrinsicCfg_rs.Camera_Z } };
		// preroll: preroll90?? i_IntrinsicCfg_rs.preroll does not exist!!
		// Use cameraID instead:

		mecl::model::PreRoll_e camera = mecl::model::e_PreRoll90;
		if (cameraID == 0) { // front
			camera = mecl::model::e_PreRoll90;
		}
		else if (cameraID == 1) { // left
			camera = mecl::model::e_PreRoll180;
		}
		else if (cameraID == 2) { // rear
			camera = mecl::model::e_PreRoll270;
		}
		else if (cameraID == 3) { // right
			camera = mecl::model::e_PreRoll0;
		}
		else
		{
			//Do Nothing
		}

		const mecl::model::Pinhole<float32_t>::Extrinsic_s c_Extrinsic_s = { c_Cfg_s, c_EulerAngles_s, camera, mecl::model::e_WorldToCamera };
		const mecl::model::Pinhole<float32_t>::Intrinsic_s c_Intrinsic_s = { i_IntrinsicCfg_rs.IntrFocalLength, i_IntrinsicCfg_rs.IntrFocalLength,{ { 0.0f, 0.0f } } };  // Values for K-matrix (focal length in x and y and principal point in camera coordinates) 
		const mecl::model::Pinhole<float32_t>::Config_s c_PinholeCfg_s = { c_Extrinsic_s, c_Intrinsic_s };
		c_Pinhole_s[cameraID].updateConfig_v(c_PinholeCfg_s);
		// Sensor
		const mecl::core::Point2D<float32_t>::Config_s c_PszPxCfg_s = { { i_IntrinsicCfg_rs.IntrPixelSizeX, i_IntrinsicCfg_rs.IntrPixelSizeY } };
		const mecl::core::Point2D<float32_t>::Config_s c_PppPxCfg_s = { { i_IntrinsicCfg_rs.IntrPpX, i_IntrinsicCfg_rs.IntrPpY } };
		const  mecl::model::Sensor<float32_t>::Config_s c_sensorCfg_s = { c_PppPxCfg_s, c_PszPxCfg_s, mecl::model::e_UpperLeft,{ { 0.0F, 0.0F } } ,{ { i_IntrinsicCfg_rs.IntrWidth - 1.0F, i_IntrinsicCfg_rs.IntrHeight - 1.0F } } };
		c_Sensor_s[cameraID].updateConfig_v(c_sensorCfg_s);

		// Camera
		return mecl::model::Camera<float32_t>(c_Pinhole_s[cameraID], c_Lens_as[cameraID], c_Sensor_s[cameraID]);

	}


uint16_t SensorCfg_Image2World_u16(mecl::model::Camera<float32_t> i_Camera_o, uint8_t* o_Dst_pu8, const uint8_t* i_Image_pu8, bool_t isImageFlipped_b)
{
  mecl::core::Point2D<float32_t> v_Output_x;
#ifdef OPENCV_OUT

  //cv::Mat in_img(vpcs::c_ImageHeight_u32, vpcs::c_ImageWidth_u32, CV_8UC1);
  IplImage * in_img;
  in_img = cvCreateImage(cvSize(vpcs::c_ImageWidth_u32, vpcs::c_ImageHeight_u32), IPL_DEPTH_8U, 1);
  
#endif
  for (uint32_t i = 0; i < vpcs::c_ImageWidth_u32; i++)
  {
    for (uint32_t j = 0; j < vpcs::c_ImageHeight_u32; j++)
    {
#ifdef OPENCV_OUT
		//in_img.data[i][j]=i_Image_pu8[i + j*vpcs::c_ImageWidth_u32];
		in_img->imageData[i + j*vpcs::c_ImageWidth_u32] = i_Image_pu8[i + j*vpcs::c_ImageWidth_u32];;
#endif

      // PRQA S 2420 1 // bracing of initializer is correct
      const mecl::core::Point2D<float32_t>::Config_s c_PoseCfg_s = { static_cast<float32_t>(i), static_cast<float32_t>(j) };
      const mecl::core::Point2D<float32_t> c_Pose_x(c_PoseCfg_s);
      mecl::core::Point2D<float32_t> v_MetrixPos_x;
      mecl::core::Point3D<float32_t> v_ImageUW_x;

      // undistort code
      i_Camera_o.pixelToMetric_v(c_Pose_x, v_MetrixPos_x);
      v_ImageUW_x.setPosX(v_MetrixPos_x.getPosX());
      v_ImageUW_x.setPosY(v_MetrixPos_x.getPosY());
      v_ImageUW_x.setPosZ(1.0F);
      i_Camera_o.applyDistortion_v(v_ImageUW_x, v_MetrixPos_x);
      i_Camera_o.metricToPixel_v(v_MetrixPos_x, v_Output_x);

      // my code

      const float32_t c_X_f32 = v_Output_x.getPosX();
      const float32_t c_Y_f32 = v_Output_x.getPosY();
      const float32_t c_Xover_f32 = c_X_f32 - static_cast<float32_t>(static_cast<sint32_t>(c_X_f32));
      const float32_t c_Yover_f32 = c_Y_f32 - static_cast<float32_t>(static_cast<sint32_t>(c_Y_f32));
      if (isImageFlipped_b)
      {
        //flip
        o_Dst_pu8[(vpcs::c_ImageWidth_u32 - 1 - i) + (vpcs::c_ImageHeight_u32 - 1 - j) * vpcs::c_ImageWidth_u32] = static_cast<uint8_t>(
          static_cast<float32_t>(i_Image_pu8[static_cast<sint32_t>(c_X_f32)        + static_cast<sint32_t>(c_Y_f32)        * vpcs::c_ImageWidth_u32]) * (1.0F - c_Xover_f32)*(1.0F - c_Yover_f32) +
          static_cast<float32_t>(i_Image_pu8[static_cast<sint32_t>(c_X_f32 + 1.0F) + static_cast<sint32_t>(c_Y_f32)        * vpcs::c_ImageWidth_u32]) * (c_Xover_f32)       *(1.0F - c_Yover_f32) +
          static_cast<float32_t>(i_Image_pu8[static_cast<sint32_t>(c_X_f32)        + static_cast<sint32_t>(c_Y_f32 + 1.0F) * vpcs::c_ImageWidth_u32]) * (1.0F - c_Xover_f32)*(c_Yover_f32)        +
          static_cast<float32_t>(i_Image_pu8[static_cast<sint32_t>(c_X_f32 + 1.0F) + static_cast<sint32_t>(c_Y_f32 + 1.0F) * vpcs::c_ImageWidth_u32]) * (c_Xover_f32)       *(c_Yover_f32));
      }
      else
      {
        //copy
        o_Dst_pu8[i + j * vpcs::c_ImageWidth_u32] = static_cast<uint8_t>(
          static_cast<float32_t>(i_Image_pu8[static_cast<sint32_t>(c_X_f32)        + static_cast<sint32_t>(c_Y_f32)        * vpcs::c_ImageWidth_u32]) * (1.0F - c_Xover_f32)*(1.0F - c_Yover_f32) +
          static_cast<float32_t>(i_Image_pu8[static_cast<sint32_t>(c_X_f32 + 1.0F) + static_cast<sint32_t>(c_Y_f32)        * vpcs::c_ImageWidth_u32]) * (c_Xover_f32)       *(1.0F - c_Yover_f32) +
          static_cast<float32_t>(i_Image_pu8[static_cast<sint32_t>(c_X_f32)        + static_cast<sint32_t>(c_Y_f32 + 1.0F) * vpcs::c_ImageWidth_u32]) * (1.0F - c_Xover_f32)*(c_Yover_f32)        +
          static_cast<float32_t>(i_Image_pu8[static_cast<sint32_t>(c_X_f32 + 1.0F) + static_cast<sint32_t>(c_Y_f32 + 1.0F) * vpcs::c_ImageWidth_u32]) * (c_Xover_f32)       *(c_Yover_f32));
      }
    }
  }
//#ifdef OPENCV_OUT
#if 1
  cvShowImage("sensor in_img", in_img);
  cv::waitKey(0);
#endif
  return 0;
}
mecl::core::Point2D<float32_t> Sensorcfg::SensorCfg_Unwarp2Metric_u16(const mecl::model::Camera<float32_t> v_Camera_ps, const mecl::core::Point2D<float32_t>& warped_image_point_f32)
{
	mecl::core::Point2D < float32_t > unwarped_image_point_f32;
	// PRQA S 2420 1 // bracing of initializer is correct
	mecl::core::Point2D<float32_t>::Config_s c_PoseCfg_s = { warped_image_point_f32.getPosX(), warped_image_point_f32.getPosY() };
	mecl::core::Point2D<float32_t> c_Pose_x(c_PoseCfg_s);
	mecl::core::Point2D<float32_t> v_MetrixPos_x;
	mecl::core::Point3D<float32_t> v_ImageUW_x;

	// undistort code
	v_Camera_ps.pixelToMetric_v(c_Pose_x, v_MetrixPos_x);
	v_ImageUW_x.setPosX(v_MetrixPos_x.getPosX());
	v_ImageUW_x.setPosY(v_MetrixPos_x.getPosY());
	v_ImageUW_x.setPosZ(1.0F);
	v_Camera_ps.applyDistortion_v(v_ImageUW_x, v_MetrixPos_x);
	v_Camera_ps.metricToPixel_v(v_MetrixPos_x, unwarped_image_point_f32);

	// my code

	/*float32_t c_X_f32 = unwarped_image_point_f32.getPosX();
	float32_t c_Y_f32 = unwarped_image_point_f32.getPosY();*/


	//v_Camera_ps.pixelToMetric_v(warped_image_point_f32, unwarped_image_point_f32);
	//mecl::core::Point3D < float32_t > v_Pos3D_rx;
	//v_Camera_ps.applyUndistortion_v(unwarped_image_point_f32, v_Pos3D_rx);
	//// normalize and convert to pixel from Metric
	//const float32_t c_W_f32 = v_Pos3D_rx.getW();

	//if (mecl::math::numeric_limits < float32_t > ::epsilon_x() < mecl::math::abs_x(c_W_f32))
	//{
	//	v_Camera_ps.applyNormalization_v(v_Pos3D_rx, 1.0F, unwarped_image_point_f32);
	//}
	//else
	//{
	//	unwarped_image_point_f32(0) = 0.0F;
	//	unwarped_image_point_f32(1) = 0.0F;
	//}

	// UnWarped
#ifdef _WINDOWS
	log_printf(" Warped -> Unwarped: %f, %f -> %f, %f\n", warped_image_point_f32.getPosX(), warped_image_point_f32.getPosY(), unwarped_image_point_f32.getPosX(), unwarped_image_point_f32.getPosY());
#endif
	return unwarped_image_point_f32;
}

};
