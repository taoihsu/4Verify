//--------------------------------------------------------------------------
/// @file KLTSensor.h
/// @brief declaration of methods
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

#ifndef _KLTSENSOR_H_
#define _KLTSENSOR_H_

#include "mecl/core/MeclTypes.h"
#include "mecl/model/Camera.h"
#include "World2ImageTypes.h"
#include "./src/Vpcs_types.h"

namespace SensorCfg
{
	class Sensorcfg
	{
	public:
		static mecl::model::Camera<float32_t> Create_Camera(const W2Image::SetupIntrinsicCfg_s & i_IntrinsicCfg_rs, const W2Image::SetupExtrinsicCfg_s & i_ExtrinsicCfg_rs, const uint8_t cameraID);
		static mecl::core::Point2D<float32_t> SensorCfg_Unwarp2Metric_u16(const mecl::model::Camera<float32_t> v_Camera_ps, const mecl::core::Point2D<float32_t>& warped_image_point_f32);
	};
	
  uint16_t SensorCfg_Image2World_u16(mecl::model::Camera<float32_t> i_Camera_o, uint8_t* o_Dst_pu8, const uint8_t* i_Image_pu8, bool_t i_IsImageFlipped_b);
};

#endif /* _KLTSENSOR_H_ */
