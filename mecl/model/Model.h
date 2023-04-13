///--------------------------------------------------------------------------
/// @file Model.h
/// @brief This is the short description of the template module
///
/// Here may follow a longer description for the template module with examples.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Sebastian Pliefke (sebastian.pliefke@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{
/// @brief Magna Electronics Model Library
/// 
/// @section Camera Camera Model
/// The camera model related classes are modeling real camera and lens system 
/// to map point locations between world and imager coordinate system. 
///
/// @par Example of creation of camera model object
/// A camera model object conists of a imager (most pinhole model based) and a
/// lens object. These two objects needs to be parametriesed depending of the 
/// utilized implementation class.
// PRQA S 1051 42 reason: code example, no source code commented
/// @code{.cpp}
/// // configurate pinhole model
///	mecl::model::Pinhole<float32_t>::Config_s pinholeCfg;
///	pinholeCfg.intrinsic_s.ppp_o(0,0)			= (1280-1)/2.0f;
/// pinholeCfg.intrinsic_s.ppp_o(1,0)			= ( 800-1)/2.0f;
/// pinholeCfg.intrinsic_s.focl_f32				= 1.04f;
/// pinholeCfg.intrinsic_s.psz_o(0,0)			= 0.003f;
/// pinholeCfg.intrinsic_s.psz_o(1,0)			= 0.003f;
/// pinholeCfg.extrinsic_s.pos_o(0,0)			= 10;
/// pinholeCfg.extrinsic_s.pos_o(1,0)			= 20;
/// pinholeCfg.extrinsic_s.pos_o(2,0)			= 30;
/// pinholeCfg.extrinsic_s.eulerAngles_s.yaw_o  = 10.0f/180*mecl::math::getPi_x<float32_t>();
/// pinholeCfg.extrinsic_s.eulerAngles_s.pitch_o= 20.0f/180*mecl::math::getPi_x<float32_t>();
/// pinholeCfg.extrinsic_s.eulerAngles_s.roll_o = 30.0f/180*mecl::math::getPi_x<float32_t>();
/// pinholeCfg.extrinsic_s.preRoll_e      = mecl::model::e_PreRoll90;
/// mecl::model::Pinhole<float32_t> pinhole(pinholeCfg);
/// pinhole.init_v();
///
/// // configurate lens model
///	mecl::model::LensRadial<float32_t>::Config_s lensCfg;
///	lensCfg.world2image_t[0]					=  0.0f;
///	lensCfg.world2image_t[1]					=  0.887869903810519f;
///	lensCfg.world2image_t[2]					=  0.114438979514063f;
///	lensCfg.world2image_t[3]					= -0.136796572418897f;
///	lensCfg.world2image_t[4]					=  0.227954940480195f;
///	lensCfg.world2image_t[5]					= -0.086004646558784f;
/// mecl::model::Lens<float32_t> myLens(lensCfg);
///
/// // configurate camera
/// mecl::model::Camera<float32_t> rCamera (pinhole, myLens);
/// @endcode
///
/// @par Example Project point (World --> Imager plane)
/// This example descripts the transformation of 3D points into the imager plane.
/// @code{.cpp}
/// mecl::core::Point4D<float32_t> PWorld; // input point
///	PWorld.setPosX(0.0f);
///	PWorld.setPosY(0.0f);
///	PWorld.setPosZ(0.0f);
///	PWorld.setPosW(1.0f);
///
/// mecl::core::Point2D<float32_t> PImager; // output point
/// rCamera.applyFullProjection_v(PWorld, 1.0, PImager);
/// @endcode

#ifndef MECL_MODEL_MODEL_H_
#define MECL_MODEL_MODEL_H_

// optical model
#include "Camera.h"
#include "LensRadial.h"
#include "LensCylinder.h"
#include "ModelTypes.h"
#include "Projection.h"
#include "Pinhole.h"
#include "SingleView.h"
#include "Sensor.h"

// trailer angle detection
#include "CarModel.h"
#include "TrailerModel.h"

#endif // MECL_MODEL_MODEL_H_
/// @}
/// @}
