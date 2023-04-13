//----------------------------------------------------------------------------
/// @file ViewGen_Hydra2.cpp
/// @brief Implementations for view generators and codecs for Hydra2RadialLens
/// 
/// This file contains all implementations view generators and codec used in Hydra2
/// plattform. (forward projection)
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
/// @date 12/14/2015
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#include "ViewGen_Hydra2.h"

#ifndef MECL_MODEL_VIEWGEN_HYDRA2_CPP_
#define MECL_MODEL_VIEWGEN_HYDRA2_CPP_

//! @cond
// PRQA S 1020 1 // macro is used by MKS
#define ViewGen_Hydra2_d_VERSION_ID "$Id: ViewGen_Hydra2.cpp 1.6 2016/12/06 04:47:13EST Schulte, Michael (MEE_MiSch) draft  $"
//! @endcond

namespace mecl {
namespace model {

// ----------------------------------------------------------------------------
/// @brief check if Hydra2 Undistortion ViewGen is configured
///
/// ViewGen is supposed to be configured if all conditions are met:
/// - configuration set by c-tor or by setter
/// - configured camera model refference unequal NULL
/// - lens and sensor model of the camera are configured
///
/// @return true, if configured, otherwise false
// ----------------------------------------------------------------------------
template<>
bool_t                              // return type
ViewGen< e_Hydra2RadialLens,        // ViewGen type
         e_LensUndistortion,        // Lookup table type
         core::Point3D<float32_t>,  // Type of decoded value
         uint32_t,                  // Type of encoded value
         core::Point2D<float32_t>,  // Type of value encoded by index value
         float32_t >                // Base type of camera
::isConfigured_b(void) const
{
  return (      this->lutConfigured_b
           &&  (this->lut_s.config_s.camera_po != NULL)
           &&   this->lut_s.config_s.camera_po->getLens_rx().isConfigured_b()
           &&   this->lut_s.config_s.camera_po->getSensor_rx().isConfigured_b()
         );
}

// ----------------------------------------------------------------------------
/// @brief initialize Hydra2 Undistortion Viewgen (produce lookup table)
///
/// Initialization includes generation of configured lookup table
/// and sets generated_b = true and info struct parameters.
// ----------------------------------------------------------------------------
template<>
void  ViewGen< e_Hydra2RadialLens,
               e_LensUndistortion,
               core::Point3D<float32_t>,
               uint32_t,
               core::Point2D<float32_t>,
               float32_t >
::init_v(void)
{
  // initialize principal point and codec

  AssertFunction(true == this->isConfigured_b(), "ViewGen is not configured.");

  Vgc_Hydra2_P3f32_u32 v_Vgc_P3f32_u32;

  core::Point2D<float32_t> v_Pos2d_f32;
  core::Point2D<float32_t> v_Ppp_f32(this->lut_s.config_s.camera_po->getSensor_rx().getPpp_rx());
  core::Point3D<float32_t> v_Pos3d_f32, v_Pos3dComp_f32;

  v_Ppp_f32.copyConfig_v(this->lut_s.info_s.pppCfg_f32);
  this->lut_s.config_s.camera_po->getSensor_rx().pixelToMetric_v(v_Ppp_f32, v_Ppp_f32);

  // calculate offsets and update lookup table info

  this->lut_s.info_s.startAddr_pu32 = this->lut_s.config_s.addr_pu32;

  this->lut_s.info_s.numSupportPoints_u32 = (    this->lut_s.config_s.resHoriz_u32
                                              +  this->lut_s.config_s.stepSize_u32 - 1
                                            ) /  this->lut_s.config_s.stepSize_u32
                                            + 1;

  this->lut_s.info_s.numSupportLines_u32 = (   this->lut_s.config_s.resVert_u32
                                             + this->lut_s.config_s.stepSize_u32 - 1
                                           ) / this->lut_s.config_s.stepSize_u32
                                           + 1;

  // write entries

  uint32_t * const c_AddrEnd_pu32 = &this->lut_s.config_s.addr_pu32[this->lut_s.config_s.maxSize_u32 -1];
  uint32_t* v_Addr_pu32 = this->lut_s.config_s.addr_pu32;
  float32_t v_SumError_f32 = math::constants<float32_t>::zero_x();

  this->lut_s.info_s.maxError_f32 = math::constants<float32_t>::zero_x();
  this->lut_s.info_s.maxErrorIdx_u32 = 0U;


  for (uint32_t v_SupportLine_u32 = 0U;
      v_SupportLine_u32 < this->lut_s.info_s.numSupportLines_u32;
      v_SupportLine_u32++)
  {
    for (uint32_t v_SupportPoint_u32 = 0U;
        v_SupportPoint_u32 < this->lut_s.info_s.numSupportPoints_u32;
        v_SupportPoint_u32++)
    {
      AssertMsg(v_Addr_pu32 <= c_AddrEnd_pu32, "Memory offset is out of bounds.");

      v_Pos2d_f32(0) =   static_cast<float32_t>(v_SupportPoint_u32 * this->lut_s.config_s.stepSize_u32);
      v_Pos2d_f32(1) =   static_cast<float32_t>(v_SupportLine_u32  * this->lut_s.config_s.stepSize_u32);

      this->lut_s.config_s.camera_po->pixelToMetric_v(v_Pos2d_f32, v_Pos2d_f32);
      this->lut_s.config_s.camera_po->applyUndistortion_v(v_Pos2d_f32, v_Pos3d_f32);

      v_Vgc_P3f32_u32.encode_v(v_Pos3d_f32, *v_Addr_pu32);

      // calculate error if flag enabled in config
      if (true == this->lut_s.config_s.calcError_b)
      {
        const SideFaces_e     c_FaceSide_e =
            v_Pos2d_f32(0) >= v_Ppp_f32(0) ? e_RightSide : e_LeftSide;
        const VerticalFaces_e c_FaceVert_e =
            v_Pos2d_f32(1) >= v_Ppp_f32(1) ? e_BottomSide : e_TopSide;

        v_Vgc_P3f32_u32.setFace_v(c_FaceSide_e, c_FaceVert_e);
        v_Vgc_P3f32_u32.decode_v(*v_Addr_pu32, v_Pos3dComp_f32);
        float32_t v_AbsError_f32= (v_Pos3d_f32.getCubeCoords_x() - v_Pos3dComp_f32).norm();

        if (v_AbsError_f32 > this->lut_s.info_s.maxError_f32)
        {
          this->lut_s.info_s.maxError_f32 = v_AbsError_f32;
          this->lut_s.info_s.maxErrorIdx_u32  = v_Addr_pu32 - &this->lut_s.config_s.addr_pu32[0];
        }

        v_SumError_f32 += v_AbsError_f32;
      }
      v_Addr_pu32++;
    }
  }

  this->lut_s.info_s.endAddr_pu32 = v_Addr_pu32;
  if (true == this->lut_s.config_s.calcError_b)
  {
    const uint32_t c_LutSize_u32 = v_Addr_pu32 - this->lut_s.info_s.startAddr_pu32;
    this->lut_s.info_s.avgError_f32 = v_SumError_f32 / c_LutSize_u32;
  }
  this->generated_b = true;

  return;
};

// ----------------------------------------------------------------------------
/// @brief Get encoded data of lookup table entry
///
/// Gets the encoded value of the lookup table entry of index \i_Index_u32
/// For the view Hydra2 Undistortion view generator this has uint32_t type.
///
/// @param[in] i_Index_u32  Index of entry in lookup table
/// @return Lookup table entry encoded
// ----------------------------------------------------------------------------
template<>
uint32_t                            // return type
ViewGen< e_Hydra2RadialLens,        // ViewGen type
         e_LensUndistortion,        // Lookup table type
         core::Point3D<float32_t>,  // Type of decoded value
         uint32_t,                  // Type of encoded value
         core::Point2D<float32_t>,  // Type of value encoded by index value
         float32_t >                // Base type of camera
::getEncoded_x(const uint32_t i_Index_u32) const
{
  AssertFunction (true == this->generated_b, "ViewGen is not initialized.");
  return this->lut_s.config_s.addr_pu32[i_Index_u32];
};

// ----------------------------------------------------------------------------
/// @brief Get decoded data element of lookup table entry with given index
///
/// For the view Hydra2 Undistortion view generator this yields the 2D pixel
/// coordinates associated with lookup table entry of index \i_Idx_u32.
///
/// @param[in] i_Idx_u32 Index of entry in lookup table
/// @return Point location of the \i_Idx_u32 - th entry in lookup table
// ----------------------------------------------------------------------------
template<>
core::Point2D<float32_t>            // return type
ViewGen< e_Hydra2RadialLens,        // ViewGen type
         e_LensUndistortion,        // Lookup table type
         core::Point3D<float32_t>,  // Type of decoded value
         uint32_t,                  // Type of encoded value
         core::Point2D<float32_t>,  // Type of value encoded by index value
         float32_t >
::getIndexedValue_x(const uint32_t i_Idx_u32) const
{
  AssertFunction (true == this->isConfigured_b(), "ViewGen is not configured.");


  const uint32_t c_SupportPoints_u32 = (   this->lut_s.config_s.resHoriz_u32
                                         + this->lut_s.config_s.stepSize_u32 - 1
                                       ) /  this->lut_s.config_s.stepSize_u32
                                       + 1;

  const uint32_t c_HMod_u32 = i_Idx_u32 %  c_SupportPoints_u32;
  const uint32_t c_VDiv_u32 = (i_Idx_u32 - c_HMod_u32) / c_SupportPoints_u32 ;

  return   core::Point2D<float32_t>(static_cast<float32_t>( c_HMod_u32 * this->lut_s.config_s.stepSize_u32),
                                    static_cast<float32_t>( c_VDiv_u32 * this->lut_s.config_s.stepSize_u32) );
};

// ----------------------------------------------------------------------------
/// @brief check if Hydra2 Distortion ViewGen is configured
///
/// ViewGen is supposed to be configured if all conditions are met:
/// - configuration set by c-tor or by setter
/// - configured camera model refference unequal NULL
/// - lens and sensor model of the camera are configured
///
/// @return true, if configured, otherwise false
// ----------------------------------------------------------------------------
template<>
bool_t                              // return type
ViewGen< e_Hydra2RadialLens,        // ViewGen type
         e_LensDistortion,          // Lookup table type
         core::Point2D<float32_t>,  // Type of decoded value
         uint32_t,                  // Type of encoded value
         core::Point3D<float32_t>,  // Type of value encoded by index value
         float32_t >                // Base type of camera
::isConfigured_b(void) const
{
  return (      this->lutConfigured_b
           &&  (this->lut_s.config_s.camera_po != NULL)
           &&   this->lut_s.config_s.camera_po->getLens_rx().isConfigured_b()
           &&  (   (false == this->lut_s.config_s.applySensor_b)
                || (true  == this->lut_s.config_s.camera_po->getSensor_rx().isConfigured_b())
               )
         );
}

// ----------------------------------------------------------------------------
/// @brief initialize Hydra2 Distortion Viewgen (produce lookup table)
///
/// Initialization includes generation of configured lookup table
/// and sets generated_b = true and info struct parameters.
// ----------------------------------------------------------------------------
template<>
void  ViewGen< e_Hydra2RadialLens,
               e_LensDistortion,
               core::Point2D<float32_t>,
               uint32_t,
               core::Point3D<float32_t>,
               float32_t >
::init_v(void)
{

  // check if ViewGen is configured
  AssertFunction(true == this->isConfigured_b(), "ViewGen is not configured.");

  // intialize codec

  const Vgc_Hydra2_P2f32_u32 v_Vgc_P2f32_u32( this->lut_s.config_s.camera_po->getSensor_rx() , this->lut_s.config_s.applySensor_b);

  float32_t v_SumError_f32 = math::constants<float32_t>::zero_x();

   this->lut_s.info_s.maxError_f32 = math::numeric_limits<float32_t>::min_x();
   this->lut_s.info_s.maxErrorIdx_u32 = 0U;
   this->lut_s.info_s.minHoriz_f32 = math::numeric_limits<float32_t>::max_x();
   this->lut_s.info_s.maxHoriz_f32 = math::numeric_limits<float32_t>::lowest_x();
   this->lut_s.info_s.minVert_f32 = math::numeric_limits<float32_t>::max_x();
   this->lut_s.info_s.maxVert_f32 = math::numeric_limits<float32_t>::lowest_x();

  // calculate offsets and update lookup table info
  uint32_t* v_Addr_pu32 = this->lut_s.config_s.addr_pu32;
  this->lut_s.info_s.frontFaceStartAddr_pu32  = this->lut_s.config_s.addr_pu32;
  uint32_t * const c_AddrEnd_pu32 = &this->lut_s.config_s.addr_pu32[this->lut_s.config_s.maxSize_u32 -1];

  const float32_t c_StepWidth_f32 =
        math::constants<float32_t>::one_x() / static_cast<float32_t>(this->lut_s.config_s.spread_u32);

  // write entries for front face

  uint32_t v_NumFrontFaceEntries_u32 = 0U;

  for (uint32_t v = 0; v <= this->lut_s.config_s.spread_u32; v++)
  {
    const float32_t c_YPos_f32 = c_StepWidth_f32 * v;
    for (uint32_t u = 0; u <= this->lut_s.config_s.spread_u32; u++)
    {
      AssertFunction(v_Addr_pu32 != c_AddrEnd_pu32,"Error generating front face: Memory offset is out of bounds.");
      const float32_t c_XPos_f32 = c_StepWidth_f32 * u;
      const core::Point3D<float32_t> c_Pos3d_f32 (c_XPos_f32, c_YPos_f32, math::constants<float32_t>::one_x());
      core::Point2D<float32_t> v_Pos2d_f32;
      bool_t v_IsApplicable_b = true;
      this->lut_s.config_s.camera_po->applyDistortion_v(c_Pos3d_f32, v_Pos2d_f32, v_IsApplicable_b);
      if (not v_IsApplicable_b)
      {
        *v_Addr_pu32 = v_Vgc_P2f32_u32.getInvisibleValue_u32();
      }
      else
      {
        v_Vgc_P2f32_u32.encode_v(v_Pos2d_f32, *v_Addr_pu32);
      }

      if (v_Pos2d_f32(0) < this->lut_s.info_s.minHoriz_f32)
      {
        this->lut_s.info_s.minHoriz_f32 = v_Pos2d_f32(0);
      }
      if (v_Pos2d_f32(0) > this->lut_s.info_s.maxHoriz_f32)
      {
        this->lut_s.info_s.maxHoriz_f32 = v_Pos2d_f32(0);
      }

      if (v_Pos2d_f32(1) < this->lut_s.info_s.minVert_f32)
      {
        this->lut_s.info_s.minVert_f32 = v_Pos2d_f32(1);
      }
      if (v_Pos2d_f32(1) > this->lut_s.info_s.maxVert_f32)
      {
        this->lut_s.info_s.maxVert_f32 = v_Pos2d_f32(1);
      }



      // compute error (conditional)
      if (true == this->lut_s.config_s.calcError_b)
      {
        v_NumFrontFaceEntries_u32++;
        core::Point2D<float32_t> v_PosComp2d_f32;
        v_Vgc_P2f32_u32.decode_v(*v_Addr_pu32, v_PosComp2d_f32);
        const float32_t c_Error_f32 = (v_Pos2d_f32  - v_PosComp2d_f32).norm();
        v_SumError_f32 += c_Error_f32;

        if (c_Error_f32 > this->lut_s.info_s.maxError_f32)
        {
          this->lut_s.info_s.maxError_f32 = c_Error_f32;
          this->lut_s.info_s.maxErrorIdx_u32 = v_Addr_pu32 - &this->lut_s.config_s.addr_pu32[0];
        }

      }
      v_Addr_pu32++;
    }
  }
  if (true == this->lut_s.config_s.calcError_b)
  {
    this->lut_s.info_s.avgErrorFront_f32 = v_SumError_f32 / static_cast<float32_t> (v_NumFrontFaceEntries_u32);
    v_SumError_f32 = math::constants<float32_t>::zero_x();
  }

  // write entries for side faces (e_Left == e_Right)

  uint32_t v_NumSideFacesEntries_u32 = 0U;
  const uint32_t c_Spread2x_u32 = this->lut_s.config_s.spread_u32 * 2U;
  this->lut_s.info_s.sideFaceStartAddr_pu32 = v_Addr_pu32;
  for (uint32_t v=0; v <= this->lut_s.config_s.spread_u32; v++)
  {
    const float32_t c_YPos_f32 = c_StepWidth_f32 * v;
    for (uint32_t u=0; u <= c_Spread2x_u32; u++)
    {
      AssertFunction(v_Addr_pu32 != c_AddrEnd_pu32,"Error generating side faces: Memory offset is out of bounds.");
      const float32_t c_ZPos_f32 = c_StepWidth_f32 * u - math::constants<float32_t>::one_x();
      const mecl::core::Point3D<float32_t> c_Pos3d_f32 (math::constants<float32_t>::minusOne_x(), c_YPos_f32, c_ZPos_f32);
      mecl::core::Point2D<float32_t> v_Pos2d_f32;
      bool_t b_CheckApplicable_b = true;
      this->lut_s.config_s.camera_po->applyDistortion_v(c_Pos3d_f32, v_Pos2d_f32, b_CheckApplicable_b);
      if (not b_CheckApplicable_b)
      {
        *v_Addr_pu32 = v_Vgc_P2f32_u32.getInvisibleValue_u32();
      }
      else
      {
        v_Vgc_P2f32_u32.encode_v(v_Pos2d_f32, *v_Addr_pu32);
      }
      
      if (v_Pos2d_f32(0) < this->lut_s.info_s.minHoriz_f32)
      {
        this->lut_s.info_s.minHoriz_f32 = v_Pos2d_f32(0);
      }
      if (v_Pos2d_f32(0) > this->lut_s.info_s.maxHoriz_f32)
      {
        this->lut_s.info_s.maxHoriz_f32 = v_Pos2d_f32(0);
      }

      if (v_Pos2d_f32(1) < this->lut_s.info_s.minVert_f32)
      {
        this->lut_s.info_s.minVert_f32 = v_Pos2d_f32(1);
      }
      if (v_Pos2d_f32(1) > this->lut_s.info_s.maxVert_f32)
      {
        this->lut_s.info_s.maxVert_f32 = v_Pos2d_f32(1);
      }

      // compute error (conditional)
      if (true == this->lut_s.config_s.calcError_b)
      {
        v_NumSideFacesEntries_u32++;
        core::Point2D<float32_t> v_PosComp2d_f32;
        v_Vgc_P2f32_u32.decode_v(*v_Addr_pu32, v_PosComp2d_f32);
        const float32_t c_Error_f32 = (v_Pos2d_f32  - v_PosComp2d_f32).norm();
        v_SumError_f32 += c_Error_f32;

        if (c_Error_f32 > this->lut_s.info_s.maxError_f32)
        {
          this->lut_s.info_s.maxError_f32 = c_Error_f32;
          this->lut_s.info_s.maxErrorIdx_u32 = v_Addr_pu32 - &this->lut_s.config_s.addr_pu32[0];
        }
      }
      v_Addr_pu32++;
    }
  }

  if (true == this->lut_s.config_s.calcError_b)
   {
     this->lut_s.info_s.avgErrorSide_f32 = v_SumError_f32 / static_cast<float32_t> (v_NumSideFacesEntries_u32);
     v_SumError_f32 = math::constants<float32_t>::zero_x();
   }

  // write entries for vertical faces (e_Top == e_Bottom)

  uint32_t v_NumVerticalFacesEntries_u32 = 0U;
  this->lut_s.info_s.verticalFaceStartAddr_pu32 = v_Addr_pu32;

  for (uint32_t v=0; v <= c_Spread2x_u32 ; v++)
  {
    const float32_t c_ZPos_f32 = c_StepWidth_f32 * v - math::constants<float32_t>::one_x();
    for (uint32_t u=0; u <= this->lut_s.config_s.spread_u32; u++)
    {
      AssertFunction(v_Addr_pu32 != c_AddrEnd_pu32,"Error generating vertical faces: Memory offset is out of bounds.");
      const float32_t c_XPos_f32 = c_StepWidth_f32 * u;
      const mecl::core::Point3D<float32_t> c_Pos3d_f32 (c_XPos_f32, math::constants<float32_t>::minusOne_x(), c_ZPos_f32);
      mecl::core::Point2D<float32_t> v_Pos2d_f32;
      bool_t b_CheckApplicable_b = true;
      this->lut_s.config_s.camera_po->applyDistortion_v(c_Pos3d_f32, v_Pos2d_f32, b_CheckApplicable_b);
      if (not b_CheckApplicable_b)
      {
        *v_Addr_pu32 = v_Vgc_P2f32_u32.getInvisibleValue_u32();
      }
      else
      {
        v_Pos2d_f32 = -v_Pos2d_f32;
        v_Vgc_P2f32_u32.encode_v(v_Pos2d_f32, *v_Addr_pu32);
      }

      if (v_Pos2d_f32(0) < this->lut_s.info_s.minHoriz_f32)
      {
        this->lut_s.info_s.minHoriz_f32 = v_Pos2d_f32(0);
      }
      if (v_Pos2d_f32(0) > this->lut_s.info_s.maxHoriz_f32)
      {
        this->lut_s.info_s.maxHoriz_f32 = v_Pos2d_f32(0);
      }

      if (v_Pos2d_f32(1) < this->lut_s.info_s.minVert_f32)
      {
        this->lut_s.info_s.minVert_f32 = v_Pos2d_f32(1);
      }
      if (v_Pos2d_f32(1) > this->lut_s.info_s.maxVert_f32)
      {
        this->lut_s.info_s.maxVert_f32 = v_Pos2d_f32(1);
      }

      // compute error (conditional)
      if (true == this->lut_s.config_s.calcError_b)
      {
        v_NumVerticalFacesEntries_u32++;
        core::Point2D<float32_t> v_PosComp2d_f32;
        v_Vgc_P2f32_u32.decode_v(*v_Addr_pu32, v_PosComp2d_f32);
        const float32_t c_Error_f32 = (v_Pos2d_f32  - v_PosComp2d_f32).norm();
        v_SumError_f32 += c_Error_f32;

        if (c_Error_f32 > this->lut_s.info_s.maxError_f32)
        {
          this->lut_s.info_s.maxError_f32 = c_Error_f32;
          this->lut_s.info_s.maxErrorIdx_u32 = v_Addr_pu32 - &this->lut_s.config_s.addr_pu32[0];
        }
      }
      v_Addr_pu32++;
    }
  }

  if (true == this->lut_s.config_s.calcError_b)
  {
    this->lut_s.info_s.avgErrorVertical_f32 = v_SumError_f32 / static_cast<float32_t> (v_NumVerticalFacesEntries_u32);
    this->lut_s.info_s.avgError_f32 = (  this->lut_s.info_s.avgErrorFront_f32
                                       + this->lut_s.info_s.avgErrorSide_f32
                                       + this->lut_s.info_s.avgErrorVertical_f32
                                      ) / 3.0F;
  }

  this->lut_s.info_s.stepWidth_f32 = c_StepWidth_f32;
  this->lut_s.info_s.endAddr_pu32 = v_Addr_pu32;

  this->generated_b = true;

  return;
};

// ----------------------------------------------------------------------------
/// @brief Get encoded data of lookup table entry
///
/// Gets the encoded value of the lookup table entry of index \i_Index_u32
/// For the Hydra2 Distortion view generator this has uint32_t type.
///
/// @param[in] i_Index_u32  Index of entry in lookup table
/// @return Lookup table entry encoded
// ----------------------------------------------------------------------------
template<>
uint32_t                            // return type
ViewGen< e_Hydra2RadialLens,        // ViewGen type
         e_LensDistortion,          // Lookup table type
         core::Point2D<float32_t>,  // Type of decoded value
         uint32_t,                  // Type of encoded value
         core::Point3D<float32_t>,  // Type of value encoded by index value
         float32_t >                // Base type of camera
::getEncoded_x(const uint32_t i_Index_u32) const
{
  AssertFunction (true == this->generated_b, "ViewGen is not initialized.");
  return this->lut_s.config_s.addr_pu32[i_Index_u32];
};

//  ----------------------------------------------------------------------------
/// @brief Get decoded data element of lookup table entry with given index
///
/// For the view Hydra2 Undistortion view generator this yields the 2D pixel
/// coordinates associated with lookup table entry of index \i_Idx_u32.
///
/// @param[in] i_Idx_u32 Index of entry in lookup table
/// @return Point location of the \i_Idx_u32 - th entry in lookup table
// ----------------------------------------------------------------------------
template<>
core::Point3D<float32_t>            // return type
ViewGen< e_Hydra2RadialLens,        // ViewGen type
         e_LensDistortion,          // Lookup table type
         core::Point2D<float32_t>,  // Type of decoded value
         uint32_t,                  // Type of encoded value
         core::Point3D<float32_t>,  // Type of value encoded by index value
         float32_t >                // Base type of camera
::getIndexedValue_x(const uint32_t i_Idx_u32) const
{

  AssertFunction (true == this->isConfigured_b(), "ViewGen is not configured.");

  const uint32_t c_SpreadSqr_u32 = this->lut_s.config_s.spread_u32 * this->lut_s.config_s.spread_u32;
  const uint32_t c_SpreadSqr2x_u32 = c_SpreadSqr_u32 * 2U;

  const float32_t c_StepWidth_f32 =
          math::constants<float32_t>::one_x() / static_cast<float32_t>(this->lut_s.config_s.spread_u32);

  core::Point3D<float32_t> v_Pos3d_f32(math::constants<float32_t>::minusOne_x());

  if (i_Idx_u32 <= c_SpreadSqr_u32) // front face
  {
    v_Pos3d_f32(0) += static_cast<float32_t>(i_Idx_u32 / this->lut_s.config_s.spread_u32) * c_StepWidth_f32;
    v_Pos3d_f32(1) += static_cast<float32_t>(i_Idx_u32 % this->lut_s.config_s.spread_u32) * c_StepWidth_f32;
    v_Pos3d_f32(2) = math::constants<float32_t>::one_x();

  }
  else if (i_Idx_u32 <= c_SpreadSqr_u32 + c_SpreadSqr2x_u32) // side faces (left face)
  {
    v_Pos3d_f32(0) = math::constants<float32_t>::minusOne_x();
    v_Pos3d_f32(1) += static_cast<float32_t>(i_Idx_u32 / this->lut_s.config_s.spread_u32) * c_StepWidth_f32;
    v_Pos3d_f32(2) += static_cast<float32_t>(i_Idx_u32 % this->lut_s.config_s.spread_u32) * c_StepWidth_f32;
  }
  else if (i_Idx_u32 <= c_SpreadSqr_u32 + c_SpreadSqr2x_u32 + c_SpreadSqr2x_u32) // vertical faces (top face)
  {
    v_Pos3d_f32(0) += static_cast<float32_t>(i_Idx_u32 % this->lut_s.config_s.spread_u32) * c_StepWidth_f32;
    v_Pos3d_f32(1) = math::constants<float32_t>::minusOne_x();
    v_Pos3d_f32(2) += static_cast<float32_t>(i_Idx_u32 / this->lut_s.config_s.spread_u32) * c_StepWidth_f32;
  }
  else // remark: no rear face assumed
  {
    AssertFunction(false, "Error: Lookup table index out of bounds.");
  }

  return v_Pos3d_f32;
};

} // namespace model
} // namespace mecl

#endif

/// @}
/// @}
