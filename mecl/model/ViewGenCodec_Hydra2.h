/*
 * ViewGenCodec_Hydra2.h
 *
 *  Created on: 08.03.2016
 *      Author: sai_hezol
 */
//----------------------------------------------------------------------------
/// @file ViewGenCodec_Hydra2.h
/// @brief Implementations for view generators codecs for Hydra2RadialLens
///
/// This file contains all implementations for view generator codecs used in Hydra2
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

#ifndef MODEL_VIEWGENCODEC_HYDRA2_H_
#define MODEL_VIEWGENCODEC_HYDRA2_H_

#include "ViewGen.h"
#include "mecl/core/FixedPoint.h"

namespace mecl {
namespace model {

// ----------------------------------------------------------------------------
/// @class ViewGenCodec< e_Hyra2RadialLens, core::Point2D<float32_t>, uint32_t >
/// @brief Codec used in Hydra2RadialLens view generator for distortion
///
/// Codec defines encoding/decoding of 2D point location into a 32bit unsigned integer.
///
/// @remark template specialization needs to be declare for complete class, since
/// specific method members have to be added here.
// ----------------------------------------------------------------------------

template<>
class ViewGenCodec< e_Hydra2RadialLens, core::Point2D<float32_t>, uint32_t >
{
public:

  /// Get encoded value for point not visible in sensor area
  static uint32_t getInvisibleValue_u32(void)
  {
    return 0x80008000U;
  }

  /// Default c-tor
  explicit ViewGenCodec(void)
  : sensor_f32(NULL)
  {}

  /// C-tor with Sensor
  explicit ViewGenCodec(const ISensor<float32_t>& i_Sensor_rf32, bool_t i_ApplySensor_b = true)
  : sensor_f32(i_Sensor_rf32.isConfigured_b()  && i_ApplySensor_b? &i_Sensor_rf32 : NULL)
  {
    AssertFunction(true == i_Sensor_rf32.isConfigured_b() || false == i_ApplySensor_b,
                  "Codec could not be initialized. Sensor not configured.");
  }

  /// Virtual destructor
  virtual ~ViewGenCodec(void) {};

  /// Encode point location to a 32bit unsigned integer
  /// @param[in] i_Pos_rx 2D point location
  /// @param[out] o_Val_rx encoded value as integer
  virtual void encode_v( const core::Point2D<float32_t>& i_Pos_rx, uint32_t& o_Val_rx) const
  {
    core::Point2D<float32_t> v_Pos_f32(i_Pos_rx * getScaleMetric_f32());
    if (NULL != this->sensor_f32)
    {
      sensor_f32->metricToPixel_v(i_Pos_rx, v_Pos_f32);
    }
    o_Val_rx = (static_cast<sint16_t>(v_Pos_f32(0)) << 16) | (static_cast<sint16_t>(v_Pos_f32(1)) & 0x0000ffff);
    return;
  }

  /// Decode 2D point location from 32bit unsigned integer
  /// @param[in] i_Val_rx encoded value as integer
  /// @param[out] i_Pos_rx decoded 2D point location
  virtual void decode_v( const uint32_t& i_Val_rx, core::Point2D<float32_t>& o_Pos_rx) const
  {
    o_Pos_rx(1) = static_cast<float32_t>( static_cast<sint16_t>(i_Val_rx & 0x0000ffff));
    o_Pos_rx(0) = static_cast<float32_t>( static_cast<sint16_t>(i_Val_rx >> 16U));
    o_Pos_rx /= this->getScaleMetric_f32();
    if (NULL != this->sensor_f32)
    {
      sensor_f32->pixelToMetric_v(o_Pos_rx, o_Pos_rx);
    }
    return;
  }

private:

  /// Get metric scale of fixed-point value
  static float32_t getScaleMetric_f32(void)
  {
    return static_cast<float32_t> (1 << 13);
  }

  const ISensor<float32_t>* sensor_f32;

};

// ----------------------------------------------------------------------------
/// @class ViewGenCodec< e_Hyra2RadialLens, core::Point3D<float32_t>, uint32_t >
/// @brief Codec used in Hydra2RadialLens view generator for undistortion
///
/// Codec defines encoding/decoding of 3D directions into a 32bit unsigned integer.
///
/// @remark template specialization needs to be declare for complete class, since
/// specific method members have to be added here.
// ----------------------------------------------------------------------------

template<>
class ViewGenCodec< e_Hydra2RadialLens, core::Point3D<float32_t>, uint32_t >
: IViewGenCodec< core::Point3D<float32_t>, uint32_t>
{
public:

  /// Default c-tor, set default values for members
  ViewGenCodec(void)
  : faceSide_e(e_LeftSide)
  , faceVertical_e(e_TopSide)
  {};

  /// Virtual destructor
  virtual ~ViewGenCodec(void) {};

  /// Encode 3D direction to 32bit unsigned integer
  /// @param[in] i_Pos_rx 3D direction as point coordinates
  /// @param[out o_Val_rx encoded value as integer
  virtual void encode_v( const core::Point3D<float32_t>& i_Pos_rx,
                                              uint32_t& o_Val_rx) const
  {

    uint32_t v_NormDim_u32 = 0U;
    core::Point3D<float32_t> v_Pos_x(i_Pos_rx.getCubeCoords_x(v_NormDim_u32));

    switch(v_NormDim_u32)
    {
      case 0U: // side
      {
        const uint16_t c_U_u16 = calcF32S15_u16(v_Pos_x(2));
        const uint16_t c_V_u16 = calcF32S15_u16(v_Pos_x(1));
        o_Val_rx = e_Side | (c_U_u16  << 15) | c_V_u16;
      }
      break;
      case 1U: // vertical
      {
        const uint16_t c_U_u16 = calcF32S15_u16(v_Pos_x(0));
        const uint16_t c_V_u16 = calcF32S15_u16(v_Pos_x(2));
        o_Val_rx = e_Vertical | (c_U_u16 << 15) | c_V_u16;
      }
      break;
      case 2U: // front or rear
      {
        const uint16_t c_U_u16 = calcF32S15_u16(v_Pos_x(0));
        const uint16_t c_V_u16 = calcF32S15_u16(v_Pos_x(1));
        o_Val_rx = ( v_Pos_x(2)>=0 ? e_Front : e_Rear ) | (c_U_u16 << 15) | (uint16_t) c_V_u16;
      }
      break;
      default:
        AssertFunction(false, "ViewGenCodec error.");
        break;
    }
    return;
  };

  /// Decode 3D cube coordinate from 32bit unsigned integer
  /// @param[in] i_Val_u32 encoded value as integer
  /// @param[in] o_Pos_rx decoded 3D cube coordinates
  /// @remark: decoded direction is ambigous in case face is not front. Use setFace_v() to
  /// pre-set side and vertical face.
  virtual void decode_v(               const uint32_t& i_Val_rx,
                               core::Point3D<float32_t>& o_Pos_rx) const
  {
    const uint32_t c_Face_e = i_Val_rx & 0xC0000000;
    // SF draft
    const sint32_t c_U_s32 = i_Val_rx<<2;
    const float32_t c_U_f32 = calcS15F32_f32(c_U_s32>>17);
    const sint32_t c_V_s32 = i_Val_rx<<17;
    const float32_t c_V_f32 = calcS15F32_f32(c_V_s32>>17);

    switch(c_Face_e)
    {
      case e_Front:
      {
        o_Pos_rx(0) = c_U_f32;
        o_Pos_rx(1) = c_V_f32;
        o_Pos_rx(2) = 1.0f;
      }
      break;

      case e_Rear:
      {
        o_Pos_rx(0) = c_U_f32;
        o_Pos_rx(1) = c_V_f32;
        o_Pos_rx(2) = -1.0f;
      }
      break;

      case e_Side:
      {
        o_Pos_rx(0) = this->faceSide_e == e_RightSide ? 1.0f : -1.0f;
        o_Pos_rx(1) = c_V_f32;
        o_Pos_rx(2) = c_U_f32;
      }
      break;

      case e_Vertical:
      {
        o_Pos_rx(0) = c_U_f32;
        o_Pos_rx(1) = this->faceVertical_e == e_BottomSide ? 1.0f : -1.0f;
        o_Pos_rx(2) = c_V_f32;
      }
      break;

      default:
        AssertFunction(false, "ViewGenCodec error: Cube face not defined");
        break;
    }
    return;
  }

  /// Tell decoder which face to select if decoded cube face is ambiguous
  /// @param[in] i_FaceSide_e Face to select if face = e_Side (e_Right or e_Left)
  /// @param[in] i_FaceVertical_e Face to select if face = e_Vertical (e_Top or e_Bottom)
  void setFace_v(const SideFaces_e i_FaceSide_e, const VerticalFaces_e i_FaceVertical_e)
  {
    this->faceSide_e = i_FaceSide_e;
    this->faceVertical_e = i_FaceVertical_e;
  }

private:

  SideFaces_e         faceSide_e;   //< store current setting for side to select if face = e_Side
  VerticalFaces_e faceVertical_e;   //< store current setting for side to select if face = e_Vertical

  static const uint32_t c_FracBits_u32 = 13U; 
  typedef mecl::core::FixedPoint<15-c_FracBits_u32, c_FracBits_u32> Fp_t;

  /// calculate 16-bit fixed-point value from 32-bit float number
  static uint16_t calcF32S15_u16(float32_t i_Val_f32)
  {
    Fp_t fp_t(i_Val_f32);
    return fp_t.getBits_x();
  }

  /// calculate 32-bit float number from 16-bit fixed-point value
  static float32_t calcS15F32_f32(sint16_t i_Val_s16)
  {
    Fp_t fp_t(i_Val_s16);
    return fp_t.toFloat_f32();
  }

};

}
}

#endif /* MODEL_VIEWGENCODEC_HYDRA2_H_ */

/// @}
/// @}
