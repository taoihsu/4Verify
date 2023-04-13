/*
 * SensorVarScale.hpp
 *
 *  Created on: 06.12.2016
 *      Author: sai_hezol
 */
//--------------------------------------------------------------------------
/// @file Sensor.hpp
/// @brief Implementation of standard sensor model in MECL.
/// The Sensor object provides functionality for conversion between metric and
/// pixel coordinate spaces based on principal point and resolution.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
/// @date 10/12/2015
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef SRC_MODEL_SENSORVARSCALE_HPP_
#define SRC_MODEL_SENSORVARSCALE_HPP_

#include "SensorVarScale.h"

namespace mecl
{
namespace model
{

// --------------------------------------------------------------------------
/// @brief Default constructor
///
/// @par Example use:
/// @snippet ModelTester.cpp Sensor_Constructor1
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
SensorVarScale<T, SType_e, RLen_u32>::SensorVarScale(void)
: sensorUniform_x()
, varScaleBuffer_px(NULL)
, varScaleBufferSize_u32(0U)
, revMetricStartPos_x(math::constants<T>::zero_x())
, stepSizeMetric_x(math::constants<T>::zero_x())
, initialized_b(false)
, configured_b(false)
{}

// --------------------------------------------------------------------------
/// @brief Constructor copying configuration from existing configuration data set
///
/// @param[in] i_Config_rs Sensor model configuration
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
SensorVarScale<T, SType_e, RLen_u32>::SensorVarScale(const Config_s& i_Config_rs)
: sensorUniform_x(i_Config_rs.sensorUniform_s)
, varScaleBuffer_px(i_Config_rs.varScaleBuffer_px)
, varScaleBufferSize_u32(i_Config_rs.varScaleBufferSize_u32)
, revScaleBuffer_px()
, revMetricStartPos_x(math::constants<T>::zero_x())
, stepSizeMetric_x(math::constants<T>::zero_x())
, initialized_b(false)
, configured_b(true)
{}

// --------------------------------------------------------------------------
/// @brief Constructor initializing  uniform sensor + VarScale Buffer/Size
///
/// @param[in] i_Config_rs Sensor model configuration
/// @param[in] i_Sensor_rx Pointer to first entry of the variable scaling buffer (array)
/// @param[in] i_Sensor_rx Size of the array = number of enties within scaling buffer
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
SensorVarScale<T, SType_e, RLen_u32>::SensorVarScale(const Sensor<T>& i_Sensor_rx,
                                           T* i_VarScaleBuffer_px,
                                           const uint32_t i_VarScaleBufferSize_u32)
: sensorUniform_x(i_Sensor_rx)
, varScaleBuffer_px(i_VarScaleBuffer_px)
, varScaleBufferSize_u32(i_VarScaleBufferSize_u32)
, revMetricStartPos_x(math::constants<T>::zero_x())
, stepSizeMetric_x(math::constants<T>::zero_x())
, initialized_b(false)
, configured_b(i_Sensor_rx.isConfigured_b())
{}

// --------------------------------------------------------------------------
/// @brief Initialize SensorVarScale instance
///
/// Initialization re-calculates variable scale scale buffer, passed over by
/// c-tor or configuration update, and calculates reverse scale buffer.
///
/// @remark Private method is called by init_v()
/// @attention Buffer is not a class member, but get's also altered after init
///
/// @return void
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::init_v(void)
{
  AssertFunction(true == this->isConfigured_b(), "Cannot initialize SensorVarScale Buffers. Not configured.");

  this->calcVarScaleBuffer_v();
  this->calcRevScaleBuffer_v();
  this->initialized_b = true;
  return;
}

// --------------------------------------------------------------------------
/// @brief Calculate variable scaling buffer (called by init_v()
///
/// Calculate variable scaling buffer by replacing zero entry by interpolated value.
///
/// @remark Private method is called by init_v()
/// @attention Buffer is not a class member, but get's also altered after init
///
/// @return void
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::calcVarScaleBuffer_v(void)
{
  const T c_ZeroVal_x = static_cast<T>(0.0f);

  T v_VarScale_x = this->varScaleBuffer_px[0];
  if (v_VarScale_x == c_ZeroVal_x)
  {
    v_VarScale_x = static_cast<T>(1.0f);
  }

  for (uint32_t v_Idx_u32 = 0U; v_Idx_u32 < varScaleBufferSize_u32; ++v_Idx_u32)
  {
    uint32_t v_IntermediateIdx_u32 = v_Idx_u32;
    const bool_t c_EndOfBuffer_b = calcNextNonZeroEntry_v(v_Idx_u32, v_IntermediateIdx_u32);
    const T v_VarScaleNew_x = this->varScaleBuffer_px[v_IntermediateIdx_u32];
    const uint32_t v_IdxDiff_u32 = v_IntermediateIdx_u32 - v_Idx_u32;

    if (false == c_EndOfBuffer_b && (v_IdxDiff_u32 > 1) )
    {
      const T c_ScaleDiff_x =    v_VarScaleNew_x - v_VarScale_x;
      const T c_ScaleStep_x =  c_ScaleDiff_x / static_cast<T> (v_IdxDiff_u32);
      for ( ; v_IntermediateIdx_u32 > v_Idx_u32; ++v_Idx_u32 )
      {
        this->varScaleBuffer_px[v_Idx_u32] = v_VarScale_x;
        v_VarScale_x += c_ScaleStep_x;
      }
    }
    else // at the end of buffer (-> last value is zero)
    {
      this->varScaleBuffer_px[v_Idx_u32] = v_VarScale_x;
    }
  }
  return;
}

// --------------------------------------------------------------------------
/// @brief Find next non-zero entry in variable scale buffer
///
/// Find next non-zero entry in variable scale buffer start at some index position
/// \p i_Idx_u32
///
/// @remark Private method is called by calcVarScaleBuffer_v()
///
/// @param[in]  i_Idx_u32 Index from where to start the search
/// @param[out] o_Idx_u32 Index of next non-zero entry, equals zero if none was found
///
/// @return true, if non-zero entry was found, false otherwise
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
bool_t SensorVarScale<T, SType_e, RLen_u32>::calcNextNonZeroEntry_v(const uint32_t i_Idx_u32, uint32_t& o_Idx_u32) const
{
  bool_t v_EndOfBuffer_b = true;
  for (o_Idx_u32 = i_Idx_u32; o_Idx_u32 < this->varScaleBufferSize_u32; ++o_Idx_u32)
  {
    if ( this->varScaleBuffer_px[o_Idx_u32] != static_cast<T>(0.0f) )
    {
      v_EndOfBuffer_b = false;
      break;
    }
  }
  return v_EndOfBuffer_b;
}

// --------------------------------------------------------------------------
/// @brief Calculate reverse scaling buffer (called by init_v()
///
/// Calculate reverse scaling buffer on behalf of the already calculated
/// variable scaling buffer.
///
/// @remark Private method is called by init_v()

/// @return void
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::calcRevScaleBuffer_v(void)

{
  // initialize variable scaling for metric coordinates
  const uint32_t c_CoordIdx_u32 = SType_e == e_VarScaleY ? 1U : 0U;
  this->revMetricStartPos_x = this->varScaleBuffer_px[0] *
                              this->sensorUniform_x.getPsz_rx()(c_CoordIdx_u32)
                              *  ( - this->sensorUniform_x.getPpp_rx()(c_CoordIdx_u32) );

  const T c_MetricMaxPos_x =   this->varScaleBuffer_px[this->varScaleBufferSize_u32-1U] *
                               this->sensorUniform_x.getPsz_rx()(1)
                             * ( this->varScaleBufferSize_u32 - 1 - this->sensorUniform_x.getPpp_rx()(c_CoordIdx_u32));

  if (SType_e == e_VarScaleY)
  {
  this->minPos_x.setPosX(this->sensorUniform_x.getMinPos_rx().getPosX());
  this->minPos_x.setPosY(this->revMetricStartPos_x);
  this->maxPos_x.setPosX(this->sensorUniform_x.getMaxPos_rx().getPosX());
  this->maxPos_x.setPosY(c_MetricMaxPos_x);
  }
  else if (SType_e == e_VarScaleX)
  {
    this->minPos_x.setPosX(this->revMetricStartPos_x);
    this->minPos_x.setPosY(this->sensorUniform_x.getMinPos_rx().getPosY());
    this->maxPos_x.setPosX(c_MetricMaxPos_x);
    this->maxPos_x.setPosY(this->sensorUniform_x.getMaxPos_rx().getPosY());
  }
  else
  {
    AssertFunction(false, "SensorVarScale for 2 dimensions not implemented.");
  }

  const T c_MinMaxInterval_x = c_MetricMaxPos_x - this->revMetricStartPos_x;
  this->stepSizeMetric_x = c_MinMaxInterval_x / static_cast<T>(RLen_u32);
  const T c_BufferSizeRatio_x = static_cast<T> (this->varScaleBufferSize_u32) / RLen_u32;

  uint32_t v_IdxPrev_u32 = 0;
  T v_ScalePrev_x = this->varScaleBuffer_px[0]; //  math::constants<T>::one_x();

  for (uint32_t v_RevBufferIdx_u32 = 0; v_RevBufferIdx_u32 < RLen_u32; ++v_RevBufferIdx_u32)
  {
    const T c_Pixel_x = c_BufferSizeRatio_x * static_cast<T>(v_RevBufferIdx_u32);
    mecl::core::Point2D<T>  v_MetricPos_x;
    mecl::core::Point2D<T> v_PixelPos_x(math::constants<T>::zero_x(), c_Pixel_x);
    this->sensorUniform_x.pixelToMetric_v(v_PixelPos_x, v_MetricPos_x);
    const T c_MetricUniform_x = v_MetricPos_x(c_CoordIdx_u32);
    this->pixelToMetric_v(v_PixelPos_x, v_MetricPos_x);
    const T c_Metric_x = v_MetricPos_x(c_CoordIdx_u32);
    const T c_Scale_x = c_Metric_x/ c_MetricUniform_x;
    const uint32_t c_Idx_u32 = static_cast<uint32_t>( (c_Metric_x -this->revMetricStartPos_x) / this->stepSizeMetric_x);

    if (c_Idx_u32 < RLen_u32)
    {
        const uint32_t  c_DiffIdx_u32 = c_Idx_u32 - v_IdxPrev_u32;
        const T c_StepSize_x = (c_Scale_x - v_ScalePrev_x) / static_cast<T>(c_DiffIdx_u32);
        for (uint32_t v_StepIdx_u32 = 1U; v_StepIdx_u32 <= c_DiffIdx_u32; ++v_StepIdx_u32)
        {
          const uint32_t c_BufIdx_u32 = v_IdxPrev_u32 + v_StepIdx_u32;
          const T c_StepScale_x = v_ScalePrev_x + static_cast<T>(v_StepIdx_u32) * c_StepSize_x;
          this->revScaleBuffer_px[c_BufIdx_u32] = c_StepScale_x;
        }

        if (c_DiffIdx_u32 == 0U) // ->for loop not iterated
        {
          this->revScaleBuffer_px[c_Idx_u32] = c_Scale_x;
        }

        v_IdxPrev_u32 = c_Idx_u32;
        v_ScalePrev_x = c_Scale_x;
    }
    else if (v_IdxPrev_u32 < RLen_u32)
    {
      const uint32_t  c_DiffRLen_u32 = RLen_u32 - v_IdxPrev_u32 - 1;
      const T c_LastScale_x = this->varScaleBuffer_px[this->varScaleBufferSize_u32-1];
      const T c_StepSize_x = (c_LastScale_x - v_ScalePrev_x) / static_cast<T>(c_DiffRLen_u32);
      for (uint32_t v_StepIdx_u32 = 1U; v_StepIdx_u32 <= c_DiffRLen_u32; ++v_StepIdx_u32)
      {
        const uint32_t c_BufIdx_u32 = v_IdxPrev_u32 + v_StepIdx_u32;
        const T c_StepScale_x = v_ScalePrev_x + static_cast<T>(v_StepIdx_u32) * c_StepSize_x;
        this->revScaleBuffer_px[c_BufIdx_u32] = c_StepScale_x;
      }
      v_IdxPrev_u32 = c_Idx_u32;
      v_ScalePrev_x = c_Scale_x;
    }
    else
    {
      break;
    }

  } // for
  return;
}

// --------------------------------------------------------------------------
/// @brief Copy sensor configuration to POD object
///
/// The function copies all elements from configuration object of sensor to struct \p o_Config_rs.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_ConfigConstructor
/// @snippet ModelTester.cpp Sensor_Constructor2
/// @snippet ModelTester.cpp Sensor_copyConfig_v
///
/// @param[in] o_Config_rs Configuration to be overwritten with contents from sensor configuration
/// @return void
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::copyConfig_v(Config_s& o_Config_rs) const
{
  this->sensorUniform_x.copyConfig_v(o_Config_rs.sensorUniform_s);
  o_Config_rs.varScaleBuffer_px = this->varScaleBuffer_px;
  o_Config_rs.varScaleBufferSize_u32 = this->varScaleBufferSize_u32;
  return;
}


// --------------------------------------------------------------------------
/// @brief Copy sensor configuration to POD object of (different) base type T1
///
/// The function copies all elements from configuration object of Sensor to
/// struct \p o_Config_rs. of (different) base type T1
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_ConfigConstructor
/// @snippet ModelTester.cpp Sensor_Constructor2
/// @snippet ModelTester.cpp Sensor_copyConfig_otherBaseType
///
/// @param[in] o_Config_rs Configuration to be overwritten with contents from sensor configuration
/// @return void
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
template<typename T1>
void SensorVarScale<T, SType_e, RLen_u32>::copyConfig_v(typename Sensor<T1>::Config_s& o_Config_rs) const
{
  this->convert_x<T1>().copyConfig_v(o_Config_rs);
  return;
}

// --------------------------------------------------------------------------
/// @brief Copy sensor configuration to volatilePOD object
///
/// The function copies all elements from configuration object of sensor to volatilestruct \p o_Config_rs.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_ConfigConstructor
/// @snippet ModelTester.cpp Sensor_Constructor2
/// @snippet ModelTester.cpp Sensor_copyConfigVolatile_v
///
/// @param[in] o_Config_rs Configuration in volatile memory to be overwritten
/// with contents from sensor configuration
/// @return void
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::copyConfigVolatile_v(volatile Config_s& o_Config_rs) const
{
  this->sensorUniform_x.copyConfigVolatile_v(o_Config_rs.sensorUniform_s);
  o_Config_rs.varScaleBuffer_px = this->varScaleBuffer_px;
  o_Config_rs.varScaleBufferSize_u32 = this->varScaleBufferSize_u32;
  return;
}

// -------------------------------------------------------------------------
/// @brief Update configuration
///
/// The function sets sensor model parameters from
/// \p i_Config_rs and marks the sensor matrix object as not configured.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_Constructor1
/// @snippet ModelTester.cpp Sensor_ConfigConstructor
/// @snippet ModelTester.cpp Sensor_updateConfig_v
///
/// @param[in] i_Config_rs Configuration to be applied to this Projection
/// @return void
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::updateConfig_v(const Config_s& i_Config_rs)
{
  this->sensorUniform_x.updateConfig_v(i_Config_rs.sensorUniform_s);
  this->varScaleBuffer_px = i_Config_rs.varScaleBuffer_px;
  this->varScaleBufferSize_u32 = i_Config_rs.varScaleBufferSize_u32;
  this->initialized_b = false;
  return;
}

// -------------------------------------------------------------------------
/// @brief Update configuration by POD struct of (different) base type T1
///
/// The function sets sensor model parameters from  \p i_Config_rs
/// of (different) base type T1 and marks the sensor matrix object as not configured.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_Constructor1
/// @snippet ModelTester.cpp Sensor_ConfigConstructor
/// @snippet ModelTester.cpp Sensor_updateConfig_otherBaseType
///
/// @param[in] i_Config_rs Configuration to be applied to this Projection
/// @return void
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
template<typename T1>
void SensorVarScale<T, SType_e, RLen_u32>::updateConfig_v(const typename Sensor<T1>::Config_s& i_Config_rs)
{
  *this = SensorVarScale<T1, SType_e>(i_Config_rs).convert_x<T>();
  this->initialized_b = false;
}

// -------------------------------------------------------------------------
/// @brief Get configuration status
///
/// Return true if sensor object was properly configured, otherwise false
///
/// @return Boolean, True = configured, false = not configured
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
bool_t SensorVarScale<T, SType_e, RLen_u32>::isConfigured_b(void) const
{
  return this->configured_b && this->sensorUniform_x.isConfigured_b();
}

// -------------------------------------------------------------------------
/// @brief Get reference to principal point of uniform sensor
///
/// Returns principal point position in pixels
///
/// @remark: Wrapper function (calls Sensor<T>::getPpp_rx()
///
/// @return Reference to principal point position
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
const core::Point2D<T>& SensorVarScale<T, SType_e, RLen_u32>::getPpp_rx() const
{
  return this->sensorUniform_x.getPpp_rx();
}

// -------------------------------------------------------------------------
/// @brief Get reference to image resolution of uniform sensor
///
/// Returns image resolution for x and y in mm/pixels for uniform sensor
///
/// @remark: Wrapper function (calls Sensor<T>::getPsz_rx()
///
/// @return Reference to image resolution
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
const core::Point2D<T>& SensorVarScale<T, SType_e, RLen_u32>::getPsz_rx() const
{
  return this->sensorUniform_x.getPsz_rx();
}

// -------------------------------------------------------------------------
/// @brief Get position of metric image origin
///
/// Returns position of image origin with respect to sensor frame
///
/// @remark: Wrapper function (calls Sensor<T>::getImageOriginPosition_e()
///
/// @return Image origin as ImageOriginPosition_e, see ModelTypes.h.
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
ImageOriginPosition_e SensorVarScale<T, SType_e, RLen_u32>::getImageOriginPosition_e() const
{
  return this->sensorUniform_x.imageOriginPosition_e;
}

// -------------------------------------------------------------------------
/// @brief Get upper left corner of sensor area in metric coordinates
///
///
/// @return Position of upper left corner in metric coordinates
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
core::Point2D<T>& SensorVarScale<T, SType_e, RLen_u32>::getMinPos_rx(void) const
{
  return this->minPos_x;
}

// -------------------------------------------------------------------------
/// @brief Get lower right corner of sensor area in metric coordinates
///
/// @return Position of lower right corner in metric coordinates
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
core::Point2D<T>& SensorVarScale<T, SType_e, RLen_u32>::getMaxPos_rx(void) const
{
  return this->maxPos_x;
}

// -------------------------------------------------------------------------
/// @brief Get upper left corner of sensor area in pixel coordinates
///
/// @return Position of upper left corner in pixel coordinates
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
core::Point2D<T> SensorVarScale<T, SType_e, RLen_u32>::getUpperLeft_x(void) const
{
  core::Point2D<T> v_Pos_x;
  this->metricToPixel_v(this->minPos_x, v_Pos_x);
  return v_Pos_x;
}

// -------------------------------------------------------------------------
/// @brief Get lower right corner of sensor area in pixel coordinates
///
/// @return Position of lower right corner in pixel coordinates
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
core::Point2D<T> SensorVarScale<T, SType_e, RLen_u32>::getLowerRight_x(void) const
{
  core::Point2D<T> v_Pos_x;
  this->metricToPixel_v(this->maxPos_x, v_Pos_x);
  return v_Pos_x;
}

// -------------------------------------------------------------------------
/// @brief Copy variable scale buffer
///
/// Get a full or partial copy of the variable scale buffer to array of floats defined by a pointer
/// starting index and the size of the array.
///
/// @oaram[out] i_RevScaleBuffer_px Pointer to the first element of the array where to copy to
/// @param[in] i_StartPos_u32      Starting index of the first entry in the variable scale buffer (0 by default)
/// @param[in] i_NumOfEntries_u32  Number of entries to copy (length of the variable scale buffer by default)
///
/// @return void
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
uint32_t SensorVarScale<T, SType_e, RLen_u32>::copyVarScaleBuffer_u32( T* o_VarScaleBuffer_px,
                                                                       const uint32_t i_StartPos_u32,
                                                                       const uint32_t i_NumOfEntries_u32 ) const
{
  AssertFunction(  (i_StartPos_u32 + i_NumOfEntries_u32) < this->varScaleBufferSize_u32 ,
                   "Starting position and number of entries exceed VarScaleBuffer size");

  const T*       c_StartAddr_px = &this->varScaleBuffer_px[i_StartPos_u32];
  const uint32_t c_NumOfEntries_u32 = i_NumOfEntries_u32 != 0 ? i_NumOfEntries_u32 : this->varScaleBufferSize_u32 - i_StartPos_u32;
  memcpy(o_VarScaleBuffer_px, c_StartAddr_px, c_NumOfEntries_u32);
  return c_NumOfEntries_u32;
}

// -------------------------------------------------------------------------
/// @brief Copy reverse scale buffer
///
/// Get a full or partial copy of the reverse scaling buffer to array of floats defined by a pointer
/// starting index and the size of the array.
///
/// @oaram[out] i_RevScaleBuffer_px Pointer to the first element of the array where to copy to
/// @param[in] i_StartPos_u32      Starting index of the first entry in the reverse scalebuffer (0 by default)
/// @param[in] i_NumOfEntries_u32  Number of entries to copy (length of the reverse scale buffer by default)
///
/// @return void
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
uint32_t SensorVarScale<T, SType_e, RLen_u32>::copyRevScaleBuffer_u32( T* o_RevScaleBuffer_px,
                                                                       const uint32_t i_StartPos_u32,
                                                                       const uint32_t i_NumOfEntries_u32 ) const
 {
   AssertFunction( (i_StartPos_u32 + i_NumOfEntries_u32) < RLen_u32,
                   "Starting position and number of entries exceed VarScaleBuffer size");

   AssertFunction( this->initialized_b == true, "SensorVarScale instance not initialized.");

   const T*       c_StartAddr_px = &this->revScaleBuffer_px[i_StartPos_u32];
   const uint32_t c_NumOfEntries_u32 = i_NumOfEntries_u32 != 0 ? i_NumOfEntries_u32 : RLen_u32 - i_StartPos_u32;
   memcpy(o_RevScaleBuffer_px, c_StartAddr_px, c_NumOfEntries_u32);
   return c_NumOfEntries_u32;
 }

// -------------------------------------------------------------------------
/// @brief Copy reverse buffer info values
///
/// Get a copy of a the reverse scaling buffer info, containing its defining sizes:
/// metric start position and metric step size of index difference one
///
/// @param[out] o_Info_rs Reference to struct where to copy to
///
/// @return void
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::copyRevBufferInfo_v(RevBufferInfo_s& o_Info_rs) const
{
  AssertFunction( this->initialized_b == true, "SensorVarScale instance not initialized.");

  o_Info_rs.startPos_x = this->revMetricStartPos_x;
  o_Info_rs.stepSizeMetric_x = this->stepSizeMetric_x;

  return;
}

// -------------------------------------------------------------------------
/// @brief Get size of reverse scale buffer
///
/// Get number of entries in reverse scale buffer
///
/// @return Size of reverse scale buffer
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
uint32_t SensorVarScale<T, SType_e, RLen_u32>::getRevScaleBufferSize_u32(void) const
{
  return RLen_u32;
}

  // -------------------------------------------------------------------------
  /// @brief Set principal point
  ///
  /// Function sets principal point location
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Sensor_Constructor1
  /// @snippet ModelTester.cpp Sensor_setPpp_v
  ///
  /// @return void
  // -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::setPpp_v(const core::Point2D<T>& i_Ppp_x)
{
  this->sensorUniform_x.setPpp_v(i_Ppp_x);
  return;
}

// -------------------------------------------------------------------------
/// @brief Set image resolution
///
/// Function sets image resolution for x and y in mm/pixels
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_Constructor1
/// @snippet ModelTester.cpp Sensor_setPsz_v
///
/// @return void
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::setPsz_v(const core::Point2D<T>& i_Psz_x)
{
  this->sensorUniform_x.setPsz_v(i_Psz_x);
  return;
}

// -------------------------------------------------------------------------
/// @brief set position of metric image origin
///
/// Function sets position of image origin with respect to sensor frame
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_Constructor1
/// @snippet ModelTester.cpp Sensor_setImageOriginPosition_v
///
/// @return void
// -------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T , SType_e, RLen_u32>::setImageOriginPosition_v(const ImageOriginPosition_e i_ImageOriginPosition_e)
{
  this->sensorUniform_x.imageOriginPosition_e = i_ImageOriginPosition_e;
  this->sensorUniform_x.initMinMax_v();
  return;
}

// -------------------------------------------------------------------------
/// @brief Set upper left corner of valid sensor area in pixel coordinates
///
/// Function sets position of upper left corner to pixel postion \p i_Pos_rx.
/// Additionally this point is checked, if it is neither left of nor above the
/// already defined lower right corner, by an assertion.
///
/// @param[in] i_Pos_rx New position of upper left corner in pixel
/// @return void
// -------------------------------------------------------------------------

template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T , SType_e, RLen_u32>::setUpperLeft_v(const core::Point2D<T>& i_Pos_rx)
{
  this->sensorUniform_x.setUpperLeft_v(i_Pos_rx);
  return;
}

// -------------------------------------------------------------------------
/// @brief Set lower right corner of valid sensor area in pixel coordinates
///
/// Function sets position of lower right  corner to pixel postion \p i_Pos_rx.
/// Additionally this point is checked, if it is neither left of nor above the
/// already defined upper left corner, by an assertion.
///
/// @param[in] i_Pos_rx New position of upper left corner in pixel
/// @return void
// -------------------------------------------------------------------------

template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T , SType_e, RLen_u32>::setLowerRight_v(const core::Point2D<T>& i_Pos_rx)
{
  this->sensorUniform_x.setLowerRight_v(i_Pos_rx);
  return;
}

//--------------------------------------------------------------------------
/// @brief Create a copy of this instance of different base type T1
///
/// The function returns a copy of this instance having converted all its \n
/// floating point values of base type T to new base type T1
///
/// @par Example usage:
/// @snippet Modeltester.cpp Sensor_Constructor1
/// @snippet Modeltester.cpp Sensor_convert_x
///
/// @return Copy of this instance of base type T1
//--------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
template<typename T1>
SensorVarScale<T1, SType_e, RLen_u32> SensorVarScale<T , SType_e, RLen_u32>::convert_x(void) const
{
  typename SensorVarScale<T1, SType_e, RLen_u32>::Config_s v_SensorCfg_x = {};
  this->sensorUniform_x.copyConfig_v<T1>(v_SensorCfg_x.sensorUniform_s);
  v_SensorCfg_x.varScaleBuffer_px = static_cast<T1*> (this->varScaleBuffer_px);
  v_SensorCfg_x.varScaleBufferSize_u32 = this->varScaleBufferSize_u32;

  return  SensorVarScale<T1, SType_e, RLen_u32>(v_SensorCfg_x);
}

//--------------------------------------------------------------------------
/// @brief Get a Sensor reference by down-casting a reference to ISensor
///
/// The functions tries to cast the ISensor reference to a reference of a Sensor
/// instance. On failure this causes an assertion, otherwise the function returns
/// a valid reference to a Sensor instance
///
/// @par Example usage:
/// @snippet Modeltester.cpp Camera_SensorConstructor
/// @snippet Modeltester.cpp Camera_Constructor9
/// @snippet Modeltester.cpp Camera_downcastSensor
///
/// @return Valid reference to an instance of Sensor
//--------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
SensorVarScale<T, SType_e, RLen_u32>& SensorVarScale<T, SType_e, RLen_u32>::get_rx(ISensor<T>& i_ISensor_rx)
{
  SensorVarScale<T, SType_e, RLen_u32 >* v_Sensor_px = dynamic_cast<SensorVarScale<T, SType_e, RLen_u32>*>(&i_ISensor_rx);
  AssertFunction(core::isNotNull_b(v_Sensor_px),"Reference to ISensor is not a SensorVarScale object");
  return *v_Sensor_px;
}

//--------------------------------------------------------------------------
/// @brief Get an instance of Sensor<T> referenced by Sensor<T1>&
///
/// The functions tries to cast the Sensor of base type T1 reference to a Sensor
/// instance of base type T1. On failure this causes an assertion,
/// otherwise the function returns a copy of that instance converted to base type T.
///
/// @par Example usage:
/// @snippet Modeltester.cpp Camera_CylinderLensConstructor
/// @snippet Modeltester.cpp Camera_Constructor9
/// @snippet Modeltester.cpp Camera_convertLensCylinder
///
/// @return Instance of Sensor of base type T
//--------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
template<typename T1>
SensorVarScale<T, SType_e, RLen_u32> SensorVarScale<T, SType_e, RLen_u32>::convert_x(ISensor<T1>& i_ISensor_rx)
{
  SensorVarScale<T1, SType_e, RLen_u32>* v_Sensor_px = dynamic_cast<SensorVarScale<T1, SType_e>*>(&i_ISensor_rx);
  AssertFunction(core::isNotNull_b(v_Sensor_px),"Reference to ISensor of different base type is not a SensorVarScale object");
  return v_Sensor_px->convert_x<T>();
}

//--------------------------------------------------------------------------
/// @brief Cast operator for conversion to different base type T1
///
/// Operator casts a Sensor<T> object to different base type T1.
///
/// @par Example usage:
/// @snippet Modeltester.cpp LensCylinder_Constructor1
/// @snippet Modeltester.cpp LensCylinder_convertByCast
/// @snippet Modeltester.cpp LensCylinder_convertByCtor
/// @snippet Modeltester.cpp LensCylinder_convertByAssignment
///
/// @return Instance of LensCylinder of base type T1
//--------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
template<typename T1>
SensorVarScale<T, SType_e, RLen_u32>::operator SensorVarScale<T1, SType_e, RLen_u32>() const
{
  return this->convert_x<T1>();
}

// --------------------------------------------------------------------------
/// @brief Convert point location from pixel to metric
///
/// The function converts 2D point location on image plane from pixel
/// coordinates into metric dimension
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_ConfigConstructor
/// @snippet ModelTester.cpp Sensor_Constructor2
/// @snippet ModelTester.cpp Sensor_pixelToMetric_v
///
/// @param[in] i_Pos_rx    Image points location in pixels
/// @param[out] o_Pos_rx   Image point in metric coordinates
/// @return void
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::pixelToMetric_v(const core::Point2D<T>& i_Pos_rx,
                                                 core::Point2D<T>& o_Pos_rx) const
{
  // const T v_ScaleBuffer_ax[] = this->varScaleBuffer_px;
  T v_VarScale_x = static_cast<T>(1.0f);
  this->sensorUniform_x.pixelToMetric_v(i_Pos_rx, o_Pos_rx);
  switch(SType_e)
  {
    case e_VarScaleX:
      break;

    case e_VarScaleY:
    {
      this->calcScale_v(i_Pos_rx, v_VarScale_x);
      o_Pos_rx(1) *= v_VarScale_x;
      break;
    }


    case e_NonUniform:
    {
      break;
    }

    default: // uniform sensor
    {
      break;
    }
  }

  return;
}

// --------------------------------------------------------------------------
/// @brief Convert point location from metric to pixel
///
/// The function converts 2D point location on image plane from metric
/// dimension into pixel coordinates.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_ConfigConstructor
/// @snippet ModelTester.cpp Sensor_Constructor2
/// @snippet ModelTester.cpp Sensor_metricToPixel_v
///
/// @param[in] i_Pos_rx     Image points in metric coordinates
/// @param[out] o_Pos_rx    Image points locations in pixel
/// @return void
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::metricToPixel_v(const core::Point2D<T>& i_Pos_rx,
                                      core::Point2D<T>& o_Pos_rx) const
{

  AssertFunction(true == this->initialized_b, "SensorVarScale instance not inizitalized.");

  switch(SType_e)
  {
    case e_VarScaleX:
      break;
    case e_VarScaleY:
    {
      T v_RevScale_x = 1.0f;
      this->calcRevScale_v(i_Pos_rx, v_RevScale_x);
      o_Pos_rx(0) = i_Pos_rx(0);
      o_Pos_rx(1) = i_Pos_rx(1) / v_RevScale_x;

    }
    break;
    case e_NonUniform:
      break;
    default: // non-uniform
      break;
  }
  this->sensorUniform_x.metricToPixel_v(o_Pos_rx, o_Pos_rx);
  return;
}

// --------------------------------------------------------------------------
/// @brief Check if metric point location is visible
///
/// The methods checks if metric point location is visible within sensor area
///
/// @return true, if visible, otherwise false
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
bool_t SensorVarScale<T, SType_e, RLen_u32>::isVisible_b(const core::Point2D<T>& i_Pos_rx) const
{
  AssertFunction(true == this->initialized_b, "SensorVarScale instance not initialized.");

  return    ( i_Pos_rx(0) >= this->minPos_x(0) )
         && ( i_Pos_rx(0) <= this->maxPos_x(0) )
         && ( i_Pos_rx(1) >= this->minPos_x(1) )
         && ( i_Pos_rx(1) <= this->maxPos_x(1) );

  // return this->sensorUniform_x.isVisible_b(i_Pos_rx);
}

//  --------------------------------------------------------------------------
/// @brief Get variable scaling value for pixel coordinates
///
/// Get variable scaling value for pixel coordinates
///
/// @param[in] i_Pos_rx Position in pixel coordinates
///
/// @return scaling value
//  --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
T SensorVarScale<T, SType_e, RLen_u32>::getVarScaleForPixel_x(const core::Point2D<T>& i_Pos_rx) const
{
  T v_VarScale_x = static_cast<T>(1.0f);
  this->calcScale_v(i_Pos_rx, v_VarScale_x);
  return v_VarScale_x;
}
//  --------------------------------------------------------------------------
/// @brief Get variable scaling value for metric coordinates
///
/// Get variable scaling value for metric coordinates
///
/// @param[in] i_Pos_rx Position in metric coordinates
///
/// @return scaling value
//  --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
T SensorVarScale<T, SType_e, RLen_u32>::getVarScaleForMetric_x(const core::Point2D<T>& i_Pos_rx) const
{
  T v_VarScale_x = static_cast<T>(1.0f);

  this->calcRevScale_v(i_Pos_rx, v_VarScale_x);
  return v_VarScale_x;
}

//  --------------------------------------------------------------------------
/// @brief Calculate variable scaling value for pixel coordinates
///
/// Calculate variable scaling value for pixel coordinates and copy to variable
///
/// @param[in]  i_Pos_rx Position in pixel coordinates
/// @param[out] o_VarScale_x Output variable containing scaling
///
/// @return void
//  --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::calcScale_v(const mecl::core::Point2D<T>& i_Pos_rx, T& o_VarScale_rx) const
{
    switch(SType_e)
    {
      case e_VarScaleX: // fall through
      case e_VarScaleY:
      {
        const uint32_t c_CoordIdx_u32 = SType_e == e_VarScaleX ? 0U: 1U;
        const T c_BufferIdxMax_x = ceil(i_Pos_rx(c_CoordIdx_u32));
        if ( c_BufferIdxMax_x >=  this->varScaleBufferSize_u32 ) // take last varScale value if index is out of range
        {
          o_VarScale_rx = this->varScaleBuffer_px[this->varScaleBufferSize_u32-1];
        }
        else if (c_BufferIdxMax_x < math::constants<T>::zero_x() ) // take first varScale value if index is negative
        {
          o_VarScale_rx = this->varScaleBuffer_px[0U];
        }
        else // index is in range of varScaleBuffer
        {
          const T c_BufferIdxMin_x = floor(i_Pos_rx(c_CoordIdx_u32));
          if (c_BufferIdxMin_x != c_BufferIdxMax_x) // interpolate if index is not an integer
          {
            const T c_WeightMin_x = i_Pos_rx(1) - c_BufferIdxMin_x;
            const T c_WeightMax_x = c_BufferIdxMax_x - i_Pos_rx(1);
            o_VarScale_rx =    c_WeightMax_x * this->varScaleBuffer_px[static_cast<uint32_t>(c_BufferIdxMin_x)]
                             + c_WeightMin_x * this->varScaleBuffer_px[static_cast<uint32_t>(c_BufferIdxMax_x)];
          }
          else
          {
            o_VarScale_rx = this->varScaleBuffer_px[static_cast<uint32_t>(c_BufferIdxMin_x)];
          }
        }
        break;
      }
      case e_NonUniform:
      {
        break;
      }
      default: // uniform sensor
      {
        break;
      }
    }
    return;
}

//  --------------------------------------------------------------------------
/// @brief Calculate reverse scaling value for metric coordinates
///
/// Calculate reverse scaling value for metric coordinates and copy to variable
///
/// @param[in]  i_Pos_rx Position in metric coordinates
/// @param[out] o_VarScale_x Output variable containing scaling
///
/// @return void
//  --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e, uint32_t RLen_u32>
void SensorVarScale<T, SType_e, RLen_u32>::calcRevScale_v(const mecl::core::Point2D<T>& i_Pos_rx, T& o_VarScale_rx) const
{
  switch(SType_e)
  {

    case e_VarScaleX:
    case e_VarScaleY:
    {
      const uint32_t c_CoordIdx_u32 = SType_e == e_VarScaleX ? 0U: 1U;
      const T c_RevScale_f32 = (i_Pos_rx(c_CoordIdx_u32) - this->revMetricStartPos_x) / this->stepSizeMetric_x;
      const T c_RefScaleMin_f32 = floor(c_RevScale_f32);

      if (c_RefScaleMin_f32 >= math::constants<T>::zero_x() ) // metric value is greater than start position of revScaleBuffer
      {
        const T c_WeightMax_f32 = c_RevScale_f32 - c_RefScaleMin_f32;
        const T c_WeightMin_f32 = math::constants<T>::one_x() - c_WeightMax_f32;

        const uint32_t c_RevScaleIdxMin_u32 = static_cast<uint32_t>(c_RefScaleMin_f32);
        const uint32_t c_RevScaleIdxMax_u32 = c_RevScaleIdxMin_u32 + 1;

        if (c_RevScaleIdxMax_u32 < RLen_u32) // out of range: maximum value
        {
          o_VarScale_rx =    this->revScaleBuffer_px[c_RevScaleIdxMin_u32] * c_WeightMin_f32
              +  this->revScaleBuffer_px[c_RevScaleIdxMax_u32] * c_WeightMax_f32;

        }
        else if (c_RevScaleIdxMin_u32 < RLen_u32) // out of range: minimum value
        {
          o_VarScale_rx =    this->revScaleBuffer_px[c_RevScaleIdxMin_u32] * c_WeightMin_f32
              +  this->revScaleBuffer_px[RLen_u32-1U]          * c_WeightMax_f32;
        }
        else
        {
          o_VarScale_rx =   this->revScaleBuffer_px[RLen_u32-1];
        }
      }
      else // metric value is smaller than start position of revScaleBuffer
      {
        o_VarScale_rx = this->revScaleBuffer_px[0U];
      }
    }
    break;
    case e_NonUniform:
      break;
    default: // non-uniform
    break;
  }
  return;
}


} /* namespace model */
} /* namespace mecl */

#endif /* SRC_MODEL_SENSOR_HPP_ */

/// @}
/// @}




