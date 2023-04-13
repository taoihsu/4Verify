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

#ifndef SRC_MODEL_SENSOR_HPP_
#define SRC_MODEL_SENSOR_HPP_

#include "Sensor.h"

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
template<typename T>
Sensor<T>::Sensor(void)
: ppp_x(math::constants<T>::zero_x())
, psz_x(math::constants<T>::one_x())
, imageOriginPosition_e(e_UpperLeft)
, minPos_x(math::numeric_limits<T>::lowest_x())
, maxPos_x(math::numeric_limits<T>::max_x())
, configured_b(false)
{}

// --------------------------------------------------------------------------
/// @brief Constructor copying configuration from existing configuration data set
///
/// @par Example use:
/// @snippet ModelTester.cpp Sensor_ConfigConstructor
/// @snippet ModelTester.cpp Sensor_Constructor2
///
/// @param[in] i_Config_rs Sensor model configuration
// --------------------------------------------------------------------------
template<typename T>
Sensor<T>::Sensor(const Config_s& i_Config_rs)
: ppp_x(i_Config_rs.pppCfg_x)
, psz_x(i_Config_rs.pszCfg_x)
, imageOriginPosition_e(i_Config_rs.imageOriginPositionCfg_e)
, minPos_x(i_Config_rs.pos1Cfg_x)
, maxPos_x(i_Config_rs.pos2Cfg_x)
, configured_b(false)
{
  initMinMax_v();
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
template<typename T>
void Sensor<T>::copyConfig_v(Config_s& o_Config_rs) const
{
  this->ppp_x.copyConfig_v(o_Config_rs.pppCfg_x);
  this->psz_x.copyConfig_v(o_Config_rs.pszCfg_x);
  o_Config_rs.imageOriginPositionCfg_e = this->imageOriginPosition_e;

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
template<typename T>
template<typename T1>
void Sensor<T>::copyConfig_v(typename Sensor<T1>::Config_s& o_Config_rs) const
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
template<typename T>
void Sensor<T>::copyConfigVolatile_v(volatile Config_s& o_Config_rs) const
{
  this->ppp_x.copyConfigVolatile_v(o_Config_rs.pppCfg_x);
  this->psz_x.copyConfigVolatile_v(o_Config_rs.pszCfg_x);
  o_Config_rs.imageOriginPositionCfg_e = this->imageOriginPosition_e;
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
template<typename T>
void Sensor<T>::updateConfig_v(const Config_s& i_Config_rs)
{
  this->ppp_x.updateConfig_v(i_Config_rs.pppCfg_x);
  this->psz_x.updateConfig_v(i_Config_rs.pszCfg_x);
  this->minPos_x.updateConfig_v(i_Config_rs.pos1Cfg_x);
  this->maxPos_x.updateConfig_v(i_Config_rs.pos2Cfg_x);
  this->imageOriginPosition_e = i_Config_rs.imageOriginPositionCfg_e;
  this->initMinMax_v();
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
template<typename T>
template<typename T1>
void Sensor<T>::updateConfig_v(const typename Sensor<T1>::Config_s& i_Config_rs)
{
  *this = Sensor<T1>(i_Config_rs).convert_x<T>();
}

// -------------------------------------------------------------------------
/// @brief Get configuration status
///
/// Return true if sensor object was properly configured, otherwise false
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_Constructor1
/// @snippet ModelTester.cpp Sensor_isConfigured_b
///
/// @return Boolean, True = configured, false = not configured
// -------------------------------------------------------------------------
template<typename T>
bool_t Sensor<T>::isConfigured_b(void) const
{
  return this->configured_b;
}

// -------------------------------------------------------------------------
/// @brief Get reference to principal point
///
/// Returns principal point position in pixels
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_ConfigConstructor
/// @snippet ModelTester.cpp Sensor_Constructor2
/// @snippet ModelTester.cpp Sensor_getPpp_rx
///
/// @return Reference to principal point position
// -------------------------------------------------------------------------
template<typename T>
const core::Point2D<T>& Sensor<T>::getPpp_rx() const
{
  return this->ppp_x;
}

// -------------------------------------------------------------------------
/// @brief Get reference to image resolution
///
/// Returns image resolution for x and y in mm/pixels
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_ConfigConstructor
/// @snippet ModelTester.cpp Sensor_Constructor2
/// @snippet ModelTester.cpp Sensor_getPsz_rx
///
/// @return Reference to image resolution
// -------------------------------------------------------------------------
template<typename T>
const core::Point2D<T>& Sensor<T>::getPsz_rx() const
{
  return this->psz_x;
}

// -------------------------------------------------------------------------
/// @brief Get position of metric image origin
///
/// Returns position of image origin with respect to sensor frame
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_ConfigConstructor
/// @snippet ModelTester.cpp Sensor_Constructor2
/// @snippet ModelTester.cpp Sensor_getImageOriginPosition_e
///
/// @return Image origin as ImageOriginPosition_e, see ModelTypes.h.
// -------------------------------------------------------------------------
template<typename T>
ImageOriginPosition_e Sensor<T>::getImageOriginPosition_e() const
{
  return this->imageOriginPosition_e;
}

template<typename T>
const core::Point2D<T>& Sensor<T>::getMinPos_rx(void) const
{
  return this->minPos_x;
}

template<typename T>
const core::Point2D<T>& Sensor<T>::getMaxPos_rx(void) const
{
  return this->maxPos_x;
}

template<typename T>
core::Point2D<T> Sensor<T>::getUpperLeft_x(void) const
{
  core::Point2D<T> v_Pos_x;
  this->metricToPixel_v(this->minPos_x, v_Pos_x);
  return v_Pos_x;
}

template<typename T>
core::Point2D<T> Sensor<T>::getLowerRight_x(void) const
{
  core::Point2D<T> v_Pos_x;
  this->metricToPixel_v(this->maxPos_x, v_Pos_x);
  return v_Pos_x;
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
template<typename T>
void Sensor<T>::setPpp_v(const core::Point2D<T>& i_Ppp_x)
{
  this->ppp_x = i_Ppp_x;
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
template<typename T>
void Sensor<T>::setPsz_v(const core::Point2D<T>& i_Psz_x)
{
  this->psz_x = i_Psz_x;
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
template<typename T>
void Sensor<T>::setImageOriginPosition_v(const ImageOriginPosition_e i_ImageOriginPosition_e)
{
  this->imageOriginPosition_e = i_ImageOriginPosition_e;
  this->initMinMax_v();
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

template<typename T>
void Sensor<T>::setUpperLeft_v(const core::Point2D<T>& i_Pos_rx)
{
  core::Point2D<T> c_LowerRightPx_x = this->getLowerRight_x();
  AssertFunction(    ( c_LowerRightPx_x(0) > i_Pos_rx(0) )
                  && ( c_LowerRightPx_x(1) > i_Pos_rx(1) ),
                  "Tried to set upper left corner to invalid pixel position."
                );
  this->pixelToMetric_v(i_Pos_rx, this->minPos_x);
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

template<typename T>
void Sensor<T>::setLowerRight_v(const core::Point2D<T>& i_Pos_rx)
{
  core::Point2D<T> c_UpperLeftPx_x = this->getUpperLeft_x();
  AssertFunction(    ( c_UpperLeftPx_x(0) < i_Pos_rx(0) )
                  && ( c_UpperLeftPx_x(1) < i_Pos_rx(1) ),
                  "Tried to set lower right corner to invalid pixel position."
                );
  this->pixelToMetric_v(i_Pos_rx, this->maxPos_x);
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
template<typename T>
template<typename T1>
Sensor<T1> Sensor<T>::convert_x(void) const
{
  typename Sensor<T1>::Config_s v_SensorCfg_x = {};

  this->ppp_x.template copyConfig_v<T1>(v_SensorCfg_x.pppCfg_x);
  this->psz_x.template copyConfig_v<T1>(v_SensorCfg_x.pszCfg_x);
  v_SensorCfg_x.imageOriginPositionCfg_e = this->imageOriginPosition_e;

  return  Sensor<T1>(v_SensorCfg_x);
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
template<typename T>
Sensor<T>& Sensor<T>::get_rx(ISensor<T>& i_ISensor_rx)
{
  Sensor<T>* v_Sensor_px = dynamic_cast<Sensor<T>*>(&i_ISensor_rx);
  AssertFunction(core::isNotNull_b(v_Sensor_px),"Reference to ISensor is not a Sensor object");
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
template<typename T>
template<typename T1>
Sensor<T> Sensor<T>::convert_x(ISensor<T1>& i_ISensor_rx)
{
  Sensor<T1>* v_Sensor_px = dynamic_cast<Sensor<T1>*>(&i_ISensor_rx);
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
template<typename T>
template<typename T1>
Sensor<T>::operator Sensor<T1>() const
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
template<typename T>
void Sensor<T>::pixelToMetric_v(const core::Point2D<T>& i_Pos_rx,
                                      core::Point2D<T>& o_Pos_rx) const
{
  o_Pos_rx = (i_Pos_rx - this->ppp_x) * this->psz_x;
  this->changeOrientation_v(o_Pos_rx, o_Pos_rx);
  return;
}

// --------------------------------------------------------------------------
/// @brief Convert point location from metric to pixel
///
/// The function converts 2D point location on image plane from metric
/// dimension into pixel coordinates
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
template<typename T>
void Sensor<T>::metricToPixel_v(const core::Point2D<T>& i_Pos_rx,
                                      core::Point2D<T>& o_Pos_rx) const
{
  this->changeOrientation_v(i_Pos_rx,o_Pos_rx);
  o_Pos_rx = o_Pos_rx / this->psz_x + this->ppp_x;
  return;
}

// --------------------------------------------------------------------------
/// @brief Check if metric point location is visible
///
/// The methods checks if metric point location is visible within sensor area
///
/// @return true, if visible, otherwise false
// --------------------------------------------------------------------------
template<typename T>
bool_t Sensor<T>::isVisible_b(const core::Point2D<T>& i_Pos_rx) const
{
  return (   ( i_Pos_rx.getPosX() >= this->minPos_x.getPosX() )
          && ( i_Pos_rx.getPosY() >= this->minPos_x.getPosY() )
          && ( i_Pos_rx.getPosX() <= this->maxPos_x.getPosX() )
          && ( i_Pos_rx.getPosY() <= this->maxPos_x.getPosY() )
         );
}

// -------------------------------------------------------------------------
/// @brief Change image orientation
///
/// Function converts pixel point location based on origin position. Default is
/// upper left corner in which case point remain unchanged.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Sensor_ConfigConstructor
/// @snippet ModelTester.cpp Sensor_Constructor2
/// @snippet ModelTester.cpp Sensor_changeOrientation_v
/// @code{.cpp}
/// // Calculate pixel location based on image origin.
/// v_Sensor2_o.changeOrientation_v(v_InputPos_x, v_OutputPos_x);
/// @endcode
///
/// @return void
// -------------------------------------------------------------------------
template<typename T>
void Sensor<T>::changeOrientation_v(const core::Point2D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const
{
  switch(this->imageOriginPosition_e)
  {
    case e_UpperRight:
      o_Pos_rx(0) = i_Pos_rx(0) * math::constants<T>::minusOne_x();
      o_Pos_rx(1) = i_Pos_rx(1);
      break;
    case e_LowerLeft:
      o_Pos_rx(0) = i_Pos_rx(0);
      o_Pos_rx(1) = i_Pos_rx(1) * math::constants<T>::minusOne_x();
      break;
    case e_LowerRight:
      o_Pos_rx(0) = i_Pos_rx(0) * math::constants<T>::minusOne_x();
      o_Pos_rx(1) = i_Pos_rx(1) * math::constants<T>::minusOne_x();
      break;
    case e_UpperLeft: // fall through
    default:
      o_Pos_rx = i_Pos_rx;
      break;
  }
  return;
}

// -------------------------------------------------------------------------
/// @brief Initialize sensor area (minimal/maximal position)
///
/// @return void
// -------------------------------------------------------------------------
/// @brief Change image orientation
template<typename T>
void Sensor<T>::initMinMax_v(void)
{
  // Find upper left and lower right corner from input coordinate pair
    const T c_Leftmost_x = math::min_x(this->minPos_x.getPosX(), this->maxPos_x.getPosX());
    const T c_Rightmost_x = math::max_x(this->minPos_x.getPosX(), this->maxPos_x.getPosX());
    const T c_Upmost_x = math::min_x(this->minPos_x.getPosY(), this->maxPos_x.getPosY());
    const T c_Lowest_x = math::max_x(this->minPos_x.getPosY(), this->maxPos_x.getPosY());

    this->minPos_x.setPosX(c_Leftmost_x);
    this->minPos_x.setPosY(c_Upmost_x);
    this->maxPos_x.setPosX(c_Rightmost_x);
    this->maxPos_x.setPosY(c_Lowest_x);

    // convert private members for visibility area to metric coordinates
    this->pixelToMetric_v(this->minPos_x, this->minPos_x);
    this->pixelToMetric_v(this->maxPos_x, this->maxPos_x);
    this->configured_b = true;

    return;
}

} /* namespace model */
} /* namespace mecl */

#endif /* SRC_MODEL_SENSOR_HPP_ */

/// @}
/// @}
