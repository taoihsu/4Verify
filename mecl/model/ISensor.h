
//--------------------------------------------------------------------------
/// @file ISensor.h
/// @brief Interface for Sensors and Definition of DefaultSensor class template
/// Defines common interface for sensors and a default sensor class template used
/// if camera model defines no sensor.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
/// @date 04/15/2015
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef SRC_MODEL_ISENSOR_H_
#define SRC_MODEL_ISENSOR_H_

#include "config/IConfig.h"
#include "core/MeclAssert.h"

namespace mecl
{
namespace model
{

//--------------------------------------------------------------------------
/// @class ISensor
/// @brief Interface for sensor class templates
/// Defines interface for all generic classes that define a sensor model.
/// Remark: Input coordinates are strictly metric, output coordinates dims can be either metric or
/// in pixel.
//  --------------------------------------------------------------------------

template<typename T>
class ISensor : public config::IConfig
{
public:

  // getters

  // --------------------------------------------------------------------------
  /// @brief Get principal point pixel coordinates (sensor)
  ///
  /// Return principal point location as pixel coordinates in sensor coordinate
  /// system
  ///
  /// @par Example use:
  /// @snippet ModelTester.cpp Pinhole_getPppPx_px
  ///
  /// @return Principle point location in pixels
  // --------------------------------------------------------------------------
  virtual const core::Point2D<T>& getPpp_rx() const = 0;

  // --------------------------------------------------------------------------
  /// @brief Get pixel size (sensor)
  ///
  /// Return sensor pixel size in millimeters per pixel
  ///
  /// @par Example use:
  /// @snippet ModelTester.cpp Pinhole_getPszPx_px
  ///
  /// @return Pixel size
  // --------------------------------------------------------------------------
  virtual const core::Point2D<T>& getPsz_rx() const = 0;

  // --------------------------------------------------------------------------
  /// @brief Get upper left corner of sensor area in pixel coordinates
  ///
  /// Return logical upper left corner in pixel coordinates
  ///
  /// @return Upper left corner coordinate of sensor oixels
  // --------------------------------------------------------------------------
  virtual core::Point2D<T> getUpperLeft_x(void) const = 0;

  // --------------------------------------------------------------------------
  /// @brief Get lower right corner of sensor area in pixel coordinates
  ///
  /// Return logical lower right corner in pixel coordinates
  ///
  /// @return Lower right corner coordinate of sensor oixels
  // --------------------------------------------------------------------------
  virtual core::Point2D<T> getLowerRight_x(void) const = 0;

  // --------------------------------------------------------------------------
  /// @brief Set pixel size (sensor)
  ///
  /// Set sensor pixel size in millimeters per pixel
  // --------------------------------------------------------------------------
  virtual void setPpp_v(const core::Point2D<T>& i_Ppp_rx) = 0;

  // --------------------------------------------------------------------------
  /// @brief Set pixel size (sensor)
  ///
  /// Set sensor pixel size in millimeters per pixel
  // --------------------------------------------------------------------------
  virtual void setPsz_v(const core::Point2D<T>& i_Psz_rx) = 0;

  // processing functions
  //--------------------------------------------------------------------------
  /// @brief Convert point location from pixel to metric dimension
  ///
  /// The function converts 2D point location on image plane from pixel
  /// coordinates into metric dimension
  ///
  /// @par Example usage:
  /// See mecl::model::Pinhole for further details.
  ///
  /// @param[in] i_Pos_rx    Image points location in pixels
  /// @param[out] o_Pos_rx   Image point in metric coordinates
  /// @return void
  // --------------------------------------------------------------------------
  virtual void pixelToMetric_v(const core::Point2D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const = 0;

  //--------------------------------------------------------------------------
  /// @brief Convert point location from metric to pixel
  ///
  /// The function converts 2D point location on image plane from metric
  /// dimension into pixel coordinates
  ///
  /// @par Example usage:
  /// See mecl::model::Pinhole for further details.
  ///
  /// @param[in] i_Pos_rx     Image points in metric coordinates
  /// @param[out] o_Pos_rx     Image points locations in pixel
  /// @return void
  //--------------------------------------------------------------------------
  virtual void metricToPixel_v(const core::Point2D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const = 0;

  virtual bool_t isVisible_b(const core::Point2D<T>& i_Pos_rx) const = 0;


  // virtual destructor
  virtual ~ISensor() {};
};

//--------------------------------------------------------------------------
/// @class DefaultSensor
/// @brief Default generic class for implementing ISensor
/// Represents null value for instances of ISensor. If camera instances are created without a sensor instance,
/// singleton container core::Singleton\< DefaultSensor\<T\> \>::getInstance_rx() is set as reference to sensor.
/// Alternatively a camera instance can be initialized by non-singleton instance of this class:
/// Camera( \<anImager\> , \<aLens\>, model::DefaultSensor\<T\>() ) or \<aCamera\>.setSensor_rx(DefaultSensor\<T\>())
//  --------------------------------------------------------------------------


template<typename T>
class DefaultSensor : public ISensor<T>
{
public:

  // getters

  //! implements pure virtual method from IConfig, default sensor is always configured
  virtual bool_t isConfigured_b(void) const
  {
    return true;
  }

  //! get principal point reference
  virtual const core::Point2D<T>& getPpp_rx(void) const
  {
    static core::Point2D<T> s_DefaultPpp_x(math::constants<T>::zero_x());
    return s_DefaultPpp_x;
  }

  //! get pixel size scaling vector by reference
  virtual const core::Point2D<T>& getPsz_rx(void) const
  {
    static core::Point2D<T> s_DefaultPsz_x(math::constants<T>::one_x());
    return s_DefaultPsz_x;
  }

  //! get pixel size of upper left corner of default sensor area
  virtual core::Point2D<T> getUpperLeft_x(void) const
  {
    return core::Point2D<T>( math::numeric_limits<T>::lowest_x(),
                             math::numeric_limits<T>::lowest_x() );
  }

  //! get pixel size of lower right corner of default sensor area
  virtual core::Point2D<T> getLowerRight_x(void) const
  {
    return core::Point2D<T>( math::numeric_limits<T>::max_x(),
                             math::numeric_limits<T>::max_x() );
  }


  // setters

  //! set principal point location, causes assertion: not allowed for default sensor
  virtual void setPpp_v(const core::Point2D<T>&)
  {
    AssertFunction(false,"Tried to set principal point location for default sensor object.");
    return;
  }

  //! set pixel size scaling vector, causes assertion: not allowed for default sensor
  virtual void setPsz_v(const core::Point2D<T>&)
  {
    AssertFunction(false,"Tried to set pixel size for default sensor object");
    return;
  }

  // processing functions

  //! implements pure virtual mehtod of ISensor: function just returns input point (metric coordinates)
  virtual void pixelToMetric_v(const core::Point2D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const
  {
    o_Pos_rx = i_Pos_rx;
    return;
  }

  //! implements pure virtual method of ISensor: function just returns input point (metric coordinates)
  virtual void metricToPixel_v(const core::Point2D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const
  {
    o_Pos_rx = i_Pos_rx;
    return;
  }

  //! check if point location is visible in sensor area
  virtual bool_t isVisible_b(const core::Point2D<T>& i_Pos_rx) const
  {
	static_cast<void> (i_Pos_rx);
    return true;
  }

  //! virtual destructor
  virtual ~DefaultSensor()
  {}



}; // Default Sensor

} /* namespace model */
} /* namespace mecl */

#endif /* SRC_MODEL_ISENSOR_H_ */

/// @}
/// @}
