//--------------------------------------------------------------------------
/// @file Sensor.h
/// @brief Definition of standard sensor model in MECL.
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
/// @date 04/15/2015
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef MODEL_SENSOR_H_
#define MODEL_SENSOR_H_

#include "ModelTypes.h"
#include "ISensor.h"
#include "core/Point.h"

namespace mecl
{

namespace model
{

//--------------------------------------------------------------------------
/// @class Sensor
/// @brief Image sensor model
///
/// Model of imaging sensor providing functionality for transforming points
/// between metric and pixel coordinate spaces
// --------------------------------------------------------------------------
template<typename T>
class Sensor : public ISensor<T>
{
public:

  // --------------------------------------------------------------------------
  /// @struct Config_s
  /// @brief Sensor configuration parameters
  // --------------------------------------------------------------------------
  struct Config_s
  {
      typename core::Point2D<T>::Config_s pppCfg_x;           ///< Principal point location in pixel
      typename core::Point2D<T>::Config_s pszCfg_x;           ///< Pixel sizes in millimeter/pixel
      ImageOriginPosition_e imageOriginPositionCfg_e;         ///< Sensor origin xy-position in camera coordinate frame
      typename core::Point2D<T>::Config_s pos1Cfg_x;          ///< First pixel position defining rectangular sensor area
      typename core::Point2D<T>::Config_s pos2Cfg_x;          ///< Second pixel position defining rectangular sensor area
  };

  // --------------------------------------------------------------------------
  // Constructors/Destructor + Update/Retrieve configuration data
  // --------------------------------------------------------------------------

  /// Default constructor
  Sensor(void);

  /// Constructor copying configuration from existing configuration data set
  explicit Sensor(const Config_s& i_Config_rs);

  /// Virtual destructor
  virtual ~Sensor() {};

  /// Copy sensor configuration to POD object
  void copyConfig_v(Config_s& o_Config_rs) const;

  /// Copy sensor configuration to volatile POD object
  void copyConfigVolatile_v(volatile Config_s& o_Config_rs) const;

  /// Copy sensor configuration to POD object of (different) base type T1
  template<typename T1>
  void copyConfig_v(typename Sensor<T1>::Config_s& o_Config_rs) const;

  /// Update configuration
  void updateConfig_v(const Config_s& i_Config_rs);

  /// Update configuration by POD struct of (different) base type T1
  template<typename T1>
  void updateConfig_v(const typename Sensor<T1>::Config_s& i_Config_rs);

  // --------------------------------------------------------------------------
  // Getters
  // --------------------------------------------------------------------------

  /// Get configuration status
  virtual bool_t isConfigured_b(void) const;

  /// Get reference to principal point location in pixel
  virtual const core::Point2D<T>& getPpp_rx() const;

  /// Get reference to image resolution
  virtual const core::Point2D<T>& getPsz_rx() const;

  /// Get position of metric image origin
  ImageOriginPosition_e getImageOriginPosition_e() const;

  /// Get upper left corner of sensor area in metric coordinates
  const core::Point2D<T>& getMinPos_rx(void) const;

  /// Get lower right corner of sensor area in metric coordinates
  const core::Point2D<T>& getMaxPos_rx(void) const;

  /// Get upper left corner of sensor area in pixel coordinates
  virtual core::Point2D<T> getUpperLeft_x(void) const;

  /// Get lower right corner of sensor area in pixel coordinates
  virtual core::Point2D<T> getLowerRight_x(void) const;

  // --------------------------------------------------------------------------
  // Setters
  // --------------------------------------------------------------------------

  /// Set principal point
  virtual void setPpp_v(const core::Point2D<T>& i_Ppp_x);

  /// Set image resolution
  virtual void setPsz_v(const core::Point2D<T>& i_Psz_x);

  /// Set position of sensor origin in frame
  void setImageOriginPosition_v(const ImageOriginPosition_e i_ImageOriginPosition_e);

  /// Set upper left corner of valid sensor area in pixel coordinates
  void setUpperLeft_v(const core::Point2D<T>& v_Pos_rx);

  /// Set lower right corner of valid sensor area in pixel coordinates
  void setLowerRight_v(const core::Point2D<T>& v_Pos_rx);

  // --------------------------------------------------------------------------
  // Conversions
  // --------------------------------------------------------------------------

  /// Create a copy of this instance of different base type T1
  template<typename T1>
  Sensor<T1> convert_x(void) const;

  // --------------------------------------------------------------------------
  // Down-casting methods and cast operator
  // --------------------------------------------------------------------------

  /// Get a Sensor reference by down-casting a reference to ISensor<T>
  static Sensor<T>& get_rx(ISensor<T>& i_ISensor_rx);

  /// Get an instance of this class by down-casting a reference to ISensor<T1> of different type
  template<typename T1>
  static Sensor<T> convert_x(ISensor<T1>& i_ISensor_rx);

  /// Cast operator for conversion to different base type T1
  template<typename T1>
  operator Sensor<T1>() const;

  // --------------------------------------------------------------------------
  // Processing methods
  // --------------------------------------------------------------------------

  /// Convert point location from pixel to metric
  virtual void pixelToMetric_v(const core::Point2D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const;

  /// Convert point location from metric to pixel
  virtual void metricToPixel_v(const core::Point2D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const;

  /// Check if metric point location is visible
  virtual bool_t isVisible_b(const core::Point2D<T>& i_Pos_rx) const;


private:

  // --------------------------------------------------------------------------
  // Private data members
  // --------------------------------------------------------------------------

  core::Point2D<T> ppp_x;                         ///< Principle point location in pixel
  core::Point2D<T> psz_x;                         ///< Pixel sizes in millimeter/pixel (sensor resolution)
  ImageOriginPosition_e imageOriginPosition_e;    ///< Position of the image origin
  core::Point2D<T> minPos_x;                      ///< Metric coordinate of the pixel in upper left corner
  core::Point2D<T> maxPos_x;                      ///< Metric coordinate of the pixel in lower left corner

  bool_t configured_b;

  // --------------------------------------------------------------------------
  // Private methods
  // --------------------------------------------------------------------------

  /// Change sensor orientation
  void changeOrientation_v(const core::Point2D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const;

  /// Initialize min and max Position
  void initMinMax_v();

}; // class Sensor

} /* namespace model */
} /* namespace mecl */

#include "Sensor.hpp"

#endif /* MODEL_SENSOR_H_ */

/// @}
/// @}
