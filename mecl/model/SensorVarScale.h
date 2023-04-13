//--------------------------------------------------------------------------
/// @file SensorVarScale.h
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
/// @date 12/06/2016
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef SRC_MODEL_SENSORVARSCALE_H_
#define SRC_MODEL_SENSORVARSCALE_H_

#include "ModelTypes.h"
#include "Sensor.h"
#include "mecl/core/Point.h"

namespace mecl
{

namespace model
{

//--------------------------------------------------------------------------
/// @class SensorVarScale
/// @brief Image sensor model with varying scaling
///
/// Model of imaging sensor providing functionality for transforming points
/// between metric and pixel coordinate spaces. The scale is not constant and
/// depends on the position in the uniform sensor which is contained in this
/// instance.
// --------------------------------------------------------------------------
template<typename T, SensorType_e SType_e = e_VarScaleY, uint32_t RLen_u32 = 1000U>
class SensorVarScale : public ISensor<T>
{
public:

  // --------------------------------------------------------------------------
  /// @struct Config_s
  /// @brief Sensor configuration parameters
  // --------------------------------------------------------------------------
  struct Config_s
  {
      typename Sensor<T>::Config_s sensorUniform_s;
      T* varScaleBuffer_px;
      uint32_t varScaleBufferSize_u32;
  };

  // --------------------------------------------------------------------------
  /// @struct RevBufferInfo_s
  /// @brief Values for pixel position calculation using the reverse scaling values
  // --------------------------------------------------------------------------
  struct RevBufferInfo_s
  {
    T startPos_x;        //< metric position of first entry in reverse scale buffer
    T stepSizeMetric_x;  //< metric distance of the adjacent entries in reverse scale buffer
  };

  // --------------------------------------------------------------------------
  // Constructors/Destructor + Update/Retrieve configuration data
  // --------------------------------------------------------------------------

  /// Default constructor
  SensorVarScale(void);

  /// Constructor copying configuration from existing configuration data set
  explicit SensorVarScale(const Config_s& i_Config_rs);

  /// Constructor initializing  uniform sensor + VarScale Buffer/Size
  explicit SensorVarScale(const Sensor<T>& i_Sensor_rx,
                          T* i_VarScaleBuffer_px,
                          const uint32_t i_VarScaleBufferSize_u32);

  /// Initializer for VarScaleBuffer
  void init_v(void);

  /// Virtual destructor
  virtual ~SensorVarScale() {};

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
  core::Point2D<T>& getMinPos_rx(void) const;

  /// Get lower right corner of sensor area in metric coordinates
  core::Point2D<T>& getMaxPos_rx(void) const;

  /// Get upper left corner of sensor area in pixel coordinates
  virtual core::Point2D<T> getUpperLeft_x(void) const;

  /// Get lower right corner of sensor area in pixel coordinates
  virtual core::Point2D<T> getLowerRight_x(void) const;

  /// Copy variable scale buffer
  uint32_t copyVarScaleBuffer_u32(T* o_VarScaleBuffer_px,
                                  const uint32_t i_StartPos_u32 = 0,
                                  const uint32_t i_NumOfEntries_u32 = 0) const;

  /// Copy reverse scale buffer
  uint32_t copyRevScaleBuffer_u32(T* o_RevScaleBuffer_px,
                                  const uint32_t i_StartPos_u32 = 0,
                                  const uint32_t i_NumOfEntries_u32 = 0) const;

  /// Copy reverse buffer info values
  void copyRevBufferInfo_v(RevBufferInfo_s& o_Info_rs) const;

  // Get reverse scale buffer size
  uint32_t getRevScaleBufferSize_u32(void) const;


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
  SensorVarScale<T1, SType_e, RLen_u32> convert_x(void) const;

  // --------------------------------------------------------------------------
  // Down-casting methods and cast operator
  // --------------------------------------------------------------------------

  /// Get a Sensor reference by down-casting a reference to ISensor<T>
  static SensorVarScale<T, SType_e, RLen_u32>& get_rx(ISensor<T>& i_ISensor_rx);

  /// Get an instance of this class by down-casting a reference to ISensor<T1> of different type
  template<typename T1>
  static SensorVarScale<T, SType_e, RLen_u32> convert_x(ISensor<T1>& i_ISensor_rx);

  /// Cast operator for conversion to different base type T1
  template<typename T1>
  operator SensorVarScale<T1, SType_e, RLen_u32>() const;

  // --------------------------------------------------------------------------
  // Processing methods
  // --------------------------------------------------------------------------

  /// Convert point location from pixel to metric
  virtual void pixelToMetric_v(const core::Point2D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const;

  /// Convert point location from metric to pixel
  virtual void metricToPixel_v(const core::Point2D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const;

  /// Check if metric point location is visible
  virtual bool_t isVisible_b(const core::Point2D<T>& i_Pos_rx) const;

  /// Get variable scaling value for pixel coordinates
  T getVarScaleForPixel_x(const core::Point2D<T>& i_Pos_rx) const;

  /// Get variable scaling value for metric coordinates
  T getVarScaleForMetric_x(const core::Point2D<T>& i_Pos_rx) const;


private:

  // --------------------------------------------------------------------------
  // Private data members
  // --------------------------------------------------------------------------

  Sensor<T>  sensorUniform_x;                   ///< Instance of uniform sensor
  T*         varScaleBuffer_px;                 ///< Pointer to a buffer of variable scaling entries
  uint32_t   varScaleBufferSize_u32;            ///< Number of variable scaling entries

  T          revScaleBuffer_px[RLen_u32];       ///< Reverse Buffer entries
  T          revMetricStartPos_x;               ///< Metric position at start of buffer
  T          stepSizeMetric_x;                  ///< Stepsize of reverse Buffer (metric value = startpose + index * stepsize)

  // metric sensor area
  core::Point2D<T> minPos_x;                    ///< Upper left corner of visible sensor area, different from uniform sensor
  core::Point2D<T> maxPos_x;                    ///< Lower right corener of visible sensor area, different from uniform sensor

  // private status flags
  bool_t initialized_b;                         ///< true, if SensorVarScale buffers have bean calculated, false otherwise
  bool_t configured_b;                          ///< true, if SensorVarScale is configured, false otherwise

  // --------------------------------------------------------------------------
  // Private methods
  // --------------------------------------------------------------------------

  /// Calculate variable scaling buffer
  void calcVarScaleBuffer_v(void);

  /// Calculate reverse scaling buffer (called by init_v()
  void calcRevScaleBuffer_v(void);

  /// Calculate scale value from pixel coordinate
  void calcScale_v(const mecl::core::Point2D<T>& i_Pos_rx, T& o_VarScale_rx) const;

  /// Calculate scale value from metric coordinate
  void calcRevScale_v(const mecl::core::Point2D<T>& i_Pos_rx, T& o_VarScale_rx) const;

  /// Find next non-zero entry is varScaleBuffer (need for initialization), true if not found and end of buffer reached, false otherwise
  bool_t calcNextNonZeroEntry_v(const uint32_t i_Idx_u32, uint32_t& o_Idx_u32) const;

}; // class Sensor


} /* namespace model */
} /* namespace mecl */

#include "SensorVarScale.hpp"

#endif /* SRC_MODEL_SENSORVARSCALE_H_ */

/// @}
/// @}





