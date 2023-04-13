//--------------------------------------------------------------------------
/// @file LensRadial.h
/// @brief Radial lens model
///
/// The lens model provides a warping and an un-warping function.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef MECL_MODEL_LENSRADIAL_H_
#define MECL_MODEL_LENSRADIAL_H_


#include "math/Math.h"
#include "math/FPolynomial.h"
#include "core/MeclAssert.h"
#include "core/Matrix.h"
#include "core/Point.h"
#include "ILens.h"

namespace mecl
{
namespace model
{

//--------------------------------------------------------------------------
/// @class Lens
/// @brief Radial lens model
///
/// Model of a radial camera lens.
/// Implements radial lens correction functionality (distortion/undistortion)
// --------------------------------------------------------------------------
template<typename T, uint32_t od=5, uint32_t ou=5>
class LensRadial : public ILens<T>
{
public:

  /// @struct Config_s
  /// @brief Configuration data set
  struct Config_s
  {
    T world2image_x[od+1];	 ///< Radial distortion polynomial (world to image)
    T image2world_x[ou+1];	 ///< Radial undistortion polynomial (image to world)
    T elevationMaxCfg_x;        ///< Maximal applicable elevation angle in degrees,
                             ///< valid range is [0,180]
  };

  /// Default constructor
  LensRadial(void);

  /// Constructor with initialization
  explicit LensRadial(const Config_s& i_LensConfig_rs);

  /// Virtual destructor
  virtual ~LensRadial(void) {};

  /// Copy configuration of lens parameters to POD
  void copyConfig_v(Config_s& o_Config_rs) const;

  /// Copy configuration of lens parameters to POD of different base type T1
  template<typename T1>
  void copyConfig_v(typename LensRadial<T1, od, ou>::Config_s& o_Config_rs) const;

  /// Copy configuration of lens parameters to volatile POD
  void copyConfigVolatile_v(volatile Config_s& o_Config_rs) const;

  /// Update configuration of lens parameters
  void updateConfig_v(const Config_s& i_LensConfig_rs);

  /// Update configuration of lens parameter with POD of different base type T1
  template<typename T1>
  void updateConfig_v(const typename LensRadial<T1, od, ou>::Config_s& i_Config_rs);

  /// Returns true if lens was properly configured
  virtual bool_t isConfigured_b(void) const;

  /// Applies lens distortion to undistorted points in camera coordinate system
  virtual bool_t applyDistortion_b(const core::Point3D<T>& i_Pos_rx,
                                         core::Point2D<T>& o_Pos_rx) const;

  /// Removes distortion of metric image coordinates
  virtual void applyUndistortion_v(const core::Point2D<T>& i_Pos_rx,
                                         core::Point3D<T>& o_Pos_rx) const;

  /// Checks 3D coordinates if it is applicable to the lens model
  virtual bool_t isApplicable_b(const core::Point3D<T>& i_Pos_rx) const;

  /// Get field of view for which this lens model is applicable
  virtual typename ILens<T>::FieldOfView_s getFieldOfView_s(AngleUnit_e i_AngleUnit_e = e_Degrees) const;

  /// Get field of view for a image area defined by rectangle spanned by 2 metric coordinates
  virtual typename ILens<T>::FieldOfView_s getFieldOfView_s(const core::Point2D<T>& i_Pos1_rx,
                                                            const core::Point2D<T>& i_Pos2_rx,
                                                            AngleUnit_e i_AngleUnit_e = e_Degrees) const;

  /// convert this lens to a different base type
  template<typename T1>
  LensRadial<T1, od, ou> convert_x(void) const;

  /// try to get radial lens model from ILens reference
  static LensRadial<T, od, ou>& get_rx(ILens<T>& i_ILens_rx);

  /// try to convert radial lens model from ILens reference of different base type
  template<typename T1>
  static LensRadial<T, od, ou> convert_x(ILens<T1>& i_Lens_rx);

  /// cast operator for conversion
  template<typename T1>
  operator LensRadial<T1, od, ou>() const;

private:

  Config_s coeff_s;       ///< Coefficients of distortion+udistortion polynomialys
  T elevationMax_x;       ///< Maximal elevation angle in radians (@attention: configugration value is in degrees!)
  bool_t   configured_b;  ///< Flag indicating if lens object has been configured (true) or not (false)

};



} // namespace model
} // namespace mecl

#include "LensRadial.hpp"

#endif // MECL_MODEL_LENSRADIAL_H_
/// @}
/// @}
