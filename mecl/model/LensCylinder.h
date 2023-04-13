//--------------------------------------------------------------------------
/// @file LensCylinder.h
/// @brief Cylinder lens model
///
/// The lens model provides a warping and an un-warping function.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Hagen Marczok (hagen.marczok@magna.com), Helmut Zollner (helmut.zollner@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef MECL_MODEL_LENSCYLINDER_H_
#define MECL_MODEL_LENSCYLINDER_H_

#include "math/Math.h"
#include "core/MeclTypes.h"
#include "core/Matrix.h"
#include "core/Point.h"
#include "ILens.h"

namespace mecl
{
namespace model
{

//--------------------------------------------------------------------------
/// @class LensCylinder
/// @brief Cylinder lens model
///
/// Model of a cylinder camera lens.
/// Implements cylinder lens correction functionality (distortion/undistortion)
///
/// Projects the metric image onto the lateral area of an elliptic cylinder.
/// In general the cylinder lens has elliptic cross sections.
/// The model parameter excentricity describes the ratio of minor : major axis of
/// the elliptic cross sections (x-radius : z-radius) where major axis radius
/// is assumed to be equal to one.
/// By setting excentricity to one the cylinder's cross sections are circles.
///
/// Distortion : f: (X,Y,Z) -> (x,y), where x is arc length on cylinder
/// alpha = arctan( X / (Z * excentricity) )
/// rho = sqrt(X^2 + (Z * excentricity)^2)
/// x = alpha  * excentricity
/// y = Y / rho
///
/// Undistortion: f: (x,y) -> (X,Y,Z), where x is arc length on cylinder
/// alpha = x / excentricity
/// X = sin (alpha) * excentricity
/// Y = y
/// Z = cos (alpha)
///
/// @remarks
/// In order to simplify the model description, the definition of excentricity\n
/// deviates from usual mathematical terminology here.
///
// --------------------------------------------------------------------------
template<typename T>
class LensCylinder : public ILens<T>
{
public:

  /// @struct Config_s
  /// @brief Configuration data set
  struct Config_s
  {
    T excentricity_x;      //< Eccentricity of cylinder cross sections
  };

  /// Default constructor
  LensCylinder(void);

  /// Constructor with initialization
  explicit LensCylinder(const Config_s& i_LensConfig_rs);

  /// Virtual destructor
  virtual ~LensCylinder(void) {};

  /// Copy configuration of lens parameter to POD struct
  void copyConfig_v(Config_s& o_LensConfig_rs) const;

  /// Copy configuration of lens parameter to volatile POD struct
  void copyConfigVolatile_v(volatile Config_s& o_LensConfig_rs) const;

  /// Copy configuration of lens parameter to POD struct of different base type T1
  template<typename T1>
  void copyConfig_v(typename LensCylinder<T1>::Config_s& o_LensConfig_rs) const;

  /// Configuration of lens parameters
  void updateConfig_v(const Config_s& i_LensConfig_rs);

  /// Configure lens parameter by POD struct of different base type
  template<typename T1>
  void updateConfig_v(const typename LensCylinder<T1>::Config_s& i_LensConfig_rs);

  /// Returns true if lens was properly configured
  virtual bool_t isConfigured_b(void) const;

  /// Applies lens distortion to undistorted points in camera coordinate system
  virtual bool_t applyDistortion_b(const core::Point3D<T>& i_Pos_rx,
                                         core::Point2D<T>& o_Pos_rx) const;

  /// Removes distortion of metric image coordinates
  virtual void applyUndistortion_v(const core::Point2D<T>& i_Pos_rx,
                                         core::Point3D<T>& o_Pos_rx) const;

  /// Checks if 3D coordinate is applicable to cylinder lens model
  virtual bool_t isApplicable_b(const core::Point3D<T>& i_Pos_rx) const;

  /// Gets field of view for which this lens model is applicable
  virtual typename ILens<T>::FieldOfView_s getFieldOfView_s(AngleUnit_e i_AngleUnit_e = e_Degrees) const;

  /// Gets field of view for a image area defined by rectangle spanned by 2 metric coordinates
  virtual typename ILens<T>::FieldOfView_s getFieldOfView_s(const core::Point2D<T>& i_Pos1_rx,
                                                            const core::Point2D<T>& i_Pos2_rx,
                                                            AngleUnit_e i_AngleUnit_e = e_Degrees) const;

  /// Convert this lens to a different base type
  template<typename T1>
  LensCylinder<T1> convert_x(void) const;

  /// Try to convert cylinder lens from ILens reference
  static LensCylinder<T>& get_rx(ILens<T>& i_ILens_rx);

  /// Try to convert cylinder lens from ILens reference of different base type
  template<typename T1>
  static LensCylinder<T> convert_x(ILens<T1>& i_ILens_rx);

  /// Cast operator for conversion to different base type
  template<typename T1>
  operator LensCylinder<T1>() const;

private:

  T excentr_x;       ///< Excentricity of cylinder's cross sections
  T excentricitySqr_x;    ///< Squared value for excentricity (only computed once)
  bool_t configured_b;    ///< Flag indicating if lens object has been configured (true) or not (false)
};

} // namespace model
} // namespace mecl

#include "LensCylinder.hpp"

#endif // MECL_MODEL_LENSCYLINDER_H_
/// @}
/// @}
