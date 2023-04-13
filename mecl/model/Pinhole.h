//--------------------------------------------------------------------------
/// @file Pinhole.h
/// @brief Projective pinhole camera model
///
/// The Pinhole model provides a projection from metric 3D world to 2D image pixel coordinates.
/// The projection is parameterized by POD structures for intrinsic and extrinsic orientation.
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

#ifndef MECL_MODEL_PINHOLE_H_
#define MECL_MODEL_PINHOLE_H_

//! @cond
// PRQA S 1020 1 // macro is used by MKS
#define Pinhole_D_VERSION_ID "$Id: Pinhole.h 1.41 2016/11/30 07:17:42EST Zollner, Helmut (SAI_HeZol) draft  $"
//! @endcond

#include "core/MeclAssert.h"
#include "core/Point.h"
#include "core/RotationMatrix.h"
#include "Projection.h"
#include "ModelTypes.h"

namespace mecl
{
namespace model
{

//--------------------------------------------------------------------------
/// @class Pinhole
/// @brief Pinhole camera model
///
/// Model of ideal perspective central projection:
/// Projection is strictly linear and applies projective transformation matrix
/// to known world coordinates to project them to the image plane
// --------------------------------------------------------------------------
template<typename T>
class Pinhole : public IImager<T>
{
public:

  // --------------------------------------------------------------------------------------------
  // Embedded data structs (POD)

  // --------------------------------------------------------------------------
  /// @struct Intrinsic_s
  /// @brief Set of intrinsic model parameters
  // --------------------------------------------------------------------------
  struct Intrinsic_s
  {
    // Parameters for K matrix
    T foclX_x;                                      ///< Focal length x axis
    T foclY_x;                                      ///< Focal length y axis
    typename core::Point2D<T>::Config_s pppCfg_x;   ///< Position of principal point (metric)
  };

  // --------------------------------------------------------------------------
  /// @struct Extrinsic_s
  /// @brief Set of extrinsic model parameters
  // --------------------------------------------------------------------------
  struct Extrinsic_s
  {
    typename core::Point3D<T>::Config_s pos_x;                      ///< Location of camera center
    typename core::RotationMatrix<T>::EulerAngles_s eulerAngles_s;  ///< Euler Angles wrt. to mounting position
    model::PreRoll_e preRoll_e;                                     ///< Pre-rotation about z-axis (depends on mounting position)
    model::RotationType_e rotationType_e;                           ///< WorldToCamera or CameraToWorld Rotation Type
  };

  
  // --------------------------------------------------------------------------
  /// @struct Config_s
  /// @brief Configuration data set
  // --------------------------------------------------------------------------
  struct Config_s
  {
    Extrinsic_s extrinsic_s; ///< position dependent extrinsic parameter of pinhole model
    Intrinsic_s intrinsic_s; ///< position independent intrinsic parameters of pinhole model
  };

  // --------------------------------------------------------------------------------------------
  // c-tors + destructor

  /// Default constructor
  Pinhole(void);

  /// Constructor copying configuration from existing configuration data set
  explicit Pinhole(const Config_s& i_Config_rs);

  /// Virtual destructor
  virtual ~Pinhole(void) {}

  // --------------------------------------------------------------------------------------------
  // getters

  /// Get intrinsic calibration matrix K
  virtual core::Matrix3x3<T> getKMatrix_x(void) const;

  /// Get extrinsic rotation matrix R
  core::RotationMatrix<T> getRotationMatrix_x(void) const;

  /// Get translation vector of this pinhole camera model
  core::Point3D<T> getTranslationVector_x(void) const;

  /// Get translation vector of this pinhole camera model with given RotationMatrix
  core::Point3D<T> getTranslationVector_x(const core::RotationMatrix<T>& i_RotMat_rx) const;

  /// Get camera center of this pinhole camera model
  core::Point3D<T> getCameraCenter_x(void) const;

  /// Get camera center of this pinhole camera model with given translation vector
  core::Point3D<T> getCameraCenter_x(const core::Point3D<T>& i_T_rx) const;

  /// Get camera center of this pinhole camera model with given rotation matrix and translation vector
  core::Point3D<T> getCameraCenter_x(const core::RotationMatrix<T>& i_RotMat_rx,
                                     const core::Point3D<T>& i_T_rx) const;

  /// Get Euler angles according to extrinsic camera setup
  typename core::RotationMatrix<T>::EulerAngles_s getEulerAngles_s(AngleUnit_e i_AngleUnit_e = e_Radians) const;

  /// Get Euler angles according to extrinsic camera setup with given rotation matrix
  typename core::RotationMatrix<T>::EulerAngles_s getEulerAngles_s(const core::RotationMatrix<T>& i_RotMat_rx,
                                                                   AngleUnit_e i_AngleUnit_e = e_Radians) const;

  /// Get reference to projection matrix
  virtual core::Matrix<T, 3, 4>& getProjectionMatrix_rx(void);

  /// Get copy of projection matrix
  virtual core::Matrix<T, 3, 4> copyProjectionMatrix_x() const;

  /// Get planar homography of image points (located on z=0 plane in camera coordinate frame) onto z=0 plane in world coordinate frame
  virtual const core::Matrix3x3<T> getZPlaneHomography_x(void) const;

  /// Get an orthonormal base of the projection as 3x3 Matrix
  const core::Matrix3x3<T> getOrthonormalBase_x(void) const;

  /// Get pointer reference to extrinsic parameters for pinhole in configuration data
  const Extrinsic_s& getExtrinsic_rs(void) const;

  /// Get a copy of extrinsic parameters for this pinhole in configuration data POD
  void copyExtrinsic_v(Extrinsic_s& o_Extrinsic_rs) const;

  /// Get a copy of extrinsic parameter of this pinhole of different base type T1
  template<typename T1>
  void copyExtrinsic_v(typename Pinhole<T1>::Extrinsic_s& o_Extrinsic_rs) const;

  /// Get a copy of extrinsic parameters for this pinhole in volatile configuration data POD
  void copyExtrinsicVolatile_v(volatile Extrinsic_s& o_Extrinsic_rs) const;

  /// Get pointer reference to intrinsic parameters for pinhole in configuration data
  const Intrinsic_s& getIntrinsic_rs(void);

  /// Get a copy of intrinsic parameters for this pinhole in configuration data POD
  void copyIntrinsic_v(Intrinsic_s& o_Intrinsic_rs) const;

  /// Get a copy of intrinsic parameters of this pinhole of different base type T1
  template<typename T1>
  void copyIntrinsic_v(typename Pinhole<T1>::Intrinsic_s& o_Intrinsic_rs) const;

  /// Get a copy of intrinsic parameters for this pinhole in volatile configuration data POD
  void copyIntrinsicVolatile_v(volatile Intrinsic_s& o_Intrinsic_rs) const;

  /// Get a copy of all configuration data for this pinhole in configuration data POD
  void copyConfig_v(Config_s& o_Config_rs) const;

  /// Get a copy of all configuration data for this pinhole of different base type T1
  template<typename T1>
  void copyConfig_v(typename Pinhole<T1>::Config_s& o_Config_rs) const;

  /// Get a copy of all configuration data for this pinhole in volatile configuration data POD
  void copyConfigVolatile_v(volatile Config_s& o_Config_rs) const;

  /// Return true if pinhole was properly configured
  virtual bool_t isConfigured_b(void) const;

  // --------------------------------------------------------------------------------------------
  // setters and (re)-configuring methods

  /// Update configuration
  void updateConfig_v(const Config_s& i_Config_rs);

  /// Update configuration with POD struct of different base type T1
  template<typename T1>
  void updateConfig_v(const typename Pinhole<T1>::Config_s& i_Config_rs);

  /// Update only exterior orientation
  void updateExtrinsic_v(const Config_s& i_Config_rs);

  /// Update only exterior orientation with POD struct of different base type T1
  template<typename T1>
  void updateExtrinsic_v(const typename Pinhole<T1>::Config_s& i_Config_rs);

  /// Initialization of internal data structures,
  virtual void init_v(void);

  /// Calculate projection matrix
  void calc_v(mecl::core::Matrix<T, 3, 4>& o_Projection_rx) const;

  /// (re) set pre-roll factor (change camera position)
  void setPreRoll_v(const mecl::model::PreRoll_e& i_PreRoll_re);

  /// Set Extrinsic parameters for pinhole in configuration data
  void setExtrinsic_v(const Extrinsic_s& i_Extrinsic_rs);

  /// Set Extrinsic parameter for pinhole in configuration data with POD struct of different base type T1
  template<typename T1>
  void setExtrinsic_v(const typename Pinhole<T1>::Extrinsic_s& i_Extrinsic_rs);

  /// Set Intrinsic parameters for pinhole in configuration data
  void setIntrinsic_v(const Intrinsic_s& i_Intrinsic_rs);

  /// Set Intrinsic parameters for pinhole in configuration data with POD struct of different base type T1
  template<typename T1>
  void setIntrinsic_v(const typename Pinhole<T1>::Intrinsic_s& i_Intrinsic_rs);

  // --------------------------------------------------------------------------------------------
  // convert to instance of different base type

  template<typename T1>
  Pinhole<T1> convert_x(void) const;

  // --------------------------------------------------------------------------------------------
  // static methods for down-casts and conversions

  /// Try to get Pinhole model instance from IImager reference
  static Pinhole<T>& get_rx(IImager<T>& i_Imager_rx);

  /// Try to get a copy of Pinhole model instance of different base type from IImager reference
  template<typename T1>
  static Pinhole<T> convert_x(IImager<T1>& i_Imager_rx);

  // --------------------------------------------------------------------------------------------
  // operator for casting to instance of different base type T1
  template<typename T1>
  operator Pinhole<T1>() const;

  // --------------------------------------------------------------------------------------------
  // processing methods

  /// Apply projection from projective world 3D coordinates to projective image coordinates in imager fram
  virtual void applyProjectionW2I_v(const core::Point4D<T>& i_Pos_rx,
                                          core::Point3D<T>& o_Pos_rx) const;

private:

  // --------------------------------------------------------------------------------------------
  // static class literals

  static const PitchOffset_e pitchOffset_e;     ///< Pitch offset about x-axis
  static const Extrinsic_s extrinsicDefault_s;  ///< default values for extrinsics
  static const Intrinsic_s intrinsicDefault_s;  ///< default values for intrinsics


  // --------------------------------------------------------------------------------------------
  // private members

  Extrinsic_s extrinsics_s;               ///< Extrinsic model parameters
  Intrinsic_s intrinsics_s;               ///< Intrinsic model parameters

  model::Projection<T> projection_x;     ///< Internal projection object with projection matrix

  bool_t proMatValid_b;                  ///< true if projection matrix is valid
};

} // namespace model
} // namespace mecl

#include "Pinhole.hpp"

#endif // MECL_MODEL_PINHOLE_H_
/// @}
/// @}

