//--------------------------------------------------------------------------
/// @file Camera.h
/// @brief Definition and implementation of standard camera model in MECL
///
/// MECL standard camera model
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
/// @date 01/22/2015
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef MECL_MODEL_CAMERA_H_
#define MECL_MODEL_CAMERA_H_



#include "IImager.h"
#include "ILens.h"
#include "ISensor.h"

#include "core/Core.h"
#include "core/MeclTypes.h"
#include "core/Point.h"
#include "core/Singleton.h"
#include "math/Math.h"
#include "Pinhole.h"
#include "LensRadial.h"
#include "LensCylinder.h"
#include "Sensor.h"

namespace mecl
{
namespace model
{

//--------------------------------------------------------------------------
/// @class Camera
/// @brief Camera model
///
/// Class implements combination object consisting of sensor, lens and imager forming
/// the Camera model. The sensor contains the resolution and the pixel position of the
/// principal point in the sensor frame. If no sensor is defined sensor resolution is 1.0 [mm/px].
/// Lens object is either LensRadial or LensCylinder or the default lens (NoLens) modeling a camera
/// model without any lens distortion.
/// The imager object is either Projection Pinhole, or a unit projection (DefaultImager)
// --------------------------------------------------------------------------
template<typename T>
class Camera
{

public:

  /// Embedded POD struct for field of view info
  typedef typename ILens<T>::FieldOfView_s FieldOfViewInfo_s;

  // -------------------------------------------------------------------------
  // * Constructors and Destructor
  // -------------------------------------------------------------------------

  /// Default constructor: instantiates a unit camera (all sub-models default)
  Camera(void);

  /// C-tor with no lens and default sensor (pinhole projection to metric image coordinates)
  explicit Camera(IImager<T>& i_Imager_rx);


  /// C-tor with default imager and default sensor (lens projection to metric image coordinates)
  explicit Camera(ILens<T>& i_Lens_rx);

  /// C-tor with default imager and no lens (sensor projection mapping of 3D metric to pixel)
  explicit Camera(ISensor<T>& i_Sensor_rx);

  /// C-tor with defaut imager (lens projection to pixel image coordinates)
  explicit Camera(ILens<T>& i_Lens_rx, ISensor<T>& i_Sensor_rx);

  /// C-tor with default sensor (pinhole + lens projection to metric image coordinates)
  explicit Camera(IImager<T>& i_Imager_rx, ILens<T>& i_Lens_rx);

  /// C-tor with no lens (pinhole projection to pixel image coordinates)
  explicit Camera(IImager<T>& i_Imager_rx, ISensor<T>& i_Sensor_rx);

  /// C-tor with full model (full camera model maps 3D metric coordinates to pixel coordinates)
  explicit Camera(IImager<T>& i_Imager_rx, ILens<T>& i_Lens_rx, ISensor<T>& i_Sensor_rx);

  //--------------------------------------------------------------------------
  /// @brief Destructor is not virtual since class has no super class
  //--------------------------------------------------------------------------
  ~Camera(void) {}

  // -------------------------------------------------------------------------
  // * Getters and setters
  // -------------------------------------------------------------------------

  /// Returns true if imager and lens objects are properly configured
  bool_t isConfigured_b(void) const;

  /// Resets camera instance to default unit camera
  void reset_v(void);

  /// Get reference to imager
  IImager<T>& getImager_rx(void) const;

  /// Set new imager reference
  void setImager_v(IImager<T>& i_Imager_rx);

  /// Get reference to lens
  ILens<T>& getLens_rx(void) const;

  /// Set new lens reference
  void setLens_v(ILens<T>& i_Lens_rx);

  /// Remove lens reference
  void setNoLens_v(void);

  /// Check if camera has no lens (refers to NoLens<T> instance)
  bool_t hasNoLens_b() const;

  /// Check if camera has no lens and references instance in singleton container for Nolens<T>
  bool_t hasNoLensSingleton_b() const;

  /// Get reference to sensor
  ISensor<T>& getSensor_rx(void) const;

  /// Set new sensor reference
  void setSensor_v(ISensor<T>& i_Sensor_rx);

  /// Get Field of view: all directions within FOV range that are applicable and visible
  FieldOfViewInfo_s getFieldOfView_s(AngleUnit_e i_AngleUnit_e = e_Degrees) const;

  /// Get Field of view: select FOV range according to input flags
  FieldOfViewInfo_s getFieldOfView_s(     bool_t i_IsApplicable_b,
                                          bool_t i_IsVisible_b,
                                          AngleUnit_e i_AngleUnit_e = e_Degrees) const;

  // -------------------------------------------------------------------------
  // * Processing methods
  // -------------------------------------------------------------------------

  /// Project point in world coordinates onto image sensor coordinate system
  void applyFullProjection_v(const core::Point4D<T>& i_WorldPos_rx,
                                   core::Point2D<T>& o_ImagePos_rx) const;

  /// Project point in world coordinates onto image sensor coordinate system (variant)
  void applyFullProjection_v(const core::Point4D<T>& i_WorldPos_rx,
                                   core::Point2D<T>& o_ImagePos_rx,
                                             bool_t& b_CheckApplicable_rb,
                                             bool_t& b_CheckVisible_rb) const;

  /// Project point in world coordinates onto image sensor coordinate system with residual distortion
  void applyFullProjection_v(const core::Point4D<T>& i_WorldPos_rx,
                                            const T& i_DistortionLevel_rx,
                                   core::Point2D<T>& o_ImagePos_rx) const;

  /// Project point in world coordinates onto image sensor coordinate system with residual distortion (variant)
  void applyFullProjection_v(const core::Point4D<T>& i_WorldPos_rx,
                                            const T& i_DistortionLevel_rx,
                                   core::Point2D<T>& o_ImagePos_rx,
                                             bool_t& b_CheckApplicable_rb,
                                             bool_t& b_CheckVisible_rb) const;

  /// Project point in image frame onto a plane implicitly defined by a homography matrix
  void applyBackProjection_v(  const core::Point2D<T>& i_ImagePos_rx,
                             const core::Matrix3x3<T>& i_InvHomography_rx,
                                     core::Point4D<T>& o_WorldPos_rx,
                                     const bool_t i_Normalize_b = true) const;

  /// Check if image position can be projected onto a plane implicitly defined by a homography matrix
  bool_t isOnPlane_b(const core::Point2D<T>& i_ImagePos_rx,
                     const core::Matrix3x3<T>& i_InvHomography_rx) const;

  /// Project point in image frame  onto Z=0 plane in world coordinate system
  void applyZPlaneBackProjection_v(const core::Point2D<T>& i_ImagePos_rx,
                                         core::Point4D<T>& o_WorldPos_rx,
                                         const bool i_Normalize_b = true) const;

  /// Check if image position can be projected to ground plane Z=0
  bool_t isOnZPlane_b(const core::Point2D<T> i_ImagePos_rx) const;

  /// Normalize homogeneous 2D point
  void applyNormalization_v(const core::Point3D<T>& i_Pos_rx,
                                           const T& i_Factor_x,
                                  core::Point2D<T>& o_Pos_rx) const;

  /// Project 3D point location to camera image plane.
  void applyProjectionW2I_v(const core::Point4D<T>& i_Pos_rx,
                                  core::Point3D<T>& o_Pos_rx) const;

  /// Add lens distortion
   void applyDistortion_v(const core::Point3D<T>& i_Pos_rx,
                                core::Point2D<T>& o_Pos_rx) const;

   /// Add lens distortion (variant)
  void applyDistortion_v(const core::Point3D<T>& i_Pos_rx,
                               core::Point2D<T>& o_Pos_rx,
                                         bool_t& b_CheckApplicable_rb) const;


  /// Apply residual distortion correction
  void applyDistortion_v(const core::Point3D<T>& i_Pos_rx,
                                        const T& i_DistortionLevel_rx,
                               core::Point2D<T>& o_Pos_rx) const;

  /// Apply residual distortion correction (variant)
  void applyDistortion_v(const core::Point3D<T>& i_Pos_rx,
                                         const T& i_DistortionLevel_rx,
                                core::Point2D<T>& o_Pos_rx,
                                          bool_t& b_CheckApplicable_rb) const;

  /// Removes distortion of metric image coordinates
  void applyUndistortion_v(const core::Point2D<T>& i_Pos_rx,
                                 core::Point3D<T>& o_Pos_rx) const;
  
  /// Convert point location from metric to pixel
  void metricToPixel_v(const core::Point2D<T>& i_Pos_rx,
                               core::Point2D<T>& o_Pos_rx) const;


  /// Convert point location from metric to pixel (variant)
  void metricToPixel_v(const core::Point2D<T>& i_Pos_rx,
                             core::Point2D<T>& o_Pos_rx,
                                       bool_t& b_CheckVisible_rb) const;

  /// Convert point location from pixel to metric
  void pixelToMetric_v(const core::Point2D<T>& i_Pos_rx,
                             core::Point2D<T>& o_Pos_rx) const;

  /// Convert point location from pixel to metric (variant)
  void pixelToMetric_v(const core::Point2D<T>& i_Pos_rx,
                             core::Point2D<T>& o_Pos_rx,
                                       bool_t& o_CheckVisible_rb) const;

private:

  IImager<T>* imager_px;    ///< Pointer reference to imager model object
  ILens<T>*   lens_px;      ///< Pointer reference to lens model object
  ISensor<T>* sensor_px;    ///< Pointer reference to sensor model object

};
// template<typename T, class IM, class LM, class SM>




template< template<typename T> class IM,
          template<typename T, uint32_t ou, uint32_t od> class LM,
          template<typename T> class SM,
          typename T, uint32_t ou, uint32_t od >
class GenericCameraRadial
{
public:

  // default c
  GenericCameraRadial(void);

  // copy c-tor for Camera object (deep copy)
  GenericCameraRadial(const Camera<T>& i_Camera_rx);

  ~GenericCameraRadial(void);

  Camera<T>& getCamera_rx(void) const;
  IM<T>& getImager_rx(void) const;
  LM<T,ou,od>& getLens_rx(void) const;
  SM<T>& getSensor_rx(void) const;

  template<typename T1>
  GenericCameraRadial< IM, LM, SM, T1,ou, od> convert_x(void) const;

  template<typename T1>
  operator GenericCameraRadial<IM, LM, SM, T1, ou, od>() const;

private:

  IM<T> imager_x;
  LM<T,ou,od> lens_x;
  SM<T> sensor_x;

};

template< template<typename T> class IM,
          template<typename T> class LM,
          template<typename T> class SM,
          typename T>
class GenericCamera
{
public:

  /// default c-tor
  GenericCamera(void);

  /// copy c-tor (deep copy)
  GenericCamera(const GenericCamera& i_GenCam_rx);

  /// copy c-tor for Camera object (deep copy)
  explicit GenericCamera(const Camera<T>& i_Camera_rx);

  /// non-virtual destructor
  ~GenericCamera(void) {};

  ///
  const Camera<T>& getAsCamera_rx(void) const;

  IM<T>& getImager_rx(void) const;
  LM<T>& getLens_rx(void) const;
  SM<T>& getSensor_rx(void) const;

  void setImager_v(const IImager<T>& i_Imager_rx);
  void setLens_v(const ILens<T>& i_Lens_rx);
  void setSensor_v(const ISensor<T>& i_Sensor_rx);

  template<typename T1>
  GenericCamera< IM, LM, SM, T1> convert_x(void) const;

  template<typename T1>
  operator GenericCamera<IM, LM, SM, T1>() const;

private:

  IM<T> imager_x;
  LM<T> lens_x;
  SM<T> sensor_x;

};


// template<typename T>
// class LensRadiald5d7u : public LensRadial<T, 5, 7> {};
//
// GenericCamera<Pinhole, LensRadiald5d7u, Sensor, float32_t> genCam;
// GenericCamera<Pinhole, LensCylinder, Sensor, float32_t> genCam2;

} // namespace model
} // namespace mecl

#include "Camera.hpp"

#endif // MECL_MODEL_CAMERA_H_
/// @}
/// @}
