// --------------------------------------------------------------------------
/// @file Camera.hpp
/// @brief This is the short description of the template module
///
/// Here may follow a longer description for the template module with examples.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Andrej Wagner (andrej.wagner@magna.com)
/// @date Created 14.01.2015
///
// --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include "Camera.h"

#include <ciso646>


namespace mecl {
namespace model {

  // --------------------------------------------------------------------------
  // * Constructors and Destructor
  // --------------------------------------------------------------------------

  // --------------------------------------------------------------------------
  /// @brief Default constructor
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor1
  // --------------------------------------------------------------------------
  template <typename T>
  Camera<T>::Camera()
  : imager_px (& ( core::Singleton< DefaultImager<T> >::getInstance_rx() ) )
  , lens_px   (& ( core::Singleton< NoLens<T> >::getInstance_rx() ) )
  , sensor_px (& ( core::Singleton< DefaultSensor<T> >::getInstance_rx() ) )
  {}

  // --------------------------------------------------------------------------
  /// @brief Constructor for camera without lens and sensor model
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_PinholeConstructor
  /// @snippet ModelTester.cpp Camera_Constructor2
  ///
  /// @param[in] i_Imager_rx Imager object, i.e. Pinhole or Projection
  // --------------------------------------------------------------------------
  // PRQA S 2516 4
  template <typename T>
  Camera<T>::Camera(IImager<T>& i_Imager_rx)
  : imager_px ( &i_Imager_rx )
  , lens_px   ( & (core::Singleton< NoLens<T> >::getInstance_rx() ) )
  , sensor_px ( & (core::Singleton< DefaultSensor<T> >::getInstance_rx()) )
  {}

  // --------------------------------------------------------------------------
  /// @brief Constructor for camera without imager model and sensor model
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_LensConstructor
  /// @snippet ModelTester.cpp Camera_Constructor5
  ///
  /// @param[in] i_Lens_rx Lens object,
  // --------------------------------------------------------------------------
  // PRQA S 2516 4
  template <typename T>
  Camera<T>::Camera(ILens<T>& i_Lens_rx)
  : imager_px ( & (core::Singleton< DefaultImager<T> >::getInstance_rx()) )
  , lens_px   ( &i_Lens_rx )
  , sensor_px ( & (core::Singleton< DefaultSensor<T> >::getInstance_rx()) )
  {}

  // --------------------------------------------------------------------------
  /// @brief Constructor for camera without imager model and lens model
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_SensorConstructor
  /// @snippet ModelTester.cpp Camera_Constructor6
  ///
  /// @param[in] i_Sensor_rx Sensor object
  // --------------------------------------------------------------------------
  // PRQA S 2516 4
  template <typename T>
  Camera<T>::Camera(ISensor<T>& i_Sensor_rx)
  : imager_px ( & (core::Singleton< DefaultImager<T> >::getInstance_rx()) )
  , lens_px   ( & (core::Singleton< NoLens<T> >::getInstance_rx()) )
  , sensor_px ( &i_Sensor_rx )
  {}

  // --------------------------------------------------------------------------
  /// @brief Constructor for camera without imager model
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_LensConstructor
  /// @snippet ModelTester.cpp Camera_SensorConstructor
  /// @snippet ModelTester.cpp Camera_Constructor7
  ///
  /// @param[in] i_Lens_rx Lens object,
  /// @param[in] i_Sensor_rx Sensor object
  // --------------------------------------------------------------------------
  // PRQA S 2516 4
  template <typename T>
  Camera<T>::Camera(ILens<T>& i_Lens_rx, ISensor<T>& i_Sensor_rx)
  : imager_px ( & (core::Singleton< DefaultImager<T> >::getInstance_rx()) )
  , lens_px   ( &i_Lens_rx )
  , sensor_px ( &i_Sensor_rx )
  {}


  // --------------------------------------------------------------------------
  /// @brief Constructor for camera without sensor model
  ///
  ///
  /// @param[in] i_Imager_rx Imager object,
  /// @param[in] i_Lens_rx Lens object
  // --------------------------------------------------------------------------
  // PRQA S 2516 4
  template <typename T>
  Camera<T>::Camera(IImager<T>& i_Imager_rx, ILens<T>& i_Lens_rx)
  : imager_px ( &i_Imager_rx )
  , lens_px   ( &i_Lens_rx )
  , sensor_px ( & (core::Singleton< DefaultSensor<T> >::getInstance_rx()) )
  {}

  // --------------------------------------------------------------------------
  /// @brief Constructor for camera without lens model (no lens)
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_PinholeConstructor
  /// @snippet ModelTester.cpp Camera_LensConstructor
  /// @snippet ModelTester.cpp Camera_Constructor3
  ///
  /// @param[in] i_Imager_rx Imager object
  /// @param[in] i_Sensor_rx Sensor object
  // --------------------------------------------------------------------------
  // PRQA S 2516 4
  template <typename T>
  Camera<T>::Camera(IImager<T>& i_Imager_rx, ISensor<T>& i_Sensor_rx)
  : imager_px ( &i_Imager_rx )
  , lens_px   ( &(core::Singleton< NoLens<T> >::getInstance_rx()) )
  , sensor_px ( &i_Sensor_rx )
  {}

  // --------------------------------------------------------------------------
  /// @brief Constructor for camera with full camera model (imager, lens, sensor)
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_PinholeConstructor
  /// @snippet ModelTester.cpp Camera_LensConstructor
  /// @snippet ModelTester.cpp Camera_SensorConstructor
  /// @snippet ModelTester.cpp Camera_Constructor8
  ///
  /// @param[in] i_Imager_rx Imager object, i.e. Pinhole or Projection
  /// @param[in] i_Lens_rx lens object, i.e. LensRadial or LensCylinder
  /// @param[in] i_Sensor_rx Sensor object
  // --------------------------------------------------------------------------
  // PRQA S 2516 4
  template <typename T>
  Camera<T>::Camera(IImager<T>& i_Imager_rx, ILens<T>& i_Lens_rx, ISensor<T>& i_Sensor_rx)
  : imager_px  (&i_Imager_rx)
  , lens_px    (&i_Lens_rx)
  , sensor_px  (&i_Sensor_rx)
  {}

  // --------------------------------------------------------------------------
  // * Getters and setters
  // --------------------------------------------------------------------------


  // --------------------------------------------------------------------------
  /// @brief Returns true if imager and lens objects are properly configured
  ///
  /// The function returns a boolean indicating if lens and imager objects
  /// have been configured (true) or not (false).
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor1
  /// @snippet ModelTester.cpp Camera_isConfigured_b
  ///
  /// @return Boolean, True = lens and imager configured, False = lens and/or
  /// not configured.
  // --------------------------------------------------------------------------
  template <typename T>
  bool_t Camera<T>::isConfigured_b(void) const
  {
    return   ( true == this->getImager_rx().isConfigured_b() )
          && ( true == this->getLens_rx().isConfigured_b() )
          && ( true == this->getSensor_rx().isConfigured_b() );
  }

  // --------------------------------------------------------------------------
  /// @brief Resets camera instance to default unit camera
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor8
  /// @snippet ModelTester.cpp Camera_reset_v
  // --------------------------------------------------------------------------
  template <typename T>
  void Camera<T>::reset_v(void)
  {
    this->imager_px = & ( core::Singleton< DefaultImager<T> >::getInstance_rx() );
    this->lens_px =   & ( core::Singleton< NoLens<T> >::getInstance_rx() );
    this->sensor_px = & ( core::Singleton< DefaultSensor<T> >::getInstance_rx() );
  }

  // --------------------------------------------------------------------------
  /// @brief Get reference to imager
  ///
  /// Function returns pointer reference to imager object. If no imager object
  /// was set, a reference to the singleton instance of mecl::model::DefaultImager<T>
  /// is returned. This instance is never configured, so also the camera is not in
  /// this case.
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor1
  /// @snippet ModelTester.cpp Camera_getImager_rx
  ///
  /// @return Reference to imager object
  // --------------------------------------------------------------------------
  template <typename T>
  IImager<T>& Camera<T>::getImager_rx(void) const
  {
    return *(this->imager_px);
  }

  // --------------------------------------------------------------------------
  /// @brief Set new imager reference
  ///
  /// Function sets imager pointer reference to imager object supplied in \p
  /// i_Imager_rx.
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor1
  /// @snippet ModelTester.cpp Camera_setImager_v
  ///
  /// @param[in] i_Imager_rx Imager object reference
  /// @return void
  // --------------------------------------------------------------------------
  // PRQA S 2516 4
  template <typename T>
  void Camera<T>::setImager_v(IImager<T>& i_Imager_rx)
  {
    this->imager_px = &i_Imager_rx;
  }

  // --------------------------------------------------------------------------
  /// @brief Get reference to lens
  ///
  /// Function returns reference to lens object. If no lens object was
  /// set, a reference to the singleton instance of mecl::model::NoLens\<T\> class is returned.
  /// This instance is never configured, so also the camera is not in this case.
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor1
  /// @snippet ModelTester.cpp Camera_getLens_rx
  ///
  /// @return Reference to lens object
  // --------------------------------------------------------------------------
  template <typename T>
  ILens<T>& Camera<T>::getLens_rx(void) const
  {
    return *(this->lens_px);
  }

  // --------------------------------------------------------------------------
  /// @brief Set new lens reference
  ///
  /// Function sets lens pointer reference to lens object supplied in \p
  /// i_Lens_rx.
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor1
  /// @snippet ModelTester.cpp Camera_setLens_v
  ///
  /// @param[in] i_Lens_rx Lens object reference
  /// @return void
  // --------------------------------------------------------------------------
  // PRQA S 2516 4
  template <typename T>
  void Camera<T>::setLens_v(ILens<T>& i_Lens_rx)
  {
    this->lens_px = &i_Lens_rx;
  }

  // --------------------------------------------------------------------------
  /// @brief Remove lens reference
  ///
  /// Function sets lens pointer reference to singleton instance of mecl::core::NoLens\<T\>
  /// This instance is always configured.
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_setNoLens_v
  ///
  /// @return void
  // --------------------------------------------------------------------------
  // PRQA S 2516 4
  template <typename T>
  void Camera<T>::setNoLens_v()
  {
    this->lens_px = &( core::Singleton< NoLens<T> >::getInstance_rx() );
  }

  // --------------------------------------------------------------------------
  /// @brief Check if camera has no lens (refers to NoLens\<T\> instance)
  ///
  /// Function checks whether lens reference refers to an instance of type NoLens\<T\>
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor1
  /// @snippet ModelTester.cpp Camera_hasNoLens_b
  ///
  /// @return true : Camera has no lens, false: otherwise
  // --------------------------------------------------------------------------
  template <typename T>
  bool_t Camera<T>::hasNoLens_b() const
  {
    NoLens<T> const* c_NoLens_px = dynamic_cast< NoLens<T>* >(this->lens_px);
    return (c_NoLens_px != NULL);
  }

  // --------------------------------------------------------------------------
  /// @brief Check if camera has no lens and references instance in singleton container for NoLens\<T\>
  ///
  /// Function checks whether lens reference refers to Singleton\<NoLens\<T\>\>::getInstance_rx() or not
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor1
  /// @snippet ModelTester.cpp Camera_hasNoLensSingleton_b
  ///
  /// @return true : Camera references instance in singleton container of NoLens\<T\>, false: otherwise
  // --------------------------------------------------------------------------
  template <typename T>
  bool_t Camera<T>::hasNoLensSingleton_b() const
  {
    NoLens<T> const* c_NoLens_px = dynamic_cast< NoLens<T>* >(this->lens_px);
    return (c_NoLens_px != NULL) &&  core::Singleton< NoLens<T> >::isSingleton_b(*c_NoLens_px);
  }

  // --------------------------------------------------------------------------
  /// @brief Get reference to sensor
  ///
  /// Function returns pointer reference to lens object. If no sensor object was
  /// set, a reference to the singleton instance of mecl::model::DefaultSensor<T>
  /// is returned. This instance is never configured, so also the camera is not in
  /// this case.
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor1
  /// @snippet ModelTester.cpp Camera_getSensor_rx
  ///
  /// @return Reference to sensor object
  // --------------------------------------------------------------------------
  template <typename T>
  ISensor<T>& Camera<T>::getSensor_rx(void) const
  {
    return *( this->sensor_px );
  }

  // --------------------------------------------------------------------------
  /// @brief Set new sensor reference
  ///
  /// Function sets sensor reference to lens object supplied in \p
  /// i_Lens_rx.
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor1
  /// @snippet ModelTester.cpp Camera_setSensor_v
  ///
  /// @param[in] i_Sensor_rx Lens object reference
  /// @return void
  // --------------------------------------------------------------------------
  // PRQA S 2516 4
  template <typename T>
  void Camera<T>::setSensor_v(ISensor<T>& i_Sensor_rx)
  {
    this->sensor_px = &i_Sensor_rx;
  }

  // --------------------------------------------------------------------------
  /// @brief Get Field of view: all
  ///
  /// Get Field of view: all directions within FOV range that are applicable and visible
  ///
  /// @param[in] i_AngleUnit_e Unit of Angles specifing Field of View (radians, degrees)
  ///
  /// @return FieldOfView_s struct specifing Field of View
  /// --------------------------------------------------------------------------
  template<typename T>
  typename Camera<T>::FieldOfViewInfo_s Camera<T>::getFieldOfView_s(AngleUnit_e i_AngleUnit_e) const
  {
    return this->getFieldOfView_s( true, true, i_AngleUnit_e);
  }

  /// Get Field of view: select FOV range according to input flags
  template<typename T>
  typename Camera<T>::FieldOfViewInfo_s Camera<T>::getFieldOfView_s(     bool_t i_IsApplicable_b,
                                                                         bool_t i_IsVisible_b,
                                                                    AngleUnit_e i_AngleUnit_e) const
  {

    FieldOfViewInfo_s v_FOVInfo_s = {};
    v_FOVInfo_s.angleUnit_e = i_AngleUnit_e;

    if (false == i_IsVisible_b)
    {
      if (true == i_IsApplicable_b)
      {
        v_FOVInfo_s = this->getLens_rx().getFieldOfView_s(i_AngleUnit_e);
      }
      else
      {
        // Return FOV an ideal fisheye camera
        const T c_PiTwoTimes_x = i_AngleUnit_e == e_Radians ? math::constants<T>::pi_x() * static_cast<T>(2.0f)
                                                                   : static_cast<T>(360.0f);
        v_FOVInfo_s.horizontal_x = c_PiTwoTimes_x;
        v_FOVInfo_s.centricHorizontal_x = c_PiTwoTimes_x;
        v_FOVInfo_s.vertical_x  = c_PiTwoTimes_x;
        v_FOVInfo_s.centricVertical_x = c_PiTwoTimes_x;
        v_FOVInfo_s.maximal_x   = c_PiTwoTimes_x;
        v_FOVInfo_s.horizontalShift_x = math::constants<T>::zero_x();
        v_FOVInfo_s.verticalShift_x = math::constants<T>::zero_x();
      }
    }
    else
    {
      if (true == i_IsApplicable_b)
      {
        core::Point2D<T> v_MinPos_x;
        core::Point2D<T> v_MaxPos_x;

        this->getSensor_rx().pixelToMetric_v(this->getSensor_rx().getUpperLeft_x(), v_MinPos_x);
        this->getSensor_rx().pixelToMetric_v(this->getSensor_rx().getLowerRight_x(), v_MaxPos_x);
        v_FOVInfo_s = this->getLens_rx().getFieldOfView_s(v_MinPos_x, v_MaxPos_x, i_AngleUnit_e);
      }
      else
      {

      }
    }





    return v_FOVInfo_s;
  }

  // --------------------------------------------------------------------------
  /// @brief Project point in world coordinates onto image sensor coordinate system
  ///
  /// Function project 3D point locations to camera image plane, applies distortion
  /// and converts point location on image plane from metric into pixel coordinates
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Construtor8
  /// @snippet ModelTester.cpp Camera_applyFullProjectionInit_v
  /// @snippet ModelTester.cpp Camera_applyFullProjection_v_1
  ///
  /// @param[in] i_WorldPos_rx        Metric point location in vehicle coordinate system
  /// @param[out] o_ImagePos_rx       Resulting image coordinates in pixels
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::applyFullProjection_v(const core::Point4D<T>& i_WorldPos_rx,
                                              core::Point2D<T>& o_ImagePos_rx) const
  {
    core::Point3D<T> v_CameraPos_x;

    this->applyProjectionW2I_v(i_WorldPos_rx, v_CameraPos_x);
    (void) this->applyDistortion_v(v_CameraPos_x, o_ImagePos_rx);
    this->metricToPixel_v(o_ImagePos_rx, o_ImagePos_rx);

    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Project point in world coordinates onto image sensor coordinate system (variant 1)
  ///
  /// Function project 3D point locations to camera image plane, applies distortion
  /// and converts point location on image plane from metric into pixel coordinates
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Construtor8
  /// @snippet ModelTester.cpp Camera_applyFullProjectionInit_v
  /// @snippet ModelTester.cpp Camera_InitFlags
  /// @snippet ModelTester.cpp Camera_applyFullProjection_v_2
  /// @snippet ModelTester.cpp Camera_AlterFlags_1
  /// @snippet ModelTester.cpp Camera_applyFullProjection_v_2
  /// @snippet ModelTester.cpp Camera_AlterFlags_2
  /// @snippet ModelTester.cpp Camera_applyFullProjection_v_2
  /// @snippet ModelTester.cpp Camera_AlterFlags_3
  /// @snippet ModelTester.cpp Camera_applyFullProjection_v_2
  ///
  /// @param[in] i_WorldPos_rx        Metric point location in vehicle coordinate system
  /// @param[out] o_ImagePos_rx       Resulting image coordinates in pixels
  /// @param[out] o_CheckApplicable_b true, if input point is applicable to lens model, \n
  ///                                 otherwise false (projected point coords \o_ImagePos_rx are not valid)
  /// @param[out] o_CheckVisible_b    true, if projection of input point is visibile in sensor area,\n
  ///                                 otherwise false
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::applyFullProjection_v(
                             const core::Point4D<T>& i_WorldPos_rx,
                                   core::Point2D<T>& o_ImagePos_rx,
                                             bool_t& b_CheckApplicable_rb,
                                             bool_t& b_CheckVisible_rb
                                       ) const
  {
    core::Point3D<T> v_CameraPos_x;

    this->applyProjectionW2I_v(i_WorldPos_rx, v_CameraPos_x);
    this->applyDistortion_v(v_CameraPos_x, o_ImagePos_rx, b_CheckApplicable_rb);
    b_CheckVisible_rb = this->getSensor_rx().isVisible_b(o_ImagePos_rx);
    this->metricToPixel_v(o_ImagePos_rx, o_ImagePos_rx);

    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Project point in world coordinates onto image sensor coordinate system with residual distortion
  ///
  /// Function project 3D point locations to camera image plane, applies distortion
  /// and converts point location on image plane from metric into pixel coordinates
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor8
  /// @snippet ModelTester.cpp Camera_applyFullProjection_v_1
  /// @snippet ModelTester.cpp Camera_applyFullProjectionInit_v
  ///
  /// @param[in] i_WorldPos_rx        Metric point location in vehicle coordinate system
  /// @param[in] i_DistortionLevel_rx Distortion level
  /// @param[out] o_ImagePos_rx       Resulting image coordinates in pixels
  /// @return void
  // --------------------------------------------------------------------------

  template<typename T>
  void Camera<T>::applyFullProjection_v(
                             const core::Point4D<T>& i_WorldPos_rx,
                                            const T& i_DistortionLevel_rx,
                                   core::Point2D<T>& o_ImagePos_rx
                                        ) const
  {
    core::Point3D<T> v_CameraPos_x;

    this->applyProjectionW2I_v(i_WorldPos_rx, v_CameraPos_x);
    (void) this->applyDistortion_v(v_CameraPos_x, i_DistortionLevel_rx, o_ImagePos_rx);
    this->metricToPixel_v(o_ImagePos_rx, o_ImagePos_rx);

    return;
  }


  // --------------------------------------------------------------------------
  /// @brief Project point in world coordinates onto image sensor coordinate system with residual distortion (variant)
  ///
  /// Function project 3D point locations to camera image plane, applies distortion
  /// and converts point location on image plane from metric into pixel coordinates
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Construtor8
  /// @snippet ModelTester.cpp Camera_applyFullProjectionInit_v
  /// @snippet Camera_applyFullProjectionInit_v_3
  /// @snippet ModelTester.cpp Camera_InitFlags
  /// @snippet ModelTester.cpp Camera_applyFullProjection_v_3
  /// @snippet ModelTester.cpp Camera_AlterFlags_1
  /// @snippet ModelTester.cpp Camera_applyFullProjection_v_3
  /// @snippet ModelTester.cpp Camera_AlterFlags_2
  /// @snippet ModelTester.cpp Camera_applyFullProjection_v_3
  /// @snippet ModelTester.cpp Camera_AlterFlags_3
  /// @snippet ModelTester.cpp Camera_applyFullProjection_v_3
  ///
  /// @param[in] i_WorldPos_rx        Metric point location in vehicle coordinate system
  /// @param[in] i_DistortionLevel_rx Distortion level
  /// @param[out] o_ImagePos_rx       Resulting image coordinates in pixels
  /// @param[out] o_CheckApplicable_b true, if input point is applicable to lens model, \n
  ///                                 otherwise false (projected point coords \o_ImagePos_rx are not valid)
  /// @param[out] o_CheckVisible_b    true, if projection of input point is visibile in sensor area,\n
  ///                                 otherwise false
  /// @return void
  // --------------------------------------------------------------------------

  template<typename T>
  void Camera<T>::applyFullProjection_v(
                             const core::Point4D<T>& i_WorldPos_rx,
                                            const T& i_DistortionLevel_rx,
                                   core::Point2D<T>& o_ImagePos_rx,
                                             bool_t& b_CheckApplicable_rb,
                                             bool_t& b_CheckVisible_rb
                                        ) const
  {
    core::Point3D<T> v_CameraPos_x;

    this->applyProjectionW2I_v(i_WorldPos_rx, v_CameraPos_x);
    this->applyDistortion_v(v_CameraPos_x, i_DistortionLevel_rx, o_ImagePos_rx, b_CheckApplicable_rb);
    b_CheckVisible_rb = this->getSensor_rx().isVisible_b(o_ImagePos_rx);
    this->metricToPixel_v(o_ImagePos_rx, o_ImagePos_rx);

    return;
  }


  // --------------------------------------------------------------------------
  /// @brief Project point in image onto Z=0 plane in world coordinates
  ///
  /// Function converts point location in pixels on image plane into metric
  /// coordinates, applies undistortion, and projects point onto arbitrary plane
  /// given its inverse homography to coordinate frame
  /// in world coordinates.
  ///
  /// @param[in] i_ImagePos_rx       Pixel point location in image coordinate system
  /// @param[in] i_InvHomography_rx  Inverse homography matrix, supplied as input\n
  /// argument to avoid redundant validity checks
  /// @param[in] i_Normalize_b       Normalize projected point to W=1 (true=default), or not (false)
  /// @param[out] o_WorldPos_rx       Resulting metric point location on z=0 plane
  /// in vehicle coordinate system
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::applyBackProjection_v(const core::Point2D<T>& i_ImagePos_rx,
                             const core::Matrix3x3<T>& i_InvHomography_rx,
                             core::Point4D<T>& o_WorldPos_rx,
                             const bool_t i_Normalize_b) const
  {
    core::Point2D<T> v_ImagePosMetric_x;
    core::Point3D<T> v_CameraPos_x;

    this->pixelToMetric_v(i_ImagePos_rx, v_ImagePosMetric_x);
    this->applyUndistortion_v(v_ImagePosMetric_x, v_CameraPos_x);

    v_CameraPos_x = i_Normalize_b
                    ? i_InvHomography_rx.mmulFast(v_CameraPos_x).getNormalized()
                    : i_InvHomography_rx.mmulFast(v_CameraPos_x);

    o_WorldPos_rx(0) = v_CameraPos_x(0);
    o_WorldPos_rx(1) = v_CameraPos_x(1);
    o_WorldPos_rx(2) = math::constants<T>::zero_x();
    o_WorldPos_rx(3) = v_CameraPos_x(2);

    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Check if image position can be projected onto a plane implicitly defined homography
  ///
  /// Function checks whether the projected point's homogeneous coordinate
  /// is equal or greater than zero, which is indictates that it can be projected
  /// on the ground plane.
  ///
  /// @param[in] i_ImagePos_rx       Pixel point location in image coordinate system
  /// @param[in] i_InvHomography_rx  Inverse homography matrix, supplied as input
  /// @return   true, if pixel location can be projected to plan, otherwise false
  // --------------------------------------------------------------------------
  template<typename T>
  bool_t Camera<T>::isOnPlane_b(const core::Point2D<T>& i_ImagePos_rx,
                     const core::Matrix3x3<T>& i_InvHomography_rx) const
  {
    core::Point4D<T> v_Pos_x;
    this->applyBackProjection_v(i_ImagePos_rx, i_InvHomography_rx, v_Pos_x);
    return (v_Pos_x.getW() >= math::constants<T>::zero_x() );
  }

  // --------------------------------------------------------------------------
  /// @brief Project point in image onto Z=0 plane in world coordinates
  ///
  /// Function converts point location in pixels on image plane into metric
  /// coordinates, applies undistortion, and projects point onto Z=0 plane
  /// in world coordinates.
  ///
  /// @remark: Inverse homography for Z=0 is recomputed every time the method is called
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor8
  /// @snippet ModelTester.cpp Camera_applyFullProjection_v
  /// @snippet ModelTester.cpp Camera_applyZPlaneBackProjection_v
  ///
  /// @param[in] i_ImagePos_rx       Pixel point location in image coordinate system
  /// argument to avoid redundant validity checks
  /// @param[in] i_Normalize_b       Normalize projected point to W=1 (true=default), or not (false)
  /// @param[out] o_WorldPos_rx       Resulting metric point location on z=0 plane
  /// in vehicle coordinate system
  /// @return void
    // -------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::applyZPlaneBackProjection_v(const core::Point2D<T>& i_ImagePos_rx,
                               core::Point4D<T>& o_WorldPos_rx,
                               const bool_t i_Normalize_b) const
  {
    this->getImager_rx().init_v();
    core::Matrix3x3<T> v_H_x = this->getImager_rx().getZPlaneHomography_x().inverse();
    applyBackProjection_v(i_ImagePos_rx, v_H_x, o_WorldPos_rx, i_Normalize_b);
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Checks if a point in the image can be projected onto Z=0 plane
  ///
  /// Function checks whether the projected point's homogeneous coordinate
  /// is equal or greater than zero, which is indicates that it can be projected
  /// on the ground plane.
  ///
  /// @remark: Inverse homography for Z=0 is recomputed every time the method is called
  ///
  /// @param[in] i_ImagePos_rx       Pixel point location in image coordinate system
  /// @return   true, if pixel location can be projected to Z=0, otherwise false
  // --------------------------------------------------------------------------
  template<typename T>
  bool_t Camera<T>::isOnZPlane_b(const core::Point2D<T> i_ImagePos_rx) const
  {
    core::Point4D<T> v_ProjCamPos_x;
    this->applyZPlaneBackProjection_v(i_ImagePos_rx, v_ProjCamPos_x, false);
    return ( v_ProjCamPos_x.getW() >= math::constants<T>::zero_x() );
  }

  // --------------------------------------------------------------------------
  /// @brief Normalize homogeneous 2D point
  ///
  /// Function normalizes 2D homogeneous point coordinates with factor given
  /// in \p i_Factor_x.
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor8
  /// @snippet ModelTester.cpp Camera_applyNormalization_v
  ///
  /// @param[in]  i_Pos_rx    Homogeneous 2D point coordinates
  /// @param[in]  i_Factor_x  Normalization factor
  /// @param[out] o_Pos_rx    Normalized cartesian point coordinates
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::applyNormalization_v(const core::Point3D<T>& i_Pos_rx,
                                           const T& i_Factor_x,
                                  core::Point2D<T>& o_Pos_rx) const
  {
    o_Pos_rx  =
        static_cast< core::Point2D<T> > (
            (i_Pos_rx.getNormalized(i_Factor_x)).template subVector<2>(0)
        );
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Project 3D point location to camera image plane.
  ///
  /// Function initializes imager object and applies world to imager coordinate
  /// projection.
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor8
  /// @snippet ModelTester.cpp Camera_applyProjectionW2I_v
  ///
  /// @param[in]  i_Pos_rx    Metric point location in vehicle coordinate system
  /// @param[out] o_Pos_rx    Resulting metric image coordinates
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::applyProjectionW2I_v(const core::Point4D<T>& i_Pos_rx,
                                             core::Point3D<T>& o_Pos_rx) const
  {
    // check and calculate e.g. Projection Matrix for Pinhole<T>
    this->getImager_rx().init_v();

    this->getImager_rx().applyProjectionW2I_v(i_Pos_rx, o_Pos_rx);

    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Apply lens distortion
  ///
  /// Function applies lens distortion to undistorted points in camera coordinate
  /// system
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor8
  /// @snippet ModelTester.cpp Camera_applyDistortion_v
  ///
  /// @param[in] i_Pos_rx       3D coordinate of point to be distorted
  /// @param[out] o_Pos_rx      Distorted and normalized 2D coordinates of input
  /// point i_Pos_rx in image coordinate system.
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::applyDistortion_v(const core::Point3D<T>& i_Pos_rx,
                                          core::Point2D<T>& o_Pos_rx) const
  {
    (void) this->getLens_rx().applyDistortion_b(i_Pos_rx, o_Pos_rx);
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Apply lens distortion
  ///
  /// Function applies lens distortion to undistorted points in camera coordinate
  /// system
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor8
  /// @snippet ModelTester.cpp Camera_applyDistortion_v
  ///
  /// @param[in] i_Pos_rx       3D coordinate of point to be distorted
  /// @param[out] o_Pos_rx      Distorted and normalized 2D coordinates of input
  /// point i_Pos_rx in image coordinate system.
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::applyDistortion_v(const core::Point3D<T>& i_Pos_rx,
                                          core::Point2D<T>& o_Pos_rx,
                                          bool_t& b_CheckApplicable_rb ) const
  {
    if ( false == b_CheckApplicable_rb )
    {
        b_CheckApplicable_rb =
            this->getLens_rx().applyDistortion_b(i_Pos_rx, o_Pos_rx);
    }
    else
    {
      b_CheckApplicable_rb = this->getLens_rx().isApplicable_b(i_Pos_rx);
      if (true == b_CheckApplicable_rb)
      {
        (void) this->getLens_rx().applyDistortion_b(i_Pos_rx, o_Pos_rx);
      }
    }
    return;
  }


  // --------------------------------------------------------------------------
  /// @brief Apply lens distortion
  ///
  /// Function applies lens distortion to undistorted points in camera coordinate
  /// system with residual correction factor
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor8
  /// @snippet ModelTester.cpp Camera_applyDistortion_v
  ///
  /// @param[in] i_Pos_rx       3D coordinate of point to be distorted
  /// @param[in] i_DistortionLevel_rx Distortion level to be applied
  /// @param[out] o_Pos_rx      Distorted and normalized 2D coordinates of input
  /// point i_Pos_rx in image coordinate system.
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::applyDistortion_v(const core::Point3D<T>& i_Pos_rx,
                                        const T& i_DistortionLevel_rx,
                               core::Point2D<T>& o_Pos_rx) const
  {

    this->applyDistortion_v(i_Pos_rx, o_Pos_rx);

    // apply residual distortion correction ?
    if ( not (math::isAboutZero_b(i_DistortionLevel_rx) ) )
    {
      // point near horizon of image plane? (elevation = 90°) ?
      if (true == math::isAboutZero_b( i_Pos_rx.getW() ) )
      {
        o_Pos_rx = math::numeric_limits<T>::infinity_x();
      }
      else
      {
        o_Pos_rx +=
                  ( ( i_Pos_rx.getNormalized().template subVector<2>() - o_Pos_rx)
                * i_DistortionLevel_rx );
      }
    }
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Apply lens distortion checking/indicating applicability
  ///
  /// Function applies lens distortion to undistorted points in camera coordinate
  /// system with residual correction factor. Furthermore the method takes
  /// a boolean flag as \p b_CheckApplicable_b bi-directional parameter. If its
  /// input value is true, the input point \p i_Pos_rx is checked if its is
  /// applicable to the lens model (withing calibrated range) first and,
  /// only in this case, the distorted point in \p o_Pos_rx is updated.
  /// If the input value for \p b_CheckApplicable_b is set false, \p o_Pos_rx
  /// is updated in any case.
  ///
  /// @param[in] i_Pos_rx             3D coordinate of point to be distorted
  /// @param[in] i_DistortionLevel_rx Distortion level to be applied
  /// @param[out] o_Pos_rx            Distorted and normalized 2D coordinates of input
  /// @param[out] b_CheckApplicable_b Bi-directional flag handling applicablity
  /// point i_Pos_rx in image coordinate system.
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::applyDistortion_v(const core::Point3D<T>& i_Pos_rx,
      const T& i_DistortionLevel_rx,
      core::Point2D<T>& o_Pos_rx,
      bool_t& b_CheckApplicable_rb) const
      {

    this->applyDistortion_v(i_Pos_rx, o_Pos_rx, b_CheckApplicable_rb);

    if ( true == b_CheckApplicable_rb )
    {
      // apply residual distortion correction ?
      if ( not (math::isAboutZero_b(i_DistortionLevel_rx) ) )
      {
        // point near horizon of image plane? (elevation = 90°) ?
        if (true == math::isAboutZero_b( i_Pos_rx.getW() ) )
        {
          o_Pos_rx = math::numeric_limits<T>::infinity_x();
        }
        else
        {
          o_Pos_rx +=
              ( ( i_Pos_rx.getNormalized().template subVector<2>() - o_Pos_rx)
                  * i_DistortionLevel_rx );
        }
      }
    }
    return;
  }


  // --------------------------------------------------------------------------
  /// @brief Removes distortion of metric image coordinates
  ///
  /// The function applies undistortion of 2D image input point \p i_Pos_rx and returns undistorted 3D point
  /// in camera coordinate system.
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor8
  /// @snippet ModelTester.cpp Camera_applyUndistortion_v
  ///
  /// @param[in] i_Pos_rx       Distorted 2D coordinate of image point to be undistorted
  /// @param[out] o_Pos_rx      Undistorted 3D coordinates of input point i_Pos_rx in camera coordinate system.
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::applyUndistortion_v(const core::Point2D<T>& i_Pos_rx,
                                 core::Point3D<T>& o_Pos_rx) const
  {
    this->getLens_rx().applyUndistortion_v(i_Pos_rx, o_Pos_rx);
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Convert point location from metric to pixel
  ///
  /// The function converts 2D point location on image plane from metric
  /// dimension into pixel coordinates
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor8
  /// @snippet ModelTester.cpp Camera_metricToPixel_v
  ///
  /// @param[in] i_Pos_rx     Image points in metric coordinates
  /// @param[out] o_Pos_rx    Image points locations in pixel
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::metricToPixel_v(const core::Point2D<T>& i_Pos_rx,
                             core::Point2D<T>& o_Pos_rx) const
  {
    this->getSensor_rx().metricToPixel_v(i_Pos_rx, o_Pos_rx);
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Convert point location from metric to pixel
  ///
  /// The function converts 2D point location on image plane from metric
  /// dimension into pixel coordinates
  ///
  /// @par Example usage:
  ///
  /// @param[in] i_Pos_rx     Image points in metric coordinates
  /// @param[out] o_Pos_rx    Image points locations in pixel
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::metricToPixel_v(const core::Point2D<T>& i_Pos_rx,
                                        core::Point2D<T>& o_Pos_rx,
                                                  bool_t& b_CheckVisible_rb) const
  {
    const bool_t c_CheckVisible_x = this->getSensor_rx().isVisible_b(i_Pos_rx);
    if ( (false == b_CheckVisible_rb) || (true == c_CheckVisible_x) )
    {
      this->getSensor_rx().metricToPixel_v(i_Pos_rx, o_Pos_rx);
    }
    b_CheckVisible_rb = c_CheckVisible_x;
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Convert point location from pixel to metric
  ///
  /// The function converts 2D point location on image plane from pixel
  /// coordinates into metric dimension
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Camera_Constructor8
  /// @snippet ModelTester.cpp Camera_pixelToMetric_v
  ///
  /// @param[in] i_Pos_rx    Image points location in pixels
  /// @param[out] o_Pos_rx   Image point in metric coordinates
  /// @return void
  // --------------------------------------------------------------------------
  template<typename T>
  void Camera<T>::pixelToMetric_v(const core::Point2D<T>& i_Pos_rx,
                                        core::Point2D<T>& o_Pos_rx) const
  {
    this->getSensor_rx().pixelToMetric_v(i_Pos_rx, o_Pos_rx);
    return;
  }

  // --------------------------------------------------------------------------
   /// @brief Convert point location from pixel to metric
   ///
   /// The function converts 2D point location on image plane from pixel
   /// coordinates into metric dimension
   ///
   /// @par Example usage:
   /// @snippet ModelTester.cpp Camera_Constructor8
   /// @snippet ModelTester.cpp Camera_pixelToMetric_v
   ///
   /// @param[in] i_Pos_rx    Image points location in pixels
   /// @param[out] o_Pos_rx   Image point in metric coordinates
   /// @return void
   // --------------------------------------------------------------------------
   template<typename T>
   void Camera<T>::pixelToMetric_v(const core::Point2D<T>& i_Pos_rx,
                                         core::Point2D<T>& o_Pos_rx,
                                                   bool_t& o_CheckVisible_rb) const
   {
     this->getSensor_rx().pixelToMetric_v(i_Pos_rx, o_Pos_rx);
     o_CheckVisible_rb = this->getSensor_rx().isVisible_b(o_Pos_rx);
     return;
   }

   template< template<typename T> class IM,
             template<typename T> class LM,
             template<typename T> class SM,
             typename T>
   GenericCamera<IM, LM, SM, T>::GenericCamera(void)
   : imager_x()
   , lens_x()
   , sensor_x()
   {
     AssertFunction(dynamic_cast<IImager<T>*> (&this->imager_x) != NULL,
                        "Template argument 1 for generic camera class is not an implementer of IImager.");
     AssertFunction(dynamic_cast<ILens<T>*> (&this->lens_x) != NULL,
                        "Template argument 2 for generic camera class is not an implementer of ILens.");
     AssertFunction(dynamic_cast<ISensor<T>*> (&this->sensor_x) != NULL,
                        "Template argument 3 for generic camera class is not an implementer of ISensor.");
   };

   template< template<typename T> class IM,
             template<typename T> class LM,
             template<typename T> class SM,
             typename T>
   GenericCamera<IM, LM, SM, T>::GenericCamera(const GenericCamera<IM, LM, SM, T>& i_GenCam_rx)
   : imager_x(i_GenCam_rx.getImager_rx())
   , lens_x(i_GenCam_rx.getLens_rx())
   , sensor_x(i_GenCam_rx.getSensor_rx())
   {};

   template< template<typename T> class IM,
             template<typename T> class LM,
             template<typename T> class SM,
             typename T>
   GenericCamera<IM, LM, SM, T>::GenericCamera(const Camera<T>& i_Camera_rx)
   : imager_x(IM<T>::get_rx(i_Camera_rx.getImager_rx()))
   , lens_x(LM<T>::get_rx(i_Camera_rx.getLens_rx()))
   , sensor_x(SM<T>::get_rx(i_Camera_rx.getSensor_rx()))
   {};

   template< template<typename T> class IM,
             template<typename T> class LM,
             template<typename T> class SM,
             typename T>
   const Camera<T>& GenericCamera<IM, LM, SM, T>::getAsCamera_rx(void) const
   {
     return Camera<T>(this->imager_x, this->lens_x, this->sensor_x);
   }

   template< template<typename T> class IM,
             template<typename T> class LM,
             template<typename T> class SM,
             typename T>
   IM<T>& GenericCamera<IM, LM, SM, T>::getImager_rx(void) const
   {
     return imager_x;
   }

   template< template<typename T> class IM,
             template<typename T> class LM,
             template<typename T> class SM,
             typename T>
   LM<T>& GenericCamera<IM, LM, SM, T>::getLens_rx(void) const
   {
     return lens_x;
   }

   template< template<typename T> class IM,
             template<typename T> class LM,
             template<typename T> class SM,
             typename T>
   SM<T>& GenericCamera<IM, LM, SM,T>::getSensor_rx(void) const
   {
     return sensor_x;
   }

   template< template<typename T> class IM,
             template<typename T> class LM,
             template<typename T> class SM,
             typename T>
   void GenericCamera<IM, LM, SM, T>::setImager_v(const IImager<T>& i_Imager_rx)
   {
     const IM<T>* v_Imager_px = dynamic_cast<IM<T>*>(&i_Imager_rx);
     AssertFunction(v_Imager_px != NULL, "Type of input parameter is not compliant with generic imager class");
     memcpy(&this->imager_x, v_Imager_px, sizeof(IM<T>));
     return;
   }

   template< template<typename T> class IM,
             template<typename T> class LM,
             template<typename T> class SM,
             typename T>
   void GenericCamera<IM, LM, SM, T>::setLens_v(const ILens<T>& i_Lens_rx)
   {
     const IM<T>* v_Lens_px = dynamic_cast<LM<T>*>(&i_Lens_rx);
     AssertFunction(v_Lens_px != NULL, "Type of input parameter is not compliant with generic lens class");
     memcpy(&this->lens_x, v_Lens_px, sizeof(LM<T>));
     return;
   }

   template< template<typename T> class IM,
             template<typename T> class LM,
             template<typename T> class SM,
             typename T>
   void GenericCamera<IM, LM, SM, T>::setSensor_v(const ISensor<T>& i_Sensor_rx)
   {
     const IM<T>* v_Sensor_px = dynamic_cast<SM<T>*>(&i_Sensor_rx);
     AssertFunction(v_Sensor_px != NULL, "Type of input parameter is not compliant with generic sensor class");
     memcpy(&this->lens_x, v_Sensor_px, sizeof(SM<T>));
     return;
   }

   template< template<typename T> class IM,
             template<typename T> class LM,
             template<typename T> class SM,
             typename T>
   template< typename T1>
   GenericCamera<IM, LM, SM, T1> GenericCamera<IM, LM, SM, T>::convert_x(void) const
   {
     return GenericCamera<IM, LM, SM, T1> (this->getAsCamera_rx().convert_x<T1>());
   }

   template< template<typename T> class IM,
             template<typename T> class LM,
             template<typename T> class SM,
             typename T>
   template< typename T1 >
   GenericCamera<IM, LM, SM, T>::operator GenericCamera<IM, LM, SM, T1>() const
   {
     // PRQA S 1124 1 The template parameter is supposed to be of fundametnal type (float, double)
     return this>convert_x<T1>();
   }

}
}

#endif

/// @}
/// @}
