/// @file SingleView.hpp
/// @brief Implementation of SingleView class
///
/// File contains implementation of SingleView class. This class provides
/// functionality for the defining configuration of and projection between
/// virtual and real camera and vice versa.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
/// @date created on 10/15/15
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef MECL_MODEL_SINGLEVIEW_HPP_
#define MECL_MODEL_SINGLEVIEW_HPP_

//! @cond
// PRQA S 1020 1 // macro is used by MKS
//! #define SingleView_D_VERSION_ID "$Id: SingleView.hpp 1.4 2016/04/22 10:49:00EDT Zollner, Helmut (SAI_HeZol) draft  $"
//! @endcond

#include "SingleView.h"

namespace mecl {

namespace model {

// --------------------------------------------------------------------------
  /// @brief Default constructor
  ///
  /// @par Example use:
  /// @snippet ModelTester.cpp SingleView_Constructor1
  // --------------------------------------------------------------------------
template<typename T>
SingleView<T>::SingleView()
  : realCam_px(NULL)
  , syntCam_px(NULL)
  , realCorrectionLvl_x(math::constants<T>::zero_x())
  , syntCorrectionLvl_x(math::constants<T>::zero_x())
  , initializedSynt2Real_b(false)
  , initializedReal2Synt_b(false)
{}

// --------------------------------------------------------------------------
/// @brief Constructor with camera references and default distortion level
///
/// @attention Examples on camera object configuration and instantation are given in
/// the Camera class.
///
/// @par Example use:
/// @snippet ModelTester.cpp SingleView_Constructor2
///
/// @param[in] realCam_rx Real camera model object
/// @param[in] syntCam_rx Synthetic camera model object
// --------------------------------------------------------------------------
template<typename T>
SingleView<T>::SingleView(Camera<T>& realCam_rx, Camera<T>& syntCam_rx)
  : realCam_px(&realCam_rx)
  , syntCam_px(&syntCam_rx)
  , realCorrectionLvl_x(math::constants<T>::zero_x())
  , syntCorrectionLvl_x(math::constants<T>::zero_x())
  , initializedSynt2Real_b(false)
  , initializedReal2Synt_b(false)
{}

// --------------------------------------------------------------------------
/// @brief Constructor with camera references and real camera distortion level
///
/// @attention Examples on camera object configuration and instantation are given in
/// the Camera class.
///
/// @par Example use:
/// @snippet ModelTester.cpp SingleView_Constructor3
///
/// @param[in] realCam_rx Real camera model object
/// @param[in] syntCam_rx Synthetic camera model object
/// @param[in] i_RealDistortionCorrectionLvl_rx Distortion level for real camera
// --------------------------------------------------------------------------
template<typename T>
SingleView<T>::SingleView(Camera<T>& realCam_rx, Camera<T>& syntCam_rx,
                     const T& i_RealDistortionCorrectionLvl_rx)
 : realCam_px(&realCam_rx)
 , syntCam_px(&syntCam_rx)
 , realCorrectionLvl_x(i_RealDistortionCorrectionLvl_rx)
 , syntCorrectionLvl_x(math::constants<T>::zero_x())
 , initializedSynt2Real_b(false)
 , initializedReal2Synt_b(false)
{}

// --------------------------------------------------------------------------
/// @brief Constructor with camera references and camera specific distortion
/// levels
///
/// @attention Examples on camera object configuration and instantiation are given in
/// the Camera class.
///
/// @par Example use:
/// @snippet ModelTester.cpp SingleView_Constructor4
///
/// @param[in] realCam_rx Real camera model object
/// @param[in] syntCam_rx Synthetic camera model object
/// @param[in] i_RealDistortionCorrectionLvl_rx Distortion level for real camera
/// @param[in] i_SyntDistortionCorrectionLvl_rx Distortion level for synthetic camera
// --------------------------------------------------------------------------
template<typename T>
SingleView<T>::SingleView(Camera<T>& realCam_rx, Camera<T>& syntCam_rx,
                       const T& i_RealDistortionCorrectionLvl_rx,
                       const T& i_SyntDistortionCorrectionLvl_rx)
 : realCam_px(&realCam_rx)
 , syntCam_px(&syntCam_rx)
 , realCorrectionLvl_x(i_RealDistortionCorrectionLvl_rx)
 , syntCorrectionLvl_x(i_SyntDistortionCorrectionLvl_rx)
 , initializedSynt2Real_b(false)
 , initializedReal2Synt_b(false)
{}

// --------------------------------------------------------------------------
 /// @brief Get reference to real to synthetic homography matrix
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_getRealToSyntheticHomography_rx
 ///
 /// @return Returns const reference to real to synthetic homography matrix
 // --------------------------------------------------------------------------
template<typename T>
void SingleView<T>::init_v(void)
{
  AssertFunction( this->isConfigured_b(),"Error initializing. View is not configured.");
  this->calcReal2SyntHomography_v();
  this->calcSynt2RealHomography_v();
  return;
}

// --------------------------------------------------------------------------
 /// @brief Get reference to real to synthetic homography matrix
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_getRealToSyntheticHomography_rx
 ///
 /// @return Returns const reference to real to synthetic homography matrix
 // --------------------------------------------------------------------------
template<typename T>
const core::Matrix3x3<T>& SingleView<T>::getRealToSyntheticHomography_rx() const
{
  AssertFunction(true == this->initializedReal2Synt_b,
                "Homography not initialized yet.");
  return this->real2SyntHomography_x;
}

// --------------------------------------------------------------------------
 /// @brief Get reference to synthetic to real homography matrix
 ///
 /// @attention Examples on camera object configuration and instantation are given in
 /// the Camera class.
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_getSyntheticToRealHomography_rx
 ///
 /// @return Returns const reference to synthetic to real homography matrix
 // --------------------------------------------------------------------------
 template<typename T>
 const core::Matrix3x3<T>& SingleView<T>::getSyntheticToRealHomography_rx() const
 {
   AssertFunction(true == this->initializedSynt2Real_b,
                  "Homography not initialized yet.");
   return this->synt2RealHomography_x;
 }

 // --------------------------------------------------------------------------
 /// @brief Returns true if camera objects are properly configured
 ///
 /// The function returns a boolean indicating if real and synthetic camera objects
 /// have been configured (true) or not (false).
 ///
 /// @par Example usage:
 /// @snippet ModelTester.cpp SingleView_Constructor1
 /// @snippet ModelTester.cpp SingleView_isConfigured_b
 ///
 /// @return Boolean, True = both camera configured, False = one or both cameras
 /// not configured
 // --------------------------------------------------------------------------
 template<typename T>
 bool_t SingleView<T>::isConfigured_b(void) const
 {
   return (NULL != this->realCam_px) && (NULL != this->syntCam_px);
 }

 // --------------------------------------------------------------------------
 /// @brief Set new real camera reference
 ///
 /// Function sets internal real camera to camera object reference supplied in \p
 /// i_RealCam_rx.
 ///
 /// @par Example usage:
 /// @snippet ModelTester.cpp SingleView_Constructor1
 /// @snippet ModelTester.cpp SingleView_setRealCam_v
 ///
 /// @param[in] i_RealCam_rx Real camera object reference
 // --------------------------------------------------------------------------
 template<typename T>
 void SingleView<T>::setRealCam_v(Camera<T>& i_RealCam_rx)
 {
   this->realCam_px = &i_RealCam_rx;
   this->initializedReal2Synt_b = false;
   this->initializedSynt2Real_b = false;
   return;
 }

 // --------------------------------------------------------------------------
 /// @brief Set synthetic camera reference
 ///
 /// Function sets internal synthetic camera to camera object reference supplied in \p
 /// i_SyntCam_rx.
 ///
 /// @par Example usage:
 /// @snippet ModelTester.cpp SingleView_Constructor1
 /// @snippet ModelTester.cpp SingleView_setSyntCam_v
 ///
 /// @param[in] i_SyntCam_rx Synthetic camera object reference
 // --------------------------------------------------------------------------
 template<typename T>
 void SingleView<T>::setSyntCam_v(Camera<T>& i_SyntCam_rx)
 {
   this->syntCam_px = &i_SyntCam_rx;
   this->initializedReal2Synt_b = false;
   this->initializedSynt2Real_b = false;
   return;
 }

 // --------------------------------------------------------------------------
/// @brief Set distortion levels
///
/// Function sets individual distortion levels for real and synthetic cameras
///
/// @par Example use:
/// @snippet ModelTester.cpp SingleView_Constructor2
/// @snippet ModelTester.cpp SingleView_setCorrectionLvls_v
///
/// @param[in] i_RealDistortionCorrectionLvl_rx Distortion level for real camera
/// @param[in] i_SyntDistortionCorrectionLvl_rx Distortion level for synthetic camera
// --------------------------------------------------------------------------
 template<typename T>
 void SingleView<T>::setCorrectionLvls_v(const T& i_RealDistortionCorrectionLvl_rx, const T& i_SyntDistortionCorrectionLvl_rx)
 {
   this->realCorrectionLvl_x = i_RealDistortionCorrectionLvl_rx;
   this->syntCorrectionLvl_x = i_SyntDistortionCorrectionLvl_rx;
   return;
 }

 // --------------------------------------------------------------------------
 /// @brief Set real camera distortion level
 ///
 /// Function sets distortion levels for real cameras object
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_setRealCorrectionLvl_v
 ///
 /// @param[in] i_DistortionCorrectionLvl_rx Distortion level for real camera
 // --------------------------------------------------------------------------
 template<typename T>
 void SingleView<T>::setRealCorrectionLvl_v(const T& i_DistortionCorrectionLvl_rx)
 {
   this->realCorrectionLvl_x = i_DistortionCorrectionLvl_rx;
   return;
 }

 // --------------------------------------------------------------------------
   /// @brief Set synthetic camera distortion level
   ///
   /// Function sets distortion levels for synthetic cameras object
   ///
   /// @par Example use:
   /// @snippet ModelTester.cpp SingleView_Constructor2
   /// @snippet ModelTester.cpp SingleView_setSyntheticCorrectionLvl_v
   ///
   /// @param[in] i_DistortionCorrectionLvl_rx Distortion level for synthetic camera
   // --------------------------------------------------------------------------
 template<typename T>
 void SingleView<T>::setSyntheticCorrectionLvl_v(const T& i_DistortionCorrectionLvl_rx)
 {
   this->syntCorrectionLvl_x = i_DistortionCorrectionLvl_rx;
   return;
 }

 // --------------------------------------------------------------------------
 /// @brief Get real camera distortion level
 ///
 /// Function returns a copy of the distortion level for the real cameras object
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_copyRealCorrectionLvl_x
 ///
 /// @return Copy of the real camera distortion level
 // --------------------------------------------------------------------------
 template<typename T>
 T SingleView<T>::copyRealCorrectionLvl_x() const
 {
   return this->realCorrectionLvl_x;
 }

 // --------------------------------------------------------------------------
 /// @brief Get synthetic camera distortion level
 ///
 /// Function returns a copy of the distortion level for the synthetic cameras object
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_copySyntheticCorrectionLvl_x
 ///
 /// @return Copy of the synthetic camera distortion level
 // --------------------------------------------------------------------------
 template<typename T>
 T SingleView<T>::copySyntheticCorrectionLvl_x() const
 {
   return this->syntCorrectionLvl_x;
 }

 // --------------------------------------------------------------------------
 /// @brief Get distortion levels for both cameras
 ///
 /// Function returns distortion levels for synthetic and real camera objects
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_copyCorrectionLvls_v
 ///
 /// @return Copy of the synthetic camera distortion level
 // --------------------------------------------------------------------------
 template<typename T>
 void SingleView<T>::copyCorrectionLvls_v(T& o_RealDistortionCorrectionLvl_rx, T& o_SyntDistortionCorrectionLvl_rx)
 {
   o_RealDistortionCorrectionLvl_rx = this->realCorrectionLvl_x;
   o_SyntDistortionCorrectionLvl_rx = this->syntCorrectionLvl_x;
   return;
 }

 // --------------------------------------------------------------------------
 /// @brief Project real image point to synthetic image point
 ///
 /// Function projects a point in pixels on the image plane of the real camera
 /// to the corresponding point on the image plane of the synthetic camera.
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_applyRealToSyntheticProjection_v
 ///
 /// @param[in] i_Pos_rx Point in pixels on the image plane of a real camera
 /// @param[out] o_Pos_rx Point in pixels on the image plane of a synthetic camera
 // --------------------------------------------------------------------------
 template<typename T>
 void SingleView<T>::applyRealToSyntheticProjection_v(const core::Point2D<T>& i_Pos_rx,
                                                            core::Point2D<T>& o_Pos_rx)
 {
   core::Point3D<T> v_TempPos_x;

   this->realCam_px->pixelToMetric_v(i_Pos_rx, o_Pos_rx);

   this->realCam_px->applyUndistortion_v(o_Pos_rx, v_TempPos_x);

   this->applyHomography_v(*this->realCam_px, *this->syntCam_px, v_TempPos_x, v_TempPos_x);

   this->syntCam_px->applyDistortion_v(v_TempPos_x, this->syntCorrectionLvl_x, o_Pos_rx);

   this->syntCam_px->metricToPixel_v(o_Pos_rx, o_Pos_rx);

   return;
 }

 // --------------------------------------------------------------------------
  /// @brief Project real image point to synthetic image point
  ///
  /// Function projects a point in pixels on the image plane of the real camera
  /// to the corresponding point on the image plane of the synthetic camera.
  ///
  /// @param[in] i_Pos_rx Point in pixels on the image plane of a real camera
  /// @param[out] o_Pos_rx Point in pixels on the image plane of a synthetic camera
  // --------------------------------------------------------------------------
  template<typename T>
  void SingleView<T>::applyRealToSyntheticProjection_v(const core::Point2D<T>& i_Pos_rx,
                                                             core::Point2D<T>& o_Pos_rx,
                                                                     bool_t& o_IsApplicable_rb,
                                                                     bool_t& o_IsVisible_rb)
  {
    core::Point3D<T> v_TempPos_x;

    this->realCam_px->pixelToMetric_v(i_Pos_rx, o_Pos_rx);

    this->realCam_px->applyUndistortion_v(o_Pos_rx, v_TempPos_x);

    this->applyHomography_v(*this->realCam_px, *this->syntCam_px, v_TempPos_x, v_TempPos_x);

    this->syntCam_px->applyDistortion_v(v_TempPos_x, this->syntCorrectionLvl_x, o_Pos_rx, o_IsApplicable_rb);

    this->syntCam_px->metricToPixel_v(o_Pos_rx, o_Pos_rx, o_IsVisible_rb);

    return;
  }

 // --------------------------------------------------------------------------
 /// @brief Project synthetic image point to real image point
 ///
 /// Function projects a point in pixels on the image plane of the synthetic camera
 /// to the corresponding point on the image plane of the real camera.
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_applySyntheticToRealProjection_v
 ///
 /// @param[in] i_Pos_rx Point in pixels on the image plane of a synthetic camera
 /// @param[out] o_Pos_rx Point in pixels on the image plane of a real camera
 // --------------------------------------------------------------------------
 template<typename T>
 void SingleView<T>::applySyntheticToRealProjection_v(const core::Point2D<T>& i_Pos_rx,
     core::Point2D<T>& o_Pos_rx)
 {
   core::Point3D<T> v_TempPos_x;

   this->syntCam_px->pixelToMetric_v(i_Pos_rx, o_Pos_rx);

   this->syntCam_px->applyUndistortion_v(o_Pos_rx, v_TempPos_x);

   this->applyHomography_v(*this->syntCam_px, *this->realCam_px, v_TempPos_x, v_TempPos_x);

   this->realCam_px->applyDistortion_v(v_TempPos_x, this->realCorrectionLvl_x, o_Pos_rx);

   this->realCam_px->metricToPixel_v(o_Pos_rx, o_Pos_rx);

   return;
 }

 // --------------------------------------------------------------------------
  /// @brief Project synthetic image point to real image point
  ///
  /// Function projects a point in pixels on the image plane of the synthetic camera
  /// to the corresponding point on the image plane of the real camera.
  ///
  /// @param[in] i_Pos_rx Point in pixels on the image plane of a synthetic camera
  /// @param[out] o_Pos_rx Point in pixels on the image plane of a real camera
  // --------------------------------------------------------------------------
  template<typename T>
  void SingleView<T>::applySyntheticToRealProjection_v(const core::Point2D<T>& i_Pos_rx,
                                                             core::Point2D<T>& o_Pos_rx,
                                                                       bool_t& o_IsApplicable_rb,
                                                                       bool_t& o_IsVisible_rb)
  {
    core::Point3D<T> v_TempPos_x;

    this->syntCam_px->pixelToMetric_v(i_Pos_rx, o_Pos_rx);

    this->syntCam_px->applyUndistortion_v(o_Pos_rx, v_TempPos_x);

    this->applyHomography_v(*this->syntCam_px, *this->realCam_px, v_TempPos_x, v_TempPos_x);

    this->realCam_px->applyDistortion_v(v_TempPos_x, this->realCorrectionLvl_x, o_Pos_rx, o_IsApplicable_rb);

    this->realCam_px->metricToPixel_v(o_Pos_rx, o_Pos_rx, o_IsVisible_rb);

    return;
  }

 // --------------------------------------------------------------------------
 /// @brief Project world metric point onto image plane
 ///
 /// Function projects a world point in metrics onto the image plane of camera
 /// specified in /p i_CamTo_rx.
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_applyProjectionW2I_v
 ///
 /// @param[in] i_CamTo_rx Camera object onto whos image plane to project the input point
 /// @param[in] i_Pos_rx World point in metrics
 /// @param[out] o_Pos_rx Point in pixels on the image plane of the specified camera
 // --------------------------------------------------------------------------
 template<typename T>
 void SingleView<T>::applyProjectionW2I_v(const Camera<T>& i_CamTo_rx,
                            const core::Point4D<T>& i_Pos_rx,
                                  core::Point2D<T>& o_Pos_rx) const
 {
   // singleview is not configured
   AssertFunction(true == this->isConfigured_b(),
             "This SingleView is not configured.");

   if(&i_CamTo_rx == this->realCam_px)
   {
     this->realCam_px->applyFullProjection_v(i_Pos_rx, this->realCorrectionLvl_x, o_Pos_rx);
   }
   else if(&i_CamTo_rx == this->syntCam_px)
   {
     this->syntCam_px->applyFullProjection_v(i_Pos_rx, this->syntCorrectionLvl_x, o_Pos_rx);
   }
   else
   {
     // camera is neither real nor synthetic
     AssertFunction(false, "Camera is neither real nor synthetic.");
   }

   return;
 }

 // --------------------------------------------------------------------------
  /// @brief Project world metric point onto image plane
  ///
  /// Function projects a world point in metrics onto the image plane of camera
  /// specified in /p i_CamTo_rx.
  ///
  /// @param[in] i_CamTo_rx Camera object onto whos image plane to project the input point
  /// @param[in] i_Pos_rx World point in metrics
  /// @param[out] o_Pos_rx Point in pixels on the image plane of the specified camera
  // --------------------------------------------------------------------------
  template<typename T>
  void SingleView<T>::applyProjectionW2I_v(const Camera<T>& i_CamTo_rx,
                             const core::Point4D<T>& i_Pos_rx,
                                   core::Point2D<T>& o_Pos_rx,
                                             bool_t& o_IsApplicable_rb,
                                             bool_t& o_IsVisible_rb) const
  {
    // singleview is not configured
    AssertFunction(true == this->isConfigured_b(),
              "This SingleView is not configured.");

    if(&i_CamTo_rx == this->realCam_px)
    {
      this->realCam_px->applyFullProjection_v(i_Pos_rx,
                                              this->realCorrectionLvl_x,
                                              o_Pos_rx,
                                              o_IsApplicable_rb,
                                              o_IsVisible_rb);
    }
    else if(&i_CamTo_rx == this->syntCam_px)
    {
      this->syntCam_px->applyFullProjection_v(i_Pos_rx,
                                              this->syntCorrectionLvl_x,
                                              o_Pos_rx,
                                              o_IsApplicable_rb,
                                              o_IsVisible_rb);
    }
    else
    {
      // camera is neither real nor synthetic
      AssertFunction(false, "Camera is neither real nor synthetic.");
    }

    return;
  }

 // --------------------------------------------------------------------------
 /// @brief Get pointer to real camera object
 ///
 /// Function returns a pointer to the real camera object
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_getRealCamera_px
 ///
 /// @return Returns pointer to real camera object
 // --------------------------------------------------------------------------
 template<typename T>
 Camera<T>* SingleView<T>::getRealCamera_px()
 {
   return this->realCam_px;
 }

 // --------------------------------------------------------------------------
 /// @brief Get pointer to synthetic camera object
 ///
 /// Function returns a pointer to the synthetic camera object
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_getSyntheticCamera_px
 ///
 /// @return Returns pointer to synthetic camera object
 // --------------------------------------------------------------------------
 template<typename T>
 Camera<T>* SingleView<T>::getSyntheticCamera_px()
 {
   return this->syntCam_px;
 }

 // --------------------------------------------------------------------------
 /// @brief Project world metric point onto real camera image plane
 ///
 /// Function projects a world point in metrics onto the image plane of the real camera
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_applyRealProjectionW2I_v
 ///
 /// @param[in] i_Pos_rx World point in metrics
 /// @param[out] o_Pos_rx Point in pixels on the image plane of the real camera
 // --------------------------------------------------------------------------
 template<typename T>
 void SingleView<T>::applyRealProjectionW2I_v(const core::Point4D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const
 {
   this->applyProjectionW2I_v(const_cast<Camera<T>&>( *(this->realCam_px) ), i_Pos_rx, o_Pos_rx);
   return;
 }

 // --------------------------------------------------------------------------
 /// @brief Project world point onto synthetic camera image plane
 ///
 /// Function projects a world point in metrics onto the image plane of the synthetic camera
 ///
 /// @par Example use:
 /// @snippet ModelTester.cpp SingleView_Constructor2
 /// @snippet ModelTester.cpp SingleView_applySyntheticProjectionW2I_v
 ///
 /// @param[in] i_Pos_rx World point in metrics
 /// @param[out] o_Pos_rx Point in pixels on the image plane of the synthetic camera
 // --------------------------------------------------------------------------
 template<typename T>
 void SingleView<T>::applySyntheticProjectionW2I_v(const core::Point4D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const
 {
   this->applyProjectionW2I_v(const_cast<Camera<T>&>( *(this->syntCam_px) ), i_Pos_rx, o_Pos_rx);
   return;
 }

 // --------------------------------------------------------------------------
/// @brief Apply homography from one camera to another on an image point
///
/// Function applies homography on a point in pixels on the image plane of one camera
/// to another..
///
/// @par Example use:
/// @snippet ModelTester.cpp SingleView_Constructor2
/// @snippet ModelTester.cpp SingleView_applyHomography_v
///
/// @param[in] i_CamFrom_rx From camera
/// @param[in] i_CamTo_rx To camera
/// @param[in] i_Pos_rx Point in pixels on image plane of from camera
/// @param[out] o_Pos_rx Transformed point in pixels on image plane of to camera
// --------------------------------------------------------------------------
template<typename T>
void SingleView<T>::applyHomography_v(Camera<T>& i_CamFrom_rx,
                               Camera<T>& i_CamTo_rx,
                  const core::Point3D<T>& i_Pos_rx,
                        core::Point3D<T>& o_Pos_rx)
{
  if(true == this->isConfigured_b())
  {
    if(    (&i_CamFrom_rx == this->realCam_px)
        && (  &i_CamTo_rx == this->syntCam_px))
    {
      if(false == this->initializedReal2Synt_b)
      {
        calcReal2SyntHomography_v();
      }

      o_Pos_rx = this->real2SyntHomography_x % i_Pos_rx;
    }
    else if(    (&i_CamFrom_rx == this->syntCam_px)
             && (  &i_CamTo_rx == this->realCam_px))
    {
      if(false == this->initializedSynt2Real_b)
      {
        calcSynt2RealHomography_v();
      }

      o_Pos_rx = this->synt2RealHomography_x % i_Pos_rx;
    }
    else
    {
      // at least one of the cameras is not referenced in this view
      AssertFunction(false, "At least one of the cameras is not referenced in this view.");
    }
  }

  return;
}

// --------------------------------------------------------------------------
/// @brief Clears homography initialization flags
///
/// The function clears the real-to-synthetic and synthetic-to-real
/// initialization flags, forcing the SingleView object to recalculate
/// homographies when accessed.
///
/// @par Example usage:
/// @snippet ModelTester.cpp SingleView_Constructor1
/// @snippet ModelTester.cpp SingleView_reset_v
// --------------------------------------------------------------------------
template<typename T>
void SingleView<T>::reset_v(void)
{
  this->initializedSynt2Real_b = false;
  this->initializedReal2Synt_b = false;
}

// --------------------------------------------------------------------------
/// @brief Calculate homography for mapping points form real to synthetic image frame
///
/// If the synthtetic-to-real mapping was previously calculated, then this
/// is just the inverse of it. Otherwise, the homography is computed on behalf
/// of the projection models of both cameras.
///
/// @return void
// --------------------------------------------------------------------------

template<typename T>
void SingleView<T>::calcReal2SyntHomography_v(void)
{
  syntCam_px->getImager_rx().init_v();

  if(false == this->initializedSynt2Real_b)
  {
    this->realCam_px->getImager_rx().init_v();
  }

  if(true == this->initializedSynt2Real_b)
  {
    // compute inverse of already computed homography
    this->real2SyntHomography_x = this->synt2RealHomography_x.inverse();
  }
  else
  {
    // compute homography out of z plane homographies
    this->real2SyntHomography_x = syntCam_px->getImager_rx().getZPlaneHomography_x()
                                   % realCam_px->getImager_rx().getZPlaneHomography_x().inverse();
  }

  this->initializedReal2Synt_b = true;

  return;
}


// --------------------------------------------------------------------------
/// @brief Calculate homography for mapping points form synthetic to real image frame
///
/// If the real-to-synthetic mapping was previously calculated, then this
/// is just the inverse of it. Otherwise, the homography is computed on behalf
/// of the projection models of both cameras.
///
/// @return void
// --------------------------------------------------------------------------

template<typename T>
void SingleView<T>::calcSynt2RealHomography_v(void)
{
  realCam_px->getImager_rx().init_v();

  if(false == this->initializedReal2Synt_b)
  {
    this->syntCam_px->getImager_rx().init_v();
  }

  if(true == this->initializedReal2Synt_b)
  {
    // compute inverse of already computed homography
    this->synt2RealHomography_x = this->real2SyntHomography_x.inverse();
  }
  else
  {
    // compute homography out of z plane homographies
    this->synt2RealHomography_x = realCam_px->getImager_rx().getZPlaneHomography_x()
                                % syntCam_px->getImager_rx().getZPlaneHomography_x().inverse();
  }

  this->initializedSynt2Real_b = true;

  return;
}

}  // namespace model
}  // namespace mecl

#endif

/// @}
/// @}

