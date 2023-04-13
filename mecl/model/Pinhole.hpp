// --------------------------------------------------------------------------
/// @file Pinhole.hpp
/// @brief Contains the pinhole camera model implementation.
///
/// The camera model provides a projection and an inverse projection function.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
///
// --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef MECL_MODEL_PINHOLE_HPP_
#define MECL_MODEL_PINHOLE_HPP_

#include "Pinhole.h"

namespace mecl
{
namespace model
{

// --------------------------------------------------------------------------
/// @brief set standard value for picthOffset_e
// --------------------------------------------------------------------------
template<typename T>
const PitchOffset_e Pinhole<T>::pitchOffset_e = e_PitchOffset180;

// --------------------------------------------------------------------------
/// @brief define default values for extrinsics
// --------------------------------------------------------------------------
template<typename T>
const typename Pinhole<T>::Extrinsic_s Pinhole<T>::extrinsicDefault_s = // {};
{
    // camera position is origin
    {
      math::constants<T>::zero_x(),
      math::constants<T>::zero_x(),
      math::constants<T>::zero_x()
    },

    // Set all Euler angles to zero
    {
      math::constants<T>::zero_x(),
      math::constants<T>::zero_x(),
      math::constants<T>::zero_x()
    },


    // No preroll
    e_PreRoll0,

    // rotation from world to camera
    e_WorldToCamera
};

// --------------------------------------------------------------------------
/// @brief define default values for intrinsics
// --------------------------------------------------------------------------
template<typename T>
const typename Pinhole<T>::Intrinsic_s Pinhole<T>::intrinsicDefault_s =
{
    // x and y scaling is one
    math::constants<T>::one_x(), math::constants<T>::one_x() ,

    // principal point is at origin
    {
        math::constants<T>::zero_x(),
        math::constants<T>::zero_x()
    }
};

// --------------------------------------------------------------------------
/// @brief Default constructor
///
/// @par Example use:
/// @snippet ModelTester.cpp Pinhole_Constructor1
// --------------------------------------------------------------------------
template<typename T>
Pinhole<T>::Pinhole(void)
: extrinsics_s(Pinhole<T>::extrinsicDefault_s)
, intrinsics_s(Pinhole<T>::intrinsicDefault_s)
, projection_x()
, proMatValid_b(false)
{}

// --------------------------------------------------------------------------
/// @brief Constructor copying configuration from existing configuration data set
/// 
/// @par Example use:
/// @snippet ModelTester.cpp Pinhole_ExtrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_IntrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_ConfigConstructor
/// @snippet ModelTester.cpp Pinhole_Constructor2
/// 
/// @param[in] i_Config_rs Intrinsic and extrinsic model configuration
// --------------------------------------------------------------------------
template<typename T>
Pinhole<T>::Pinhole(const Config_s& i_Config_rs)
: extrinsics_s(i_Config_rs.extrinsic_s)
, intrinsics_s(i_Config_rs.intrinsic_s)
, projection_x()
, proMatValid_b(false)
{}



// --------------------------------------------------------------------------
/// @brief Get intrinsic calibration matrix K
/// 
/// Return copy of intrinsic calibration matrix K
/// 
/// @par Example use:
/// @snippet ModelTester.cpp Pinhole_getKMatrix_x
/// 
/// @return Calibration matrix K
// --------------------------------------------------------------------------
template<typename T>
core::Matrix3x3<T> Pinhole<T>::getKMatrix_x(void) const
{
  core::Matrix3x3<T> v_K_x = core::Matrix3x3<T>::eye_x();

  core::Point2D<T> v_Ppp_x(this->intrinsics_s.pppCfg_x);

  v_K_x(0,0) = this->intrinsics_s.foclX_x;
  v_K_x(1,1) = this->intrinsics_s.foclY_x;
  v_K_x(0,2) = v_Ppp_x.getPosX();
  v_K_x(1,2) = v_Ppp_x.getPosY();

  return v_K_x;
}

// --------------------------------------------------------------------------
/// @brief Get reference to projection matrix
/// 
/// Return pointer reference to projection matrix
/// 
/// @par Example use:
/// @snippet ModelTester.cpp Pinhole_getProjectionMatrix_rx
/// 
/// @return Reference to projection matrix
// --------------------------------------------------------------------------
template<typename T>
core::Matrix<T, 3, 4>& Pinhole<T>::getProjectionMatrix_rx(void)
{
  return this->projection_x.getProjectionMatrix_rx();
}


// --------------------------------------------------------------------------
/// @brief Get copy of projection matrix
/// 
/// Return copy of projection matrix
/// 
/// @par Example use:
/// @snippet ModelTester.cpp Pinhole_copyProjectionMatrix_x
/// 
/// @return Copy of projection matrix
// --------------------------------------------------------------------------
template<typename T>
core::Matrix<T, 3, 4> Pinhole<T>::copyProjectionMatrix_x(void) const
{
  return this->projection_x.copyProjectionMatrix_x();
}

// --------------------------------------------------------------------------
/// @brief Get copy of z=0 plane homography of the projection
/// 
/// Get planar homography of image points (located on z=0 plane in camera 
/// coordinate frame) onto z=0 plane in world coordinate frame
/// 
/// @par Example use:
/// @snippet ModelTester.cpp Pinhole_getZPlaneHomography_x
/// 
/// @return Copy of z=0 plane homography of the projection
/// (projective 2D linear transformation matrix)
// --------------------------------------------------------------------------
template<typename T>
const core::Matrix3x3<T> Pinhole<T>::getZPlaneHomography_x(void) const
{
  return this->projection_x.getZPlaneHomography_x();
}

// --------------------------------------------------------------------------
/// @brief Get copy of orthonormal basis
/// 
/// Get a copy of an orthonormal basis of the projection as 3x3 Matrix
/// 
/// @par Example use:
/// @snippet ModelTester.cpp Pinhole_getOrthonormalBase_x
/// 
/// @return Copy of orthonormal basis
// --------------------------------------------------------------------------
template<typename T>
const core::Matrix3x3<T> Pinhole<T>::getOrthonormalBase_x(void) const
{
  return this->projection_x.getOrthonormalBase_x();
}

// -------------------------------------------------------------------------
/// @brief Get configuration status
///
/// Return true if pinhole was properly configured
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_isConfigured_b
///
/// @return Boolean, True = configured, false = not configured
// -------------------------------------------------------------------------
template<typename T>
bool_t Pinhole<T>::isConfigured_b(void) const
{
  return this->projection_x.isConfigured_b();
}

// -------------------------------------------------------------------------
/// @brief Update configuration
///
/// The function sets intrinsic, extrinsic and projection model parameters from 
/// \p i_Config and marks the projection matrix object as not configured.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_ExtrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_IntrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_ConfigConstructor
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_updateConfig_v
///
/// @param[in] i_Config_rs Configuration to be applied to this Pinhole
/// @return void
// -------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::updateConfig_v(const Config_s& i_Config_rs)
{
  this->setExtrinsic_v(i_Config_rs.extrinsic_s);
  this->setIntrinsic_v(i_Config_rs.intrinsic_s);

  this->proMatValid_b = false;

  return;
}

// -------------------------------------------------------------------------
/// @brief Update configuration by POD struct of different base type T1
///
/// The function sets intrinsic, extrinsic and projection model parameters from
/// \p i_Config_rs of different base type T1 and marks the projection matrix object
/// as not configured.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_ExtrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_IntrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_ConfigConstructor
/// @snippet ModelTester.cpp Pinhole_Constructor_double
/// @snippet ModelTester.cpp Pinhole_updateConfig_v_otherBaseType
///
/// @param[in] i_Config_rs Configuration to be applied to this Pinhole
/// @return void
// -------------------------------------------------------------------------
template<typename T>
template<typename T1>
void Pinhole<T>::updateConfig_v(const typename Pinhole<T1>::Config_s& i_Config_rs)
{
  this->setExtrinsic_v<T1>(i_Config_rs.extrinsic_s);
  this->setIntrinsic_v<T1>(i_Config_rs.intrinsic_s);

  this->proMatValid_b = false;

  return;
}

// --------------------------------------------------------------------------
/// @brief Copy pinhole configuration to POD object
///
/// The function copies all elements from configuration object of Pinhole to struct \p o_Config_rs.
///
/// @par Example usage:
/// @snippet Modeltester.cpp Camera_PinholeConstructor
/// @snippet ModelTester.cpp Pinhole_Constructor2
/// @snippet ModelTester.cpp Pinhole_copyConfig_v
///
/// @param[in] o_Config_rs Configuration to be overwritten with contents from pinhole configuration
/// @return void
// --------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::copyConfig_v(Config_s& o_Config_rs) const
{
  this->copyExtrinsic_v(o_Config_rs.extrinsic_s);
  this->copyIntrinsic_v(o_Config_rs.intrinsic_s);
  return;
}

// --------------------------------------------------------------------------
/// @brief Copy pinhole configuration to POD object of (different) base type T1
///
/// The function copies all elements from configuration object of Pinhole to struct \p o_Config_rs
/// of (different) base type T1
///
/// @par Example usage:
/// @snippet Modeltester.cpp Camera_PinholeConstructor
/// @snippet ModelTester.cpp Pinhole_Constructor2
/// @snippet ModelTester.cpp Pinhole_copyConfig_otherBaseType
///
/// @param[in] o_Config_rs Configuration to be overwritten with contents from pinhole configuration
/// @return void
// --------------------------------------------------------------------------

template<typename T>
template<typename T1>
void Pinhole<T>::copyConfig_v(typename Pinhole<T1>::Config_s& o_Config_rs) const
{
  this->copyExtrinsic_v<T1>(o_Config_rs.extrinsic_s);
  this->copyIntrinsic_v<T1>(o_Config_rs.intrinsic_s);
  return;
}

// --------------------------------------------------------------------------
/// @brief Copy pinhole configuration to volatilePOD object
///
/// The function copies all elements from configuration object of Pinhole to volatilestruct \p o_Config_rs.
///
/// @par Example usage:
/// @snippet Modeltester.cpp Camera_PinholeConstructor
/// @snippet ModelTester.cpp Pinhole_Constructor2
/// @snippet ModelTester.cpp Pinhole_copyConfigVolatile_v
///
/// @param[in] o_Config_rs Configuration in volatile memory to be overwritten with contents from pinhole configuration
/// @return void
// --------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::copyConfigVolatile_v(volatile Config_s& o_Config_rs) const
{
  this->copyExtrinsicVolatile_v(o_Config_rs.extrinsic_s);
  this->copyIntrinsicVolatile_v(o_Config_rs.intrinsic_s);
  return;
}



// -------------------------------------------------------------------------
/// @brief Update extrinsic model parameters
///
/// The function updates extrinsic model parameters from \p i_Config and marks 
/// the projection matrix as not configured.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_ExtrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_IntrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_ConfigConstructor
/// @snippet ModelTester.cpp Pinhole_updateExtrinsic_v
///
/// @param[in] i_Config_rs Configuration from which to extract extrinsic
/// model parameters
/// @return void
// -------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::updateExtrinsic_v(const Config_s& i_Config_rs)
{
  this->setExtrinsic_v(i_Config_rs.extrinsic_s);

  this->proMatValid_b = false;

  return;
}

// -------------------------------------------------------------------------
/// @brief Update extrinsic model parameters by different POD struct of (different) base type T1
///
/// The function updates extrinsic model parameters from \p i_Config_rs of (different)
/// base type T1 and marks the projection matrix as not configured.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor_double
/// @snippet ModelTester.cpp Pinhole_ExtrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_IntrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_ConfigConstructor
/// @snippet ModelTester.cpp Pinhole_updateExtrinsic_v_otherBaseType
///
/// @param[in] i_Config_rs Configuration from which to extract extrinsic
/// model parameters
/// @return void
// -------------------------------------------------------------------------
template<typename T>
template<typename T1>
void Pinhole<T>::updateExtrinsic_v(const typename Pinhole<T1>::Config_s& i_Config_rs)
{
  this->setExtrinsic_v<T1>(i_Config_rs.extrinsic_s);

  this->proMatValid_b = false;

  return;
}


// --------------------------------------------------------------------------
/// @brief Set intrinsic parameters for pinhole in configuration data
///
/// The function sets intrinsic model parameters from \p i_Intrinsic_rs and marks 
/// the projection matrix as not configured.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_IntrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_setIntrinsic_v
///
/// @param[in] i_Intrinsic_rs Intrinsic parameters
/// @return void
// --------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::setIntrinsic_v(const Intrinsic_s& i_Intrinsic_rs)
{
  this->intrinsics_s.foclX_x = i_Intrinsic_rs.foclX_x;
  this->intrinsics_s.foclY_x = i_Intrinsic_rs.foclY_x;
  this->intrinsics_s.pppCfg_x = i_Intrinsic_rs.pppCfg_x;

  this->proMatValid_b = false;

  return;
}

// --------------------------------------------------------------------------
/// @brief Set intrinsic parameters by configuration POD struct of different base type T1
///
/// The function sets intrinsic model parameters from \p i_Intrinsic_rs of (different)
/// base type T1 and marks the projection matrix as not configured.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_IntrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_Constructor_double
/// @snippet ModelTester.cpp Pinhole_setIntrinsic_v_otherBaseType
///
/// @param[in] i_Intrinsic_rs Intrinsic parameters
/// @return void
// --------------------------------------------------------------------------
template<typename T>
template<typename T1>
void Pinhole<T>::setIntrinsic_v(const typename Pinhole<T1>::Intrinsic_s& i_Intrinsic_rs)
{
  this->intrinsics_s.foclX_x = static_cast<T>(i_Intrinsic_rs.foclX_x);
  this->intrinsics_s.foclY_x = static_cast<T>(i_Intrinsic_rs.foclY_x);
  core::Point2D<T1>(i_Intrinsic_rs.pppCfg_x).copyConfig_v<T>(this->intrinsics_s.pppCfg_x);

  this->proMatValid_b = false;

  return;
}

// --------------------------------------------------------------------------
/// @brief Set extrinsic parameters for pinhole in configuration data
///
/// The function sets extrinsic model parameters from \p i_Extrinsic_rs and marks 
/// the projection matrix as not configured.
/// 
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_ExtrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_setExtrinsic_v
///
/// @param[in] i_Extrinsic_rs Extrinsic model parameters
/// @return void
// --------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::setExtrinsic_v(const Extrinsic_s& i_Extrinsic_rs)
{
  this->extrinsics_s.pos_x                 = i_Extrinsic_rs.pos_x;
  this->extrinsics_s.eulerAngles_s.pitch_x = i_Extrinsic_rs.eulerAngles_s.pitch_x;
  this->extrinsics_s.eulerAngles_s.yaw_x   = i_Extrinsic_rs.eulerAngles_s.yaw_x;
  this->extrinsics_s.eulerAngles_s.roll_x  = i_Extrinsic_rs.eulerAngles_s.roll_x;
  this->extrinsics_s.preRoll_e             = i_Extrinsic_rs.preRoll_e;
  this->extrinsics_s.rotationType_e        = i_Extrinsic_rs.rotationType_e;

  this->proMatValid_b = false;

  return;
}

// --------------------------------------------------------------------------
/// @brief Set extrinsic parameters by configuration POD struct of different base type T1
///
/// The function sets extrinsic model parameters from \p i_Extrinsic_rs of
/// (different) base type T1 and marks the projection matrix as not configured.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_ExtrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_Constructor_double
/// @snippet ModelTester.cpp Pinhole_setExtrinsic_v_otherBaseType
///
/// @param[in] i_Extrinsic_rs Extrinsic model parameters
/// @return void
// --------------------------------------------------------------------------
template<typename T>
template<typename T1>
void Pinhole<T>::setExtrinsic_v(const typename Pinhole<T1>::Extrinsic_s& i_Extrinsic_rs)
{
  core::Point3D<T1>(i_Extrinsic_rs.pos_x).copyConfig_v<T>(this->extrinsics_s.pos_x);
  this->extrinsics_s.eulerAngles_s.pitch_x = i_Extrinsic_rs.eulerAngles_s.pitch_x;
  this->extrinsics_s.eulerAngles_s.yaw_x   = i_Extrinsic_rs.eulerAngles_s.yaw_x;
  this->extrinsics_s.eulerAngles_s.roll_x  = i_Extrinsic_rs.eulerAngles_s.roll_x;
  this->extrinsics_s.preRoll_e             = i_Extrinsic_rs.preRoll_e;
  this->extrinsics_s.rotationType_e        = i_Extrinsic_rs.rotationType_e;

  this->proMatValid_b = false;

  return;
}

// --------------------------------------------------------------------------
/// @brief Get pointer reference to intrinsic parameters for pinhole in 
/// configuration data
///
/// Returns a pointer reference to the internal intrinsic model parameter 
/// structure
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_getIntrinsic_rs
///
/// @return Reference to intrinsic model parameters
// --------------------------------------------------------------------------
template<typename T>
const typename Pinhole<T>::Intrinsic_s& Pinhole<T>::getIntrinsic_rs(void)
{
  return this->intrinsics_s;
}

// --------------------------------------------------------------------------
/// @brief Get pointer reference to extrinsic parameters for pinhole in 
/// configuration data
///
/// Returns a pointer reference to the internal extrinsic model parameter 
/// structure
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_getExtrinsic_rs
///
/// @return Reference to extrinsic model parameters
// --------------------------------------------------------------------------
template<typename T>
const typename Pinhole<T>::Extrinsic_s& Pinhole<T>::getExtrinsic_rs(void) const
{
  return this->extrinsics_s;
}

// --------------------------------------------------------------------------
/// @brief Get a copy of extrinsic parameters for this pinhole in configuration data
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_copyExtrinsic_v
///
/// @param[out] o_Extrinsic_rs Reference to the copy of extrinsic parameter configuration
// --------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::copyExtrinsic_v(Extrinsic_s& o_Extrinsic_rs) const
{
  o_Extrinsic_rs.pos_x                 = this->extrinsics_s.pos_x;
  o_Extrinsic_rs.eulerAngles_s.pitch_x = this->extrinsics_s.eulerAngles_s.pitch_x;
  o_Extrinsic_rs.eulerAngles_s.yaw_x   = this->extrinsics_s.eulerAngles_s.yaw_x;
  o_Extrinsic_rs.eulerAngles_s.roll_x  = this->extrinsics_s.eulerAngles_s.roll_x;
  o_Extrinsic_rs.preRoll_e             = this->extrinsics_s.preRoll_e;
  o_Extrinsic_rs.rotationType_e        = this->extrinsics_s.rotationType_e;

  return;
}
// --------------------------------------------------------------------------
/// @brief Get a copy of extrinsic parameters for this pinhole in of (different) base type T1
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_copyExtrinsic_v_otherBaseType
///
/// @param[out] o_Extrinsic_rs Reference to the copy of extrinsic parameter configuration
// --------------------------------------------------------------------------
template<typename T>
template<typename T1>
void Pinhole<T>::copyExtrinsic_v(typename Pinhole<T1>::Extrinsic_s& o_Extrinsic_rs) const
{
  core::Point3D<T>(this->extrinsics_s.pos_x).copyConfig_v<T1>(o_Extrinsic_rs.pos_x);
  o_Extrinsic_rs.eulerAngles_s.pitch_x = static_cast<T1>(this->extrinsics_s.eulerAngles_s.pitch_x);
  o_Extrinsic_rs.eulerAngles_s.yaw_x   = static_cast<T1>(this->extrinsics_s.eulerAngles_s.yaw_x);
  o_Extrinsic_rs.eulerAngles_s.roll_x  = static_cast<T1>(this->extrinsics_s.eulerAngles_s.roll_x);
  o_Extrinsic_rs.preRoll_e             = this->extrinsics_s.preRoll_e;
  o_Extrinsic_rs.rotationType_e        = this->extrinsics_s.rotationType_e;
}

// --------------------------------------------------------------------------
/// @brief Get a volatile copy of extrinsic parameters for this pinhole in configuration data
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_copyExtrinsicVolatile_v
///
/// @param[out] o_Extrinsic_rs Reference to the volatile copy of extrinsic parameter configuration
// --------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::copyExtrinsicVolatile_v(volatile Extrinsic_s& o_Extrinsic_rs) const
{
  const core::Point3D<T> v_Pos_x(this->extrinsics_s.pos_x);
  v_Pos_x.copyConfigVolatile_v(o_Extrinsic_rs.pos_x);
  o_Extrinsic_rs.eulerAngles_s.pitch_x = this->extrinsics_s.eulerAngles_s.pitch_x;
  o_Extrinsic_rs.eulerAngles_s.yaw_x   = this->extrinsics_s.eulerAngles_s.yaw_x;
  o_Extrinsic_rs.eulerAngles_s.roll_x  = this->extrinsics_s.eulerAngles_s.roll_x;
  o_Extrinsic_rs.preRoll_e             = this->extrinsics_s.preRoll_e;
  o_Extrinsic_rs.rotationType_e        = this->extrinsics_s.rotationType_e;

  return;
}


// --------------------------------------------------------------------------
/// @brief Get a copy of intrinsic parameters for this pinhole in configuration data
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_copyIntrinsic_v
///
/// @param[out] o_Intrinsic_rs Reference to the copy of intrinsic parameter configuration
// --------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::copyIntrinsic_v(Intrinsic_s& o_Intrinsic_rs) const
{
  o_Intrinsic_rs.foclX_x = this->intrinsics_s.foclX_x;
  o_Intrinsic_rs.foclY_x = this->intrinsics_s.foclY_x;
  const core::Point2D<T> v_Ppp_x(this->intrinsics_s.pppCfg_x);
  v_Ppp_x.copyConfig_v(o_Intrinsic_rs.pppCfg_x);
  return;
}

// --------------------------------------------------------------------------
/// @brief Get a copy of intrinsic parameters for this pinhole of different base type T1
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_copyExtrinsic_v_otherBaseType
///
/// @param[out] o_Intrinsic_rs Reference to the copy of intrinsic parameter configuration
// --------------------------------------------------------------------------
template<typename T>
template<typename T1>
void Pinhole<T>::copyIntrinsic_v(typename Pinhole<T1>::Intrinsic_s& o_Intrinsic_rs) const
{
  o_Intrinsic_rs.foclX_x = static_cast<T1>(this->intrinsics_s.foclX_x);
  o_Intrinsic_rs.foclY_x = static_cast<T1>(this->intrinsics_s.foclY_x);
  core::Point2D<T>(this->intrinsics_s.pppCfg_x).copyConfig_v<T1>(o_Intrinsic_rs.pppCfg_x);
  return;
}

// --------------------------------------------------------------------------
/// @brief Get a volatile copy of intrinsic parameters for this pinhole in configuration data
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_copyIntrinsicVolatile_v
///
/// @param[out] o_Intrinsic_rs Reference to the volatile copy of intrinsic parameter configuration
// --------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::copyIntrinsicVolatile_v(volatile Intrinsic_s& o_Intrinsic_rs) const
{
  o_Intrinsic_rs.foclX_x = this->intrinsics_s.foclX_x;
  o_Intrinsic_rs.foclY_x = this->intrinsics_s.foclY_x;
  const core::Point2D<T> v_Ppp_x(this->intrinsics_s.pppCfg_x);
  v_Ppp_x.copyConfigVolatile_v(o_Intrinsic_rs.pppCfg_x);
  return;
}

// --------------------------------------------------------------------------
/// @brief Check if calculation of Projection Matrix is necessary and do if so
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_ExtrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_IntrinsicConstructor
/// @snippet ModelTester.cpp Pinhole_Constructor2
/// @snippet ModelTester.cpp Pinhole_init_v
///
/// @return void
// ---------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::init_v(void)
{
  if(false == this->proMatValid_b)
  {
    this->calc_v(this->getProjectionMatrix_rx());
    this->proMatValid_b = true;
  }

  return;
}

// --------------------------------------------------------------------------
/// @brief Set the pre-roll angle (depends on mounting position)
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_setPreRoll_v
///
/// @param[in] i_PreRoll_re pre roll angle (definition see MeclTypes.h)
/// @return void
// --------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::setPreRoll_v(const mecl::model::PreRoll_e& i_PreRoll_re)
{
  this->extrinsics_s.preRoll_e = i_PreRoll_re;
  return;
};

// --------------------------------------------------------------------------
/// @brief Calculates projection matrix of pinhole model
/// 
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_calc_v
///
/// @param[out] o_Projection_rx: Reference to calculated projection matrix
/// @return void
// --------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::calc_v(mecl::core::Matrix<T, 3, 4>& o_Projection_rx) const
{
  core::RotationMatrix<T> v_R_x = this->getRotationMatrix_x();
  core::Point3D<T> v_T_x = this->getTranslationVector_x(v_R_x);
  core::Matrix<T, 3, 3> v_K_x = this->getKMatrix_x();

  if (not core::Matrix<T, 3, 3>::eye_x().equals(v_K_x)) {
    o_Projection_rx = v_K_x.mmulFast(v_R_x.joinCol(v_T_x));
  } else {
    o_Projection_rx = v_R_x.joinCol(v_T_x);
  }
  return;
}


// ------------------------------------------------------------------------------
/// @brief Get copy of the translation vector  
/// 
/// Function calculates and returns a copy of the translation vector as given by 
/// the rotation matrix and extrinsic model parameters.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_getTranslationVector_x_1
///
/// @return Copy of translation vector
// ------------------------------------------------------------------------------
template<typename T>
core::Point3D<T> Pinhole<T>::getTranslationVector_x(void) const
{
  return this->getTranslationVector_x(this->getRotationMatrix_x());
}

// ------------------------------------------------------------------------------
/// @brief Get copy of the translation vector
/// 
/// Function calculates and returns a copy of the translation vector as given by 
/// the rotation matrix and extrinsic model parameters.
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_RotationMatrixConstructor
/// @snippet ModelTester.cpp Pinhole_getTranslationVector_x_2
///
/// @param[in] i_RotMat_rx Rotation matrix
/// @return Copy of translation vector
// ------------------------------------------------------------------------------
template<typename T>
core::Point3D<T> Pinhole<T>::getTranslationVector_x(const core::RotationMatrix<T>& i_RotMat_rx) const
{
  return core::Point3D<T>( (i_RotMat_rx.mmulFast(core::Point3D<T>(this->extrinsics_s.pos_x) ) ).minus() );
}

// ------------------------------------------------------------------------------
/// @brief Get camera center of this pinhole camera model
///
/// Function calculates camera position wrt. to world coordinate system
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_getCameraCenter_x_0
///
/// @return Camera position in world coordinates (euclidean 3D)
// ------------------------------------------------------------------------------
template<typename T>
core::Point3D<T> Pinhole<T>::getCameraCenter_x(void) const
{
  const core::RotationMatrix<T> c_Rot_x = this->getRotationMatrix_x();
  return getCameraCenter_x(c_Rot_x, this->getTranslationVector_x(c_Rot_x));
}

// ------------------------------------------------------------------------------
/// @brief Get camera center of this pinhole camera model with given translation vector
///
/// Function calculates camera position wrt. to world coordinate system on behalf of given translation
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_getCameraCenter_x_1
///
/// @param[in] i_T_rx Reference to given translation vector
/// @return Camera position in world coordinates (euclidean 3D)
// ------------------------------------------------------------------------------
template<typename T>
core::Point3D<T> Pinhole<T>::getCameraCenter_x(const core::Point3D<T>& i_T_rx) const
{
  return getCameraCenter_x( this->getRotationMatrix_x(), i_T_rx);
}

// ------------------------------------------------------------------------------
/// @brief Get camera center of this pinhole camera model with given rotation matrix and translation vector
///
/// Function calculates camera position wrt. to world coordinate system on behalf of given translation and rotation
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_getCameraCenter_x_2
///
/// @param[in] i_RotMat_rx Reference to given rotation matrix  translation vector
/// @param[in] i_T_rx Reference to given translation vector
/// @return Camera position in world coordinates (euclidean 3D)
// ------------------------------------------------------------------------------
template<typename T>
core::Point3D<T> Pinhole<T>::getCameraCenter_x(const core::RotationMatrix<T>& i_RotMat_rx,
                                               const core::Point3D<T>& i_T_rx) const
{
  return core::Point3D<T>( (- i_RotMat_rx.inverse()) % i_T_rx );
}
// ------------------------------------------------------------------------------
/// @brief Get Euler angles of rotation matrix of this pinhole camera model according to extrinsic setup
///
/// Function calculates Eulerangles wrt. to the camera's extrinsic configuration parameters 
/// that are: Preroll Factor, Rotation Type
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_getEulerAngles_s_1
/// 
/// @param[in] i_AngleUnit_e Defines whether angles are in radians (e_Radians) or 
/// degrees (e_Degrees), optional, default value is e_Radians
/// @return Euler angles of rotation matrix of this pinhole wrt. extrinsic setup
// ------------------------------------------------------------------------------

template<typename T>
typename core::RotationMatrix<T>::EulerAngles_s Pinhole<T>::getEulerAngles_s(AngleUnit_e i_AngleUnit_e) const
{
  return getEulerAngles_s( this->getRotationMatrix_x(), i_AngleUnit_e);
}

// ------------------------------------------------------------------------------
/// @brief Get Euler angles of a rotation matrix according to extrinsic setup
///
/// Function calculates Eulerangles wrt. this pinhole camera model's extrinsic configuration parameters
/// that are: Preroll Factor, Rotation Type
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_RotationMatrixConstructor
/// @snippet ModelTester.cpp Pinhole_getEulerAngles_s_2
///
/// @param[in] i_RotMat_rx Rotation matrix given
/// @param[in] i_AngleUnit_e Defines whether angles are in radians (e_Radians) or 
/// degrees (e_Degrees), optional, default value is e_Radians
/// @return Euler angles of given rotation matrix wrt. extrinsic setup
// ------------------------------------------------------------------------------
template<typename T>
typename core::RotationMatrix<T>::EulerAngles_s Pinhole<T>::getEulerAngles_s(const core::RotationMatrix<T>& i_RotMat_rx,
                                                                             AngleUnit_e i_AngleUnit_e) const
{
  typename core::RotationMatrix<T>::EulerAngles_s v_EulerAngles_s = { 0, 0, 0} ;

  switch(this->extrinsics_s.preRoll_e)
  {
    case e_PreRoll0:
    {
      const typename core::RotationMatrix<T>::EulerAngles_s c_EAWorld_s = i_RotMat_rx.calcEA_s();

      switch (this->extrinsics_s.rotationType_e)
      {
        case e_CameraToWorld:
          v_EulerAngles_s.pitch_x =   c_EAWorld_s.pitch_x + math::toRadians_x<T>(static_cast<T>(pitchOffset_e));
          v_EulerAngles_s.yaw_x   =   c_EAWorld_s.yaw_x;
          v_EulerAngles_s.roll_x  =   c_EAWorld_s.roll_x;
          break;
        case e_WorldToCamera: // fall through
        default:
          v_EulerAngles_s.pitch_x =   math::toRadians_x<T>(static_cast<T>(pitchOffset_e)) - c_EAWorld_s.pitch_x;
          v_EulerAngles_s.yaw_x   =   - c_EAWorld_s.yaw_x;
          v_EulerAngles_s.roll_x  =   - c_EAWorld_s.roll_x;
          break;
      }
      break;
    }

    case e_PreRoll90:
    {
      const typename core::RotationMatrix<T>::EulerAngles_s c_EAWorld_s =
          static_cast< core::RotationMatrix<T> >(i_RotMat_rx % core::RotationMatrix<T>::preRoll270() ).calcEA_s();

      switch (this->extrinsics_s.rotationType_e)
      {
        case e_CameraToWorld:
          v_EulerAngles_s.pitch_x =   math::toRadians_x<T>(static_cast<T>(pitchOffset_e)) - c_EAWorld_s.pitch_x;
          v_EulerAngles_s.yaw_x   = - c_EAWorld_s.yaw_x;
          v_EulerAngles_s.roll_x  =   c_EAWorld_s.roll_x + math::constants<T>::pi_x();
          break;
        case e_WorldToCamera: // fall through
        default:
          v_EulerAngles_s.pitch_x =   c_EAWorld_s.pitch_x + math::toRadians_x<T>(static_cast<T>(pitchOffset_e));
          v_EulerAngles_s.yaw_x   =   c_EAWorld_s.yaw_x;
          v_EulerAngles_s.roll_x  = - c_EAWorld_s.roll_x + math::constants<T>::pi_x();
          break;
      }
      break;
    }

    case e_PreRoll180:
    {
      const typename core::RotationMatrix<T>::EulerAngles_s c_EAWorld_s = i_RotMat_rx.calcEA_s();

      switch (this->extrinsics_s.rotationType_e)
      {
        case e_CameraToWorld:
          v_EulerAngles_s.pitch_x =   math::toRadians_x<T>(static_cast<T>(pitchOffset_e)) - c_EAWorld_s.pitch_x;
          v_EulerAngles_s.yaw_x   = - c_EAWorld_s.yaw_x;
          v_EulerAngles_s.roll_x  =   c_EAWorld_s.roll_x + math::constants<T>::pi_x();
          break;
        case e_WorldToCamera: // fall through
        default:
          v_EulerAngles_s.pitch_x =   c_EAWorld_s.pitch_x + math::toRadians_x<T>(static_cast<T>(pitchOffset_e));
          v_EulerAngles_s.yaw_x   =   c_EAWorld_s.yaw_x;
          v_EulerAngles_s.roll_x  = - c_EAWorld_s.roll_x + math::constants<T>::pi_x();
          break;
      }

      break;
    }

    case e_PreRoll270:
    {
      const typename core::RotationMatrix<T>::EulerAngles_s c_EAWorld_s =
          static_cast< core::RotationMatrix<T> >(i_RotMat_rx % core::RotationMatrix<T>::preRoll90() ).calcEA_s();

      switch (this->extrinsics_s.rotationType_e)
      {
        case e_CameraToWorld:
          v_EulerAngles_s.pitch_x =   math::toRadians_x<T>(static_cast<T>(pitchOffset_e)) - c_EAWorld_s.pitch_x;
          v_EulerAngles_s.yaw_x   =   - c_EAWorld_s.yaw_x;
          v_EulerAngles_s.roll_x  =     c_EAWorld_s.roll_x + math::constants<T>::pi_x();
          break;
        case e_WorldToCamera: // fall through
        default:
          v_EulerAngles_s.pitch_x =   c_EAWorld_s.pitch_x + math::toRadians_x<T>(static_cast<T>(pitchOffset_e));
          v_EulerAngles_s.yaw_x   =   c_EAWorld_s.yaw_x;
          v_EulerAngles_s.roll_x  = - c_EAWorld_s.roll_x + math::constants<T>::pi_x();;
          break;
      }
      break;
    }
  } // switch

  if (e_Degrees == i_AngleUnit_e) {
    v_EulerAngles_s.pitch_x = math::toDegrees_x(v_EulerAngles_s.pitch_x);
    v_EulerAngles_s.yaw_x   = math::toDegrees_x(v_EulerAngles_s.yaw_x);
    v_EulerAngles_s.roll_x  = math::toDegrees_x(v_EulerAngles_s.roll_x);
  }

  return v_EulerAngles_s;
}

// --------------------------------------------------------------------------
/// @brief Calculate and return copy of rotation matrix 
/// 
/// Function calculates rotation matrix based on Euler angles given in 
/// configuration data and initialize projection matrix with results.
/// 
/// @attention Euler angles define camera orientation as see from world coordinate 
/// system. To project points from world coordinates into camera coordinates all angles
/// have to be inverted prior to building the rotation matrix.
/// 
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_getRotationMatrix_x
/// 
/// @return Copy of rotation matrix
// --------------------------------------------------------------------------
template<typename T>
core::RotationMatrix<T> Pinhole<T>::getRotationMatrix_x(void) const
{
  typename core::RotationMatrix<T>::EulerAngles_s v_EulerAngles_s = { 0,0,0 };

  bool_t v_ComputePreRoll_b = false;

  const T c_PitchWithOffset_x =
      e_WorldToCamera  == this->extrinsics_s.rotationType_e
      ?   math::toRadians_x<T>(static_cast<T>(pitchOffset_e)) - this->extrinsics_s.eulerAngles_s.pitch_x
      :   math::toRadians_x<T>(static_cast<T>(pitchOffset_e)) + this->extrinsics_s.eulerAngles_s.pitch_x;

  // See Doxygen comments above for explanation of euler angle inversion in case rotationType_e == e_CameraToWorld.
  switch(this->extrinsics_s.preRoll_e)
  {
    case e_PreRoll0: // right
    {
      v_EulerAngles_s.pitch_x =   c_PitchWithOffset_x;

      switch (this->extrinsics_s.rotationType_e)
      {
        case e_CameraToWorld:
          v_EulerAngles_s.yaw_x   =   this->extrinsics_s.eulerAngles_s.yaw_x;
          v_EulerAngles_s.roll_x  =   this->extrinsics_s.eulerAngles_s.roll_x;
          break;
        case e_WorldToCamera: // fall through
        default:
          v_EulerAngles_s.yaw_x   =   - this->extrinsics_s.eulerAngles_s.yaw_x;
          v_EulerAngles_s.roll_x  =   - this->extrinsics_s.eulerAngles_s.roll_x;
          break;
      }

      break;
    }

    case e_PreRoll90: // front
    {
      v_EulerAngles_s.pitch_x = - c_PitchWithOffset_x;
      switch (this->extrinsics_s.rotationType_e)
      {
        case e_CameraToWorld:
          v_EulerAngles_s.yaw_x   = - this->extrinsics_s.eulerAngles_s.yaw_x;
          v_EulerAngles_s.roll_x  =   this->extrinsics_s.eulerAngles_s.roll_x + math::constants<T>::pi_x();
          break;
        case e_WorldToCamera: // fall through
        default:
          v_EulerAngles_s.yaw_x   =   this->extrinsics_s.eulerAngles_s.yaw_x;
          v_EulerAngles_s.roll_x  = - this->extrinsics_s.eulerAngles_s.roll_x + math::constants<T>::pi_x();
          break;
      }
      v_ComputePreRoll_b = true;
      break;
    }

    case e_PreRoll180: // left
    {
      v_EulerAngles_s.pitch_x = - c_PitchWithOffset_x;
      switch (this->extrinsics_s.rotationType_e)
      {
        case e_CameraToWorld:
          v_EulerAngles_s.yaw_x   = - this->extrinsics_s.eulerAngles_s.yaw_x;
          v_EulerAngles_s.roll_x  =   this->extrinsics_s.eulerAngles_s.roll_x + math::constants<T>::pi_x();
          break;
        case e_WorldToCamera: // fall through
        default:
          v_EulerAngles_s.yaw_x   =   this->extrinsics_s.eulerAngles_s.yaw_x;
          v_EulerAngles_s.roll_x  = - this->extrinsics_s.eulerAngles_s.roll_x + math::constants<T>::pi_x();
          break;
      }

      break;
    }

    case e_PreRoll270: // rear
    {
      v_EulerAngles_s.pitch_x =   c_PitchWithOffset_x;
      switch (this->extrinsics_s.rotationType_e)
      {
        case e_CameraToWorld:
          v_EulerAngles_s.yaw_x   =   this->extrinsics_s.eulerAngles_s.yaw_x;
          v_EulerAngles_s.roll_x  =   this->extrinsics_s.eulerAngles_s.roll_x;
          break;
        case e_WorldToCamera: // fall through
        default:
          v_EulerAngles_s.yaw_x   = - this->extrinsics_s.eulerAngles_s.yaw_x;
          v_EulerAngles_s.roll_x  = - this->extrinsics_s.eulerAngles_s.roll_x;
          break;
      }
      v_ComputePreRoll_b = true;

      break;
    }

    default:
    {
      AssertFunction(false,"Tried to set unknown preRoll definition.");
      break;
    }
  }
  return core::RotationMatrix<T>(v_EulerAngles_s, v_ComputePreRoll_b);
}

//--------------------------------------------------------------------------
/// @brief Create a copy of this instance of different base type T1
///
/// The function returns a copy of this instance having converted all its \n
/// floating point values of base type T to new base type T1
///
/// @par Example usage:
/// @snippet Modeltester.cpp Pinhole_Constructor1
/// @snippet Modeltester.cpp Pinhole_convert_x
/// @snippet Modeltester.cpp
///
/// @return Copy of this instance of base type T1
//--------------------------------------------------------------------------
template<typename T>
template<typename T1>
Pinhole<T1> Pinhole<T>::convert_x(void) const
{
   typename Pinhole<T1>::Config_s v_PinholeCfg_s = {};
   this->copyConfig_v<T1>(v_PinholeCfg_s);
   return Pinhole<T1>(v_PinholeCfg_s);
}

//--------------------------------------------------------------------------
/// @brief Get a Pinhole reference by down-casting a reference to IImager
///
/// The functions tries to cast the IImager reference to a reference of an LensRadial
/// instance. On failure this causes an assertion, otherwise the function returns
/// a valid reference to a LensRadial instance
///
/// @par Example usage:
/// @snippet Modeltester.cpp Camera_PinholeConstructor
/// @snippet Modeltester.cpp Camera_Constructor4
/// @snippet Modeltester.cpp Camera_downcastPinhole
///
/// @return Valid reference to an instance of Pinhole
//--------------------------------------------------------------------------
template<typename T>
Pinhole<T>& Pinhole<T>::get_rx(IImager<T>& i_Imager_rx)
{
  Pinhole<T>* v_Pinhole_px = dynamic_cast<Pinhole<T>*>(&i_Imager_rx);
  AssertFunction(core::isNotNull_b(v_Pinhole_px), "Reference to imager is not a pinhole model.");
  return *v_Pinhole_px;
}

//--------------------------------------------------------------------------
/// @brief Get an instance of Pinhole<T> referenced by IImager<T1>&
///
/// The functions tries to cast the IImager of base type T1 reference to a Pinhole
/// instance of base type T1. On failure this causes an assertion,
/// otherwise the function returns a copy of that instance converted to base type T.
///
/// @par Example usage:
/// @snippet Modeltester.cpp Camera_PinholeConstructor
/// @snippet Modeltester.cpp Camera_Constructor4
/// @snippet Modeltester.cpp Camera_convertPinhole
///
/// @return Instance of Pinhole of base type T
//--------------------------------------------------------------------------
template<typename T>
template<typename T1>
Pinhole<T> Pinhole<T>::convert_x(IImager<T1>& i_Imager_rx)
{
  const Pinhole<T1>& c_Pinhole_rx = Pinhole<T1>::get_rx(i_Imager_rx);
  return c_Pinhole_rx.convert_x<T>();
}

//--------------------------------------------------------------------------
/// @brief Cast operator for conversion to different base type T1
///
/// Operator casts a Pinhole<T> object to different base type T1.
///
/// @par Example usage:
/// @snippet Modeltester.cpp Pinhole_Constructor1
/// @snippet Modeltester.cpp Pinhole_convertByCast
/// @snippet Modeltester.cpp Pinhole_convertByCtor
/// @snippet Modeltester.cpp Pinhole_convertByAssignment
///
/// @return Instance of LensRadial of base type T1
template<typename T>
template<typename T1>
Pinhole<T>::operator Pinhole<T1>() const
{
  return this->convert_x<T1>();
}


// --------------------------------------------------------------------------
/// @brief Apply projection to given points
///
/// Function applies projection of input point given in metric world coordinates 
/// and onto output point in metric image coordinate system
///
/// @par Example usage:
/// @snippet ModelTester.cpp Pinhole_Constructor1
/// @snippet ModelTester.cpp Pinhole_applyProjectionW2I_v
///
/// @param[in] i_Pos_rx     Metric world point location (projective 3D)
/// @param[out] o_Pos_rx    Metric image coordinates (projective 2D)
/// @return void
// --------------------------------------------------------------------------
template<typename T>
void Pinhole<T>::applyProjectionW2I_v(const core::Point4D<T>& i_Pos_rx,
                                            core::Point3D<T>& o_Pos_rx) const
{
  this->projection_x.applyProjectionW2I_v(i_Pos_rx, o_Pos_rx);

  return;
}

} // namespace model
} // namespace mecl

#endif // MECL_MODEL_PINHOLE_HPP_

/// @}
/// @}

