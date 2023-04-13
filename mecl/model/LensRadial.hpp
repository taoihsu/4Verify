//--------------------------------------------------------------------------
/// @file LensRadial.hpp
/// @brief Contains the radial lens model implementation.
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

#ifndef MECL_MODEL_LENSRADIAL_HPP_
#define MECL_MODEL_LENSRADIAL_HPP_

#include "LensRadial.h"

namespace mecl
{
namespace model
{

//--------------------------------------------------------------------------
/// @brief Default constructor
///
/// The default construtor initializes all member variables to default values.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensRadial_Constructor1
///
// -------------------------------------------------------------------------
template<typename T, uint32_t od, uint32_t ou>
LensRadial<T, od, ou>::LensRadial(void)
: coeff_s()
, elevationMax_x(math::constants<T>::pi_x())
, configured_b(false)
{}

//--------------------------------------------------------------------------
/// @brief Constructor with initialization
///
/// The construtor initializes configuration data set based on input argument 
/// and marks object as configured.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensRadial_Constructor2
///
/// @param[in] i_LensConfig_ps Lens configuration to be applied to this LensRadial
// -------------------------------------------------------------------------
template<typename T, uint32_t od, uint32_t ou>
LensRadial<T, od, ou>::LensRadial(const Config_s& i_LensConfig_rs)
: coeff_s()
, elevationMax_x(math::constants<T>::pi_x())
, configured_b(true)
{
  this->updateConfig_v(i_LensConfig_rs);
  return;
}

//--------------------------------------------------------------------------
/// @brief Update configuration of lens parameters
///
/// The function copies all elements from \p i_LensConfig_rs struct to configuration object of LensRadial.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensRadial_Constructor1
/// @snippet ModelTester.cpp LensRadial_updateConfig_v
///
/// @param[in] i_LensConfig_rs Lens configuration to be applied to this LensRadial
/// @return void
// -------------------------------------------------------------------------
template<typename T, uint32_t od, uint32_t ou>
void LensRadial<T, od, ou>::updateConfig_v(const Config_s& i_LensConfig_rs)
{ 
  T* v_Val_px = &(this->coeff_s.world2image_x[0]); 
  const T* v_Coeff_px = &(i_LensConfig_rs.world2image_x[0]);
  const T* const c_DistCoeffEnd_px = &(i_LensConfig_rs.world2image_x[od+1]);

  do
  {
	  *(v_Val_px++) = *(v_Coeff_px++);
  } while (c_DistCoeffEnd_px != v_Coeff_px);

  v_Val_px = &(this->coeff_s.image2world_x[0]);
  v_Coeff_px = &(i_LensConfig_rs.image2world_x[0]);
  const T* const c_UndistCoeffEnd_px = &(i_LensConfig_rs.image2world_x[ou+1]);

  do
  {
	  *(v_Val_px++) = *(v_Coeff_px++);
  } while (c_UndistCoeffEnd_px != v_Coeff_px);

  const T c_MaxAngleRads_x = math::toRadians_x<T>(i_LensConfig_rs.elevationMaxCfg_x);

 
   AssertFunction(
             (c_MaxAngleRads_x > math::constants<T>::zero_x())
          && (c_MaxAngleRads_x <= math::constants<T>::pi_x()),
              "Maximal valid elevation angle for radial lens model is not valid."
             );

  this->elevationMax_x = math::toRadians_x<T>(i_LensConfig_rs.elevationMaxCfg_x);

  this->configured_b = true;

  return;
}

//--------------------------------------------------------------------------
/// @brief Update configuration of lens parameters with configuration data of different base type
///
/// The function copies all elements from \p i_LensConfig_rs  struct of base type T1 to configuration
/// object of LensRadial of (different) base type T.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensRadial_Constructor1
/// @snippet ModelTester.cpp LensRadial_Config_single
/// @snippet ModelTester.cpp LensRadial_updateConfig_otherBaseType
///
/// @param[in] i_LensConfig_rs Lens configuration to be applied to this LensRadial
/// @return void
// -------------------------------------------------------------------------
template<typename T, uint32_t od, uint32_t ou>
template<typename T1>
void LensRadial<T, od, ou>::updateConfig_v(const typename LensRadial<T1,od,ou>::Config_s& i_Config_rs)
{
  *this = LensRadial<T1>(i_Config_rs).convert_x<T>();
  return;
}

//--------------------------------------------------------------------------
/// @brief Copy configuration of lens parameters to POD
///
/// The function copies all elements from configuration object of LensRadial to struct \p o_Config_rs.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensRadial_Constructor2
/// @snippet ModelTester.cpp LensRadial_copyConfig_v
///
/// @param[in] o_Config_rs Configuration to be overwritten with contents from lens configuration
/// @return void
// -------------------------------------------------------------------------
template<typename T, uint32_t od, uint32_t ou>
void LensRadial<T, od, ou>::copyConfig_v(Config_s& o_Config_rs) const
{
  T* v_OutVal_px = &(o_Config_rs.world2image_x[0]);
  const T* v_Val_px = &(this->coeff_s.world2image_x[0]);
  const T* const c_ValEndDist_px = &(this->coeff_s.world2image_x[od+1]);

  do
  {
	  *(v_OutVal_px++) = *(v_Val_px++);
  } while (c_ValEndDist_px != v_Val_px);

  v_OutVal_px = &(o_Config_rs.image2world_x[0]);
  v_Val_px = &(this->coeff_s.image2world_x[0]);
  const T* const c_ValEndUndist_px = &(this->coeff_s.image2world_x[ou+1]);

  do
  {
	  *(v_OutVal_px++) = *(v_Val_px++);
  } while (c_ValEndUndist_px != v_Val_px);
  
  o_Config_rs.elevationMaxCfg_x = math::toDegrees_x<T>(this->elevationMax_x);

  return;
}

//--------------------------------------------------------------------------
/// @brief Copy configuration of lens parameters to POD of different base type T1
///
/// The function copies all elements from a radial lens model of base type T to
/// a configuration POD  struct \p o_Config_rs of (different) base type T1.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensRadial_Constructor2
/// @snippet ModelTester.cpp LensRadial_Config_single_default
/// @snippet ModelTester.cpp LensRadial_copyConfig_otherBaseType
///
/// @param[in] o_Config_rs Configuration to be overwritten with contents from lens configuration
/// @return void
// -------------------------------------------------------------------------

template<typename T, uint32_t od, uint32_t ou>
template<typename T1>
void LensRadial<T, od, ou>::copyConfig_v(typename LensRadial<T1,od, ou>::Config_s& o_Config_rs) const
{
  LensRadial<T1, od, ou>(this->convert_x<T1>()).copyConfig_v(o_Config_rs);
  return;
}


//--------------------------------------------------------------------------
/// @brief Copy configuration of lens parameters to volatile POD
///
/// The function copies all elements from configuration object of LensRadial to volatile struct \p o_Config_rs.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensRadial_Constructor2
/// @snippet ModelTester.cpp LensRadial_copyConfigVolatile_v
///
/// @param[in] o_Config_rs Configuration in volatile memory to be overwritten with contents from lens configuration
/// @return void
// -------------------------------------------------------------------------
template<typename T, uint32_t od, uint32_t ou>
void LensRadial<T, od, ou>::copyConfigVolatile_v(volatile Config_s& o_Config_rs) const
{
  volatile T* v_OutVal_px = &(o_Config_rs.world2image_x[0]);
  const T* v_Val_px = &(this->coeff_s.world2image_x[0]);
  const T* const c_ValEndDist_px = &(this->coeff_s.world2image_x[od+1]);

  do
  {
	  *(v_OutVal_px++) = *(v_Val_px++);
  } while (c_ValEndDist_px != v_Val_px);

  v_OutVal_px = &(o_Config_rs.image2world_x[0]);
  v_Val_px = &(this->coeff_s.image2world_x[0]);
  const T* const c_ValEndUndist_px = &(this->coeff_s.image2world_x[ou+1]);

  do
  {
	  *(v_OutVal_px++) = *(v_Val_px++);
  } while (c_ValEndUndist_px != v_Val_px);
  
  o_Config_rs.elevationMaxCfg_x = math::toDegrees_x<T>(this->elevationMax_x);

  return;
}

//--------------------------------------------------------------------------
/// @brief Returns true if lens was properly configured
///
/// The function returns a boolean indicating if lens object has been configured (true) or not (false).
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensRadial_Constructor1
/// @snippet ModelTester.cpp LensRadial_isConfigured_b
///
/// @return Boolean, True = configured, False = not configured.
// -------------------------------------------------------------------------
template<typename T, uint32_t od, uint32_t ou>
bool_t LensRadial<T, od, ou>::isConfigured_b(void) const
{
  return this->configured_b;
}

//--------------------------------------------------------------------------
/// @brief Applies lens distortion to undistorted points in camera coordinate system
///
/// The function applies distortion to 3D (projecitve 2D) camera input point \p i_Pos_rx and returns 2D distorted point in
/// image coordinate system.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensRadial_Constructor2
/// @snippet ModelTester.cpp LensRadial_applyDistortion_v
///
/// @param[in] i_Pos_rx       3D coordinate of point to be distorted
/// @param[in] i_DistLevel_rx Distortion level to be applied
/// @param[out] o_Pos_rx      Distorted 2D coordinates of input point i_Pos_rx in image coordinate system.
/// @return true, if point is within defined FOV range (applicable), otherwise false
// -------------------------------------------------------------------------
template<typename T, uint32_t od, uint32_t ou>
bool_t LensRadial<T, od, ou>::applyDistortion_b(const core::Point3D<T>& i_Pos_rx,
                                                      core::Point2D<T>& o_Pos_rx) const
{
	
  bool_t v_IsApplicable_b = true;

  // check if at least x or y coordinate is larger than epsilon, otherwise v_Phi_x is undefined
  if( not (math::isAboutZero_b(i_Pos_rx(0)) && math::isAboutZero_b(i_Pos_rx(1))) )
  {
    // get polar 3D coordinates
    const typename core::Point3D<T>::Polar_s c_Polar3D_s = i_Pos_rx.getPolar(e_Radians);
    v_IsApplicable_b = ( this->elevationMax_x > c_Polar3D_s.theta_x );

    // apply distortion polynomial f(theta) = radius (horner scheme)
	  T v_Radius_x =
	      math::FPolynomial<T, od>::eval_x(this->coeff_s.world2image_x, c_Polar3D_s.theta_x);
	
    // convert sensor polar 2D coordinates into metric 2D image location
    const typename core::Point2D<T>::Polar_s c_Polar2D_s = { v_Radius_x, c_Polar3D_s.phi_x, e_Radians };
    o_Pos_rx.setPolar(c_Polar2D_s);
  }
  else
  {
    // camera coordinate center is equal to distortion center
    o_Pos_rx = math::constants<T>::zero_x();
  }

  return v_IsApplicable_b;
}

template<typename T, uint32_t od, uint32_t ou>
bool_t LensRadial<T, od, ou>::isApplicable_b(const core::Point3D<T>& i_Pos_rx) const
{
  return ( i_Pos_rx.getElevation(e_Radians) < this->elevationMax_x);
}

template<typename T, uint32_t od, uint32_t ou>
typename ILens<T>::FieldOfView_s LensRadial<T, od, ou>::getFieldOfView_s(AngleUnit_e i_AngleUnit_e) const
{
  const T c_MaximalFOV_x = this->elevationMax_x + this->elevationMax_x;
  const T c_OrthoFOV_x = c_MaximalFOV_x / math::algebra<T>::sqrt_x(2.0f);
  const typename ILens<T>::FieldOfView_s c_FOV_s =
             { c_OrthoFOV_x, c_OrthoFOV_x, c_OrthoFOV_x, c_OrthoFOV_x,
               c_MaximalFOV_x,
               math::constants<T>::zero_x(), math::constants<T>::zero_x(),
               i_AngleUnit_e };
  return c_FOV_s;
}

/// Gets field of view for a image area defined by rectangle spanned by 2 metric coordinates
template<typename T, uint32_t od, uint32_t ou>
typename ILens<T>::FieldOfView_s LensRadial<T, od, ou>::getFieldOfView_s(const core::Point2D<T>& i_Pos1_rx,
                                                                         const core::Point2D<T>& i_Pos2_rx,
                                                                         AngleUnit_e i_AngleUnit_e) const
{
  typename ILens<T>::FieldOfView_s v_FieldOfView_s = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, i_AngleUnit_e};

  core::Point3D<T> v_Pos1_x;
  core::Point3D<T> v_Pos2_x;

  this->applyUndistortion_v(i_Pos1_rx, v_Pos1_x);
  this->applyUndistortion_v(i_Pos2_rx, v_Pos2_x);

  T v_Sign_x = math::sgn_x(i_Pos1_rx.getPosX()) * math::sgn_x(i_Pos2_rx.getPosX());

  v_FieldOfView_s.horizontal_x =
      v_Pos1_x.getElevationHorizontal(i_AngleUnit_e)
    - v_Sign_x * v_Pos2_x.getElevationHorizontal(i_AngleUnit_e);

  v_Sign_x = math::sgn_x(i_Pos1_rx.getPosY()) * math::sgn_x(i_Pos2_rx.getPosY());

  v_FieldOfView_s.vertical_x =
      v_Pos1_x.getElevationVertical(i_AngleUnit_e)
    - v_Sign_x * v_Pos2_x.getElevationVertical(i_AngleUnit_e);

  const T c_DiagFOV_x = math::algebra<T>::sqrt_x(   v_FieldOfView_s.horizontal_x * v_FieldOfView_s.horizontal_x
                                                  + v_FieldOfView_s.vertical_x * v_FieldOfView_s.vertical_x);

  const core::Point2D<T> c_Centroid_x ( (i_Pos1_rx + i_Pos2_rx) / static_cast<T> (2.0f) );
  core::Point3D<T> v_Centroid_x;

  this->applyUndistortion_v(c_Centroid_x , v_Centroid_x);

  v_FieldOfView_s.horizontalShift_x = v_Centroid_x.getElevationHorizontal(i_AngleUnit_e);
  v_FieldOfView_s.verticalShift_x = v_Centroid_x.getElevationVertical(i_AngleUnit_e);

  const core::Point2D<T> c_LeftMiddle_x ( i_Pos1_rx.getPosX(), c_Centroid_x.getPosY());
  const core::Point2D<T> c_RightMiddle_x (i_Pos2_rx.getPosX(), c_Centroid_x.getPosY());

  this->applyUndistortion_v(c_LeftMiddle_x , v_Pos1_x);
  this->applyUndistortion_v(c_RightMiddle_x, v_Pos2_x);

  v_Sign_x = math::sgn_x(c_LeftMiddle_x.getPosX()) * math::sgn_x(c_RightMiddle_x.getPosX());

  v_FieldOfView_s.centricHorizontal_x =
      v_Pos1_x.getElevationHorizontal(i_AngleUnit_e)
    - v_Sign_x * v_Pos2_x.getElevationHorizontal(i_AngleUnit_e);

  const core::Point2D<T> c_UpperMiddle_x ( c_Centroid_x.getPosX() , i_Pos1_rx.getPosY());
  const core::Point2D<T> c_LowerMiddle_x ( c_Centroid_x.getPosX() , i_Pos2_rx.getPosY());

  this->applyUndistortion_v(c_UpperMiddle_x, v_Pos1_x);
  this->applyUndistortion_v(c_LowerMiddle_x, v_Pos2_x);

  v_Sign_x = math::sgn_x(c_UpperMiddle_x.getPosY()) * math::sgn_x(c_LowerMiddle_x.getPosY());

  v_FieldOfView_s.centricVertical_x =
      v_Pos1_x.getElevationVertical(i_AngleUnit_e)
    - v_Sign_x * v_Pos2_x.getElevationVertical(i_AngleUnit_e);

  const typename core::Matrix<T, 5, 1>::Config_s c_FovValuesCfg_s =
  {
      v_FieldOfView_s.horizontal_x, v_FieldOfView_s.vertical_x,
      v_FieldOfView_s.centricHorizontal_x, v_FieldOfView_s.centricVertical_x,
      c_DiagFOV_x
  };

  const core::Matrix<T, 5, 1> c_FovValues_x(c_FovValuesCfg_s);

  v_FieldOfView_s.maximal_x = c_FovValues_x.maximum();

  return v_FieldOfView_s;

}

//--------------------------------------------------------------------------
/// @brief Removes distortion of metric image coordinates
///
/// The function applies undistortion of 2D image input point \p i_Pos_rx and returns undistorted 3D point
/// (projective 2D point) in camera coordinate system.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensRadial_Constructor2
/// @snippet ModelTester.cpp LensRadial_applyUndistortion_v
///
/// @param[in] i_Pos_rx       Distorted 2D coordinate of image point to be undistorted
/// @param[out] o_Pos_rx      Undistorted 3D coordinates of input point i_Pos_rx in camera coordinate system.
/// @return void
// -------------------------------------------------------------------------
template<typename T, uint32_t od, uint32_t ou>
void LensRadial<T, od, ou>::applyUndistortion_v(const core::Point2D<T>& i_Pos_rx,
                                                      core::Point3D<T>& o_Pos_rx) const
{
  // check at least x or y coordinate is larger than epsilon, otherwise v_Phi_x is undefined
  if ( not (math::isAboutZero_b(i_Pos_rx(0)) && math::isAboutZero_b(i_Pos_rx(1))) )
  {
    // get 2D polar coordinates of input point
    const typename core::Point2D<T>::Polar_s c_Polar2D_s = i_Pos_rx.getPolar(e_Radians);

    // apply undistortion polynomial f(radius) = theta (horner scheme)
	  const T c_Theta_x =
	      math::FPolynomial<T, ou>::eval_x(this->coeff_s.image2world_x, c_Polar2D_s.radius_x);

    // compute projective 2D coordinates out of polar coordinates of unit sphere
    const typename core::Point3D<T>::Polar_s c_Polar3D_s = {
        math::constants<T>::one_x(), c_Polar2D_s.phi_x , c_Theta_x, e_Radians
    };
    o_Pos_rx.setPolar(c_Polar3D_s);
  }
  else
  {
    // direction coincides with optical axis
    o_Pos_rx = math::constants<T>::zero_x();
    o_Pos_rx.setW(math::constants<T>::one_x());
  }

  return;
}

//--------------------------------------------------------------------------
/// @brief Create a copy of this instance of different base type T1
///
/// The function returns a copy of this instance having converted all its \n
/// floating point values of base type T to new base type T1
///
/// @par Example usage:
/// @snippet Modeltester.cpp LensRadial_Constructor1
/// @snippet Modeltester.cpp LensRadial_convert_x
///
/// @return Copy of this instance of base type T1
//--------------------------------------------------------------------------
template<typename T, uint32_t od, uint32_t ou>
template<typename T1>
LensRadial<T1, od, ou> LensRadial<T, od, ou>::convert_x(void) const
{
  typename LensRadial<T1, od, ou>::Config_s v_Config_s = {};
  for (uint32_t i=0; i <= od; i++)
  {
    v_Config_s.world2image_x[i] = static_cast<T1>(this->coeff_s.world2image_x[i]);
  }
  for (uint32_t i=0; i <= ou; i++)
  {
    v_Config_s.image2world_x[i] = static_cast<T1>(this->coeff_s.image2world_x[i]);
  }

  v_Config_s.elevationMaxCfg_x =
      math::toDegrees_x<T1>( static_cast<T1>(this->elevationMax_x) );

  return LensRadial<T1, od, ou>(v_Config_s);
}

//--------------------------------------------------------------------------
/// @brief Get a LensRadial reference by down-casting a reference to ILens
///
/// The functions tries to cast the ILens reference to a reference of an LensRadial
/// instance. On failure this causes an assertion, otherwise the function returns
/// a valid reference to a LensRadial instance
///
/// @par Example usage:
/// @snippet Modeltester.cpp Camera_LensConstructor
/// @snippet Modeltester.cpp Camera_Constructor5
/// @snippet Modeltester.cpp Camera_downcastLensRadial
///
/// @return Valid reference to an instance of LensRadial
//--------------------------------------------------------------------------
template<typename T, uint32_t od, uint32_t ou>
LensRadial<T, od, ou>& LensRadial<T, od, ou>::get_rx(ILens<T>& i_ILens_rx)
{
  LensRadial<T, od, ou>* v_Lens_px = dynamic_cast<LensRadial<T, od, ou>*>(&i_ILens_rx);
  AssertFunction(core::isNotNull_b(v_Lens_px), "Reference to lens is not a radial lens model.");
  return *v_Lens_px;
}

//--------------------------------------------------------------------------
/// @brief Get an instance of LensRadial<T> referenced by ILens<T1>&
///
/// The functions tries to cast the ILens of base type T1 reference to a LensRadial
/// instance of base type T1. On failure this causes an assertion,
/// otherwise the function returns a copy of that instance converted to base type T.
///
/// @par Example usage:
/// @snippet Modeltester.cpp Camera_LensConstructor
/// @snippet Modeltester.cpp Camera_Constructor5
/// @snippet Modeltester.cpp Camera_convertLensRadial
///
/// @return Instance of LensRadial of base type T
//--------------------------------------------------------------------------
template<typename T, uint32_t od, uint32_t ou>
template<typename T1>
LensRadial<T, od, ou> LensRadial<T, od, ou>::convert_x(ILens<T1>& i_Lens_rx)
{
  LensRadial<T1>& v_Lens_rx = LensRadial<T1, od, ou>::get_rx(i_Lens_rx);
  return v_Lens_rx.convert_x<T>();
}

//--------------------------------------------------------------------------
/// @brief Cast operator for conversion to different base type T1
///
/// Operator casts a LensRadial<T> object to different base type T1.
///
/// @par Example usage:
/// @snippet Modeltester.cpp LensRadial_Constructor1
/// @snippet Modeltester.cpp LensRadial_convertByCast
/// @snippet Modeltester.cpp LensRadial_convertByCtor
/// @snippet Modeltester.cpp LensRadial_convertByAssignment
///
/// @return Instance of LensRadial of base type T1
template<typename T, uint32_t od, uint32_t ou>
template<typename T1>
LensRadial<T, od, ou>::operator LensRadial<T1, od, ou>() const
{
    return this->convert_x<T1>();
}

} // namespace model
} // namespace mecl

#endif // MECL_MODEL_LENSRADIAL_HPP_

/// @}
/// @}
