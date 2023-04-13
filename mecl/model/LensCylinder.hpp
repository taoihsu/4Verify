//--------------------------------------------------------------------------
/// @file LensCylinder.hpp
/// @brief Contains the cylinder lens model implementation.
///
/// The lens model provides a warping and an un-warping function.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Hagen Marczok (hagen.marczok@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef MECL_MODEL_LENSCYLINDER_HPP_
#define MECL_MODEL_LENSCYLINDER_HPP_

#include "LensCylinder.h"

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
/// @snippet ModelTester.cpp LensCylinder_Constructor1
// -------------------------------------------------------------------------
template<typename T>
LensCylinder<T>::LensCylinder(void)
: excentr_x(math::constants<T>::one_x())
, excentricitySqr_x(math::constants<T>::one_x())
, configured_b(false)
{}

//--------------------------------------------------------------------------
/// @brief Constructor with initialization
///
/// The construtor initializes lens configuration data based on input argument.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensCylinder_Constructor2
///
/// @param[in] i_LensConfig_rs Lens configuration to be applied to this LensCylinder
// -------------------------------------------------------------------------
template<typename T>
LensCylinder<T>::LensCylinder(const Config_s& i_LensConfig_rs)
: excentr_x(i_LensConfig_rs.excentricity_x)
, excentricitySqr_x(i_LensConfig_rs.excentricity_x * i_LensConfig_rs.excentricity_x)
, configured_b(true)
{}

//--------------------------------------------------------------------------
/// @brief Copy configuration of lens parameters to POD
///
/// The function copies all elements from configuration object of LensCylinder to struct \p o_Config_rs.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensCylinder_Constructor2
/// @snippet ModelTester.cpp LensCylinder_copyConfig_v
///
/// @param[in] o_Config_rs Configuration to be overwritten with contents from lens configuration
/// @return void
// -------------------------------------------------------------------------
template<typename T>
void LensCylinder<T>::copyConfig_v(Config_s& o_LensConfig_rs) const
{
  o_LensConfig_rs.excentricity_x = this->excentr_x;
  return;
}

//--------------------------------------------------------------------------
/// @brief Copy configuration of lens parameters to volatile POD
///
/// The function copies all elements from configuration object of LensCylinder to volatile struct \p o_Config_rs.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensCylinder_Constructor2
/// @snippet ModelTester.cpp LensClyinder_copyConfigVolatile_v
///
/// @param[in] o_Config_rs Configuration in volatile memory to be overwritten with contents from lens configuration
/// @return void
// -------------------------------------------------------------------------
template<typename T>
void LensCylinder<T>::copyConfigVolatile_v(volatile Config_s& o_LensConfig_rs) const
{
  o_LensConfig_rs.excentricity_x = this->excentr_x;
  return;
}

//--------------------------------------------------------------------------
/// @brief Copy configuration of lens parameters to POD of different base type T1
///
/// The function copies all elements from a cylinder lens model of base type T to
/// a configuration POD  struct \p o_Config_rs of (different) base type T1.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensCylinder_Constructor2
/// @snippet ModelTester.cpp LensCylinder_Config_single
/// @snippet ModelTester.cpp LensCylinder_copyConfig_v_otherBaseType
///
/// @param[in] o_Config_rs Configuration to be overwritten with contents from lens configuration
/// @return void
// -------------------------------------------------------------------------
template<typename T>
template<typename T1>
void LensCylinder<T>::copyConfig_v(typename LensCylinder<T1>::Config_s& o_LensConfig_rs) const
{
  o_LensConfig_rs.excentricity_x = static_cast<T1>(this->excentr_x);
  return;
}

//--------------------------------------------------------------------------
/// @brief Update configuration of lens parameters
///
/// The function copies all elements from \p i_LensConfig_rs struct to configuration object of LensCylinder.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensCylinder_Constructor1
/// @snippet ModelTester.cpp LensCylinder_updateConfig_v
///
/// @param[in] i_LensConfig_rs Lens configuration to be applied to this LensCylinder
/// @return void
// -------------------------------------------------------------------------
template<typename T>
void LensCylinder<T>::updateConfig_v(const Config_s& i_LensConfig_rs)
{
  this->excentr_x = i_LensConfig_rs.excentricity_x;

  this->configured_b = true;
  return;
}

//--------------------------------------------------------------------------
/// @brief Update configuration of lens parameters with configuration data of different base type T1
///
/// The function copies all elements from \p i_LensConfig_rs  struct of base type T1 to configuration
/// object of LensCylinder of (different) base type T.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensCylinder_Constructor1
/// @snippet ModelTester.cpp LensCylinder_Config_single
/// @snippet ModelTester.cpp LensCylinder_updateConfig_v_otherBaseType
///
/// @param[in] i_LensConfig_rs Lens configuration to be applied to this LensCylinder
/// @return void
// -------------------------------------------------------------------------
template<typename T>
template<typename T1>
void LensCylinder<T>::updateConfig_v(const typename LensCylinder<T1>::Config_s& i_LensConfig_rs)
{
  this->excentr_x = static_cast<T1>(i_LensConfig_rs.excentricity_x);
  this->excentricitySqr_x = this->excentr_x * this->excentr_x;
  return;
}

//--------------------------------------------------------------------------
/// @brief Returns true if lens was properly configured
///
/// The function returns a boolean indicating if lens object has been configured (true) or not (false).
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensCylinder_Constructor1
/// @snippet ModelTester.cpp LensCylinder_isConfigured_b
///
/// @return Boolean, Always True = configured.
// -------------------------------------------------------------------------
template<typename T>
bool_t LensCylinder<T>::isConfigured_b(void) const
{
  return this->configured_b;
}

//--------------------------------------------------------------------------
/// @brief Applies lens distortion to undistorted points in camera coordinate system
///
/// The function applies distortion to 3D camera input point \p i_Pos_rx and returns 2D distorted point in
/// image coordinate system
///
/// @param[in]  i_Pos_rx        3D coordinate of point to be distorted
/// @param[in]  i_DistLevel_rx  Distortion level to be applied
/// @param[out] o_Pos_rx        Distorted 2D coordinates of input point i_Pos_rx in image coordinate system.
/// @return true, if 3D point (direction) is applicable, otherwise false
// -------------------------------------------------------------------------
template<typename T>
bool_t LensCylinder<T>::applyDistortion_b(const core::Point3D<T>& i_Pos_rx,
                                                core::Point2D<T>& o_Pos_rx) const
{
  bool_t v_Applicable_b = true;

  AssertFunction(not math::isAboutZero_b<T>(this->excentr_x),
                 "Too small value for excentricity of cylinder lens.");

  const T c_Angle_x =
      math::trigonometry<T>::atan2_x(  i_Pos_rx.getPosX(),
                                       i_Pos_rx.getPosZ() * this->excentr_x
                                    );

  const T c_Rho_x =
      math::algebra<T>::sqrt_x(  i_Pos_rx.getPosX() * i_Pos_rx.getPosX()
                               + i_Pos_rx.getPosZ() * i_Pos_rx.getPosZ()
                                                    * this->excentricitySqr_x
                              );

  // assign arclength on cylinder cross-section to x
  o_Pos_rx.setPosX( c_Angle_x * this->excentr_x );

  v_Applicable_b = not(math::isZero_b(c_Rho_x));

  // assign normalized y value
  o_Pos_rx.setPosY( v_Applicable_b ?

                         // normalize to cylinder with
                         // rho^2 = X^2 + excentricity^2 * Z^2 = 1
                         // -> y = Y/rho
                         i_Pos_rx(1) / c_Rho_x

                         // in case point is not applicable set y.to
                         // numeric limits of the arithmetic type
                       : ( i_Pos_rx.getPosY() > 0 ?
                                mecl::math::numeric_limits<T>::max_x()
                             :  mecl::math::numeric_limits<T>::lowest_x()
                         )
                  );

  return v_Applicable_b;
}

//--------------------------------------------------------------------------
/// @brief Checks if 3D coordinate is applicable to cylinder lens model
///
/// Point is applicable iff its direction doesn't exceed an Y elevation angle of 90°.
/// This is the case if sqrt(x^2 + excentricity^2 * z^2) equals zero.
///
/// @return true, if 3D Point (direction) is applicable, otherwise false
// -------------------------------------------------------------------------

template<typename T>
bool_t LensCylinder<T>::isApplicable_b(const core::Point3D<T>& i_Pos_rx) const
{
  const T c_Rho_x =  math::algebra<T>::sqrt_x(  i_Pos_rx.getPosX() * i_Pos_rx.getPosX()
                                              + i_Pos_rx.getPosZ() * i_Pos_rx.getPosZ()
                                                * this->excentr_x * this->excentr_x);

  return not( math::isAboutZero_b(c_Rho_x) );
}

//--------------------------------------------------------------------------
/// @brief Gets field of view for which this lens model is applicable
///
/// Gets Field of View of this lens in horizonal, vertical and applicable range
template<typename T>
typename ILens<T>::FieldOfView_s LensCylinder<T>::getFieldOfView_s(AngleUnit_e i_AngleUnit_e) const
{
  const T c_Pi_x =
      i_AngleUnit_e == e_Radians ? math::constants<T>::pi_x()
                                 : static_cast<T>(180.0f);
  const typename ILens<T>::FieldOfView_s c_FOV_x =
                              { 2 * c_Pi_x, 2 * c_Pi_x,
                                    c_Pi_x, c_Pi_x,
                                    2 * c_Pi_x,
                                    0.0f,
                                    0.0f,
                                    i_AngleUnit_e};
  return c_FOV_x;
}

/// Gets field of view for a image area defined by rectangle spanned by 2 metric coordinates
template<typename T>
typename ILens<T>::FieldOfView_s LensCylinder<T>::getFieldOfView_s(const core::Point2D<T>& i_Pos1_rx,
                                                                   const core::Point2D<T>& i_Pos2_rx,
                                                                   AngleUnit_e i_AngleUnit_e) const
{
  typename ILens<T>::FieldOfView_s v_FieldOfView_s =  { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, i_AngleUnit_e };

  core::Point3D<T> v_Pos1_x;
  core::Point3D<T> v_Pos2_x;

  this->applyUndistortion_v(i_Pos1_rx, v_Pos1_x);
  this->applyUndistortion_v(i_Pos2_rx, v_Pos2_x);

  T v_Sign_x = math::sgn_x(i_Pos1_rx.getPosX()) * math::sgn_x(i_Pos2_rx.getPosX());

  v_FieldOfView_s.horizontal_x =
      v_Pos1_x.getElevationHorizontal(i_AngleUnit_e)
    - v_Sign_x * v_Pos2_x.getElevationHorizontal(i_AngleUnit_e);

  v_FieldOfView_s.centricHorizontal_x = v_FieldOfView_s.horizontal_x;

  v_Sign_x = math::sgn_x(i_Pos1_rx.getPosY()) * math::sgn_x(i_Pos2_rx.getPosY());

  v_FieldOfView_s.vertical_x =
      v_Pos1_x.getElevationVertical(i_AngleUnit_e)
    - v_Sign_x * v_Pos2_x.getElevationVertical(i_AngleUnit_e);

  v_FieldOfView_s.centricVertical_x = v_FieldOfView_s.vertical_x;

  const core::Point2D<T> c_Centroid_x ( (i_Pos1_rx + i_Pos2_rx) / static_cast<T> (2.0f) );
  core::Point3D<T> v_Centroid_x;

  this->applyUndistortion_v(c_Centroid_x, v_Centroid_x);

  v_FieldOfView_s.horizontalShift_x = v_Centroid_x.getElevationHorizontal(i_AngleUnit_e);
  v_FieldOfView_s.verticalShift_x = v_Centroid_x.getElevationVertical(i_AngleUnit_e);

  return v_FieldOfView_s;

}



//--------------------------------------------------------------------------
/// @brief Removes distortion of metric image coordinates
///
/// The function applies undistortion of 2D image input point \p i_Pos_rx and returns undistorted 3D point
/// in camera coordinate system.
///
/// @par Example usage:
/// @snippet ModelTester.cpp LensCylinder_Constructor2
/// @snippet ModelTester.cpp LensCylinder_applyUndistortion_v
///
/// @param[in] i_Pos_rx       Distorted 2D coordinate of image point to be undistorted
/// @param[out] o_Pos_rx      Undistorted 3D coordinates of input point i_Pos_rx in camera coordinate system.
/// @return void
// -------------------------------------------------------------------------
template<typename T>
void LensCylinder<T>::applyUndistortion_v(const core::Point2D<T>& i_Pos_rx,
                                                core::Point3D<T>& o_Pos_rx) const
{
  // check that cylinder excentricity is larger than epsilon, otherwise division by zero
  if(math::numeric_limits<T>::epsilon_x() < this->excentr_x)
  {
    // calculate angle under circle arc
    T v_Angle_x = i_Pos_rx.getPosX() / this->excentr_x;

    o_Pos_rx.setPosX(math::trigonometry<T>::sin_x(v_Angle_x) * this->excentr_x);
    o_Pos_rx.setPosZ(math::trigonometry<T>::cos_x(v_Angle_x));
  }
  else
  {
    // x coordinate is not modified, z coordinate is set to 1
    o_Pos_rx.setPosX(i_Pos_rx.getPosX());
    o_Pos_rx.setPosZ(math::constants<T>::one_x());
  }
  o_Pos_rx.setPosY(i_Pos_rx.getPosY());

  return;
}

//--------------------------------------------------------------------------
/// @brief Create a copy of this instance of different base type T1
///
/// The function returns a copy of this instance having converted all its \n
/// floating point values of base type T to new base type T1
///
/// @par Example usage:
/// @snippet Modeltester.cpp LensCylinder_Constructor1
/// @snippet Modeltester.cpp LensCylinder_convert_x
///
/// @return Copy of this instance of base type T1
//--------------------------------------------------------------------------
template<typename T>
template<typename T1> inline
LensCylinder<T1> LensCylinder<T>::convert_x(void) const
{
  typename LensCylinder<T1>::Config_s v_Config_s = {};
  v_Config_s.excentricity_x = this->excentr_x;
  return LensCylinder<T1>(v_Config_s);
}

//--------------------------------------------------------------------------
/// @brief Get a LensCylinder reference by down-casting a reference to ILens
///
/// The functions tries to cast the ILens reference to a reference of an LensCylinder
/// instance. On failure this causes an assertion, otherwise the function returns
/// a valid reference to a LensRadial instance
///
/// @par Example usage:
/// @snippet Modeltester.cpp Camera_CylinderLensConstructor
/// @snippet Modeltester.cpp Camera_Constructor9
/// @snippet Modeltester.cpp Camera_downcastLensCylinder
///
/// @return Valid reference to an instance of LensCylinder
//--------------------------------------------------------------------------
template<typename T>
LensCylinder<T>& LensCylinder<T>::get_rx(ILens<T>& i_ILens_rx)
{
  LensCylinder<T>* v_Lens_px = dynamic_cast<LensCylinder<T>*>(&i_ILens_rx);
  AssertFunction(core::isNotNull_b(v_Lens_px), "ILens reference is not a cylinder lens.");
  return *v_Lens_px;
}

//--------------------------------------------------------------------------
/// @brief Get an instance of LensCylinder<T> referenced by ILens<T1>&
///
/// The functions tries to cast the ILens of base type T1 reference to a LensRadial
/// instance of base type T1. On failure this causes an assertion,
/// otherwise the function returns a copy of that instance converted to base type T.
///
/// @par Example usage:
/// @snippet Modeltester.cpp Camera_CylinderLensConstructor
/// @snippet Modeltester.cpp Camera_Constructor9
/// @snippet Modeltester.cpp Camera_convertLensCylinder
///
/// @return Instance of LensCylinder of base type T
//--------------------------------------------------------------------------
template<typename T>
template<typename T1>
LensCylinder<T> LensCylinder<T>::convert_x(ILens<T1>& i_ILens_rx)
{
  LensCylinder<T1>& v_Lens_rx = LensCylinder<T1>::get_rx(i_ILens_rx);
  return v_Lens_rx.convert_x<T>();
}

//--------------------------------------------------------------------------
/// @brief Cast operator for conversion to different base type T1
///
/// Operator casts a LensCylinder<T> object to different base type T1.
///
/// @par Example usage:
/// @snippet Modeltester.cpp LensCylinder_Constructor1
/// @snippet Modeltester.cpp LensCylinder_convertByCast
/// @snippet Modeltester.cpp LensCylinder_convertByCtor
/// @snippet Modeltester.cpp LensCylinder_convertByAssignment
///
/// @return Instance of LensCylinder of base type T1
//--------------------------------------------------------------------------
template<typename T>
template<typename T1>
LensCylinder<T>::operator LensCylinder<T1>() const
{
    return this->convert_x<T1>();
}

} // namespace model
} // namespace mecl

#endif // MECL_MODEL_LENSCYLINDER_HPP_

/// @}
/// @}
