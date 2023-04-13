//--------------------------------------------------------------------------
/// @file ILens.h
/// @brief Interface for Lens and Definition of NoLens class template
///
/// Defines common interface for lenses and a default lens class template used
/// if camera model defines no lens.
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

#ifndef MECL_MODEL_ILENS_H_
#define MECL_MODEL_ILENS_H_

//! @cond
// PRQA S 1020 1 // macro is used by MKS
#define ILens_D_VERSION_ID "$Id: ILens.h 1.22 2016/12/06 04:46:15EST Schulte, Michael (MEE_MiSch) draft  $"
//! @endcond

#include "config/IConfig.h"
#include "core/MeclTypes.h"
#include "core/Point.h"
#include "core/Matrix.h"
#include "math/Math.h"

namespace mecl
{
namespace model
{

//--------------------------------------------------------------------------
/// @class ILens
/// @brief Interface for lenses
/// Defines interface for all generic classes that implement a lens model.
/// Remark: input and output coordinates are assumed to be metric.
///
//  --------------------------------------------------------------------------
template<typename T>
class ILens : public config::IConfig
{
public:

  /// POD struct for field of view values
  struct FieldOfView_s
  {
    T horizontal_x;             //< Maximal Field of View in horizontal direction
    T centricHorizontal_x;      //< Horizontal FOV measured from image center
    T vertical_x;               //< Maximal Field of View in vertical direction
    T centricVertical_x;        //< Vertical FOV measured from image center
    T maximal_x;                //< Maximal  overall field of view
    T horizontalShift_x;        //< Horizontal shift angle (@remark: can be negative)
    T verticalShift_x;          //< Vertical shift angle (@remark: can be negative)
    AngleUnit_e angleUnit_e;    //< Angle unit of FOV values (e_Radians, e_Degrees)
  };

  /// Destructor
  virtual ~ILens(void) {}

  /// Applies lens distortion to undistorted points in camera coordinate system
  virtual bool_t applyDistortion_b(const core::Point3D<T>& i_Pos_rx,
                                         core::Point2D<T>& o_Pos_rx) const = 0;

  /// Removes distortion of metric image coordinates
  virtual void applyUndistortion_v(const core::Point2D<T>& i_Pos_rx,
                                         core::Point3D<T>& o_Pos_rx) const = 0;

  /// Checks 3D coordinates if it is applicable to the lens model
  virtual bool_t isApplicable_b(const core::Point3D<T>& i_Pos_rx) const = 0;

  /// Gets field of view for which this lens model is applicable
  virtual FieldOfView_s getFieldOfView_s(AngleUnit_e i_AngleUnit_e = e_Degrees) const = 0;

  /// Gets field of view for a image area defined by rectangle spanned by 2 metric coordinates
  virtual FieldOfView_s getFieldOfView_s(const core::Point2D<T>& i_Pos1_x,
                                         const core::Point2D<T>& i_MaxPos_x,
                                         AngleUnit_e i_AngleUnit_e = e_Degrees) const = 0;

};

//--------------------------------------------------------------------------
/// @class NoLens
/// @brief Default generic class implementing ILens\<T\>
/// Represents null value for instances of ILens\<T\>&. If camera instances are created without a lens instance,
/// singleton container core::Singleton\< NoLens\<T\> >::getInstance_rx() is set as reference to the lens model.
/// Alternatively a camera instance can be initialized by non-singleton instance of this class:
/// Camera( \<anImager\> , model::NoLens\<T\>(), \<aSensor\> ) or \<aCamera\>.setNoLens_v()
//  --------------------------------------------------------------------------

template<typename T>
class NoLens : public ILens<T>
{
public:

    /// Returns true, no lens is always configured
    virtual bool_t isConfigured_b(void) const
    {
      return true;
    }

    /// Applies lens distortion to undistorted points: default lens just returns normalized points
    virtual bool_t applyDistortion_b(const core::Point3D<T>& i_Pos_rx,
                                         core::Point2D<T>& o_Pos_rx) const
    {
      bool_t c_IsAbpplicable_b = this->isApplicable_b(i_Pos_rx);
      if (c_IsAbpplicable_b)
      {
        o_Pos_rx = i_Pos_rx.getNormalized().template subVector<2>();
      }
      return c_IsAbpplicable_b;
    }

    /// Removes distortion of metric image coordinates: default lens just adds z-coordinate and sets it to 1
    virtual void applyUndistortion_v(const core::Point2D<T>& i_Pos_rx,
                                           core::Point3D<T>& o_Pos_rx) const
    {
      o_Pos_rx(0) = i_Pos_rx(0);
      o_Pos_rx(1) = i_Pos_rx(1);
      o_Pos_rx(2) = math::constants<T>::one_x();
      return;
    }

    /// Checks if point can be applied to default (no) lens projection
    virtual bool_t isApplicable_b(const core::Point3D<T>& i_Pos_x) const
    {
      // return i_Pos_x.getPosZ() > math::constants<T>::zero_x();
      return (i_Pos_x.getElevation(e_Degrees) < 70.0f);
    }

    /// Return Field of View value for no lens (pinhole camera)
    virtual typename ILens<T>::FieldOfView_s getFieldOfView_s(AngleUnit_e i_AngleUnit_e) const
    {
      const T c_Pi_x =
          i_AngleUnit_e == e_Radians ? math::constants<T>::pi_x()
                                    : static_cast<T>(180.0f);

      const typename ILens<T>::FieldOfView_s c_FOV_x =
                                      { c_Pi_x, c_Pi_x, c_Pi_x ,
                                        math::constants<T>::zero_x(),
                                        math::constants<T>::zero_x(),
                                        math::constants<T>::zero_x(),
                                        math::constants<T>::zero_x(),
                                        i_AngleUnit_e};
      return c_FOV_x;
    }

    /// Gets field of view for a image area defined by rectangle spanned by 2 metric coordinates
    virtual typename ILens<T>::FieldOfView_s getFieldOfView_s(const core::Point2D<T>& i_Pos1_rx,
                                           const core::Point2D<T>& i_Pos2_rx,
                                           AngleUnit_e i_AngleUnit_e = e_Degrees) const
    {
      const T c_Pi_x =
                i_AngleUnit_e == e_Radians ? math::constants<T>::pi_x()
                                          : static_cast<T>(180.0f);
      typename ILens<T>::FieldOfView_s v_FieldOfView_s = { c_Pi_x, c_Pi_x, c_Pi_x ,
                                                           math::constants<T>::zero_x(),
                                                           math::constants<T>::zero_x(),
                                                           math::constants<T>::zero_x(),
                                                           math::constants<T>::zero_x(),
                                                           i_AngleUnit_e};

      v_FieldOfView_s.angleUnit_e = i_AngleUnit_e;

      const core::Point3D<T> c_Pos1_x(i_Pos1_rx(0) , i_Pos1_rx(1), math::constants<T>::one_x());
      const core::Point3D<T> c_Pos2_x(i_Pos2_rx(0) , i_Pos2_rx(1), math::constants<T>::one_x());

      T v_Sign_x = math::sgn_x(i_Pos1_rx.getPosX()) * math::sgn_x(i_Pos2_rx.getPosX());

      v_FieldOfView_s.horizontal_x =
            c_Pos1_x.getElevationHorizontal(i_AngleUnit_e)
          - v_Sign_x * c_Pos2_x.getElevationHorizontal(i_AngleUnit_e);

      v_Sign_x = math::sgn_x(i_Pos1_rx.getPosY()) * math::sgn_x(i_Pos2_rx.getPosY());

      v_FieldOfView_s.vertical_x =
             c_Pos1_x.getElevationVertical(i_AngleUnit_e)
           - v_Sign_x * c_Pos2_x.getElevationVertical(i_AngleUnit_e);

      v_FieldOfView_s.maximal_x =
          math::algebra<T>::sqrt_x(   v_FieldOfView_s.horizontal_x * v_FieldOfView_s.horizontal_x
                                    + v_FieldOfView_s.vertical_x * v_FieldOfView_s.vertical_x);

      const core::Point2D<T> c_PosCenter_x ( (i_Pos1_rx + i_Pos2_rx )/static_cast<T>(2.0f));
      const core::Point3D<T> c_Pos3dCenter_x( c_PosCenter_x.getPosX(),
                                              c_PosCenter_x.getPosY(),
                                              math::constants<T>::one_x() );

      v_FieldOfView_s.horizontalShift_x = c_Pos3dCenter_x.getElevationHorizontal(i_AngleUnit_e);
      v_FieldOfView_s.verticalShift_x = c_Pos3dCenter_x.getElevationVertical(i_AngleUnit_e);

      const core::Point3D<T> c_PosUpperMiddle_x ( c_PosCenter_x.getPosX(), i_Pos1_rx.getPosY(), math::constants<T>::one_x() );
      const core::Point3D<T> c_PosLowerMiddle_x ( c_PosCenter_x.getPosX(), i_Pos2_rx.getPosY(), math::constants<T>::one_x() );

      v_FieldOfView_s.centricHorizontal_x = v_FieldOfView_s.horizontal_x;
      v_FieldOfView_s.centricVertical_x = v_FieldOfView_s.vertical_x;

      return v_FieldOfView_s;
    }


    /// Virtual Destructor
    virtual ~NoLens()
    {};

};

} // namespace model
} // namespace mecl

#endif // MECL_MODEL_ILENS_H_
/// @}
/// @}
