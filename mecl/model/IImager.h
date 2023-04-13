//--------------------------------------------------------------------------
/// @file IImager.h
/// @brief Interface for Imagers and Definition of DefaultImager class template
///
/// Defines common interface for imagers and a default imager class template used
/// if camera model defines no imager.
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

#ifndef MECL_MODEL_IIMAGER_H_
#define MECL_MODEL_IIMAGER_H_

#include "config/IConfig.h"
#include "core/MeclTypes.h"
#include "core/Point.h"
#include "core/Matrix3x3.h"
#include "math/math.h"

namespace mecl
{
namespace model
{

//--------------------------------------------------------------------------
/// @class IImager
/// @brief Interface for imagers
///
/// Defines interface for all derived generic classes that implement an imager model
/// Remark: input and output coordinates are assumed to be metric
// --------------------------------------------------------------------------

template <typename T>
class IImager : public config::IConfig
{

public:
  /// @brief Destructor
  virtual ~IImager() {}

  /// Initialize internal data structures, e.g. projection matrix. Implementation is optional for derived classes.
  virtual void init_v(void) = 0;

  /// @brief Get planar homography of image points (located on z=0 plane in camera coordinate frame) onto z=0 plane in world coordinate frame
  /// @return z=0 plane homography of the projection (projective 2D linear transformation matrix)
  virtual const mecl::core::Matrix3x3<T> getZPlaneHomography_x(void) const = 0;

  virtual mecl::core::Matrix3x3<T> getKMatrix_x(void) const = 0;

  /// @brief Get reference to projection matrix
  /// @return Reference to projection matrix (projective 3D to 2D linear transformation matrix)
  virtual core::Matrix<T, 3, 4>& getProjectionMatrix_rx(void) = 0;

  /// @brief copy of project matrix
  /// @return Copy of projection matrix (projective 3D to 2D linear transformation matrix)
  virtual core::Matrix<T, 3, 4> copyProjectionMatrix_x(void) const = 0;

  /// @brief Project 3D point locations to camera image plane. Projections are have projective coordinates.
  /// @param[in] i_Pos_rx Reference to input point position (projective 3D coordinates)
  /// @param[out] o_Pos_rx Reference to output point after projection was applied (projective 2D coordinates)
  virtual void applyProjectionW2I_v(const core::Point4D<T>& i_Pos_rx, core::Point3D<T>& o_Pos_rx) const = 0;
};


//--------------------------------------------------------------------------
/// @class DefaultImager
/// @brief Default generic class implementing IImager\<T\>
/// Represents null value for instances of IImager\<T\>&. If camera instances are created without an imager instance,
/// singleton container core::Singleton\< DefaultImager\<T\> \>::getInstance_rx() is set as reference to the imager model.
/// Alternatively a camera instance can be initialized by non-singleton instance of this class:
/// Camera( model::DefaultImager\<T\>(), \<aLens\>, \<aSensor\> ) or \<aCamera\>.setImager_v(DefaultImager\<T\>())
//  --------------------------------------------------------------------------

template<typename T>
class DefaultImager : public IImager<T>
{
public:

  /// implements virtual method of IImager: implementation is void, since nothing to initialize
  virtual void init_v(void)
  {}

  /// implements virtual method of IConfig: default imager is always configured
  virtual bool_t isConfigured_b(void) const
  {
    return true;
  }

  /// implements virtual method of IImager: z=0 homography for unit projection is identity
  virtual const mecl::core::Matrix3x3<T> getZPlaneHomography_x(void) const
  {
    return core::Matrix3x3<T>::eye_x();
  }

  virtual mecl::core::Matrix3x3<T> getKMatrix_x(void) const
  {
    return mecl::core::Matrix3x3<T>::eye_x();
  }

  /// implements virtal method of IImager: gets reference to unit projection matrix
  virtual core::Matrix<T, 3, 4>& getProjectionMatrix_rx(void)
  {
    return this->getDefaultProjectionMatrix_rx();
  }

  /// implements virtual method of IImager: gets a copy of unit projection matrix
  virtual core::Matrix<T, 3, 4> copyProjectionMatrix_x(void) const
  {
     return core::Matrix<T, 3, 4>(this->getUnitProjectionMatrixCfg_s());
  }

  /// implements virutal method of IImager
  virtual void applyProjectionW2I_v(const core::Point4D<T>& i_Pos_rx, core::Point3D<T>& o_Pos_rx) const
  {
    o_Pos_rx.setRow(this->getDefaultProjectionMatrix_rx() % i_Pos_rx);
    return;
  }

  /// virtual destructor
  virtual ~DefaultImager()
  {}

private:

  /// gets unit projection as POD
  static const typename core::Matrix<T,3,4>::Config_s getUnitProjectionMatrixCfg_s(void)
  {
    static const typename core::Matrix<T, 3, 4>::Config_s c_DefaultProjectionConfig_s = {
               math::constants<T>::one_x() , math::constants<T>::zero_x(), math::constants<T>::zero_x(), math::constants<T>::zero_x(),
               math::constants<T>::zero_x(), math::constants<T>::one_x() , math::constants<T>::zero_x(), math::constants<T>::zero_x(),
               math::constants<T>::zero_x(), math::constants<T>::zero_x(), math::constants<T>::one_x() , math::constants<T>::one_x()
            };
    return c_DefaultProjectionConfig_s;
  };

  /// gets unit projection as reference to matrix
  static core::Matrix<T,3,4>& getDefaultProjectionMatrix_rx(void)
  {
    static core::Matrix<T,3,4> unitProjection_x( getUnitProjectionMatrixCfg_s() );
    return unitProjection_x;
  }

};

} // namespace model
} // namespace mecl

#endif // MECL_MODEL_IIMAGER_H_
/// @}
/// @}
