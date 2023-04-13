//--------------------------------------------------------------------------
/// @file Projection.h
/// @brief Mapping from 3D to 2D image coordinate as non-parametric projection
///
/// Unlike the Pinhole model, the projection is non-parametric and is only defined by its projection
/// matrix.
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

#ifndef MECL_MODEL_PROJECTION_H_
#define MECL_MODEL_PROJECTION_H_

#include "core/MeclTypes.h"
#include "core/Point.h"
#include "core/Matrix.h"
#include "core/Matrix3x3.h"
#include "config/IConfig.h"
#include "IImager.h"
#include "ModelTypes.h"

namespace mecl
{
namespace model
{

//--------------------------------------------------------------------------
/// @class Projection
/// @brief Class implements linear projection to image frame
///
/// Class implements linear projection from 3D to 2D projective space
// --------------------------------------------------------------------------
template<typename T>
class Projection : public core::Matrix<T,3,4>, public IImager<T>
{
public:

  // --------------------------------------------------------------------------
  /// @struct Config_s
  /// @brief Class handling configuration parameters of Projection instance
  // --------------------------------------------------------------------------
  typedef typename core::Matrix<T, 3, 4>::Config_s Config_s;

  // --------------------------------------------------------------------------
  /// @brief Default constructor
  ///
  /// @par Example use:
  /// @snippet ModelTester.cpp Projection_Constructor1
  // --------------------------------------------------------------------------
  explicit Projection(void)
  : core::Matrix<T, 3, 4>()
  , configured_b(false)
  {}

  // --------------------------------------------------------------------------
  /// @brief Constructor copying configuration from existing configuration data set
  /// 
  /// @par Example use:
  /// @snippet ModelTester.cpp Projection_Constructor2
  /// 
  /// @param[in] i_Config_rs Projection model configuration
  // --------------------------------------------------------------------------
  explicit Projection(const Config_s& i_Config_rs)
  : core::Matrix<T, 3, 4>(i_Config_rs)
  , configured_b(true)
  {}

  // --------------------------------------------------------------------------
  /// @brief Virtual destructor
  // --------------------------------------------------------------------------
  virtual ~Projection(void) {}

  // --------------------------------------------------------------------------
  /// @brief Get copy of projection matrix
  /// 
  /// Return copy of projection matrix
  /// 
  /// @par Example use:
  /// @snippet ModelTester.cpp Projection_Constructor2
  /// @snippet ModelTester.cpp Projection_copyProjectionMatrix_x
  /// 
  /// @return Copy of projection matrix
  // --------------------------------------------------------------------------
  virtual core::Matrix<T, 3, 4> copyProjectionMatrix_x(void) const
  {
    return static_cast< core::Matrix<T, 3, 4> >(*this);
  }

  // --------------------------------------------------------------------------
  /// @brief Get reference to projection matrix
  /// 
  /// Return pointer reference to projection matrix
  /// 
  /// @par Example use:
  /// @snippet ModelTester.cpp Projection_Constructor2
  /// @snippet ModelTester.cpp Projection_getProjectionMatrix_rx
  /// 
  /// @return Reference to projection matrix
  // --------------------------------------------------------------------------
  virtual core::Matrix<T, 3, 4>& getProjectionMatrix_rx(void)
  {
    return dynamic_cast< core::Matrix<T, 3, 4>& >(*this);
  }

  // -------------------------------------------------------------------------
  /// @brief Get configuration status
  ///
  /// Return true if projection was properly configured
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Projection_Constructor1
  /// @snippet ModelTester.cpp Projection_isConfigured_b
  ///
  /// @return Boolean, True = configured, false = not configured
  // -------------------------------------------------------------------------
  virtual bool_t isConfigured_b(void) const
  {
    return this->configured_b;
  }

  // --------------------------------------------------------------------------
  /// @brief Initialize projection
  ///
  /// Implementation is empty here since nothing to do for projection
  ///
  /// @cond 
  /// @par Example usage:
  /// No source code example necessary.
  /// @endcond 
  ///
  /// @return void
  // ---------------------------------------------------------------------------
  virtual void init_v(void)
  {
    return;
  }

  // -------------------------------------------------------------------------
  /// @brief Update configuration
  ///
  /// The function sets projection model parameters from 
  /// \p i_Config_rs and marks the projection matrix object as not configured.
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Projection_Constructor1
  /// @snippet ModelTester.cpp Projection_updateConfig_v
  ///
  /// @param[in] i_Config_rs Configuration to be applied to this Projection
  /// @return void
  // -------------------------------------------------------------------------
  void updateConfig_v(const Config_s& i_Config_rs)
  {
    core::Matrix<T, 3, 4>::updateConfig_v(i_Config_rs);
    this->configured_b = true;
    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Project 3D point locations to camera image plane
  ///
  /// Function applies projection of input point given in metric world coordinates 
  /// and onto output point in metric image coordinate system
  ///
  /// @par Example usage:
  /// @snippet ModelTester.cpp Projection_Constructor2
  /// @snippet ModelTester.cpp Projection_applyProjectionW2I_v
  ///
  /// @param[in] i_Pos_rx     Metric world point location (projective 3D)
  /// @param[out] o_Pos_rx    Metric image coordinates (projective 2D)
  /// @return void
  // --------------------------------------------------------------------------
  virtual void applyProjectionW2I_v(const core::Point4D<T>& i_Pos_rx,
                                          core::Point3D<T>& o_Pos_rx) const
  {
    o_Pos_rx = this->mmul(i_Pos_rx);

    return;
  }

  // --------------------------------------------------------------------------
  /// @brief Get copy of z=0 plane homography of the projection
  /// 
  /// Get planar homography of image points (located on z=0 plane in camera 
  /// coordinate frame) onto z=0 plane in world coordinate frame
  /// 
  /// @par Example use:
  /// @snippet ModelTester.cpp Projection_Constructor2
  /// @snippet ModelTester.cpp Projection_getZPlaneHomography_x
  /// 
  /// @return Copy of z=0 plane homography of the projection
  /// (projective 2D linear transformation matrix)
  // --------------------------------------------------------------------------
  virtual const core::Matrix3x3<T> getZPlaneHomography_x(void) const
  {
    core::Matrix3x3<T> v_ZPlaneHomography_x;

    v_ZPlaneHomography_x.setCol(0, this->col(0));
    v_ZPlaneHomography_x.setCol(1, this->col(1));
    v_ZPlaneHomography_x.setCol(2, this->col(3));

    AssertFunction(true == v_ZPlaneHomography_x.isRegular_b(),
                   "Homography is not invertible, since origin of coordinate frame lies on z=0 plane.");

    return v_ZPlaneHomography_x;
  }

  virtual core::Matrix3x3<T> getKMatrix_x(void) const
    {
      AssertFunction(false,"Function not applicable for Default Imager.");
      return core::Matrix3x3<T>::zeros_x();
    }

  // --------------------------------------------------------------------------
  /// @brief Get copy of orthonormal basis
  /// 
  /// Get a copy of an orthonormal basis of the projection as 3x3 Matrix
  /// 
  /// @par Example use:
  /// @snippet ModelTester.cpp Projection_Constructor2
  /// @snippet ModelTester.cpp Projection_getOrthonormalBase_x
  /// 
  /// @return Copy of orthonormal basis
  // --------------------------------------------------------------------------
    virtual const core::Matrix3x3<T> getOrthonormalBase_x(void) const
  {
    core::Matrix3x3<T> v_OrthonormalBase_x;

    v_OrthonormalBase_x.setCol(0, this->col(0));
    v_OrthonormalBase_x.setCol(1, this->col(1));
    v_OrthonormalBase_x.setCol(2, this->col(3) - this->col(2));

    return v_OrthonormalBase_x;
  }

private:

  bool_t                configured_b;     ///< Flag indicating if projection is configured or not

};

} // namespace model
} // namespace mecl


#endif // MECL_MODEL_PROJECTION_H_
/// @}
/// @}
