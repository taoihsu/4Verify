//--------------------------------------------------------------------------
/// @file IView.h
/// @brief Definition of the View Interface IView
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

#ifndef MECL_MODEL_IVIEW_H_
#define MECL_MODEL_IVIEW_H_

#include "config/IConfig.h"
#include "Camera.h"
#include "core/Point.h"

namespace mecl
{
namespace model
{
//--------------------------------------------------------------------------
/// @class IView
/// @brief Interface for the class SingleView.
///
//  --------------------------------------------------------------------------
template<typename T>
class IView : public config::IConfig
{
public:

  /// Deconstructor
  virtual ~IView(void) {}

  virtual void init_v(void) = 0;

  /// Project 3D point locations to camera image plane. 
  virtual void applyProjectionW2I_v(const model::Camera<T>& i_CamTo_rx,
                                    const core::Point4D<T>& i_Pos_rx,
                                          core::Point2D<T>& o_Pos_rx) const = 0;

  /// Project world metric point onto image plane and check visiblity and applicability
  virtual void applyProjectionW2I_v(const Camera<T>& i_CamTo_rx,
                                 const core::Point4D<T>& i_Pos_rx,
                                       core::Point2D<T>& o_Pos_rx,
                                                 bool_t& o_IsApplicable_rb,
                                                 bool_t& o_IsVisible_rb) const = 0;

  /// Apply homography from real to synthesized camera coordinates or vice versa.
  virtual void applyHomography_v(Camera<T>& i_CamFrom_rx,
                                 Camera<T>& i_CamTo_rx,
                    const core::Point3D<T>& i_Pos_rx,
                          core::Point3D<T>& o_Pos_rx) = 0;

};

} // namespace model
} // namespace mecl

#endif // MECL_MODEL_IVIEW_H_
/// @}
/// @}
