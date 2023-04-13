//--------------------------------------------------------------------------
/// @file SingleView.h
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
/// @author Helmut Zollner (helmut.zollner@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef MECL_MODEL_SINGLEVIEW_H_
#define MECL_MODEL_SINGLEVIEW_H_

//! @cond 
// PRQA S 1020 1 // macro is used by MKS
#define SingleView_D_VERSION_ID "$Id: SingleView.h 1.28 2016/04/22 10:48:58EDT Zollner, Helmut (SAI_HeZol) draft  $"
//! @endcond 

#include "IView.h"
#include "Camera.h"
#include "core/Matrix3x3.h"
#include "core/MeclAssert.h"
#include "core/Point.h"

namespace mecl
{
namespace model
{
//--------------------------------------------------------------------------
/// @class SingleView
/// @brief SingleView projection model
///
/// Models projection between virtual and real camera and vice versa.
// --------------------------------------------------------------------------
template <typename T>
class SingleView : public IView<T>
{
public:

  // --------------------------------------------------------------------------
  // Constructors, Destructor

  /// Default constructor
  SingleView();

  /// Constructor with camera references and default distortion level
  explicit SingleView(Camera<T>& realCam_rx, Camera<T>& syntCam_rx);

  /// Constructor with camera references and real camera distortion level
  explicit SingleView(Camera<T>& realCam_rx, Camera<T>& syntCam_rx,
                      const T& i_RealDistortionCorrectionLvl_rx);

  /// Constructor with camera references and camera specific distortion
  explicit SingleView(Camera<T>& realCam_rx, Camera<T>& syntCam_rx,
                        const T& i_RealDistortionCorrectionLvl_rx,
                        const T& i_SyntDistortionCorrectionLvl_rx);

  /// Virtual destructor
  virtual ~SingleView(void) {};

  /// Initialization
  virtual void init_v(void);

  // --------------------------------------------------------------------------
  // Getters

  /// Get reference to real to synthetic homography matrix
  const core::Matrix3x3<T>& getRealToSyntheticHomography_rx() const;

  /// Get reference to synthetic to real homography matrix
  const core::Matrix3x3<T>& getSyntheticToRealHomography_rx() const;

  /// Returns true if camera objects are properly configured
  virtual bool_t isConfigured_b(void) const;

  /// Get pointer to real camera object
  Camera<T>* getRealCamera_px();

  /// Get pointer to synthetic camera object
  Camera<T>* getSyntheticCamera_px();

  /// Copy real camera distortion level
   T copyRealCorrectionLvl_x() const;

  /// Copy synthetic camera distortion level
  T copySyntheticCorrectionLvl_x() const;

  /// Get distortion levels for both cameras
  void copyCorrectionLvls_v(T& o_RealDistortionCorrectionLvl_rx, T& o_SyntDistortionCorrectionLvl_rx);

  // --------------------------------------------------------------------------
  // Setters

  /// Set new real camera reference
  void setRealCam_v(Camera<T>& i_RealCam_rx);

  /// Set synthetic camera reference
  void setSyntCam_v(Camera<T>& i_SyntCam_rx);

  /// Set distortion levels
  void setCorrectionLvls_v(const T& i_RealDistortionCorrectionLvl_rx, const T& i_SyntDistortionCorrectionLvl_rx);

  /// Set real camera distortion level
  void setRealCorrectionLvl_v(const T& i_DistortionCorrectionLvl_rx);

  /// Set synthetic camera distortion level
  void setSyntheticCorrectionLvl_v(const T& i_DistortionCorrectionLvl_rx);

  // --------------------------------------------------------------------------
  // Processing methods

  /// Project real image point to synthetic image point
  void applyRealToSyntheticProjection_v(const core::Point2D<T>& i_Pos_rx,
                                              core::Point2D<T>& o_Pos_rx);

  /// Project real image point to synthetic image point and check visibility and applicability
  void applyRealToSyntheticProjection_v(const core::Point2D<T>& i_Pos_rx,
                                              core::Point2D<T>& o_Pos_rx,
                                                        bool_t& o_IsApplicable_rb,
                                                        bool_t& o_IsVisible_rb);

  /// Project synthetic image point to real image point
  void applySyntheticToRealProjection_v(const core::Point2D<T>& i_Pos_rx,
                                              core::Point2D<T>& o_Pos_rx);

  /// Project synthetic image point to real image point and check visibility and applicability
  void applySyntheticToRealProjection_v(const core::Point2D<T>& i_Pos_rx,
                                              core::Point2D<T>& o_Pos_rx,
                                                        bool_t& o_IsApplicable_rb,
                                                        bool_t& o_IsVisible_rb);

  /// Project world metric point onto image plane
  virtual void applyProjectionW2I_v(const Camera<T>& i_CamTo_rx,
                             const core::Point4D<T>& i_Pos_rx,
                                   core::Point2D<T>& o_Pos_rx) const;

  /// Project world metric point onto image plane and check visiblity and applicability
  virtual void applyProjectionW2I_v(const Camera<T>& i_CamTo_rx,
                               const core::Point4D<T>& i_Pos_rx,
                                     core::Point2D<T>& o_Pos_rx,
                                               bool_t& o_IsApplicable_rb,
                                               bool_t& o_IsVisible_rb) const;

  /// Project world metric point onto real camera image plane
  void applyRealProjectionW2I_v(const core::Point4D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const;

  /// Project world metric point onto real camera image plane and check visibility and applicability
  void applyRealProjectionW2I_v(const core::Point4D<T>& i_Pos_rx,
                                      core::Point2D<T>& o_Pos_rx,
                                                bool_t& o_IsApplicable_rb,
                                                bool_t& o_IsVisible_rb) const;

  /// Project world point onto synthetic camera image plane
  void applySyntheticProjectionW2I_v(const core::Point4D<T>& i_Pos_rx, core::Point2D<T>& o_Pos_rx) const;

  /// Project world metric point onto real camera image plane and check visibility and applicability
  void applySyntheticProjectionW2I_v(const core::Point4D<T>& i_Pos_rx,
                                           core::Point2D<T>& o_Pos_rx,
                                                     bool_t& o_IsApplicable_rb,
                                                     bool_t& o_IsVisible_rb) const;

  /// Apply homography from one camera to another on an image point
  virtual void applyHomography_v(Camera<T>& i_CamFrom_rx,
                                 Camera<T>& i_CamTo_rx,
                    const core::Point3D<T>& i_Pos_rx,
                          core::Point3D<T>& o_Pos_rx);

 /// Clears homography initialization flags
 void reset_v(void);
 
private:

  // --------------------------------------------------------------------------
  // Private methods

  /// Calculate homography for mapping points form real to synthetic image frame
  void calcReal2SyntHomography_v(void);

  /// Calculate homography for mapping points form synthetic to real image frame
  void calcSynt2RealHomography_v(void);

  // --------------------------------------------------------------------------
  // Private data members

  model::Camera<T>* realCam_px; ///< real camera, projection is determined by EOL and OC calibration
  model::Camera<T>* syntCam_px; ///< synthetic camera with design parameters

  core::Matrix3x3<T> real2SyntHomography_x; ///< homography for mapping form real to synthetic camera image plane
  core::Matrix3x3<T> synt2RealHomography_x; ///< homography for mapping form synthetic to real camera image plane

  T realCorrectionLvl_x; ///< distortion correction level for real camera, default: 0.0f (100% distortion)
  T syntCorrectionLvl_x; ///< distortion correction level for synthetic camera, default: 0.0f (100% distortion)

  bool_t initializedSynt2Real_b;  ///< True if synthetic to real homograhpy is initialized, otherwise false
  bool_t initializedReal2Synt_b;  ///< True if real to synthetic homograhpy is initialized, otherwise false

}; // class SingleView

} // namespace model
} // namespace mecl

#include "SingleView.hpp"

#endif // MECL_MODEL_SINGLEVIEW_H_
/// @}
/// @}
