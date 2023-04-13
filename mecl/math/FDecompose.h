/*
 * FDecompose.h
 *
 *  Created on: 18.03.2015
 *      Author: sai_hezol
 */

#ifndef SRC_MATH_FDECOMPOSE_H_
#define SRC_MATH_FDECOMPOSE_H_

#include "mecl/core/Array.h"
#include "mecl/core/RotationMatrix.h"
#include "mecl/core/MeclAssert.h"
#include "Math.h"

namespace mecl
{
namespace math
{

/*
 *
 */
template<typename T>
class FDecompose
{
public:

  // --------------------------------------------------------------------------
  // Rotation Matrix Decomposition
  // --------------------------------------------------------------------------
  struct Rotation
  {
    // Rotation matrix decomposition
    typedef enum { rx = 0, ry = 1, rz =2 } DecompIdx;
    typedef core::Array<core::RotationMatrix<T> ,3> Decomposition_ax;
    static void calcXYZAxisRotations_v(const core::RotationMatrix<T>& i_R_x,
                                       typename FDecompose<T>::Rotation::Decomposition_ax& o_Decomp_rax);
  };

  // --------------------------------------------------------------------------
  // Homography Matrix Decomposition into Rotation and Translation
  // Remark: K = I or known
  // --------------------------------------------------------------------------
  struct Homography
  {
    // Decompose Homography matrix into Rotation and Translation
    static void calcRotationAndTranslation_v(const core::Matrix3x3<T>& i_H_x,
                                             const core::Matrix3x3<T> i_K_x,
                                             core::RotationMatrix<T>& o_R_x,
                                             core::Point3D<T>& o_T_x);


    // Decompose Homography matrix into Rotation and Translation assuming K=I
    static void calcRotationAndTranslation_v(const core::Matrix3x3<T>& i_H_x,
                                             core::RotationMatrix<T>& o_R_x,
                                             core::Point3D<T>& o_T_x);
  private:

    // Calculate [r1 r2 t] (RT Matrix) from Homography and K matrix
    static void calcRT_v(const core::Matrix3x3<T>& i_H_x,
                         const core::Matrix3x3<T>& i_K_x,
                         core::Matrix3x3<T>& o_Rt_x,
                         T& o_Lambda_x);

    // Calculate [r1 r2 t] (RT Matrix) assuming K=I
    static void calcRT_v(const core::Matrix3x3<T>& i_H_x,
                         core::Matrix3x3<T>& o_RT_x);

    // Calculate Rotation from RT matrix
    static void calcRotationFromRT_v(const core::Matrix3x3<T>& i_RT_x,
                                     core::RotationMatrix<T>& o_R_x);
    // Calculate Translation from RT matrix
    static void calcTranslationFromRT_v(const core::Matrix3x3<T>& i_RT_x,
                                        core::Point3D<T> i_T_x);
  };

  // --------------------------------------------------------------------------
  // Singular Value Decomposition
  // --------------------------------------------------------------------------

  template<uint32_t m, uint32_t n>
  struct SingularValue
  {

    static void calcSVD_v(const core::Matrix<T,m,n>& i_A_x,
                          core::Matrix<T,m,m>& o_U_x,
                          core::Matrix<T,m,n>& o_S_x,
                          core::Matrix<T,n,n>& o_V_x);

    static void calcSVD_v( core::Matrix<T,m,m>& o_U_x,
                           core::Matrix<T,m,n>& b_S_x,
                           core::Matrix<T,n,n>& o_V_x,
                           const T& i_Value_x = math::constants<T>::minusOne_x());
  };


};

} /* namespace math */
} /* namespace mecl */

#include "FDecompose.hpp"

#endif /* SRC_MATH_FDECOMPOSE_H_ */
