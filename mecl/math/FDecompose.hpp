/*
 * FDecompose.hpp
 *
 *  Created on: 18.03.2015
 *      Author: sai_hezol
 */

#ifndef SRC_MATH_FDECOMPOSE_HPP_
#define SRC_MATH_FDECOMPOSE_HPP_

#include "FDecompose.h"

namespace mecl {
namespace math {

template<typename T>
void FDecompose<T>::Rotation::calcXYZAxisRotations_v(const core::RotationMatrix<T>& i_R_x,
                                                     Decomposition_ax& o_Decomp_rax)
{
  const typename core::RotationMatrix<T>::EulerAngles_s c_EulerAngles_s = i_R_x.calcEA_s();

  // x axis
  o_Decomp_rax[rx] = static_cast<core::RotationMatrix<T> >(core::Matrix<T, 3, 3>::eye_x());
  o_Decomp_rax[rx](1,1) =   math::trigonometry<T>::cos_x(c_EulerAngles_s.pitch_x);
  o_Decomp_rax[rx](2,1) =   math::trigonometry<T>::sin_x(c_EulerAngles_s.pitch_x);
  o_Decomp_rax[rx](1,2) = - o_Decomp_rax[0](2,1);
  o_Decomp_rax[rx](2,2) =   o_Decomp_rax[0](1,1);

  // y-axis
  o_Decomp_rax[ry] = static_cast<core::RotationMatrix<T> >(core::Matrix<T, 3, 3>::eye_x());
  o_Decomp_rax[ry](0,0) =   math::trigonometry<T>::cos_x(c_EulerAngles_s.yaw_x);
  o_Decomp_rax[ry](0,2) =   math::trigonometry<T>::sin_x(c_EulerAngles_s.yaw_x);
  o_Decomp_rax[ry](2,0) = - o_Decomp_rax[1](0,2);
  o_Decomp_rax[ry](2,2) =   o_Decomp_rax[1](0,0);

  // z-axis
  o_Decomp_rax[rz] = static_cast<core::RotationMatrix<T> >(core::Matrix<T, 3, 3>::eye_x());
  o_Decomp_rax[rz](0,0) =   math::trigonometry<T>::cos_x(c_EulerAngles_s.roll_x);
  o_Decomp_rax[rz](1,0) =   math::trigonometry<T>::sin_x(c_EulerAngles_s.roll_x);
  o_Decomp_rax[rz](0,1) = - o_Decomp_rax[2](1,0);
  o_Decomp_rax[rz](1,1) =   o_Decomp_rax[2](0,0);

  return;
};



// Decompose Homography matrix into Rotation
template<typename T>
void FDecompose<T>::Homography::calcRT_v(const core::Matrix3x3<T>& i_H_x,
                                         const core::Matrix3x3<T>& i_K_x,
                                         core::Matrix3x3<T>& o_RT_x,
                                         T& o_Lambda_x)
{
  const core::Matrix3x3<T> c_KInv_x = i_K_x.inverse();
  o_Lambda_x = static_cast< core::Vector<T,3> >( c_KInv_x % i_H_x.col(0) ).norm();
  o_RT_x = c_KInv_x % i_H_x;
  o_RT_x /= o_Lambda_x ;
  return;
}

// Decompose Homography matrix into Rotation + translation assuming K=eye()
template<typename T>
void FDecompose<T>::Homography::calcRT_v(const core::Matrix3x3<T>& i_H_x,
                                         core::Matrix3x3<T>& o_RT_x)
{
  const T c_h1Norm_x = static_cast< core::Vector<T,3> >( i_H_x.col(0) ).norm();
  o_RT_x = i_H_x.div(c_h1Norm_x);
  return;
}

// Calculate Rotation from RT matrix
template<typename T>
void FDecompose<T>::Homography::calcRotationFromRT_v(const core::Matrix3x3<T>& i_RT_x,
                                                     core::RotationMatrix<T>& o_R_x)
{
  o_R_x.setCol(0, i_RT_x.col(0) );
  o_R_x.setCol(1, i_RT_x.col(1) );
  core::Vector<T,3> c_R3_x =
      static_cast<core::Point3D<T> >(i_RT_x.col(0)) ^ static_cast<core::Point3D<T> >(i_RT_x.col(1));
  o_R_x.setCol(2, c_R3_x);
  // orthogonalize here: [U S V] = svd(T) , Tcorrected = U * V'
  return;
}

template<typename T>
void FDecompose<T>::Homography::calcRotationAndTranslation_v(const core::Matrix3x3<T>& i_H_x,
                                                             const core::Matrix3x3<T> i_K_x,
                                                             core::RotationMatrix<T>& o_R_x,
                                                             core::Point3D<T>& o_T_x)
{
  core::Matrix3x3<T> v_RT_x;
  T v_Lambda_x;
  calcRT_v(i_H_x, i_K_x, v_RT_x, v_Lambda_x );
  calcRotationFromRT_v(v_RT_x, o_R_x);
  o_T_x = v_RT_x.col(2);
  return;
}

template<typename T>
void FDecompose<T>::Homography::calcRotationAndTranslation_v(const core::Matrix3x3<T>& i_H_x,
                                                             core::RotationMatrix<T>& o_R_x,
                                                             core::Point3D<T>& o_T_x)
{
  core::Matrix3x3<T> v_RT_x;
  calcRT_v(i_H_x, v_RT_x);
  calcRotationFromRT_v(v_RT_x, o_R_x);
  o_T_x = v_RT_x.col(2);
  return;
}

template<typename T> template<uint32_t m, uint32_t n>
void FDecompose<T>::SingularValue<m,n>::calcSVD_v(const core::Matrix<T,m,n>& i_A_x,
                                                  core::Matrix<T,m,m>& o_U_x,
                                                  core::Matrix<T,m,n>& o_S_x,
                                                  core::Matrix<T,n,n>& o_V_x)
{

  if (m < n) { // first form of the singular value decomposition
    o_S_x = i_A_x;
  } else {     // second form
    o_S_x = i_A_x.t();
  }
  o_U_x.setEye();
  o_V_x.setEye();

  calcSVD_v(o_U_x, o_S_x, o_V_x);

  // tbc ...

  return;
}

template<typename T> template<uint32_t m, uint32_t n>
void FDecompose<T>::SingularValue<m,n>::calcSVD_v( core::Matrix<T,m,m>& o_U_x,
    core::Matrix<T,m,n>& b_S_x,
    core::Matrix<T,n,n>& o_V_x,
    const T& i_Value_x)
{
  core::Vector<T,m> v_HouseholderVec_x;

  // bi-diagonalization

  core::Vector<T,n> v_AbsDiagS_x = b_S_x.diag().abs();

  for (uint32_t i=0; i<n; i++) {

    // column householder
    T c_InvNorm_x = math::constants<T>::one_x() / b_S_x.col(i).norm();

    T c_Alpha_x =
        math::algebra<T>::sqrt_x(   math::constants<T>::one_x()
                                  + v_AbsDiagS_x(i) * c_InvNorm_x);

    T c_Beta_x =     b_S_x(i,i) < math::constants<T>::zero_x() ?
                     c_InvNorm_x / c_Alpha_x
                  :  math::neg_x(c_InvNorm_x / c_Alpha_x);

    v_HouseholderVec_x(i) = math::neg_x(c_Alpha_x);
    for (uint32_t j=i+1; j<m; j++) {
      v_HouseholderVec_x(i) = c_Beta_x * b_S_x(j,i);
    }

    o_U_x -= (o_U_x % v_HouseholderVec_x) % v_HouseholderVec_x;

    // row householder,
    if ( (n-1) == i ) { // skip following part of loop for last householder vector element
      b_S_x -= (b_S_x % v_HouseholderVec_x) % v_HouseholderVec_x;
      continue;
    }

    const core::Vector<T,n-1> c_AbsSubDiagS_x = b_S_x.subMatrix(0,1).diag().abs();

    c_InvNorm_x = math::constants<T>::one_x() /  b_S_x.row(i+1).norm();

    c_Alpha_x = math::algebra<T>::sqrt_x(   math::constants<T>::one_x()
                                          + c_AbsSubDiagS_x(i) * c_InvNorm_x);

    c_Beta_x =  b_S_x(i,i+1) < math::constants<T>::zero_x() ?
                     c_InvNorm_x / c_Alpha_x
                   : math::neg_x(c_InvNorm_x / c_Alpha_x);

    v_HouseholderVec_x(i+1) = math::neg_x(c_Alpha_x);
    for (uint32_t j=i+2; j<n; j++) {
      v_HouseholderVec_x(i) = c_Beta_x * b_S_x(j,i);
    }

    o_V_x -= (o_V_x % v_HouseholderVec_x) % v_HouseholderVec_x;

    b_S_x -= (b_S_x % v_HouseholderVec_x) % v_HouseholderVec_x;


  }

  // diagonalization
  uint32_t v_K0_u32 = 0;
  v_AbsDiagS_x = b_S_x.diag().abs();
  while (v_K0_u32 < n-1) {

    T v_SMax_x = math::constants<T>::zero_x();
    for (uint32_t i=0; i<n; i++) {
      v_SMax_x = math::max_x<T>(v_AbsDiagS_x(i), v_SMax_x);
    }
    // .. tbc
  }


}


} /* namespace math */
} /* namespace mecl */

#endif /*  */



