#ifndef _EXTRINSICREFINEMENT_H_
#define _EXTRINSICREFINEMENT_H_

//#define MYPYR false
#define MYPYR true
#include <mecl/mecl.h>
#include <mecl/math/Math.h>
#include <mecl/core/Matrix3x3.h>
#include <mecl/core/RotationMatrix.h>
#include "Point.h"
#include "./src/Vpcs_types.h"
#ifdef PIKEOS
#include "logging/LogCtx.h"
#include "logging/ILogSender.h"
#include "logging/logprovider.h"
#endif

#ifndef M_PI
#define M_PI 3.141592653589793238462643
#endif

#ifndef M_PI_2
#define M_PI_2 1.570796326794896619231321
#endif

#include "IntCalib_Prj.h"


// Extrinsic refinement requires sorted lines (align with x or y axis) and approximate R & T fit
class ExRefine
{
public:
	// Use 3D Vanishing Points (VPs) to initialize RAdjust
	void VP3D_Extrinsics(MD_Data &DatCam, FILE *fp0);
	
	// Match detected lines with calibration target lines
	void MatchLines(MD_Data &DatCam, TargetLines &CalibTLines, bool MatchAllDL, mecl::core::ArrayList<sint32_t,vpcs::LINES_SIZE_MAX> *Matches, mecl::core::ArrayList<sint32_t,vpcs::LINES_SIZE_MAX> *MatchesAll, FILE *fp0); // MD_MOD_OPTIM_ALL_LINES this line

	// Refine extrinsics (i.e. GetCamPose) => return: Refined_R_A_C & Refined_R_C_A & Refined_xyz_CamPos
	void RefineExtrinsics(MD_Data &DatCam, TargetLines &CalibTLines, mecl::core::ArrayList<sint32_t,vpcs::LINES_SIZE_MAX> *Matches, float64_t &TotEr, bool MatchAllDL, FILE *fp0); // Basically GetCamPose from CamLocalization MD_MOD_OPTIM_ALL_LINES this line

	// Helper functions for GetCamPose
	mecl::core::Matrix<float64_t,9,1> CGR(const mecl::core::Matrix<float64_t,3,1> &rho);
	float64_t F_CGR(float64_t rho1, float64_t rho2, float64_t rho3, const mecl::core::Matrix<float64_t,9,9> &E);
	float64_t F_CGR(mecl::core::Matrix<float64_t,3,9> &BBinvBA, mecl::core::Matrix<float64_t,3,1> &T0, mecl::core::Matrix<float64_t,3,1> &Weights,
						  float64_t rho1, float64_t rho2, float64_t rho3, const mecl::core::Matrix<float64_t,9,9> &E);
	mecl::core::Matrix<float64_t,3,3> r9_to_R3(const mecl::core::Matrix<float64_t,9,1> &r9);
	mecl::core::Matrix<float64_t,9,1> R3_to_r9(const mecl::core::Matrix<float64_t,3,3> &R3);

	// Other operations
	template<typename T> mecl::core::Matrix<T,3,1> CrossProd(mecl::core::Matrix<T,3,1> &xyz1, mecl::core::Matrix<T,3,1> &xyz2);
	template<typename T> mecl::core::Matrix<T,3,3> Inverse3x3Mat(mecl::core::Matrix<T,3,3> &M0);
	template<typename T> mecl::core::Matrix<T,3,3> Ax2Ax_to_R3(mecl::core::Matrix<T,3,1> &Vec1, mecl::core::Matrix<T,3,1> &Vec2); // Find rotation matrix between two vectors/axis

	mecl::core::Matrix<float32_t,3,1> R3_to_pyr(mecl::core::Matrix<float32_t,3,3> &R3); // Rotation matrix => pitch/yaw/roll
	mecl::core::Matrix<float32_t,3,3> pyr_to_R3(mecl::core::Matrix<float32_t,3,1> &PYR); // pitch/yaw/roll => Rotation matrix

	// MATLAB_MATCH add following 4 lines:
	mecl::core::Matrix<float32_t,3,1> R3_to_pyr_Flip(mecl::core::Matrix<float32_t,3,3> &R3);
	mecl::core::Matrix<float32_t,3,3> pyr_to_R3_Flip(mecl::core::Matrix<float32_t,3,1> &PYR); // pitch/yaw/roll => Rotation matrix
	mecl::core::Matrix<float32_t,3,3> get_R_C_A(mecl::core::Matrix<float32_t,3,1> &PYR, int CamNum, FILE *fp0);
//#ifndef MYPYR
#if MYPYR == false
	mecl::core::Matrix<float32_t,3,1> get_pyr(mecl::core::Matrix<float32_t,3,3> &R_A_C, int CamNum, FILE *fp0);
#else
mecl::core::Matrix<float32_t, 3, 1> get_pyr(mecl::core::Matrix<float32_t, 3, 3> &R_A_C, double& pitch, double& yaw, double& roll, int CamNum, FILE *fp0);
#endif
public:
	// Vars
};
#endif
