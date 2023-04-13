#ifndef _INTCALIB_PRJ_H_
#define _INTCALIB_PRJ_H_

#include <mecl/mecl.h>
#include <mecl/math/Math.h>
#include <mecl/core/Matrix3x3.h>
#include <mecl/core/RotationMatrix.h>
#include "Point.h"

#include "MD_VarStore.h"


// 3D camera coordinates => 2D pixels
template<int n, typename T>
inline mecl::core::Matrix<T,2,n> Project3D_to_2D(mecl::core::Matrix<T,3,n> &xyz_C, MD_Data &DatCam, int DistType, mecl::core::Matrix<T,1,n> &r2) 
// DistType: 0 = NoDistortion, 1 = RadialDistortion
{
	// Setup
	T fx = (T)DatCam.CamFC[0];
	T fy = (T)DatCam.CamFC[1];
	T u0 = (T)DatCam.CamPP[0];
	T v0 = (T)DatCam.CamPP[1];
	mecl::core::Matrix<T,2,n> uv_C;

	if (DistType==1) // Radial distortion
	{
		int Order = 6 - 1;
		for (int i=0; i<n; i++)
		{
			// Convert to polar for radial distortion
			T rad = pow( xyz_C(0,i)*xyz_C(0,i) + xyz_C(1,i)*xyz_C(1,i) + xyz_C(2,i)*xyz_C(2,i) , 0.5);
			T phi = atan2( xyz_C(1,i) , xyz_C(0,i) );
			T theta = acos( xyz_C(2,i) / rad ); // acos(zVec/rad)
			
			// Apply distortion
			T theta_adj = DatCam.Dist_world2im[Order];
			for (int j=0; j<Order; j++)
			{
				theta_adj *= theta;
				theta_adj += DatCam.Dist_world2im[Order-j-1];
			}
			r2(0,i) = theta_adj;

			// Convert back to pixels
			uv_C(0,i) = (theta_adj*cos(phi)) * fx + u0;
			uv_C(1,i) = (theta_adj*sin(phi)) * fy + v0;
		}
	}
	else // No distortion
	{
		for (int i=0; i<n; i++)
		{
			T xOffset = (xyz_C(0,i)/xyz_C(2,i)) * fx;
			T yOffset = (xyz_C(1,i)/xyz_C(2,i)) * fy;
			r2(0,i) = xOffset*xOffset + yOffset*yOffset;
			uv_C(0,i) = xOffset + u0;
			uv_C(1,i) = yOffset + v0;
		}
	}
	
	return uv_C;
}


// 2D pixels => 3D camera coordinates
template<uint32_t n, typename T>
inline mecl::core::Matrix<T,3,n> Project2D_to_3D(mecl::core::Matrix<T,2,n> &uv_C,  MD_Data &DatCam, int DistType)
// DistType: 0 = NoDistortion, 1 = RadialDistortion
{
	// Setup
	T fx = (T)DatCam.CamFC[0];
	T fy = (T)DatCam.CamFC[1];
	T u0 = (T)DatCam.CamPP[0];
	T v0 = (T)DatCam.CamPP[1];
	mecl::core::Matrix<T,3,n> xyz_C;

	if (DistType==1) // Radial distortion
	{
	
	}
	else // No distortion
	{
		for (uint32_t i=0; i<n; i++)
		{
			xyz_C(0,i) = (uv_C(0,i)-u0)/fx;
			xyz_C(1,i) = (uv_C(1,i)-v0)/fy;
			xyz_C(2,i) = 1;
		}
	}

	return xyz_C;
}

#endif
