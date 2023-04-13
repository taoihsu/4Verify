#include "MD_VarStore.h"




// Add on some extrinsics needed to relate reference frames
void MD_Data::ExtrinsicsMD_C_A(void)
{

	// RPY = roll pitch yaw from VanishingPoint analysis
	// A = cAr Axle reference frame
	// C = cam reference frame

	R_A_C0 = mecl::core::Matrix<float32_t,3,3>::zeros_x();
	if (CamNum==0) // Front camera
	{
		R_A_C0(0,1) =  1; // x_A =>  y_C
		R_A_C0(1,0) =  1; // y_A =>  x_C
		R_A_C0(2,2) = -1; // z_A => -z_C	
	}
	else if (CamNum==1) // Left camera
	{
		R_A_C0(0,0) = -1; // x_A => -x_C
		R_A_C0(1,1) =  1; // y_A =>  y_C
		R_A_C0(2,2) = -1; // z_A => -z_C
	}
	else if (CamNum==2) // Rear camera
	{
		R_A_C0(0,1) = -1; // x_A => -y_C
		R_A_C0(1,0) = -1; // y_A => -x_C
		R_A_C0(2,2) = -1; // z_A => -z_C
	}
	else // Right camera
	{
		R_A_C0(0,0) =  1; // x_A =>  x_C
		R_A_C0(1,1) = -1; // y_A => -y_C
		R_A_C0(2,2) = -1; // z_A => -z_C
	}
}