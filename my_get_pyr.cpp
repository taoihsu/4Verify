get_pyr(mecl::core::Matrix<float32_t, 3, 3> &R, double& pitch, double& yaw, double& roll, int CamNum, FILE *fp0)
//(const mecl::math::Matrix3d& R, double& pitch, double& yaw, double& roll) 
//mecl::core::Matrix<float32_t,3,3> &R_A_C, int CamNum, FILE *fp0
{
	double r11 = R(0, 0);
	double r12 = R(0, 1);
	double r13 = R(0, 2);
	double r21 = R(1, 0);
	double r22 = R(1, 1);
	double r23 = R(1, 2);
	double r31 = R(2, 0);
	double r32 = R(2, 1);
	double r33 = R(2, 2);

	
	 pitch = asin(r32);

	if (cos(pitch) == 0) {
		yaw = 0;
		roll = atan2(-r21, r11);
	}
	else {
		yaw = atan2(-r31, r33);
		roll = atan2(-r12, r22);
	}
	bool_t isImageFlipped_b = false;
	if (CamNum == 0) isImageFlipped_b = true;
	if (CamNum == 1) isImageFlipped_b = true;
	if (CamNum == 2) isImageFlipped_b = true;
	//if (CamNum == 3) isImageFlipped_b = true;
	// Adjust pitch, yaw, and roll based on camera orientation
	if (isImageFlipped_b) {
		// Front, left, and rear cameras are flipped, right camera is not flipped
		if (CamNum == 0) {
			// Front camera
			pitch = pitch;
			yaw = yaw ;
			roll = -roll + M_PI/2;
		}
		else if (CamNum == 1) {
			// Left camera
			pitch = -pitch;
			yaw = yaw + M_PI ;
			roll = roll;
		}
		else if (CamNum == 2) {
			// Rear camera
			pitch = pitch;
			yaw = yaw;
			roll = roll + M_PI/2;
		}
		else if (CamNum == 3) {
			// Right camera
			pitch = pitch;
			yaw = yaw - M_PI / 2;
			roll = -roll - M_PI / 2;
		}
	}
	else {
		// All cameras are not flipped
		if (CamNum == 0) {
			// Front camera
			pitch = -pitch;
			yaw = yaw;
			roll = -roll;
		}
		else if (CamNum == 1) {
			// Left camera
			pitch = -pitch;
			yaw = yaw - M_PI / 2;
			roll = -roll + M_PI / 2;
		}
		else if (CamNum == 2) {
			// Rear camera
			pitch = -pitch;
			yaw = yaw + M_PI;
			roll = -roll;
		}
		else if (CamNum == 3) {
			// Right camera
			pitch = pitch;
			yaw = yaw - M_PI ;
			roll = -roll ;
		}
	}
	mecl::core::Matrix<float32_t, 3, 1> PYR;
	PYR(0) = pitch;
	PYR(1) = yaw;
	PYR(2) = roll;
	return PYR;
}