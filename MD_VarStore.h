#ifndef _MD_VARSTORE_H_
#define _MD_VARSTORE_H_

#include <mecl/mecl.h>
#include <mecl/math/Math.h>
#include <mecl/core/Matrix3x3.h>
#include <mecl/core/RotationMatrix.h>
#include "Point.h"

#include "./src/Vpcs_types.h"




class MD_Data
{
public:
	MD_Data()
    : numEndPts_s32(0)
    , numLinLab_s32(0)
    , CamNum(0)
  {
	}

	// Functions
	void ExtrinsicsMD_C_A(void);

	// Reference Frame (RF) definitions
	// A = cAr Axle RF with x pointing backwards, y pointing right, z pointing up. Origin at center of the front axle projected onto the ground plane (z=0)
	// C = Camera   RF with x pointing right,     y pointing down,  z pointing forward
	// C0 = Camera pointing down i.e. with Roll, Pitch, Yaw = 0. From the camera perspective, x points right, y points down, z points forward. R_A_C = R_A_C0 * RAdjust; RAdjust=R_C0_C;

	// Extrinsics known ahead of time
	mecl::core::Matrix<float32_t,3,3> R_A_C0; // Rotation matrix from Camera with no RPY from VP (C0) to cAr Axle (A). R_A_C = R_A_C0 * RAdjust;

	// Extrinsics set ahead of time (approx)
	mecl::core::Matrix<float32_t,3,1> Orig_xyz_CamPos;
	mecl::core::Matrix<float32_t,3,1> Orig_pyr_CamPos;
	mecl::core::Matrix<float32_t,3,3> Orig_RAdjust;
	mecl::core::Matrix<float32_t,3,3> Orig_R_A_C;
#ifdef VPCS2D
	// Fitted camera extrinsics - 2D VP's
	mecl::core::Matrix<float32_t,3,1> VP2D_xyz_CamPos; // [x/y/z] camera position written in cAr Axle (A) reference frame
	mecl::core::Matrix<float32_t,3,1> VP2D_pyr_CamPos; // [pitch/yaw/roll] relative to the C0 RF
	mecl::core::Matrix<float32_t,3,3> VP2D_RAdjust; // Extrinsic matrix solved by vanishing points = R_C0_C => pitch/yaw/roll
	mecl::core::Matrix<float32_t,3,3> VP2D_R_A_C; // Rotation matrix from Camera with RPY from VP (C) to cAr Axle (A)
#endif
	// Fitted camera rotation using 3D VP's - MDeetjen algorithm
	mecl::core::Matrix<float32_t,3,1> VP3D_pyr_CamPos;
	mecl::core::Matrix<float32_t,3,3> VP3D_RAdjust;
	mecl::core::Matrix<float32_t,3,3> VP3D_R_A_C;

	// Refined camera extrinsics - MDeetjen algorithm
	mecl::core::Matrix<float32_t,3,1> Refined_xyz_CamPos;
	mecl::core::Matrix<float32_t,3,1> Refined_pyr_CamPos;
	mecl::core::Matrix<float32_t,3,3> Refined_RAdjust;
	mecl::core::Matrix<float32_t,3,3> Refined_R_A_C;

	// Lines used in vanishing points
	float32_t Lines_EndPts[vpcs::LINES_SIZE_MAX][4]; // [Line#][x1/x2/y1/y2]
	sint32_t Lines_Lab[vpcs::LINES_SIZE_MAX]; // [Line#] label (horizontal=1 or vertical=0 or other=-1)

	sint32_t numEndPts_s32;
	sint32_t numLinLab_s32;

	// Camera intrinsics
	float32_t CamFC[2]; // FoCal length
	float32_t CamPP[2]; // Principal point
	float32_t Dist_im2world[6]; // Distortion coeff
	float32_t Dist_world2im[6]; // Distortion coeff

	int CamNum; // Front:0, Left:1, Rear:2, Right:3

	// Images
	uint8_t ImUD[vpcs::c_ImageSize_u32]; // Undistorted Image
#ifdef OPENCV_OUT
	uint8_t ImD[vpcs::c_ImageSize_u32]; // Distorted Image
#endif
};



// Calibration target lines
// MD_MOD_3LINES start
enum class NUM_LINES
{
	TWO_LINES,
	THREE_LINES
};
class TargetLines
{
public:
	TargetLines() {
		sint32_t tempyCamSee[4][4] = { { 0,0,0,0 } , { 1,2,3,4 } , { 5,0,0,0 } , { 1,2,3,4 } }; // Which yLines each camera can see {Front, Left, Rear, Right} => make it empty to not use yLines for that camera
		sint32_t tempyCamSeeLen[4] = { 1,4,1,4 };
		memcpy(yCamSee,tempyCamSee,4*4*4);
		memcpy(yCamSeeLen, tempyCamSeeLen, 4 * 4);
	}

	// Set separation distance and 2 vs. 3 line calibration target
	void SetSepDist(float32_t SepDist_, NUM_LINES nLines_)
	{
		SepDist = SepDist_; // Separation distance between calibration targets (inner lines)
		LinesLen[1] = 6;
		yL0 = -2730; // APPROXIMATE... BAD!!
		//yL0 = -2250;
		yL1 = -500;
		yL2 = -400;
		yL3 = 1400;
		yL4 = 1500;
		yL5 = 7170; // APPROXIMATE... BAD!!
		//yL5 = 5750;
		xR1 = 0.5*SepDist;
		xR2 = 0.5*SepDist + 100;
		xR3 = 0.5*SepDist + 200;
		xR4 = 0.5*SepDist + 700;
		xR5 = 0.5*SepDist + 900;
		if (nLines_ == NUM_LINES::TWO_LINES)
		{
 			nLines = nLines_;
			LinesLen[0] = 12;
			xR6 = 0.5*SepDist + 1000;
			xR7 = 0;
			xR8 = 0;
			xOuter = xR6;
			// BW label: for xL/R# which align with x axis: BW=0 if LoY is white & HiY is black,       BW=1 if LoY is black & HiY is white
			//			 for y#    which align with y axis: BW=0 if LoX is white & HiX is black/mixed, BW=1 if LoX is black/mixed & HiX is white
			float32_t Lines_[2][16][5] = { {    {yL0,yL5 , -xR1,-xR1 , 1}, // xL1
												{yL0,yL5 , -xR2,-xR2 , 0}, // xL2
												{yL0,yL5 , -xR3,-xR3 , 1}, // xL3
												{yL0,yL5 , -xR4,-xR4 , 0}, // xL4
												{yL0,yL5 , -xR5,-xR5 , 1}, // xL5
												{yL0,yL5 , -xR6,-xR6 , 0}, // xL6
												 {yL0,yL5 ,  xR1, xR1 , 0}, // xR1
												 {yL0,yL5 ,  xR2, xR2 , 1}, // xR2
												 {yL0,yL5 ,  xR3, xR3 , 0}, // xR3
												 {yL0,yL5 ,  xR4, xR4 , 1}, // xR4
												 {yL0,yL5 ,  xR5, xR5 , 0}, // xR5
												 {yL0,yL5 ,	xR6, xR6 , 1},  // xR6
												{ 0,0 , 0, 0 , 0 },
												{ 0,0 , 0, 0 , 0 },
												{ 0,0 , 0, 0 , 0 },
												{ 0,0 , 0, 0 , 0 }},
											  { {yL0,yL0 , -xOuter, xOuter , 0}, // y0
												{yL1,yL1 , -xOuter, xOuter , 1}, // y1
												{yL2,yL2 , -xOuter, xOuter , 0}, // y2
												{yL3,yL3 , -xOuter, xOuter , 1}, // y3
												{yL4,yL4 , -xOuter, xOuter , 0}, // y4
												{yL5,yL5 , -xOuter, xOuter , 1}, // y5
												{ 0,0 , 0, 0 , 0 },
												{ 0,0 , 0, 0 , 0 }, 
												{ 0,0 , 0, 0 , 0 }, 
												{ 0,0 , 0, 0 , 0 },
												{ 0,0 , 0, 0 , 0 },
												{ 0,0 , 0, 0 , 0 }, 
												{ 0,0 , 0, 0 , 0 }, 
												{ 0,0 , 0, 0 , 0 },
												{ 0,0 , 0, 0 , 0 },
												{ 0,0 , 0, 0 , 0 } } }; // [x/y lines][line#][x1/x2/y1/y2 / BW]
			memcpy(Lines, Lines_,2*16*5*4);
		}
		//else if (nLines_ == NUM_LINES::THREE_LINES)
		//{
		//	nLines = nLines_;
		//	LinesLen[0] = 16;
		//	xR6 = 0.5*SepDist + 1400;
		//	xR7 = 0.5*SepDist + 1500;
		//	xR8 = 0.5*SepDist + 1600;
		//	xOuter = xR8;
		//	// BW label: for xL/R# which align with x axis: BW=0 if LoY is white & HiY is black,       BW=1 if LoY is black & HiY is white
		//	//			 for y#    which align with y axis: BW=0 if LoX is white & HiX is black/mixed, BW=1 if LoX is black/mixed & HiX is white
		//	float32_t Lines_[2][16][5] = { {    {yL0,yL5 , -xR1,-xR1 , 1}, // xL1
		//										{yL0,yL5 , -xR2,-xR2 , 0}, // xL2
		//										{yL0,yL5 , -xR3,-xR3 , 1}, // xL3
		//										{yL0,yL5 , -xR4,-xR4 , 0}, // xL4
		//										{yL0,yL5 , -xR5,-xR5 , 1}, // xL5
		//										{yL0,yL5 , -xR6,-xR6 , 0}, // xL6
		//										{yL0,yL5 , -xR7,-xR7 , 1}, // xL7
		//										{yL0,yL5 , -xR8,-xR8 , 0}, // xL8
		//										 {yL0,yL5 ,  xR1, xR1 , 0}, // xR1
		//										 {yL0,yL5 ,  xR2, xR2 , 1}, // xR2
		//										 {yL0,yL5 ,  xR3, xR3 , 0}, // xR3
		//										 {yL0,yL5 ,  xR4, xR4 , 1}, // xR4
		//										 {yL0,yL5 ,  xR5, xR5 , 0}, // xR5
		//										 {yL0,yL5 ,	xR6, xR6 , 1},  // xR6
		//										 {yL0,yL5 ,	xR7, xR7 , 0},  // xR7
		//										 {yL0,yL5 ,	xR8, xR8 , 1}}, // xR8
		//									  { {yL0,yL0 , -xOuter, xOuter , 0}, // y0
		//										{yL1,yL1 , -xOuter, xOuter , 1}, // y1
		//										{yL2,yL2 , -xOuter, xOuter , 0}, // y2
		//										{yL3,yL3 , -xOuter, xOuter , 1}, // y3
		//										{yL4,yL4 , -xOuter, xOuter , 0}, // y4
		//										{yL5,yL5 , -xOuter, xOuter , 1}, // y5
		//										{ 0,0 , 0, 0 , 0 },
		//										{ 0,0 , 0, 0 , 0 }, 
		//										{ 0,0 , 0, 0 , 0 }, 
		//										{ 0,0 , 0, 0 , 0 },
		//										{ 0,0 , 0, 0 , 0 },
		//										{ 0,0 , 0, 0 , 0 }, 
		//										{ 0,0 , 0, 0 , 0 }, 
		//										{ 0,0 , 0, 0 , 0 },
		//										{ 0,0 , 0, 0 , 0 },
		//										{ 0,0 , 0, 0 , 0 } } }; // [x/y lines][line#][x1/x2/y1/y2 / BW]
		//	memcpy(Lines, Lines_,2*16*5*4);
		//}
		
	};
	
	// Variables
	float32_t SepDist;
	NUM_LINES nLines;
	float32_t yL0, yL1, yL2, yL3, yL4, yL5;
	float32_t xR1, xR2, xR3, xR4, xR5, xR6, xR7, xR8, xOuter;
	float32_t Lines[2][16][5];
	sint32_t LinesLen[2];
	uint32_t CamNum;
	sint32_t yCamSee[4][4]; // Which yLines each camera can see {Front, Left, Rear, Right} => make it empty to not use yLines for that camera
	sint32_t yCamSeeLen[4];
};
#endif