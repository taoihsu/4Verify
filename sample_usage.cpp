/*
 * sample_usage.cpp
 * This file is part of VanPoints
 *
 // This is an implementation of
 // Non-Iterative Approach for Fast and Accurate Vanishing m_Point Detection
 // by Jean-Philippe Tardif

 // --- Modified by David Hsu [24-June-2019]
 // --- Modifications for Ijob Cameron Taylor [July-2019]
 // --- Copyright (c) Magna Vectrics (MEVC) 2019
 //

 */
#ifdef PIKEOS
#include "JobAlgoVpcs.h"
#include "./src/Vpcs_types.h"

#include "logging/Logger.h"
#include "MeclLogging.h"
#include "logging/LogCtx.h"
#include "logging/ILogSender.h"
#include "logging/LogProvider.h"
#include "LogSenderBase.h"

class LogCtxVpcs
{
public:
	static void registerCtxId_v()
	{
		logging::LogProvider v_Lp_o;
		v_Lp_o.registerCtxId_v(LogCtxVpcs::c_LogCtxVpcs_u32);
	}
	static const uint32_t c_LogCtxVpcs_u32 = 0x10000U;
};
#else
#include "sample_usage.h"
#include "VanPoints.h"
#define V6_FLIP
#include "./src/Vpcs_types.h"
//#define NOSHOW_TARGET_LINES
void PrintR__(mecl::core::Matrix<float32_t, 3, 3> &R)
{
	for (int i = 0; i<3; i++)
	{
		vm_cprintf("\n     ");
		for (int j = 0; j<3; j++)
		{
			vm_cprintf("%f, ", R(i, j));
		}
	}
	vm_cprintf("\n");
}
void PrintT__(mecl::core::Matrix<float32_t, 3, 1> &T)
{
	vm_cprintf("[%f, ", T(0, 0));
	vm_cprintf("%f, ", T(1, 0));
	vm_cprintf("%f]\n", T(2, 0));
}
void FILE_PrintR_S(mecl::core::Matrix<float32_t, 3, 3> &R, FILE *fp0)
{
	for (int i = 0; i<3; i++)
	{
		fprintf(fp0, "\n     ");
		for (int j = 0; j<3; j++)
		{
			fprintf(fp0, "%f, ", R(i, j));
		}
	}
	fprintf(fp0, "\n");
}
void FILE_PrintT__(mecl::core::Matrix<float32_t, 3, 1> &T, FILE *fp0)
{
	fprintf(fp0, "[\n%f, \n", T(0, 0));
	fprintf(fp0, "%f, \n", T(1, 0));
	fprintf(fp0, "%f]\n", T(2, 0));
}
/*
void PrintR__(mecl::core::Matrix<float32_t,3,3> &R)
{
	for (int i=0; i<3; i++)
	{
		vm_cprintf("\n     ");
		for (int j=0; j<3; j++)
		{
			vm_cprintf("%f, ",R(i,j));
		}
	}
	vm_cprintf("\n");
}
void PrintT__(mecl::core::Matrix<float32_t,3,1> &T)
{
	vm_cprintf("[%f, ",T(0,0));
	vm_cprintf("%f, ",T(1,0));
	vm_cprintf("%f]\n",T(2,0));
}
*/
#endif


int main()
{
#ifdef WINDOWS 

	
	#ifdef PIKEOS
	logging::ILogSender *b_LogSender_ro;
	static const uint32_t c_LogCtxVPCS_u32 = 0x10000U;
	b_LogSender_ro->init_v(0);
	b_LogSender_ro->registerCtxId_v(c_LogCtxVPCS_u32);
	#endif
#ifndef VPCS3D

	vpcs::DataProviderVpcs i_dataProviderVpcs;
	i_dataProviderVpcs.wfmd.frameID = 0; //No setter, straight from MCU
	i_dataProviderVpcs.setLastRequestedAt_v(0);  
	
	#ifdef PIKEOS
	vpcs::JobAlgoVpcs i_jobVpcs(i_dataProviderVpcs, *b_LogSender_ro);
	#else
	vpcs::JobAlgoVpcs i_jobVpcs(i_dataProviderVpcs );
	#endif
	
	i_dataProviderVpcs.wfmd.responseState = shmdata::ResponseState_e::e_ResponsePositive; //No setter, straight from MCU

	i_jobVpcs.init_v();








	/**************************** Run main VP algorithm for each camera ******************************************/

	#ifdef OPENCV_OUT
		std::vector<MD_Data> DatAll(4);
	#endif
		char dirname[5][250];
		char _szFileName[250];
		char buffer[250];

		FILE *in = NULL;
		strcpy(_szFileName, ".\\Config\\dir_list.txt");

		in = fopen(_szFileName, "r");
		int i = 0;
		while (!feof(in))
		{
			if (fgets(buffer, sizeof(buffer), in))
			{
				//break;

				int erg = sscanf(buffer, "%s", dirname[i]);
				printf(" %s \n", dirname[i]);
			}

			//******log output file
			std::string NowTime;
			time_t now;
			struct tm nowLocal;
			now = time(NULL); // get the time from the OS

			nowLocal = *localtime(&now);
			char tempFn1[250];

			char outputfilename[250] = { "Log.txt" };
			//char tempFn[250];

			strcpy(tempFn1, ".\\Logs\\");
			strcat(tempFn1, dirname[i]);
			
			sprintf(outputfilename, "%s_%2d%2d_%d_%d", tempFn1, nowLocal.tm_mon+1, nowLocal.tm_mday, nowLocal.tm_hour, nowLocal.tm_min);
			strcat(outputfilename, "_log.txt");

			FILE *fp0;
			char filename0[100];

			fp0 = fopen(outputfilename, "w");
			fprintf(fp0, "%s log start ********************\n", outputfilename);

			//cycle time log
			strcpy(tempFn1, ".\\Log_Cycletime\\");
			strcat(tempFn1, dirname[i]);

			sprintf(outputfilename, "%s_%d%d_%d_%d", tempFn1, nowLocal.tm_mon+1, nowLocal.tm_mday, nowLocal.tm_hour, nowLocal.tm_min);
			strcat(outputfilename, "_log.txt");

			FILE *fp1;
			//char filename0[100];

			fp1 = fopen(outputfilename, "w");
	// Front cam (0)
	i_dataProviderVpcs.setMcuCameraId_v(vpcs::e_FrontCamAlgo);
	i_dataProviderVpcs.wfmd.frameID += 1;
	i_dataProviderVpcs.wfmd.RequestedAt = i_dataProviderVpcs.wfmd.frameID;
	i_dataProviderVpcs.wfmd.algoCommand = shmdata::AlgoCommand_e::e_Start;
	const uint8_t *inImage = i_dataProviderVpcs.getInputImage_pu8(dirname[i], fp0);
	uint8_t i_WarpedImage_pu8[vpcs::c_ImageSize_u32];
#ifdef CYCLETIME
	auto t1 = high_resolution_clock::now();
#endif
	i_jobVpcs.start_v(inImage, i_WarpedImage_pu8, dirname[i], fp0);
	//i_jobVpcs.start_v();
	i_jobVpcs.execute_v(dirname[i], fp0);
	i_jobVpcs.end_v(dirname[i], fp0);
	#ifdef OPENCV_OUT
		DatAll[0] = i_jobVpcs.MD_DataC;
		TargetLines CalibTLines = i_jobVpcs.CalibTLinesC;
		const uint8_t *ImDIn = i_dataProviderVpcs.getInputImage_pu8(dirname[i],fp0);
#ifdef V6_FLIP
		{
			for (uint32_t i = 0; i < vpcs::c_ImageWidth_u32; i++)
			{
				for (uint32_t j = 0; j < vpcs::c_ImageHeight_u32; j++)
				{
					DatAll[0].ImD[(vpcs::c_ImageWidth_u32 - 1 - i) + (vpcs::c_ImageHeight_u32 - 1 - j) * vpcs::c_ImageWidth_u32] =
						ImDIn[i + j * vpcs::c_ImageWidth_u32];
				}
			}
		}
#else
		
		{
			for (int i = 0; i < vpcs::c_ImageSize_u32; i++)
			{
				DatAll[0].ImD[i] = ImDIn[i]; // Distorted image

			}
		}
#endif
	#endif
#ifdef CYCLETIME
		auto t2 = high_resolution_clock::now();
		/* Getting number of milliseconds as an integer. */
		fsec fs = t2 - t1;
		ms d = std::chrono::duration_cast<ms>(fs);
		std::cout << "Front cycle time is " << fs.count() << "s\n";
		printf("Camera Front cycle time is %f\n", fs.count());
		fprintf(fp1, "%f for Camera Front cycle time \n",  fs.count());
		std::cout << "Front cycle time is " << d.count() << "ms\n";
#endif	
	// Left cam (1)
	i_dataProviderVpcs.setMcuCameraId_v(vpcs::e_LeftCamAlgo);
	i_dataProviderVpcs.wfmd.frameID += 1;
	i_dataProviderVpcs.wfmd.RequestedAt = i_dataProviderVpcs.wfmd.frameID;
	i_dataProviderVpcs.wfmd.algoCommand = shmdata::AlgoCommand_e::e_Start;
	inImage = i_dataProviderVpcs.getInputImage_pu8(dirname[i],fp0);
#ifdef CYCLETIME
	auto t21 = high_resolution_clock::now();
#endif
	i_jobVpcs.start_v(inImage, i_WarpedImage_pu8, dirname[i], fp0);
	//i_jobVpcs.start_v();
	i_jobVpcs.execute_v(dirname[i], fp0);
	i_jobVpcs.end_v(dirname[i], fp0);
	#ifdef OPENCV_OUT
		DatAll[1] = i_jobVpcs.MD_DataC;
		ImDIn = i_dataProviderVpcs.getInputImage_pu8(dirname[i], fp0);
#ifdef V6_FLIP
		{
			for (uint32_t i = 0; i < vpcs::c_ImageWidth_u32; i++)
			{
				for (uint32_t j = 0; j < vpcs::c_ImageHeight_u32; j++)
				{
					DatAll[1].ImD[(vpcs::c_ImageWidth_u32 - 1 - i) + (vpcs::c_ImageHeight_u32 - 1 - j) * vpcs::c_ImageWidth_u32] =
						ImDIn[i + j * vpcs::c_ImageWidth_u32];
				}
			}
		}
#else
		for (int i=0; i<vpcs::c_ImageSize_u32; i++)
		{
			DatAll[1].ImD[i] = ImDIn[i]; // Distorted image
		}
#endif
	#endif
#ifdef CYCLETIME
		auto t22 = high_resolution_clock::now();

		/* Getting number of milliseconds as an integer. */
		 fs = t22 - t21;
		 d = std::chrono::duration_cast<ms>(fs);
		std::cout << "Left cam is " << fs.count() << "s\n";
		std::cout << "Left cam is " << d.count() << "ms\n";
		fs = (t22 - t21) + (t2 - t1);
		d = std::chrono::duration_cast<ms>(fs);
		std::cout << "Front and Left cycle time is " << fs.count() << "s\n";
		fprintf(fp1, "%f for Camera Front + Left cycle time \n", fs.count());
		
		std::cout << "Front and Left cycle time is " << d.count() << "ms\n";
#endif	

	// rerun new frame since the EOL setup this is based on can only run either start or execute+end each frame
	i_dataProviderVpcs.wfmd.frameID += 1;

	// Rear cam (2)
	i_dataProviderVpcs.setMcuCameraId_v(vpcs::e_RearCamAlgo);
	i_dataProviderVpcs.wfmd.frameID += 1;
	i_dataProviderVpcs.wfmd.RequestedAt = i_dataProviderVpcs.wfmd.frameID;
	i_dataProviderVpcs.wfmd.algoCommand = shmdata::AlgoCommand_e::e_Start;
	inImage = i_dataProviderVpcs.getInputImage_pu8(dirname[i], fp0);
#ifdef CYCLETIME
	auto t31 = high_resolution_clock::now();
#endif
	i_jobVpcs.start_v(inImage, i_WarpedImage_pu8, dirname[i], fp0);
	//i_jobVpcs.start_v();
	i_jobVpcs.execute_v(dirname[i], fp0);
	i_jobVpcs.end_v(dirname[i], fp0);
	#ifdef OPENCV_OUT
		DatAll[2] = i_jobVpcs.MD_DataC;
		ImDIn = i_dataProviderVpcs.getInputImage_pu8(dirname[i], fp0);
#ifdef V6_FLIP
		{
			for (uint32_t i = 0; i < vpcs::c_ImageWidth_u32; i++)
			{
				for (uint32_t j = 0; j < vpcs::c_ImageHeight_u32; j++)
				{
					DatAll[2].ImD[(vpcs::c_ImageWidth_u32 - 1 - i) + (vpcs::c_ImageHeight_u32 - 1 - j) * vpcs::c_ImageWidth_u32] =
						ImDIn[i + j * vpcs::c_ImageWidth_u32];
				}
			}
		}
#else
		for (int i=0; i<vpcs::c_ImageSize_u32; i++)
		{
			DatAll[2].ImD[i] = ImDIn[i]; // Distorted image
		}
#endif
	#endif
#ifdef CYCLETIME
		auto t32 = high_resolution_clock::now();

		/* Getting number of milliseconds as an integer. */
		fs = t32 - t31;
		d = std::chrono::duration_cast<ms>(fs);
		std::cout << "Left cam is " << fs.count() << "s\n";
		std::cout << "Left cam is " << d.count() << "ms\n";
		fs = (t32 - t31) + (t22-t21) + (t2 - t1);
		d = std::chrono::duration_cast<ms>(fs);
		std::cout << "Front and Left and Rear cycle time is " << fs.count() << "s\n";
		fprintf(fp1, "%f for Camera Front + Left + Rear cycle time \n", fs.count());
		
		std::cout << "Front and Left and Rear cycle time is " << d.count() << "ms\n";
#endif	

	// Right cam (3)
	i_dataProviderVpcs.setMcuCameraId_v(vpcs::e_RightCamAlgo);
	i_dataProviderVpcs.wfmd.frameID += 1;
	i_dataProviderVpcs.wfmd.RequestedAt = i_dataProviderVpcs.wfmd.frameID;
	i_dataProviderVpcs.wfmd.algoCommand = shmdata::AlgoCommand_e::e_Start;
	inImage = i_dataProviderVpcs.getInputImage_pu8(dirname[i], fp0);
#ifdef CYCLETIME
	auto t41 = high_resolution_clock::now();
#endif
	i_jobVpcs.start_v(inImage, i_WarpedImage_pu8, dirname[i], fp0);
	//i_jobVpcs.start_v();
	i_jobVpcs.execute_v(dirname[i], fp0);
	i_jobVpcs.end_v(dirname[i], fp0);
	#ifdef OPENCV_OUT
		DatAll[3] = i_jobVpcs.MD_DataC;
		ImDIn = i_dataProviderVpcs.getInputImage_pu8(dirname[i], fp0);
		//for (int i=0; i<vpcs::c_ImageSize_u32; i++)
		//{
		//	DatAll[3].ImD[i] = ImDIn[i]; // Distorted image
		//}
		//if (i == 3)
#ifdef V6_FLIP
		for (int i=0; i<vpcs::c_ImageSize_u32; i++)
		{
			DatAll[2].ImD[i] = ImDIn[i]; // Distorted image
		}
#else
		{
			for (uint32_t i = 0; i < vpcs::c_ImageWidth_u32; i++)
			{
				for (uint32_t j = 0; j < vpcs::c_ImageHeight_u32; j++)
				{
					DatAll[3].ImD[(vpcs::c_ImageWidth_u32 - 1 - i) + (vpcs::c_ImageHeight_u32 - 1 - j) * vpcs::c_ImageWidth_u32] =
						ImDIn[i + j * vpcs::c_ImageWidth_u32];
				}
			}
		}
#endif
	#endif
#ifdef CYCLETIME
		auto t42 = high_resolution_clock::now();

		/* Getting number of milliseconds as an integer. */
		fs = t42 - t41;
		d = std::chrono::duration_cast<ms>(fs);
		std::cout << "Left cam is " << fs.count() << "s\n";
		std::cout << "Left cam is " << d.count() << "ms\n";
		fs = (t42 - t41) + (t32 - t31) + (t22 - t21) + (t2 - t1);
		d = std::chrono::duration_cast<ms>(fs);
		std::cout << "Front and Left and Rear and Right cycle time is " << fs.count() << "s\n";
		
		fprintf(fp1, "%f for Camera Front + Left + Rear + Right cycle time \n", fs.count());
		std::cout << "Front and Left and Rear and Right cycle time is " << d.count() << "ms\n";
#endif
#ifdef CYCLETIME
		auto t4 = high_resolution_clock::now();

		/* Getting number of milliseconds as an integer. */
		fs = t4 - t1;
		d = std::chrono::duration_cast<ms>(fs);
		std::cout << "Total Four cam is " << fs.count() << "s\n";
   		std::cout << "Total Four cam is " << d.count() << "ms\n";
#endif	
	
		

	/**************************** Figures ******************************************/
	#ifdef OPENCV_OUT
		// Plot each image with sorted lines on top
		if (1==0)
		{
			for (int c=0; c<4; c++)
			{
				Fig_DispLines(DatAll[c], true); // Distorted image
				Fig_DispLines(DatAll[c], false); // Undistorted image
			}
		}

		// Plot Bird's eye with all cameras: 2D VP's
		//Fig_BirdsEyeAll(DatAll, CalibTLines, false, 2); // Use undistorted image + extrinsics from 2D VP's
		//Fig_BirdsEyeAll(DatAll, CalibTLines, true,  2); // Use distorted image   + extrinsics from 2D VP's

		// Plot Bird's eye with all cameras: 3D VP's
#ifdef PREWARPED
		Fig_BirdsEyeAll(DatAll, CalibTLines, false, 3); // Use undistorted image + extrinsics from 3D VP's
		Fig_BirdsEyeAll(DatAll, CalibTLines, false, 1); // Use distorted image + refined extrinsics
#else

//// Print out
vm_cprintf("Cam 0 Orig_R_A_C:"); PrintR__(DatAll[0].Orig_R_A_C);
vm_cprintf("Cam 1 Orig_R_A_C:"); PrintR__(DatAll[1].Orig_R_A_C);
vm_cprintf("Cam 2 Orig_R_A_C:"); PrintR__(DatAll[2].Orig_R_A_C);
vm_cprintf("Cam 3 Orig_R_A_C:"); PrintR__(DatAll[3].Orig_R_A_C);

vm_cprintf("Cam 0 Orig_R_A_C reverse:"); PrintR__(DatAll[0].Orig_R_A_C.t());
vm_cprintf("Cam 1 Orig_R_A_C reverse:"); PrintR__(DatAll[1].Orig_R_A_C.t());
vm_cprintf("Cam 2 Orig_R_A_C reverse:"); PrintR__(DatAll[2].Orig_R_A_C.t());
vm_cprintf("Cam 3 Orig_R_A_C reverse:"); PrintR__(DatAll[3].Orig_R_A_C.t());

		vm_cprintf("Cam 0 Orig_xyz:"); PrintT__(DatAll[0].Orig_xyz_CamPos);
		vm_cprintf("Cam 1 Orig_xyz:"); PrintT__(DatAll[1].Orig_xyz_CamPos);
		vm_cprintf("Cam 2 Orig_xyz:"); PrintT__(DatAll[2].Orig_xyz_CamPos);
		vm_cprintf("Cam 3 Orig_xyz:"); PrintT__(DatAll[3].Orig_xyz_CamPos);

		vm_cprintf("Cam 0 Orig_pyr:"); PrintT__(DatAll[0].Orig_pyr_CamPos* 180 / M_PI);
		vm_cprintf("Cam 1 Orig_pyr:"); PrintT__(DatAll[1].Orig_pyr_CamPos* 180 / M_PI);
		vm_cprintf("Cam 2 Orig_pyr:"); PrintT__(DatAll[2].Orig_pyr_CamPos* 180 / M_PI);
		vm_cprintf("Cam 3 Orig_pyr:"); PrintT__(DatAll[3].Orig_pyr_CamPos* 180 / M_PI);
#if HardCoded
		DatAll[0].Refined_pyr_CamPos = { 51.999359*M_PI / 180,
			0.038417*M_PI / 180,
			-0.641775 *M_PI / 180 };
		DatAll[1].Refined_pyr_CamPos = { 34.186325*M_PI / 180,
			1.090297*M_PI / 180,
			0.890953*M_PI / 180 };
		DatAll[2].Refined_pyr_CamPos = { 49.775867*M_PI / 180,
			-0.198067*M_PI / 180,
			0.426731*M_PI / 180 };
		DatAll[3].Refined_pyr_CamPos = { 32.412304*M_PI / 180,
			-0.397155*M_PI / 180,
			-178.924850*M_PI / 180 };
		ExRefine ECalib;
		for (int camID = 0; camID<4; camID++)
			DatAll[camID].Refined_R_A_C = ECalib.get_R_C_A(DatAll[camID].Refined_pyr_CamPos, camID, fp0);

		DatAll[0].Refined_xyz_CamPos = { -883.954590,
			7.161889,
			940.011292 };
		DatAll[1].Refined_xyz_CamPos = { 733.063416,
			-1092.284180,
			1281.800537 };
		DatAll[2].Refined_xyz_CamPos = { 4626.260254,
			-14.535686,
			1032.556519 };
		DatAll[3].Refined_xyz_CamPos = { 764.655518,
			1082.549438,
			1295.538940 };
#endif
		vm_cprintf("Cam 0 Refined_pyr:"); PrintT__(DatAll[0].Refined_pyr_CamPos*180/M_PI);
		vm_cprintf("Cam 1 Refined_pyr:"); PrintT__(DatAll[1].Refined_pyr_CamPos* 180 / M_PI);
		vm_cprintf("Cam 2 Refined_pyr:"); PrintT__(DatAll[2].Refined_pyr_CamPos* 180 / M_PI);
		vm_cprintf("Cam 3 Refined_pyr:"); PrintT__(DatAll[3].Refined_pyr_CamPos* 180 / M_PI);

vm_cprintf("Cam 0 Refined_xyz:"); PrintT__(DatAll[0].Refined_xyz_CamPos);
vm_cprintf("Cam 1 Refined_xyz:"); PrintT__(DatAll[1].Refined_xyz_CamPos);
vm_cprintf("Cam 2 Refined_xyz:"); PrintT__(DatAll[2].Refined_xyz_CamPos);
vm_cprintf("Cam 3 Refined_xyz:"); PrintT__(DatAll[3].Refined_xyz_CamPos);
fprintf(fp0,"Cam 0 Refined_xyz:"); FILE_PrintT__(DatAll[0].Refined_xyz_CamPos,fp0);
fprintf(fp0,"Cam 1 Refined_xyz:"); FILE_PrintT__(DatAll[1].Refined_xyz_CamPos, fp0);
fprintf(fp0,"Cam 2 Refined_xyz:"); FILE_PrintT__(DatAll[2].Refined_xyz_CamPos, fp0);
fprintf(fp0,"Cam 3 Refined_xyz:"); FILE_PrintT__(DatAll[3].Refined_xyz_CamPos, fp0);
fprintf(fp0,"Cam 0 Orig_R_A_C:"); FILE_PrintR_S(DatAll[0].Orig_R_A_C, fp0);
fprintf(fp0,"Cam 1 Orig_R_A_C:"); FILE_PrintR_S(DatAll[1].Orig_R_A_C, fp0);
fprintf(fp0,"Cam 2 Orig_R_A_C:"); FILE_PrintR_S(DatAll[2].Orig_R_A_C, fp0);
fprintf(fp0,"Cam 3 Orig_R_A_C:"); FILE_PrintR_S(DatAll[3].Orig_R_A_C, fp0);

fprintf(fp0,"Cam 0 Orig_R_A_C reverse:"); FILE_PrintR_S(DatAll[0].Orig_R_A_C.t(), fp0);
fprintf(fp0,"Cam 1 Orig_R_A_C reverse:"); FILE_PrintR_S(DatAll[1].Orig_R_A_C.t(), fp0);
fprintf(fp0,"Cam 2 Orig_R_A_C reverse:"); FILE_PrintR_S(DatAll[2].Orig_R_A_C.t(), fp0);
fprintf(fp0,"Cam 3 Orig_R_A_C reverse:"); FILE_PrintR_S(DatAll[3].Orig_R_A_C.t(), fp0);

fprintf(fp0,"Cam 0 Orig_xyz:"); FILE_PrintT__(DatAll[0].Orig_xyz_CamPos, fp0);
fprintf(fp0,"Cam 1 Orig_xyz:"); FILE_PrintT__(DatAll[1].Orig_xyz_CamPos, fp0);
fprintf(fp0,"Cam 2 Orig_xyz:"); FILE_PrintT__(DatAll[2].Orig_xyz_CamPos, fp0);
fprintf(fp0,"Cam 3 Orig_xyz:"); FILE_PrintT__(DatAll[3].Orig_xyz_CamPos, fp0);

fprintf(fp0,"Cam 0 Orig_pyr:"); FILE_PrintT__(DatAll[0].Orig_pyr_CamPos * 180 / M_PI, fp0);
fprintf(fp0,"Cam 1 Orig_pyr:"); FILE_PrintT__(DatAll[1].Orig_pyr_CamPos * 180 / M_PI, fp0);
fprintf(fp0,"Cam 2 Orig_pyr:"); FILE_PrintT__(DatAll[2].Orig_pyr_CamPos * 180 / M_PI, fp0);
fprintf(fp0,"Cam 3 Orig_pyr:"); FILE_PrintT__(DatAll[3].Orig_pyr_CamPos * 180 / M_PI, fp0);

fprintf(fp0,"Cam 0 Refined_pyr:"); FILE_PrintT__(DatAll[0].Refined_pyr_CamPos * 180 / M_PI, fp0);
fprintf(fp0,"Cam 1 Refined_pyr:"); FILE_PrintT__(DatAll[1].Refined_pyr_CamPos * 180 / M_PI, fp0);
fprintf(fp0,"Cam 2 Refined_pyr:"); FILE_PrintT__(DatAll[2].Refined_pyr_CamPos * 180 / M_PI, fp0);
fprintf(fp0,"Cam 3 Refined_pyr:"); FILE_PrintT__(DatAll[3].Refined_pyr_CamPos * 180 / M_PI, fp0);

fprintf(fp0,"Cam 0 Refined_xyz:"); FILE_PrintT__(DatAll[0].Refined_xyz_CamPos, fp0);
fprintf(fp0,"Cam 1 Refined_xyz:"); FILE_PrintT__(DatAll[1].Refined_xyz_CamPos, fp0);
fprintf(fp0,"Cam 2 Refined_xyz:"); FILE_PrintT__(DatAll[2].Refined_xyz_CamPos, fp0);
fprintf(fp0,"Cam 3 Refined_xyz:"); FILE_PrintT__(DatAll[3].Refined_xyz_CamPos, fp0);
		// Reverse Euler angle order
		ExRefine ExR;
		//for (int c=0; c<4; c++)
		//{
		//	DatAll[c].Orig_RAdjust = ExR.pyr_to_R3_Flip(DatAll[c].Orig_pyr_CamPos); // Instead of pyr_to_R3
		//	DatAll[c].Orig_R_A_C = DatAll[c].R_A_C0.mmul(DatAll[c].Orig_RAdjust);
		//}
		//vm_cprintf("Cam 0 Orig_R_A_C reverse:"); PrintR__(DatAll[0].Orig_R_A_C.t());
		//vm_cprintf("Cam 1 Orig_R_A_C reverse:"); PrintR__(DatAll[1].Orig_R_A_C.t());
		//vm_cprintf("Cam 2 Orig_R_A_C reverse:"); PrintR__(DatAll[2].Orig_R_A_C.t());
		//vm_cprintf("Cam 3 Orig_R_A_C reverse:"); PrintR__(DatAll[3].Orig_R_A_C.t());

		// Reverse Euler angle order
		mecl::core::Matrix<float32_t,3,3> N1 = mecl::core::Matrix<float32_t,3,3>::zeros_x();
		N1(0,0) = 1;
		N1(1,1) = -1;
		N1(2,2) = -1;
		mecl::core::Matrix<float32_t,3,3> N2 = mecl::core::Matrix<float32_t,3,3>::zeros_x();
		N2(0,0) = -1;
		N2(1,1) = -1;
		N2(2,2) = 1;
		mecl::core::Matrix<float32_t,3,3> N3 = mecl::core::Matrix<float32_t,3,3>::zeros_x();
		N3(0,0) = 1;
		N3(1,1) = -1;
		N3(2,2) = 1;
		mecl::core::Matrix<float32_t,3,3> N4 = mecl::core::Matrix<float32_t,3,3>::zeros_x();
		N4(0,0) = 1;
		N4(1,1) = 1;
		N4(2,2) = -1;
		mecl::core::Matrix<float32_t,3,3> N5 = mecl::core::Matrix<float32_t,3,3>::zeros_x();
		N5(0,0) = -1;
		N5(1,1) = 1;
		N5(2,2) = -1;
		for (int c=0; c<4; c++)
		{
			DatAll[c].Orig_RAdjust = ExR.pyr_to_R3_Flip(DatAll[c].Orig_pyr_CamPos); // Instead of pyr_to_R3
			mecl::core::Matrix<float32_t,3,3> R_A_C0_Matlab = DatAll[c].R_A_C0.mmul(N1);
			mecl::core::Matrix<float32_t,3,3> R_A_C_Matlab = R_A_C0_Matlab.mmul(DatAll[c].Orig_RAdjust);
			vm_cprintf("DatAll[%d].Orig_RAdjust:",c); PrintR__(DatAll[c].Orig_RAdjust);
			vm_cprintf("R_A_C0:"); PrintR__(DatAll[c].R_A_C0);
			vm_cprintf("R_A_C0_Matlab:"); PrintR__(R_A_C0_Matlab);
			vm_cprintf("R_C_A_Matlab:"); PrintR__(R_A_C_Matlab);
			vm_cprintf("R_C_A_Matlab inverse:"); PrintR__(R_A_C_Matlab.t());

			fprintf(fp0,"DatAll[%d].Orig_RAdjust:", c); FILE_PrintR_S(DatAll[c].Orig_RAdjust,fp0);
			fprintf(fp0,"R_A_C0:"); FILE_PrintR_S(DatAll[c].R_A_C0, fp0);
			fprintf(fp0,"R_A_C0_Matlab:"); FILE_PrintR_S(R_A_C0_Matlab, fp0);
			fprintf(fp0,"R_C_A_Matlab:"); FILE_PrintR_S(R_A_C_Matlab, fp0);
			fprintf(fp0,"R_C_A_Matlab inverse:"); FILE_PrintR_S(R_A_C_Matlab.t(), fp0);
			if (c==0 || c==2)
				DatAll[c].Orig_R_A_C = N1.mmul(R_A_C_Matlab.mmul(N2));
			else if (c==1)
				DatAll[c].Orig_R_A_C = N3.mmul(R_A_C_Matlab.mmul(N4));
			else if (c==3)
				DatAll[c].Orig_R_A_C = N5.mmul(R_A_C_Matlab);
		}
		vm_cprintf("Cam 0 Orig_R_C_A reverse:"); PrintR__(DatAll[0].Orig_R_A_C.t());
		vm_cprintf("Cam 1 Orig_R_C_A reverse:"); PrintR__(DatAll[1].Orig_R_A_C.t());
		vm_cprintf("Cam 2 Orig_R_C_A reverse:"); PrintR__(DatAll[2].Orig_R_A_C.t());
		vm_cprintf("Cam 3 Orig_R_C_A reverse:"); PrintR__(DatAll[3].Orig_R_A_C.t());

		fprintf(fp0,"Cam 0 Orig_R_C_A reverse:"); FILE_PrintR_S(DatAll[0].Orig_R_A_C.t(), fp0);
		fprintf(fp0,"Cam 1 Orig_R_C_A reverse:"); FILE_PrintR_S(DatAll[1].Orig_R_A_C.t(), fp0);
		fprintf(fp0,"Cam 2 Orig_R_C_A reverse:"); FILE_PrintR_S(DatAll[2].Orig_R_A_C.t(), fp0);
		fprintf(fp0,"Cam 3 Orig_R_C_A reverse:"); FILE_PrintR_S(DatAll[3].Orig_R_A_C.t(), fp0);

		for (int c=0; c<4; c++)
		{
			DatAll[c].Orig_R_A_C = ExR.get_R_C_A(DatAll[c].Orig_pyr_CamPos, c, fp0);
			mecl::core::Matrix<float32_t,3,1> r0 = ExR.R3_to_pyr_Flip(DatAll[c].Orig_R_A_C);
			mecl::core::Matrix<float32_t,3,3> R1 = ExR.pyr_to_R3_Flip(r0);
			mecl::core::Matrix<float32_t,3,1> r1 = ExR.R3_to_pyr_Flip(R1);
			vm_cprintf("Check1:"); PrintT__(r0); PrintT__(r1); PrintR__(DatAll[c].Orig_R_A_C-R1);
			fprintf(fp0,"Check1:"); FILE_PrintT__(r0,fp0); FILE_PrintT__(r1,fp0); FILE_PrintR_S(DatAll[c].Orig_R_A_C - R1,fp0);
#ifndef V6_FLIP
			mecl::core::Matrix<float32_t,3,1> r2 = ExR.get_pyr(DatAll[c].Orig_R_A_C, c,fp0);
#else
			double pitch, yaw, roll;
			mecl::core::Matrix<float32_t, 3, 1> r2 = ExR.get_pyr(DatAll[c].Orig_R_A_C, pitch, yaw, roll, c, fp0);
#endif
			vm_cprintf("Check2:"); PrintT__(r2); PrintT__(DatAll[c].Orig_pyr_CamPos*180.0/M_PI);
			fprintf(fp0, "Check2:"); FILE_PrintT__(r2, fp0); FILE_PrintT__(DatAll[c].Orig_pyr_CamPos*180.0/M_PI, fp0);
		}


		Fig_BirdsEyeAll(DatAll, CalibTLines, true, 0);
		Fig_BirdsEyeAll(DatAll, CalibTLines, true, 3); // Use distorted image + extrinsics from 3D VP's													// Plot Bird's eye with all cameras with refined extrinsics
		Fig_BirdsEyeAll(DatAll, CalibTLines, true, 1); // Use distorted image + refined extrinsics
#endif
		
	#endif

#endif

#endif
		fprintf(fp0, "log end **********************\n");
		fclose(fp0);
		fclose(fp1);
		i++;
		}
		fclose(in);
    return 0;
	
}



/************************************************************* Figures ******************************************************************/

// X-Y plane (XYP) image (z=0)
#ifdef OPENCV_OUT
//#ifdef SHOW_DEBUG_IMAGE
void Fig_BirdsEyeAll(std::vector<MD_Data> &DatAll, TargetLines &CalibTLines, bool ImDistorted, int ExtrinsicsChoose)
// ExtrinsicsChoose: 0=Original, 1=FinalSoln(refined), 2=2D VP, 3=3D VP
{
	// Settings
	float32_t XYP_xMin =  CalibTLines.yL0 - 1500;
	float32_t XYP_xMax =  CalibTLines.yL5 + 1500;
	float32_t XYP_yMin = -CalibTLines.xR6 - 1500;
	float32_t XYP_yMax =  CalibTLines.xR6 + 1500;
	float32_t XYP_Res = 14;
	float32_t CenterAxisLen = 200;
	vm_cprintf("XYP_xMin:%f XYP_xMax: %f XYP_yMin: %f XYP_yMax: %f\n", XYP_xMin, XYP_xMax, XYP_yMin, XYP_yMax);
	// Setup
	int nC = DatAll.size(); // Number of cameras
	int ImRows = vpcs::c_ImageHeight_u32;
	int ImCols = vpcs::c_ImageWidth_u32;
	int XYP_cols = (sint32_t)((XYP_xMax-XYP_xMin)/XYP_Res + 0.5f);
	int XYP_rows = (sint32_t)((XYP_yMax-XYP_yMin)/XYP_Res + 0.5f);
	cv::Mat XYP_Im0 = 255*cv::Mat::ones(XYP_rows,XYP_cols, CV_8UC1);
	cv::Scalar Col;
	int LineWidth;
	cv::Point Pt0, Pt1, Pt2;
	float32_t x0, x1, x2, y0, y1, y2;
	mecl::core::Matrix<float32_t,3,3> R_C_A[vpcs::LINES_SIZE_MAX];
	mecl::core::Matrix<float32_t,3,1> T_A_C_A[vpcs::LINES_SIZE_MAX];
	vm_cprintf("Cam 0 Orig_R_C_A reverse:"); PrintR__(DatAll[0].Orig_R_A_C.t());
	vm_cprintf("Cam 1 Orig_R_C_A reverse:"); PrintR__(DatAll[1].Orig_R_A_C.t());
	vm_cprintf("Cam 2 Orig_R_C_A reverse:"); PrintR__(DatAll[2].Orig_R_A_C.t());
	vm_cprintf("Cam 3 Orig_R_C_A reverse:"); PrintR__(DatAll[3].Orig_R_A_C.t());
	std::string ExtrinsicFile;
	if (ExtrinsicsChoose == 0) // Use original extrinsics
	{
		for (int c=0; c<nC; c++)
		{
			R_C_A[c] = DatAll[c].Orig_R_A_C.t();
			for (int i=0; i<3; i++)
				T_A_C_A[c](i) = DatAll[c].Orig_xyz_CamPos(i);
			vm_cprintf("Check1: T_A_C_A[%d]", c); PrintT__(T_A_C_A[c]);
		}
		ExtrinsicFile = "Orig";
	}
	else if (ExtrinsicsChoose == 1) // Use final (refined) extrinsics (R & T)
	{
		for (int c=0; c<nC; c++)
		{
			
			R_C_A[c] = DatAll[c].Refined_R_A_C.t();
		
			for (int i=0; i<3; i++)
				T_A_C_A[c](i) = DatAll[c].Refined_xyz_CamPos(i);
		}
		ExtrinsicFile = "Final";
	}
#ifdef VPCS2D
	else if (ExtrinsicsChoose == 2) // Use 2D VP extrinsics (R & T)
	{
		for (int c=0; c<nC; c++)
		{
			R_C_A[c] = DatAll[c].VP2D_R_A_C.t();
			for (int i=0; i<3; i++)
				T_A_C_A[c](i) = DatAll[c].VP2D_xyz_CamPos(i);
		}
		ExtrinsicFile = "VP2d";
	}
#endif
	else if (ExtrinsicsChoose == 3) // Use 3D VP extrinsics (R & T)
	{
		for (int c=0; c<nC; c++)
		{
			R_C_A[c] = DatAll[c].VP3D_R_A_C.t();
			for (int i=0; i<3; i++)
				T_A_C_A[c](i) = DatAll[c].Orig_xyz_CamPos(i);
		}
		ExtrinsicFile = "VP3d";
	}
	else // Not valid
	{
		return;
	}

	// Project x-y plane in cAr Axle RF (A) into camera image => display nearest pixel
	float32_t r2[vpcs::c_ImageHeight_u32][vpcs::c_ImageWidth_u32] = { mecl::math::numeric_limits<float64_t>::infinity_x() }; // r^2 best distance to center of camera to choose best camera image to use
	memset(r2, 0x70, vpcs::c_ImageSize_u32 * 4);
	mecl::core::Matrix<float32_t,1,1> r2_;
	mecl::core::Matrix<float32_t,2,1> uv_C;
	for (int c=0; c<nC; c++)
	{
		vm_cprintf("Cam %d: Dist_W2I = [%f, %f, %f, %f, %f, %f] \n", c, DatAll[c].Dist_world2im[0], DatAll[c].Dist_world2im[1],
			DatAll[c].Dist_world2im[2], DatAll[c].Dist_world2im[3], DatAll[c].Dist_world2im[4], DatAll[c].Dist_world2im[5]);
		vm_cprintf("B4 Project3D_to_2D: R_C_A[c]"); PrintR__(R_C_A[c]);
		vm_cprintf("B4 Project3D_to_2D: T_A_C_A[c]"); PrintT__(T_A_C_A[c]);
		for (int i=0; i<XYP_rows; i++)
		{
			for (int j=0; j<XYP_cols; j++)
			{
				// xyz_A = cAr Axle reference frame
				mecl::core::Matrix<float32_t,3,1> xyz_A;
				xyz_A(0) = j * XYP_Res + XYP_xMin;
				xyz_A(1) = i * XYP_Res + XYP_yMin;
				xyz_A(2) = 0;
				// xyz_A => xyz_C = cam reference frame
				mecl::core::Matrix<float32_t,3,1> xyz_C = R_C_A[c].mmul(xyz_A.add(-T_A_C_A[c]));
				if (xyz_C(2)>0)
				{
					if (ImDistorted)
					{
						uv_C = Project3D_to_2D(xyz_C, DatAll[c], 1, r2_);
					}
					else
					{
						uv_C = Project3D_to_2D(xyz_C, DatAll[c], 0, r2_);
					}
					int col = (sint32_t)( uv_C(0) + 0.5f);
					int row = (sint32_t)( uv_C(1) + 0.5f);
					if (col>=0 && row>=0 && col<ImCols && row<ImRows)
					{
						if (r2_(0) < r2[i][j])
						{
							r2[i][j] = r2_(0);
							if (ImDistorted)
								XYP_Im0.at<uint8_t>(i,j) = DatAll[c].ImD[row*ImCols+col];
							else
								XYP_Im0.at<uint8_t>(i,j) = DatAll[c].ImUD[row*ImCols+col];
						}
					}
				}
			}
		}
		imshow("XYP_Im0Figure", XYP_Im0);
		cv::waitKey(0);
	}

	// Convert image to color
	cv::Mat XYP_Im3;
	cv::cvtColor(XYP_Im0, XYP_Im3, cv::COLOR_GRAY2BGR);

	// Plot SVC's
	for (int c=0; c<nC; c++)
	{
		Col = cv::Scalar(255,0,0); // Blue
		x0 = T_A_C_A[c](0);
		y0 = T_A_C_A[c](1);
		Pt0 = cv::Point((x0-XYP_xMin)/XYP_Res, (y0-XYP_yMin)/XYP_Res);
		cv::drawMarker(XYP_Im3, Pt0, Col, cv::MARKER_DIAMOND, 5, 2, cv::LINE_AA);
		if (c==0)
			cv::putText(XYP_Im3, "Front", Pt0, cv::FONT_HERSHEY_DUPLEX, 1.0, Col, 1.5);
		else if (c==1)
			cv::putText(XYP_Im3, "Left",  Pt0, cv::FONT_HERSHEY_DUPLEX, 1.0, Col, 1.5);
		else if (c==2)
			cv::putText(XYP_Im3, "Rear",  Pt0, cv::FONT_HERSHEY_DUPLEX, 1.0, Col, 1.5);
		else
			cv::putText(XYP_Im3, "Right", Pt0, cv::FONT_HERSHEY_DUPLEX, 1.0, Col, 1.5);
	}

	// Plot x/y axis
	Col = cv::Scalar(0,0,0); // Black
	LineWidth = 2;
	x0 = 0;
	x1 = 0;
	x2 = CenterAxisLen;
	y0 = 0;
	y1 = CenterAxisLen;
	y2 = 0;
	Pt0 = cv::Point((x0-XYP_xMin)/XYP_Res, (y0-XYP_yMin)/XYP_Res);
	Pt1 = cv::Point((x1-XYP_xMin)/XYP_Res, (y1-XYP_yMin)/XYP_Res);
	Pt2 = cv::Point((x2-XYP_xMin)/XYP_Res, (y2-XYP_yMin)/XYP_Res);
	cv::line(XYP_Im3, Pt0, Pt1, Col, LineWidth, cv::LINE_AA);
	cv::line(XYP_Im3, Pt0, Pt2, Col, LineWidth, cv::LINE_AA);
#ifndef NOSHOW_TARGET_LINES
	// Plot calibration target lines
	LineWidth = 1;
	for (int i=0; i<2; i++)
	{
		if (i==0)
			Col = cv::Scalar(0,0,255); // Red
		else
			Col = cv::Scalar(0,255,0); // Green
		for (int j=0; j<CalibTLines.LinesLen[i]; j++)
		{
			x1 = CalibTLines.Lines[i][j][0];
			x2 = CalibTLines.Lines[i][j][1];
			y1 = CalibTLines.Lines[i][j][2];
			y2 = CalibTLines.Lines[i][j][3];
			Pt1 = cv::Point((x1-XYP_xMin)/XYP_Res, (y1-XYP_yMin)/XYP_Res);
			Pt2 = cv::Point((x2-XYP_xMin)/XYP_Res, (y2-XYP_yMin)/XYP_Res);
			cv::line(XYP_Im3, Pt1, Pt2, Col, LineWidth, cv::LINE_AA);
		}
	}
#endif

	// Display image
	cv::flip(XYP_Im3, XYP_Im3, 0); // Flip image vertically
	if ((ExtrinsicsChoose == 1) || (ExtrinsicsChoose == 3))
	{
		cv::imshow("MD_Im2", XYP_Im3);
		cvWaitKey(0);
	}
	// Save figure
	cv::imwrite("debug.png", XYP_Im3);
	std::string ImageFile = ".\\Images\\Debug\\BirdsEye_" + ExtrinsicFile + ".png";
	cv::imwrite(ImageFile, XYP_Im3);
}
#endif




// Display image with sorted lines on top
#ifdef OPENCV_OUT
void Fig_DispLines(MD_Data &DatCam, bool ImDistorted)
{
	// Settings
	int LineWidth = 1;

	// Setup
	int ImRows = vpcs::c_ImageHeight_u32;
	int ImCols = vpcs::c_ImageWidth_u32;
	cv::Scalar Col;
	float32_t x1, x2, y1, y2;
	// Load image
	cv::Mat Im3;
	cv::Mat Im0(ImRows, ImCols, CV_8UC1);

	
	for (int row=0; row<ImRows; row++)
	{
		for (int col=0; col<ImCols; col++)
		{
			if (ImDistorted)
				Im0.at<uint8_t>(row,col) = DatCam.ImD[row*ImCols+col];
			else
				Im0.at<uint8_t>(row,col) = DatCam.ImUD[row*ImCols+col];
		
				
		}
	}
	cv::cvtColor(Im0, Im3, CV_GRAY2BGR);

	// Draw lines
	if (!ImDistorted)
	{
		int nL = DatCam.numLinLab_s32;
		for (int i=0; i<nL; i++)
		{
			if (DatCam.Lines_Lab[i] == 0)
				Col = cv::Scalar(0,0,255); // Red
			else if (DatCam.Lines_Lab[i] == 1)
				Col = cv::Scalar(0,255,0); // Green
			else if (DatCam.Lines_Lab[i] == 2)
				Col = cv::Scalar(255,0,0); // Blue
			else
				Col = cv::Scalar(0,255,255); // Yellow
			x1 = DatCam.Lines_EndPts[i][0];
			x2 = DatCam.Lines_EndPts[i][1];
			y1 = DatCam.Lines_EndPts[i][2];
			y2 = DatCam.Lines_EndPts[i][3];
			cv::line(Im3, cv::Point(x1,y1), cv::Point(x2,y2), Col, LineWidth, cv::LINE_AA);
		}
	}

	//// Display image
	//cv::imshow("MD_Im1", Im3);
	//cvWaitKey(0);
}


#endif
// Error plot
//#ifdef OPENCV_OUT
#if 0
        bool PltInit = true;
        float32_t LineLen1 = 200;
        float32_t LineLen2 = 200;
        cv::Mat Im3;
        uint32_t ImRows = vpcs::c_ImageHeight_u32;
        uint32_t ImCols = vpcs::c_ImageWidth_u32;
		cv::Mat Im0(ImRows, ImCols, CV_8UC1); 

		//cv::cvtColor(Im00, Im03, CV_GRAY2BGR);
        cv::cvtColor(Im0, Im3, CV_GRAY2BGR);
        cv::Scalar Col;
        int LineWidth =1;
        float32_t x1, x2, y1, y2;
        int nLMx = 0;
        int nLMy = 0;
#if 0
               for (int i=0; i<2; i++) // x/y
               {
                    int nTL = CalibTLines.LinesLen[i];
                    for (uint32_t j=0; j<Matches[i].size_u32(); j++) // For each CalibTLine
                    {
                    int DL = Matches[i][j];
                    if (DL < 0)
                                continue;
                    if (i==0) nLMx++;
                    if (i==1) nLMy++;
                    int nCol = 6;
                    int TLcol = j;
                    if (TLcol >= nTL/2)
                                TLcol -= nTL/2;
                    if (TLcol >= nCol)
                                TLcol -= nCol;
                    if (TLcol == 0)
                                Col = cv::Scalar(0,0,255); // Red
                    else if (TLcol == 1)
                                Col = cv::Scalar(0,140,255); // Orange
                    else if (TLcol == 2)
                                Col = cv::Scalar(0,255,255); // Yellow
                    else if (TLcol == 3)
                                Col = cv::Scalar(0,255,0); // Green
                    else if (TLcol == 4)
                                Col = cv::Scalar(255,0,0); // Blue
                    else if (TLcol == 5)
                                Col = cv::Scalar(130,0,130); // Purple
                    else
                                continue;
                    LineWidth = 2;
                    x1 = DatCam.Lines_EndPts[DL][0];
                    x2 = DatCam.Lines_EndPts[DL][1];
                    y1 = DatCam.Lines_EndPts[DL][2];
                    y2 = DatCam.Lines_EndPts[DL][3];
                    cv::line(Im3, cv::Point(x1,y1), cv::Point(x2,y2), Col, LineWidth, cv::LINE_AA);
                    }
               }
#endif
               mecl::core::Matrix<float64_t,1U,1U> r2;
               LineWidth = 1;
               for(int i=0; i<nL; i++)
               {
                              mecl::core::Matrix<float64_t,3U,1U> x0_A = p1_A[i];
                              mecl::core::Matrix<float64_t,3U,1U> x1_A = p1_A[i] + v_A[i]*LineLen1;
                              mecl::core::Matrix<float64_t,3U,1U> x2_A = p1_A[i] - v_A[i]*LineLen1;
                              // Plot initial pose (white)
                              Col = cv::Scalar(255,255,255);
                              mecl::core::Matrix<float64_t,3U,1U> x0_C0 = R0_C_A.mmul(x0_A - T0_A_C_A);
                              mecl::core::Matrix<float64_t,3U,1U> x1_C0 = R0_C_A.mmul(x1_A - T0_A_C_A);
                              mecl::core::Matrix<float64_t,3U,1U> x2_C0 = R0_C_A.mmul(x2_A - T0_A_C_A);
                              mecl::core::Matrix<float64_t,2U,1U> uv0_C0 = Project3D_to_2D(x0_C0, DatCam, 0U, r2);
                              mecl::core::Matrix<float64_t,2U,1U> uv1_C0 = Project3D_to_2D(x1_C0, DatCam, 0U, r2);
                              mecl::core::Matrix<float64_t,2U,1U> uv2_C0 = Project3D_to_2D(x2_C0, DatCam, 0U, r2);
                              cv::line(Im3, cv::Point(uv1_C0(0,0),uv1_C0(1,0)), cv::Point(uv2_C0(0,0),uv2_C0(1,0)), Col, LineWidth, cv::LINE_AA);
                              cv::circle(Im3, cv::Point(uv0_C0(0,0),uv0_C0(1,0)), 3, Col, 3);
                              // Plot final pose (black)
                              Col = cv::Scalar(0,0,0);
                              mecl::core::Matrix<float64_t,3U,1U> x0_C = R_C_A.mmul(x0_A - T_A_C_A);
                              mecl::core::Matrix<float64_t,3U,1U> x1_C = R_C_A.mmul(x1_A - T_A_C_A);
                              mecl::core::Matrix<float64_t,3U,1U> x2_C = R_C_A.mmul(x2_A - T_A_C_A);
                              mecl::core::Matrix<float64_t,2U,1U> uv0_C = Project3D_to_2D(x0_C, DatCam, 0U, r2);
                              mecl::core::Matrix<float64_t,2U,1U> uv1_C = Project3D_to_2D(x1_C, DatCam, 0U, r2);
                              mecl::core::Matrix<float64_t,2U,1U> uv2_C = Project3D_to_2D(x2_C, DatCam, 0U, r2);
                              cv::line(Im3, cv::Point(uv1_C(0,0),uv1_C(1,0)), cv::Point(uv2_C(0,0),uv2_C(1,0)), Col, LineWidth, cv::LINE_AA);
                              cv::circle(Im3, cv::Point(uv0_C(0,0),uv0_C(1,0)), 3, Col, 3);
               }
               // Print text in top left corner: stats
               int ySpace = 15;
               int yStart = 20;
               int xStart = 10;
               if(CalibTLines.nLines == NUM_LINES::TWO_LINES)
                              cv::putText(Im3, "2 lines", cv::Point(xStart,yStart), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,0,255), 1);
               if (CalibTLines.nLines == NUM_LINES::THREE_LINES)
                              cv::putText(Im3, "3 lines", cv::Point(xStart,yStart), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,0,255), 1);
               char TextSD[100];
               sprintf(TextSD,"SepDist: %f", CalibTLines.SepDist);
               cv::putText(Im3, TextSD, cv::Point(xStart,yStart+ySpace), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,0,255), 1);
               char TextX[100];
               sprintf(TextX,"# x line matches: %d", nLMx);
               cv::putText(Im3, TextX, cv::Point(xStart,yStart+ySpace*2), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,0,255), 1);
               char TextY[100];
               sprintf(TextY,"# y line matches: %d", nLMy);
               cv::putText(Im3, TextY, cv::Point(xStart,yStart+ySpace*3), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,0,255), 1);
               // Display figure
               cv::imshow("MD_ImFit", Im3);
               cvWaitKey(0);
#endif