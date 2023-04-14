#include "ExtrinsicRefinement.h"
#define SHOW_IMAGE
//#define MYPYR
//#define NO_XPOS_IMPROVE
#ifdef PIKEOS
#include "stdlib.h"
#endif
// Search for "MD_PRINT" to find added print statments for PC vs. embedded comparison

#ifdef PIKEOS
// MD_PRINT start
void PrintT_(mecl::core::Matrix<float32_t,3,1> &T)
{
	logging::LogInstance log_ro;
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"[%f, ",T(0,0)
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"%f, ",T(1,0)
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"%f]\n",T(2,0)
	);
}
void PrintT_(mecl::core::Matrix<float64_t,3,1> &T)
{
	logging::LogInstance log_ro;
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"[%f, ",(float32_t)T(0,0)
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"%f, ",(float32_t)T(1,0)
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"%f]\n ",(float32_t)T(2,0)
	);
}
void PrintR_(mecl::core::Matrix<float32_t,3,3> &R)
{
	logging::LogInstance log_ro;
	for (int i=0; i<3; i++)
	{
		log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"\n     "
		);
		for (int j=0; j<3; j++)
		{
			log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
				"%f, ",R(i,j)
			);

		}
	}
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"\n"
	);
}
void PrintR_(mecl::core::RotationMatrix<float32_t> &R)
{
	logging::LogInstance log_ro;
	for (int i=0; i<3; i++)
	{
		log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"\n     "
		);
		for (int j=0; j<3; j++)
		{
			log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
				"%f, ",R(i,j)
			);
		}
	}
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"\n"
	);
}
void PrintR_(mecl::core::Matrix<float64_t,3,3> &R)
{
	logging::LogInstance log_ro;
	for (int i=0; i<3; i++)
	{
		log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"\n     "
		);
		for (int j=0; j<3; j++)
		{
			log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
				"%f, ",(float32_t)R(i,j)
			);
		}
	}
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"\n"
	);
}
void PrintE_(mecl::core::Matrix<float64_t,9,9> &E)
{
	logging::LogInstance log_ro;
	for (int i=0; i<9; i++)
	{
		log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"\n     "
		);
		for (int j=0; j<9; j++)
		{
			log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
				"%f, ",(float32_t)E(i,j)
			);
		}
	}
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"\n"
	);
}
void PrintM39_(mecl::core::Matrix<float64_t,3,9> &M)
{
	logging::LogInstance log_ro;
	for (int i=0; i<3; i++)
	{
		log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"\n     "
		);
		for (int j=0; j<9; j++)
		{
			log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
				"%f, ",(float32_t)M(i,j)
			);
		}
	}
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"\n"
	);
}
#else
void JobPrintR__(mecl::core::Matrix<float32_t, 3, 3> &R)
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
void PrintT_(mecl::core::Matrix<float32_t,3,1> &T)
{
	vm_cprintf("[\n%f, \n",T(0,0));
	vm_cprintf("%f, \n",T(1,0));
	vm_cprintf("%f]\n",T(2,0));
}
void PrintT_(mecl::core::Matrix<float64_t,3,1> &T)
{
	vm_cprintf("[%\nf, \n",(float32_t)T(0,0));
	vm_cprintf("%f, \n",(float32_t)T(1,0));
	vm_cprintf("%f]\n",(float32_t)T(2,0));
}
void PrintR_(mecl::core::Matrix<float32_t,3,3> &R)
{
	for (int i=0; i<3; i++)
	{
		vm_cprintf("\n     ");
		for (int j=0; j<3; j++)
		{
			vm_cprintf("%f, \n",R(i,j));
		}
	}
	vm_cprintf("\n");
}
void PrintR_(mecl::core::RotationMatrix<float32_t> &R)
{
	for (int i=0; i<3; i++)
	{
		vm_cprintf("\n     ");
		for (int j=0; j<3; j++)
		{
			vm_cprintf("%f, \n",R(i,j));
		}
	}
	vm_cprintf("\n");
}
void PrintR_(mecl::core::Matrix<float64_t,3,3> &R)
{
	for (int i=0; i<3; i++)
	{
		vm_cprintf("\n     ");
		for (int j=0; j<3; j++)
		{
			vm_cprintf("%f, \n",(float32_t)R(i,j));
		}
	}
	vm_cprintf("\n");
}
void PrintE_(mecl::core::Matrix<float64_t,9,9> &E)
{
	for (int i=0; i<9; i++)
	{
		vm_cprintf("\n     ");
		for (int j=0; j<9; j++)
		{
			vm_cprintf("%f, \n",(float32_t)E(i,j));
		}
	}
	vm_cprintf("\n");
}
void PrintM39_(mecl::core::Matrix<float64_t,3,9> &M)
{
	for (int i=0; i<3; i++)
	{
		vm_cprintf("\n     ");
		for (int j=0; j<9; j++)
		{
			vm_cprintf("%f, \n",(float32_t)M(i,j));
		}
	}
	vm_cprintf("\n");
}
//************************
void FILE_PrintR__(mecl::core::Matrix<float32_t, 3, 3> &R, FILE *fp0)
{
	for (int i = 0; i<3; i++)
	{
		fprintf(fp0,"\n     ");
		for (int j = 0; j<3; j++)
		{
			fprintf(fp0,"%f, ", R(i, j));
		}
	}
	fprintf(fp0,"\n");
}
void FILE_PrintT_(mecl::core::Matrix<float32_t, 3, 1> &T, FILE *fp0)
{
	fprintf(fp0,"[\n%f, \n", T(0, 0));
	fprintf(fp0,"%f, \n", T(1, 0));
	fprintf(fp0,"%f]\n", T(2, 0));
}
void FILE_PrintT_(mecl::core::Matrix<float64_t, 3, 1> &T, FILE *fp0)
{
	vm_cprintf("[%f, ",(float32_t)T(0,0));
	vm_cprintf("%f, ",(float32_t)T(1,0));
	vm_cprintf("%f]\n",(float32_t)T(2,0));
}
void FILE_PrintR_(mecl::core::Matrix<float32_t, 3, 3> &R, FILE *fp0)
{
	for (int i = 0; i<3; i++)
	{
		fprintf(fp0,"\n     ");
		for (int j = 0; j<3; j++)
		{
			fprintf(fp0,"%f, ", R(i, j));
		}
	}
	fprintf(fp0,"\n");
}
void FILE_PrintR_(mecl::core::RotationMatrix<float32_t> &R, FILE *fp0)
{
	for (int i = 0; i<3; i++)
	{
		fprintf(fp0,"\n     ");
		for (int j = 0; j<3; j++)
		{
			fprintf(fp0,"%f, ", R(i, j));
		}
	}
	fprintf(fp0,"\n");
}
void FILE_PrintR_(mecl::core::Matrix<float64_t, 3, 3> &R, FILE *fp0)
{
	for (int i = 0; i<3; i++)
	{
		fprintf(fp0,"\n     ");
		for (int j = 0; j<3; j++)
		{
			fprintf(fp0,"%f, ", (float32_t)R(i, j));
		}
	}
	fprintf(fp0,"\n");
}
void FILE_PrintE_(mecl::core::Matrix<float64_t, 9, 9> &E, FILE *fp0)
{
	for (int i = 0; i<9; i++)
	{
		fprintf(fp0,"\n     ");
		for (int j = 0; j<9; j++)
		{
			fprintf(fp0,"%f, ", (float32_t)E(i, j));
		}
	}
	fprintf(fp0,"\n");
}
void FILE_PrintM39_(mecl::core::Matrix<float64_t, 3, 9> &M, FILE *fp0)
{
	for (int i = 0; i<3; i++)
	{
		fprintf(fp0,"\n     ");
		for (int j = 0; j<9; j++)
		{
			fprintf(fp0,"%f, ", (float32_t)M(i, j));
		}
	}
	fprintf(fp0,"\n");
}
#endif

// MD_PRINT end


// Use 3D Vanishing Points (VPs) to initialize RAdjust = R_C0_C = R_A_C0' * R_A_C;
void ExRefine::VP3D_Extrinsics(MD_Data &DatCam, FILE *fp0)
{
	// RANSAC_EDIT start
	float32_t MaxAngEr1 = 2.0 / 180.0*M_PI; // Tuning parameter for max error allowed in axis 1
	float32_t MaxAngEr2 = 5.0 / 180.0*M_PI; // Tuning parameter for max error allowed in axis 2
	// RANSAC_EDIT end
#ifdef PIKEOS
	logging::LogInstance log_ro;
	// MD_PRINT start
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"\n\nCam: %f\n",(float32_t)DatCam.CamNum
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"ExRefine::VP3D_Extrinsics input summary (DatCam)\n"
	);


	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Camera intrinsics\n"
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"FC = [%f, %f]\n",DatCam.CamFC[0], DatCam.CamFC[1]
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"PP = [%f, %f]\n",DatCam.CamPP[0], DatCam.CamPP[1]
	);

	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"PP = [%f, %f]\n",DatCam.CamPP[0], DatCam.CamPP[1]
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Dist_im2world = [%f, %f, %f, %f, %f, %f]\n",DatCam.Dist_im2world[0],DatCam.Dist_im2world[1],DatCam.Dist_im2world[2],DatCam.Dist_im2world[3],DatCam.Dist_im2world[4],DatCam.Dist_im2world[5]
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Dist_world2im = [%f, %f, %f, %f, %f, %f]\n",DatCam.Dist_world2im[0],DatCam.Dist_world2im[1],DatCam.Dist_world2im[2],DatCam.Dist_world2im[3],DatCam.Dist_world2im[4],DatCam.Dist_world2im[5]
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"R_A_C0:");
	PrintR_(DatCam.R_A_C0
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Orig_xyz_CamPos: ");
	PrintT_(DatCam.Orig_xyz_CamPos
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Orig_pyr_CamPos: "
	);
	PrintT_(DatCam.Orig_pyr_CamPos);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Orig_RAdjust:"
	);
	PrintR_(DatCam.Orig_RAdjust);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Orig_R_A_C:"
	);
	PrintR_(DatCam.Orig_R_A_C);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Lines used in VP's:\n"
	);
#else
	// MD_PRINT start
	vm_cprintf("\n\nCam: %f\n",(float32_t)DatCam.CamNum);
	vm_cprintf("ExRefine::VP3D_Extrinsics input summary (DatCam)\n");
	vm_cprintf("Camera intrinsics\n");
	vm_cprintf("FC = [%f, %f]\n",DatCam.CamFC[0], DatCam.CamFC[1]);
	vm_cprintf("PP = [%f, %f]\n",DatCam.CamPP[0], DatCam.CamPP[1]);
	vm_cprintf("Dist_im2world = [%f, %f, %f, %f, %f, %f]\n",DatCam.Dist_im2world[0],DatCam.Dist_im2world[1],DatCam.Dist_im2world[2],DatCam.Dist_im2world[3],DatCam.Dist_im2world[4],DatCam.Dist_im2world[5]);
	vm_cprintf("Dist_world2im = [%f, %f, %f, %f, %f, %f]\n",DatCam.Dist_world2im[0],DatCam.Dist_world2im[1],DatCam.Dist_world2im[2],DatCam.Dist_world2im[3],DatCam.Dist_world2im[4],DatCam.Dist_world2im[5]);
	vm_cprintf("R_A_C0:"); PrintR_(DatCam.R_A_C0);
	vm_cprintf("Orig_xyz_CamPos: "); PrintT_(DatCam.Orig_xyz_CamPos);
	vm_cprintf("Orig_pyr_CamPos: "); PrintT_(DatCam.Orig_pyr_CamPos);
	vm_cprintf("Orig_RAdjust:"); PrintR_(DatCam.Orig_RAdjust);
	vm_cprintf("Orig_R_A_C:"); PrintR_(DatCam.Orig_R_A_C);
	vm_cprintf("Lines used in VP's:\n");

	fprintf(fp0,"\n\nCam: %f\n", (float32_t)DatCam.CamNum);
	fprintf(fp0,"ExRefine::VP3D_Extrinsics input summary (DatCam)\n");
	fprintf(fp0,"Camera intrinsics\n");
	fprintf(fp0,"FC = [%f, %f]\n", DatCam.CamFC[0], DatCam.CamFC[1]);
	fprintf(fp0,"PP = [%f, %f]\n", DatCam.CamPP[0], DatCam.CamPP[1]);
	fprintf(fp0,"Dist_im2world = [%f, %f, %f, %f, %f, %f]\n", DatCam.Dist_im2world[0], DatCam.Dist_im2world[1], DatCam.Dist_im2world[2], DatCam.Dist_im2world[3], DatCam.Dist_im2world[4], DatCam.Dist_im2world[5]);
	fprintf(fp0,"Dist_world2im = [%f, %f, %f, %f, %f, %f]\n", DatCam.Dist_world2im[0], DatCam.Dist_world2im[1], DatCam.Dist_world2im[2], DatCam.Dist_world2im[3], DatCam.Dist_world2im[4], DatCam.Dist_world2im[5]);
	fprintf(fp0,"R_A_C0:"); FILE_PrintR_(DatCam.R_A_C0, fp0);
	fprintf(fp0,"Orig_xyz_CamPos: "); FILE_PrintT_(DatCam.Orig_xyz_CamPos, fp0);
	fprintf(fp0,"Orig_pyr_CamPos: "); FILE_PrintT_(DatCam.Orig_pyr_CamPos, fp0);
	fprintf(fp0,"Orig_RAdjust:"); FILE_PrintR_(DatCam.Orig_RAdjust, fp0);
	fprintf(fp0,"Orig_R_A_C:"); FILE_PrintR_(DatCam.Orig_R_A_C, fp0);
	fprintf(fp0,"Lines used in VP's:\n");
#endif
	int E0 = DatCam.numEndPts_s32;
#ifdef PIKEOS
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"numEndPts_s32 = %f\n",(float32_t)E0
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"numLinLab_s32 = %f\n",(float32_t)DatCam.numLinLab_s32
	);
	for (int i=0; i<E0; i++)
		log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"Line %f: Label: %f, Endpoints: [x1=%f, x2=%f, y1=%f, y2=%f]\n",(float32_t)i,(float32_t)DatCam.Lines_Lab[i],DatCam.Lines_EndPts[i][0],DatCam.Lines_EndPts[i][1],DatCam.Lines_EndPts[i][2],DatCam.Lines_EndPts[i][3]
			);
#else
	vm_cprintf("numEndPts_s32 = %f\n",(float32_t)E0);
	vm_cprintf("numLinLab_s32 = %f\n",(float32_t)DatCam.numLinLab_s32);
	fprintf(fp0,"numEndPts_s32 = %f\n", (float32_t)E0);
	fprintf(fp0,"numLinLab_s32 = %f\n", (float32_t)DatCam.numLinLab_s32);
	for (int i = 0; i < E0; i++)
	{
		vm_cprintf("Line %f: Label: %f, Endpoints: [x1=%f, x2=%f, y1=%f, y2=%f]\n", (float32_t)i, (float32_t)DatCam.Lines_Lab[i], DatCam.Lines_EndPts[i][0], DatCam.Lines_EndPts[i][1], DatCam.Lines_EndPts[i][2], DatCam.Lines_EndPts[i][3]);
		fprintf(fp0,"Line %f: Label: %f, Endpoints: [x1=%f, x2=%f, y1=%f, y2=%f]\n", (float32_t)i, (float32_t)DatCam.Lines_Lab[i], DatCam.Lines_EndPts[i][0], DatCam.Lines_EndPts[i][1], DatCam.Lines_EndPts[i][2], DatCam.Lines_EndPts[i][3]);
	}
#endif
	// MD_PRINT end
	// Get categories of lines based on labels
	mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> Idx0, Idx1, Idx2;
	for (int i=0; i<DatCam.numLinLab_s32; i++)
	{
		if (DatCam.Lines_Lab[i]==0)
			Idx0.pushBack_v(i);
		else if (DatCam.Lines_Lab[i]==1)
			Idx1.pushBack_v(i);
	}
	// Idx1 should be larger than Idx2 => start with Idx1
	if (Idx1.size_u32() > Idx0.size_u32())
		Idx2.copyAll_v(Idx0);
	else
	{
		Idx2.copyAll_v(Idx1);
		Idx1.copyAll_v(Idx0);
	}
	int nL1 = Idx1.size_u32();
	int nL2 = Idx2.size_u32();

	// RANSAC_EDIT start
	if (nL1<2)
	{
		DatCam.VP3D_R_A_C = DatCam.Orig_R_A_C;
		return;
	}
	// RANSAC_EDIT end
	// Fit the first axis based on which CalibTLines axis has the most lines = Idx1
	mecl::core::Matrix<float32_t,3,1> PrjNorm1_C[vpcs::LINES_SIZE_MAX]; // Normal to projection plane * LineLen
	for (int i=0; i<nL1; i++)
	{
		int DL = Idx1[i];
		float32_t u1 = DatCam.Lines_EndPts[DL][0];
		float32_t u2 = DatCam.Lines_EndPts[DL][1];
		float32_t v1 = DatCam.Lines_EndPts[DL][2];
		float32_t v2 = DatCam.Lines_EndPts[DL][3];
		float32_t LineLen = pow( static_cast<float64_t>((u2-u1)*(u2-u1) + (v2-v1)*(v2-v1)) , 0.5);
		mecl::core::Matrix<float32_t,2U,1U> uv1_C, uv2_C;
		uv1_C(0) = u1;
		uv2_C(0) = u2;
		uv1_C(1) = v1;
		uv2_C(1) = v2;

		mecl::core::Matrix<float32_t,3U,1U> xyz1_C = Project2D_to_3D(uv1_C,  DatCam, 0); // 0 for no distortion
		mecl::core::Matrix<float32_t,3U,1U> xyz2_C = Project2D_to_3D(uv2_C,  DatCam, 0); // 0 for no distortion
		PrjNorm1_C[i] = CrossProd(xyz1_C, xyz2_C);
		PrjNorm1_C[i] = PrjNorm1_C[i].mul(LineLen/PrjNorm1_C[i].norm());
	}
	// RANSAC_EDIT start
	// RANSAC to find outliers
	int nRounds = 10;
	bool AcceptLine[vpcs::LINES_SIZE_MAX];
	bool AcceptLineBest[vpcs::LINES_SIZE_MAX];
	int BestNLines = -1;
	srand(0); // Consistent seed every time
	for (int i=0; i<nRounds; i++)
	{
		int i1 = rand() % nL1;
		int i2 = rand() % nL1;
		while (i1==i2)
			i2 = rand() % nL1;
		mecl::core::Matrix<float32_t,3,1> Axis1_C0 = CrossProd(PrjNorm1_C[i1], PrjNorm1_C[i2]);
		Axis1_C0 = Axis1_C0 / Axis1_C0.norm();
		int nGoodLines = 0;
		for (int j=0; j<nL1; j++)
		{
			double DotProd0 = ( Axis1_C0(0)*PrjNorm1_C[j](0) + Axis1_C0(1)*PrjNorm1_C[j](1) + Axis1_C0(2)*PrjNorm1_C[j](2) ) / PrjNorm1_C[j].norm();
			double AngEr0 = fabs( M_PI/2.0 - acos(DotProd0) );
			if (AngEr0 < MaxAngEr1)
			{
				AcceptLine[j] = true;
				nGoodLines++;
			}
			else
				AcceptLine[j] = false;
		}
		if (nGoodLines > BestNLines)
		{
			BestNLines = nGoodLines;
			for (int j=0; j<nL1; j++)
				AcceptLineBest[j] = AcceptLine[j];
		}
	}
	// FIX_MATCH start
	for (int i=0; i<nL1; i++)
	{
		if (!AcceptLineBest[i])
			DatCam.Lines_Lab[Idx1[i]] = -1;
	}
	// FIX_MATCH end

	// Find the cross product of each pair of lines
	mecl::core::Matrix<float32_t,3,1> CrossProd1[vpcs::LINES_SIZE_MAX*(vpcs::LINES_SIZE_MAX -1)];
	float32_t MaxLen1 = 0;
	int MaxLenIdx1 = 0;
	int Cnt1 = 0;
	for (int i=0; i<nL1; i++)
	{
		for (int j=i+1; j<nL1; j++)
		{
			// RANSAC_EDIT start
			if (AcceptLineBest[i] & AcceptLineBest[j])
			// RANSAC_EDIT end
			{
				CrossProd1[Cnt1] = CrossProd(PrjNorm1_C[i], PrjNorm1_C[j]);
				float32_t Len = CrossProd1[Cnt1].norm();
				if (Len > MaxLen1)
				{
					MaxLen1 = Len;
					MaxLenIdx1 = Cnt1;
				}
				Cnt1++;
			}
		}
	}

	// Make all the cross product lines face the same direction & sum them
	mecl::core::Matrix<float32_t,3,1> Axis1_C(0);
	for (int i=0; i<Cnt1; i++)
	{
		mecl::core::Matrix<float32_t,1,1> DotProd = (CrossProd1[i].t()).mmul(CrossProd1[MaxLenIdx1]);
		if (DotProd(0) > 0)
			Axis1_C = Axis1_C.add( CrossProd1[i] );
		else
			Axis1_C = Axis1_C.add( -CrossProd1[i] );
	}

	// Final normalized Axis1_C i.e. 3D vanishing point in this direction
	Axis1_C = Axis1_C.mul(1.0/Axis1_C.norm());

	// Find RAdjust1 based on Axis1_C
	mecl::core::Matrix<float32_t,3,3> RAdjust1, R1_A_C;
	mecl::core::Matrix<float32_t,3,3> R0_A_C = DatCam.Orig_R_A_C; // Approximate input Rotation matrix
	mecl::core::Matrix<float32_t,3,1> x_A(0); x_A(0)=1; // x axis in A
	mecl::core::Matrix<float32_t,3,1> y_A(0); y_A(1)=1; // y axis in A
	mecl::core::Matrix<float32_t,3,1> x0_C = (R0_A_C.t()).mmul(x_A);
	mecl::core::Matrix<float32_t,3,1> y0_C = (R0_A_C.t()).mmul(y_A);
	mecl::core::Matrix<float32_t,1,1> xDot = (x0_C.t()).mmul(Axis1_C);
	mecl::core::Matrix<float32_t,1,1> yDot = (y0_C.t()).mmul(Axis1_C);
	bool xAx1 = true;
	if (fabs(xDot(0)) > fabs(yDot(0))) // Axis1_C is aligned with x_A axis
	{
		if (xDot(0) < 0)
			Axis1_C = Axis1_C.mul(-1.0);
		RAdjust1 = Ax2Ax_to_R3(x0_C, Axis1_C);
	}
	else // Axis1_C is aligned with y_A axis
	{
		if (yDot(0) < 0)
			Axis1_C = Axis1_C.mul(-1.0);
		RAdjust1 = Ax2Ax_to_R3(y0_C, Axis1_C);
		xAx1 = false;
	}
	R1_A_C = R0_A_C.mmul(RAdjust1.t()); // Rotation matrix lined up with Axis1_C

	// Return if no more lines to process
	if (nL2 == 0)
	{
		DatCam.VP3D_R_A_C = R1_A_C;
		return;
	}

	// Use Axis2 lines to align R_A_C further (single degree of freedom)
	mecl::core::Matrix<float32_t,2,1> PrjNorm2_A[vpcs::LINES_SIZE_MAX]; // Project PrjNorm2_C => PrjNorm2_A based on R1_A_C (x OR y direction doesn't matter)
	float32_t MaxLen2 = 0;
	int MaxLenIdx2 = 0;
	for (int i=0; i<nL2; i++)
	{
		int DL = Idx2[i];
		float32_t u1 = DatCam.Lines_EndPts[DL][0];
		float32_t u2 = DatCam.Lines_EndPts[DL][1];
		float32_t v1 = DatCam.Lines_EndPts[DL][2];
		float32_t v2 = DatCam.Lines_EndPts[DL][3];
		float32_t LineLen = pow( static_cast<float64_t>((u2-u1)*(u2-u1) + (v2-v1)*(v2-v1)) , 0.5);
		mecl::core::Matrix<float32_t,2,1> uv1_C, uv2_C;
		uv1_C(0) = u1;
		uv2_C(0) = u2;
		uv1_C(1) = v1;
		uv2_C(1) = v2;
		mecl::core::Matrix<float32_t,3,1> xyz1_C = Project2D_to_3D(uv1_C,  DatCam, 0); // 0 for no distortion
		mecl::core::Matrix<float32_t,3,1> xyz2_C = Project2D_to_3D(uv2_C,  DatCam, 0); // 0 for no distortion
		mecl::core::Matrix<float32_t,3,1> PrjNorm2_C = CrossProd(xyz1_C, xyz2_C);
		PrjNorm2_C = PrjNorm2_C.mul(LineLen/PrjNorm2_C.norm()); // Normal to projection plane * LineLen
		mecl::core::Matrix<float32_t,3,1> PrjNorm2_A_ = R1_A_C.mmul(PrjNorm2_C);
		if (xAx1) // Don't care about the xAxis
		{
			PrjNorm2_A[i](0) = PrjNorm2_A_(1);
			PrjNorm2_A[i](1) = PrjNorm2_A_(2);
		}
		else // Don't care about the yAxis
		{
			PrjNorm2_A[i](0) = PrjNorm2_A_(2);
			PrjNorm2_A[i](1) = PrjNorm2_A_(0);
		}
		if (PrjNorm2_A[i].norm() > MaxLen2)
		{
			MaxLen2 = PrjNorm2_A[i].norm();
			MaxLenIdx2 = i;
		}
	}

	// RANSAC_EDIT start
	// "RANSAC" to find any outliers
	bool AcceptLine2[vpcs::LINES_SIZE_MAX];
	bool AcceptLineBest2[vpcs::LINES_SIZE_MAX] = { true };
	float32_t LineLenTotBest2 = 0;
	for (int i=0; i<nL2; i++)
	{
		float32_t LineLenTot2 = PrjNorm2_A[i].norm();
		AcceptLine2[i] = true;
		for (int j=0; j<nL2; j++)
		{
			if (i != j)
			{
				double DotProd2 = ( PrjNorm2_A[i](0)*PrjNorm2_A[j](0) + PrjNorm2_A[i](1)*PrjNorm2_A[j](1) ) / ( PrjNorm2_A[i].norm() * PrjNorm2_A[j].norm() );
				double AngEr2 = acos(fabs(DotProd2));
				if (AngEr2 < MaxAngEr2)
				{
					AcceptLine2[j] = true;
					LineLenTot2 += PrjNorm2_A[j].norm();
				}
				else
					AcceptLine2[j] = false;
			}
		}
		if (LineLenTot2 > LineLenTotBest2)
		{
			MaxLenIdx2 = i;
			LineLenTotBest2 = LineLenTot2;
			for (int j=0; j<nL2; j++)
				AcceptLineBest2[j] = AcceptLine2[j];
		}
	}
	// FIX_MATCH start
	for (int i=0; i<nL2; i++)
	{
		if (!AcceptLineBest2[i])
			DatCam.Lines_Lab[Idx2[i]] = -1;
	}
	// FIX_MATCH end

	// Project PrjNorm2_C => PrjNorm2_A based on R1_A_C
	mecl::core::Matrix<float32_t,2,1> Axis2_C(0); // x OR y direction doesn't matter
	for (int i=0; i<nL2; i++)
	{
		// RANSAC_EDIT start
		if (AcceptLineBest2[i])
		// RANSAC_EDIT end
		{
			mecl::core::Matrix<float32_t,1,1> DotProd = (PrjNorm2_A[i].t()).mmul(PrjNorm2_A[MaxLenIdx2]);
			if (DotProd(0) > 0)
				Axis2_C = Axis2_C.add( PrjNorm2_A[i] );
			else
				Axis2_C = Axis2_C.add( -PrjNorm2_A[i] );
		}
	}

	// Find the Axis2 angle
	float32_t Ax2Ang = atan2(Axis2_C(1),Axis2_C(0)); // if(xAx1) => (y,z). if(yAx1) => (z,x)
	Ax2Ang = fmod(static_cast<float64_t>(Ax2Ang), 0.5*M_PI);
	if (Ax2Ang > 0.25*M_PI)
		Ax2Ang -= 0.5*M_PI;
	if (Ax2Ang < -0.25*M_PI)
		Ax2Ang += 0.5*M_PI;

	// Get the final RAdjust
	mecl::core::Matrix<float32_t,3,3> RAdjust2(0), R2_A_C;
	if (xAx1) // Spin about x axis
	{
		RAdjust2(0,0) = 1;
		RAdjust2(1,1) = cos(Ax2Ang);
		RAdjust2(2,2) = cos(Ax2Ang);
		RAdjust2(1,2) =-sin(Ax2Ang);
		RAdjust2(2,1) = sin(Ax2Ang);
	}
	else // Spin about y axis
	{
		RAdjust2(0,0) = cos(Ax2Ang);
		RAdjust2(1,1) = 1;
		RAdjust2(2,2) = cos(Ax2Ang);
		RAdjust2(2,0) =-sin(Ax2Ang);
		RAdjust2(0,2) = sin(Ax2Ang);
	}
	R2_A_C = (RAdjust2.t()).mmul(R1_A_C); // Rotation matrix lined up with Axis2_C

	DatCam.VP3D_R_A_C = R2_A_C;
#ifdef PIKEOS
	// MD_PRINT start
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Axis1_C: "
	);
	PrintT_(Axis1_C);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"R1_A_C:"
	);
	PrintR_(R1_A_C);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Axis2_C: [%f, %f]\n",Axis2_C(0),Axis2_C(1)
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"VP3D_R_A_C:"
	);
	PrintR_(DatCam.VP3D_R_A_C);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"ExRefine::VP3D_Extrinsics end\n\n"
	);
#else
	vm_cprintf("Axis1_C: "); PrintT_(Axis1_C);
	vm_cprintf("R1_A_C:"); PrintR_(R1_A_C);
	vm_cprintf("Axis2_C: [%f, %f]\n",Axis2_C(0),Axis2_C(1));
	vm_cprintf("VP3D_R_A_C:"); PrintR_(DatCam.VP3D_R_A_C);
	vm_cprintf("ExRefine::VP3D_Extrinsics end\n\n");

	fprintf(fp0,"Axis1_C: "); FILE_PrintT_(Axis1_C, fp0);
	fprintf(fp0,"R1_A_C:"); FILE_PrintR_(R1_A_C, fp0);
	fprintf(fp0,"Axis2_C: [%f, %f]\n", Axis2_C(0), Axis2_C(1));
	fprintf(fp0,"VP3D_R_A_C:"); FILE_PrintR_(DatCam.VP3D_R_A_C, fp0);
	fprintf(fp0,"ExRefine::VP3D_Extrinsics end\n\n");
#endif
	// MD_PRINT end
}


// MD_MOD_02_27 the function inputs below
void ExRefine::MatchLines(MD_Data &DatCam, TargetLines &CalibTLines, bool TrustT_A_C_A, mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> *Matches, mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> *MatchesAll, FILE *fp0)
// Matches    = [x/y][CalibTargetLine#] = Detected line #
// MatchesAll = [x/y][DetLine#] = CalibTargetLine# <= match extra detected lines to each target line
// MD_MOD_OPTIM_ALL_LINES end
{
#ifdef PIKEOS
	logging::LogInstance log_ro;
	// MD_PRINT start
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"\n\nExRefine::MatchLines start\n"
	);
#else
	vm_cprintf("\n\nExRefine::MatchLines start\n");
	fprintf(fp0, "\n\nExRefine::MatchLines start\n");
#endif
	int nImPtsDisp = 20;

#ifdef PIKEOS
	for (int i = 0; i < nImPtsDisp; i++)
	{
		int j = vpcs::c_ImageSize_u32 / (nImPtsDisp + 1) * i;
		log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"ImUD[%f] = %f\n", (float32_t)j, (float32_t)DatCam.ImUD[j]
		);
	}
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"CalibTLines.SepDist = %f", CalibTLines.SepDist
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"CalibTLines.LinesLen = [%f, %f]", (float32_t)CalibTLines.LinesLen[0], (float32_t)CalibTLines.LinesLen[1]
	);
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < CalibTLines.LinesLen[0]; j++)
		{
			log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
				"CalibTLines.Lines[%f][%f] = [", (float32_t)i, (float32_t)j
			);
			for (int k = 0; k < 5; k++)
				log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
					"%f, ", CalibTLines.Lines[i][j][k]
				);
			log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
				"]\n"
			);
		}
	}
	for (int i = 0; i < 4; i++)
	{
		log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"CalibTLines.yCamSee[%f] = [", (float32_t)i
		);
		for (int j = 0; j < 4; j++)
			log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
				"%f, ", (float32_t)CalibTLines.yCamSee[i][j]
			);
		log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"], Len = %f\n", (float32_t)CalibTLines.yCamSeeLen[i]
		);
	}
#else
	for (int i = 0; i < nImPtsDisp; i++)
	{
		int j = vpcs::c_ImageSize_u32 / (nImPtsDisp + 1) * i;
		vm_cprintf("ImUD[%f] = %f\n", (float32_t)j, (float32_t)DatCam.ImUD[j]);
		fprintf(fp0, "ImUD[%f] = %f\n", (float32_t)j, (float32_t)DatCam.ImUD[j]);
	}
	vm_cprintf("CalibTLines.SepDist = %f", CalibTLines.SepDist);
	vm_cprintf("CalibTLines.LinesLen = [%f, %f]", (float32_t)CalibTLines.LinesLen[0], (float32_t)CalibTLines.LinesLen[1]);
	fprintf(fp0, "CalibTLines.SepDist = %f", CalibTLines.SepDist);
	fprintf(fp0, "CalibTLines.LinesLen = [%f, %f]", (float32_t)CalibTLines.LinesLen[0], (float32_t)CalibTLines.LinesLen[1]);

	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < CalibTLines.LinesLen[0]; j++)
		{
			vm_cprintf("CalibTLines.Lines[%f][%f] = [", (float32_t)i, (float32_t)j);
			fprintf(fp0, "CalibTLines.Lines[%f][%f] = [", (float32_t)i, (float32_t)j);
			for (int k = 0; k < 5; k++)
			{
				vm_cprintf("%f, \n", CalibTLines.Lines[i][j][k]);
				fprintf(fp0, "%f, \n", CalibTLines.Lines[i][j][k]);
			}
			vm_cprintf("]\n");
			fprintf(fp0, "]\n");
		}
	}
	for (int i = 0; i < 4; i++)
	{
		vm_cprintf("CalibTLines.yCamSee[%f] = [", (float32_t)i);
		fprintf(fp0, "CalibTLines.yCamSee[%f] = [", (float32_t)i);
		for (int j = 0; j < 4; j++)
		{
			vm_cprintf("%f, \n", (float32_t)CalibTLines.yCamSee[i][j]);
			fprintf(fp0, "%f, \n", (float32_t)CalibTLines.yCamSee[i][j]);
		}
		vm_cprintf("], Len = %f\n", (float32_t)CalibTLines.yCamSeeLen[i]);
		fprintf(fp0, "], Len = %f\n", (float32_t)CalibTLines.yCamSeeLen[i]);
	}
#endif
	// MD_PRINT end

	// Settings
	float32_t dPix = 3; // Number of pixels perpendicular to the line to check
	float32_t SCORE_LineLen_Exp = 0.5; // Reward for longer detected line length (>0), but with diminishing returns (<1)
	float32_t SCORE_LineColDiff_Exp = 1.5; // Reward for greater color (grayscale intensity) difference between the different sides of the line
	float32_t Match_MaxOffsetDistX = 500; // For initial matching. EVENTUALLY SET THIS TO 1000 AND MATCH LINES BETTER THAN CLOSEST MATCH...
	//float32_t Match_MaxOffsetDistX = 1000;

 //float32_t Match_MaxOffsetDistX = 750;	
    float32_t Match_MaxOffsetDistXf = 75; // Final matching max offset distance. MUST BE <200
	//float32_t Match_MaxOffsetDistXf = 100;
	//float32_t Match_MaxOffsetDistXf = 125; // Final matching max offset distance. MUST BE <200
	//float32_t Match_MaxOffsetDistXf = 150;
	//float32_t Match_MaxOffsetDistXf = 175;
	float32_t Match_MaxOffsetDistY = 600;
	//float32_t Match_MaxOffsetDistY = 800;
	//float32_t Match_MaxOffsetDistY = 1000;
	float32_t MinScoreCompBestMatch = 0.3; // Min score compared to the best match

	// Read in approximate calibration information. xyz_C = R_C_A * (xyz_A - T_A_C_A);    xyz_A = R_A_C * xyz_C + T_A_C_A
	mecl::core::Matrix<float32_t, 3, 1> T_A_C_A = DatCam.Orig_xyz_CamPos; // Translation from cAr Axle (A) reference frame (RF) => Cam (C) RF written in terms of A coordinates
	mecl::core::Matrix<float32_t, 3, 3> R0_A_C = DatCam.VP3D_R_A_C;
#ifndef PIKEOS
	uint32_t ImRows = vpcs::c_ImageHeight_u32;
#endif
	uint32_t ImCols = vpcs::c_ImageWidth_u32;


	// For each detected line in the image, project it onto z=0 using the approx 3D VP extrinsics & record scoring metrics
	int nDL = DatCam.numLinLab_s32;
	mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> DLx_Idx;
	mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX>  DLy_Idx;
	float32_t SCORE_LineLen[vpcs::LINES_SIZE_MAX] = { 0 };
	float32_t SCORE_LineColDiff[vpcs::LINES_SIZE_MAX] = { 0 };
#ifndef PIKEOS
	bool_t SCORE_XY_axis[vpcs::LINES_SIZE_MAX] = { true }; // true if aligned with x axis; false if aligned with y axis
#endif
	bool_t SCORE_BW_side[vpcs::LINES_SIZE_MAX] = { true };  // For x/y# Lines which align with x/y axis: BW=0 if LoY/X is white & HiY/X is black, BW=1 if LoY/X is black & HiY/X is white (all in cAr Axle refFrame)
	float32_t xyz1_A[vpcs::LINES_SIZE_MAX][3] = { 0 }; // Point 1 on the ground
	float32_t xyz2_A[vpcs::LINES_SIZE_MAX][3] = { 0 };  // Point 2 on the ground
	for (int DL = 0; DL < nDL; DL++)
	{
		if (DatCam.Lines_Lab[DL] == -1)
			continue;

		// Read in line in image coords
		float32_t u1 = DatCam.Lines_EndPts[DL][0];
		float32_t u2 = DatCam.Lines_EndPts[DL][1];
		float32_t v1 = DatCam.Lines_EndPts[DL][2];
		float32_t v2 = DatCam.Lines_EndPts[DL][3];

		// Find average white/black shading on either side of the line
		mecl::core::Matrix<float32_t, 2, 1> VecPara, VecPerp;
		VecPara(0) = u2 - u1;
		VecPara(1) = v2 - v1;
		float32_t LineLen2D = VecPara.norm();
		VecPara = VecPara.mul(1.0 / LineLen2D); // Unit vector parallel to 2D line in the direction from point 1 => 2
		VecPerp(0) = VecPara(1);
		VecPerp(1) = -VecPara(0); // Unit vector perpendicular to 2D line in the direction to the right of the line when looking from point 1 => 2
		mecl::core::Matrix<float32_t, 2, 1> AveRL(0); // Average image brightness on the Right & Left of the line when looking from point 1 => 2
		int nPts = (int)LineLen2D;
		uint32_t U0, V0;
		for (sint32_t i = 0; i < nPts; i++)
		{
			float32_t Pct = (float32_t)i;
			float32_t u0 = u1 + Pct*VecPara(0);
			float32_t v0 = v1 + Pct*VecPara(1);
			// Right side of line
			U0 = (uint32_t)(u0 + dPix*VecPerp(0) + 0.5f);
			V0 = (uint32_t)(v0 + dPix*VecPerp(1) + 0.5f);
			AveRL(0) += DatCam.ImUD[V0*ImCols + U0];
			// Left side of line
			U0 = (uint32_t)(u0 - dPix*VecPerp(0) + 0.5f);
			V0 = (uint32_t)(v0 - dPix*VecPerp(1) + 0.5f);
			AveRL(1) += DatCam.ImUD[V0*ImCols + U0];
		}
		AveRL = AveRL.mul(1.0 / (float32_t)nPts);

		// Find line projected onto the ground (z=0) in cAr Axle (A) coords
		mecl::core::Matrix<float32_t, 2, 1> uv1_C, uv2_C;
		uv1_C(0) = u1;
		uv2_C(0) = u2;
		uv1_C(1) = v1;
		uv2_C(1) = v2;
		mecl::core::Matrix<float32_t, 3, 1> xyz1_C = Project2D_to_3D(uv1_C, DatCam, 0); // 0 for no distortion
		mecl::core::Matrix<float32_t, 3, 1> xyz2_C = Project2D_to_3D(uv2_C, DatCam, 0); // 0 for no distortion
		mecl::core::Matrix<float32_t, 3, 1> xyzV1_A = R0_A_C.mmul(xyz1_C); // This is just a vector so only use rotation
		mecl::core::Matrix<float32_t, 3, 1> xyzV2_A = R0_A_C.mmul(xyz2_C); // This is just a vector so only use rotation
		float32_t Dist2Ground1 = -T_A_C_A(2) / xyzV1_A(2);
		float32_t Dist2Ground2 = -T_A_C_A(2) / xyzV2_A(2);
		mecl::core::Matrix<float32_t, 3, 1> xyzG1_A = T_A_C_A + xyzV1_A.mul(Dist2Ground1);
		mecl::core::Matrix<float32_t, 3, 1> xyzG2_A = T_A_C_A + xyzV2_A.mul(Dist2Ground2);

		// Determine BW direction & axis on the ground plane
		float32_t dX = xyzG2_A(0) - xyzG1_A(0);
		float32_t dY = xyzG2_A(1) - xyzG1_A(1);
		float32_t dAxis;
		if (fabs(dX) > fabs(dY)) // Aligned with x axis
		{
#ifndef PIKEOS
			SCORE_XY_axis[DL] = true;
#endif
			dAxis = dX;
			DLx_Idx.pushBack_v(DL);
		}
		else // Aligned with y axis
		{
#ifndef PIKEOS
			SCORE_XY_axis[DL] = false;
#endif
			dAxis = dY;
			DLy_Idx.pushBack_v(DL);
		}
		if (dAxis > 0) // Point 1 => point 2 goes in the same direction as the aligned (x/y) axis
		{
			if (AveRL(1) > AveRL(0)) // Left side is white & right side is black
				SCORE_BW_side[DL] = true;
			else
				SCORE_BW_side[DL] = false;
		}
		else
		{
			if (AveRL(1) > AveRL(0)) // Left side is white & right side is black
				SCORE_BW_side[DL] = false;
			else
				SCORE_BW_side[DL] = true;
		}

		// Record scoring metrics & other stored variables
		SCORE_LineLen[DL] = pow(LineLen2D, SCORE_LineLen_Exp);
		SCORE_LineColDiff[DL] = pow(fabs(AveRL(0) - AveRL(1)), SCORE_LineColDiff_Exp);
		xyz1_A[DL][0] = xyzG1_A(0);
		xyz1_A[DL][1] = xyzG1_A(1);
		xyz2_A[DL][0] = xyzG2_A(0);
		xyz2_A[DL][1] = xyzG2_A(1);
	}


	// Match detected (Det) lines <=> calibration target (Tar)
	int nDLx = DLx_Idx.size_u32();
	int nDLy = DLy_Idx.size_u32();
	int nTLx = CalibTLines.LinesLen[0];
	int nTLy = CalibTLines.LinesLen[1];

	// MD_MOD_02_27 start
	// All recorded in original detected line indexing (DL)
	mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> DetMatchAllX;   // [DetLine#] = Calib Target Line #           <- MatchesAll[x]
	mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> DetMatchAllY;   // [DetLine#] = Calib Target Line #           <- MatchesAll[y]
	mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> BestDetMatchX0; // [CalibTargetLine#] = Best Detected line #  <- Matches[x]
	mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> BestDetMatchY;  // [CalibTargetLine#] = Best Detected line #  <- Matches[y]
	for (int DL=0; DL<nDL; DL++)
	{
		DetMatchAllX.pushBack_v(-1);
		DetMatchAllY.pushBack_v(-1);
	}
	for (int TL = 0; TL<nTLx; TL++)
		BestDetMatchX0.pushBack_v(-1);
	for (int TL = 0; TL<nTLy; TL++)
		BestDetMatchY.pushBack_v(-1);

	if (TrustT_A_C_A) // Use the T_A_C_A to determine distance between projected lines & calib target
	{
		// Match xLines
		mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> ClosestMatchX; // Keep track of the closest match
		for (int TL = 0; TL<nTLx; TL++)
			ClosestMatchX.pushBack_v(Match_MaxOffsetDistXf+1);
		for (int DLx=0; DLx<nDLx; DLx++)
		{
			int DL = DLx_Idx[DLx];
			bool DL_BW_side = SCORE_BW_side[DL];
			float32_t DL_y = 0.5*(xyz1_A[DL][1] + xyz2_A[DL][1]); // Find the average y_A coordinate of the projected detected line
			for (int TL=0; TL<nTLx; TL++)
			{
				bool TL_BW_side = CalibTLines.Lines[0][TL][4] < 0.5;
				float32_t TL_y = CalibTLines.Lines[0][TL][2]; // = CalibTLines.Lines[0][TL][3]
				if ((TL_BW_side && DL_BW_side) || (!TL_BW_side && !DL_BW_side)) // Make sure the black-white stripe orientation matches
				{
					if (fabs(TL_y-DL_y) < Match_MaxOffsetDistXf)
					{
						DetMatchAllX[DL] = TL;
						if (ClosestMatchX[TL] > fabs(TL_y-DL_y))
						{
							ClosestMatchX[TL] = fabs(TL_y-DL_y);
							BestDetMatchX0[TL] = DL;
						}
					}
				}
			}
		}
		// Match yLines
		mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> ClosestMatchY; // Keep track of the closest match
		for (int TL = 0; TL<nTLy; TL++)
			ClosestMatchY.pushBack_v(Match_MaxOffsetDistY+1);
		for (int DLy=0; DLy<nDLy; DLy++)
		{
			int DL = DLy_Idx[DLy];
			bool DL_BW_side = SCORE_BW_side[DL];
			float32_t DL_x = 0.5*(xyz1_A[DL][0] + xyz2_A[DL][0]); // Find the average x_A coordinate of the projected detected line
			for (int TL=0; TL<nTLy; TL++)
			{
				bool TL_BW_side = CalibTLines.Lines[1][TL][4] > 0.5;
				float32_t TL_x = CalibTLines.Lines[1][TL][0]; // = CalibTLines.Lines[1][TL][1]
				if ((TL_BW_side && DL_BW_side) || (!TL_BW_side && !DL_BW_side)) // Make sure the black-white stripe orientation matches
				{
					if (fabs(TL_x-DL_x) < Match_MaxOffsetDistY)
					{
						DetMatchAllY[DL] = TL;
						if (ClosestMatchY[TL] > fabs(TL_x-DL_x))
						{
							ClosestMatchY[TL] = fabs(TL_x-DL_x);
							BestDetMatchY[TL] = DL;
						}
					}
				}
			}
		}
	}

	else // Shift projected lines to fit all lines well
	{
		// Match in x axis. CalibTLines.Lines[0][TL] from front camera: 0-5 are on the left side going right to left. 6-11 are on the right side going left to right.
		float32_t MatchOffsetX[vpcs::LINES_SIZE_MAX][vpcs::LINES_SIZE_MAX] = {NAN}; // Nan if match not possible, OffsetY distance otherwise
		for (int TL=0; TL<nTLx; TL++)
		{
			bool TL_BW_side = CalibTLines.Lines[0][TL][4] < 0.5;
			float32_t TL_y = CalibTLines.Lines[0][TL][2]; // = CalibTLines.Lines[0][TL][3]
			for (int DLx=0; DLx<nDLx; DLx++)
			{
				int DL = DLx_Idx[DLx];
				bool DL_BW_side = SCORE_BW_side[DL];
				if ((TL_BW_side && DL_BW_side) || (!TL_BW_side && !DL_BW_side)) // Make sure the black-white stripe orientation matches
				{
					float32_t DL_y = 0.5*(xyz1_A[DL][1] + xyz2_A[DL][1]); // Find the average y_A coordinate of the projected detected line
					if (fabs(TL_y-DL_y) < Match_MaxOffsetDistX) // Make sure the matched lines are close enough to each other => should limit each detected line to a max of 3 possible target line matches
						MatchOffsetX[TL][DLx] = TL_y - DL_y;
					else
						MatchOffsetX[TL][DLx] = NAN;
				}
				else
					MatchOffsetX[TL][DLx] = NAN;
			}
		}
		mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> BestDetMatchX0_; // Recorded in original detected line indexing (DL)
		mecl::core::ArrayList<float32_t, vpcs::LINES_SIZE_MAX> BestDetMatchScore;
		for (int TL = 0; TL<nTLx; TL++)
		{
			BestDetMatchX0_.pushBack_v(-1);
			BestDetMatchScore.pushBack_v(0);
		}
		float32_t BestScoreX0[2]; BestScoreX0[0]=INFINITY; BestScoreX0[1]=INFINITY;
		for (int TL=0; TL<nTLx; TL++)
		{
			int TL0_Start = 0;
			int TL0_End = nTLx/2;
			int LRIdx = 0;
			if (TL >= nTLx/2)
			{
				TL0_Start = nTLx/2;
				TL0_End = nTLx;
				LRIdx = 1;
			}
			for (int DLx=0; DLx<nDLx; DLx++)
			{
				if (!isnan(MatchOffsetX[TL][DLx]))
				{
					// For each possible match, try matching that offset and checking how well the other lines match
					float32_t ScoreTot = 0; // Left & right side of car
					mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> DetMatchX0, DetMatchX0_;
					mecl::core::ArrayList<float32_t, vpcs::LINES_SIZE_MAX> DetMatchScore;
					for (int TL1 = 0; TL1<nTLx; TL1++)
					{
						DetMatchX0.pushBack_v(-1);
						DetMatchX0_.pushBack_v(-1);
						DetMatchScore.pushBack_v(0);
					}
					for (int TL0=TL0_Start; TL0<TL0_End; TL0++)
					{
						float32_t ScoreTL = Match_MaxOffsetDistXf; // Minimum offset error for this map line
						for (int DL0x=0; DL0x<nDLx; DL0x++)
						{
							if (!isnan(MatchOffsetX[TL0][DL0x]))
							{
								float32_t OffsetEr = fabs( MatchOffsetX[TL0][DL0x] - MatchOffsetX[TL][DLx] );
								if (OffsetEr < ScoreTL)
								{
									ScoreTL = OffsetEr;
									DetMatchX0[TL0] = DLx_Idx[DL0x];
									DetMatchX0_[TL0] = DL0x;
									DetMatchScore[TL0] = ScoreTL;
								}
							}
						}
						ScoreTot += ScoreTL;
					}
					if (ScoreTot < BestScoreX0[LRIdx])
					{
						BestScoreX0[LRIdx] = ScoreTot;
						for (int TL0=TL0_Start; TL0<TL0_End; TL0++)
						{
							BestDetMatchX0[TL0] = DetMatchX0[TL0];
							BestDetMatchX0_[TL0] = DetMatchX0_[TL0];
							BestDetMatchScore[TL0] = DetMatchScore[TL0];
						}
					}
				}
			}
		}
		// Find the longest segment
		float32_t MaxOffsetEr = 30;
		for (int TL=0; TL<nTLx; TL++)
		{
			if (BestDetMatchX0[TL] != -1)
			{
				int DL = BestDetMatchX0[TL];
				int DLx = BestDetMatchX0_[TL];
				float32_t OffsetInit = MatchOffsetX[TL][DLx];
				float32_t ScoreInit = SCORE_LineLen[DL];
				for (int DL0x=0; DL0x<nDLx; DL0x++)
				{
					if(!isnan(MatchOffsetX[TL][DL0x]) && fabs(MatchOffsetX[TL][DL0x]-OffsetInit)<MaxOffsetEr && SCORE_LineLen[DLx_Idx[DL0x]]>ScoreInit)
					{
						BestDetMatchX0[TL] = DLx_Idx[DL0x];
						BestDetMatchX0_[TL] = DL0x;
						ScoreInit = SCORE_LineLen[DLx_Idx[DL0x]];
					}
				}
			}
		}
		// Record all segments which match
		for (int TL=0; TL<nTLx; TL++) // Target line idx
		{
			if (BestDetMatchX0[TL] != -1)
			{
				int DLx = BestDetMatchX0_[TL];
				float32_t OffsetInit = MatchOffsetX[TL][DLx];
				for (int DL0x=0; DL0x<nDLx; DL0x++)
				{
					if (!isnan(MatchOffsetX[TL][DL0x]) && fabs(MatchOffsetX[TL][DL0x]-OffsetInit)<MaxOffsetEr)
					{
						int DL = DLx_Idx[DL0x]; // Detected line idx
						DetMatchAllX[DL] = TL;
					}
				}
			}
		}

	// Match in y axis 
	float32_t SCORE_Overall_y[vpcs::LINES_SIZE_MAX][vpcs::LINES_SIZE_MAX] = { 0 };
	float32_t BestScoreY = 0;
	for (int i = 0; i < CalibTLines.yCamSeeLen[CalibTLines.CamNum]; i++)
	{
		int TL = CalibTLines.yCamSee[CalibTLines.CamNum][i];
		bool TL_BW_side = CalibTLines.Lines[1][TL][4] > 0.5;
		float32_t TL_x = CalibTLines.Lines[1][TL][0]; // = CalibTLines.Lines[1][TL][1]
		for (int DLy = 0; DLy < nDLy; DLy++)
		{
			int DL = DLy_Idx[DLy];
			bool DL_BW_side = SCORE_BW_side[DL];
			if ((TL_BW_side && DL_BW_side) || (!TL_BW_side && !DL_BW_side)) // Make sure the black-white stripe orientation matches
			{
				float32_t DL_x = 0.5*(xyz1_A[DL][0] + xyz2_A[DL][0]); // Find the average x_A coordinate of the projected detected line
				if (fabs(TL_x - DL_x) < Match_MaxOffsetDistY) // Make sure the matched lines are close enough to each other => should limit each detected line to a max of 3 possible target line matches
				{


					// LATER ADD CODE TO AUTO-MATCH BASED ON LINE SPACING NOT ASSUMING THAT THE 3D VP EXTRINSIC FIT IS GOOD ENOUGH


					SCORE_Overall_y[TL][DLy] = SCORE_LineLen[DL] * SCORE_LineColDiff[DL] * 1.0;
					BestScoreY = mecl::math::max_x<float32_t>(BestScoreY, SCORE_Overall_y[TL][DLy]);
				}
			}
		}
	}
	// Select the best detected line matches for each target line
	float32_t MinScoreY = MinScoreCompBestMatch * BestScoreY;
	for (int TL = 0; TL < nTLy; TL++)
	{
		float32_t BestScore = MinScoreY;
		for (int DLy = 0; DLy < nDLy; DLy++)
		{
			if (SCORE_Overall_y[TL][DLy] > BestScore)
			{
				BestScore = SCORE_Overall_y[TL][DLy];
				BestDetMatchY[TL] = DLy_Idx[DLy];
			}
		}
	}
	// MD_MOD_OPTIM_ALL_LINES start
	// Record all segments which match
	for (int TL = 0; TL < nTLy; TL++)
	{
		if (BestDetMatchY[TL] != -1)
		{
			int DL = BestDetMatchY[TL]; // Detected line Idx
			DetMatchAllY[DL] = TL;
		}
	}
	}

	// MD_MOD_2021_03_01 start
	// For yLines only choose the single best line because otherwise we might get some strange stuff
	for (int TL=0; TL<nTLy; TL++)
	{
		if (BestDetMatchY[TL] > -1)
		{
			for (int DLy=0; DLy<nDLy; DLy++)
			{
				int DL = DLy_Idx[DLy];
				if (DetMatchAllY[DL]==TL && BestDetMatchY[TL]!=DL)
					DetMatchAllY[DL] = -1;
			}
		}
	}
	// MD_MOD_2021_03_01 end

	MatchesAll[0].clear_v();
	MatchesAll[1].clear_v();
	MatchesAll[0].copy_v(DetMatchAllX, nDL);
	MatchesAll[1].copy_v(DetMatchAllY, nDL);
	// MD_MOD_OPTIM_ALL_LINES end


	// Save result
	Matches[0].clear_v();
	Matches[1].clear_v();
	Matches[0].copy_v(BestDetMatchX0, nTLx);
	Matches[1].copy_v(BestDetMatchY, nTLy);


	// MD_PRINT start
#ifdef PIKEOS
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"BestScoreX0 = [%f, %f]\n", BestScoreX0[0], BestScoreX0[1]
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"[BestDetMatchX0, BestDetMatchX0_, BestDetMatchScore]:\n"
	);
	for (int i = 0; i < nTLx; i++)
		log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"   [%f, %f, %f]\n", (float32_t)BestDetMatchX0[i], (float32_t)BestDetMatchX0_[i], (float32_t)BestDetMatchScore[i]
		);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"BestScoreY = %f\n", BestScoreY
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"BestDetMatchY = ["
	);
	for (int i = 0; i < nTLy; i++)
		log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
			"%f, ", (float32_t)BestDetMatchY[i]
		);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"]\n"
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"ExRefine::MatchLines end\n\n"
	);
#else
	//vm_cprintf("BestScoreX0 = [%f, %f]\n",BestScoreX0[0],BestScoreX0[1]);
	vm_cprintf("[BestDetMatchX0, BestDetMatchX0_, BestDetMatchScore]:\n");
	//fprintf(fp0, "BestScoreX0 = [%f, %f]\n", BestScoreX0[0], BestScoreX0[1]);
	fprintf(fp0, "[BestDetMatchX0, BestDetMatchX0_, BestDetMatchScore]:\n");
	//for (int i = 0; i < nTLx; i++)
	//{
	//	vm_cprintf("   [%f, %f, %f]\n", (float32_t)BestDetMatchX0[i], (float32_t)BestDetMatchX0_[i], (float32_t)BestDetMatchScore[i]);
	//	fprintf(fp0, "   [%f, %f, %f]\n", (float32_t)BestDetMatchX0[i], (float32_t)BestDetMatchX0_[i], (float32_t)BestDetMatchScore[i]);
	//}
	//vm_cprintf("BestScoreY = %f\n", BestScoreY);
	//fprintf(fp0, "BestScoreY = %f\n", BestScoreY);
	vm_cprintf("BestDetMatchY = [");
	fprintf(fp0, "BestDetMatchY = [");
	for (int i = 0; i < nTLy; i++)
	{
		vm_cprintf("%f, ", (float32_t)BestDetMatchY[i]);
		fprintf(fp0,"%f, ", (float32_t)BestDetMatchY[i]);
	}
	vm_cprintf("]\n");
	fprintf(fp0,"]\n");
	vm_cprintf("ExRefine::MatchLines end\n\n");
	fprintf(fp0,"ExRefine::MatchLines end\n\n");
#endif
	// MD_PRINT end


	// MD_MOD_3LINES start
	// Number of line matches
	int nLMx = 0;
	int nLMy = 0;
	for (uint32_t j=0; j<Matches[0].size_u32(); j++)
	{
		if (Matches[0][j] > -1)
			nLMx++;
	}
	for (uint32_t j=0; j<Matches[1].size_u32(); j++)
	{
		if (Matches[1][j] > -1)
			nLMy++;
	}	

	// Debug figure showing matches
	#ifdef OPENCV_OUT
		cv::Mat Im3;
		cv::Mat Im0(ImRows, ImCols, CV_8UC1);
		for (int row=0; row<ImRows; row++)
		{
			for (int col=0; col<ImCols; col++)
			{
				Im0.at<uint8_t>(row,col) = DatCam.ImUD[row*ImCols+col];
			}
		}
		cv::cvtColor(Im0, Im3, CV_GRAY2BGR);
		cv::Scalar Col;
		float32_t x1, x2, y1, y2;
		for (int DLx=0; DLx<nDLx; DLx++)
		{
			int DL = DLx_Idx[DLx];
			Col = cv::Scalar(0,0,0); // Black
			int LineWidth = 1;
			x1 = DatCam.Lines_EndPts[DL][0];
			x2 = DatCam.Lines_EndPts[DL][1];
			y1 = DatCam.Lines_EndPts[DL][2];
			y2 = DatCam.Lines_EndPts[DL][3];
			cv::line(Im3, cv::Point(x1,y1), cv::Point(x2,y2), Col, LineWidth, cv::LINE_AA);
			char TextX[100];
			sprintf(TextX,"%d", DL);
			cv::putText(Im3, TextX, cv::Point(0.5*(x1+x2),0.5*(y1+y2)), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,0,0), 1);
		}
		// MD_MOD_OPTIM_ALL_LINES start
		for (int xy=0; xy<2; xy++)
		{
			for (int DL=0; DL<nDL; DL++)
			{
				int TL = MatchesAll[xy][DL];
				int nCol = 6;
				int TLcol = TL;
				if (TLcol >= nTLx/2)
					TLcol -= nTLx/2;
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
				int LineWidth = 2;
				x1 = DatCam.Lines_EndPts[DL][0];
				x2 = DatCam.Lines_EndPts[DL][1];
				y1 = DatCam.Lines_EndPts[DL][2];
				y2 = DatCam.Lines_EndPts[DL][3];
				cv::line(Im3, cv::Point(x1,y1), cv::Point(x2,y2), Col, LineWidth, cv::LINE_AA);
			}
		}
		// Print text in top left corner: stats
		int ySpace = 15;
		int yStart = 20;
		int xStart = 10;
		if (CalibTLines.nLines == NUM_LINES::TWO_LINES)
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
		/*cv::imshow("MD_Im000", Im3);
		cvWaitKey(1);*/
		// Save figure
		std::string CamNum = std::to_string(DatCam.CamNum);
		std::string SepDist = std::to_string(int(CalibTLines.SepDist));
		std::string Iter = TrustT_A_C_A ? "1" : "0";
		std::string ImageFile = "Images/Debug/Match_Cam" + CamNum + "_SepDist" + SepDist + "_Iter" + Iter + ".png";
		cv:imwrite(ImageFile, Im3);
	#endif
	// MD_MOD_3LINES end
}






// Refine extrinsics => return: Refined_R_A_C & Refined_R_C_A & Refined_xyz_CamPos. A = cAr Axle refFrame, C = camera refFrame
// MD_MOD_OPTIM_ALL_LINES start
// If MatchAllDL==true  -> Matches=MatchesAll in MatchLines()
// If MatchAllDL==false -> Matches=Matches    in MatchLines()
void ExRefine::RefineExtrinsics(MD_Data &DatCam, TargetLines &CalibTLines, mecl::core::ArrayList<sint32_t, vpcs::LINES_SIZE_MAX> *Matches, float64_t &TotEr, bool MatchAllDL, FILE *fp0)
// MD_MOD_OPTIM_ALL_LINES end
{
	// Settings
	int MaxIter = 20;
    float64_t MinStepSize = 1e-11;
    float64_t dRho = 1e-5;
	bool ErUseInitRT = true; // If true, then the original DatCam.Orig_xyz_CamPos matters and the distance from that is an error metric
	// Settings: weights
	float64_t T_WeightMult = 10.0;
	float64_t AveMainErDeg = 0.1; // Max expected error in degrees for the main fitting code
	float64_t AveR_ErDeg = 2.0; // Max expected error for rho R adjustment
	float64_t AveT_ErMM = 30; // Max expected error for T adjustment
	float64_t MainErRatio = 10.0; // Main error should be this much more weighted on average. LOWER THIS VALUE IF YOU TRUST THE HAND MEASUREMENT OF THE CAMERA POSITION
	//float64_t MainErRatio = 0.2; // Main error should be this much more weighted on average. LOWER THIS VALUE IF YOU TRUST THE HAND MEASUREMENT OF THE CAMERA POSITION
	// Setup
	float64_t dRho2 = pow(dRho, 2.0);
    float64_t d2Rho = 2.0*dRho;
    float64_t dRhoInv = 1.0/dRho;
    float64_t dRho2Inv = 1.0/dRho2;
	mecl::core::Matrix<float64_t,3,3> R0_C_A; // Initial approximate rotation matrix
	mecl::core::Matrix<float64_t,3,1> T0_A_C_A; // Initial approximate translation vector (placement of camera in A refFrame)
	for (int i=0; i<3; i++)
	{
		T0_A_C_A(i) = (float64_t)DatCam.Orig_xyz_CamPos(i);
		for (int j=0; j<3; j++)
		{
			R0_C_A(i,j) = (float64_t)DatCam.VP3D_R_A_C(j,i);
		}
	}
	mecl::core::Matrix<float64_t,3,1> T0_C_A_C = -R0_C_A.mmul(T0_A_C_A);
	int nL = 0; // Number of line matches
	for (int i=0; i<2; i++) // x/y
	{
		for (uint32_t j=0; j<Matches[i].size_u32(); j++)
		{
			if (Matches[i][j] > -1)
				nL++;
		}
	}
	// Weights setup
	mecl::core::Matrix<float64_t,3,1> Weights; // Relative weight of [Main fit, dRho, dT] error
	float64_t AveMainEr = pow( sin(AveMainErDeg/180.0*M_PI) , 2.0) * nL;
	float64_t AveREr = pow( AveR_ErDeg/180.0*M_PI , 2.0);
	float64_t AveTEr = pow( AveT_ErMM , 2.0);
	Weights(0) = 1.0;
	Weights(1) = AveMainEr / MainErRatio / AveREr; // dRho error weight
	Weights(2) = AveMainEr / MainErRatio / AveTEr; // dT error weight

	// Read in 2D (detected line) & 3D (calibration target) line info
	mecl::core::Matrix<float64_t,3,1> n_C[100];  // 2D: normal vector to projection plane
	mecl::core::Matrix<float64_t,3,1> c_C[100];  // 2D: vector extending out from midpoint of line
	mecl::core::Matrix<float64_t,3,1> v_A[100];  // 3D: normal vector parallel to line
	mecl::core::Matrix<float64_t,3,1> p0_A[100]; // 3D: any point on the line
	mecl::core::Matrix<float64_t,3,1> p1_A[100]; // 3D: best point on the line (closest to c_C)
	mecl::core::Matrix<bool,1,1> UseTrans[100];
	int Cnt = 0;
	for (int i=0; i<2; i++) // x/y
	{
		// MD_MOD_OPTIM_ALL_LINES start
		for (uint32_t j=0; j<Matches[i].size_u32(); j++) // For each CalibTLine if MatchAllDL==false   OR   For each DetLine if MatchAllDL==true
		{
			int DL, TL; // DL = DetectedLine, TL = TargetLine
			if (MatchAllDL)
			{
				DL = j;
				TL = Matches[i][j];
			}
			else
			{
				DL = Matches[i][j];
				TL = j;
			}
			if (Matches[i][j] > -1)
			{
				// MD_MOD_OPTIM_ALL_LINES end
#ifndef NO_XPOS_IMPROVE
				// y target Line unreliable for front & rear cameras
				if ( ((DatCam.CamNum == 0) || (DatCam.CamNum == 2)) && i==1)
					UseTrans[Cnt](0) = false;
				else
					UseTrans[Cnt](0) = true;
#else
				// y target Line unreliable for front & rear cameras
				
					UseTrans[Cnt](0) = true;
				
#endif

				// 2D line info
				mecl::core::Matrix<float64_t,2,1> uv1_C, uv2_C, uvC_C;
				float64_t u1 = (float64_t)DatCam.Lines_EndPts[DL][0];
				float64_t u2 = (float64_t)DatCam.Lines_EndPts[DL][1];
				float64_t v1 = (float64_t)DatCam.Lines_EndPts[DL][2];
				float64_t v2 = (float64_t)DatCam.Lines_EndPts[DL][3];
				uv1_C(0) = u1;
				uv2_C(0) = u2;
				uv1_C(1) = v1;
				uv2_C(1) = v2;
				uvC_C(0) = 0.5*(u1+u2);
				uvC_C(1) = 0.5*(v1+v2);
				mecl::core::Matrix<float64_t,3U,1U> xyz1_C = Project2D_to_3D(uv1_C,  DatCam, 0U); // 0 for no distortion
				mecl::core::Matrix<float64_t,3U,1U> xyz2_C = Project2D_to_3D(uv2_C,  DatCam, 0U);
				c_C[Cnt] = Project2D_to_3D(uvC_C,  DatCam, 0U);
				n_C[Cnt] = CrossProd(xyz1_C, xyz2_C);
				n_C[Cnt] = n_C[Cnt].mul(1.0/n_C[Cnt].norm());

				// 3D line info
				// MD_MOD_OPTIM_ALL_LINES start
				float64_t x1_A = (float64_t)CalibTLines.Lines[i][TL][0];
				float64_t x2_A = (float64_t)CalibTLines.Lines[i][TL][1];
				float64_t y1_A = (float64_t)CalibTLines.Lines[i][TL][2];
				float64_t y2_A = (float64_t)CalibTLines.Lines[i][TL][3];
				// MD_MOD_OPTIM_ALL_LINES end
				v_A[Cnt](0) = x2_A - x1_A;
				v_A[Cnt](1) = y2_A - y1_A;
				v_A[Cnt](2) = 0;
				v_A[Cnt] = v_A[Cnt].mul(1.0/v_A[Cnt].norm());
				p0_A[Cnt](0) = x1_A;
				p0_A[Cnt](1) = y1_A;
				p0_A[Cnt](2) = 0;

				// Solve for p1_A by making the 3x3 matrix CV = [c_C , c_C x v_C0 , v_C0]
				mecl::core::Matrix<float64_t,3,1> v_C0 = R0_C_A.mmul(v_A[Cnt]); // C0 = approx
				mecl::core::Matrix<float64_t,3,1> cv_C0 = CrossProd(c_C[Cnt], v_C0);
				float64_t CV11 = c_C[Cnt](0);
				float64_t CV21 = c_C[Cnt](1);
				float64_t CV31 = c_C[Cnt](2);
				float64_t CV12 = cv_C0(0);
				float64_t CV22 = cv_C0(1);
				float64_t CV32 = cv_C0(2);
				float64_t CV13 = v_C0(0);
				float64_t CV23 = v_C0(1);
				float64_t CV33 = v_C0(2);
				// Find inverse of CV 3x3 matrix = M*det(CV) (not entire inverse is needed)
				float64_t M11 = CV22*CV33 - CV23*CV32;
				float64_t M12 = CV12; // CV21*CV33 - CV31*CV23;
				float64_t M13 = CV21*CV32 - CV31*CV22;
				// float64_t M21 = CV12*CV33 - CV32*CV13;
				// float64_t M22 = -CV22; // CV11*CV33 - CV31*CV13;
				float64_t M23 = CV11*CV32 - CV31*CV12;
				// float64_t M31 = CV12*CV23 - CV22*CV13;
				// float64_t M32 = CV32; // CV11*CV23 - CV21*CV13;
				float64_t M33 = CV11*CV22 - CV21*CV12;
				float64_t Det = CV11*M11 - CV12*M12 + CV13*M13;
				// Find t3 needed to adjust p0_A => p1_A
				mecl::core::Matrix<float64_t,3,1> p0_C0 = (R0_C_A.mmul(p0_A[Cnt])).add(T0_C_A_C);
				float64_t t3 = (M13*p0_C0(0) - M23*p0_C0(1) + M33*p0_C0(2)) / Det;
				p1_A[Cnt] = p0_A[Cnt] - (v_A[Cnt] * t3);

				Cnt++;
			}
		}
	}

	// Make A & B matrices
	mecl::core::Matrix<float64_t,1,9> A_Rot[100];   // nL x 9 matrix
	mecl::core::Matrix<float64_t,1,9> A_Trans[100]; // nL x 9 matrix
	mecl::core::Matrix<float64_t,1,3> B_Trans[100]; // nL x 3 matrix
	for (int i=0; i<nL; i++)
	{
		// Transform variables
		mecl::core::Matrix<float64_t,3,1> v_C0 = R0_C_A.mmul(v_A[i]); // C0 = approx
		mecl::core::Matrix<float64_t,3,1> p1_C1 = R0_C_A.mmul(p1_A[i]); // C1 = approx & no translation
		mecl::core::Matrix<float64_t,3,1> p1_C0 = p1_C1.add(T0_C_A_C); // C0 = approx

		// Rotational constraints
		float64_t nx = n_C[i](0);
		float64_t ny = n_C[i](1);
		float64_t nz = n_C[i](2);
		float64_t vx = v_C0(0);
		float64_t vy = v_C0(1);
		float64_t vz = v_C0(2);
		A_Rot[i](0) = nx * vx;
		A_Rot[i](1) = ny * vx;
		A_Rot[i](2) = nz * vx;
		A_Rot[i](3) = nx * vy;
		A_Rot[i](4) = ny * vy;
		A_Rot[i](5) = nz * vy;
		A_Rot[i](6) = nx * vz;
		A_Rot[i](7) = ny * vz;
		A_Rot[i](8) = nz * vz;
		//B_Rot[i](0) = 0;
		//B_Rot[i](1) = 0;
		//B_Rot[i](2) = 0;

		// Translational constraints
		float64_t p1_C0_norm = p1_C0.norm();
		float64_t Nx = nx / p1_C0_norm;
		float64_t Ny = ny / p1_C0_norm;
		float64_t Nz = nz / p1_C0_norm;
		float64_t Px = p1_C1(0);
		float64_t Py = p1_C1(1);
		float64_t Pz = p1_C1(2);
		A_Trans[i](0) = Nx * Px;
		A_Trans[i](1) = Ny * Px;
		A_Trans[i](2) = Nz * Px;
		A_Trans[i](3) = Nx * Py;
		A_Trans[i](4) = Ny * Py;
		A_Trans[i](5) = Nz * Py;
		A_Trans[i](6) = Nx * Pz;
		A_Trans[i](7) = Ny * Pz;
		A_Trans[i](8) = Nz * Pz;
		B_Trans[i](0) = Nx;
		B_Trans[i](1) = Ny;
		B_Trans[i](2) = Nz;
	}
	
	// Get 9x9 E matrix = D' * D
	mecl::core::Matrix<float64_t,3,3> BB(0); // Get BB = B' * B for translation only
	mecl::core::Matrix<float64_t,3,9> BA(0); // Get BB = B' * A for translation only
	for (int i=0; i<nL; i++)
	{
		mecl::core::Matrix<float64_t,3,3> BB_i = (B_Trans[i].t()).mmul(B_Trans[i]);
		mecl::core::Matrix<float64_t,3,9> BA_i = (B_Trans[i].t()).mmul(A_Trans[i]);
		if (UseTrans[i](0))
		{
			BB = BB.add(BB_i);
			BA = BA.add(BA_i);
		}
	}
	// Compute inv(BB) for translation only
	mecl::core::Matrix<float64_t,3,3> BBinv = Inverse3x3Mat(BB);
	// Compute inv(BB) * BA
	mecl::core::Matrix<float64_t,3,9> BBinvBA = BBinv.mmul(BA);
	// Compute D_Trans = A - B * inv(B' * B) * B' * A
	mecl::core::Matrix<float64_t,1,9> D_Trans[vpcs::LINES_SIZE_MAX]; // nL x 9
	for (int i=0; i<nL; i++)
	{
		D_Trans[i] = A_Trans[i].sub(B_Trans[i].mmul(BBinvBA));
	}
	// Get E_Trans & E_Rot
	mecl::core::Matrix<float64_t,9,9> E_Trans(0), E_Rot(0);
	for (int i=0; i<nL; i++)
	{
		if (UseTrans[i](0))
			E_Trans = E_Trans.add((D_Trans[i].t()).mmul(D_Trans[i]));
		E_Rot = E_Rot.add((A_Rot[i].t()).mmul(A_Rot[i]));
	}
	for (int i=0; i<9; i++)
	{
		for (int j=0; j<9; j++)
			E_Trans(i,j) = T_WeightMult*E_Trans(i,j);
	}
	mecl::core::Matrix<float64_t,9,9> E = E_Trans.add(E_Rot);











	// While loop to converge
    float64_t StepSize = mecl::math::numeric_limits<float64_t>::infinity_x();
    int Iter = 1;
    float64_t F0, F1, F2, F3, Step0;
    mecl::core::Matrix<float64_t,3,1> F_Grad, F_GradUpdate;
    mecl::core::Matrix<float64_t,3,3> F_Hess, F_HessInv;
    float64_t rho1=0, rho2=0, rho3=0;
    float64_t rhoNew1, rhoNew2, rhoNew3;
    while(Iter<=MaxIter && StepSize>MinStepSize)
    {
		if (ErUseInitRT)
		{
			F0 = F_CGR(BBinvBA,T0_C_A_C,Weights, rho1,rho2,rho3,E); // This equals the error for this step
			F1 = F_CGR(BBinvBA,T0_C_A_C,Weights, rho1+dRho,rho2,rho3,E);
			F2 = F_CGR(BBinvBA,T0_C_A_C,Weights, rho1,rho2+dRho,rho3,E);
			F3 = F_CGR(BBinvBA,T0_C_A_C,Weights, rho1,rho2,rho3+dRho,E);
			F_Hess(0,0) = (F_CGR(BBinvBA,T0_C_A_C,Weights, rho1+d2Rho,rho2,rho3,E) - F1-F1 + F0) * dRho2Inv; // Hessian(F) See: https://v8doc.sas.com/sashtml/ormp/chap5/sect28.htm
			F_Hess(1,1) = (F_CGR(BBinvBA,T0_C_A_C,Weights, rho1,rho2+d2Rho,rho3,E) - F2-F2 + F0) * dRho2Inv;
			F_Hess(2,2) = (F_CGR(BBinvBA,T0_C_A_C,Weights, rho1,rho2,rho3+d2Rho,E) - F3-F3 + F0) * dRho2Inv;
			F_Hess(0,1) = (F_CGR(BBinvBA,T0_C_A_C,Weights, rho1+dRho,rho2+dRho,rho3,E) - F1-F2 + F0) * dRho2Inv;
			F_Hess(0,2) = (F_CGR(BBinvBA,T0_C_A_C,Weights, rho1+dRho,rho2,rho3+dRho,E) - F1-F3 + F0) * dRho2Inv;
			F_Hess(1,2) = (F_CGR(BBinvBA,T0_C_A_C,Weights, rho1,rho2+dRho,rho3+dRho,E) - F2-F3 + F0) * dRho2Inv;
		}
		else
		{
			F0 = F_CGR(rho1,rho2,rho3,E); // This equals the error for this step
			F1 = F_CGR(rho1+dRho,rho2,rho3,E);
			F2 = F_CGR(rho1,rho2+dRho,rho3,E);
			F3 = F_CGR(rho1,rho2,rho3+dRho,E);
			F_Hess(0,0) = (F_CGR(rho1+d2Rho,rho2,rho3,E) - F1-F1 + F0) * dRho2Inv; // Hessian(F) See: https://v8doc.sas.com/sashtml/ormp/chap5/sect28.htm
			F_Hess(1,1) = (F_CGR(rho1,rho2+d2Rho,rho3,E) - F2-F2 + F0) * dRho2Inv;
			F_Hess(2,2) = (F_CGR(rho1,rho2,rho3+d2Rho,E) - F3-F3 + F0) * dRho2Inv;
			F_Hess(0,1) = (F_CGR(rho1+dRho,rho2+dRho,rho3,E) - F1-F2 + F0) * dRho2Inv;
			F_Hess(0,2) = (F_CGR(rho1+dRho,rho2,rho3+dRho,E) - F1-F3 + F0) * dRho2Inv;
			F_Hess(1,2) = (F_CGR(rho1,rho2+dRho,rho3+dRho,E) - F2-F3 + F0) * dRho2Inv;
		}
        F_Hess(1,0)=F_Hess(0,1); F_Hess(2,0)=F_Hess(0,2); F_Hess(2,1)=F_Hess(1,2);
		F_Grad(0) = (F1-F0) * dRhoInv; // Gradient(F)
        F_Grad(1) = (F2-F0) * dRhoInv;
        F_Grad(2) = (F3-F0) * dRhoInv;
		F_HessInv = Inverse3x3Mat(F_Hess);
        F_GradUpdate = F_HessInv.mmul(F_Grad);
        rhoNew1 = rho1 - F_GradUpdate(0);
        rhoNew2 = rho2 - F_GradUpdate(1);
        rhoNew3 = rho3 - F_GradUpdate(2);
        Step0 = pow( (rho1-rhoNew1)*(rho1-rhoNew1) + (rho2-rhoNew2)*(rho2-rhoNew2) + (rho3-rhoNew3)*(rho3-rhoNew3) , 0.5);
        if(Step0 > StepSize) // Diverging! Stop!
            Iter = MaxIter + 1;
        else
        {
            StepSize = Step0;
            Iter += 1;
        }
        rho1 = rhoNew1;
        rho2 = rhoNew2;
        rho3 = rhoNew3;
    }
    mecl::core::Matrix<float64_t,3,1> rho;
	rho(0) = rho1;
	rho(1) = rho2;
	rho(2) = rho3;
	// MD_MOD_3LINE_FIX1 start
	//TotEr = F_CGR(rho1,rho2,rho3,E_Trans);
	TotEr = F_CGR(BBinvBA,T0_C_A_C,Weights, rho1,rho2,rho3,E_Trans);
	// MD_MOD_3LINE_FIX1 start

    // Compute final solution
    mecl::core::Matrix<float64_t,9,1> r = CGR(rho);
    mecl::core::Matrix<float64_t,3,3> R_Adjust = r9_to_R3(r);
    mecl::core::Matrix<float64_t,3,3> R_C_A = R_Adjust.mmul(R0_C_A);
	mecl::core::Matrix<float64_t,3,3> R_A_C = R_C_A.t();
	mecl::core::Matrix<float64_t,3,1> T = -BBinvBA.mmul(r);
	mecl::core::Matrix<float64_t,3,1> T_A_C_A = -R_A_C.mmul(T);
	
	// Record final solution
	for (int i=0; i<3; i++)
	{
		DatCam.Refined_xyz_CamPos(i) = (float64_t)T_A_C_A(i);
		for (int j=0; j<3; j++)
		{
			DatCam.Refined_R_A_C(i,j) = (float64_t)R_A_C(i,j);
		}
	}

	// Error line by line for debugging
	float64_t Error_DotProd[100];
	for (int i=0; i<nL; i++)
	{
		mecl::core::Matrix<float64_t,1,1> aa = A_Trans[i].mmul(r);
		mecl::core::Matrix<float64_t,1,1> bb = B_Trans[i].mmul(T);
		Error_DotProd[i] = aa(0,0) + bb(0,0);
	}



// Error plot
#ifndef OPENCV_OUT
#include <cv.h>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#endif
#ifdef OPENCV_OUT
	bool PltInit = true;
	float32_t LineLen1 = 200;
	float32_t LineLen2 = 200;
	cv::Mat Im3;
	uint32_t ImRows = vpcs::c_ImageHeight_u32;
	uint32_t ImCols = vpcs::c_ImageWidth_u32;
	cv::Mat Im0(ImRows, ImCols, CV_8UC1);
	cv::cvtColor(Im0, Im3, CV_GRAY2BGR);
	cv::Scalar Col;
	int LineWidth;
	float32_t x1, x2, y1, y2;
	int nLMx = 0;
	int nLMy = 0;
	for (int i=0; i<2; i++) // x/y
	{
		int nTL = CalibTLines.LinesLen[i];
		// MD_MOD_OPTIM_ALL_LINES start
		for (uint32_t j=0; j<Matches[i].size_u32(); j++) // For each CalibTLine if MatchAllDL==false   OR   For each DetLine if MatchAllDL==true
		{
			if (Matches[i][j] == -1)
				continue;
			int DL, TL; // DL = DetectedLine, TL = TargetLine
			if (MatchAllDL)
			{
				DL = j;
				TL = Matches[i][j];
			}
			else
			{
				DL = Matches[i][j];
				TL = j;
			}
			if (i==0) nLMx++;
			if (i==1) nLMy++;
			int nCol = 6;
			int TLcol = TL;
			// MD_MOD_OPTIM_ALL_LINES end
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
	mecl::core::Matrix<float64_t,1U,1U> r2;
	LineWidth = 1;
	for (int i=0; i<nL; i++)
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
	if (CalibTLines.nLines == NUM_LINES::TWO_LINES)
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
	char TextLab[100];
	sprintf(TextLab,"White = init pose, Black = final pose");
	cv::putText(Im3, TextLab, cv::Point(xStart,yStart+ySpace*4), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0,0,255), 1);
	// Display figure
	//cv::imshow("MD_ImFit", Im3);
	// Save figure
	std::string CamNum = std::to_string(DatCam.CamNum);
	std::string SepDist = std::to_string(int(CalibTLines.SepDist));
	std::string ImageFile = "Images/Debug/Refine_Cam" + CamNum + "_SepDist" + SepDist + ".png";
	cv:imwrite(ImageFile, Im3);
#endif

#ifdef PIKEOS
	logging::LogInstance log_ro;


	// MD_PRINT start
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"\n\nExRefine::RefineExtrinsics end\n"
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"R0_C_A: "
	);
	PrintR_(R0_C_A);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"T0_A_C_A: "
	);
	PrintT_(T0_A_C_A);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Weights: "
	);
	PrintT_(Weights);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"BB: "
	);
	PrintR_(BB);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"BA: "
	);
	PrintM39_(BA);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"BBinv: "
	);
	PrintR_(BBinv);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"BBinvBA: "
	);
	PrintM39_(BBinvBA);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"E_Trans: "
	);
	PrintE_(E_Trans);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"E_Rot: "
	);
	PrintE_(E_Rot);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"E: "
	);
	PrintE_(E);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"Iter: %f\n", (float32_t)Iter
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"StepSize: %f\n", (float32_t)StepSize
	);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"rho: "
	);
	PrintT_(rho);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"R_Adjust: "
	);
	PrintR_(R_Adjust);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"R_C_A: "
	);
	PrintR_(R_C_A);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"T: "
	);
	PrintT_(T);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"T_A_C_A: "
	);
	PrintT_(T_A_C_A);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"DatCam.Refined_xyz_CamPos: "
	);
	PrintT_(DatCam.Refined_xyz_CamPos);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"DatCam.Refined_R_A_C: "
	);
	PrintR_(DatCam.Refined_R_A_C);
	log_ro.getInstance_rx().logError_v(logging::LogCtx::c_LogCtxDefault_u32,
		"ExRefine::RefineExtrinsics end\n\n"
	);
#else
	vm_cprintf("\n\nExRefine::RefineExtrinsics end\n");
	fprintf(fp0,"\n\nExRefine::RefineExtrinsics end\n");
	//vm_cprintf("R0_C_A: "); PrintR_(R0_C_A);
	//vm_cprintf("T0_A_C_A: "); PrintT_(T0_A_C_A);
	//vm_cprintf("Weights: "); PrintT_(Weights);
	//vm_cprintf("BB: "); PrintR_(BB);
	//vm_cprintf("BA: "); PrintM39_(BA);
	//vm_cprintf("BBinv: "); PrintR_(BBinv);
	//vm_cprintf("BBinvBA: "); PrintM39_(BBinvBA);
	//vm_cprintf("E_Trans: "); PrintE_(E_Trans);
	//vm_cprintf("E_Rot: "); PrintE_(E_Rot);
	//vm_cprintf("E: "); PrintE_(E);
	//vm_cprintf("Iter: %f\n", (float32_t)Iter);
	//vm_cprintf("StepSize: %f\n", (float32_t)StepSize);
	vm_cprintf("rho: "); PrintT_(rho);
	fprintf(fp0,"rho: "); PrintT_(rho);
	//vm_cprintf("R_Adjust: "); PrintR_(R_Adjust);
	vm_cprintf("R_C_A: "); PrintR_(R_C_A);
	fprintf(fp0,"R_C_A: "); PrintR_(R_C_A);
	//vm_cprintf("T: "); PrintT_(T);
	//vm_cprintf("T_A_C_A: "); PrintT_(T_A_C_A);
	//vm_cprintf("DatCam.Refined_xyz_CamPos: "); PrintT_(DatCam.Refined_xyz_CamPos);
	//vm_cprintf("DatCam.Refined_R_A_C: "); PrintR_(DatCam.Refined_R_A_C);
	vm_cprintf("ExRefine::RefineExtrinsics end\n\n");
	fprintf(fp0,"ExRefine::RefineExtrinsics end\n\n");
#endif
	// MD_PRINT end
}

mecl::core::Matrix<float64_t,9,1> ExRefine::CGR(const mecl::core::Matrix<float64_t,3,1> &rho)
{
    float64_t r1 = rho(0);
    float64_t r2 = rho(1);
    float64_t r3 = rho(2);
    float64_t r1_2 = r1*r1;
    float64_t r2_2 = r2*r2;
    float64_t r3_2 = r3*r3;
    mecl::core::Matrix<float64_t,9,1> r;
    r(0) = 1.0+r1_2-r2_2-r3_2;
    r(1) = 2.0*r1*r2+2.0*r3;
    r(2) = 2.0*r1*r3-2.0*r2;
    r(3) = 2.0*r1*r2-2.0*r3;
    r(4) = 1.0-r1_2+r2_2-r3_2;
    r(5) = 2.0*r3*r2+2.0*r1;
    r(6) = 2.0*r1*r3+2.0*r2;
    r(7) = 2.0*r2*r3-2.0*r1;
    r(8) = 1.0-r1_2-r2_2+r3_2;
    r /= 1.0+r1_2+r2_2+r3_2;
    return r;
}

float64_t ExRefine::F_CGR(float64_t rho1, float64_t rho2, float64_t rho3, const mecl::core::Matrix<float64_t,9,9> &E)
{
    float64_t rho1_2 = rho1*rho1;
    float64_t rho2_2 = rho2*rho2;
    float64_t rho3_2 = rho3*rho3;
    float64_t r[9];
    r[0] = 1.0+rho1_2-rho2_2-rho3_2;
    r[1] = 2.0*rho1*rho2+2.0*rho3;
    r[2] = 2.0*rho1*rho3-2.0*rho2;
    r[3] = 2.0*rho1*rho2-2.0*rho3;
    r[4] = 1.0-rho1_2+rho2_2-rho3_2;
    r[5] = 2.0*rho3*rho2+2.0*rho1;
    r[6] = 2.0*rho1*rho3+2.0*rho2;
    r[7] = 2.0*rho2*rho3-2.0*rho1;
    r[8] = 1.0-rho1_2-rho2_2+rho3_2;
	// MD_MOD_3LINE_FIX1 start
    float64_t rInv = 1.0 / (1.0+rho1_2+rho2_2+rho3_2); // r = r * rInv;
	for (int i=0; i<9; i++)
		r[i] *= rInv;
	// MD_MOD_3LINE_FIX1 end
    float64_t F = 0;
    for (int i=0; i<9; i++)
    {
        float64_t ri = r[i];
        F += ri * ri * E(i,i);
        for (int j=i+1; j<9; j++)
        {
            float64_t Val = ri * r[j] * E(i,j);
            F += Val;
            F += Val;
        }
    }
    return F;
}

float64_t ExRefine::F_CGR(mecl::core::Matrix<float64_t,3,9> &BBinvBA, mecl::core::Matrix<float64_t,3,1> &T0, mecl::core::Matrix<float64_t,3,1> &Weights,
						  float64_t rho1, float64_t rho2, float64_t rho3, const mecl::core::Matrix<float64_t,9,9> &E)
{
	float64_t F0 = F_CGR(rho1, rho2, rho3, E); // Normal error

	// Error from rho.norm and/or T.norm being large
	mecl::core::Matrix<float64_t,3,1> rho;
	rho(0) = rho1;
	rho(1) = rho2;
	rho(2) = rho3;
    mecl::core::Matrix<float64_t,9,1> r = CGR(rho);
	mecl::core::Matrix<float64_t,3,1> T = -BBinvBA.mmul(r);
	float64_t F_R = rho.norm();
	float64_t F_T = (T.sub(T0)).norm();

	// Total weighted error
	float64_t F = Weights(0)*F0 + Weights(1)*F_R*F_R + Weights(2)*F_T*F_T;
	return F;
}

// 9x1 vector => 3x3 rotation matrix
mecl::core::Matrix<float64_t,3,3> ExRefine::r9_to_R3(const mecl::core::Matrix<float64_t,9,1> &r9)
{
    mecl::core::Matrix<float64_t,3,3> R3;
    R3(0,0) = r9(0);
    R3(1,0) = r9(1);
    R3(2,0) = r9(2);
    R3(0,1) = r9(3);
    R3(1,1) = r9(4);
    R3(2,1) = r9(5);
    R3(0,2) = r9(6);
    R3(1,2) = r9(7);
    R3(2,2) = r9(8);
    return R3;
}

// 3x3 rotation matrix => 9x1 vector
mecl::core::Matrix<float64_t,9,1> ExRefine::R3_to_r9(const mecl::core::Matrix<float64_t,3,3> &R3)
{
    mecl::core::Matrix<float64_t,9,1> r9;
    r9(0) = R3(0,0);
    r9(1) = R3(1,0);
    r9(2) = R3(2,0);
    r9(3) = R3(0,1);
    r9(4) = R3(1,1);
    r9(5) = R3(2,1);
    r9(6) = R3(0,2);
    r9(7) = R3(1,2);
    r9(8) = R3(2,2);
    return r9;
}







template<typename T> mecl::core::Matrix<T,3,1> ExRefine::CrossProd(mecl::core::Matrix<T,3,1> &xyz1, mecl::core::Matrix<T,3,1> &xyz2)
{
	mecl::core::Matrix<T,3,1> xyzOut;
	xyzOut(0) = xyz1(1)*xyz2(2) - xyz1(2)*xyz2(1);
	xyzOut(1) = xyz1(2)*xyz2(0) - xyz1(0)*xyz2(2);
	xyzOut(2) = xyz1(0)*xyz2(1) - xyz1(1)*xyz2(0);
	return xyzOut;
}

template<typename T> mecl::core::Matrix<T,3,3> ExRefine::Inverse3x3Mat(mecl::core::Matrix<T,3,3> &M0)
{
	mecl::core::Matrix<T,3,3> M1;
	T M11 = M0(1,1)*M0(2,2) - M0(1,2)*M0(2,1);
	T M12 = M0(1,0)*M0(2,2) - M0(2,0)*M0(1,2);
	T M13 = M0(1,0)*M0(2,1) - M0(2,0)*M0(1,1);
	T M21 = M0(0,1)*M0(2,2) - M0(2,1)*M0(0,2);
	T M22 = M0(0,0)*M0(2,2) - M0(2,0)*M0(0,2);
	T M23 = M0(0,0)*M0(2,1) - M0(2,0)*M0(0,1);
	T M31 = M0(0,1)*M0(1,2) - M0(1,1)*M0(0,2);
	T M32 = M0(0,0)*M0(1,2) - M0(1,0)*M0(0,2);
	T M33 = M0(0,0)*M0(1,1) - M0(1,0)*M0(0,1);
	T Det = M0(0,0)*M11 - M0(0,1)*M12 + M0(0,2)*M13;
	M1(0,0) = M11;
	M1(0,1) =-M12;
	M1(0,2) = M13;
	M1(1,0) =-M21;
	M1(1,1) = M22;
	M1(1,2) =-M23;
	M1(2,0) = M31;
	M1(2,1) =-M32;
	M1(2,2) = M33;
	M1 = M1.mul(1.0/Det);
	return M1;
}

// Find rotation matrix between two vectors/axis
template<typename T> mecl::core::Matrix<T,3,3> ExRefine::Ax2Ax_to_R3(mecl::core::Matrix<T,3,1> &Vec1, mecl::core::Matrix<T,3,1> &Vec2)
{
	mecl::core::Matrix<T,3,1> RotAxis = CrossProd(Vec1, Vec2);
	RotAxis = RotAxis.mul(1.0/RotAxis.norm());
	mecl::core::Matrix<T,1,1> DotProd = (Vec1.t()).mmul(Vec2);
	T Angle = acos( DotProd(0) / (Vec1.norm()*Vec2.norm()) );

	T c = cos(Angle);
	T s = sin(Angle);
	T ux = RotAxis(0);
	T uy = RotAxis(1);
	T uz = RotAxis(2);
	mecl::core::Matrix<T,3,3> R;
	R(0,0) = c+ux*ux*(1-c);
	R(0,1) = ux*uy*(1-c)-uz*s;
	R(0,2) = ux*uz*(1-c)+uy*s;
	R(1,0) = uy*ux*(1-c)+uz*s;
	R(1,1) = c+uy*uy*(1-c);
	R(1,2) = uy*uz*(1-c)-ux*s;
	R(2,0) = uz*ux*(1-c)-uy*s;
	R(2,1) = uz*uy*(1-c)+ux*s;
	R(2,2) = c+uz*uz*(1-c);
	return R;
}



// Rotation matrix => pitch/yaw/roll
mecl::core::Matrix<float32_t,3,1> ExRefine::R3_to_pyr(mecl::core::Matrix<float32_t,3,3> &R3)
{
	float32_t Pitch = 0;
	float32_t Yaw = atan2(static_cast<float64_t>(R3(2,0)), pow(static_cast<float64_t>(R3(2,1)*R3(2,1) + R3(2,2)*R3(2,2)) , 0.5) );
	float32_t Roll;

	if (fabs(Yaw-M_PI_2) < mecl::math::numeric_limits<float32_t>::epsilon_x())
	{
		Roll = atan2(R3(1,0) / cos(Yaw), R3(0,0) / cos(Yaw));
	}
	else if (fabs(Yaw+M_PI_2) < mecl::math::numeric_limits<float32_t>::epsilon_x())
	{
		Roll = atan2(-R3(1,0) / cos(Yaw), R3(0,0) / cos(Yaw));
	}
	else
	{
		Pitch = atan2(R3(2,1), R3(2,2));
		Roll  = atan2(R3(1,0), R3(0,0));
	}

	mecl::core::Matrix<float32_t,3,1> PYR;
	PYR(0) = Pitch;
	PYR(1) = Yaw;
	PYR(2) = Roll;
	return PYR;
}

mecl::core::Matrix<float32_t,3,3> ExRefine::pyr_to_R3(mecl::core::Matrix<float32_t,3,1> &PYR)
{
	float32_t Pitch = PYR(0);
	float32_t Yaw = PYR(1);
	float32_t Roll = PYR(2);

	mecl::core::Matrix<float32_t,3,3> RP(0), RY(0), RR(0), R3;

	// Pitch
	RP(0,0) = 1;
	RP(1,1) = cos(Pitch);
	RP(2,2) = cos(Pitch);
	RP(1,2) =-sin(Pitch);
	RP(2,1) = sin(Pitch);

	// Yaw
	RY(0,0) = cos(Yaw);
	RY(1,1) = 1;
	RY(2,2) = cos(Yaw);
	RY(0,2) =-sin(Yaw);
	RY(2,0) = sin(Yaw);

	// Roll
	RR(0,0) = cos(Roll);
	RR(1,1) = cos(Roll);
	RR(2,2) = 1;
	RR(0,1) =-sin(Roll);
	RR(1,0) = sin(Roll);

	R3 = RR.mmul(RY.mmul(RP));
	return R3;
}


// MATLAB_MATCH add following 4 functions:
mecl::core::Matrix<float32_t,3,1> ExRefine::R3_to_pyr_Flip(mecl::core::Matrix<float32_t,3,3> &R3)
{
	mecl::core::Matrix<float32_t,3,3> R3t = R3.t();
	mecl::core::Matrix<float32_t,3,1> PYR_t = R3_to_pyr(R3t);

	mecl::core::Matrix<float32_t,3,1> PYR;
	PYR(0) = PYR_t(0);
	PYR(1) = -PYR_t(1);
	PYR(2) = -PYR_t(2);
	return PYR;
}

mecl::core::Matrix<float32_t,3,3> ExRefine::pyr_to_R3_Flip(mecl::core::Matrix<float32_t,3,1> &PYR)
{
	float32_t Pitch = PYR(0);
	float32_t Yaw = PYR(1);
	float32_t Roll = PYR(2);

	mecl::core::Matrix<float32_t,3,3> RP(0), RY(0), RR(0), R3;

	// Pitch
	RP(0,0) = 1;
	RP(1,1) = cos(Pitch);
	RP(2,2) = cos(Pitch);
	RP(1,2) = sin(Pitch);
	RP(2,1) =-sin(Pitch);

	// Yaw
	RY(0,0) = cos(Yaw);
	RY(1,1) = 1;
	RY(2,2) = cos(Yaw);
	RY(0,2) =-sin(Yaw);
	RY(2,0) = sin(Yaw);

	// Roll
	RR(0,0) = cos(Roll);
	RR(1,1) = cos(Roll);
	RR(2,2) = 1;
	RR(0,1) =-sin(Roll);
	RR(1,0) = sin(Roll);

	R3 = RP.mmul(RY.mmul(RR));
	return R3;
}

mecl::core::Matrix<float32_t,3,3> ExRefine::get_R_C_A(mecl::core::Matrix<float32_t,3,1> &PYR, int CamNum, FILE *fp0)
{
	// MD old format
	// RPY = roll pitch yaw from VanishingPoint analysis
	// A = cAr Axle reference frame
	// C = cam reference frame
	mecl::core::Matrix<float32_t,3,3> R_A_C0 = mecl::core::Matrix<float32_t,3,3>::zeros_x();
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

	// Adjustment factors to convert from Matlab format to MD format
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
	vm_cprintf("R_A_C0 in ExR:"); JobPrintR__(R_A_C0);
	fprintf(fp0,"R_A_C0 in ExR:"); FILE_PrintR__(R_A_C0, fp0);
	ExRefine ExR;
	mecl::core::Matrix<float32_t,3,3> RAdjust = ExR.pyr_to_R3_Flip(PYR); // Instead of pyr_to_R3
	fprintf(fp0, "RAdjust in ExR:"); FILE_PrintR__(RAdjust, fp0);
	fprintf(fp0, "PYR in ExR:"); FILE_PrintT_(PYR, fp0);
	mecl::core::Matrix<float32_t,3,3> R_A_C0_Matlab = R_A_C0.mmul(N1);
	mecl::core::Matrix<float32_t,3,3> R_A_C_Matlab = R_A_C0_Matlab.mmul(RAdjust);
	mecl::core::Matrix<float32_t,3,3> R_A_C;
	vm_cprintf("R_A_C0_Matlab in ExR:"); JobPrintR__(R_A_C0_Matlab);
	vm_cprintf("R_A_C_Matlab in ExR:"); JobPrintR__(R_A_C_Matlab);
	fprintf(fp0,"R_A_C0_Matlab in ExR:"); FILE_PrintR__(R_A_C0_Matlab, fp0);
	fprintf(fp0,"R_A_C_Matlab in ExR:"); FILE_PrintR__(R_A_C_Matlab, fp0);
	if (CamNum==0 || CamNum==2)
		R_A_C = N1.mmul(R_A_C_Matlab.mmul(N2));
	else if (CamNum==1)
		R_A_C = N3.mmul(R_A_C_Matlab.mmul(N4));
	else if (CamNum==3)
		R_A_C = N5.mmul(R_A_C_Matlab);
	vm_cprintf("R_A_C in ExR:"); JobPrintR__(R_A_C);
	fprintf(fp0,"R_A_C in ExR:"); FILE_PrintR__(R_A_C, fp0);
	
	return R_A_C;
}
#ifndef MYPYR
mecl::core::Matrix<float32_t,3,1> ExRefine::get_pyr(mecl::core::Matrix<float32_t,3,3> &R_A_C, int CamNum, FILE *fp0)
{
	// MD old format
	// RPY = roll pitch yaw from VanishingPoint analysis
	// A = cAr Axle reference frame
	// C = cam reference frame
	mecl::core::Matrix<float32_t,3,3> R_A_C0 = mecl::core::Matrix<float32_t,3,3>::zeros_x();
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

	// Adjustment factors to convert from Matlab format to MD format
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

//	ExRefine ExR;
	mecl::core::Matrix<float32_t,3,3> R_Matlab_Inv = R_A_C0.t().mmul(N1.mmul(R_A_C));
	mecl::core::Matrix<float32_t,3,3> R3;
	if (CamNum==0 || CamNum==2)
		R3 = N1.mmul(R_Matlab_Inv.mmul(N2));
	else if (CamNum==1)
		R3 = N3.mmul(R_Matlab_Inv.mmul(N4));
	else if (CamNum==3)
		R3 = N5.mmul(R_Matlab_Inv);
	vm_cprintf("R_A_C0 in ExR:"); JobPrintR__(R_A_C0);
	fprintf(fp0, "R_A_C0 in ExR:"); FILE_PrintR__(R_A_C0, fp0);
	vm_cprintf("R_A_C in ExR:"); JobPrintR__(R_A_C);
	fprintf(fp0, "R_A_C in ExR:"); FILE_PrintR__(R_A_C, fp0);
	vm_cprintf("R_Matlab_Inv in ExR:"); JobPrintR__(R_Matlab_Inv);
	fprintf(fp0, "R_Matlab_Inv in ExR:"); FILE_PrintR__(R_Matlab_Inv, fp0);

	mecl::core::Matrix<float32_t,3,1> PYR = R3_to_pyr_Flip(R3);
	return PYR;
}
#else
// calculate pitch, yaw, and roll from rotation matrix
mecl::core::Matrix<float32_t, 3, 1> ExRefine::get_pyr(mecl::core::Matrix<float32_t, 3, 3> &R, double& pitch, double& yaw, double& roll, int CamNum, FILE *fp0)
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
#endif
