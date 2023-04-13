#pragma once

#include "JobAlgoVpcs.h"
#include "./src/Vpcs_types.h"

#include "IntCalib_Prj.h"
#include "ExtrinsicRefinement.h"
//#define VPCS2D

#ifdef PIKEOS
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
#endif


// Functions
float32_t PredicSepDist(vpcs::JobAlgoVpcs &DatOld);

// Make figures
void Fig_DispLines(MD_Data &DatCam, bool ImDistorted);
#ifdef OPENCV_OUT
void Fig_BirdsEyeAll(std::vector<MD_Data> &DatAll, TargetLines &CalibTLines, bool ImDistorted, int ExtrinsicsChoose);
#endif

