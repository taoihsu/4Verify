#ifndef JOBALGOVPCS_H
#define JOBALGOVPCS_H
//--------------------------------------------------------------------------
/// @file JobVPCS.h
/// @brief Contains the definition of VPCS Job 
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Cameron Taylor (cameron.taylor@magna.com)
///
//  --------------------------------------------------------------------------

#include "VanPoints.h"

#include <mecl/mecl.h>
#include <mecl/math/Math.h>
#include <mecl/core/Matrix3x3.h>
#include <mecl/core/RotationMatrix.h>
#include "Point.h"
//#include "BlockMatcher.h"
#include "MD_VarStore.h"

#include "SensorCfg.h"
#include "ExtrinsicRefinement.h"

#ifdef PIKEOS
#include "container/JobBaseAlgo.h"
#else
#include "JobBaseAlgo.h"
#endif

#ifdef PIKEOS
#include "logging/LogCtx.h"
#include "logging/ILogSender.h"
#define VPCS_SW_BUILD_DATE "09/08/2020"
#endif
#include "DataProviderVpcs.h"

#include "mecl/core/Matrix.h"

#ifdef VPCS3D
#include "IntCalib_Prj.h"
#include "ExtrinsicRefinement.h"
#endif
namespace vpcs
{

  class JobAlgoVpcs
  {
    public:
#ifdef PIKEOS
     JobAlgoVpcs(IDataProviderVpcs& b_DPVpcs_ro, logging::ILogSender& b_LogSender_ro);
#else
		JobAlgoVpcs(IDataProviderVpcs& b_DPVpcs_ro);
#endif
     ~JobAlgoVpcs();

     // Main functions
     void init_v();
     bool_t hasNext_b() const;
     void start_v(const uint8_t* i_InputImage_pu8,
		 uint8_t* b_WarpedImage_pu8, char *dirname, FILE *fp0);
	 //void start_v();
     void execute_v(char *dirname, FILE *fp0);
     void end_v(char *dirname, FILE *fp0);

     // Other functions
#ifdef PIKEOS
     void PrintData();
#else
     void PrintData(MD_Data &MD_DataC, FILE *fp0);
	 //void PrintData(MD_Data &MD_DataC);
#endif
	


     public: // MOD_MD
        MD_Data MD_DataC;
        TargetLines CalibTLinesC;

	private:


	  float32_t predSepDis_f32;
	  float32_t PredicSepDist();
	  void StoreVars_MD(mecl::core::ArrayList<MD_Data, 4> &DatAll, const uint8_t *ImDIn);


      enum JobVpcsState_e
      {
        e_VpcsJobAlgoUnInitialized = 0,
        e_VpcsJobAlgoRunStage1,
		e_VpcsJobAlgoRunStage2,
        e_VpcsJobAlgoJobFinished
      };

      const uint8_t *c_InImage_pu8;// image pointer
	  bool_t isImageFlipped_b;
#ifndef PIKEOS
	  uint8_t i_WarpedImage_pu8[c_ImageSize_u32];
#endif
      VanPoints vpExtractor_o;
      VpcsConfig_s mConfig_s;

      IDataProviderVpcs& dataProvider_ro;

      mecl::model::Camera<float32_t> mCameraObject_o;

      float32_t intrPpx_f32;
      float32_t intrPpy_f32;
      E_CameraId_t cameraID_t;

      // Image
      uint8_t dstMecl_au8[c_ImageSize_u32]; // Undistorted image
		        
     

      // Vanishing point lines
      VanPoints::VanPoint_s vanpoints_s; // Lines used to get the Vanishing Points
	  

	private:
#ifdef PIKEOS
       logging::ILogSender &log_ro;
#endif
       JobVpcsState_e jobAlgoVpcsState_e;
	   //BM::PointAnchole_s DoTemplateMatchingSAD(unsigned char * img, unsigned char * imgTemp, const int crop_sz);// , int templateIndex, uint32_t cam_id);
	};
}

#endif
