#ifndef VPCS_TYPES_DEF
#define VPCS_TYPES_DEF

#define BOSCH
	//Platform specific code switches
	//PRQA S 1051 5
    //#define PIKEOS
	#define WINDOWS
	//#define DEBUG_LOG

	//#define OPENCV
#define OPENCV_OUT 
#define SHOW_IMAGE
//#define VPCS2D
//#define VPCS3D
#define CYCLETIME
#ifdef CYCLETIME
#include <cv.h>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <chrono>
#include <iostream>
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;
#endif

#ifdef OPENCV_OUT


#include <cv.h>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
//#include <opencv2\imgproc.hpp>
#endif
//	#define PREWARPED
#define DBG_PRINT

#include "pikeos_defs.h"
#include "mecl/core/MeclTypes.h"
#include "mecl/core/matrix.h"
#ifdef PIKEOS
#include "shmdata/McuData.h"
#include "shmdata/VpcsSegment.h"
#endif
#include "cannyEdgeDetector/Point.h"

namespace vpcs
{



typedef shmdata::ResponseState_e ResponseState_t;
typedef shmdata::AlgoCommand_e AlgoCommand_t;

typedef shmdata::VpcsAlgoState_e VpcsAlgoState_t;
typedef shmdata::VpcsErrorCode_e VpcsErrorCode_t;

extern shmdata::VpcsErrorCode_e v_FLAG_t;

	//defining Array maximum sizes
	const uint32_t LINES_SIZE_MAX = 250;
	//const uint32_t LINES_INNER_SIZE = 4;
	const uint32_t VP_SIZE = 3;
	const uint32_t MAX_CLUSTER_SIZE = LINES_SIZE_MAX;
////const float32_t c_MismatchVDist_f32 = 80.0F;
//const float32_t c_MismatchHDist_f32 = 40.0F;
//const float32_t c_MismatchROIVDist_f32 = 180.0F;

	////definition of image size
 //   const uint32_t c_ImageScale_u32 = 4;
	//const uint32_t c_ImageWidthScaled_u32 = 320;
	//const uint32_t c_ImageHeightScaled_u32 = 200;
	//const uint32_t c_ImageSizeScaled_u32 = c_ImageWidthScaled_u32*c_ImageHeightScaled_u32;
#ifndef BOSCH
	const uint32_t c_ImageWidth_u32 = 1280;
	const uint32_t c_ImageHeight_u32 = 800;
#else
	const uint32_t c_ImageWidth_u32 = 1920;
	const uint32_t c_ImageHeight_u32 = 1280;
#endif
	const uint32_t c_ImageSize_u32 = c_ImageWidth_u32*c_ImageHeight_u32;
	
	//const uint32_t c_FRTempImageWidth_u32 = 114;
	//const uint32_t c_FRTempImageHeight_u32 = 58;

	/*const uint32_t c_FRTempImageWidth_u32 = 85;
	const uint32_t c_FRTempImageHeight_u32 = 25;*/
	/*const uint32_t c_FRTempImageWidth_u32 = 70;
	const uint32_t c_FRTempImageHeight_u32 = 30;*/


	//const uint32_t c_FRTempImageSize_u32 = c_FRTempImageWidth_u32*c_FRTempImageHeight_u32;
	/*const uint32_t c_LRTempImageWidth_u32 = 320;
	const uint32_t c_LRTempImageHeight_u32 = 57;*/
	////const uint32_t c_LRTempImageWidth_u32 = 319;
	//const uint32_t c_LRTempImageHeight_u32 = 78;
	//const uint32_t c_LRTempImageSize_u32 = c_LRTempImageWidth_u32*c_LRTempImageHeight_u32;

	//defining adjustable parameters
	const uint32_t c_ModelSize_u32 = 600;//0313
	//const uint32_t c_ModelSize_u32 = 800;
	const uint32_t c_LineLength_u32 = 35;
	const uint32_t c_ClusterNum_u32 = 2;
	//const float32_t c_UnsharpAlpha_f32 = 0.14F;
	//const float32_t c_TargetAlignmentThreshold_f32 = 94.0F;
	//const float32_t c_TargetAlignmentThreshold_f32 = 130.0F;
	const uint32_t c_LineLengthFrontRear_u32 = 25;
	//const uint32_t c_LineLengthLeftRight_u32 = 50;
	//const uint32_t c_LineLengthFrontRear_u32 = 70;
	//const uint32_t c_LineLengthFrontRear_u32 = 100;
	const uint32_t c_LineLengthLeftRight_u32 = 30;


	typedef mecl::core::Matrix<float32_t, c_ImageWidth_u32, c_ImageHeight_u32> imageMatFloat_t;

	typedef enum CameraId_e
	{
		e_FrontCamAlgo = 0,
		e_LeftCamAlgo = 1,
		e_RearCamAlgo = 2,
		e_RightCamAlgo = 3
	} E_CameraId_t;


	typedef enum ROIID_e
	{
		e_RoiL = 0,
		e_RoiR = 1,
		e_Roi = 2
	} E_ROIID_t;



	typedef struct AParaMgrSCalibration_s
	{
		float32_t		camYaw_f32;
		float32_t		camPitch_f32;
		float32_t		camRoll_f32;
		float32_t		camX_f32;
		float32_t		camY_f32;
		float32_t		camZ_f32;
		E_CameraId_t	camId_e;
		VpcsErrorCode_t errorCode_e;
	}aParaMgr_S_Calibration_t;

	struct VpcsConfig_s {
		sint32_t lineLength;
		sint32_t modelSize;
	};

	//typedef prjcontainer::CameraId_e          CameraId_t;

	 
}
#ifdef OPENCV_OUT
class DisplayArray2D {
private:
	int rows, cols;
	uint8_t *data;
public:
	DisplayArray2D(const int r, const int c, uint8_t* d) : rows(r), cols(c), data(new uint8_t[rows * cols]) {
		for (int i = 0; i<rows; i++) {
			for (int j = 0; j<cols; j++) {
				(*this)[i][j] = d[j*r + i];
				//(*this)[i][j] = d[i*cols + j]; // swap i and j
			}
		}
	}

	~DisplayArray2D() {
		delete[] data;
	}

	uint8_t* operator[](int i) {
		return data + (i * cols);
	}

	void display() {
		cv::Mat image(cols, rows, CV_8UC1, cv::Scalar(0));
		for (int i = 0; i<rows; i++) {
			for (int j = 0; j<cols; j++) {
				image.at<uchar>(j, i) = (*this)[i][j];
			}
		}
		//cv::Mat flipped;
		//cv::flip(image, flipped, 0); // Flip the image along the horizontal axis
		imshow("Array Image", image);
		cv::waitKey(0);
		//cv::destroyAllWindows();
	}
};

#endif

//int main() {
//	int r = 3, c = 4;
//	int arr[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
//	DisplayArray2D a(r, c, arr);
//	a.display();
//
//	return 0;
//}
#endif
