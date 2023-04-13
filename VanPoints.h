/*
 * VanPoints.cpp
 * This file is part of VanPoints
 *
 */

// This is an implementation of
// Non-Iterative Approach for Fast and Accurate Vanishing m_Point Detection
// by Jean-Philippe Tardif

// --- Modified by David Hsu [24-June-2019]
// --- Copyright (c) Magna Vectrics (MEVC) 2019
// 


#ifndef _VANPOINTS_H_
#define _VANPOINTS_H_


#include <mecl/math/Math.h>
#include <mecl/mecl.h>
#include <SVD_vpcs.h>
#include "./src/Vpcs_types.h"
#include "EdgeDetector.h"
#include "EigenVector.h"
#include "findcontours.h"
#include "BlockMatcher.h"
#include "mecl/core/Point.h"
#include "rtwtypes.h"

#ifdef OPENCV
#include <cv.h>
#endif
#ifdef OPENCV_OUT
#include <highgui.h>
#endif

#ifndef M_PI
#define M_PI 3.141592653589793238462643
#endif



namespace vpcs
{

class VanPoints
{
public:
  VanPoints();
  virtual ~VanPoints();

  void setCameraID(E_CameraId_t i_CameraID_e)
  {
    cameraID_e = i_CameraID_e;
  }
  
  // Some primitive structs
  // Line_s structure
  struct Line_s
  {
    float32_t x1_af32[2]; /* end point 1 */
    float32_t x2_af32[2]; /* end point 2 */
    float32_t mean_af32[2]; /* middle point */
    float32_t l_af32[3]; /* line representation in homogeneous coordinates */
    float32_t theta_f32; /* line angle */
    float32_t r_f32; /* line length */
  };
  // Point_s structure
  struct Point_s
  {
    float32_t x_f32;
    float32_t y_f32;
    float32_t z_f32;
  };
  struct Point2D_s
  {
	  uint32_t x_i32;
	  uint32_t y_i32;
	 
  };
  // Output of the vanishing point detection with all possible results
  struct VanPoint_s
  {
    Line_s lines_as[LINES_SIZE_MAX]; // All lines
    sint32_t lineNum; // Number of lines
    sint32_t (*clusters)[MAX_CLUSTER_SIZE]; // Individual line clusters
    Point_s vanPoints_as[VP_SIZE]; // Vanishing points
	Point2D_s Corner_s[8];
  };

private:
  void extractLongLine(uint8_t* b_Img_pu8,
                       uint32_t i_MinLen_u32,
                       uint32_t* o_Lnum_pu32,
                       Line_s* o_Lines_ps,
                       uint32_t i_Width_u32,
                       uint32_t i_Height_u32) const;
  void findVanPoints(sint32_t i_ModelSize_s32,
                     const Line_s* i_Edges_ps,
                     sint32_t i_LineCount_s32,
                     sint32_t* o_VpNum_ps32,
                     VanPoints::Point_s* o_Output_ps,
                     sint32_t i_LineLength_s32);
  // JLinkage Stuff
  // Model_s - two lines intersecting at a point is the model
  struct Model_s
  {
    Line_s line1_s;
    Line_s line2_s;
    Point_s intersectionPt_s;
  };

  struct MinimalSet_s
  {
    Model_s minSet_as[c_ModelSize_u32];
    sint32_t size;
  };

  float32_t clusterThresh_f32;
  sint32_t m_clusters[c_ClusterNum_u32][MAX_CLUSTER_SIZE];
  E_CameraId_t cameraID_e;

  static MinimalSet_s selectMinimalSets(sint32_t i_ModelSize_s32,
                                        const Line_s* i_Edges_ps,
                                        sint32_t i_LineCount_s32,
										E_CameraId_t cameraID_e);
  void makePSMatrix(const MinimalSet_s &i_RandomMS_rs,
                    const Line_s* i_Edges_ps,
                    sint32_t i_LineCount_s32,
                    sint32_t (*PSMatrix)[c_ModelSize_u32]) const;
  void clusterPSMatrix(sint32_t (*PSMatrix)[c_ModelSize_u32],
                       sint32_t lineNum,
                       sint32_t modelNum,
                       const sint32_t* clusterNum);
  static Point_s estimateVanPoint(const Line_s* i_Edges_ps,
                                  sint32_t i_LineNum_s32,
                                  const sint32_t* i_Cluster_ps32);

  // Some handy functions
 /* static Point_s findIntersection_s(const Line_s& i_Line1_rs,
                                    const Line_s& i_Line2_rs);*/
  static float32_t findOrthDistance(const Line_s &i_Line_rs,
                                    const Point_s &i_Point_rs);
  static float32_t jaccardDist(const sint32_t* A,
                               const sint32_t* B,
                               sint32_t len);
 
  mecl::core::Point2D<uint32_t> DoTemplateMatchingSAD(unsigned char * img, unsigned char * imgTemp, const int crop_sz);// , int templateIndex, uint32_t cam_id);
  
  //typedef unsigned char boolean_T;
public:
  VanPoint_s findVanishingPoints(uint8_t* b_Img_pu8,
                                 sint32_t i_lineLength_s32,
                                 sint32_t i_modelSize_s32);
 
  static Point_s findIntersection_s(const Line_s& i_Line1_rs,
	  const Line_s& i_Line2_rs);
  #ifdef OPENCV_OUT
   sint32_t superposeLines(IplImage * img, Line_s *lines, sint32_t lineNum, sint32_t(*clusters)[MAX_CLUSTER_SIZE], VanPoints::Point_s *, sint32_t lineLength);
  #endif
   
};

}
#endif
