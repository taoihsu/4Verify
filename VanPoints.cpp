/*
 * VanPoints.cpp
 * This file is part of VanPoints
 *
 */
// This is an implementation of
// Non-Iterative Approach for Fast and Accurate Vanishing m_Point Detection
// --- Modified by David Hsu [24-June-2019]
// --- Copyright (c) Magna Vectrics (MEVC) 2019
// 
#include "VanPoints.h"
#ifdef PIKEOS
#include "stdlib.h"
#endif
#define PRINT_DEBUG
#define ROUGH_ROI
#define HIST2THRESHOLD
//#define SHOW_CORNER
//#define SHOW_CORNER_RECT
#define OLDIMAGES
#ifdef BOSCH
#define HULLMASK
#ifdef HULLMASK
struct Point {
	int x;
	int y;
};
#endif
// Helper function to calculate the cross product of two vectors
int cross_product(Point A, Point B, Point C) {
	return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
}
// Returns the squared Euclidean distance between two points.
int distSq(Point p1, Point p2) {
	return (p1.x - p2.x) * (p1.x - p2.x) +
		(p1.y - p2.y) * (p1.y - p2.y);
}

// Returns the orientation of the triplet (p, q, r).
// Returns:
// 0 if p, q, r are collinear.
// 1 if p, q, r are clockwise.
// 2 if p, q, r are counterclockwise.
int orientation(Point p, Point q, Point r) {
	int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
	if (val == 0) return 0;
	return (val > 0) ? 1 : 2;
}

// Returns the point with the smallest y-coordinate.
Point findBottommostPoint(Point points[], int n) {
	Point bottommost = points[0];
	for (int i = 1; i < n; i++) {
		if (points[i].y < bottommost.y) {
			bottommost = points[i];
		}
		else if (points[i].y == bottommost.y && points[i].x < bottommost.x) {
			bottommost = points[i];
		}
	}
	return bottommost;
}

// Returns true if p2 is closer to p1 than p3.
bool closer(Point p1, Point p2, Point p3) {
	int d1 = distSq(p1, p2);
	int d2 = distSq(p1, p3);
	return (d1 < d2);
}

// Computes the convex hull of a set of points.
// Function to build the convex hull of a set of points.
void buildConvexHull(Point points[], int n, Point hull[], int& hn) {
	// Sort the points by x-coordinate.
	for (int i = 0; i < n; ++i) {
		for (int j = i + 1; j < n; ++j) {
			if (points[j].x < points[i].x) {
				Point temp = points[i];
				points[i] = points[j];
				points[j] = temp;
			}
		}
	}

	// Build the lower hull.
	hn = 0;
	for (int i = 0; i < n; ++i) {
		while (hn >= 2 && cross_product(hull[hn - 2], hull[hn - 1], points[i]) <= 0) {
			--hn;
		}
		hull[hn++] = points[i];
	}

	// Build the upper hull.
	int t = hn;
	for (int i = n - 2; i >= 0; --i) {
		while (hn > t && cross_product(hull[hn - 2], hull[hn - 1], points[i]) <= 0) {
			--hn;
		}
		hull[hn++] = points[i];
	}

	// Remove the last point.
	if (hn > 1) {
		--hn;
	}
}
// Returns the convex hull number of points .
void convex_hull(Point points[], int n, Point hull[]) {
	int leftmost = 0;
	for (int i = 1; i < n; i++) {
		if (points[i].x < points[leftmost].x) {
			leftmost = i;
		}
	}

	int curr = leftmost, next;
	int k = 0;
	do {
		hull[k++] = points[curr];
		next = (curr + 1) % n;
		for (int i = 0; i < n; i++) {
			if (cross_product(points[curr], points[i], points[next]) > 0) {
				next = i;
			}
		}
		curr = next;
	} while (curr != leftmost);
}
//Point convexHull(Point points[], int n, Point hull[]) {
//	// Find the bottommost point and swap it with the first point.
//	Point bottommost = findBottommostPoint(points, n);
//	Point temp = points[0];
//	points[0] = points[bottommost.x];
//	points[bottommost.x] = temp;
//	//myswap(points[0], points[bottommost.x]);
//
//	// Sort the points by their polar angle with respect to the bottommost point.
//	for (int i = 1; i < n; i++) {
//		for (int j = i + 1; j < n; j++) {
//			int o = orientation(points[0], points[i], points[j]);
//			if (o == 2 || (o == 0 && closer(points[0], points[j], points[i]))) {
//				//myswap(points[i], points[j]);
//				Point temp = points[i];
//				points[i] = points[j];
//				points[j] = temp;
//			}
//		}
//	}
//
//	// Traverse the sorted points to construct the convex hull.
//	int m = 0;
//	for (int i = 0; i < n; i++) {
//		while (m >= 2 && orientation(hull[m - 2], hull[m - 1], points[i]) != 2) {
//			m--;
//		}
//		hull[m++] = points[i];
//	}
//
//	return *hull;
//}
bool myisInsideConvexHull(Point *hull, int n, Point p) {
	// If the convex hull has less than 3 points, it does not enclose any area.
	if (n < 3) return false;

	// Check the orientation of each edge of the convex hull.
	int prev_or = orientation(hull[0], hull[1], p);
	for (int i = 1; i < n; i++) {
		int curr_or = orientation(hull[i], hull[(i + 1) % n], p);
		if (curr_or == 0) {
			// The point lies on an edge of the convex hull.
			return true;
		}
		if (curr_or != prev_or) {
			// The point lies on the opposite side of an edge.
			return false;
		}
		prev_or = curr_or;
	}

	// The point lies inside the convex hull.
	return true;
}
void isInsideConvexHull(Point *hull, int hullSize, imtp::Image<bool_t>&  mask, mecl::Rect *rect) {
	int i = 0;
	int j = hullSize - 1;
	bool inside = false;
	int min_x = hull[0].x, min_y = hull[0].y, max_x = hull[0].x, max_y = hull[0].y;
	for (int idx = 0; idx < hullSize; idx++) {
		Point* p = &hull[idx];
		//for (Point p : &hull[0]) {
		if (min_x > p->x)min_x = p->x;
		if (min_y > p->y)min_y = p->y;
		if (max_x < p->x)max_x = p->x;
		if (max_y < p->y)max_y = p->y;
		std::cout << "min(" << min_x << ", " << min_y << ")\n";
		std::cout << "max(" << max_x << ", " << max_y << ")\n";
		std::cout << "(" << p->x << ", " << p->y << ")\n";
	}
	rect->x = min_x;
	rect->width = (max_x - min_x);
	rect->y = min_y;
	rect->height=(max_y - min_y);
		for (int x = min_x; x<max_x; x++)
			for (int y = min_y; y < max_y; y++)
			{
				Point point;

				point.x = x;
				point.y = y;
				if (myisInsideConvexHull(hull, hullSize, point))
				{
					mask.data_px[x + y*mask.width] = true;
				}
			}
		//	int pre_crossProduct= (hull[1].x - hull[0].x) * (y - hull[0].y) - (hull[1].y - hull[0].y) * (x - hull[0].x);
		//for (i = 1; i < hullSize; i++) {
		//	if (hullSize < 3) return false;
		//	// Calculate the cross product of the current edge and the vector from the starting point to the given point
		//	Point* start = &hull[i];
		//	Point* end = &hull[(i + 1) % hullSize];
		//	int crossProduct = (end->x - start->x) * (y - start->y) - (end->y - start->y) * (x - start->x);
		//	if (crossProduct == 0)
		//	{
		//		mask.data_px[y + x*mask.width] = true;
		//		//mask.at(y, x) = true;

		//		return true;
		//	}
		//	if (crossProduct != pre_crossProduct)
		//	{
		//		return false;
		//	}
		//	/*j = i + 1;
		//	if (((hull[i].y > y) != (hull[j].y > y)) &&
		//		(x < (hull[j].x - hull[i].x) * (y - hull[i].y) / (hull[j].y - hull[i].y) + hull[i].x))
		//		inside = !inside;*/
		//	pre_crossProduct = crossProduct;
		//}

		//if (inside) {
		//	// Calculate the index of the pixel in the mask array and set it to true
		//	int index = p.y * mask.width + p.x;
		//	*mask.at(p.x, p.y) = true;
		//	
		//	return true;
		//}
		//else {
		//	return false;
		//}
	
}
#endif
//#define FORD_QA

#define INITIALROI
//#define FINDCENTER
//#define ENHANCE_MASK
//templateFiles

#include "Ft_LTL.h"
#include "Ft_LTR.h"
#include "Ft_LBL.h"
#include "Ft_LBR.h"

#include "Ft_RTL.h"
#include "Ft_RTR.h"
#include "Ft_RBL.h"
#include "Ft_RBR.h"

//left template
#include "Lt_TL.h"
#include "Lt_TR.h"
#include "Lt_BL.h"
#include "Lt_BR.h"

//Rear template
#include "Rr_LTL.h"
#include "Rr_LTR.h"
#include "Rr_LBR.h"
#include "Rr_LBL.h"

#include "Rr_RTL.h"
#include "Rr_RTR.h"
#include "Rr_RBR.h"
#include "Rr_RBL.h"

//Right template
#include "Rt_TL.h"
#include "Rt_TR.h"
#include "Rt_BL.h"
#include "Rt_BR.h"


float H_line_AngleF = 0.3f;
// PRQA S 3706 EOF // applying subscript operators to pointer values cannot be avoided here

#ifdef OPENCV_OUT
#include <opencv2/imgproc/imgproc.hpp>
#endif

// define DEBUG_LOG

#define DEBUG_LOG2


namespace vpcs
{
	shmdata::VpcsErrorCode_e v_FLAG_t;
	static void computeHistogram2ThresholdBinary(int* threshold,  uint8_t* v_GrayImage_o, const uint32_t crop_width, const uint32_t crop_height)
	{
		uint8_t hist[256];
		
		unsigned int i;
		float sum = 0;
		float sumB = 0;
		int q1 = 0;
		int q2 = 0;
		float varMax = 0;
		float m1;
		float m2;
		float varBetween;
		uint32_t ImSize = crop_width * crop_height;

		memset(hist, 0, 256);

		//int i = 0;
		for (uint32_t row = 0; row < crop_height; row++)
		{
			for (uint32_t col = 0; col < crop_width; col++)

			{

					int value = v_GrayImage_o[col + row * crop_width];
					hist[value]++;
				//i++;

			}
		}
	
			// Auxiliary value for computing m2
			for (i = 0; i < 256; i++) {
				sum += i * hist[i];
			}

			for (int i = 0; i < 256; i++) {
				// Update q1
				q1 += hist[i];
				if (q1 == 0)
					continue;
				// Update q2
				q2 = ImSize - q1;

				if (q2 == 0)
					break;
				// Update m1 and m2
				sumB += i * hist[i];
				m1 = sumB / q1;
				m2 = (sum - sumB) / q2;

				// Update the between class variance
				varBetween = q1 * q2 * (m1 - m2) * (m1 - m2);

				// Update the threshold if necessary
				if (varBetween > varMax) {
					varMax = varBetween;
					*threshold = i;
				}
			}
			
	}
#ifdef FINDCENTER
		bool FindPoint(int x1, int y1, int x2,
		int y2, int x, int y)
	{
		if (x < x1 || x > x2 || y < y1 || y > y2)
			return true;

		return false;
	}

#endif
	
	#if 1
#ifdef _WINDOWS

	void DoTemplateMatchingSAD(unsigned char *img, unsigned char *imgTemp, const int crop_sz, mecl::core::Point2D<uint32_t> *point_val)
		
	{
		
		const int size = c_ImageSize_u32;
		const int windowSize = 22;
		
		
#else
//	void VanPoints::DoTemplateMatchingSAD(const uint8_t *img, const uint8_t *imgTemp, uint32_t templateIndex, uint32_t cam_id)
//	{
//		static_cast<void>(*img);
#endif

#ifdef SPIRAL_SEARCH


		uint32_t min_spiral = 10000;


		
		BM::IMG_mad_8x8_v2_spiral(img, crop_sz, &imgTemp[0], windowSize + 1, crop_sz, crop_sz, point_val, &min_spiral);
		uint32_t r = point_val->getPosX() + windowSize / 2;
		uint32_t c = point_val->getPosY() + windowSize / 2;
		printf("levelConfiden %f\n", static_cast<float64_t>(min_spiral)/(static_cast<float64_t>(crop_sz)*static_cast<float64_t>(crop_sz)));
		if ((r > static_cast<uint32_t>(crop_sz)) || (c > static_cast<uint32_t>(crop_sz)))
		{
			r = crop_sz / 2;
			c = crop_sz / 2;
		}
#ifdef FINDCENTER
		int x1 = crop_sz / 6;
		int y1 = crop_sz / 6;
		int x2 = 5 * x1;
		int y2 = 5 * y1;
		if(FindPoint( x1,  y1,  x2, y2,  r,  c))
		{
			r = crop_sz / 2;
			c = crop_sz / 2;
		}

#endif
		point_val->setPosX(r);
		point_val->setPosY(c);
		


#else
		uint32_t v_mad_result[2];
		uint8_t RefBlock[(windowSize + 1)*(windowSize + 1)];		
		BM::IMG_mad_8x8(img, &imgTemp[0], crop_sz, crop_sz, crop_sz, windowSize + 1, windowSize + 1, v_mad_result);
		
		uint32_t r = ((v_mad_result[0] >> 16) & 0xFFFF);
		uint32_t c = ((v_mad_result[0] >> 0) & 0xFFFF);
		point_val(0) = r;
		point_val(1) = c;
#endif	
#ifdef DEBUG_LOG2
		vm_cprintf("VPCS_DBG: min_spiral = %d  ", static_cast<uint32_t>(min_spiral/ crop_sz));
		vm_cprintf("VPCS_DBG: min_spiral = %d  (r, c) =(%d, %d)\r\n", min_spiral, r, c);
#endif
#ifdef OPENCV_OUT


		cv::Mat Iimg(crop_sz, crop_sz, CV_8UC1, img);
		/*imshow("in src", Iimg);
		cv::waitKey(0);*/
		cv::Mat c3;
		cv::cvtColor(Iimg, c3, CV_GRAY2BGR);

		cv::String info = cv::format("r: %d c: %d ", r, c);
		cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
		cv::Point2d vp2D;
		vp2D.x = r;
		vp2D.y = c;

		cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.02, cv::Scalar(0, 0, 255), -1);
#ifdef SHOW_CORNER
		cv::imshow("roicenter", c3);
		cv::waitKey(0);
#endif

#endif
		//return point_val;
	}
#endif
#ifdef SHOW_CORNER_RECT
	static mecl::core::Point2D<uint32_t> CornerMatch(E_CameraId_t cameraID_e, unsigned char *Img, uint8_t *imgTemp_Ft_L, int Pt_idx, mecl::core::Point2D<uint32_t> *Anchor_pt, cv::Mat c3)
#else
	static mecl::core::Point2D<uint32_t> CornerMatch(E_CameraId_t cameraID_e, unsigned char *Img, uint8_t *imgTemp_Ft_L, int Pt_idx, mecl::core::Point2D<uint32_t> *Anchor_pt)
#endif
	{
//#ifdef SHOW_CORNER_RECT
//
//
//		cv::Mat Iimg(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1, Img);
//		cv::Mat c3;
//		cv::cvtColor(Iimg, c3, CV_GRAY2BGR);
//
//
//#endif
#if 1
		if (cameraID_e == e_FrontCamAlgo)
		{


			
			 const uint32_t crop_width = 101;
			 const uint32_t crop_height = 101;

			
			//uint8_t crop_img[crop_width*crop_height];

	
			
			//Front left TOP left corner
			if (Pt_idx == 0)
			{
				const uint32_t crop_width_0 = 131;
				const uint32_t crop_height_0 = 131;
				uint8_t crop_img_0[crop_width_0*crop_height_0];
				mecl::Recti Ft_LTL_ROI;
#ifndef OLDIMAGES				
				Ft_LTL_ROI.x = 111; Ft_LTL_ROI.y = 360;
#else
				//Ft_LTL_ROI.x = 165; Ft_LTL_ROI.y = 360;
				Ft_LTL_ROI.x = 130; Ft_LTL_ROI.y = 360;
#endif
				
				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width_0 / 2);

			
				for (uint32_t i = Ft_LTL_ROI.x; i < (Ft_LTL_ROI.x + crop_width_0); i++) {
					for (uint32_t j = Ft_LTL_ROI.y; j < (Ft_LTL_ROI.y + crop_height_0); j++) {

					
						crop_img_0[(i - Ft_LTL_ROI.x) + (j - Ft_LTL_ROI.y) * crop_width_0] = Img[(i + j * c_ImageWidth_u32)];
					}
				}
				
				int threshold = 0;
				uint32_t Crop_size = crop_width_0*crop_height_0;

				computeHistogram2ThresholdBinary(&threshold, crop_img_0,  crop_width_0, crop_height_0);
				for (uint32_t i = 0; i < Crop_size; i++)
				{
					if (crop_img_0[i] <= 0.3*threshold){
						crop_img_0[i] = 0;
					}
				}

			//#ifdef SHOW
#if 1
				cv::Mat crop_img_Arry(crop_height_0, crop_width_0, CV_8U, crop_img_0);
			imshow("crop_img Arry", crop_img_Arry);
			cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img_0[0], imgTemp_Ft_L, crop_width_0, &Anchor_pt[0]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[0](0), Anchor_pt[0](1));
				int r, c;
				r = Anchor_pt[0](0) + Ft_LTL_ROI.x;
				c = Anchor_pt[0](1) + Ft_LTL_ROI.y;
				Anchor_pt[0].setPosX(r);
				Anchor_pt[0].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LTL_ROI.x, Ft_LTL_ROI.y, crop_width_0, crop_height_0);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif

			}
			
			uint8_t crop_img[crop_width*crop_height];
			
			//end of LTL
			if (Pt_idx == 1)
			{
				
				mecl::Recti Ft_LTR_ROI;
				
#ifndef OLDIMAGES				
				Ft_LTR_ROI.x = 293; Ft_LTR_ROI.y = 360;
#else
				Ft_LTR_ROI.x = 305; Ft_LTR_ROI.y = 360;
#endif
				
//#ifdef Front_Top
#if 1
				//Ft LTR
				
				memset(crop_img, 0, crop_width*crop_height);
				
				memset(crop_img, 0, crop_width*crop_height);
				for (uint32_t i = Ft_LTR_ROI.x; i < (Ft_LTR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LTR_ROI.y; j < (Ft_LTR_ROI.y + crop_height); j++) {


						crop_img[(i - Ft_LTR_ROI.x) + (j - Ft_LTR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				int threshold = 0;
				uint32_t Crop_size = crop_width*crop_height;

				computeHistogram2ThresholdBinary(&threshold, crop_img, crop_width, crop_height);
				for (uint32_t i = 0; i < Crop_size; i++)
				{
					if (crop_img[i] <= 0.4*threshold)
					{
						crop_img[i] = 0;
					}
				}
				 DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[1]);
				 ////printf("Anchor_pt pts x %d y %d\n", Anchor_pt[1](0), Anchor_pt[1](1));
				 int r, c;
				 r = Anchor_pt[1](0)+Ft_LTR_ROI.x;
				 c = Anchor_pt[1](1)+Ft_LTR_ROI.y;
				 //printf("Anchor_pt pts r %d c %d\n", r, c);
				 Anchor_pt[1].setPosX(r);
				 Anchor_pt[1].setPosY(c);

#endif
#ifdef SHOW_CORNER_RECT
				 cv::String info = cv::format("r: %d c: %d ", r, c);
				 cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				 cv::Point2d vp2D;
				 vp2D.x = r;
				 vp2D.y = c;
				 cv::Rect rect(Ft_LTR_ROI.x, Ft_LTR_ROI.y, crop_width, crop_height);
				 cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				 cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif

			}
// end of LTR
			if (Pt_idx == 3)
			{
				
				mecl::Recti Ft_LBL_ROI;
				Ft_LBL_ROI.x = 1; Ft_LBL_ROI.y = 430;
				
//#ifdef LBL
#if 1
				memset(crop_img, 0, crop_width*crop_height);
				for (uint32_t i = Ft_LBL_ROI.x; i < (Ft_LBL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LBL_ROI.y; j < (Ft_LBL_ROI.y + crop_height); j++) {

						if (j > uint32_t(Ft_LBL_ROI.y + 60))

						{
							
							crop_img[(i - Ft_LBL_ROI.x) + (j - Ft_LBL_ROI.y) * crop_width] = 0;
						}
						else{
							crop_img[(i - Ft_LBL_ROI.x) + (j - Ft_LBL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
						}
					}
				}
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				
				int threshold = 0;
				uint32_t Crop_size = crop_width*crop_height;

				computeHistogram2ThresholdBinary(&threshold, crop_img, crop_width, crop_height);
				for (uint32_t i = 0; i < Crop_size; i++)
				{
					if (crop_img[i] <= 0.6*threshold){
						crop_img[i] = 0;
					}
				}

				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[3]);
				////printf("Anchor_pt pts x %d y %d\n", Anchor_pt[3](0), Anchor_pt[3](1));
#endif
				int r, c;
				r = Anchor_pt[3](0) + Ft_LBL_ROI.x;
				c = Anchor_pt[3](1) + Ft_LBL_ROI.y;
				Anchor_pt[3].setPosX(r);
				Anchor_pt[3].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LBL_ROI.x, Ft_LBL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			
			}
			//Front left bottom right
			if (Pt_idx == 2)
			{
				
				mecl::Recti Ft_LBR_ROI;
				Ft_LBR_ROI.x = 230; Ft_LBR_ROI.y = 430;
				//#ifdef LBR
#if 1
				memset(crop_img, 0, crop_width*crop_height);
				for (uint32_t i = Ft_LBR_ROI.x; i < (Ft_LBR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LBR_ROI.y; j < (Ft_LBR_ROI.y + crop_height); j++) {

						if (j > uint32_t (Ft_LBR_ROI.y + 60))
							
						{
							
							crop_img[(i - Ft_LBR_ROI.x) + (j - Ft_LBR_ROI.y) * crop_width] = 0;
						}
						else{
							crop_img[(i - Ft_LBR_ROI.x) + (j - Ft_LBR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
						}
					}
				}
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				int threshold = 0;
				uint32_t Crop_size = crop_width*crop_height;

				computeHistogram2ThresholdBinary(&threshold, crop_img, crop_width, crop_height);
				for (uint32_t i = 0; i < Crop_size; i++)
				{
					if (crop_img[i] <= 0.6*threshold){
						crop_img[i] = 0;
					}
				}
				

				 DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[2]);
				 ////printf("Anchor_pt pts x %d y %d\n", Anchor_pt[2](0), Anchor_pt[2](1));
#endif
				 int r, c;
				 r = Anchor_pt[2](0) + Ft_LBR_ROI.x;
				 c = Anchor_pt[2](1) + Ft_LBR_ROI.y;
				 Anchor_pt[2].setPosX(r);
				 Anchor_pt[2].setPosY(c);
#ifdef SHOW_CORNER_RECT
				 cv::String info = cv::format("r: %d c: %d ", r, c);
				 cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				 cv::Point2d vp2D;
				 vp2D.x = r;
				 vp2D.y = c;
				 cv::Rect rect(Ft_LBR_ROI.x, Ft_LBR_ROI.y, crop_width, crop_height);
				 cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				 cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}
			//return Anchor_pt[i];
			
//#ifdef Front_Right
#if 1
//Front RTL
			if (Pt_idx == 4)
			{
				const uint32_t crop_width_0 = 131;
				const uint32_t crop_height_0 = 131;
				uint8_t crop_img_0[crop_width_0*crop_height_0];
				mecl::Recti Ft_RTL_ROI;

				//Ft_RTL_ROI.x = 867; Ft_RTL_ROI.y = 353;
				Ft_RTL_ROI.x = 867; Ft_RTL_ROI.y = 353;

				//bool_t LeftHalf = true;
				int halfwidth = (crop_width_0 * 0.75);

				int sum = 0;
				//for (int k = 0; k < n; ++k)
				sum += rand() % 255;
				//pixel_i_j = sum / n;
				for (uint32_t i = Ft_RTL_ROI.x; i < (Ft_RTL_ROI.x + crop_width_0); i++) {
					for (uint32_t j = Ft_RTL_ROI.y; j < (Ft_RTL_ROI.y + crop_height_0); j++) {

						if (i > uint32_t (Ft_RTL_ROI.x + halfwidth))
						{
						sum = rand() % 128;
						crop_img_0[(i - Ft_RTL_ROI.x) + (j - Ft_RTL_ROI.y) * crop_width_0] = sum + Img[(i + j * c_ImageWidth_u32)] / 2;
						}
						else{
						crop_img_0[(i - Ft_RTL_ROI.x) + (j - Ft_RTL_ROI.y) * crop_width_0] = Img[(i + j * c_ImageWidth_u32)];
						}
					}
				}

				//#ifdef SHOW
#if 1
				cv::Mat crop_img_Arry(crop_height_0, crop_width_0, CV_8U, crop_img_0);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
#ifdef AUTOTHRESHOLD
				/*int threshold = 0;
				uint32_t Crop_size = crop_width*crop_height;

				computeHistogram2ThresholdBinary(&threshold, crop_img, crop_width, crop_height);
				for (int i = 0; i < Crop_size; i++)
				{
					if (crop_img[i] > 0.2*threshold)
						crop_img[i] = crop_img[i];
					else
						crop_img[i] = 0;
				}*/
#endif
				DoTemplateMatchingSAD(&crop_img_0[0], imgTemp_Ft_L, crop_width_0, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				////printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				/*int region_in = static_cast<uint32_t>(crop_width_0 / 4);
				int crop_center = static_cast<uint32_t>(crop_width_0 / 2);
				if ((r > static_cast<uint32_t>(crop_width_0)) || (c > static_cast<uint32_t>(crop_width_0))
					||(r < static_cast<uint32_t>(crop_width_0)))
				{
					r = crop_width_0 / 2;
					c = crop_width_0 / 2;
				}*/
				r = Anchor_pt[Pt_idx](0) + Ft_RTL_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RTL_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RTL_ROI.x, Ft_RTL_ROI.y, crop_width_0, crop_height_0);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif
#if 1
			//Front RTR
			if (Pt_idx == 5)
			{
				const uint32_t crop_width_0 = 131;
				const uint32_t crop_height_0 = 131;
				uint8_t crop_img_0[crop_width_0*crop_height_0];
				mecl::Recti Ft_RTR_ROI;

				Ft_RTR_ROI.x = 1007; Ft_RTR_ROI.y = 353;
				/*Ft_RTR_ROI.x = 1027; Ft_RTR_ROI.y = 353;*/

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width_0 / 2);


				for (uint32_t i = Ft_RTR_ROI.x; i < (Ft_RTR_ROI.x + crop_width_0); i++) {
					for (uint32_t j = Ft_RTR_ROI.y; j < (Ft_RTR_ROI.y + crop_height_0); j++) {

						
						crop_img_0[(i - Ft_RTR_ROI.x) + (j - Ft_RTR_ROI.y) * crop_width_0] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				//#ifdef SHOW
#if 1
				cv::Mat crop_img_Arry(crop_height_0, crop_width_0, CV_8U, crop_img_0);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
#ifdef AUTOTHRESHOLD
				/*int threshold = 0;
				uint32_t Crop_size = crop_width*crop_height;

				computeHistogram2ThresholdBinary(&threshold, crop_img, crop_width, crop_height);
				for (int i = 0; i < Crop_size; i++)
				{
					if (crop_img[i] > 0.4*threshold)
						crop_img[i] = crop_img[i];
					else
						crop_img[i] = 0;
				}*/
#endif
				DoTemplateMatchingSAD(&crop_img_0[0], imgTemp_Ft_L, crop_width_0, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				////printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RTR_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RTR_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RTR_ROI.x, Ft_RTR_ROI.y, crop_width_0, crop_height_0);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif
#if 1
			//Front RBL
			if (Pt_idx == 7)
			{
				mecl::Recti Ft_RBL_ROI;

				Ft_RBL_ROI.x = 957; Ft_RBL_ROI.y = 439;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width / 2);

				
				for (uint32_t i = Ft_RBL_ROI.x; i < (Ft_RBL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RBL_ROI.y; j < (Ft_RBL_ROI.y + crop_height); j++) {

						if (j > uint32_t (Ft_RBL_ROI.y + 60))
							//if (i >(Ft_LTR_ROI.x + 50))
						{
							
							crop_img[(i - Ft_RBL_ROI.x) + (j - Ft_RBL_ROI.y) * crop_width] = 0;
						}
					else{
						crop_img[(i - Ft_RBL_ROI.x) + (j - Ft_RBL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
					}
				}
				

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				////printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RBL_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RBL_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RBL_ROI.x, Ft_RBL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif
#if 1
			//Front RBR
			if (Pt_idx == 6)
			{
				mecl::Recti Ft_RBR_ROI;

				Ft_RBR_ROI.x = 1175; Ft_RBR_ROI.y = 439;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width / 2);

				
				for (uint32_t i = Ft_RBR_ROI.x; i < (Ft_RBR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RBR_ROI.y; j < (Ft_RBR_ROI.y + crop_height); j++) {

						if (j > uint32_t (Ft_RBR_ROI.y + 60))
							//if (i >(Ft_LTR_ROI.x + 50))
						{
							
							crop_img[(i - Ft_RBR_ROI.x) + (j - Ft_RBR_ROI.y) * crop_width] = 0;
						}
						else{
						crop_img[(i - Ft_RBR_ROI.x) + (j - Ft_RBR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
						}
					}
				}

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				int threshold = 0;
				uint32_t Crop_size = crop_width*crop_height;

				computeHistogram2ThresholdBinary(&threshold, crop_img, crop_width, crop_height);
				for (uint32_t i = 0; i < Crop_size; i++)
				{
					if (crop_img[i] <= 0.6*threshold)
						crop_img[i] = 0;
				}
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				////printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RBR_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RBR_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RBR_ROI.x, Ft_RBR_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif


		}
#endif //front
//#ifdef LEFT
#if 1
		if (cameraID_e == e_LeftCamAlgo)
		{



			static const uint32_t crop_width = 101;
			static const uint32_t crop_height = 101;			
			uint8_t crop_img[crop_width*crop_height];


			
			//Front left TOP left corner
			if (Pt_idx == 0)
			{
				mecl::Recti Ft_LTL_ROI;

				Ft_LTL_ROI.x = 452; Ft_LTL_ROI.y = 300;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width / 2);

				int sum = 0;
				
				sum += rand() % 255;
				
				for (uint32_t i = Ft_LTL_ROI.x; i < (Ft_LTL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LTL_ROI.y; j < (Ft_LTL_ROI.y + crop_height); j++) {

						if (i < uint32_t (Ft_LTL_ROI.x + 30))
						{
						sum = rand() % 128;
						crop_img[(i - Ft_LTL_ROI.x) + (j - Ft_LTL_ROI.y) * crop_width] = sum + Img[(i + j * c_ImageWidth_u32)] / 2;
						}
						else{
						crop_img[(i - Ft_LTL_ROI.x) + (j - Ft_LTL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
						}
					}
				}

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				int threshold = 0;
				uint32_t Crop_size = crop_width*crop_height;

				computeHistogram2ThresholdBinary(&threshold, crop_img, crop_width, crop_height);
				for (uint32_t i = 0; i < Crop_size; i++)
				{
					if (crop_img[i] <= 0.3*threshold){
						crop_img[i] = 0;
					}
				}
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[0]);// , templateIndex, cam_id);
				////printf("Anchor_pt pts x %d y %d\n", Anchor_pt[0](0), Anchor_pt[0](1));
				int r, c;
				r = Anchor_pt[0](0) + Ft_LTL_ROI.x;
				c = Anchor_pt[0](1) + Ft_LTL_ROI.y;
				Anchor_pt[0].setPosX(r);
				Anchor_pt[0].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LTL_ROI.x, Ft_LTL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}
			//end of LTL
			if (Pt_idx == 1)
			{
				mecl::Recti Ft_LTR_ROI;


				//Ft_LTR_ROI.x = 796; Ft_LTR_ROI.y = 320;
				Ft_LTR_ROI.x = 796; Ft_LTR_ROI.y = 293;

//#ifdef Front_Top
#if 1
				//Ft LTR
				
				memset(crop_img, 0, crop_width*crop_height);
				
				for (uint32_t i = Ft_LTR_ROI.x; i < (Ft_LTR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LTR_ROI.y; j < (Ft_LTR_ROI.y + crop_height); j++) {

						crop_img[(i - Ft_LTR_ROI.x) + (j - Ft_LTR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[1]);
				////printf("Anchor_pt pts x %d y %d\n", Anchor_pt[1](0), Anchor_pt[1](1));
				int r, c;
				r = Anchor_pt[1](0) + Ft_LTR_ROI.x;
				c = Anchor_pt[1](1) + Ft_LTR_ROI.y;
				Anchor_pt[1].setPosX(r);
				Anchor_pt[1].setPosY(c);

#endif
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LTR_ROI.x, Ft_LTR_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}
			// end of LTR
			
			//Front left bottom right
			if (Pt_idx == 2)
			{
				
				mecl::Recti Ft_LBR_ROI;
				Ft_LBR_ROI.x = 892; Ft_LBR_ROI.y = 503;
				//#ifdef LBR
#if 1
				memset(crop_img, 0, crop_width*crop_height);
				for (uint32_t i = Ft_LBR_ROI.x; i < (Ft_LBR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LBR_ROI.y; j < (Ft_LBR_ROI.y + crop_height); j++) {

						//if (j <(Ft_LBR_ROI.y + 30))
						if (i > uint32_t (Ft_LBR_ROI.x + 70))
						{
							int sum = rand() % 128;
							crop_img[(i - Ft_LBR_ROI.x) + (j - Ft_LBR_ROI.y) * crop_width] = sum + Img[(i + j * c_ImageWidth_u32)] / 8;
							//crop_img[(i - Ft_LBR_ROI.x) + (j - Ft_LBR_ROI.y) * crop_width] = 0;
						}
						else{
							crop_img[(i - Ft_LBR_ROI.x) + (j - Ft_LBR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
						}
					}
				}
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
#ifdef AUTOTHRESHOLD
				/*int threshold = 0;
				uint32_t Crop_size = crop_width*crop_height;

				computeHistogram2ThresholdBinary(&threshold, crop_img, crop_width, crop_height);
				for (int i = 0; i < Crop_size; i++)
				{
					if (crop_img[i] > 0.4*threshold)
						crop_img[i] = crop_img[i];
					else
						crop_img[i] = 0;
				}*/
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[2]);
				////printf("Anchor_pt pts x %d y %d\n", Anchor_pt[2](0), Anchor_pt[2](1));
#endif
				int r, c;
				r = Anchor_pt[2](0) + Ft_LBR_ROI.x;
				c = Anchor_pt[2](1) + Ft_LBR_ROI.y;
				Anchor_pt[2].setPosX(r);
				Anchor_pt[2].setPosY(c);
				//return Anchor_pt[2];
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LBR_ROI.x, Ft_LBR_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}
			if (Pt_idx == 3)
			{

				mecl::Recti Ft_LBL_ROI;
				Ft_LBL_ROI.x = 412; Ft_LBL_ROI.y = 523;

				//#ifdef LBL
#if 1
				memset(crop_img, 0, crop_width*crop_height);
				for (uint32_t i = Ft_LBL_ROI.x; i < (Ft_LBL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LBL_ROI.y; j < (Ft_LBL_ROI.y + crop_height); j++) {

							crop_img[(i - Ft_LBL_ROI.x) + (j - Ft_LBL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				

				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[3]);
				////printf("Anchor_pt pts x %d y %d\n", Anchor_pt[3](0), Anchor_pt[3](1));
#endif
				int r, c;
				r = Anchor_pt[3](0) + Ft_LBL_ROI.x;
				c = Anchor_pt[3](1) + Ft_LBL_ROI.y;
				Anchor_pt[3].setPosX(r);
				Anchor_pt[3].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LBL_ROI.x, Ft_LBL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif

			}


			//#ifdef Front_Right
#if 1
			//Front RTL
			if (Pt_idx == 4)
			{
				mecl::Recti Ft_RTL_ROI;

				Ft_RTL_ROI.x = 867; Ft_RTL_ROI.y = 353;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width / 2);


				for (uint32_t i = Ft_RTL_ROI.x; i < (Ft_RTL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RTL_ROI.y; j < (Ft_RTL_ROI.y + crop_height); j++) {

						crop_img[(i - Ft_RTL_ROI.x) + (j - Ft_RTL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RTL_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RTL_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RTL_ROI.x, Ft_RTL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif
#if 1
			//Front RTR
			if (Pt_idx == 5)
			{
				mecl::Recti Ft_RTR_ROI;

				Ft_RTR_ROI.x = 1027; Ft_RTR_ROI.y = 353;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width / 2);

				for (uint32_t i = Ft_RTR_ROI.x; i < (Ft_RTR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RTR_ROI.y; j < (Ft_RTR_ROI.y + crop_height); j++) {


						crop_img[(i - Ft_RTR_ROI.x) + (j - Ft_RTR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RTR_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RTR_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RTR_ROI.x, Ft_RTR_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif
#if 1
			//Front RBL
			if (Pt_idx == 7)
			{
				mecl::Recti Ft_RBL_ROI;

				Ft_RBL_ROI.x = 957; Ft_RBL_ROI.y = 439;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width / 2);


				for (uint32_t i = Ft_RBL_ROI.x; i < (Ft_RBL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RBL_ROI.y; j < (Ft_RBL_ROI.y + crop_height); j++) {

						if (j > uint32_t (Ft_RBL_ROI.y + 60))
							//if (i >(Ft_LTR_ROI.x + 50))
						{
							
							crop_img[(i - Ft_RBL_ROI.x) + (j - Ft_RBL_ROI.y) * crop_width] = 0;
						}
						else{
							crop_img[(i - Ft_RBL_ROI.x) + (j - Ft_RBL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
						}
					}
				}
				

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RBL_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RBL_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RBL_ROI.x, Ft_RBL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif
#if 1
			//Front RBR
			if (Pt_idx == 6)
			{
				mecl::Recti Ft_RBR_ROI;

				Ft_RBR_ROI.x = 1175; Ft_RBR_ROI.y = 439;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width / 2);

				for (uint32_t i = Ft_RBR_ROI.x; i < (Ft_RBR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RBR_ROI.y; j < (Ft_RBR_ROI.y + crop_height); j++) {

						if (j > uint32_t (Ft_RBR_ROI.y + 60))
							//if (i >(Ft_LTR_ROI.x + 50))
						{
							
							crop_img[(i - Ft_RBR_ROI.x) + (j - Ft_RBR_ROI.y) * crop_width] = 0;
						}
						else{
							crop_img[(i - Ft_RBR_ROI.x) + (j - Ft_RBR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
						}
					}
				}

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RBR_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RBR_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RBR_ROI.x, Ft_RBR_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif




		}
#endif
#if 1  //Rear
		if (cameraID_e == e_RearCamAlgo)
		{

			/*static const uint32_t crop_width = 101;
			static const uint32_t crop_height = 101;*/
			static const uint32_t crop_width = 101;
			static const uint32_t crop_height = 101;
			uint8_t crop_img[crop_width*crop_height];

			//Rear left TOP left corner
			if (Pt_idx == 0)
			{
				mecl::Recti Ft_LTL_ROI;

				Ft_LTL_ROI.x = 222; Ft_LTL_ROI.y = 298;

				//bool_t LeftHalf = true;
				

				int sum = 0;
				//for (int k = 0; k < n; ++k)
				sum += rand() % 255;
				//pixel_i_j = sum / n;
				for (uint32_t i = Ft_LTL_ROI.x; i < (Ft_LTL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LTL_ROI.y; j < (Ft_LTL_ROI.y + crop_height); j++) {

						/*if (i >(ft_lt_lt_roi.x + halfwidth))
						{
						sum = rand() % 128;
						crop_img[(i - ft_lt_lt_roi.x) + (j - ft_lt_lt_roi.y) * crop_width] = sum + c_InImage_pu8[(i + j * c_ImageWidth_u32)] / 2;
						}
						else*/
						crop_img[(i - Ft_LTL_ROI.x) + (j - Ft_LTL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				//#ifdef SHOW

				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[0]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[0](0), Anchor_pt[0](1));
				int r, c;
				r = Anchor_pt[0](0) + Ft_LTL_ROI.x;
				c = Anchor_pt[0](1) + Ft_LTL_ROI.y;
				//printf("Anchor_pt pts r %d c %d\n", r, c);
				Anchor_pt[0].setPosX(r);
				Anchor_pt[0].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LTL_ROI.x, Ft_LTL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}
			//end of LTL
			if (Pt_idx == 1)
			{
				mecl::Recti Ft_LTR_ROI;


				Ft_LTR_ROI.x = 358; Ft_LTR_ROI.y = 298;

				
#if 1
				//Rear LTR
				
				memset(crop_img, 0, crop_width*crop_height);

				memset(crop_img, 0, crop_width*crop_height);
				for (uint32_t i = Ft_LTR_ROI.x; i < (Ft_LTR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LTR_ROI.y; j < (Ft_LTR_ROI.y + crop_height); j++) {

						crop_img[(i - Ft_LTR_ROI.x) + (j - Ft_LTR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[1]);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[1](0), Anchor_pt[1](1));
				int r, c;
				r = Anchor_pt[1](0) + Ft_LTR_ROI.x;
				c = Anchor_pt[1](1) + Ft_LTR_ROI.y;
				//printf("Anchor_pt pts r %d c %d\n", r, c);
				Anchor_pt[1].setPosX(r);
				Anchor_pt[1].setPosY(c);

#endif
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LTR_ROI.x, Ft_LTR_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}
			// end of LTR
			if (Pt_idx == 3)
			{
				
				mecl::Recti Ft_LBL_ROI;
				Ft_LBL_ROI.x = 1; Ft_LBL_ROI.y = 449;

				//#ifdef LBL
#if 1
				memset(crop_img, 0, crop_width*crop_height);
				for (uint32_t i = Ft_LBL_ROI.x; i < (Ft_LBL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LBL_ROI.y; j < (Ft_LBL_ROI.y + crop_height); j++) {

						if (j > uint32_t (Ft_LBL_ROI.y + 51))
							//if (i >(Ft_LTR_ROI.x + 50))
						{
							
							crop_img[(i - Ft_LBL_ROI.x) + (j - Ft_LBL_ROI.y) * crop_width] = 0;
						}
						else{
							crop_img[(i - Ft_LBL_ROI.x) + (j - Ft_LBL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
						}
					}
				}
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				

				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[3]);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[3](0), Anchor_pt[3](1));
#endif
				int r, c;
				r = Anchor_pt[3](0) + Ft_LBL_ROI.x;
				c = Anchor_pt[3](1) + Ft_LBL_ROI.y;
				Anchor_pt[3].setPosX(r);
				Anchor_pt[3].setPosY(c);
				//printf("Anchor_pt pts r %d c %d\n", r, c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LBL_ROI.x, Ft_LBL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif

			}
			//Front left bottom right
			if (Pt_idx == 2)
			{
				
				mecl::Recti Ft_LBR_ROI;
				Ft_LBR_ROI.x = 212; Ft_LBR_ROI.y = 449;
				//#ifdef LBR
#if 1
				//memset(crop_img, 0, crop_width*crop_height);
				//for (uint32_t i = Ft_LBR_ROI.x; i < (Ft_LBR_ROI.x + crop_width); i++) {
				//	for (uint32_t j = Ft_LBR_ROI.y; j < (Ft_LBR_ROI.y + crop_height); j++) {

				//		if (j > uint32_t (Ft_LBR_ROI.y + 51))
				//			//if (i >(Ft_LTR_ROI.x + 50))
				//		{
				//			
				//			crop_img[(i - Ft_LBR_ROI.x) + (j - Ft_LBR_ROI.y) * crop_width] = 0;
				//		}
				//		else{
				//			crop_img[(i - Ft_LBR_ROI.x) + (j - Ft_LBR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
				//		}
				//	}
				//}
				memset(crop_img, 0, crop_width*crop_height);
				for (uint32_t i = Ft_LBR_ROI.x; i < (Ft_LBR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LBR_ROI.y; j < (Ft_LBR_ROI.y + crop_height); j++) {

						if (j > uint32_t(Ft_LBR_ROI.y + 51))
							//if (i >(Ft_LTR_ROI.x + 50))
						{

							crop_img[(i - Ft_LBR_ROI.x) + (j - Ft_LBR_ROI.y) * crop_width] = 0;
						}
						else {
							crop_img[(i - Ft_LBR_ROI.x) + (j - Ft_LBR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
						}
					}
				}
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif


				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[2]);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[2](0), Anchor_pt[2](1));
#endif
				int r, c;
				r = Anchor_pt[2](0) + Ft_LBR_ROI.x;
				c = Anchor_pt[2](1) + Ft_LBR_ROI.y;
				Anchor_pt[2].setPosX(r);
				Anchor_pt[2].setPosY(c);
				//printf("Anchor_pt pts r %d c %d\n", r, c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LBR_ROI.x, Ft_LBR_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}
			//return Anchor_pt[i];

			//#ifdef Front_Right
#if 1
			//Front RTL
			if (Pt_idx == 4)
			{
				mecl::Recti Ft_RTL_ROI;

				Ft_RTL_ROI.x = 814; Ft_RTL_ROI.y = 286;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width_1 / 2);

				for (uint32_t i = Ft_RTL_ROI.x; i < (Ft_RTL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RTL_ROI.y; j < (Ft_RTL_ROI.y + crop_height); j++) {

						crop_img[(i - Ft_RTL_ROI.x) + (j - Ft_RTL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RTL_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RTL_ROI.y;
				//printf("Anchor_pt pts r %d c %d\n", r, c);
				Anchor_pt[Pt_idx].setPosX(r);
 				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RTL_ROI.x, Ft_RTL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif
#if 1
			//Front RTR
			if (Pt_idx == 5)
			{
				mecl::Recti Ft_RTR_ROI;

				Ft_RTR_ROI.x = 948; Ft_RTR_ROI.y = 283;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width / 2);


				for (uint32_t i = Ft_RTR_ROI.x; i < (Ft_RTR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RTR_ROI.y; j < (Ft_RTR_ROI.y + crop_height); j++) {

						crop_img[(i - Ft_RTR_ROI.x) + (j - Ft_RTR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RTR_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RTR_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
				//printf("Anchor_pt pts r %d c %d\n", r, c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RTR_ROI.x, Ft_RTR_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif
#if 1
			//Front RBL
			if (Pt_idx == 7)
			{
				

				mecl::Recti Ft_RBL_ROI;

				Ft_RBL_ROI.x = 926; Ft_RBL_ROI.y = 409;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width_1 / 2);

				for (uint32_t i = Ft_RBL_ROI.x; i < (Ft_RBL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RBL_ROI.y; j < (Ft_RBL_ROI.y + crop_height); j++) {

						if (j > uint32_t (Ft_RBL_ROI.y + 71))
							
						{
							
							crop_img[(i - Ft_RBL_ROI.x) + (j - Ft_RBL_ROI.y) * crop_width] = 0;
						}
						else{
							crop_img[(i - Ft_RBL_ROI.x) + (j - Ft_RBL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
						}
					}
				}
				

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RBL_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RBL_ROI.y;
				//printf("Anchor_pt pts r %d c %d\n", r, c);
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RBL_ROI.x, Ft_RBL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif
#if 1
			//Front RBR
			if (Pt_idx == 6)
			{
				const uint32_t crop_width_0 = 151;
				const uint32_t crop_height_0 = 151;
				uint8_t crop_img_0[crop_width_0*crop_height_0];
				mecl::Recti Ft_RBR_ROI;

				//Ft_RBR_ROI.x = 1138; Ft_RBR_ROI.y = 389;
				Ft_RBR_ROI.x = 1088; Ft_RBR_ROI.y = 389;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width_0 / 2);

				
				for (uint32_t i = Ft_RBR_ROI.x; i < (Ft_RBR_ROI.x + crop_width_0); i++) {
					for (uint32_t j = Ft_RBR_ROI.y; j < (Ft_RBR_ROI.y + crop_height_0); j++) {

						if (j > uint32_t (Ft_RBR_ROI.y + 71))
							
						{
							crop_img_0[(i - Ft_RBR_ROI.x) + (j - Ft_RBR_ROI.y) * crop_width_0] = 0;
						}
						else
						{
							crop_img_0[(i - Ft_RBR_ROI.x) + (j - Ft_RBR_ROI.y) * crop_width_0] = Img[(i + j * c_ImageWidth_u32)];
						}
					}
				}

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height_0, crop_width_0, CV_8U, crop_img_0);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img_0[0], imgTemp_Ft_L, crop_width_0, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RBR_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RBR_ROI.y;
				//printf("Anchor_pt pts r %d c %d\n", r, c);
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RBR_ROI.x, Ft_RBR_ROI.y, crop_width_0, crop_height_0);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif

		}
#endif //Rear

//Right
#if 1
		if (cameraID_e == e_RightCamAlgo)
		{
			static const uint32_t crop_width = 101;
			static const uint32_t crop_height = 101;
			uint8_t crop_img[crop_width*crop_height];
			
			if (Pt_idx == 0)
			{
				mecl::Recti Ft_LTL_ROI;

				Ft_LTL_ROI.x = 369; Ft_LTL_ROI.y = 316;

				//bool_t LeftHalf = true;
				

				for (uint32_t i = Ft_LTL_ROI.x; i < (Ft_LTL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LTL_ROI.y; j < (Ft_LTL_ROI.y + crop_height); j++) {

							crop_img[(i - Ft_LTL_ROI.x) + (j - Ft_LTL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				//#ifdef SHOW

				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[0]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[0](0), Anchor_pt[0](1));
				int r, c;
				r = Anchor_pt[0](0) + Ft_LTL_ROI.x;
				c = Anchor_pt[0](1) + Ft_LTL_ROI.y;
				Anchor_pt[0].setPosX(r);
				Anchor_pt[0].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LTL_ROI.x, Ft_LTL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}
			//end of LTL
			if (Pt_idx == 1)
			{
				mecl::Recti Ft_LTR_ROI;


				Ft_LTR_ROI.x = 707; Ft_LTR_ROI.y = 318;

				//#ifdef Front_Top
#if 1
				//Ft LTR
				
				memset(crop_img, 0, crop_width*crop_height);

				for (uint32_t i = Ft_LTR_ROI.x; i < (Ft_LTR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LTR_ROI.y; j < (Ft_LTR_ROI.y + crop_height); j++) {

						
						crop_img[(i - Ft_LTR_ROI.x) + (j - Ft_LTR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[1]);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[1](0), Anchor_pt[1](1));
				int r, c;
				r = Anchor_pt[1](0) + Ft_LTR_ROI.x;
				c = Anchor_pt[1](1) + Ft_LTR_ROI.y;
				Anchor_pt[1].setPosX(r);
 				Anchor_pt[1].setPosY(c);

#endif
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LTR_ROI.x, Ft_LTR_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}
			// end of LTR

			//Front left bottom right
			if (Pt_idx == 2)
			{

				mecl::Recti Ft_LBR_ROI;
				Ft_LBR_ROI.x = 755; Ft_LBR_ROI.y = 534;
				//#ifdef LBR
#if 1
				memset(crop_img, 0, crop_width*crop_height);
				for (uint32_t i = Ft_LBR_ROI.x; i < (Ft_LBR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LBR_ROI.y; j < (Ft_LBR_ROI.y + crop_height); j++) {

							crop_img[(i - Ft_LBR_ROI.x) + (j - Ft_LBR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif

				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[2]);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[2](0), Anchor_pt[2](1));
#endif
				int r, c;
				r = Anchor_pt[2](0) + Ft_LBR_ROI.x;
				c = Anchor_pt[2](1) + Ft_LBR_ROI.y;
				Anchor_pt[2].setPosX(r);
				Anchor_pt[2].setPosY(c);
				//return Anchor_pt[2];
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LBR_ROI.x, Ft_LBR_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}
			if (Pt_idx == 3)
			{

				mecl::Recti Ft_LBL_ROI;
				Ft_LBL_ROI.x = 268; Ft_LBL_ROI.y = 538;

				//#ifdef LBL
#if 1
				memset(crop_img, 0, crop_width*crop_height);
				for (uint32_t i = Ft_LBL_ROI.x; i < (Ft_LBL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_LBL_ROI.y; j < (Ft_LBL_ROI.y + crop_height); j++) {

						crop_img[(i - Ft_LBL_ROI.x) + (j - Ft_LBL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif

				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[3]);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[3](0), Anchor_pt[3](1));
#endif
				int r, c;
				r = Anchor_pt[3](0) + Ft_LBL_ROI.x;
				c = Anchor_pt[3](1) + Ft_LBL_ROI.y;
				Anchor_pt[3].setPosX(r);
				Anchor_pt[3].setPosY(c);
				//return Anchor_pt[3];
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_LBL_ROI.x, Ft_LBL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}
			

			//#ifdef Front_Right
#if 1
			//Front RTL
			if (Pt_idx == 4)
			{
				mecl::Recti Ft_RTL_ROI;

				Ft_RTL_ROI.x = 867; Ft_RTL_ROI.y = 353;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width / 2);

				for (uint32_t i = Ft_RTL_ROI.x; i < (Ft_RTL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RTL_ROI.y; j < (Ft_RTL_ROI.y + crop_height); j++) {

						crop_img[(i - Ft_RTL_ROI.x) + (j - Ft_RTL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RTL_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RTL_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RTL_ROI.x, Ft_RTL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif
#if 1
			//Front RTR
			if (Pt_idx == 5)
			{
				mecl::Recti Ft_RTR_ROI;

				Ft_RTR_ROI.x = 1027; Ft_RTR_ROI.y = 353;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width / 2);

				for (uint32_t i = Ft_RTR_ROI.x; i < (Ft_RTR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RTR_ROI.y; j < (Ft_RTR_ROI.y + crop_height); j++) {

						crop_img[(i - Ft_RTR_ROI.x) + (j - Ft_RTR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RTR_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RTR_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RTR_ROI.x, Ft_RTR_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif
#if 1
			//Front RBL
			if (Pt_idx == 7)
			{
				mecl::Recti Ft_RBL_ROI;

				Ft_RBL_ROI.x = 957; Ft_RBL_ROI.y = 439;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width / 2);


				for (uint32_t i = Ft_RBL_ROI.x; i < (Ft_RBL_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RBL_ROI.y; j < (Ft_RBL_ROI.y + crop_height); j++) {

						if (j > uint32_t (Ft_RBL_ROI.y + 60))
							//if (i >(Ft_LTR_ROI.x + 50))
						{
							
							crop_img[(i - Ft_RBL_ROI.x) + (j - Ft_RBL_ROI.y) * crop_width] = 0;
						}
						else
							crop_img[(i - Ft_RBL_ROI.x) + (j - Ft_RBL_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}
			

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RBL_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RBL_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RBL_ROI.x, Ft_RBL_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif
#if 1
			//Front RBR
			if (Pt_idx == 6)
			{
				mecl::Recti Ft_RBR_ROI;

				Ft_RBR_ROI.x = 1175; Ft_RBR_ROI.y = 439;

				//bool_t LeftHalf = true;
				//int halfwidth = (crop_width / 2);

				for (uint32_t i = Ft_RBR_ROI.x; i < (Ft_RBR_ROI.x + crop_width); i++) {
					for (uint32_t j = Ft_RBR_ROI.y; j < (Ft_RBR_ROI.y + crop_height); j++) {

						if (j > uint32_t (Ft_RBR_ROI.y + 60))
							//if (i >(Ft_LTR_ROI.x + 50))
						{

							crop_img[(i - Ft_RBR_ROI.x) + (j - Ft_RBR_ROI.y) * crop_width] = 0;
						}
						else
							crop_img[(i - Ft_RBR_ROI.x) + (j - Ft_RBR_ROI.y) * crop_width] = Img[(i + j * c_ImageWidth_u32)];
					}
				}

				//#ifdef SHOW
#if 0
				cv::Mat crop_img_Arry(crop_height, crop_width, CV_8U, crop_img);
				imshow("crop_img Arry", crop_img_Arry);
				cv::waitKey(0);
#endif
				DoTemplateMatchingSAD(&crop_img[0], imgTemp_Ft_L, crop_width, &Anchor_pt[Pt_idx]);// , templateIndex, cam_id);
				//printf("Anchor_pt pts x %d y %d\n", Anchor_pt[Pt_idx](0), Anchor_pt[Pt_idx](1));
				int r, c;
				r = Anchor_pt[Pt_idx](0) + Ft_RBR_ROI.x;
				c = Anchor_pt[Pt_idx](1) + Ft_RBR_ROI.y;
				Anchor_pt[Pt_idx].setPosX(r);
				Anchor_pt[Pt_idx].setPosY(c);
#ifdef SHOW_CORNER_RECT
				cv::String info = cv::format("r: %d c: %d ", r, c);
				cv::putText(c3, info, cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
				cv::Point2d vp2D;
				vp2D.x = r;
				vp2D.y = c;
				cv::Rect rect(Ft_RBR_ROI.x, Ft_RBR_ROI.y, crop_width, crop_height);
				cv::rectangle(c3, rect, cv::Scalar(0, 255, 0));
				cv::circle(c3, vp2D, c_ImageWidth_u32 * 0.01, cv::Scalar(0, 0, 255), -1);

#endif
			}

#endif


		}
		//end of Right
#endif
#ifdef SHOW_CORNER_RECT
				cv::imshow("roicenter_rect", c3);
				cv::waitKey(0);
#endif
		return Anchor_pt[Pt_idx];
  	}
	static void Initial_ROI(boolean_T b[c_ImageSize_u32], E_CameraId_t cameraID_e)
	{
		memset(b, 1, c_ImageSize_u32);
		int Ax, Ay, Bx, By;
		int A, B, C, dir;
		int Bot_front = 1000 * vpcs::c_ImageWidth_u32;
		int Bot_rear = 1000 * vpcs::c_ImageWidth_u32;
		for (int r = 0; r < vpcs::c_ImageSize_u32; r++) {

			int height = vpcs::c_ImageWidth_u32;
			int front_row = 320;


			int right_left_botrow = 1050;
			int Px = r % vpcs::c_ImageWidth_u32;
			int Py = r / vpcs::c_ImageWidth_u32;
			switch (cameraID_e)
			{
			case 0:
			{
				//int rear_row = 390;
				bool rowBelow = (r > ((600) * vpcs::c_ImageWidth_u32) - 1);
				//if (rowBelow && !((fabs(Px - 640.0F) < 150.0F) && (Py > rear_row + 100)))
				if (rowBelow)
					b[r] = 1;
				else
					b[r] = 0;

				Ax = 370;
				Ay = 600;
				Bx = 4;
				By = 790;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool left_rowBelow = (dir < 0);
				if ((left_rowBelow))
					b[r] = 0;

				Ax = 860;
				Ay = 600;
				Bx = 500;
				By = 960;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);

				rowBelow = (dir > 0);
				/*if ((rowBelow) )
				b[r] = 0;*/


				Ax = 1090;
				Ay = 600;
				Bx = 1540;
				By = 1200;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool rowAbove = (dir > 0);


				if ((rowBelow) && (rowAbove))
					b[r] = 0;

				Ax = 1670;
				Ay = 600;
				Bx = 1900;
				By = 710;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool right_rowBelow = (dir < 0);
				if ((right_rowBelow))
					b[r] = 0;
				if (r > Bot_front)
					b[r] = 0;
			}
			break;
			case 1:
			{
				bool rowBelow = (r > ((340) * vpcs::c_ImageWidth_u32) - 1);
				//if (rowBelow && !((fabs(Px - 640.0F) < 150.0F) && (Py > rear_row + 100)))
				if (rowBelow)
					b[r] = 1;
				else
					b[r] = 0;
				Ax = 2;
				Ay = 340;
				Bx =1;
				By = 1280;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool left_rowBelow = (dir < 0);
				if ((left_rowBelow))
					b[r] = 0;
				////bool rowBelow = (r > ((right_left_toprow - 1)*height) - 1);
				Ax = 1810;
				Ay = 420;
				Bx = 280;
				By = 1280;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);

				 rowBelow = (dir > 0);
				if ((rowBelow))
					b[r] = 0;

				Ax = 1811;
				Ay = 430;
				Bx = 280;
				By = 1280;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool rowAbove = (dir > 0);


				if ((rowBelow) && (rowAbove))
					b[r] = 0;

				//Ax = 280;
				//Ay = 1280;
				//Bx = 1;
				//By = 1280;
				////int By = 540;

				//A = (Ay - By);
				//B = (Bx - Ax);
				//C = (Ax*By - Ay*Bx);
				//dir = (A*Px + B*Py + C) / (B);

				//bool bot_rowBelow = (dir > 0);
				//if ((bot_rowBelow))
				//	b[r] = 0;
				/////*bool rowAbove = (r < ((620 - 1)*vpcs::c_ImageWidth_u32) - 1);
				////if ( (rowAbove))
				////b[r] = 1;
				////else
				////b[r] = 0;*/

			}
			break;
			case 2:
			{
				//int rear_row = 330;
				bool rowBelow = (r > ((360) * vpcs::c_ImageWidth_u32) - 1);
				//if (rowBelow && !((fabs(Px - 640.0F) < 150.0F) && (Py > rear_row + 100)))
				if (rowBelow)
					b[r] = 1;
				else
					b[r] = 0;

				Ax = 260;
				Ay = 365;
				Bx = 30;
				By = 515;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool left_rowBelow = (dir < 0);
				if ((left_rowBelow))
					b[r] = 0;

				Ax = 400;
				Ay = 365;
				Bx = 257;
				By = 515;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);

				rowBelow = (dir > 0);
				/*if ((rowBelow) )
				b[r] = 0;*/



				Ax = 872;
				Ay = 365;

				Bx = 1017;
				By = 515;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool rowAbove = (dir > 0);


				if ((rowBelow) && (rowAbove))
					b[r] = 0;

				Ax = 1021;
				Ay = 365;
				Bx = 1247;
				By = 515;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool right_rowBelow = (dir < 0);
				if ((right_rowBelow))
					b[r] = 0;

				if (r > Bot_rear)
					b[r] = 0;
			}

			break;
			case 3:
			{
				Ax = 55;
				Ay = 357;
				Bx = 1237;
				By = 360;
				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool left_rowBelow = (dir < 0);
				if ((left_rowBelow))
					b[r] = 0;
				Ax = 1130;
				Ay = 360;
				Bx = 55;
				By = 357;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);

				bool rowBelow = (dir < 0);
				if ((rowBelow))
					b[r] = 0;

				Ax = 1130;
				Ay = 355;
				Bx = 915;
				By = 555;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool rowAbove = (dir > 0);


				if ((rowBelow) && (rowAbove))
					b[r] = 0;

				Ax = 915;
				Ay = 555;
				Bx = 45;
				By = 555;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);

				bool bot_rowBelow = (dir > 0);
				if ((bot_rowBelow))
					b[r] = 0;
			}
			break;
			}
		}
	}
	static void Rough_ROI(boolean_T b[c_ImageSize_u32], E_CameraId_t cameraID_e,  VanPoints::Point2D_s *Corners_Pts)
	{
#ifdef INITIALROI
		boolean_T Initial_Mask[vpcs::c_ImageSize_u32];
		//Initial_ROI(Initial_Mask, e_LeftCamAlgo);
		Initial_ROI(Initial_Mask, cameraID_e);

#ifdef OPENCV_OUT
		cv::Mat Binariz(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
		
		for (int row = 0; row < vpcs::c_ImageHeight_u32; row++)
		{
			for (int col = 0; col < vpcs::c_ImageWidth_u32; col++)

			{

				Binariz.at<uint8_t>(row, col) = Initial_Mask[col + row * vpcs::c_ImageWidth_u32] * 255;
			}
		}
		cv::imshow("MaskOUT", Binariz);
		cv::waitKey(0);
#endif
#endif
		memset(b, 1, c_ImageSize_u32);
		boolean_T left_mask[c_ImageSize_u32];
		memset(left_mask, 1, c_ImageSize_u32);
		int Ax, Ay, Bx, By;
		int A, B, C, dir;
		//int Bot_front = 530 * 1280;
		//int Bot_rear = 580 * 1280;
		int Bot_front = 600 * vpcs::c_ImageWidth_u32;
		int Bot_rear = 700 * vpcs::c_ImageWidth_u32;
		for (int i = 0; i < 4; i++)
		{
			printf("i %d pts x %d y %d\n", i, Corners_Pts[i].x_i32, Corners_Pts[i].y_i32);
		}
#ifdef FORD_QA
		cv::Mat bMask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
		cv::Mat b0Mask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
		cv::Mat b1Mask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
		cv::Mat b2Mask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
		cv::Mat b3Mask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
		cv::Mat b4Mask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
#endif
		for (int r = 0; r < vpcs::c_ImageSize_u32; r++) {

			//int height = 1280;
			//int front_row = 360;


			//int right_left_botrow = 600;
			int Px = r % vpcs::c_ImageWidth_u32;
			int Py = r / vpcs::c_ImageWidth_u32;
			switch (cameraID_e)
			{
			case 0:
			{
				
				int LowerL = mecl::math::min_x(Corners_Pts[0].y_i32, Corners_Pts[1].y_i32);
				
				bool rowBelow = (r > ((LowerL + 2) * vpcs::c_ImageWidth_u32) - 1);
				
				int LowerR = mecl::math::min_x(Corners_Pts[5].y_i32, Corners_Pts[4].y_i32);
				int Lower = mecl::math::min_x(LowerL, LowerR);
				if (((rowBelow)&&(Px<640))|| (((r >((Lower + 25) * vpcs::c_ImageWidth_u32) - 1)) && (Px>640)))

				////U725 I12
				//int Lower = mecl::math::min_x(Corners_Pts[0].y_i32, Corners_Pts[1].y_i32);				
				//bool rowBelow = (r > ((Lower - 2) * 1280) - 1);				
				//Lower = mecl::math::min_x(Corners_Pts[5].y_i32, Corners_Pts[4].y_i32);
				//if (((rowBelow) && (Px<640)) || (((r >((Lower - 2) * 1280) - 1)) && (Px>640)))
				{
					b[r] = 1;
					left_mask[r] = 1;
#ifdef FORD_QA
					bMask1.at<uint8_t>(Py, Px) = b[r];
					/*cv::Mat bMask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
					for (int row = 0; row<800; row++)
					{
						for (int col = 0; col<1280; col++)

						{
							bMask1.at<uint8_t>(row, col) =  250;

						}
					}
					*/
					/*imshow("bMaks1",bMask1);
					cv::waitKey(0);*/

#endif
				}
				else
				{
					b[r] = 0;
					left_mask[r] = 0;
#ifdef FORD_QA
					b0Mask1.at<uint8_t>(Py, Px) = b[r];
					///*cv::Mat b0Mask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);*/
					//for (int row = 0; row<800; row++)
					//{
					//	for (int col = 0; col<1280; col++)

					//	{
					//		b0Mask1.at<uint8_t>(row, col) = 0;

					//	}
					//}
					/*imshow("b0Maks1", b0Mask1);
					cv::waitKey(0);*/
#endif
				}

				/*Ax = 267;
				Ay = 390;
				Bx = 161;
				By = 538;*/
				/*Ax = Corners_Pts[0].x_i32-10;
				Ay = Corners_Pts[0].y_i32-2;
				Bx = Corners_Pts[3].x_i32+15;
				By = Corners_Pts[3].y_i32-2;*/
				Ax = Corners_Pts[0].x_i32-10;
				Ay = Corners_Pts[0].y_i32;
				Bx = Corners_Pts[3].x_i32-10;
				By = Corners_Pts[3].y_i32;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				
				if (B == 0)				
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);

				bool left_rowBelow = (dir < 0);
				if ((left_rowBelow) && (rowBelow))
				//if ((left_rowBelow)) //U725 I12
				{
					b[r] = 0;
					left_mask[r] = 0;
#ifdef FORD_QA
					b1Mask1.at<uint8_t>(Py, Px) = b[r];
					/*cv::Mat b1Mask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);*/
					/*for (int row = 0; row<vpcs::c_ImageHeight_u32; row++)
					{
						for (int col = 0; col<vpcs::c_ImageWidth_u32; col++)

						{
							b1Mask1.at<uint8_t>(row, col) = 0;

						}
					}*/
					/*imshow("b1Maks1", b1Mask1);
					cv::waitKey(0);*/
#endif
				}

				/*Ax = 400;
				Ay = 390;
				Bx = 300;
				By = 553;*/
				Ax = Corners_Pts[1].x_i32+10;
				Ay = Corners_Pts[1].y_i32;
				Bx = Corners_Pts[2].x_i32+10;
				By = Corners_Pts[2].y_i32;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if (B == 0)
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);

				rowBelow = (dir > 0);
				/*if ((rowBelow) )
				b[r] = 0;*/


				/*Ax = 871;
				Ay = 390;
				Bx = 981;
				By = 558;*/
				Ax = Corners_Pts[4].x_i32 - 10;
				Ay = Corners_Pts[4].y_i32;
				Bx = Corners_Pts[7].x_i32 - 10;
				By = Corners_Pts[7].y_i32;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if(B ==0)
					
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);
				bool rowAbove = (dir > 0);


				if ((rowBelow) && (rowAbove))
				{
					b[r] = 0;
					left_mask[r] = 0;
#ifdef FORD_QA
					b2Mask1.at<uint8_t>(Py, Px) = b[r];
					/*cv::Mat b2Mask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);*/
					/*for (int row = 0; row<vpcs::c_ImageHeight_u32; row++)
					{
						for (int col = 0; col<vpcs::c_ImageWidth_u32; col++)

						{
							b2Mask1.at<uint8_t>(row, col) = 0;

						}
					}*/
					/*imshow("b2Maks1", b2Mask1);
					cv::waitKey(0);*/
#endif
				}

				/*Ax = 1002;
				Ay = 390;
				Bx = 1108;
				By = 531;*/
				Ax = Corners_Pts[5].x_i32 + 10;
				Ay = Corners_Pts[5].y_i32;
				Bx = Corners_Pts[6].x_i32 + 10;
				By = Corners_Pts[6].y_i32;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if (B == 0)
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);
				bool right_rowBelow = (dir < 0);
				

				

				if ((right_rowBelow))
				{
					b[r] = 0;
					
#ifdef FORD_QA
					b3Mask1.at<uint8_t>(Py, Px) = b[r];
					/*cv::Mat b3Mask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);*/
					/*for (int row = 0; row<vpcs::c_ImageHeight_u32; row++)
					{
						for (int col = 0; col<vpcs::c_ImageWidth_u32; col++)

						{
							b3Mask1.at<uint8_t>(row, col) = 0;

						}
					}*/
					/*imshow("b3Maks1", b3Mask1);
					cv::waitKey(0);*/
#endif
				}
				if (r > Bot_front)
				{
					b[r] = 0;
					left_mask[r] = 0;
#ifdef FORD_QA
					b4Mask1.at<uint8_t>(Py, Px) = b[r];
					/*cv::Mat b4Mask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);*/
					/*for (int row = 0; row<vpcs::c_ImageHeight_u32; row++)
					{
						for (int col = 0; col<vpcs::c_ImageWidth_u32; col++)

						{
							b4Mask1.at<uint8_t>(row, col) = 0;

						}
				}*/
					/*imshow("b4Maks1", b4Mask1);
					cv::waitKey(0);*/
#endif
				}
#ifdef ENHANCE_MASK
				if ((Px < 640) && (left_mask[r]))
					b[r] = 1;
#endif
#ifdef INITIALROI
				if (Initial_Mask[r] || b[r])
					b[r] = 1;
#endif
			}
			break;
			case 1:
			{
				/*Ax = 487;
				Ay = 343;
				Bx = 457;
				By = 553;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool left_rowBelow = (dir < 0);
				if ((left_rowBelow))
					b[r] = 0;*/
				//bool rowBelow = (r > ((right_left_toprow - 1)*height) - 1);
				Ax = 843;
				Ay = 334;
				Bx = 487;
				By = 343;

				Ax = Corners_Pts[1].x_i32 - 10;
				Ay = Corners_Pts[1].y_i32;
				Bx = Corners_Pts[0].x_i32 - 10;
				By = Corners_Pts[0].y_i32;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if (B == 0)
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);

				bool rowBelow = (dir < 0);
				if ((rowBelow))
					b[r] = 0;

				/*Ax = 843;
				Ay = 334;
				Bx = 915;
				By = 518;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				dir = (A*Px + B*Py + C) / (B);
				bool rowAbove = (dir > 0);


				if ((rowBelow) && (rowAbove))
					b[r] = 0;*/

				Ax = 915;
				Ay = 528;
				Bx = 457;
				By = 543;
				Ax = Corners_Pts[2].x_i32 + 10;
				Ay = Corners_Pts[2].y_i32;
				Bx = Corners_Pts[3].x_i32 + 10;
				By = Corners_Pts[3].y_i32;
				//int By = 540;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if (B == 0)
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);

				bool bot_rowBelow = (dir > 0);
				if ((bot_rowBelow))
					b[r] = 0;
				// right end
				Ax = 1125;
				Ay = 330;
				Bx = 1275;
				By = 541;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if (B == 0)
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);
				bool right_rowBelow = (dir < 0);
				if ((right_rowBelow))
					b[r] = 0;
#ifdef INITIALROI
				if (Initial_Mask[r] || b[r])
					b[r] = 1;
#endif

			}
			break;
			case 2:
			{
				int Lower = mecl::math::min_x(Corners_Pts[0].y_i32, Corners_Pts[1].y_i32);
				//bool rowBelow = (r > ((Corners_Pts[0].y_i32-3) * 1280) - 1);
				bool rowBelow = (r > ((Lower - 2) * vpcs::c_ImageWidth_u32) - 1);
				//if (rowBelow && !((fabs(Px - 640.0F) < 150.0F) && (Py > rear_row + 100)))
				Lower = mecl::math::min_x(Corners_Pts[5].y_i32, Corners_Pts[4].y_i32);
				//02_17 lower is to take max
				//int Lower = mecl::math::max_x(Corners_Pts[0].y_i32, Corners_Pts[1].y_i32);
				////bool rowBelow = (r > ((Corners_Pts[0].y_i32-3) * vpcs::c_ImageWidth_u32) - 1);
				//bool rowBelow = (r > ((Lower - 2) * vpcs::c_ImageWidth_u32) - 1);
				////if (rowBelow && !((fabs(Px - 640.0F) < 150.0F) && (Py > rear_row + 100)))
				//Lower = mecl::math::m_x(Corners_Pts[5].y_i32, Corners_Pts[4].y_i32);
				if (((rowBelow)&&(Px<(vpcs::c_ImageHeight_u32>>1)))|| (((r >((Lower - 2) * vpcs::c_ImageWidth_u32) - 1)) && (Px>(vpcs::c_ImageHeight_u32>>1))))
					b[r] = 1;
				else
					b[r] = 0;

				/*Ax = 267;
				Ay = 390;
				Bx = 161;
				By = 538;*/
				Ax = Corners_Pts[0].x_i32 - 10;
				Ay = Corners_Pts[0].y_i32;
				Bx = Corners_Pts[3].x_i32 - 10;
				By = Corners_Pts[3].y_i32;
				

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if (B == 0)
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);
				bool left_rowBelow = (dir < 0);
				if ((left_rowBelow))
					b[r] = 0;

				/*Ax = 437;
				Ay = 330;
				Bx = 327;
				By = 521;*/
				Ax = Corners_Pts[1].x_i32 + 10;
				Ay = Corners_Pts[1].y_i32;
				Bx = Corners_Pts[2].x_i32 + 10;
				By = Corners_Pts[2].y_i32;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if (B == 0)
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);

				rowBelow = (dir > 0);
				/*if ((rowBelow) )
				b[r] = 0;*/


				/*Ax = 822;
				Ax = 842;
				Ay = 330;
				Bx = 937;
				Bx = 957;
				By = 526;*/
				Ax = Corners_Pts[4].x_i32 - 10;
				Ay = Corners_Pts[4].y_i32;
				Bx = Corners_Pts[7].x_i32 - 10;
				By = Corners_Pts[7].y_i32;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if (B == 0)
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);
				bool rowAbove = (dir > 0);


				if ((rowBelow) && (rowAbove))
					b[r] = 0;

				/*Ax = 958;
				Ay = 330;
				Bx = 1082;
				By = 489;
*/
				Ax = Corners_Pts[5].x_i32 + 10;
				Ay = Corners_Pts[5].y_i32;
				Bx = Corners_Pts[6].x_i32 + 10;
				By = Corners_Pts[6].y_i32;
				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if (B == 0)
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);
#ifdef INITIALROI
				if (Initial_Mask[r] || b[r])
					b[r] = 1;
#endif
				bool right_rowBelow = (dir < 0);
				if ((right_rowBelow))
					b[r] = 0;

				if (r > Bot_rear)
					b[r] = 0;
				
			}

			break;
			case 3:
			{
				Ax = 120;
				Ay = 384;
				Bx = 0;
				//int By = 400;
				By = 586;
				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if (B == 0)
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);
				//bool rowBelow = (r >((right_left_toprow - 1)*height) - 1);
				bool rowBelow = (dir < 0);
				if ((rowBelow))
					b[r] = 0;

				Ax = 1085;
				Ay = 435;
				Bx = 1225;
				By = 606;
				Ax = Corners_Pts[1].x_i32 - 10;
				Ay = Corners_Pts[1].y_i32;
				Bx = Corners_Pts[0].x_i32 - 10;
				By = Corners_Pts[0].y_i32;

				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if (B == 0)
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);
				bool right_rowBelow = (dir < 0);
				if ((right_rowBelow))
					b[r] = 0;

				//Ax = 1225;
				//Ay = 606;
				//Bx = 0;
				//By = 614;
				////int By = 540;
				Ax = Corners_Pts[2].x_i32 + 10;
				Ay = Corners_Pts[2].y_i32;
				Bx = Corners_Pts[3].x_i32 + 10;
				By = Corners_Pts[3].y_i32;
				A = (Ay - By);
				B = (Bx - Ax);
				C = (Ax*By - Ay*Bx);
				if (B == 0)
					dir = (A*Px + B*Py + C) / (0.000000001);
				else
					dir = (A*Px + B*Py + C) / (B);

				bool bot_rowBelow = (dir > 0);
				if ((bot_rowBelow))
					b[r] = 0;
#ifdef INITIALROI
				if (Initial_Mask[r] || b[r])
					b[r] = 1;
#endif
			}
			break;
			}
			
		}
#ifdef FORD_QA
		imshow("bMaks1", bMask1);
		cv::waitKey(0);
		imshow("b0Maks1", b0Mask1);
		cv::waitKey(0); 
		imshow("b1Maks1", b1Mask1);
		cv::waitKey(0); 
		imshow("b2Maks1", b2Mask1);
		cv::waitKey(0); 
		imshow("b3Maks1", b3Mask1);
		cv::waitKey(0);
		imshow("b4Maks1", b4Mask1);
		cv::waitKey(0);
#endif
	}
	/*Given a src float32_t pointer of size width * height covultes the image by a kernel of size*size and saves in dst*/
	static void convoluteU82float(float32_t* o_Dst_pf32,
		const uint8_t* i_Src_pu8,
		sint32_t i_Width_s32,
		sint32_t i_Height_s32,
		const float32_t* i_Kernel_pf32,
		sint32_t i_Size_s32)
	{
		for (sint32_t j = 0; j < (i_Height_s32 - i_Size_s32 + 1); j++)
		{
			for (sint32_t i = 0; i < (i_Width_s32 - i_Size_s32 + 1); i++)
			{
				float32_t v_Sum_f32 = 0;
				for (sint32_t k = 0; k < i_Size_s32; k++)
				{
					for (sint32_t l = 0; l < i_Size_s32; l++)
					{
						v_Sum_f32 += (static_cast<float32_t>(i_Src_pu8[i + k + (j + l) * i_Width_s32])
							* i_Kernel_pf32[k + l * i_Size_s32]);
					}
				}
				o_Dst_pf32[i + i_Size_s32 / 2 + (j + i_Size_s32 / 2) * i_Width_s32] = v_Sum_f32;
			}
		}

		//Replicate border style

		//Left Border
		for (sint32_t j = (i_Size_s32 / 2); j < i_Height_s32; j++)
		{
			for (sint32_t i = 0; i < (i_Size_s32 / 2); i++)
			{
				o_Dst_pf32[i + j * i_Width_s32] = o_Dst_pf32[i + i_Size_s32 / 2 + (j)* i_Width_s32];
			}
		}

		//Top Border
		for (sint32_t j = 0; j < (i_Size_s32 / 2); j++)
		{
			for (sint32_t i = 0; i < i_Width_s32; i++)
			{
				o_Dst_pf32[i + j * i_Width_s32] = o_Dst_pf32[i + (j + i_Size_s32 / 2) * i_Width_s32];
			}
		}

		//Right Border
		for (sint32_t j = i_Size_s32 / 2; j < i_Height_s32; j++)
		{
			for (sint32_t i = i_Width_s32 - i_Size_s32 / 2; i < i_Width_s32; i++)
			{
				o_Dst_pf32[i + j * i_Width_s32] = o_Dst_pf32[i - i_Size_s32 / 2 + j * i_Width_s32];
			}
		}

		//Bottom Border
		for (sint32_t j = i_Height_s32 - i_Size_s32 / 2; j < i_Height_s32; j++)
		{
			for (sint32_t i = 0; i < i_Width_s32; i++)
			{
				o_Dst_pf32[i + j * i_Width_s32] = o_Dst_pf32[i + (j - i_Size_s32 / 2) * i_Width_s32];
			}
		}
	}

	static void gaussionReplacement7x7(float32_t* o_Dst_pf32,
		const uint8_t* i_Src_pu8)
	{
		const float32_t c_Sigma_f32 = 1.25;
		//const float32_t c_Sigma_f32 = 1.0;
		//const float32_t c_Sigma_f32 = 3.5;
		float32_t v_Values_af32[49];
		float32_t v_Sumv_f32 = 0;
		for (sint32_t v_K_s32 = 0; v_K_s32 < 7; v_K_s32++)
		{
			for (sint32_t l = 0; l < 7; l++)
			{
				v_Values_af32[v_K_s32 + l * 7] = 1.0F
					/ (2.0F * static_cast<float32_t>(M_PI) * c_Sigma_f32 * c_Sigma_f32)
					* powf(2.71828f,
						static_cast<float32_t>(-1 * (mecl::math::abs_x<sint32_t>(3 - v_K_s32)* mecl::math::abs_x<sint32_t>(3 - v_K_s32) + mecl::math::abs_x<sint32_t>(3 - l)* mecl::math::abs_x<sint32_t>(3 - l)))
						/ (2.0F * c_Sigma_f32 * c_Sigma_f32));
				v_Sumv_f32 += v_Values_af32[v_K_s32 + l * 7];
			}
		}
		for (sint32_t v_I_s32 = 0; v_I_s32 < 49; v_I_s32++)
		{
			v_Values_af32[v_I_s32] = v_Values_af32[v_I_s32] / v_Sumv_f32;
		}
		convoluteU82float(o_Dst_pf32, i_Src_pu8, c_ImageWidth_u32, c_ImageHeight_u32, &v_Values_af32[0], 7);
	}

	static void Smooth_sobelReplacement(sint32_t i_J_s32,
		sint32_t i_I_s32,
		const float32_t* i_TempNcvSmoothed_pf32,
		float32_t& o_SobelGradientX_rf32,
		float32_t& o_SobelGradientY_rf32)
	{

		const float32_t c_Xkernel_af32[9] = { -1.0, 0.0, 1.0, -2.0, 0.0, 2.0, -1.0, 0.0, 1.0 };
		const float32_t c_Ykernel_af32[9] = { -1.0, -2.0, -1.0, 0.0, 0.0, 0.0, 1.0, 2.0, 1.0 };

		sint32_t size = 3;

		o_SobelGradientX_rf32 = 0;
		o_SobelGradientY_rf32 = 0;

		for (sint32_t k = 0; k < size; k++)
		{
			for (sint32_t l = 0; l < size; l++)
			{
				o_SobelGradientX_rf32 += i_TempNcvSmoothed_pf32[i_J_s32 + k + (i_I_s32 + l) * c_ImageWidth_u32]
					* c_Xkernel_af32[k + l * size];
				o_SobelGradientY_rf32 += i_TempNcvSmoothed_pf32[i_J_s32 + k + (i_I_s32 + l) * c_ImageWidth_u32]
					* c_Ykernel_af32[k + l * size];
			}
		}

	}

	VanPoints::VanPoints()
		: clusterThresh_f32(3.0F) // Clustering threshold
		//: clusterThresh_f32(6.5F) // Clustering threshold
		,
		cameraID_e(e_FrontCamAlgo)
	{
	}

	VanPoints::~VanPoints()
	{

	}

	VanPoints::VanPoint_s VanPoints::findVanishingPoints(uint8_t* b_Img_pu8,
		sint32_t i_lineLength_s32,
		sint32_t i_modelSize_s32)
	{
		VanPoint_s v_OutStruct_s;
		uint32_t v_LineNum_u32 = 0;
		extractLongLine(reinterpret_cast<uint8_t*>(b_Img_pu8), i_lineLength_s32, &v_LineNum_u32, &v_OutStruct_s.lines_as[0],
			c_ImageWidth_u32, c_ImageHeight_u32);

		v_OutStruct_s.clusters = m_clusters;

		if (v_FLAG_t == shmdata::e_EOL_CALIBRATION_IN_PROCESS)
		{
			/*
			if (v_LineNum_u32 >= LINES_SIZE_MAX)
			{
				v_FLAG_t = shmdata::TARGET_IDENTIFICATION_ERROR_LEFT;
				v_FLAG_t = shmdata::TARGET_IDENTIFICATION_ERROR_RIGHT;
#ifdef DEBUG_LOG
				log_printf("NOT_TARGET_LINE_FEATURE_DETECTED \r\n");
#endif
#ifdef DEBUG_LOG2
				log_printf("NOT_TARGET_LINE_FEATURE_DETECTED \r\n");
#endif
			}
			else if (v_LineNum_u32 < 2)
			{
				v_FLAG_t = shmdata::TARGET_IDENTIFICATION_ERROR_LEFT;
				v_FLAG_t = shmdata::TARGET_IDENTIFICATION_ERROR_RIGHT;
#ifdef DEBUG_LOG
				log_printf("TARGET_FEATURE_MISSING \r\n");
#endif
#ifdef DEBUG_LOG2
				log_printf("TARGET_FEATURE_MISSING \r\n");
#endif
			}
			else
			{
			
			*/

#ifdef DEBUG_LOG

				log_printf("find VanPoints \r\n");
#endif
				// Tardif's vanishing point detection algorithm starts here
				sint32_t vpNum;
				findVanPoints(i_modelSize_s32, &v_OutStruct_s.lines_as[0], v_LineNum_u32, &vpNum, &v_OutStruct_s.vanPoints_as[0],
					i_lineLength_s32);
#ifdef DEBUG_LOG
				log_printf("find VanPoints Complete \r\n");
#endif
			//}
		}
		v_OutStruct_s.lineNum = v_LineNum_u32;
		v_OutStruct_s.clusters = m_clusters;
#ifdef PRINT_DEBUG
		
		vm_cprintf("cameraID_e %d i_lineLength_s32 %d v_LineNum_u32: %d\n", cameraID_e, i_lineLength_s32, v_LineNum_u32);
#endif
		return v_OutStruct_s;
	}

	static void computeHistogram2Threshold(int* threshold, uint8_t* v_GrayImage_o, boolean_T* Mask_output)
	{
		unsigned hist[256];
		unsigned left_hist[256];
		unsigned right_hist[256];

		memset(hist, 0, 256);
		memset(left_hist, 0, 256);
		memset(right_hist, 0, 256);
		for (int i = 0; i < c_ImageSize_u32; i++) {
			if (Mask_output[i] == 1) {
				int value = v_GrayImage_o[i];
				hist[value]++;
			}
			
		}
	
	
		{
			// Compute threshold
			// Init variables
			float sum = 0;
			float sumB = 0;
			int q1 = 0;
			int q2 = 0;
			float varMax = 0;

			// Auxiliary value for computing m2
			for (int i = 0; i < 256; i++) {
				sum += i * ((int)hist[i]);
			}

			for (int i = 0; i < 256; i++) {
				// Update q1
				q1 += hist[i];
				if (q1 == 0)
					continue;
				// Update q2
				q2 = c_ImageSize_u32 - q1;

				if (q2 == 0)
					break;
				// Update m1 and m2
				sumB += (float)(i * ((int)hist[i]));
				float m1 = sumB / q1;
				float m2 = (sum - sumB) / q2;

				// Update the between class variance
				float varBetween = (float)q1 * (float)q2 * (m1 - m2) * (m1 - m2);

				// Update the threshold if necessary
				if (varBetween > varMax) {
					varMax = varBetween;
					*threshold = i;
				}
			}
		}
		//return threshold;
	}

	void VanPoints::extractLongLine(uint8_t* b_Img_pu8,
		uint32_t i_MinLen_u32,
		uint32_t* o_Lnum_pu32,
		Line_s* o_Lines_ps,
		uint32_t i_Width_u32,
		uint32_t i_Height_u32) const
	{

		AssertMsg(b_Img_pu8 != NULL, "Error. No input image for long line extraction.\n");
		if (b_Img_pu8 == NULL)
		{
			v_FLAG_t = shmdata::TARGET_NOT_FOUND_INIT;
			
			//if (cameraID_e == e_LeftCamAlgo)
			//{
			//	v_FLAG_t = shmdata::TARGET_IDENTIFICATION_ERROR_LEFT;
			//}
			//if (cameraID_e == e_RightCamAlgo)
			//{
			//	v_FLAG_t = shmdata::TARGET_IDENTIFICATION_ERROR_RIGHT;
			//}
		}
		else
		{
			uint32_t i;
			uint32_t j;
			uint32_t k;
			uint32_t lineNum = 0;
			
			float32_t v_TempNcv_af32[c_ImageSize_u32];

#ifdef OPENCV
			IplImage * grayImgF;
			IplImage * grayImgU;
			grayImgU = cvCreateImage(cvSize(i_Width_u32, i_Height_u32), IPL_DEPTH_8U, 1);
			memcpy(grayImgU->imageData, b_Img_pu8, c_ImageWidth_u32 * c_ImageHeight_u32);
			grayImgF = cvCreateImage(cvSize(i_Width_u32, i_Height_u32), IPL_DEPTH_32F, 1);
			cvConvertScale(grayImgU, grayImgF);
			CvMat * dst = cvCreateMat(i_Height_u32, i_Width_u32, CV_8UC1);

#endif

			imtp::Image<uint8_t> v_GrayImage_o(c_ImageWidth_u32, c_ImageHeight_u32, &b_Img_pu8[0], 1);


			gaussionReplacement7x7(&v_TempNcv_af32[0], b_Img_pu8); // Smooth the image with Gaussian kernel

			uint8_t v_Edges_au8[c_ImageSize_u32];
			uint8_t v_Dir_au8[c_ImageSize_u32];
			memset(v_Edges_au8, 0, c_ImageSize_u32);

			imtp::Image<uint8_t> v_EdgesImage_o(c_ImageWidth_u32, c_ImageHeight_u32, &v_Edges_au8[0], 1);
			imtp::Image<uint8_t> v_DirImage_o(c_ImageWidth_u32, c_ImageHeight_u32, &v_Dir_au8[0], 1);

			imtp::Image<float32_t> v_MagImage_o(c_ImageWidth_u32, c_ImageHeight_u32, &v_TempNcv_af32[0], 1); //reuse temp pointer to save memory

//#ifdef OPENCV_OUT
#ifdef Running_Canny_Filter
			/*IplImage * grayImgF1;
			IplImage * grayImgU1;
			grayImgU1 = cvCreateImage(cvSize(c_ImageWidth_u32, c_ImageHeight_u32), IPL_DEPTH_8U, 1);
			memcpy(grayImgU1->imageData, b_Img_pu8, c_ImageWidth_u32 * c_ImageHeight_u32);
			grayImgF1 = cvCreateImage(cvSize(c_ImageWidth_u32, c_ImageHeight_u32), IPL_DEPTH_32F, 1);
			cvConvertScale(grayImgU1, grayImgF1);*/
			cv::Mat dst1(c_ImageHeight_u32, c_ImageWidth_u32,  CV_8UC1);
			cv::Mat edge_src(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
			
			
			//cvCanny(grayImgU1, dst1, 80, 110); // Canny edge extraction
			cv::Canny(edge_src, dst1, 80, 110);
			cv::imshow("OpenCV edge", dst1);
			cvWaitKey(0);

#ifdef DEBUG_LOG
			log_printf("Running Canny Filter \r\n");
#endif
#endif
#ifdef SHOW_CORNER_RECT



			cv::Mat Iimg(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1, b_Img_pu8);
			cv::Mat c3;
			cv::cvtColor(Iimg, c3, CV_GRAY2BGR);


#endif

		boolean_T Mask_output[vpcs::c_ImageSize_u32];
		//uint8_t CamNum = 0;

		if (cameraID_e == e_FrontCamAlgo)
			{
#ifdef HULLMASK
			bool_t v_hull_mask[c_ImageSize_u32];
			memset(v_hull_mask, 0, c_ImageSize_u32);
			imtp::Image<bool_t> hull_mask(vpcs::c_ImageWidth_u32, vpcs::c_ImageHeight_u32, &v_hull_mask[0],1);
			mecl::Rect rct,rctR;
			
			Point inhullL[] = 
			{ {1,	948},
			{576,	626 },
			{ 832,	626},
		{ 410,	1100},
		{ 160,	1080}};

			/*{ { 401,	385 },
			{ 266,	392 },
			{ 140,	554 },
			{ 285,	597 },
			{ 400,	418 } };*/
			Point inhullR[] =
			{ {1498,	994},
			{1136,	628 },
			{1418,	620 },
			{1910,	852 },
			{1800,	998} };

			/*{ { 873,	400 },
			{ 908,	397 },
			{ 1005,	398 },
			{ 1120,	526 },
			{ 1129,	568 },
			{ 985,	605 } };*/
			const int hullSize = sizeof(inhullL) / sizeof(inhullL[0]);
			const int hullRSize = sizeof(inhullR) / sizeof(inhullR[0]);
			Point hullL[hullSize];
			Point hullR[hullRSize];
			int hn,hnR;
			//convex_hull(inhullL, hullSize, hullL);
			buildConvexHull(inhullL, hullSize, hullL, hn);
			isInsideConvexHull(hullL, hn, hull_mask, &rct);
			//convex_hull(inhullL, hullSize, hullL);
			buildConvexHull(inhullR, hullSize, hullR, hnR);
			isInsideConvexHull(hullR, hnR, hull_mask, &rctR);
#ifdef OPENCV_OUT
			IplImage * hull_mask_show;
			hull_mask_show = cvCreateImage(cvSize(i_Width_u32, i_Height_u32), IPL_DEPTH_8U, 1);
			memset(hull_mask_show->imageData, 0, c_ImageSize_u32);
			for (int x = rct.x; x<rct.x + rct.width; x++)
				for (int y = rct.y; y < rct.y + rct.height; y++)
				{
					
					if (hull_mask.data_px[x + y * c_ImageWidth_u32])
					{

						hull_mask_show->imageData[x + y * c_ImageWidth_u32] = 255;

					}
				}
			for (int x = rctR.x; x<rctR.x + rctR.width; x++)
				for (int y = rctR.y; y < rctR.y + rctR.height; y++)
				{

					if (hull_mask.data_px[x + y * c_ImageWidth_u32])
					{

						hull_mask_show->imageData[x + y * c_ImageWidth_u32] = 255;

					}
				}
			//cv::Mat fisheye_mask_show(i_Height_u32, i_Width_u32, CV_8UC1, fisheye_mask);


			cvShowImage("hull_mask_show", hull_mask_show);
			cvWaitKey(0);


#endif
			memcpy(Mask_output, hull_mask.data_px, c_ImageSize_u32);
#else
			//CamNum = 0;
#if 1	

				mecl::Recti roi;
				//sint32_t x = 0;
				//sint32_t y = 0;

				static const uint32_t Template_sz = 23;
				//Front left
#ifdef READ_EXTERNAL
				//cv::Mat Template_Ft_LTL = cv::imread(".\\Images\\Flatten\\Ft_LTL_temp2_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				//cv::Mat Template_Ft_LTL = cv::imread(".\\Images\\Flatten\\Ft_LTL_temp_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				cv::Mat Template_Ft_LTL = cv::imread(".\\Images\\Flatten\\Ft_LTL_temp1_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				//cv::Mat Template_Ft_LTL = cv::imread(".\\Images\\Flatten\\Ft_LTL_temp3_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				//cv::Mat Template_Ft_LTL = cv::imread(".\\Images\\Flatten\\Ft_LTL_temp4_ln3.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				cv::Mat Template_Ft_LTR = cv::imread(".\\Images\\Flatten\\Ft_LTR_temp_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				cv::Mat Template_Ft_LBL = cv::imread(".\\Images\\Flatten\\Ft_LBL_temp_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				cv::Mat Template_Ft_LBR = cv::imread(".\\Images\\Flatten\\Ft_LBR_temp_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				//Front Right
				cv::Mat Template_Ft_RTL = cv::imread(".\\Images\\Flatten\\Ft_RTL_temp_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				//cv::Mat Template_Ft_RTL = cv::imread(".\\Images\\Flatten\\Ft_RTL_temp1_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				cv::Mat Template_Ft_RTR = cv::imread(".\\Images\\Flatten\\Ft_RTR_temp_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				//cv::Mat Template_Ft_RTR = cv::imread(".\\Images\\Flatten\\Ft_RTR.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				//cv::Mat Template_Ft_RTR = cv::imread(".\\Images\\Flatten\\Ft_RTR_temp1_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				cv::Mat Template_Ft_RBL = cv::imread(".\\Images\\Flatten\\Ft_RBL_temp_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
				cv::Mat Template_Ft_RBR = cv::imread(".\\Images\\Flatten\\Ft_RBR_temp_ln1.bmp", CV_LOAD_IMAGE_GRAYSCALE);			
#endif
				//front left left

				uint8_t imgTemp_Ft_L[8][Template_sz * Template_sz];

				
				memcpy(imgTemp_Ft_L[0], imgTemp_Ft_L_0, Template_sz * Template_sz);
				memcpy(imgTemp_Ft_L[1], imgTemp_Ft_L_1, Template_sz * Template_sz);
				memcpy(imgTemp_Ft_L[2], imgTemp_Ft_L_2, Template_sz * Template_sz);
				memcpy(imgTemp_Ft_L[3], imgTemp_Ft_L_3, Template_sz * Template_sz);

				memcpy(imgTemp_Ft_L[4], imgTemp_Ft_L_4, Template_sz * Template_sz);
				memcpy(imgTemp_Ft_L[5], imgTemp_Ft_L_5, Template_sz * Template_sz);
				memcpy(imgTemp_Ft_L[6], imgTemp_Ft_L_6, Template_sz * Template_sz);
				memcpy(imgTemp_Ft_L[7], imgTemp_Ft_L_7, Template_sz * Template_sz);
				

				mecl::core::Point2D<uint32_t> Corners_Pts[8], Corners_PtsT[8];
				VanPoints::Point2D_s Corners_cls[8];
				uint8_t CornerMatchingUseImg[c_ImageSize_u32];
				memcpy(CornerMatchingUseImg, v_GrayImage_o.data_px, c_ImageSize_u32);

				for (int i = 0; i < 8; i++)
				{
					uint8_t imgTemp_Ft_L_Single[Template_sz * Template_sz];
					memcpy(imgTemp_Ft_L_Single, imgTemp_Ft_L[i], 529);
					//Corners_PtsT[i] = CornerMatch(cameraID_e, &v_GrayImage_o.data_px[0], imgTemp_Ft_L[i], i, &Corners_Pts[i]);
#ifdef SHOW_CORNER_RECT
					Corners_PtsT[i] = CornerMatch(cameraID_e, &CornerMatchingUseImg[0], &imgTemp_Ft_L_Single[0], i, &Corners_Pts[i], c3);
#else
					Corners_PtsT[i] = CornerMatch(cameraID_e, &CornerMatchingUseImg[0], &imgTemp_Ft_L_Single[0], i, &Corners_Pts[i]);
#endif

					printf("i %d pts x %d y %d\n", i, Corners_Pts[i](0), Corners_Pts[i](1));
					printf("i %d Corners_PtsT pts x %d y %d\n", i, Corners_PtsT[i](0), Corners_PtsT[i](1));
					Corners_cls[i].x_i32 = Corners_PtsT[i](0);
					Corners_cls[i].y_i32 = Corners_PtsT[i](1);
				}
#endif
				Rough_ROI(Mask_output, cameraID_e, Corners_cls);
				
#ifdef FORD_QA
				cv::Mat Mask1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
				for (int row = 0; row<vpcs::c_ImageHeight_u32; row++)
				{
					for (int col = 0; col<vpcs::c_ImageWidth_u32; col++)

					{
						Mask1.at<uint8_t>(row, col) = Mask_output[col + row * vpcs::c_ImageWidth_u32] * 255;

					}
				}
				imshow("Maks1", Mask1);
				cv::waitKey(0);

#endif
#endif
			}

				if (cameraID_e == e_LeftCamAlgo)
				{

#ifdef HULLMASK
					bool_t v_hull_mask[c_ImageSize_u32];
					memset(v_hull_mask, 0, c_ImageSize_u32);
					imtp::Image<bool_t> hull_mask(vpcs::c_ImageWidth_u32, vpcs::c_ImageHeight_u32, &v_hull_mask[0], 1);
					mecl::Rect rct, rctR;

					Point inhullL[] =
					{ {100,	324},
					{146,	1264 },
					{1480,	650 },
					{1476,	392} };


					/*{ { 1,	948 },
					{ 576,	626 },
					{ 832,	626 },
					{ 410,	1100 },
					{ 160,	1080 } };*/

					/*{ { 401,	385 },
					{ 266,	392 },
					{ 140,	554 },
					{ 285,	597 },
					{ 400,	418 } };*/
					Point inhullR[] =
					{ { 1498,	994 },
					{ 1136,	628 },
					{ 1418,	620 },
					{ 1910,	852 },
					{ 1800,	998 } };

					/*{ { 873,	400 },
					{ 908,	397 },
					{ 1005,	398 },
					{ 1120,	526 },
					{ 1129,	568 },
					{ 985,	605 } };*/
					const int hullSize = sizeof(inhullL) / sizeof(inhullL[0]);
					//const int hullRSize = sizeof(inhullR) / sizeof(inhullR[0]);
					Point hullL[hullSize];
					//Point hullR[hullRSize];
					int hn, hnR;
					//convex_hull(inhullL, hullSize, hullL);
					buildConvexHull(inhullL, hullSize, hullL, hn);
					isInsideConvexHull(hullL, hn, hull_mask, &rct);
					////convex_hull(inhullL, hullSize, hullL);
					//buildConvexHull(inhullR, hullSize, hullR, hnR);
					//isInsideConvexHull(hullR, hnR, hull_mask, &rctR);
#ifdef OPENCV_OUT
					IplImage * hull_mask_show;
					hull_mask_show = cvCreateImage(cvSize(i_Width_u32, i_Height_u32), IPL_DEPTH_8U, 1);
					memset(hull_mask_show->imageData, 0, c_ImageSize_u32);
					for (int x = rct.x; x<rct.x + rct.width; x++)
						for (int y = rct.y; y < rct.y + rct.height; y++)
						{

							if (hull_mask.data_px[x + y * c_ImageWidth_u32])
							{

								hull_mask_show->imageData[x + y * c_ImageWidth_u32] = 255;

							}
						}
					/*for (int x = rctR.x; x<rctR.x + rctR.width; x++)
						for (int y = rctR.y; y < rctR.y + rctR.height; y++)
						{

							if (hull_mask.data_px[x + y * c_ImageWidth_u32])
							{

								hull_mask_show->imageData[x + y * c_ImageWidth_u32] = 255;

							}
					}*/
					//cv::Mat fisheye_mask_show(i_Height_u32, i_Width_u32, CV_8UC1, fisheye_mask);


					cvShowImage("hull_mask_show", hull_mask_show);
					cvWaitKey(0);


#endif
					memcpy(Mask_output, hull_mask.data_px, c_ImageSize_u32);
#else
					//CamNum = 1;
//#ifdef Left
#if 1
					mecl::Recti roi;
					

					static const uint32_t Template_sz = 23;
					////Left left
#ifdef READ_EXTERNAL
					//cv::Mat Template_Lt_TL = cv::imread(".\\Images\\Flatten\\Left\\Lt_TL.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					//cv::Mat Template_Lt_TR = cv::imread(".\\Images\\Flatten\\Left\\Lt_TR.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					//cv::Mat Template_Lt_BR = cv::imread(".\\Images\\Flatten\\Left\\Lt_BR.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					//cv::Mat Template_Lt_BL = cv::imread(".\\Images\\Flatten\\Left\\Lt_BL.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#endif					

					uint8_t imgTemp_Lt[4][Template_sz * Template_sz];



					memcpy(imgTemp_Lt[0], Lt_TL, Template_sz * Template_sz);
					memcpy(imgTemp_Lt[1], Lt_TR, Template_sz * Template_sz);
					memcpy(imgTemp_Lt[2], Lt_BR, Template_sz * Template_sz);
					memcpy(imgTemp_Lt[3], Lt_BL, Template_sz * Template_sz);

					mecl::core::Point2D<uint32_t> Corners_Pts[4], Corners_PtsT[4];
					VanPoints::Point2D_s Corners_cls[4];
					uint8_t CornerMatchingUseImg[c_ImageSize_u32];
					memcpy(CornerMatchingUseImg, v_GrayImage_o.data_px, c_ImageSize_u32);
					for (int i = 0; i < 4; i++)
					{
						uint8_t imgTemp_Ft_L_Single[Template_sz * Template_sz];
						memcpy(imgTemp_Ft_L_Single, imgTemp_Lt[i], 529);
						//Corners_PtsT[i] = CornerMatch(cameraID_e, &v_GrayImage_o.data_px[0], imgTemp_Ft_L[i], i, &Corners_Pts[i]);
#ifdef SHOW_CORNER_RECT
						Corners_PtsT[i] = CornerMatch(cameraID_e, &CornerMatchingUseImg[0], &imgTemp_Ft_L_Single[0], i, &Corners_Pts[i], c3);
#else
						Corners_PtsT[i] = CornerMatch(cameraID_e, &CornerMatchingUseImg[0], &imgTemp_Ft_L_Single[0], i, &Corners_Pts[i]);
#endif
						//Corners_PtsT[i] = CornerMatch(cameraID_e, &CornerMatchingUseImg[0], &imgTemp_Ft_L_Single[0], i, &Corners_Pts[i]);
						printf("i %d pts x %d y %d\n", i, Corners_Pts[i](0), Corners_Pts[i](1));
						printf("i %d Corners_PtsT pts x %d y %d\n", i, Corners_PtsT[i](0), Corners_PtsT[i](1));
						Corners_cls[i].x_i32 = Corners_PtsT[i](0);
						Corners_cls[i].y_i32 = Corners_PtsT[i](1);
					}

					Rough_ROI(Mask_output, cameraID_e, Corners_cls);
#ifdef FORD_QA
					cv::Mat Mask2(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
					for (int row = 0; row<vpcs::c_ImageHeight_u32; row++)
					{
						for (int col = 0; col<vpcs::c_ImageWidth_u32; col++)

						{
							Mask2.at<uint8_t>(row, col) = Mask_output[col + row * vpcs::c_ImageWidth_u32] * 255;

						}
					}
					imshow("mask2", Mask2);
					cv::waitKey(0);
#endif
					
#endif
#endif
				}

				if (cameraID_e == e_RearCamAlgo)
				{
#ifdef HULLMASK
					bool_t v_hull_mask[c_ImageSize_u32];
					memset(v_hull_mask, 0, c_ImageSize_u32);
					imtp::Image<bool_t> hull_mask(vpcs::c_ImageWidth_u32, vpcs::c_ImageHeight_u32, &v_hull_mask[0], 1);
					mecl::Rect rct, rctR;

					Point inhullL[] =
					{ {26,	1084},
					{444,	730 },
					{748,	748 },
					{426,	1256 },
					{160,	1244} };

					/*{ { 1,	948 },
					{ 576,	626 },
					{ 832,	626 },
					{ 410,	1100 },
					{ 160,	1080 } };*/

					/*{ { 401,	385 },
					{ 266,	392 },
					{ 140,	554 },
					{ 285,	597 },
					{ 400,	418 } };*/
					Point inhullR[] =
					{ {1212,	770},
					{1524,	770 },
					{1902,	1066 },
					{1764,	1236 },
					{1520,	1252} };

					/*{ { 1498,	994 },
					{ 1136,	628 },
					{ 1418,	620 },
					{ 1910,	852 },
					{ 1800,	998 } };*/

					/*{ { 873,	400 },
					{ 908,	397 },
					{ 1005,	398 },
					{ 1120,	526 },
					{ 1129,	568 },
					{ 985,	605 } };*/
					const int hullSize = sizeof(inhullL) / sizeof(inhullL[0]);
					const int hullRSize = sizeof(inhullR) / sizeof(inhullR[0]);
					Point hullL[hullSize];
					Point hullR[hullRSize];
					int hn, hnR;
					//convex_hull(inhullL, hullSize, hullL);
					buildConvexHull(inhullL, hullSize, hullL, hn);
					isInsideConvexHull(hullL, hn, hull_mask, &rct);
					//convex_hull(inhullL, hullSize, hullL);
					buildConvexHull(inhullR, hullSize, hullR, hnR);
					isInsideConvexHull(hullR, hnR, hull_mask, &rctR);
#ifdef OPENCV_OUT
					IplImage * hull_mask_show;
					hull_mask_show = cvCreateImage(cvSize(i_Width_u32, i_Height_u32), IPL_DEPTH_8U, 1);
					memset(hull_mask_show->imageData, 0, c_ImageSize_u32);
					for (int x = rct.x; x<rct.x + rct.width; x++)
						for (int y = rct.y; y < rct.y + rct.height; y++)
						{

							if (hull_mask.data_px[x + y * c_ImageWidth_u32])
							{

								hull_mask_show->imageData[x + y * c_ImageWidth_u32] = 255;

							}
						}
					for (int x = rctR.x; x<rctR.x + rctR.width; x++)
						for (int y = rctR.y; y < rctR.y + rctR.height; y++)
						{

							if (hull_mask.data_px[x + y * c_ImageWidth_u32])
							{

								hull_mask_show->imageData[x + y * c_ImageWidth_u32] = 255;

							}
						}
					//cv::Mat fisheye_mask_show(i_Height_u32, i_Width_u32, CV_8UC1, fisheye_mask);


					cvShowImage("hull_mask_show", hull_mask_show);
					cvWaitKey(0);


#endif
					memcpy(Mask_output, hull_mask.data_px, c_ImageSize_u32);
#else
					//CamNum = 2;
#if 1	

					mecl::Recti roi;

					static const uint32_t Template_sz = 23;
					//Rear left
#ifdef READ_EXTERNAL
					//cv::Mat Template_Ft_LTL = cv::imread(".\\Images\\Flatten\\Rear\\Rr_LTL.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					//cv::Mat Template_Ft_LTR = cv::imread(".\\Images\\Flatten\\Rear\\Rr_LTR.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					//cv::Mat Template_Ft_LBL = cv::imread(".\\Images\\Flatten\\Rear\\Rr_LBL.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					//cv::Mat Template_Ft_LBR = cv::imread(".\\Images\\Flatten\\Rear\\Rr_LBR.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					////Front Right
					//cv::Mat Template_Ft_RTL = cv::imread(".\\Images\\Flatten\\Rear\\Rr_RTL.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					//cv::Mat Template_Ft_RTR = cv::imread(".\\Images\\Flatten\\Rear\\Rr_RTR.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					//cv::Mat Template_Ft_RBL = cv::imread(".\\Images\\Flatten\\Rear\\Rr_RBL.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					//cv::Mat Template_Ft_RBR = cv::imread(".\\Images\\Flatten\\Rear\\Rr_RBR.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#endif
					

					uint8_t imgTemp_Ft_L[8][Template_sz * Template_sz];

					memcpy(imgTemp_Ft_L[0], Rr_LTL, Template_sz * Template_sz);
					memcpy(imgTemp_Ft_L[1], Rr_LTR, Template_sz * Template_sz);
					memcpy(imgTemp_Ft_L[2], Rr_LBR, Template_sz * Template_sz);
					memcpy(imgTemp_Ft_L[3], Rr_LBL, Template_sz * Template_sz);

					memcpy(imgTemp_Ft_L[4], Rr_RTL, Template_sz * Template_sz);
					memcpy(imgTemp_Ft_L[5], Rr_RTR, Template_sz * Template_sz);
					memcpy(imgTemp_Ft_L[6], Rr_RBR, Template_sz * Template_sz);
					memcpy(imgTemp_Ft_L[7], Rr_RBL, Template_sz * Template_sz);

					mecl::core::Point2D<uint32_t> Corners_Pts[8], Corners_PtsT[8];
					VanPoints::Point2D_s Corners_cls[8];
					uint8_t CornerMatchingUseImg[c_ImageSize_u32];
					memcpy(CornerMatchingUseImg, v_GrayImage_o.data_px, c_ImageSize_u32);
					for (int i = 0; i < 8; i++)
					{
						uint8_t imgTemp_Ft_L_Single[Template_sz * Template_sz];
						memcpy(imgTemp_Ft_L_Single, imgTemp_Ft_L[i], 529);
						//Corners_PtsT[i] = CornerMatch(cameraID_e, &v_GrayImage_o.data_px[0], imgTemp_Ft_L[i], i, &Corners_Pts[i]);
#ifdef SHOW_CORNER_RECT
						Corners_PtsT[i] = CornerMatch(cameraID_e, &CornerMatchingUseImg[0], &imgTemp_Ft_L_Single[0], i, &Corners_Pts[i], c3);
#else
						Corners_PtsT[i] = CornerMatch(cameraID_e, &CornerMatchingUseImg[0], &imgTemp_Ft_L_Single[0], i, &Corners_Pts[i]);
#endif
						//Corners_PtsT[i] = CornerMatch(cameraID_e, &CornerMatchingUseImg[0], &imgTemp_Ft_L_Single[0], i, &Corners_Pts[i]);
						printf("i %d pts x %d y %d\n", i, Corners_Pts[i](0), Corners_Pts[i](1));
						printf("i %d Corners_PtsT pts x %d y %d\n", i, Corners_PtsT[i](0), Corners_PtsT[i](1));
						Corners_cls[i].x_i32 = Corners_PtsT[i](0);
						Corners_cls[i].y_i32 = Corners_PtsT[i](1);
					}
#endif
					Rough_ROI(Mask_output, cameraID_e, Corners_cls);
#ifdef FORD_QA
					cv::Mat Mask3(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
					for (int row = 0; row<800; row++)
					{
						for (int col = 0; col<1280; col++)

						{
							Mask3.at<uint8_t>(row, col) = Mask_output[col + row * vpcs::c_ImageWidth_u32] * 255;

						}
					}
					imshow("Maks3", Mask3);
					cv::waitKey(0);

#endif
#endif
				}
				if (cameraID_e == e_RightCamAlgo)
				{
#ifdef HULLMASK
					bool_t v_hull_mask[c_ImageSize_u32];
					memset(v_hull_mask, 0, c_ImageSize_u32);
					imtp::Image<bool_t> hull_mask(vpcs::c_ImageWidth_u32, vpcs::c_ImageHeight_u32, &v_hull_mask[0], 1);
					mecl::Rect rct, rctR;

					Point inhullL[] =
					{ {404,	400},
					{1676,	400 },
					{1906,	1072 },
					{1552,	1264 },
					{430,	840} };

					/*{ {478, 100},
					{ 54,	820 },
					{ 310,	1272 },
					{ 1524,	592 },
					{ 1230,	326 } };*/

					/*{ { 478,	100 },
					{ 54,	820 },
					{ 310,	1272 },
					{ 1524,	592 },
					{ 1230,	326 } };*/

					/*{ { 1,	948 },
					{ 576,	626 },
					{ 832,	626 },
					{ 410,	1100 },
					{ 160,	1080 } };*/

					/*{ { 401,	385 },
					{ 266,	392 },
					{ 140,	554 },
					{ 285,	597 },
					{ 400,	418 } };*/
					Point inhullR[] =
					{ { 1498,	994 },
					{ 1136,	628 },
					{ 1418,	620 },
					{ 1910,	852 },
					{ 1800,	998 } };

					/*{ { 873,	400 },
					{ 908,	397 },
					{ 1005,	398 },
					{ 1120,	526 },
					{ 1129,	568 },
					{ 985,	605 } };*/
					const int hullSize = sizeof(inhullL) / sizeof(inhullL[0]);
					//const int hullRSize = sizeof(inhullR) / sizeof(inhullR[0]);
					Point hullL[hullSize];
					//Point hullR[hullRSize];
					int hn, hnR;
					//convex_hull(inhullL, hullSize, hullL);
					buildConvexHull(inhullL, hullSize, hullL, hn);
					isInsideConvexHull(hullL, hn, hull_mask, &rct);
					////convex_hull(inhullL, hullSize, hullL);
					//buildConvexHull(inhullR, hullSize, hullR, hnR);
					//isInsideConvexHull(hullR, hnR, hull_mask, &rctR);
#ifdef OPENCV_OUT
					IplImage * hull_mask_show;
					hull_mask_show = cvCreateImage(cvSize(i_Width_u32, i_Height_u32), IPL_DEPTH_8U, 1);
					memset(hull_mask_show->imageData, 0, c_ImageSize_u32);
					for (int x = rct.x; x<rct.x + rct.width; x++)
						for (int y = rct.y; y < rct.y + rct.height; y++)
						{

							if (hull_mask.data_px[x + y * c_ImageWidth_u32])
							{

								hull_mask_show->imageData[x + y * c_ImageWidth_u32] = 255;

							}
						}
					/*for (int x = rctR.x; x<rctR.x + rctR.width; x++)
					for (int y = rctR.y; y < rctR.y + rctR.height; y++)
					{

					if (hull_mask.data_px[x + y * c_ImageWidth_u32])
					{

					hull_mask_show->imageData[x + y * c_ImageWidth_u32] = 255;

					}
					}*/
					//cv::Mat fisheye_mask_show(i_Height_u32, i_Width_u32, CV_8UC1, fisheye_mask);


					cvShowImage("hull_mask_show", hull_mask_show);
					cvWaitKey(0);


#endif
					memcpy(Mask_output, hull_mask.data_px, c_ImageSize_u32);
#else
					//CamNum = 3;

//#ifdef Right
#if 1
					mecl::Recti roi;


					static const uint32_t Template_sz = 23;
#ifdef READ_EXTERNAL
					////Left left
					//cv::Mat Template_Lt_TL = cv::imread(".\\Images\\Flatten\\Right\\Rt_TL.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					//cv::Mat Template_Lt_TR = cv::imread(".\\Images\\Flatten\\Right\\Rt_TR.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					//cv::Mat Template_Lt_BR = cv::imread(".\\Images\\Flatten\\Right\\Rt_BR.bmp", CV_LOAD_IMAGE_GRAYSCALE);
					//cv::Mat Template_Lt_BL = cv::imread(".\\Images\\Flatten\\Right\\Rt_BL.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#endif



					//front left left

					uint8_t imgTemp_Lt[4][Template_sz * Template_sz];

					memcpy(imgTemp_Lt[0], Rt_TL, Template_sz * Template_sz);
					memcpy(imgTemp_Lt[1], Rt_TR, Template_sz * Template_sz);
					memcpy(imgTemp_Lt[2], Rt_BR, Template_sz * Template_sz);
					memcpy(imgTemp_Lt[3], Rt_BL, Template_sz * Template_sz);

					mecl::core::Point2D<uint32_t> Corners_Pts[4], Corners_PtsT[4];
					VanPoints::Point2D_s Corners_cls[4];
					uint8_t CornerMatchingUseImg[c_ImageSize_u32];
					memcpy(CornerMatchingUseImg, v_GrayImage_o.data_px, c_ImageSize_u32);
					for (int i = 0; i < 4; i++)
					{
						uint8_t imgTemp_Ft_L_Single[Template_sz * Template_sz];
						memcpy(imgTemp_Ft_L_Single, imgTemp_Lt[i], 529);
						//Corners_PtsT[i] = CornerMatch(cameraID_e, &v_GrayImage_o.data_px[0], imgTemp_Ft_L[i], i, &Corners_Pts[i]);
#ifdef SHOW_CORNER_RECT
						Corners_PtsT[i] = CornerMatch(cameraID_e, &CornerMatchingUseImg[0], &imgTemp_Ft_L_Single[0], i, &Corners_Pts[i], c3);
#else
						Corners_PtsT[i] = CornerMatch(cameraID_e, &CornerMatchingUseImg[0], &imgTemp_Ft_L_Single[0], i, &Corners_Pts[i]);
#endif
						//Corners_PtsT[i] = CornerMatch(cameraID_e, &CornerMatchingUseImg[0], &imgTemp_Ft_L_Single[0], i, &Corners_Pts[i]);
						printf("i %d pts x %d y %d\n", i, Corners_Pts[i](0), Corners_Pts[i](1));
						printf("i %d Corners_PtsT pts x %d y %d\n", i, Corners_PtsT[i](0), Corners_PtsT[i](1));
						Corners_cls[i].x_i32 = Corners_PtsT[i](0);
						Corners_cls[i].y_i32 = Corners_PtsT[i](1);
					}

					Rough_ROI(Mask_output, cameraID_e, Corners_cls);
#ifdef FORD_QA
					cv::Mat Mask4(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
					for (int row = 0; row<800; row++)
					{
						for (int col = 0; col<1280; col++)

						{
							Mask4.at<uint8_t>(row, col) = Mask_output[col + row * vpcs::c_ImageWidth_u32] * 255;

						}
					}
					imshow("Maks4", Mask4);
					cv::waitKey(0);

#endif

#endif
#endif
				}


			boolean_T Left_Mask_output[vpcs::c_ImageSize_u32];
			boolean_T Right_Mask_output[vpcs::c_ImageSize_u32];
//#ifdef DRAWCONTOUR
#if 1
			uint8_t _Contour[vpcs::c_ImageSize_u32];
			memset(_Contour, 0, c_ImageSize_u32);
			cv::Mat Im3;
			/*uint32_t ImRows = vpcs::c_ImageHeight_u32;
			uint32_t ImCols = vpcs::c_ImageWidth_u32;*/
			imtp::Image<uint8_t> v_ContourImage_o(c_ImageWidth_u32, c_ImageHeight_u32, &_Contour[0], 1);
			memset(v_ContourImage_o.data_px, 0, c_ImageSize_u32);
			cv::Mat Im0(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
			cv::cvtColor(Im0, Im3, CV_GRAY2BGR);
#endif
			for (int row = 0; row < vpcs::c_ImageHeight_u32; row++)
			{
				for (int col = 0; col < vpcs::c_ImageWidth_u32; col++)

				{
					//if ((col > 1277) || (row > 797))Mask_output[col + row * 1280] = 0;
					if ((col + row * vpcs::c_ImageWidth_u32) % vpcs::c_ImageWidth_u32 < (vpcs::c_ImageWidth_u32>>1))
						Left_Mask_output[col + row * vpcs::c_ImageWidth_u32] = Mask_output[col + row * vpcs::c_ImageWidth_u32];
					else
						Left_Mask_output[col + row * vpcs::c_ImageWidth_u32] = 0;
					if ((col + row * vpcs::c_ImageWidth_u32) % vpcs::c_ImageWidth_u32 > (vpcs::c_ImageWidth_u32>>1))
						Right_Mask_output[col + row * vpcs::c_ImageWidth_u32] = Mask_output[col + row * vpcs::c_ImageWidth_u32];
					else
						Right_Mask_output[col + row * vpcs::c_ImageWidth_u32] = 0;


				}
			}
			
			//unsigned char output_image[c_ImageSize_u32];

#ifndef HIST2THRESHOLD
				int Hithreshold = 150;
				int Lothreshold = 80;
			// create a histogram for the pixel values
#else
#ifdef OPENCV_OUT	
			cv::Mat Binariz(vpcs::c_ImageHeight_u32, vpcs::c_ImageWidth_u32, CV_8UC1);
#endif	


		int threshold=0;
				 computeHistogram2Threshold(&threshold, &v_GrayImage_o.data_px[0], Mask_output);


			int LTthreshold;
			     computeHistogram2Threshold(&LTthreshold, &v_GrayImage_o.data_px[0], Left_Mask_output);
			int RTthreshold;
				  computeHistogram2Threshold(&RTthreshold, &v_GrayImage_o.data_px[0], Right_Mask_output);
			
		// TODO: Hithreshold set but not used. Needed?
		// int Hithreshold=0;// = ? (LTthreshold > RTthreshold) : LTthreshold,
			int Lothreshold = 0;
			if (LTthreshold > RTthreshold)
			{
			// Hithreshold = LTthreshold;
				Lothreshold = RTthreshold;
			}
			else
			{
				Lothreshold = LTthreshold;
			// Hithreshold = RTthreshold;

			}
			
				/*printf("threshold %d\n", threshold);
				printf("Hithreshold %d\n", Hithreshold);
				printf("Lothreshold %d\n", Lothreshold);*/
#endif


#ifdef OPENCV_OUT	
//#if 0
			cv::Mat Mask(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
			for (int row = 0; row<vpcs::c_ImageHeight_u32; row++)
			{
				for (int col = 0; col<vpcs::c_ImageWidth_u32; col++)

				{
					Mask.at<uint8_t>(row, col) = Mask_output[col + row * vpcs::c_ImageWidth_u32] * 255;

				}
			}
			char filename_0[250] = { ".\\Images\\Debug\\" };
			char tempFn_0[250];
			sprintf(tempFn_0, "Camera_%d_mask.png", cameraID_e);

			strcat(filename_0, tempFn_0);
			cv::imwrite(filename_0, Mask);
			/*cv::imshow("Mask",Mask);
			cv::waitKey(0);*/
			
			
			

#endif

			float mean = 0.0;
			int cnt = 0;
			for (int row = 0; row < c_ImageHeight_u32; row++)
			{
				for (int col = 0; col < c_ImageWidth_u32; col++)

				{					
					if(Mask_output[col + row * c_ImageWidth_u32])	
					{
						mean += (float)v_GrayImage_o.data_px[col + row * c_ImageWidth_u32];
						cnt = cnt + 1;
					}
					
				}
			}
			/*cvShowImage("mask_img", mask_img);
			cvWaitKey(0);*/
			mean = mean / (cnt);
			imtp::Canny v_RunCanny_o(v_GrayImage_o, mean*0.4, mean, v_EdgesImage_o, v_MagImage_o, v_DirImage_o);
			//imtp::Canny v_RunCanny_o(v_GrayImage_o, 80, 150 , v_EdgesImage_o, v_MagImage_o, v_DirImage_o);
			//imtp::Canny v_RunCanny_o(v_GrayImage_o, Lothreshold*0.48, Lothreshold*0.6, v_EdgesImage_o, v_MagImage_o, v_DirImage_o);

			

			
#ifdef Running_Canny_Filter
			for (int row = 0; row<800; row++)
			{
			for (int col = 0; col<1280; col++)

			{
				v_EdgesImage_o.data_px[col + row * 1280] = dst1.at<uint8_t>(row, col);
					
			}
			}
#endif
			

#ifdef DEBUG_LOG
			log_printf("Completed Canny Filter \r\n");
#endif


			// Remove boundary lines
			for (i = 0; i < i_Height_u32; ++i)
			{
#ifdef OPENCV

				dst->data.ptr[i*i_Width_u32] = 0;
				dst->data.ptr[i*i_Width_u32 + 1] = 0;
				dst->data.ptr[i*i_Width_u32 + i_Width_u32 - 2] = 0;
				dst->data.ptr[i*i_Width_u32 + i_Width_u32 - 1] = 0;
#else
				v_EdgesImage_o.data_px[i * i_Width_u32] = 0;
				v_EdgesImage_o.data_px[i * i_Width_u32 + 1] = 0;
				v_EdgesImage_o.data_px[i * i_Width_u32 + i_Width_u32 - 2] = 0;
				v_EdgesImage_o.data_px[i * i_Width_u32 + i_Width_u32 - 1] = 0;
#endif
			}
#ifdef PRINT_DEBUG
			uint16_t Mask_nonzero_CNT = 0;
#endif
#ifdef OPENCV_OUT
			cv::Mat MaskOUT1(vpcs::c_ImageHeight_u32, vpcs::c_ImageWidth_u32, CV_8UC1);
#endif
			for (i = 0; i < i_Height_u32; ++i)
			{
				for (j = 0; j < i_Width_u32; ++j)
				{
					mecl::Point2i v_Pt_o;
					v_Pt_o.x_x = j;
					v_Pt_o.y_x = i;
					////if ((myMask.at(i, j) == 0))
					//	if (Mask.at<uint8_t>(i,j) == maskTH)
					////if (myMask[i * i_Width_u32 + j]==0)
					//if (Mask.at<uint8_t>(i, j) == 0)
					if (Mask_output[j + i * vpcs::c_ImageWidth_u32] == 0)
					{
						v_EdgesImage_o.data_px[i * i_Width_u32 + j] = 0;
					}
#ifdef PRINT_DEBUG
					else
					{
						Mask_nonzero_CNT++;
#ifdef OPENCV_OUT
						MaskOUT1.at<uint8_t>(i, j) = 255;
#endif
					}
#endif

				}
			} //end of ROI i j
#ifdef OPENCV_OUT
			imshow("MaskOUT1", MaskOUT1);
			cv::waitKey(0);
#endif			
#ifdef PRINT_DEBUG
			vm_cprintf("cameraID_e %d Mask_nonzero_CNT %d \n", cameraID_e, Mask_nonzero_CNT);			
#endif
			for (j = 0; j < i_Width_u32; ++j)
			{
#ifdef OPENCV
				dst->data.ptr[j] = 0;
				dst->data.ptr[i_Width_u32 + j] = 0;
				dst->data.ptr[(i_Height_u32 - 2)*i_Width_u32 + j] = 0;
				dst->data.ptr[(i_Height_u32 - 1)*i_Width_u32 + j] = 0;
				if (cameraID_e == vpcs::e_FrontCamAlgo || cameraID_e == e_RearCamAlgo)
				{
					if ((j < (640 + 100)) && (j >(640 - 100)))
					{
						for (k = 0; k < i_Height_u32 - 1; k++)
						{
							dst->data.ptr[k*i_Width_u32 + j] = 0;
						}
					}
				}
#else
				v_EdgesImage_o.data_px[j] = 0;
				v_EdgesImage_o.data_px[i_Width_u32 + j] = 0;
				v_EdgesImage_o.data_px[(i_Height_u32 - 2) * i_Width_u32 + j] = 0;
				v_EdgesImage_o.data_px[(i_Height_u32 - 1) * i_Width_u32 + j] = 0;
#endif
			}
#ifdef OPENCV_OUT
			cv::Mat MaskOUT(vpcs::c_ImageHeight_u32, vpcs::c_ImageWidth_u32, CV_8UC1);
#endif
			sint32_t binNum = 8; // Bin size for edge pixel orientation

			for (i = 0; i < (i_Height_u32 - 1); ++i)
			{
				for (j = 0; j < (i_Width_u32 - 1); ++j)
				{
#ifdef OPENCV
					if (*((unsigned char *)CV_MAT_ELEM_PTR(*dst, i, j)) == 255) // if is edge pixel
#else
					if ((*v_EdgesImage_o.at(j, i) != 0) && (Mask_output[j + i * vpcs::c_ImageWidth_u32] == 1))
#endif
					{
#ifdef OPENCV_OUT
						MaskOUT.at<uint8_t>(i, j) = 255;
#endif
						float32_t v_SobelGradientX_f32;
						float32_t v_SobelGradientY_f32;
						Smooth_sobelReplacement(j, i, &v_TempNcv_af32[0], v_SobelGradientX_f32, v_SobelGradientY_f32);
						if (v_SobelGradientY_f32 == 0)
							printf("x %f , y %f\n", v_SobelGradientX_f32, v_SobelGradientY_f32);
						const float32_t c_Anger_f32 = static_cast<float32_t>(atan(
							v_SobelGradientX_f32 / (v_SobelGradientY_f32 + 1e-10))); // gradient orientation
							const sint32_t c_Index_s32 = static_cast<sint32_t>((c_Anger_f32 + static_cast<float32_t>(M_PI) / 2.0F)
							* static_cast<float32_t>(binNum)
							/ static_cast<float32_t>(M_PI)
							+ 1.0F); // bin c_Index_s32
							
						v_TempNcv_af32[i * c_ImageWidth_u32 + j] = static_cast<float32_t>(c_Index_s32); // store the bin c_Index_s32 to temp
					}
					else
					{
						v_TempNcv_af32[i * c_ImageWidth_u32 + j] = 0; // set the bin index for non-edge pixel to zero
					}
				}
			}
#ifdef OPENCV_OUT
			imshow("MaskOUT_check after use", MaskOUT);
			cv::waitKey(0);
			cv::Mat _Dirm1(c_ImageHeight_u32, c_ImageWidth_u32, CV_8UC1);
#endif
			imtp::Image<uint8_t> v_UsedImage_o(c_ImageWidth_u32, c_ImageHeight_u32, &v_Edges_au8[0], 1); // don't need edges buffer anymore so reuse it for used Image
			float32_t v_D_af32[4]; //originally 2*2 float32_t matrix
			float32_t v_Evects_af32[4]; //originally 2*2 float32_t matrix
			float32_t v_Evals_af32[2]; //originally 2*1 matrix
#ifdef DEBUG_LOG
			log_printf("Running Find Contours \r\n");
#endif
			//float32_t H_line_Angle = 0.349066;
			memset(v_UsedImage_o.data_px, 0, c_ImageSize_u32);
			for (sint32_t bin = 1; bin <= binNum; ++bin)
			{
				memset(v_DirImage_o.data_px, 0, c_ImageSize_u32);  // set the direction image to zero
				for (i = 0; i < i_Height_u32; ++i)
				{
					for (j = 0; j < i_Width_u32; ++j)
					{
						sint32_t index = static_cast<sint32_t>(v_TempNcv_af32[j + i * c_ImageWidth_u32]); // bin index
						
						if ((*v_UsedImage_o.at(j, i) == 0) && (Mask_output[j + i * vpcs::c_ImageWidth_u32] == 1)) // if it is not used
						{
							// cross bin usage
							if ((index == bin) || (index == ((bin == 1) ? binNum : (bin - 1)))
								|| (index == ((bin == binNum) ? 1 : (bin + 1))))
							{
								*v_DirImage_o.at(j, i) = 255;
								//printf("(i %d, j %d), index %d bin %d binNum %d\n", i, j, index, bin, binNum);
							}
						}
					}
				}

				//Fill single pixel Gaps in Canny image
				for (i = 1; i < (c_ImageWidth_u32 - 1); i++)
				{
					for (j = 1; j < (c_ImageHeight_u32 - 1); j++)
					{
						if ((v_DirImage_o.data_px[j * i_Width_u32 + i] == 0) && (Mask_output[j + i * vpcs::c_ImageWidth_u32] == 1))
						{
							const uint8_t c_P00_u8 = v_DirImage_o.data_px[(j - 1) * i_Width_u32 + (i - 1)];
							const uint8_t c_P01_u8 = v_DirImage_o.data_px[(j - 1) * i_Width_u32 + (i)];
							const uint8_t c_P02_u8 = v_DirImage_o.data_px[(j - 1) * i_Width_u32 + (i + 1)];
							const uint8_t c_P10_u8 = v_DirImage_o.data_px[(j)* i_Width_u32 + (i - 1)];
							uint8_t *v_P11_pu8 = &v_DirImage_o.data_px[(j)* i_Width_u32 + (i)];
							const uint8_t c_P12_u8 = v_DirImage_o.data_px[(j)* i_Width_u32 + (i + 1)];
							const uint8_t c_P20_u8 = v_DirImage_o.data_px[(j + 1) * i_Width_u32 + (i - 1)];
							const uint8_t c_P21_u8 = v_DirImage_o.data_px[(j + 1) * i_Width_u32 + (i)];
							const uint8_t c_P22_u8 = v_DirImage_o.data_px[(j + 1) * i_Width_u32 + (i + 1)];

							// PRQA S 3054 4 // we are aware that conversion from uint8_t to bool_t takes place here.
							if ((c_P00_u8 && ((c_P12_u8 && !c_P01_u8) || (c_P22_u8 && !((c_P01_u8 && c_P12_u8) || (c_P10_u8 && c_P21_u8))) || (c_P21_u8 && !c_P10_u8)))
								|| (c_P01_u8 && ((c_P20_u8 && !c_P10_u8) || (c_P21_u8 && !(c_P10_u8 || c_P12_u8)) || (c_P22_u8 && !c_P12_u8)))
								|| (c_P02_u8 && ((c_P10_u8 && !c_P01_u8) || (c_P20_u8 && !((c_P01_u8 && c_P10_u8) || (c_P12_u8 && c_P21_u8))) || (c_P21_u8 && !c_P12_u8)))
								|| (c_P10_u8 && (c_P12_u8 && !(c_P21_u8 || c_P01_u8))))
							{
								*v_P11_pu8 = 1;
							}
						}
					}
				}

#ifdef OPENCV_OUT
				
				
				for (int row = 0; row<vpcs::c_ImageHeight_u32; row++)
				{
					for (int col = 0; col<vpcs::c_ImageWidth_u32; col++)

					{
						
						_Dirm1.at<uint8_t>(row, col) = v_DirImage_o.data_px[col + row * vpcs::c_ImageWidth_u32] * 255;

					}
				}
				
				/*cv::imshow("_Dirm1", _Dirm1);
				cv::waitKey(0);
				
				cv::imwrite("_Dirm1.bmp", _Dirm1);*/

#endif

#ifdef OPENCV

				IplImage * binImage;
				binImage = cvCreateImage(cvSize(i_Width_u32, i_Height_u32), IPL_DEPTH_8U, 1);
				memcpy(binImage->imageData, v_DirImage_o.data_px, c_ImageWidth_u32 * c_ImageHeight_u32);
				cvShowImage("lines", binImage);
				cvWaitKey(0);
#endif

				cvRep::FindContours v_ContoursNcv_o(v_DirImage_o, cvRep::RETR_E_LIST, cvRep::CHAIN_E_APPROXnone,
					mecl::Point2i(0, 0)); // find contours
				sint32_t v_index = 0;

				for (i = 0; v_ContoursNcv_o.getNumPointsInContours_u32(i) > 0; i++) // consider each line support region
				{
					if (v_ContoursNcv_o.getNumPointsInContours_u32(i) > i_MinLen_u32) // if the region size is larger than threshold
					{
						sint32_t mean_x = 0;
						sint32_t mean_y = 0;
						sint32_t max_x = 0;
						sint32_t max_y = 0;
						sint32_t min_x = i_Width_u32;
						sint32_t min_y = i_Height_u32;
						for (k = 0; k < v_ContoursNcv_o.getNumPointsInContours_u32(i); k++)
						{
							const mecl::Point2i &c_P_rt = v_ContoursNcv_o.getPoint(k + v_index);
							mean_x += c_P_rt.x_x;
							mean_y += c_P_rt.y_x;
							if (c_P_rt.x_x > max_x)
							{
								max_x = c_P_rt.x_x;
							}
							if (c_P_rt.y_x > max_y)
							{
								max_y = c_P_rt.y_x;
							}
							if (c_P_rt.x_x < min_x)
							{
								min_x = c_P_rt.x_x;
							}
							if (c_P_rt.y_x < min_y)
							{
								min_y = c_P_rt.y_x;
							}
						}
						mean_x /= v_ContoursNcv_o.getNumPointsInContours_u32(i);
						mean_y /= v_ContoursNcv_o.getNumPointsInContours_u32(i);
						memset(v_D_af32, 0, sizeof(v_D_af32));
						for (k = 0; k < v_ContoursNcv_o.getNumPointsInContours_u32(i); k++)
						{
							const mecl::Point2i &c_P_rt = v_ContoursNcv_o.getPoint(k + v_index);
							v_D_af32[0] = v_D_af32[0] + static_cast<float32_t>((c_P_rt.x_x - mean_x) * (c_P_rt.x_x - mean_x));
							v_D_af32[1] = v_D_af32[1] + static_cast<float32_t>((c_P_rt.x_x - mean_x) * (c_P_rt.y_x - mean_y));
							v_D_af32[3] = v_D_af32[3] + static_cast<float32_t>((c_P_rt.y_x - mean_y) * (c_P_rt.y_x - mean_y));
							_Contour[c_P_rt.y_x + c_P_rt.x_x * vpcs::c_ImageWidth_u32] = 255;
							v_ContourImage_o.data_px[c_P_rt.y_x + c_P_rt.x_x * vpcs::c_ImageWidth_u32] = 255;
							//Im0.at<uint8_t>(c_P_rt.x_x, c_P_rt.y_x)= _Contour[c_P_rt.y_x + c_P_rt.x_x * 1280];
							cv::circle(Im3, cv::Point(c_P_rt.x_x, c_P_rt.y_x), 1, CV_RGB(255, 0, 0), 1);
							//cvCircle(&_Contour, cvPoint(c_P_rt.x_x, c_P_rt.y_x), 1, CV_RGB(255, 0, 0), 1);
						}
#ifdef OPENCV_OUT
						//imtp::Image<uint8_t> v_ContourImage_o(c_ImageWidth_u32, c_ImageHeight_u32, &_Contour[0], 1);
						cv::Mat ncv_img(vpcs::c_ImageHeight_u32, vpcs::c_ImageWidth_u32, CV_8UC1, &v_ContourImage_o);
						//imshow("contour_ncv_img", ncv_img);
						//cv::waitKey(0);
						//imshow("contour", Im3);
						//cv::waitKey(0);
#endif
						v_D_af32[2] = v_D_af32[1];
						cvRep::eigen2x2_flt(&v_D_af32[0], &v_Evals_af32[0], &v_Evects_af32[0]);
						const float32_t c_Theta_f32 = static_cast<float32_t>(mecl::math::trigonometry<float64_t>::atan2_x(
							static_cast<float64_t>(v_Evects_af32[1]), static_cast<float64_t>(v_Evects_af32[0])));
						float32_t v_Conf_f32;
						if (v_Evals_af32[1] > 0)
						{
 							v_Conf_f32 = v_Evals_af32[0] / v_Evals_af32[1];
						}
						else
						{
							v_Conf_f32 = 100000.0F;
						}

						if (v_Conf_f32 >= 400.0F)
						//	if (v_Conf_f32 >= 385.0F)
						//if (v_Conf_f32 >= 375.0F)
						//if (v_Conf_f32 >= 300.0F)//no EQ
						{
							const float32_t c_R_f32 = static_cast<float32_t>(mecl::math::algebra<float64_t>::sqrt_x(
								static_cast<float64_t>((max_x - min_x) * (max_x - min_x) + (max_y - min_y) * (max_y - min_y))));
							const float32_t c_X1_f32 = static_cast<float32_t>(mean_x) - cos(c_Theta_f32) * c_R_f32 / 2.0F;
							const float32_t c_X2_f32 = static_cast<float32_t>(mean_x) + cos(c_Theta_f32) * c_R_f32 / 2.0F;
							const float32_t c_Y1_f32 = static_cast<float32_t>(mean_y) - sin(c_Theta_f32) * c_R_f32 / 2.0F;
							const float32_t c_Y2_f32 = static_cast<float32_t>(mean_y) + sin(c_Theta_f32) * c_R_f32 / 2.0F;
							for (k = 0; k < v_ContoursNcv_o.getNumPointsInContours_u32(i); k++)
							{
								const mecl::Point2i &c_P_rt = v_ContoursNcv_o.getPoint(k + v_index);
								*v_UsedImage_o.at(c_P_rt.x_x, c_P_rt.y_x) = 1;
							}

							mecl::Point2i v_Linept_o;
							v_Linept_o.x_x = mean_x;
							v_Linept_o.y_x = mean_y;
							if(lineNum < LINES_SIZE_MAX)
							{
								o_Lines_ps[lineNum].x1_af32[0] = c_X1_f32;
								o_Lines_ps[lineNum].x1_af32[1] = c_Y1_f32;
								o_Lines_ps[lineNum].x2_af32[0] = c_X2_f32;
								o_Lines_ps[lineNum].x2_af32[1] = c_Y2_f32;
								
							//	if (((cameraID_e == e_FrontCamAlgo) || (cameraID_e == e_RearCamAlgo)) &&
							//		((fabs(c_Theta_f32) < 0.3496) && (c_Y1_f32 > 600) && (c_Y2_f32 > 600)))
							//	{
							//		continue;
							//	}
							//	else
							//	{															
									o_Lines_ps[lineNum].mean_af32[0] = static_cast<float32_t>(mean_x);
									o_Lines_ps[lineNum].mean_af32[1] = static_cast<float32_t>(mean_y);
									o_Lines_ps[lineNum].theta_f32 = c_Theta_f32;
									o_Lines_ps[lineNum].r_f32 = c_R_f32;
									o_Lines_ps[lineNum].l_af32[0] = c_Y1_f32 - c_Y2_f32;
									o_Lines_ps[lineNum].l_af32[1] = c_X2_f32 - c_X1_f32;
									o_Lines_ps[lineNum].l_af32[2] = c_X1_f32 * c_Y2_f32 - c_X2_f32 * c_Y1_f32;

									lineNum++;
							//	}
							}
							else
							{
								o_Lines_ps[lineNum].mean_af32[0] = 0;
								o_Lines_ps[lineNum].mean_af32[1] = 0;
								o_Lines_ps[lineNum].r_f32 = 0;
							}
						}
					} // end if (ptr->total > minLen)
					v_index += v_ContoursNcv_o.getNumPointsInContours_u32(i);
				} // end for ptr
#ifdef PRINT_DEBUG
				vm_cprintf("cameraID_e %d bin %d v_index %d \n", cameraID_e, bin, v_index);
#endif
			}


#ifdef DEBUG_LOG
			log_printf("Completed Find Contours \r\n");
#endif


#ifdef OPENCV
			cvReleaseImage(&grayImgU);
			cvReleaseImage(&grayImgF);
			cvReleaseMat(&dst);
#endif
			*o_Lnum_pu32 = lineNum;
		}

		return;
                  	}

	void VanPoints::findVanPoints(sint32_t i_ModelSize_s32,
		const Line_s* i_Edges_ps,
		sint32_t i_LineCount_s32,
		sint32_t* o_VpNum_ps32,
		VanPoints::Point_s* o_Output_ps,
		sint32_t i_LineLength_s32)
	{
#ifdef PIKEOS
		(void)i_LineLength_s32;
#endif
#ifdef DEBUG_LOG
		log_printf("Number of edges is %d\n", i_LineCount_s32);
#endif
		if (i_LineCount_s32 > 1)
		{
			MinimalSet_s v_RandomMS_s;
			// Select minimal sets
			v_RandomMS_s = selectMinimalSets(i_ModelSize_s32, i_Edges_ps, i_LineCount_s32, cameraID_e);
			// Construct Preference Set Matrix
			sint32_t PSMatrix[LINES_SIZE_MAX][c_ModelSize_u32];
			makePSMatrix(v_RandomMS_s, i_Edges_ps, i_LineCount_s32, &PSMatrix[0]);

			// Next 2 lines just to save the unclustered PSMatrix to an image
#ifdef OPENCV_OUT
			CvMat * v_Scaled_o = cvCreateMat(i_LineCount_s32, v_RandomMS_s.size, CV_8UC1);
			cvSetZero(v_Scaled_o);
			for (sint32_t i = 0; i < i_LineCount_s32; ++i)
			{
				for (sint32_t j = 0; j < v_RandomMS_s.size; ++j)
				{
					*((uint8_t*)CV_MAT_ELEM_PTR(*v_Scaled_o, i, j)) = PSMatrix[i][j] ? 255 : 0;
				}
			}
#if defined(WINDOWS_DEBUG) || defined(OPENCV_OUT)
			cvSaveImage("PSMatrix_raw.png", v_Scaled_o);
#endif
			cvReleaseMat(&v_Scaled_o);
#else
			mecl::core::Matrix<unsigned char, LINES_SIZE_MAX, c_ModelSize_u32> v_Scaled_o = mecl::core::Matrix<unsigned char,
				LINES_SIZE_MAX, c_ModelSize_u32>::zeros_x();

			for (sint32_t i = 0; i < i_LineCount_s32; ++i)
			{
				for (sint32_t j = 0; j < v_RandomMS_s.size; ++j)
				{
					v_Scaled_o(i, j) = (PSMatrix[i][j] >= 1) ? 255 : 0;
				}
			}
#endif
			//Perform clustering on PSMatrix
			sint32_t clusterNum = c_ClusterNum_u32;
			clusterPSMatrix(&PSMatrix[0], i_LineCount_s32, v_RandomMS_s.size, &clusterNum);

		for (sint32_t i = 0; i < clusterNum; ++i)
		{
			o_Output_ps[i] = estimateVanPoint(i_Edges_ps, i_LineCount_s32, &m_clusters[i][0]);
		}
		*o_VpNum_ps32 = clusterNum;
	}
	return;
}

	VanPoints::MinimalSet_s VanPoints::selectMinimalSets(sint32_t i_ModelSize_s32,
		const Line_s* i_Edges_ps,
		sint32_t i_LineCount_s32,
		E_CameraId_t cameraID_e)
	{
		srand(50); // Seed the rand function

		sint32_t modelsM = i_ModelSize_s32;
		MinimalSet_s v_CountedModels_s;
//#ifdef ONLY_VALID_INTERSECTIONS
		if ((cameraID_e == e_FrontCamAlgo) || (cameraID_e == e_RearCamAlgo))
		{
			sint32_t i = 0;
			while (i < modelsM)
			{
				// For each modelsM choose, randomly, two lines
				sint32_t n1 = rand() % i_LineCount_s32;
				sint32_t n2 = rand() % (i_LineCount_s32 - 1);
				if (n2 == n1)// if n2 == n1, they are both smaller than (lineCount-1), so n2 can be set to (lineCount-1) for n1!=n2
				{
					n2 = i_LineCount_s32 - 1;
				}
				// Find vanishing point (intersection) for chosen model
				

				{
					if ((mecl::math::abs_x(i_Edges_ps[n1].theta_f32) < H_line_AngleF)
						== (mecl::math::abs_x(i_Edges_ps[n2].theta_f32) < H_line_AngleF))
					{
						v_CountedModels_s.minSet_as[i].line1_s = i_Edges_ps[n1];
						v_CountedModels_s.minSet_as[i].line2_s = i_Edges_ps[n2];
						v_CountedModels_s.minSet_as[i].intersectionPt_s = findIntersection_s(v_CountedModels_s.minSet_as[i].line1_s, v_CountedModels_s.minSet_as[i].line2_s);
						i++;
					}
				}
			}
		}
		else
//#else
		{

		for (sint32_t i = 0; i < modelsM; i++)
		{
			// For each modelsM choose, randomly, two lines
			sint32_t n1 = rand() % i_LineCount_s32;
			sint32_t n2 = rand() % (i_LineCount_s32 - 1);
			if (n2 == n1) // if n2 == n1, they are both smaller than (lineCount-1), so n2 can be set to (lineCount-1) for n1!=n2
			{
				n2 = i_LineCount_s32 - 1;
			}
			v_CountedModels_s.minSet_as[i].line1_s = i_Edges_ps[n1];
			v_CountedModels_s.minSet_as[i].line2_s = i_Edges_ps[n2];
			// Find vanishing point (intersection) for chosen model

			v_CountedModels_s.minSet_as[i].intersectionPt_s = findIntersection_s(v_CountedModels_s.minSet_as[i].line1_s,
				v_CountedModels_s.minSet_as[i].line2_s);

		}
		}
//#endif

		v_CountedModels_s.size = modelsM;
		return v_CountedModels_s;
	}

	void VanPoints::makePSMatrix(const MinimalSet_s &i_RandomMS_rs,
		const Line_s* i_Edges_ps,
		sint32_t i_LineCount_s32,
		sint32_t(*PSMatrix)[c_ModelSize_u32]) const
	{
		float32_t v_Distance_f32;

		for (sint32_t i = 0; i < i_LineCount_s32; ++i)
		{
#ifdef DEBUG_LOG
			log_printf("\nEdge %d: %d (%d,%d) (%d,%d) (%d,%d)\n", i, static_cast<int>(i_Edges_ps[i].r_f32),
				static_cast<int>(i_Edges_ps[i].x1_af32[0]), static_cast<int>(i_Edges_ps[i].x1_af32[1]),
				static_cast<int>(i_Edges_ps[i].x2_af32[0]), static_cast<int>(i_Edges_ps[i].x2_af32[1]),
				static_cast<int>(i_Edges_ps[i].mean_af32[0]), static_cast<int>(i_Edges_ps[i].mean_af32[1]));
#endif
			for (sint32_t j = 0; j < i_RandomMS_rs.size; ++j)
			{
				// See proximity of ith line with jth model
				// Proximity in this case is perpendicular distance

				{
					v_Distance_f32 = findOrthDistance(i_Edges_ps[i], i_RandomMS_rs.minSet_as[j].intersectionPt_s);
					PSMatrix[i][j] = (v_Distance_f32 <= clusterThresh_f32) ? 1 : 0;
				}
#ifdef DEBUG_LOG
				log_printf("%d ", PSMatrix[i][j]);
#endif
			}
		}
		return;				// PSMatrix is output;
	}
	void VanPoints::clusterPSMatrix(sint32_t(*PSMatrix)[c_ModelSize_u32],
		sint32_t lineNum,
		sint32_t modelNum,
		const sint32_t* clusterNum)
	{
		/* allocate memory */
		float32_t distances[MAX_CLUSTER_SIZE][MAX_CLUSTER_SIZE];
		sint32_t pre_clusters[MAX_CLUSTER_SIZE][MAX_CLUSTER_SIZE];
		for (sint32_t i = 0; i < lineNum; ++i)
		{
			memset(distances[i], 0, sizeof(float32_t) * lineNum);
			memset(pre_clusters[i], 0, sizeof(sint32_t) * lineNum);
		}
		sint32_t indicators[MAX_CLUSTER_SIZE];

		/* initialization */
		for (sint32_t i = 0; i < lineNum; ++i)
		{
			pre_clusters[i][i] = 1;
		}

		float32_t v_MinDis_f32 = 1.0F;
		sint32_t indexA = 0;
		sint32_t indexB = 0;
		for (sint32_t i = 0; i < lineNum; ++i)
		{
			indicators[i] = 1;
			for (sint32_t j = i + 1; j < lineNum; ++j)
			{
				distances[i][j] = jaccardDist(&PSMatrix[i][0], &PSMatrix[j][0], modelNum);
				distances[j][i] = distances[i][j];
				if (distances[i][j] < v_MinDis_f32)
				{
					v_MinDis_f32 = distances[i][j];
					indexA = i;
					indexB = j;
				}
			}
		}
#ifdef OPENCV_OUT
		IplImage * clusterImg = cvCreateImage(cv::Size(lineNum, lineNum), IPL_DEPTH_8U, 1);
#endif
		// PRQA S 4234 1 // when no minimum is found, the loop will terminate
		while (v_MinDis_f32 < 1.0F)
		{
			/* merge two pre_clusters */
			for (sint32_t i = 0; i < lineNum; ++i)
			{
				if ((pre_clusters[indexA][i] == 1) || (pre_clusters[indexB][i] == 1))
				{
					pre_clusters[indexA][i] = 1;
					pre_clusters[indexB][i] = 1;
				}
			}
			indicators[indexB] = 0;
			for (sint32_t i = 0; i < modelNum; ++i)
			{
				if ((PSMatrix[indexA][i] != 1) || (PSMatrix[indexB][i] != 1))
				{
					PSMatrix[indexA][i] = 0;
					PSMatrix[indexB][i] = 0;
				}
			}

			/* recalculate distance */
			for (sint32_t i = 0; i < lineNum; ++i)
			{
				distances[indexA][i] = jaccardDist(&PSMatrix[indexA][0], &PSMatrix[i][0], modelNum);
				distances[i][indexA] = distances[indexA][i];
			}

			/* find minimum distance */
			v_MinDis_f32 = 1.0F;
			for (sint32_t i = 0; i < lineNum; ++i)
			{
				if (0 != indicators[i])
				{
					for (sint32_t j = i + 1; j < lineNum; ++j)
					{
						if ((0 != indicators[j]) && (distances[i][j] < v_MinDis_f32))
						{
							/* if we do not get here, the while loop will terminate */
							v_MinDis_f32 = distances[i][j];
							indexA = i;
							indexB = j;
						}
					}
				}
			}
#ifdef OPENCV_OUT
			clusterImg->imageData[indexA*lineNum + indexB] = 255;
#endif
		}
#ifdef OPENCV_OUT
		//cvSaveImage("clusterImg.bmp", clusterImg);
#endif


		/* calculate cluster size */
		sint32_t clusterSizes[MAX_CLUSTER_SIZE];
		for (sint32_t i = 0; i < lineNum; ++i)
		{
			clusterSizes[i] = 0;
			if (0 != indicators[i])
			{
				for (sint32_t j = 0; j < lineNum; ++j)
				{
					if (0 != pre_clusters[i][j])
					{
						clusterSizes[i]++;
					}
				}
			}
		}

		sint32_t count = 0;

		while (count < *clusterNum) /* choose the largest three pre_clusters */
		{
			sint32_t max_index = 0;
			sint32_t max_size = clusterSizes[0];
			for (sint32_t i = 1; i < lineNum; ++i)
			{
				if (max_size < clusterSizes[i])
				{
					max_size = clusterSizes[i];
					max_index = i;
				}
			}
			for (sint32_t i = 0; i < lineNum; ++i)
			{
				m_clusters[count][i] = pre_clusters[max_index][i];
			}
			count++;
			clusterSizes[max_index] = 0;
		}

#ifdef PRINT_DEBUG
		/* print pre_clusters */
		for (sint32_t i = 0; i < *clusterNum; ++i)
		{
			log_printf("Cluster %d: \r\n", i);
			for (sint32_t j = 0; j < lineNum; ++j)
			{
				if (pre_clusters[i][j])
				{
					vm_cprintf("%d ", j);
				}
			}
			vm_cprintf("\n");
		}
#endif

		return;
	}

	VanPoints::Point_s VanPoints::estimateVanPoint(const Line_s* i_Edges_ps,
		sint32_t i_LineNum_s32,
		const sint32_t* i_Cluster_ps32)
{
	sint32_t i;
	sint32_t num = 0;

		for (i = 0; i < i_LineNum_s32; ++i)
		{
			if (0 != i_Cluster_ps32[i])
			{
				num++;
			}
		}

#ifdef PRINT_DEBUG
		vm_cprintf("VanPoints::i_LineNum_s32 = %d\n", i_LineNum_s32);
		vm_cprintf("VanPoints::estimateVanPoint num = %d\n", num);
#endif

		float32_t v_A_af32[3 * MAX_CLUSTER_SIZE];
		sint32_t count = 0;
		for (i = 0; i < i_LineNum_s32; ++i)
		{
			if (0 != i_Cluster_ps32[i])
			{
				const float32_t c_L0_f32 = i_Edges_ps[i].l_af32[0];
				const float32_t c_L1_f32 = i_Edges_ps[i].l_af32[1];
				const float32_t c_L2_f32 = i_Edges_ps[i].l_af32[2];
				const float32_t c_Nrm_f32 = mecl::math::algebra<float32_t>::sqrt_x(
					c_L0_f32 * c_L0_f32 + c_L1_f32 * c_L1_f32 + c_L2_f32 * c_L2_f32);
				v_A_af32[3 * count] = c_L0_f32 / c_Nrm_f32;
				v_A_af32[3 * count + 1] = c_L1_f32 / c_Nrm_f32;
				v_A_af32[3 * count + 2] = c_L2_f32 / c_Nrm_f32;
				count++;
			}
		}

		float32_t v_V_af32[3 * 3];

		/*For some reason this outputs V transposed compared to cvSVD*/
		cvRep::SVDCompute_ncv(&v_A_af32[0], &v_V_af32[0], num, 3);

		Point_s v_Vp_s;
		v_Vp_s.x_f32 = v_V_af32[3 * 2];	 // V row:2 col:0
		v_Vp_s.y_f32 = v_V_af32[3 * 2 + 1]; // V row:2 col:1
		v_Vp_s.z_f32 = v_V_af32[3 * 2 + 2]; // V row:2 col:2

		return v_Vp_s;
	}

#ifdef OPENCV_OUT

	sint32_t VanPoints::superposeLines(IplImage * img, Line_s* lines, sint32_t lineNum, sint32_t(*o_clusters)[MAX_CLUSTER_SIZE], VanPoints::Point_s * vp, sint32_t lineLength)
	{
		IplImage *img1 = cvCloneImage(img);
		
		int line_CNT = 0;
		for (sint32_t i = 0; i < lineNum; ++i)
		{
			CvScalar color;

			if (o_clusters[0][i] == 1)
				color = CV_RGB(255, 0, 0);
			else if (o_clusters[1][i] == 1)
				color = CV_RGB(0, 255, 0);
			else if (o_clusters[2][i] == 1)
				color = CV_RGB(0, 0, 255);
			else continue;

			float32_t x1 = lines[i].x1_af32[0];
			float32_t y1 = lines[i].x1_af32[1];
			float32_t x2 = lines[i].x2_af32[0];
			float32_t y2 = lines[i].x2_af32[1];
			cvLine(img, cvPoint((sint32_t)x1, (sint32_t)y1), cvPoint((sint32_t)x2, (sint32_t)y2), color, 1);

			{

				
				cvLine(img, cvPoint((sint32_t)x1, (sint32_t)y1), cvPoint((sint32_t)x2, (sint32_t)y2), color, 1);
				
				cvCircle(img, cvPoint((vp[i].x_f32 / vp[i].z_f32), (vp[i].y_f32 / vp[i].z_f32)), 2, color, 2);
				cvCircle(img, cvPoint(lines[i].mean_af32[0], lines[i].mean_af32[1]), 2, CV_RGB(255, 0, 255), 2);
				char str[10];
				sprintf(str, "%d", i);
				CvFont font;
				cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
				cvPutText(img, str, cvPoint(lines[i].mean_af32[0], lines[i].mean_af32[1]), &font, 1);
				
				line_CNT++;
			}
	
		}
		printf("lineCNT %d\n", line_CNT);
		char filename_[250] = { ".\\Images\\Debug\\" };
		char tempFn_[250];
		sprintf(tempFn_, "Camera_%d_lines.png", cameraID_e);
		
		strcat(filename_, tempFn_);
		cvSaveImage(filename_, img);
 		//cvSaveImage(".\\Images\\Debug\\lines_extraction.png", img);
		/*cvShowImage("line_clusters", img);
		cvWaitKey(0);*/
#ifdef ShowLlinEmbed
	CvScalar color;
	color = CV_RGB(255, 0, 0);
	if (cameraID_e == e_FrontCamAlgo)
	{
		cvLine(img1, cvPoint(942.510254, 427.487976), cvPoint(969.489746, 450.512024), color, 1);
		cvLine(img1, cvPoint(1050.367065, 497.171692), cvPoint(1277.632935, 672.828308), color, 1);
		cvLine(img1, cvPoint(1048.274170, 516.774841), cvPoint(1073.725830, 537.225159), color, 1);
		cvLine(img1, cvPoint(1134.142822, 535.487854), cvPoint(1275.857178, 638.512146), color, 1);
		cvLine(img1, cvPoint(1162.824951, 610.706543), cvPoint(1185.175049, 629.293457), color, 1);
		cvLine(img1, cvPoint(901.179260, 363.754059), cvPoint(1142.820801, 540.245911), color, 1);
		cvLine(img1, cvPoint(885.992493, 366.009491), cvPoint(1088.007446, 525.990540), color, 1);
		cvLine(img1, cvPoint(868.965210, 365.541382), cvPoint(951.034790, 434.458618), color, 1);
		cvLine(img1, cvPoint(970.854309, 452.074188), cvPoint(1061.145752, 527.925781), color, 1);
		cvLine(img1, cvPoint(1069.336670, 533.698181), cvPoint(1162.663330, 610.301819), color, 1);
		cvLine(img1, cvPoint(1186.444458, 630.065979), cvPoint(1231.555542, 667.934021), color, 1);
		cvLine(img1, cvPoint(1016.007141, 364.485992), cvPoint(1273.992920, 495.514008), color, 1);
		cvLine(img1, cvPoint(1095.473999, 365.062134), cvPoint(1276.526001, 440.937866), color, 1);
		cvLine(img1, cvPoint(1120.875244, 367.305847), cvPoint(1277.124756, 430.694153), color, 1);
		cvLine(img1, cvPoint(1185.438232, 386.654358), cvPoint(1232.561768, 405.345642), color, 1);
		cvLine(img1, cvPoint(1084.837280, 523.209412), cvPoint(1275.162720, 670.790588), color, 1);
		cvLine(img1, cvPoint(1143.956421, 540.227539), cvPoint(1276.043579, 637.772461), color, 1);
		cvLine(img1, cvPoint(1254.425415, 686.583069), cvPoint(1275.574585, 705.416931), color, 1);
		cvLine(img1, cvPoint(0.614325, 432.550079), cvPoint(183.385681, 359.449921), color, 1);
		cvLine(img1, cvPoint(887.500000, 364.000000), cvPoint(898.500000, 364.000000), color, 1);
		cvLine(img1, cvPoint(1025.986328, 366.261169), cvPoint(1094.013672, 365.738831), color, 1);
		cvLine(img1, cvPoint(15.160169, 651.208435), cvPoint(420.839844, 342.791534), color, 1);
		cvLine(img1, cvPoint(0.168501, 421.645569), cvPoint(165.831497, 358.354431), color, 1);
		cvLine(img1, cvPoint(870.500000, 365.000000), cvPoint(881.500000, 365.000000), color, 1);
		cvLine(img1, cvPoint(250.189987, 491.738983), cvPoint(291.810028, 458.261017), color, 1);
		cvLine(img1, cvPoint(76.926498, 632.532654), cvPoint(177.073502, 551.467346), color, 1);
		cvLine(img1, cvPoint(11.834977, 685.796387), cvPoint(56.165024, 650.203613), color, 1);
		cvLine(img1, cvPoint(291.955719, 458.945953), cvPoint(414.044281, 359.054047), color, 1);
		cvLine(img1, cvPoint(175.964188, 553.456482), cvPoint(250.035812, 492.543488), color, 1);
	}
	if (cameraID_e == e_LeftCamAlgo)
	{
		cvLine(img1, cvPoint(801.918762, 111.512054), cvPoint(808.081238, 152.487946), color, 1);
		cvLine(img1, cvPoint(483.725586, 283.300385), cvPoint(548.274414, 356.699615), color, 1);
		cvLine(img1, cvPoint(755.968445, 214.508286), cvPoint(762.031555, 237.491714), color, 1);
		cvLine(img1, cvPoint(438.642517, 218.693146), cvPoint(479.357483, 267.306854), color, 1);
		cvLine(img1, cvPoint(353.059418, 231.949005), cvPoint(400.940582, 278.050995), color, 1);
		cvLine(img1, cvPoint(490.854309, 280.624725), cvPoint(519.145691, 313.375275), color, 1);
		cvLine(img1, cvPoint(213.011063, 305.044281), cvPoint(262.988922, 348.955719), color, 1);
		cvLine(img1, cvPoint(155.657928, 366.214661), cvPoint(190.342072, 385.785339), color, 1);
		cvLine(img1, cvPoint(570.470825, 233.824814), cvPoint(591.529175, 266.175171), color, 1);
		cvLine(img1, cvPoint(189.526047, 332.951965), cvPoint(226.473953, 353.048035), color, 1);
		cvLine(img1, cvPoint(148.668015, 368.686493), cvPoint(201.331985, 397.313507), color, 1);
		cvLine(img1, cvPoint(555.465027, 69.215981), cvPoint(618.534973, 68.784019), color, 1);
		cvLine(img1, cvPoint(422.938507, 160.836670), cvPoint(465.061493, 145.163330), color, 1);
		cvLine(img1, cvPoint(702.640991, 283.318481), cvPoint(739.359009, 266.681519), color, 1);
		cvLine(img1, cvPoint(570.959961, 344.651703), cvPoint(671.040039, 303.348297), color, 1);
		cvLine(img1, cvPoint(480.484924, 134.090485), cvPoint(535.515076, 87.909515), color, 1);
		cvLine(img1, cvPoint(632.133728, 190.723572), cvPoint(797.866272, 91.276436), color, 1);
		cvLine(img1, cvPoint(413.085754, 160.228516), cvPoint(450.914246, 145.771484), color, 1);
		cvLine(img1, cvPoint(574.129333, 232.721329), cvPoint(695.870667, 161.278671), color, 1);
		cvLine(img1, cvPoint(838.913208, 222.786377), cvPoint(863.086792, 213.213623), color, 1);
		cvLine(img1, cvPoint(668.985962, 302.967865), cvPoint(733.014038, 275.032135), color, 1);
		cvLine(img1, cvPoint(709.685608, 290.447296), cvPoint(744.314392, 275.552704), color, 1);
		cvLine(img1, cvPoint(269.953033, 387.433319), cvPoint(294.046967, 370.566681), color, 1);
		cvLine(img1, cvPoint(478.516571, 122.519821), cvPoint(539.483398, 71.480179), color, 1);
		cvLine(img1, cvPoint(492.724060, 279.515320), cvPoint(527.275940, 260.484680), color, 1);
		cvLine(img1, cvPoint(751.305054, 189.213852), cvPoint(810.694946, 158.786148), color, 1);
		cvLine(img1, cvPoint(682.000000, 237.500000), cvPoint(682.000000, 254.500000), color, 1);
		cvLine(img1, cvPoint(282.039398, 342.556061), cvPoint(311.960602, 321.443939), color, 1);
		cvLine(img1, cvPoint(566.017883, 227.028015), cvPoint(587.982117, 212.971985), color, 1);
	}
	if (cameraID_e == e_RearCamAlgo)
	{
		cvLine(img1, cvPoint(860.676270, 422.838257), cvPoint(1209.32373, 803.161743), color, 1);
		cvLine(img1, cvPoint(827.004761, 418.787323), cvPoint(1002.99523, 639.212646), color, 1);
		cvLine(img1, cvPoint(844.423462, 422.565643), cvPoint(1169.57653, 801.434326), color, 1);
		cvLine(img1, cvPoint(1028.346924, 410.033447), cvPoint(1277.65307, 555.966553), color, 1);
		cvLine(img1, cvPoint(1042.008179, 410.545837), cvPoint(1093.99182, 439.454163), color, 1);
		cvLine(img1, cvPoint(1021.999512, 415.500763), cvPoint(1276.00048, 570.499207), color, 1);
		cvLine(img1, cvPoint(272.937775, 420.372345), cvPoint(335.062225, 419.627655), color, 1);
		cvLine(img1, cvPoint(937.327881, 506.572174), cvPoint(976.672119, 551.427856), color, 1);
		cvLine(img1, cvPoint(1.258090, 587.192932), cvPoint(266.741913, 416.807068), color, 1);
		cvLine(img1, cvPoint(123.094109, 497.980927), cvPoint(252.905884, 418.019073), color, 1);
		cvLine(img1, cvPoint(3.548404, 601.321411), cvPoint(276.451599, 420.678558), color, 1);
		cvLine(img1, cvPoint(169.148483, 558.068054), cvPoint(334.851501, 423.931915), color, 1);
		cvLine(img1, cvPoint(427.648865, 425.695892), cvPoint(100.351135, 804.304138), color, 1);
		cvLine(img1, cvPoint(448.735596, 418.377686), cvPoint(141.264420, 797.622314), color, 1);
		cvLine(img1, cvPoint(460.748230, 416.683746), cvPoint(319.251770, 607.316284), color, 1);
		cvLine(img1, cvPoint(3.506108, 689.007690), cvPoint(172.493896, 554.992310), color, 1);
	}
	if (cameraID_e == e_RightCamAlgo)
	{
		cvLine(img1, cvPoint(495.232635, 69.044495), cvPoint(482.767365, 132.955505), color, 1);
		cvLine(img1, cvPoint(565.433838, 110.58358), cvPoint(550.566162, 185.416412), color, 1);
		cvLine(img1, cvPoint(852.132812, 297.87844), cvPoint(897.867188, 348.121552), color, 1);
		cvLine(img1, cvPoint(840.992859, 305.50650), cvPoint(871.007141, 338.493500), color, 1);
		cvLine(img1, cvPoint(911.517578, 361.98400), cvPoint(942.482422, 396.015991), color, 1);
		cvLine(img1, cvPoint(418.616364, 110.80064), cvPoint(463.383636, 137.199356), color, 1);
		cvLine(img1, cvPoint(887.989319, 357.50900), cvPoint(914.010681, 388.490997), color, 1);
		cvLine(img1, cvPoint(1105.372192, 363.1769), cvPoint(1124.627808, 376.823059), color, 1);
		cvLine(img1, cvPoint(377.923737, 130.16793), cvPoint(426.076263, 151.832062), color, 1);
		cvLine(img1, cvPoint(373.403717, 134.20738), cvPoint(506.596283, 195.792618), color, 1);
		cvLine(img1, cvPoint(431.134644, 142.76248), cvPoint(500.865356, 175.237518), color, 1);
		cvLine(img1, cvPoint(347.106262, 163.00866), cvPoint(426.893738, 192.991333), color, 1);
		cvLine(img1, cvPoint(707.704651, 219.12234), cvPoint(738.295349, 232.877655), color, 1);
		cvLine(img1, cvPoint(710.093384, 262.33117), cvPoint(759.906616, 285.668823), color, 1);
		cvLine(img1, cvPoint(630.613403, 263.23574), cvPoint(675.386597, 282.764252), color, 1);
		cvLine(img1, cvPoint(628.494202, 271.74169), cvPoint(677.505798, 290.258301), color, 1);
		cvLine(img1, cvPoint(889.986267, 303.02752), cvPoint(914.013733, 314.972473), color, 1);
		cvLine(img1, cvPoint(1122.125244, 375.7998), cvPoint(1163.874756, 402.200104), color, 1);
		cvLine(img1, cvPoint(285.490875, 69.454323), cvPoint(372.509125, 68.545677), color, 1);
		cvLine(img1, cvPoint(391.498352, 69.286583), cvPoint(492.501648, 68.713417), color, 1);
		cvLine(img1, cvPoint(765.001282, 68.464584), cvPoint(792.998718, 69.535416), color, 1);
		cvLine(img1, cvPoint(398.582703, 192.79800), cvPoint(423.417297, 203.201996), color, 1);
		cvLine(img1, cvPoint(470.488373, 216.92588), cvPoint(491.511627, 217.074112), color, 1);
		cvLine(img1, cvPoint(850.787659, 295.73901), cvPoint(885.212341, 334.260986), color, 1);
		cvLine(img1, cvPoint(852.283875, 318.24176), cvPoint(887.716125, 357.758240), color, 1);
		cvLine(img1, cvPoint(1079.952148, 362.6667), cvPoint(1116.047852, 383.333252), color, 1);
		cvLine(img1, cvPoint(1062.007568, 249.1751), cvPoint(1225.992432, 124.824898), color, 1);
		cvLine(img1, cvPoint(1105.397949, 254.5736), cvPoint(1258.602051, 147.426361), color, 1);
		cvLine(img1, cvPoint(621.000000, 194.00000), cvPoint(639.000000, 194.000000), color, 1);
		cvLine(img1, cvPoint(942.701660, 329.7683), cvPoint(1053.298340, 246.231674), color, 1);
		cvLine(img1, cvPoint(1149.383789, 313.7968), cvPoint(1208.616211, 280.203186), color, 1);
		cvLine(img1, cvPoint(1050.698730, 67.707054), cvPoint(959.301208, 162.292953), color, 1);
		cvLine(img1, cvPoint(944.940979, 151.94599), cvPoint(879.059021, 224.054001), color, 1);
		cvLine(img1, cvPoint(985.175964, 330.7744), cvPoint(1008.824036, 315.225525), color, 1);
		cvLine(img1, cvPoint(1020.013245, 70.049965), cvPoint(959.986755, 135.950043), color, 1);
		cvLine(img1, cvPoint(1050.795166, 247.7315), cvPoint(1095.204834, 214.268448), color, 1);
		cvLine(img1, cvPoint(997.956055, 289.9425), cvPoint(1024.043945, 270.057404), color, 1);
		cvLine(img1, cvPoint(1156.095703, 310.6687), cvPoint(1203.904297, 283.331268), color, 1);
	}
	cvShowImage("Embedded line_clusters", img1);
	cvWaitKey(0);

#endif

		return 0;
	}
#endif

	VanPoints::Point_s VanPoints::findIntersection_s(const Line_s& i_Line1_rs,
		const Line_s& i_Line2_rs)
	{
		Point_s v_Inter_s;

		v_Inter_s.x_f32 = i_Line1_rs.l_af32[1] * i_Line2_rs.l_af32[2] - i_Line1_rs.l_af32[2] * i_Line2_rs.l_af32[1];
		v_Inter_s.y_f32 = i_Line1_rs.l_af32[2] * i_Line2_rs.l_af32[0] - i_Line1_rs.l_af32[0] * i_Line2_rs.l_af32[2];
		v_Inter_s.z_f32 = i_Line1_rs.l_af32[0] * i_Line2_rs.l_af32[1] - i_Line1_rs.l_af32[1] * i_Line2_rs.l_af32[0];

		return v_Inter_s;
	}

	float32_t VanPoints::findOrthDistance(const Line_s &i_Line_rs,
		const Point_s &i_Point_rs)
	{
		float32_t v_Lhat_af32[3];
		v_Lhat_af32[0] = i_Line_rs.mean_af32[1] * i_Point_rs.z_f32 - i_Point_rs.y_f32;
		v_Lhat_af32[1] = i_Point_rs.x_f32 - i_Line_rs.mean_af32[0] * i_Point_rs.z_f32;
		v_Lhat_af32[2] = i_Line_rs.mean_af32[0] * i_Point_rs.y_f32 - i_Line_rs.mean_af32[1] * i_Point_rs.x_f32;

		const float32_t c_Dis_f32 = mecl::math::abs_x<float32_t>(
			v_Lhat_af32[0] * i_Line_rs.x1_af32[0] + v_Lhat_af32[1] * i_Line_rs.x1_af32[1]
			+ v_Lhat_af32[2])
			/ mecl::math::algebra<float32_t>::sqrt_x(
				v_Lhat_af32[0] * v_Lhat_af32[0] + v_Lhat_af32[1] * v_Lhat_af32[1]);
		return c_Dis_f32;
	}

	float32_t VanPoints::jaccardDist(const sint32_t* A,
		const sint32_t* B,
		sint32_t len)
	{
		sint32_t n1 = 0;
		sint32_t n2 = 0;

		for (sint32_t i = 0; i < len; ++i)
		{
			if ((A[i] == 1) || (B[i] == 1))
			{
				n1++;
			}
			if ((A[i] == 1) && (B[i] == 1))
			{
				n2++;
			}
		}

		const float32_t c_Dis_f32 = static_cast<float32_t>(n1 - n2) / static_cast<float32_t>(n1);
		return c_Dis_f32;
	}


}

