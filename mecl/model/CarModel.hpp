// --------------------------------------------------------------------------
/// @file CarModel.hpp
/// @brief Contains the template class CarModel 
///
/// --------------------------------------------------------------------------+++ 
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Jelena Zuravlova (jelena.zuravlova@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef __MECL_CAR_MODEL_HPP__
#define __MECL_CAR_MODEL_HPP__

#include "CarModel.h"

namespace mecl{
namespace model{

// --------------------------------------------------------------------------
/// @brief Precompute static const members
// --------------------------------------------------------------------------
template<typename T>
void CarModel<T>::Init()
{
	piOver2_x = mecl::math::getPi_x<T>() / 2.0F;
}

// --------------------------------------------------------------------------
/// @brief Class CarModel constructor for default current position.
///
/// The constructor initializes wheelbase length and computes
/// rear axle current position based on default current car position
/// (coordinates and angle equal to zero).
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor2
///
/// @param[in] i_WheelBase_x    Wheelbase length
// --------------------------------------------------------------------------
template<typename T>
CarModel<T>::CarModel(T i_WheelBase_x)
: wheelBase_x(i_WheelBase_x)
, frontAxleCurrentPos_s()
, frontAxlePredictedPos_s()
, rearAxleCurrentPos()
, rearAxlePredictedPos()
{
	Init();
	rearAxleCurrentPos.setPosX(frontAxleCurrentPos_s.xy.getPosX() + cos(frontAxleCurrentPos_s.angle_x) * i_WheelBase_x);
	rearAxleCurrentPos.setPosY(frontAxleCurrentPos_s.xy.getPosY() + sin(frontAxleCurrentPos_s.angle_x) * i_WheelBase_x);
}

// --------------------------------------------------------------------------
/// @brief Class CarModel constructor for custom current position.
///
/// The constructor initializes wheelbase length as well as front axle
/// position and calculates rear axle current position.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor3
///
/// @param[in] i_frontAxleCurrentPos_s		Front axle current position
/// @param[in] i_WheelBase_x				Wheelbase length
// --------------------------------------------------------------------------
template<typename T>
CarModel<T>::CarModel(Position_s<T> i_FrontAxleCurrentPos_s,
                      T i_WheelBase_x)
: wheelBase_x(i_WheelBase_x)
, frontAxleCurrentPos_s(i_FrontAxleCurrentPos_s)
, frontAxlePredictedPos_s()
, rearAxleCurrentPos()
, rearAxlePredictedPos()
{
	Init();
	rearAxleCurrentPos.setPosX(frontAxleCurrentPos_s.xy.getPosX() + cos(frontAxleCurrentPos_s.angle_x) * i_WheelBase_x);
	rearAxleCurrentPos.setPosY(frontAxleCurrentPos_s.xy.getPosY() + sin(frontAxleCurrentPos_s.angle_x) * i_WheelBase_x);
}

// --------------------------------------------------------------------------
/// @brief Default constructor
///
/// CarModel constructor without car configuration data.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor1
///
/// Contructs a CarModel object. All car data are set to zero.
// --------------------------------------------------------------------------
template<typename T>
CarModel<T>::CarModel()
: wheelBase_x(mecl::math::constants<T>::zero_x())
, frontAxleCurrentPos_s()
, frontAxlePredictedPos_s()
, rearAxleCurrentPos()
, rearAxlePredictedPos()
{
	Init();
}

// --------------------------------------------------------------------------
/// @brief Default class CarModel destructor
// --------------------------------------------------------------------------
template<typename T>
CarModel<T>::~CarModel()
 {
	// delete ;
 }

// --------------------------------------------------------------------------
/// @brief Set car configuration data.
///
///	Sets car data (wheelBase_x) and computes rear axle current position
/// based on default current car position(coordinates and angle equal to zero).
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor1
/// @snippet ModelTester.cpp CarModel_setCarConfig_v
///
/// @param[in] i_WheelBase_x	Wheelbase length
// --------------------------------------------------------------------------
template<typename T>
void CarModel<T>::setCarConfig_v(T i_WheelBase_x) 
{
	wheelBase_x = i_WheelBase_x;
	rearAxleCurrentPos.setPosX(cos(frontAxleCurrentPos_s.angle_x) * i_WheelBase_x);
	rearAxleCurrentPos.setPosY(sin(frontAxleCurrentPos_s.angle_x) * i_WheelBase_x);
}

// --------------------------------------------------------------------------
/// @brief Set current car coordinates.
/// 
/// Shifts a car to the new coordinates stated. Front axle angle 
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor3
/// @snippet ModelTester.cpp CarModel_setCurrentCoordinates_v
///
/// @param[in] i_CurrentCoordinates New (x,y)-coordinates the car should be 
/// shifted to. 
/// @return	void
///
/// @return	void
// --------------------------------------------------------------------------
template<typename T>
void CarModel<T>::setCurrentCoordinates_v(mecl::core::Point2D<T> i_CurrentCoordinates)
{
	frontAxleCurrentPos_s.xy.setPosX(i_CurrentCoordinates.getPosX());
	frontAxleCurrentPos_s.xy.setPosY(i_CurrentCoordinates.getPosY());
	rearAxleCurrentPos.setPosX(frontAxleCurrentPos_s.xy.getPosX() + cos(frontAxleCurrentPos_s.angle_x) * wheelBase_x);
	rearAxleCurrentPos.setPosY(frontAxleCurrentPos_s.xy.getPosY() + sin(frontAxleCurrentPos_s.angle_x) * wheelBase_x);
}

// --------------------------------------------------------------------------
/// @brief Set current car angle.
/// 
/// Rotates a car about the middle point of the front axle 
/// (frontAxleCurrentPos_s.xy). Front axle coordinates remain the same.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor3
/// @snippet ModelTester.cpp CarModel_setCurrentAngle_v
///
/// @param[in] i_CurrentAngle	New value for the current car angle.
/// @return	void
// --------------------------------------------------------------------------
template<typename T>
void CarModel<T>::setCurrentAngle_v(T i_CurrentAngle)
{
	frontAxleCurrentPos_s.angle_x = i_CurrentAngle;
	rearAxleCurrentPos.setPosX(frontAxleCurrentPos_s.xy.getPosX() + cos(frontAxleCurrentPos_s.angle_x) * wheelBase_x);
	rearAxleCurrentPos.setPosY(frontAxleCurrentPos_s.xy.getPosY() + sin(frontAxleCurrentPos_s.angle_x) * wheelBase_x);
}

// --------------------------------------------------------------------------
/// @brief Set both current car coordinates and current car angle.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor3
/// @snippet ModelTester.cpp CarModel_setCurrentPosition_v
///
/// @param[in] i_CurrentPosition_s	New (x,y)-coordinates and angle of the car.
/// @return	void
// --------------------------------------------------------------------------
template<typename T>
void CarModel<T>::setCurrentPosition_v(Position_s<T> i_CurrentPosition_s)
{
	frontAxleCurrentPos_s = i_CurrentPosition_s;
	rearAxleCurrentPos.setPosX(frontAxleCurrentPos_s.xy.getPosX() + cos(frontAxleCurrentPos_s.angle_x) * wheelBase_x);
	rearAxleCurrentPos.setPosY(frontAxleCurrentPos_s.xy.getPosY() + sin(frontAxleCurrentPos_s.angle_x) * wheelBase_x);
}

 // --------------------------------------------------------------------------
/// @brief Computes predicted position of a car for given path (car trajectory) 
/// length and steering angle.
/// 
/// The function performs computations for input steering angle in the range
/// ]-pi/2, pi/2[ only.
///
/// frontAxleCurrentPos_s.angle (and frontAxlePredictedPos_s.angle) is defined  
/// as the angle between the car longitudinal axis and the X-axis, increasing  
/// in the direction of the driver seat. 
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor3
/// @snippet ModelTester.cpp CarModel_computePredictedPosition_v
///
/// @param[in] i_PathLength Negative values mean forward driving, positive  
/// values mean backing up.
/// @param[in] i_SteeringAngle Only ]-pi/2, pi/2[ values are accepted
/// @return	bool True: in case of i_SteeringAngle in ]-pi/2, pi/2[, in such  
/// case computations are performed and car data altered. False: in case of  
/// i_SteeringAngle not in ]-pi/2, pi/2[, no computations are performed.
// --------------------------------------------------------------------------
template<typename T>
bool CarModel<T>::computePredictedPosition_v(T i_SteeringAngle, 
											 T i_PathLength)
{
	if (mecl::math::abs_x(i_SteeringAngle) >= piOver2_x)
	{
		return(false);
	}
	else
	{
		// transform to the CS used in the computation model
		T d = - i_PathLength;
		T autoAngle_0 = mecl::math::getPi_x<T>() + frontAxleCurrentPos_s.angle_x;
		T autoAngle_d (mecl::math::constants<T>::zero_x());
		
		if (mecl::math::abs_x(i_SteeringAngle) > mecl::math::numeric_limits<T>::epsilon_x())
		{
			T rotationRadius = wheelBase_x / tan(i_SteeringAngle);
			autoAngle_d = 1.0F / rotationRadius * d + autoAngle_0;
			T intergationConst_x = rearAxleCurrentPos.getPosX() - rotationRadius * sin(autoAngle_0);
			T intergationConst_y = rearAxleCurrentPos.getPosY() + rotationRadius * cos(autoAngle_0);
			rearAxlePredictedPos.setPosX(intergationConst_x + rotationRadius * sin(d / rotationRadius + autoAngle_0));
			rearAxlePredictedPos.setPosY(intergationConst_y - rotationRadius * cos(d / rotationRadius + autoAngle_0));
		}
		else
		{
			autoAngle_d = autoAngle_0;
			rearAxlePredictedPos.setPosX(rearAxleCurrentPos.getPosX() + d * cos(autoAngle_d));
			rearAxlePredictedPos.setPosY(rearAxleCurrentPos.getPosY() + d * sin(autoAngle_d));
		}

		frontAxlePredictedPos_s.xy.setPosX(rearAxlePredictedPos.getPosX() + wheelBase_x * cos(autoAngle_d));
		frontAxlePredictedPos_s.xy.setPosY(rearAxlePredictedPos.getPosY() + wheelBase_x * sin(autoAngle_d));
		frontAxlePredictedPos_s.angle_x = autoAngle_d - mecl::math::getPi_x<T>();
		
		return(true);
	}
}

// --------------------------------------------------------------------------
/// @brief Move a car to a computed predicted position
/// 
/// Moves a car to the predicted position stored in frontAxlePredictedPos_s 
/// by setting the position frontAxleCurrentPos_s to be equal to the position 
/// frontAxlePredictedPos_s.
/// 
/// @attention Move a car to a predicted position only after computing 
/// predicted position for a trailer as the computation uses car current
/// position data which are rewritten in case of such update.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor3
/// @snippet ModelTester.cpp CarModel_computePredictedPosition_v
/// @snippet ModelTester.cpp CarModel_moveToPredictedPosition_v
///
/// @return	void
// --------------------------------------------------------------------------
template<typename T>
void CarModel<T>::moveToPredictedPosition_v()
{
	frontAxleCurrentPos_s = frontAxlePredictedPos_s;
	rearAxleCurrentPos = rearAxlePredictedPos;
}

// --------------------------------------------------------------------------
/// @brief Compute global predicted coordinates of a car point given in local 
/// car CS.
/// 
/// Computes predicted position of an arbitrary car point described in the car
/// coordinate system. Predicted position of the car has to be computed first.
/// 
/// Car coordinate system is defined as follows: 
/// The origin of the coordinates is at the (middle of the) front axle.
/// X-axis coincides with the car longitudinal axis pointing toward the rear 
/// bodywork of the car. Y-axis coincides with the front axle of the car   
/// pointing toward the front passenger seat.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor3
/// @snippet ModelTester.cpp CarModel_computePredictedPosition_v
/// @snippet ModelTester.cpp CarModel_moveToPredictedPosition_v
/// @snippet ModelTester.cpp CarModel_localCSToGlobalCSPredicted_v
/// 
/// @param[in] i_PointInLocalCS A car point defined in the car coordinate system
/// @param[out] o_PointPredictedPosInGlobalCS Predicted position of the car point  
/// in the global coordinate system
/// @return	void
// --------------------------------------------------------------------------
template<typename T>
void CarModel<T>::localCSToGlobalCSPredicted_v
					(mecl::core::Point2D<T>    i_PointInLocalCS,
					 mecl::core::Point2D<T>&   o_PointPredictedPosInGlobalCS)
{
	o_PointPredictedPosInGlobalCS.setPosX(i_PointInLocalCS.getPosX() * cos(frontAxlePredictedPos_s.angle_x) - 
						i_PointInLocalCS.getPosY() * sin(frontAxlePredictedPos_s.angle_x) + frontAxlePredictedPos_s.xy.getPosX());
	o_PointPredictedPosInGlobalCS.setPosY(i_PointInLocalCS.getPosX() * sin(frontAxlePredictedPos_s.angle_x) + 
						i_PointInLocalCS.getPosY() * cos(frontAxlePredictedPos_s.angle_x) + frontAxlePredictedPos_s.xy.getPosY());
}

// --------------------------------------------------------------------------
/// @brief Compute global current coordinates of a car point given in local 
/// car CS.   
///
/// Car coordinate system is defined as follows: 
/// The origin of the coordinates is at the (middle of the) front axle.
/// X-axis coincides with the car's longitudinal axis pointing toward the rear 
/// bodywork of the car. Y-axis coincides with the front axle of the car   
/// pointing toward the front passenger seat.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor3
/// @snippet ModelTester.cpp CarModel_computePredictedPosition_v
/// @snippet ModelTester.cpp CarModel_moveToPredictedPosition_v
/// @snippet ModelTester.cpp CarModel_localCSToGlobalCSCurrent_v
/// 
/// @param[in] i_PointInLocalCS A car point defined in the car coordinate system
/// @param[out] o_PointCurrectPosInGlobalCS The coordinates of the point  in   
/// global CS corresponding to the currect car position
/// @return	void
// --------------------------------------------------------------------------
template<typename T>
void CarModel<T>::localCSToGlobalCSCurrent_v
					(mecl::core::Point2D<T>    i_PointInLocalCS,
					 mecl::core::Point2D<T>&   o_PointCurrectPosInGlobalCS)
{
	o_PointCurrectPosInGlobalCS.setPosX(i_PointInLocalCS.getPosX() * cos(frontAxleCurrentPos_s.angle_x) - 
						i_PointInLocalCS.getPosY() * sin(frontAxleCurrentPos_s.angle_x) + frontAxleCurrentPos_s.xy.getPosX());
	o_PointCurrectPosInGlobalCS.setPosY(i_PointInLocalCS.getPosX() * sin(frontAxleCurrentPos_s.angle_x) + 
						i_PointInLocalCS.getPosY() * cos(frontAxleCurrentPos_s.angle_x) + frontAxleCurrentPos_s.xy.getPosY());
}

// --------------------------------------------------------------------------
/// @brief Return current position of a car
///
/// Returns current position of a car as the data of the structure Position_s<T>
/// containing x,y-coordinates of the car front axle and car angle.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor3
/// @snippet ModelTester.cpp CarModel_computePredictedPosition_v
/// @snippet ModelTester.cpp CarModel_moveToPredictedPosition_v
/// @snippet ModelTester.cpp CarModel_getCurrentPosition_v
/// 
/// @return		Car current position		
// --------------------------------------------------------------------------
template<typename T>
Position_s<T> CarModel<T>::getCurrentPosition_v()
{
	return(this->frontAxleCurrentPos_s);
}

// --------------------------------------------------------------------------
/// @brief Return predicted position of a car
///
/// Returns predicted position of a car as the data of the structure 
/// Position_s<T> containing predicted x,y-coordinates of the car front axle 
/// and predicted car angle.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor3
/// @snippet ModelTester.cpp CarModel_computePredictedPosition_v
/// @snippet ModelTester.cpp CarModel_getPredictedPosition_v
///
/// @return		Car predicted position
// --------------------------------------------------------------------------
template<typename T>
Position_s<T> CarModel<T>::getPredictedPosition_v()
{
	return(this->frontAxlePredictedPos_s);
}

// --------------------------------------------------------------------------
/// @brief Outputs current and predicted position data on the console.
///
/// @attention Reserved only for debugging purposes.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor3
/// @snippet ModelTester.cpp CarModel_computePredictedPosition_v
/// @snippet ModelTester.cpp CarModel_print_v
///
/// @return	void
// --------------------------------------------------------------------------
template<typename T>
void CarModel<T>::print_v()
{
  log_printf("car data:\n-----------\n");
  log_printf("front axle:  start pos: x:%f y:%f, angle: %f\n",
      frontAxleCurrentPos_s.xy.getPosX(), frontAxleCurrentPos_s.xy.getPosY(), frontAxleCurrentPos_s.angle_x);
  log_printf("-----------  end pos:   x:%f y:%f, angle: %f\n",
      frontAxlePredictedPos_s.xy.getPosX(), frontAxlePredictedPos_s.xy.getPosY(), frontAxlePredictedPos_s.angle_x);
  log_printf("rear axle:   start pos: x:%f y:%f\n", rearAxleCurrentPos.getPosX(), rearAxleCurrentPos.getPosY());
  log_printf("-----------  end pos:   x:%f y:%f\n", rearAxlePredictedPos.getPosX(), rearAxlePredictedPos.getPosY());
  return;
}

}; // namespace model
}; // namespace mecl

#endif
/// @}
/// @}
