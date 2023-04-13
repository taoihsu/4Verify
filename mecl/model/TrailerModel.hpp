//--------------------------------------------------------------------------
/// @file TrailerModel.hpp
/// @brief Contains the template class TrailerModel 
///
/// -------------------------------------------------------------------------+++ 
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Jelena Zuravlova (jelena.zuravlova@magna.com)
///
//  -------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef __MECL_TRAILER_MODEL_HPP__
#define __MECL_TRAILER_MODEL_HPP__

#include "TrailerModel.h"

namespace mecl{
namespace model{
//--------------------------------------------------------------------------
/// @brief Precompute static const members
// -------------------------------------------------------------------------
template<typename T>
void TrailerModel<T>::Init()
{
	piOver2_x = mecl::math::getPi_x<T>() / 2.0F;
	piOver4_x = mecl::math::getPi_x<T>() / 4.0F;
	piTimes2_x = mecl::math::getPi_x<T>() * 2.0F;
}

//--------------------------------------------------------------------------
/// @brief Class TrailerModel constructor for given trailer configuration.
///
/// Contructs a trailer object, sets trailer configuration (trailerLength_x 
/// and hitchPositionInCarCS) and computes its current position with the
/// current trailer angle equal to zero. 
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor
/// @snippet ModelTester.cpp TrailerModel_Constructor3
///
/// @param[in] i_TrailerLength_x	Length of a trailer (from hitch till trailer axle)
/// @param[in] i_hitchX_x			Hitch X-coordinate in the local car CS
/// @param[in] i_car_ro				A car object the trailer is attached to
// -------------------------------------------------------------------------
template<typename T>
TrailerModel<T>::TrailerModel(T i_TrailerLength_x, 
							  T i_hitchX_x,	
                              CarModel<T>& i_car_ro) 
: 	trailerLength_x(i_TrailerLength_x)
,	hitchPositionInCarCS()
, 	trailerCurrentPos_s()
,	trailerPredictedPos_s()
,	car_ro (i_car_ro)
{
	Init();
	hitchPositionInCarCS.setPosX(i_hitchX_x);
	trailerCurrentPos_s.angle_x = 0.0F;
	T trailerwinkel_0 = piTimes2_x + car_ro.frontAxleCurrentPos_s.angle_x; 
	T hitchGlobalX  = hitchPositionInCarCS.getPosX() * cos(car_ro.frontAxleCurrentPos_s.angle_x) + car_ro.frontAxleCurrentPos_s.xy.getPosX();
	T hitchGlobalY  = hitchPositionInCarCS.getPosX() * sin(car_ro.frontAxleCurrentPos_s.angle_x) + car_ro.frontAxleCurrentPos_s.xy.getPosY();
	trailerCurrentPos_s.xy.setPosX(hitchGlobalX + cos(trailerwinkel_0) * i_TrailerLength_x);
    trailerCurrentPos_s.xy.setPosY(hitchGlobalY + sin(trailerwinkel_0) * i_TrailerLength_x);
}

//--------------------------------------------------------------------------
/// @brief Class TrailerModel constructor without trailer configuration data.
///
/// Contructs a TrailerModel object only specifying a car object the trailer   
/// is attached to. The rest of the trailer data is set to zero.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor
/// @snippet ModelTester.cpp TrailerModel_Constructor2
/// 
/// @param[in] i_car_ro				A car object the trailer is attached to
// -------------------------------------------------------------------------
template<typename T>
TrailerModel<T>::TrailerModel(CarModel<T>& i_car_ro) 
: 	trailerLength_x(mecl::math::constants<T>::zero_x())
,	hitchPositionInCarCS()
,	trailerCurrentPos_s()
,	trailerPredictedPos_s()
,	car_ro (i_car_ro)
{
	Init();
}
//--------------------------------------------------------------------------
/// @brief Default class TrailerModel deconstructor
// -------------------------------------------------------------------------
template<typename T>
TrailerModel<T>::~TrailerModel()
 {
	// delete ;
 }

//--------------------------------------------------------------------------
/// @brief Set trailer configuration data.
///
///	Sets trailer data and computes trailer current position with the current 
/// trailer angle trailerCurrentPos_s.angle_x equal to zero. Use the trailer 
/// function setCurrentAngle_v to change current trailer angle.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor
/// @snippet ModelTester.cpp TrailerModel_Constructor2
/// @snippet ModelTester.cpp TrailerModel_setTrailerConfig_v
///
/// @param[in] i_TrailerLength_x	Length of a trailer (from hitch till trailer axle)
/// @param[in] i_hitchX_x			Hitch X-coordinate in the local car CS
// -------------------------------------------------------------------------
template<typename T>
void TrailerModel<T>::setTrailerConfig_v(T i_TrailerLength_x, 
										 T i_hitchX_x) 
{
	trailerLength_x = i_TrailerLength_x;
	hitchPositionInCarCS.setPosX(i_hitchX_x);
	T trailerwinkel_0 = piTimes2_x + car_ro.frontAxleCurrentPos_s.angle_x; 
	T hitchGlobalX  = hitchPositionInCarCS.getPosX() * cos(car_ro.frontAxleCurrentPos_s.angle_x) + car_ro.frontAxleCurrentPos_s.xy.getPosX();
	T hitchGlobalY  = hitchPositionInCarCS.getPosX() * sin(car_ro.frontAxleCurrentPos_s.angle_x) + car_ro.frontAxleCurrentPos_s.xy.getPosY();
	trailerCurrentPos_s.xy.setPosX(hitchGlobalX + cos(trailerwinkel_0) * i_TrailerLength_x);
    trailerCurrentPos_s.xy.setPosY(hitchGlobalY + sin(trailerwinkel_0) * i_TrailerLength_x);
}

//--------------------------------------------------------------------------
/// @brief Set current trailer angle.
/// 
/// Rotates a trailer about the hitch. Position of a corresponding car remains 
/// the same.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor
/// @snippet ModelTester.cpp TrailerModel_Constructor3
/// @snippet ModelTester.cpp TrailerModel_setCurrentAngle_v
///
/// @param[in] i_CurrentAngle_x		New value for the current trailer angle.
/// @return	void
// -------------------------------------------------------------------------
template<typename T>
void TrailerModel<T>::setCurrentAngle_v(T i_CurrentAngle_x)
{
	trailerCurrentPos_s.angle_x = i_CurrentAngle_x;
	T trailerwinkel_0 = piTimes2_x + car_ro.frontAxleCurrentPos_s.angle_x + trailerCurrentPos_s.angle_x; 
	T hitchGlobalX  = hitchPositionInCarCS.getPosX() * cos(car_ro.frontAxleCurrentPos_s.angle_x) + car_ro.frontAxleCurrentPos_s.xy.getPosX();
	T hitchGlobalY  = hitchPositionInCarCS.getPosX() * sin(car_ro.frontAxleCurrentPos_s.angle_x) + car_ro.frontAxleCurrentPos_s.xy.getPosY();
	trailerCurrentPos_s.xy.setPosX(hitchGlobalX + cos(trailerwinkel_0) * trailerLength_x);
    trailerCurrentPos_s.xy.setPosY(hitchGlobalY + sin(trailerwinkel_0) * trailerLength_x);
}

//--------------------------------------------------------------------------
// @brief Set current trailer coordinates.
///
/// Shifts a trailer to the new coordinates stated and computes corresponding
/// location of a car. Angles (car_ro.frontAxleCurrentPos_s.angle_x and
/// trailerCurrentPos_s.angle_x) remain the same.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor
/// @snippet ModelTester.cpp TrailerModel_Constructor3
/// @snippet ModelTester.cpp TrailerModel_setCurrentCoordinates_v
///
/// @param[in] i_CurrentCoordinates		New (x,y)-coordinates the trailer  
/// should be shifted to. 
/// @return	void
//--------------------------------------------------------------------------
template<typename T>
void TrailerModel<T>::setCurrentCoordinates_v(mecl::core::Point2D<T> i_CurrentCoordinates)
{
	trailerCurrentPos_s.xy.setPosX(i_CurrentCoordinates.getPosX());
	trailerCurrentPos_s.xy.setPosY(i_CurrentCoordinates.getPosY());
	
	T trailerwinkel_0 = piTimes2_x + car_ro.frontAxleCurrentPos_s.angle_x + trailerCurrentPos_s.angle_x; 
	T hitchGlobalX  = trailerCurrentPos_s.xy.getPosX() - cos(trailerwinkel_0) * trailerLength_x;
	T hitchGlobalY  = trailerCurrentPos_s.xy.getPosY() - sin(trailerwinkel_0) * trailerLength_x;

	car_ro.frontAxleCurrentPos_s.xy.setPosX(hitchGlobalX - hitchPositionInCarCS.getPosX() * cos(car_ro.frontAxleCurrentPos_s.angle_x));
	car_ro.frontAxleCurrentPos_s.xy.setPosY(hitchGlobalY - hitchPositionInCarCS.getPosX() * sin(car_ro.frontAxleCurrentPos_s.angle_x));
	car_ro.rearAxleCurrentPos.setPosX(car_ro.frontAxleCurrentPos_s.xy.getPosX() + cos(car_ro.frontAxleCurrentPos_s.angle_x) * car_ro.wheelBase_x);
	car_ro.rearAxleCurrentPos.setPosY(car_ro.frontAxleCurrentPos_s.xy.getPosY() + sin(car_ro.frontAxleCurrentPos_s.angle_x) * car_ro.wheelBase_x);
}

//--------------------------------------------------------------------------
/// @brief Set both current trailer coordinates and current trailer angle.  
/// 
/// Shifts a trailer and hitch so that the trailer is at the new stated  
/// coordinates and computes location of a car taking into account new 
/// current coordinates of the trailer and desired trailer angle stated. 
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor
/// @snippet ModelTester.cpp TrailerModel_Constructor3
/// @snippet ModelTester.cpp TrailerModel_setCurrentPosition_v
///
/// @param[in] i_CurrentPosition_s		New (x,y)-coordinates and angle of  
/// the trailer.
/// @return	void
// -------------------------------------------------------------------------
template<typename T>
void TrailerModel<T>::setCurrentPosition_v(Position_s<T> i_CurrentPosition_s)
{
	T trailerwinkel_0 = piTimes2_x + car_ro.frontAxleCurrentPos_s.angle_x + trailerCurrentPos_s.angle_x; 
	T hitchGlobalX  = trailerCurrentPos_s.xy.getPosX() - cos(trailerwinkel_0) * trailerLength_x;
	T hitchGlobalY  = trailerCurrentPos_s.xy.getPosY() - sin(trailerwinkel_0) * trailerLength_x;
	hitchGlobalX += i_CurrentPosition_s.xy.getPosX() - trailerCurrentPos_s.xy.getPosX();
	hitchGlobalY += i_CurrentPosition_s.xy.getPosY() - trailerCurrentPos_s.xy.getPosY();

	T deltaTrailerAngle = mecl::math::abs_x(trailerCurrentPos_s.angle_x - i_CurrentPosition_s.angle_x);
	if (trailerCurrentPos_s.angle_x >= i_CurrentPosition_s.angle_x)
		car_ro.frontAxleCurrentPos_s.angle_x += deltaTrailerAngle;
	else
		car_ro.frontAxleCurrentPos_s.angle_x -= deltaTrailerAngle;

	trailerCurrentPos_s = i_CurrentPosition_s;

	car_ro.frontAxleCurrentPos_s.xy.setPosX(hitchGlobalX - hitchPositionInCarCS.getPosX() * cos(car_ro.frontAxleCurrentPos_s.angle_x));
	car_ro.frontAxleCurrentPos_s.xy.setPosY(hitchGlobalY - hitchPositionInCarCS.getPosX() * sin(car_ro.frontAxleCurrentPos_s.angle_x));
	car_ro.rearAxleCurrentPos.setPosX(car_ro.frontAxleCurrentPos_s.xy.getPosX() + cos(car_ro.frontAxleCurrentPos_s.angle_x) * car_ro.wheelBase_x);
	car_ro.rearAxleCurrentPos.setPosY(car_ro.frontAxleCurrentPos_s.xy.getPosY() + sin(car_ro.frontAxleCurrentPos_s.angle_x) * car_ro.wheelBase_x);
}

//-------------------------------------------------------------------------- 
/// @brief Update trailer position after a current car position has been changed.
/// 
/// In case if a car has been moved (shifted or rotated or both), a trailer 
/// position should be updated. Trailer angle with respect to the car remains 
/// the same. Use the trailer's function setCurrentAngle_v to change trailer 
/// angle.
///
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor3
/// @snippet ModelTester.cpp TrailerModel_Constructor3
/// @snippet ModelTester.cpp CarModel_computePredictedPosition_v
/// @snippet ModelTester.cpp CarModel_moveToPredictedPosition_v
/// @snippet ModelTester.cpp TrailerModel_updateTrailerPositionWrtCar_v
///
/// @return	void
// -------------------------------------------------------------------------
template<typename T>
void TrailerModel<T>::updateTrailerPositionWrtCar_v()
{
	T trailerwinkel_0 = piTimes2_x + car_ro.frontAxleCurrentPos_s.angle_x + trailerCurrentPos_s.angle_x; 
	T hitchGlobalX  = hitchPositionInCarCS.getPosX() * cos(car_ro.frontAxleCurrentPos_s.angle_x) + car_ro.frontAxleCurrentPos_s.xy.getPosX();
	T hitchGlobalY  = hitchPositionInCarCS.getPosX() * sin(car_ro.frontAxleCurrentPos_s.angle_x) + car_ro.frontAxleCurrentPos_s.xy.getPosY();
	trailerCurrentPos_s.xy.setPosX(hitchGlobalX + cos(trailerwinkel_0) * trailerLength_x);
    trailerCurrentPos_s.xy.setPosY(hitchGlobalY + sin(trailerwinkel_0) * trailerLength_x);
}

//--------------------------------------------------------------------------
/// @brief Move a trailer to a computed predicted position
///
/// Moves a trailer to the predicted position stored in trailerPredictedPos_s by 
/// setting its current position trailerCurrentPos_s to be equal to  
/// trailerPredictedPos_s. Also moves a respective car to its predicted position.
/// 
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor
/// @snippet ModelTester.cpp TrailerModel_Constructor3
/// @snippet ModelTester.cpp TrailerModel_computePredictedPosition_v
/// @snippet ModelTester.cpp TrailerModel_moveToPredictedPosition_v
///
/// @return	void
// -------------------------------------------------------------------------
template<typename T>
void  TrailerModel<T>::moveToPredictedPosition_v()
{
	trailerCurrentPos_s = trailerPredictedPos_s;
	car_ro.moveToPredictedPosition_v();
}

//--------------------------------------------------------------------------
/// @brief Computes predicted position of a trailer for given path (car 
/// trajectory) length and car steering angle.
/// 
/// The function performs computations for input steering angle in the range
/// ]-pi/2, pi/2[ only.
/// 
/// The trailer angle is defined as an angle between the car longitudinal 
/// axis and the trailer longitudinal axis, increasing in the direction 
/// of the front passenger seat. 
///
/// Also computes predicted position for a car the trailer is attached to.
/// 
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor
/// @snippet ModelTester.cpp TrailerModel_Constructor3
/// @snippet ModelTester.cpp TrailerModel_computePredictedPosition_v
///
/// @param[in] i_PathLength Negative values mean forward driving, positive 
/// values mean backing up.
/// @param[in] i_SteeringAngle Only values in ]-pi/2, pi/2[ are accepted
/// @return	bool True: in case of i_SteeringAngle in ]-pi/2, pi/2[, in such  
/// case computations are performed and trailer data altered. False: in case of  
/// i_SteeringAngle is not in ]-pi/2, pi/2[, no computations are performed.
// -------------------------------------------------------------------------
template<typename T>
bool  TrailerModel<T>::computePredictedPosition_v(T i_SteeringAngle,
												  T i_PathLength)
{
	if (mecl::math::abs_x(i_SteeringAngle) >= piOver2_x)
	{
		return (false);
	}
	else
	{
		if (!car_ro.computePredictedPosition_v (i_SteeringAngle, i_PathLength))
		{
			log_printf("\n |steeringAngle| > 90 \n");
		}

		mecl::core::Point2D<T> newHitchPosition;
		car_ro.localCSToGlobalCSPredicted_v(hitchPositionInCarCS, newHitchPosition);
		
		// transform to the CS used in the computation model
		T d = - i_PathLength;
		T autoAngle_0 = mecl::math::getPi_x<T>() + car_ro.frontAxleCurrentPos_s.angle_x;
		T trailerAngle_0 = car_ro.frontAxleCurrentPos_s.angle_x + trailerCurrentPos_s.angle_x; 
		T trailerAngle_d (mecl::math::constants<T>::zero_x());
		T hitchLength_x = hitchPositionInCarCS.getPosX() - car_ro.wheelBase_x;

		T C1 = hitchLength_x * tan(i_SteeringAngle) / (trailerLength_x * car_ro.wheelBase_x);
		T C2 = tan(i_SteeringAngle) / car_ro.wheelBase_x;
		T a = C2 + C1;
		T b = 2.0F / trailerLength_x;
		T c = C2 - C1;
		T determinant = b * b - 4.0F * a * c;
		
		T D (mecl::math::constants<T>::zero_x());
		T intergrationKonstant (mecl::math::constants<T>::zero_x());
		T atanArgum (mecl::math::constants<T>::zero_x());
		T e (mecl::math::constants<T>::zero_x());

		if (mecl::math::abs_x(i_SteeringAngle) > mecl::math::numeric_limits<T>::epsilon_x())
		{
			if (determinant > mecl::math::numeric_limits<T>::epsilon_x())
			{
				D = sqrt(determinant);
				T arg = 1.0F / D * ( 2.0F * a * tan( (autoAngle_0 - trailerAngle_0) / 2.0F ) + b );
				T logArgum = (arg - 1.0F) / (arg + 1.0F);
				e = logArgum * exp(D * d / 2.0F);
				atanArgum = 1.0F / (2.0F * a) * ( D * (1.0F + e) / (1.0F - e) - b );
				trailerAngle_d =  C2 * d + autoAngle_0 - 2.0F * atan(atanArgum);
			}
			else if (determinant < mecl::math::numeric_limits<T>::epsilon_x()) 
			{
				D = sqrt(-determinant);
				T arg1 = tan(D * d / 4.0F);
				T arg2 = (2.0F * a *  tan((autoAngle_0 - trailerAngle_0) / 2.0F) + b) / D;
				atanArgum = (D * ((arg1 + arg2) / (1.0F - arg1 * arg2)) - b) / (2.0F * a);
				trailerAngle_d = C2 * d + autoAngle_0 - 2.0F * atan(atanArgum);
			}
			else
			{
				intergrationKonstant = -a / 2.0F * (tan((autoAngle_0 - trailerAngle_0) / 2.0F) + b);
				atanArgum = -2.0F / (a * (d + intergrationKonstant)) - b;
				trailerAngle_d = C2 * d + autoAngle_0 - 2.0F * atan(atanArgum);
			}
		}
		else
		{
			atanArgum = exp(d / trailerLength_x) * tan((autoAngle_0 - trailerAngle_0) / 2.0F);
			trailerAngle_d = autoAngle_0 - 2.0F * atan(atanArgum);
		}
		trailerPredictedPos_s.xy.setPosX(newHitchPosition.getPosX() + cos(trailerAngle_d) * trailerLength_x);
		trailerPredictedPos_s.xy.setPosY(newHitchPosition.getPosY() + sin(trailerAngle_d) * trailerLength_x);
		
		T v_OutAngle_x = trailerAngle_d - car_ro.frontAxlePredictedPos_s.angle_x;
		
		if (v_OutAngle_x > piTimes2_x)
		{
			T k = floor (v_OutAngle_x / piTimes2_x);
			v_OutAngle_x -= k * piTimes2_x;
		}
		else if (v_OutAngle_x < -piTimes2_x)
		{
			T k = floor (-1.0F * v_OutAngle_x / piTimes2_x);
			v_OutAngle_x += k * piTimes2_x;
		}

		if (v_OutAngle_x > mecl::math::getPi_x<T>())
		{
			v_OutAngle_x = v_OutAngle_x - piTimes2_x;
		}
		else if (v_OutAngle_x < (-mecl::math::getPi_x<T>()))
		{
			v_OutAngle_x = v_OutAngle_x + piTimes2_x;
		}
		
		trailerPredictedPos_s.angle_x = v_OutAngle_x; 

		return (true);
	}
}

//--------------------------------------------------------------------------
/// @brief Computes angle of no return for a trailer with respect to a car it 
/// is attached to.
///
/// Once the angle of no return is reached, the direction of trailer movement  
/// is no longer altered while in reverse, irrespective of a steering input.
/// Computations are based the following stability criterion: the truck-and-trailer 
/// system is called stable while in reverse, if the line through trailer axle 
/// goes through the rotation centre of a car.
/// 
/// @par Example usage:
/// @snippet ModelTester.cpp CarModel_Constructor
/// @snippet ModelTester.cpp TrailerModel_Constructor3
/// @snippet ModelTester.cpp TrailerModel_noReturnAngle_v
///
/// @param[in] i_carMaxSteeringAngle_x Maximal steering angle, must be less or 
/// equal to pi/4 by absolute value.
/// @param[out] o_noReturnAngle_x Angle of no return
/// @return	bool False if i_carMaxSteeringAngle_x > pi/4 or o_noReturnAngle_x > pi/2.
///
// -------------------------------------------------------------------------
template<typename T>
bool TrailerModel<T>::noReturnAngle_v(T i_carMaxSteeringAngle_x,
									  T& o_noReturnAngle_x)
{
	if (mecl::math::abs_x(i_carMaxSteeringAngle_x) > piOver4_x)
	{
		return(false);
	}
	else
	{
		T maxSteeringAngle = mecl::math::abs_x(i_carMaxSteeringAngle_x);
		T hitchLength_x = hitchPositionInCarCS.getPosX() - car_ro.wheelBase_x;
		
		T tanMaxSteeringAngle = tan(maxSteeringAngle);
		T tanMaxSteeringAngle2 = tanMaxSteeringAngle * tanMaxSteeringAngle;
		T a = hitchLength_x * hitchLength_x * tanMaxSteeringAngle2 + car_ro.wheelBase_x * car_ro.wheelBase_x;
		T b = 2.0F * hitchLength_x * trailerLength_x * tanMaxSteeringAngle2;
		T c = trailerLength_x * trailerLength_x * tanMaxSteeringAngle2 - car_ro.wheelBase_x * car_ro.wheelBase_x;

		T acosArgum1 = (-b + sqrt(b * b - 4.0F * a * c)) / (2.0F * a);
		T acosArgum2 = (-b - sqrt(b * b - 4.0F * a * c)) / (2.0F * a);
		T acosValue1 = acos(acosArgum1);
		T acosValue2 = acos(acosArgum2);

		if (acosValue1 < piOver2_x)
		{
			o_noReturnAngle_x = acosValue1;
		}
		else if (acosValue2 < piOver2_x)                                                                           
		{
			o_noReturnAngle_x = acosValue2;
		}
		else
		{
			return(false);
		}

		return(true);
	}
}

//--------------------------------------------------------------------------
/// @brief Compute global predicted coordinates of a trailer point given in 
/// the local trailer CS.   
///
/// Computes predicted position of an arbitrary trailer point described in 
/// the trailer coordinate system. Predicted position of the trailer has
/// to be computed first.
/// 
/// Trailer coordinate system is defined as follows: 
/// The origin of the coordinates is at the middle of the trailer axle, i.e. 
/// at (HitchX+TrailerLength,0) of the car coordinate system. X-axis coincides 
/// with the trailer longitudinal axis pointing toward the back-end of the 
/// trailer and Y-axis coincides with the axle of the trailer pointing toward 
/// the front passenger seat, i.e. the orientation of the axes is the same
/// as that of the car coordinate system. 
/// 
/// @par Example usage:
/// @snippet ModelTester.cpp TrailerModel_Constructor3
/// @snippet ModelTester.cpp TrailerModel_computePredictedPosition_v
/// @snippet ModelTester.cpp TrailerModel_moveToPredictedPosition_v
/// @snippet ModelTester.cpp TrailerModel_localCSToGlobalCSPredicted_v
/// 
/// @param[in] i_PointInLocalCS A trailer point defined in the trailer 
/// coordinate system
/// @param[out] o_PointPredictedPosInGlobalCS Predicted position of the trailer 
/// point in the global coordinate system
/// @return	void
// -------------------------------------------------------------------------
template<typename T>
void  TrailerModel<T>::localCSToGlobalCSPredicted_v
					(mecl::core::Point2D<T>    i_PointInLocalCS,
					 mecl::core::Point2D<T>&   o_PointPredictedPosInGlobalCS)
{
	T trailerAngle = trailerPredictedPos_s.angle_x + car_ro.frontAxlePredictedPos_s.angle_x + piTimes2_x;
	o_PointPredictedPosInGlobalCS.setPosX(i_PointInLocalCS.getPosX() * cos(trailerAngle) - 
						i_PointInLocalCS.getPosY() * sin (trailerAngle) + trailerPredictedPos_s.xy.getPosX());
	o_PointPredictedPosInGlobalCS.setPosY(i_PointInLocalCS.getPosX() * sin(trailerAngle) + 
						i_PointInLocalCS.getPosY() * cos (trailerAngle) + trailerPredictedPos_s.xy.getPosY());
}

//--------------------------------------------------------------------------
/// @brief Compute global current coordinates of a trailer point given in 
/// the local trailer CS.   
///
/// Trailer coordinate system is defined as follows: 
/// The origin of the coordinates is at the middle of the trailer axle, i.e. 
/// at (HitchX+TrailerLength,0) of the car coordinate system. X-axis coincides 
/// with the trailer longitudinal axis pointing toward the back-end of the 
/// trailer and Y-axis coincides with the axle of the trailer pointing toward 
/// the front passenger seat, i.e. the orientation of the axes is the same
/// as that of the car coordinate system. 
/// 
/// @par Example usage:
/// @snippet ModelTester.cpp TrailerModel_Constructor3
/// @snippet ModelTester.cpp TrailerModel_computePredictedPosition_v
/// @snippet ModelTester.cpp TrailerModel_moveToPredictedPosition_v
/// @snippet ModelTester.cpp TrailerModel_localCSToGlobalCSCurrent_v
///
/// @param[in] i_PointInLocalCS A trailer point defined in trailer CS
/// @param[out] o_PointCurrectPosInGlobalCS The coordinates of the point 
/// in global CS corresponding to the currect trailer position
/// @return	void
// -------------------------------------------------------------------------
template<typename T>
void TrailerModel<T>::localCSToGlobalCSCurrent_v
					(mecl::core::Point2D<T>    i_PointInLocalCS,
					 mecl::core::Point2D<T>&   o_PointCurrectPosInGlobalCS)
{
	T trailerAngle = trailerCurrentPos_s.angle_x + car_ro.frontAxleCurrentPos_s.angle_x + piTimes2_x;
	o_PointCurrectPosInGlobalCS.setPosX(i_PointInLocalCS.getPosX() * cos(trailerAngle) - 
						i_PointInLocalCS.getPosY() * sin (trailerAngle) + trailerCurrentPos_s.xy.getPosX());
	o_PointCurrectPosInGlobalCS.setPosY(i_PointInLocalCS.getPosX() * sin(trailerAngle) + 
						i_PointInLocalCS.getPosY() * cos (trailerAngle) + trailerCurrentPos_s.xy.getPosY());
}

//--------------------------------------------------------------------------
/// @brief Return current position of a trailer
///
/// Returns current position of a trailer as the data of the structure Position_s<T>
/// containing x,y-coordinates of the trailer axle and trailer angle.
///
/// @par Example usage:
/// @snippet ModelTester.cpp TrailerModel_Constructor3
/// @snippet ModelTester.cpp TrailerModel_computePredictedPosition_v
/// @snippet ModelTester.cpp TrailerModel_moveToPredictedPosition_v
/// @snippet ModelTester.cpp TrailerModel_getCurrentPosition_v
/// 
/// @return			Trailer current position
// -------------------------------------------------------------------------
template<typename T>
Position_s<T> TrailerModel<T>::getCurrentPosition_v()
{
	return(this->trailerCurrentPos_s);
}

//--------------------------------------------------------------------------
/// @brief Return predicted position of a trailer
///
/// Returns predicted position of a trailer as the data of the structure 
/// Position_s<T> containing predicted x,y-coordinates of the trailer axle 
/// and predicted trailer angle.
///
/// @par Example usage:
/// @snippet ModelTester.cpp TrailerModel_Constructor3
/// @snippet ModelTester.cpp TrailerModel_computePredictedPosition_v
/// @snippet ModelTester.cpp TrailerModel_getPredictedPosition_v
///
/// @return			Trailer predicted position
///
// -------------------------------------------------------------------------
template<typename T>
Position_s<T> TrailerModel<T>::getPredictedPosition_v()
{
	return(this->trailerPredictedPos_s);
}

//--------------------------------------------------------------------------
/// @brief Outputs current and predicted position data on the console.
///
/// @attention Reserved only for debugging purposes.
///
/// @par Example usage:
/// @snippet ModelTester.cpp TrailerModel_Constructor3
/// @snippet ModelTester.cpp TrailerModel_computePredictedPosition_v
/// @snippet ModelTester.cpp TrailerModel_print_v
///
/// @return	void
// -------------------------------------------------------------------------
template<typename T>
void  TrailerModel<T>::print_v()
{
  log_printf("trailer data:\n-------------\n");
  log_printf("             start pos: x:%f y:%f, angle: %f\n",
      trailerCurrentPos_s.xy.getPosX(), trailerCurrentPos_s.xy.getPosY(), trailerCurrentPos_s.angle_x);
  log_printf("             end pos:   x:%f y:%f, angle: %f\n",
      trailerPredictedPos_s.xy.getPosX(), trailerPredictedPos_s.xy.getPosY(), trailerPredictedPos_s.angle_x);
  return;
}

}; // namespace model
}; // namespace mecl

#endif

/// @}
/// @}
