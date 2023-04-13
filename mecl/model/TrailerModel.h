//--------------------------------------------------------------------------
/// @file TrailerModel.h
/// @brief Contains the template class TrailerModel 
///
/// --------------------------------------------------------------------------
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

#ifndef __MECL_TRAILER_MODEL_H__
#define __MECL_TRAILER_MODEL_H__

#include "math.h"
#include "core/MeclTypes.h"
#include "math/Math.h"
#include "core/Point.h"
#include "CarModel.h"

namespace mecl{
namespace model{

//--------------------------------------------------------------------------
/// @class TrailerModel
/// @brief Template class TrailerModel.
/// 
/// Template class TrailerModel including predicted position computation 
/// for given car path (trajectory) length and steering angle for a trailer  
/// or an arbitrary point described in the trailer coordinate system and 
/// computation of the angle of no return.
/// 
/// A trailer can be defined either by specifying the trailer configuration 
/// (trailer length and hitch X-coordinate in the car CS) and a car object 
/// it is attached to or by specifying a corresponding car object only.	
/// The former contructor automatically computes current trailer coordinates, 
/// and the latter contructor sets current trailer coordinates to zero.
/// Trailer angle is set to zero by both contructors. Use the trailer function 
/// setCurrentAngle_v to change current trailer angle.
///
/// The trailer angle is defined as the angle between the car longitudinal
/// axis and the trailer longitudinal axis, increasing in the direction of 
/// the front passenger seat.
///
/// Trailer coordinate system is defined as follows: 
/// The origin of the coordinates is at the middle of the trailer axle, i.e. 
/// at (HitchX+TrailerLength,0) of the car coordinate system. X-axis coincides 
/// with the trailer longitudinal axis pointing toward the back-end of the 
/// trailer and Y-axis coincides with the axle of the trailer pointing toward 
/// the front passenger seat, i.e. the orientation of the axes is the same		
/// as that of the car coordinate system. 
///
/// The class includes functions allowing to change current trailer coordinates, 
/// current trailer angle or both as well as to move a trailer to a predicted 
/// position computed. Setting trailer current angle only rotates the trailer 
/// about the hitch with no impact on a car, but changing its current coordinates 
/// or current position also automatically relocates a car.
/// 	
///	Use the function updateTrailerPositionWrtCar_v to update trailer position 
/// after the current car position data have been changed or the function 
/// moveToPredictedPosition_v if the car has been moved to a predicted position.
/// 
/// Sequential trailer movement for a sequence of steering inputs can be ensured 
/// by moving a trailer to a predicted position after each computation 
/// computePredictedPosition_v(). 
///  
/// Note that it is actually sufficient to use the functions computePredictedPosition_v() 
/// and moveToPredictedPosition_v() of a trailer object only as they perform
/// corresponding computations both for the trailer and a corresponding car
/// the trailer is attached to.
///
/// @par Example usage:
/// @code{.cpp}
/// @endcode
/// 
/// Trajectory computation is based on the system of differential equations 
/// stated in the article "Flatness, motion planning and trailer systems",
/// P. Rouchon, M. Fliess, J. Levine, Ph. Martin.
///
//  --------------------------------------------------------------------------

template<typename T>
class TrailerModel : public IMovableObject<T>
{
public:	
  /// @brief Precompute static const members 
	void Init();

  /// @brief Class TrailerModel constructor for given trailer configuration.
	TrailerModel(T i_TrailerLength_x, 
				 T i_hitchX_x,	
                 CarModel<T>& i_car_ro);
	
  /// @brief Class TrailerModel constructor without trailer configuration data.
	TrailerModel(CarModel<T>& i_car_ro);
	
  /// @brief Default class TrailerModel deconstructor
	~TrailerModel();

  /// @brief Set trailer configuration data.
	void setTrailerConfig_v(T i_TrailerLength_x, 
							T i_hitchX_x);

  /// @brief Computes predicted position of a trailer for given path (car 
  /// trajectory) length and car steering angle.
	bool computePredictedPosition_v(T i_SteeringAngle, 
									T i_PathLength);
	
  /// @brief Move a trailer to a computed predicted position.
	void moveToPredictedPosition_v();

  /// @brief Compute angle of no return for a trailer with respect to a car it is attached to.
	bool noReturnAngle_v(T i_carMaxSteeringAngle_x,
						 T& o_noReturnAngle_x);

  /// @brief Return current position of a trailer
	Position_s<T> getCurrentPosition_v();

  /// @brief Return predicted position of a trailer
	Position_s<T> getPredictedPosition_v();

  /// @brief Set current trailer coordinates.
	void setCurrentCoordinates_v(mecl::core::Point2D<T> i_CurrentCoordinates);
  
  /// @brief Set current trailer angle.
	void setCurrentAngle_v(T i_CurrentAngle);
										
  /// @brief Set both current trailer coordinates and current trailer angle
	void setCurrentPosition_v(Position_s<T> i_CurrentPosition_s);

  /// @brief Update trailer position after a car has been moved 
	void updateTrailerPositionWrtCar_v();

  /// @brief Compute global current coordinates of a trailer point given in 
  /// the local trailer CS					
	void localCSToGlobalCSCurrent_v
						(mecl::core::Point2D<T>    i_PointInLocalCS,
						 mecl::core::Point2D<T>&   o_PointCurrectPosInGlobalCS);
	
  /// @brief Compute global predicted coordinates of a trailer point given 
  /// in the local trailer CS
	void localCSToGlobalCSPredicted_v
						(mecl::core::Point2D<T>    i_PointInLocalCS,
						 mecl::core::Point2D<T>&   o_PointPredictedPosInGlobalCS);
	
  /// @brief Outputs current and predicted position data on the console.
	void print_v();

private:
	T trailerLength_x;								///< Trailer length (distance between trailer axle and hitch)
	mecl::core::Point2D<T> hitchPositionInCarCS;	///< Hitch point (i_hitchX_x,0) defined in car CS 
	Position_s<T> trailerCurrentPos_s;				///< Trailer current position (coordinates and angle)
	Position_s<T> trailerPredictedPos_s;			///< Trailer predicted position (coordinates and angle)
	CarModel<T>& car_ro;							///< CarModel reference object

	T piOver2_x;									///< Precomputed pi/2
	T piOver4_x;									///< Precomputed pi/4
	T piTimes2_x;									///< Precomputed 2*pi
};

}; // namespace model
}; // namespace mecl

#include "TrailerModel.hpp"

#endif // __MECL_TRAILER_MODEL_H__
/// @}
/// @}
