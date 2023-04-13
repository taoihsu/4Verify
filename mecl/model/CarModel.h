// --------------------------------------------------------------------------
/// @file CarModel.h
/// @brief Contains the template class CarModel 
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Jelena Zuravlova (jelena.zuravlova@magna.com)
///
// --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef __MECL_CAR_MODEL_H__
#define __MECL_CAR_MODEL_H__

#include <math.h>
#include "core/MeclTypes.h"
#include "core/MeclLogging.h"
#include "math/Math.h"
#include "core/Point.h"
#include "IMovableObject.h"

namespace mecl{
namespace model{

template<typename T>
class TrailerModel;

// --------------------------------------------------------------------------
/// @class CarModel
/// @brief Template class CarModel.
///
/// Template class CarModel including predicted position computation for given
/// car path (trajectory) length and steering angle for a car or an arbitrary 
/// point described in the car coordinate system. 
/// 
/// A car can be defined either by wheelbase length (distance between the 
/// front axle and rear axle) and start position or by wheelbase length only.
/// The later contructor sets the car in default initial current position 
/// with xy-coordinates and car angle equal to zero. For arbitrary initial 
/// current position both wheelbase length and start position have to be 
/// submitted to the former constructor.
/// 
/// Car coordinate system is defined as follows: 
/// The origin of the coordinates is at the (middle of the) front axle.
/// X-axis coincides with the car longitudinal axis pointing toward the rear 
/// bodywork of the car. Y-axis coincides with the front axle of the car   
/// pointing toward the front passenger seat.
///
/// The car angle (frontAxleCurrentPos_s.angle and frontAxlePredictedPos_s.angle)
/// is defined as the angle between the car longitudinal axis and the X-axis, 
/// increasing in the direction of the driver seat.
///
/// The class includes functions allowing to change current car coordinates, 
/// current car angle or both as well as to move a car to a predicted position
/// computed. Use the class TrailerModel function updateTrailerPositionWrtCar_v
/// to update trailer position after current car position has been changed
/// or the function moveToPredictedPosition_v if the car has been moved to a 
/// predicted position.
/// 
/// Sequential car movement for a sequence of steering inputs can be ensured 
/// by moving a car to a predicted position using the function 
/// moveToPredictedPosition_v after the predicted position has been computed.
/// Note that a car should be moved to a predicted position only after the 
/// predicted position has been computed for a trailer (if any) too.
/// 
/// Note that, in case you have both a car object and a trailer object, it is 
/// actually sufficient to use the functions computePredictedPosition_v() 
/// and moveToPredictedPosition_v() of the trailer object only as they perform
/// corresponding computations both for a trailer and a corresponding car
/// the trailer is attached to.
///
/// @par Example usage:x
/// @code{.cpp}
/// @endcode
/// 
/// Trajectory computation is based on the system of differential equations 
/// stated in the article "Flatness, motion planning and trailer systems",
/// P. Rouchon, M. Fliess, J. Levine, Ph. Martin.
/// 
// --------------------------------------------------------------------------
template<typename T>
class CarModel : public IMovableObject<T>
{
public:
  /// @brief Precompute static const members 
	void Init();

  /// @brief Class CarModel constructor for default current position ((x,y,angle)=(0,0,0)).
	CarModel(T i_WheelBase_x);
	
  /// @brief Class CarModel constructor for custom current position.
	CarModel(Position_s<T> i_FrontAxleStartPos_s, 
			 T i_WheelBase_x);
			 
  /// @brief Class CarModel constructor without car configuration data.
	CarModel();
	
  /// @brief Default class CarModel destructor
	~CarModel();

  /// @brief Set trailer configuration data.
	void setCarConfig_v(T i_WheelBase_x);

  /// @brief Computes predicted position of a car for given path (car trajectory) 
  /// length and steering angle.
	bool computePredictedPosition_v(T i_SteeringAngle, 
									T i_PathLength);
	
  /// @brief Move a car to a computed predicted position
	void moveToPredictedPosition_v();
  
  /// @brief Return current position of a car
	Position_s<T> getCurrentPosition_v();

  /// @brief Return predicted position of a car
	Position_s<T> getPredictedPosition_v();

  /// @brief Set current car coordinates.
	void setCurrentCoordinates_v(mecl::core::Point2D<T> i_CurrentCoordinates);

  /// @brief Set current car angle.
	void setCurrentAngle_v(T i_CurrentAngle);
		
  /// @brief Set both current car coordinates and current car angle.
	void setCurrentPosition_v(Position_s<T> i_CurrentPosition_s);

  /// @brief Compute global current coordinates of a car point given in the local car CS
	void localCSToGlobalCSCurrent_v
						(mecl::core::Point2D<T>    i_PointInLocalCS,
					     mecl::core::Point2D<T>&   o_PointCurrectPosInGlobalCS);

  /// @brief Compute global predicted coordinates of a car point given in the local car CS
	void localCSToGlobalCSPredicted_v
						(mecl::core::Point2D<T>    i_PointInLocalCS,
						 mecl::core::Point2D<T>&   o_PointPredictedPosInGlobalCS);

  /// @brief Outputs current and predicted position data on the console.
	void print_v();

private:
  T wheelBase_x;									///< Length of wheel base (distance between front and rear axles).
  Position_s<T> frontAxleCurrentPos_s;				///< Current position (coordinates and angle) of front axle.
  Position_s<T> frontAxlePredictedPos_s;			///< Predicted position (coordinates and angle) of front axle.
  mecl::core::Point2D<T> rearAxleCurrentPos;		///< Current position (coordinates) of rear axle.
  mecl::core::Point2D<T> rearAxlePredictedPos;		///< Predicted position of rear axle.
 
  T piOver2_x;										///< Precomputed pi/2

  /// Allow access to private and protected members of \ref TrailerModel class
  friend class TrailerModel<T>;
};

}; // namespace model
}; // namespace mecl

#include "CarModel.hpp"

#endif // __MECL_CAR_MODEL_H__
/// @}
/// @}
