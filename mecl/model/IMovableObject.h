//--------------------------------------------------------------------------
/// @file IMovableObject.h
/// @brief Contains the interface for the classes CarModel and TrailerModel.
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

#ifndef __MECL_I_MOVABLE_OBJECT_H__
#define __MECL_I_MOVABLE_OBJECT_H__

#include "core/MeclTypes.h"
#include "core/matrix.h"
#include "core/point.h"

namespace mecl{
namespace model{

// --------------------------------------------------------------------------
/// @struct Position_s
/// @brief Position and orientation information of an object.
/// 
/// Contains coordinates of an object stored in mecl::core::Point2D<T> 
/// and an angle defined in the description of the relevant class.
///
// --------------------------------------------------------------------------
template<typename T> 
struct Position_s
{

  /// Default constructor
	Position_s()
		: xy()
		, angle_x(mecl::math::constants<T>::zero_x())
	{}

	/// Constructor
	Position_s(mecl::core::Point2D<T> i_xy,
				T i_angle_x)
		: xy(i_xy)
		, angle_x(i_angle_x)
	{}

  mecl::core::Point2D<T> xy;  ///< Point location
	T angle_x;                  ///< Angle
};

//--------------------------------------------------------------------------
/// @class IMovableObject
/// @brief Interface for the classes CarModel and TrailerModel.
///
//  --------------------------------------------------------------------------
template<typename T>
class IMovableObject
{
public:


  /// Default class constructor
	IMovableObject(void)
  {
    // empty
  }

  /// Default class deconstructor
	virtual ~IMovableObject(void)
  {
    // empty
  }
	  
  /// Computes predicted position of a vehicle for given path (car 
  /// trajectory) length and car steering angle.
	virtual bool computePredictedPosition_v(T i_pathLength, 
											T i_steeringAngle) = 0; 
	
	/// Move a vehicle to a computed predicted position
	virtual void moveToPredictedPosition_v() = 0;
	
  /// @brief Return current position of a vehicle
	virtual Position_s<T> getCurrentPosition_v() = 0;
	
  /// @brief Return predicted position of a vehicle
	virtual Position_s<T> getPredictedPosition_v() = 0;

  /// @brief Set both current vehicle coordinates and current vehicle angle
	virtual void setCurrentPosition_v(Position_s<T> i_CurrentPosition_s) = 0;
  
  /// @brief Set current vehicle coordinates.	
	virtual void setCurrentCoordinates_v(mecl::core::Point2D<T> i_CurrentCoordinates) = 0;

  /// @brief Set current vehicle angle.	
	virtual void setCurrentAngle_v(T i_CurrentAngle) = 0;
	
  /// @brief Compute global predicted coordinates of a vehicle point given 
  /// in the local vehicle CS
	virtual void localCSToGlobalCSPredicted_v
						(mecl::core::Point2D<T>    i_PointInLocalCS,
						 mecl::core::Point2D<T>&   o_PointNewPosInGlobalCS) = 0;
	
  /// @brief Compute global current coordinates of a vehicle point given in 
  /// the local vehicle CS	
	virtual	void localCSToGlobalCSCurrent_v
						(mecl::core::Point2D<T>    i_PointInLocalCS,
						 mecl::core::Point2D<T>&   o_PointCurrectPosInGlobalCS) = 0;

  /// Outputs start and end position data on the console.
	virtual void print_v() = 0;

};

}; // namespace model
}; // namespace mecl

#endif // __MECL_I_MOVABLE_OBJECT_H__

/// @}
/// @}
