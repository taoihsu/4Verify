
// --------------------------------------------------------------------------
/// @file Singleton.h
/// @brief Generic singleton container
///
/// Singleton<C> defines a generic container for a class of type C. The container
/// contains exactly one instance of class C and cannot be instantiated, copied or
/// assigned to another container. A reference to the instance inside the container
/// is returned by public member Singleton<T>::getInstance_rx(). To test if an instance
/// of C is contained by the container Singleton<T>::isSingleton_b() can be called.
///
// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
/// @date 04/23/2015
// --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup core
/// @{

#ifndef SRC_CORE_SINGLETON_H_
#define SRC_CORE_SINGLETON_H_

#include "MeclTypes.h"

namespace mecl
{

namespace core
{

template <typename C>
class Singleton
{
public:

  // --------------------------------------------------------------------------
  /// @brief Get reference
  /// 
  /// Get reference to instance contained by singleton container
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Singleton_getInstance_rx
  ///
  /// @return     Singleton instance
  // --------------------------------------------------------------------------
  static C& getInstance_rx()
  {
    static C s_Instance_x;
    return s_Instance_x;
  }

  // --------------------------------------------------------------------------
  /// @brief Check for singleton
  /// 
  /// Check if specific instance of class C is contained by singleton container
  ///
  /// @par Example usage:
  /// @snippet CoreTester.cpp Singleton_isSingleton_b
  ///
  /// @param[in]  c Instance to be tested
  /// @return     Bool, true if singleton, false otherwise
  // --------------------------------------------------------------------------
  static bool_t isSingleton_b(const C& c)
  {
    return &( getInstance_rx() ) == &c;
  }

private:
  //! prevent default constructor
  Singleton() {};

  //! prevent copy constructor
  Singleton(const C& i_CInstance_x) {};

  //! prevent assignment operator with non singleton class
  Singleton operator= (const C& i_CInstance_x) const
  {
    return Singleton(i_CInstance_x);
  }

  //! prevent assignment operator
  Singleton operator= (const Singleton<C>& i_SInstance_x) const
  {
    return i_SInstance_x;
  }
};

} // end of namespace core
} // end of namespace melc


#endif /* SRC_CORE_SINGLETON_H_ */

/// @}
/// @}
