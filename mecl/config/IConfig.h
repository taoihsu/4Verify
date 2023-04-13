//--------------------------------------------------------------------------
/// @file IConfig.h
/// @brief Interface class IConfig.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
///
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup config
/// @{

#ifndef MECL_CONFIG_ICONFIG_H_
#define MECL_CONFIG_ICONFIG_H_

#include "core/MeclTypes.h"

namespace mecl
{
namespace config
{
// --------------------------------------------------------------------------
/// @class IConfig
/// @brief Interface class IConfig.
///
// --------------------------------------------------------------------------

class IConfig
{
public:

  /// Virtual destructor
  virtual ~IConfig(void) {};

  // --------------------------------------------------------------------------
  /// @brief Returns true if mecl object was properly configured
  /// @return true if instance is configured, otherwise false
  // --------------------------------------------------------------------------
	virtual bool_t isConfigured_b(void) const = 0;
};

} // namespace config
} // namespace mecl

#endif // MECL_CONFIG_ICONFIG_H_
/// @}
/// @}
