//--------------------------------------------------------------------------
/// @file ViewGen.h
/// @brief Type definitions, interfaces and classes for view generator functionality.
/// The header file contains basic interfaces and declarations for classes
/// implementing the interface to a view generator.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
/// @date 11/18/2015
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{


#ifndef MODEL_VIEWGEN_H_
#define MODEL_VIEWGEN_H_

#include "core/MeclAssert.h"
#include "config/IConfig.h"
//#include "Camera.h"

namespace mecl {

namespace model {

// ----------------------------------------------------------------------------
/// @struct LookupTableInfo_s
/// @brief Struct template stub for lookup table info structure (POD)
///
/// Structure defines lookup table info data given back after initialization of ViewGen.
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indication Lookup table type
///
/// @remark Declaration is stub (no definition/implementation).
/// @attention Must be defined in template specialization.
// ----------------------------------------------------------------------------
template<ViewGenType_e VGT, LookupTableType_e LTT>
struct LookupTableInfo_s;

// ----------------------------------------------------------------------------
/// @struct LookupTableConfig_s
/// @brief Struct template stub for lookup table config structure (POD)
///
/// Structure defines configuration parameters for generating a lookup table.
///
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indicating lookup table type
/// @tparam DT  Lookup table entry type
/// @tparam CT  Base type of camera model on behalf of which lookup table is generated
///
/// @remark Declaration is stub (no definition/implementation).
/// @attention Must be defined in template specialization.
// ----------------------------------------------------------------------------
template<ViewGenType_e VGT, LookupTableType_e LTT, typename DT, typename CT>
struct LookupTableConfig_s;

// ----------------------------------------------------------------------------
/// @struct LookupTable
/// @brief Struct template for lookup table structure
///
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indicating lookup table type
/// @tparam DT  Lookup table entry type
/// @tparam CT  Base type of camera model on behalf of which lookup table is generated
///
/// Structure defines lookup table data contained in ViewGen.
// ----------------------------------------------------------------------------
template<ViewGenType_e VGT, LookupTableType_e LTT, typename DT, typename CT>
struct LookupTable
{
  typedef LookupTableConfig_s<VGT, LTT, DT, CT> Config_s;
  typedef LookupTableInfo_s<VGT, LTT> Info_s;

  Config_s config_s; //< configuration data
  Info_s   info_s;   //< info data
};

// ----------------------------------------------------------------------------
/// @class IViewGenCodec
/// @brief Interface template for codecs used by view generators
///
/// @tparam DT  Type of decoded data
/// @tparam ET  Type of encoded data
// ----------------------------------------------------------------------------
template <typename DT = float32_t, typename ET = uint32_t>
class IViewGenCodec
{
public:

  /// Virtual destructor
  virtual ~IViewGenCodec(void) {};

  /// Encode data
  virtual void encode_v( const DT& i_Pos_rx, ET& o_Val_rx ) const = 0;

  /// Decode data
  virtual void decode_v( const ET& i_Val_rx, DT& o_Pos_rx ) const = 0;

};

// ----------------------------------------------------------------------------
/// @class ViewGenCodec
/// @brief Codec class template for view generators
///
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indicating lookup table type
/// @tparam DT  Type of decoded data
/// @tparam ET  Type of encoded data
///
/// @remark Method Members are stub.
/// @attention Methods must be implemented in template specializations.
// ----------------------------------------------------------------------------
template <ViewGenType_e VGT, typename DT = float32_t, typename ET = uint32_t>
class ViewGenCodec : public IViewGenCodec<DT,ET>
{
public:

  /// Virtual destructor
  virtual ~ViewGenCodec(void) {};

  /// Encode data (stub)
  virtual void encode_v( const DT& i_Pos_rx, ET& o_Val_rx ) const;

  /// Decode data (stub)
  virtual void decode_v( const ET& i_Val_rx, DT& o_Pos_rx ) const;

};

// ----------------------------------------------------------------------------
/// @class IViewGen
/// @brief Interface template for view generators
// ----------------------------------------------------------------------------
class IViewGen : config::IConfig
{
public:

  /// virtual destructor
  virtual ~IViewGen(void) {};

  /// initialize  lookup table
  virtual void init_v(void) = 0;

};

// ----------------------------------------------------------------------------
/// @class ViewGen
/// @brief View generator class template
///
/// Class template generates lookup tables for ViewGen defined by template
/// parameters.
///
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indicating lookup table type
/// @tparam DT  Type of decoded lookup table data
/// @tparam ET  Type of encoded lookup table data
/// @tparam CT  Base type of camera model used
///
/// @remark Method members are  partially stub.
/// @attention Methods declared stub must be implemented in template specializations.
// ----------------------------------------------------------------------------
template <ViewGenType_e VGT,      // Type of the view generator
          LookupTableType_e LTT,  // Type of lookup table
          typename DT,            // Type of decoded lut entry
          typename ET,            // Type of encoded lut entry
          typename IT,            // Type of value encoded by index
          typename CT>            // Base type of camera model used
class ViewGen : IViewGen
{

public:

  // --------------------------------------------------------------------------
  // c-tors and destructor of class template

  // default c-tor
  ViewGen(void);

  // c-tor with lookup table configuration
  explicit ViewGen(const typename LookupTable<VGT, LTT, ET,CT>::Config_s& i_Config_rs);

  // virtual destructor
  virtual ~ViewGen(void) {};

  // --------------------------------------------------------------------------
  // implemented methods of class template

  /// Get lookup table config
  const LookupTableConfig_s<VGT, LTT,DT,CT>& getLookupTableConfig_rs(void) const;

  /// Get lookup table info
  const LookupTableInfo_s<VGT, LTT>& getLookupTableInfo_rs(void) const;

  /// Encode value to type of encoded lookup table entry
  ET encode_x(const DT& i_Val_rx) const;

  /// Decode value to type of decoded lookup table entry
  DT decode_x(const ET& i_Val_rx) const;

  /// Get decoded data element of lookup table entry with given index
  DT getDecoded_x(const uint32_t i_Index_u32) const;

  /// Set lookup table configuration
  void setLookupTableConfig_v(const LookupTableConfig_s<VGT, LTT, DT,CT>& i_LutConfig_rs);

  // --------------------------------------------------------------------------
  // methods to be implemented in template specializations

  /// Initialization method which generates lookup table (stub)
  virtual void init_v(void);

  /// Method checks if view generator is configured (stub)
  virtual bool_t isConfigured_b(void) const;

  /// Get value of indexed type from value with given index in lookup table (stub)
  IT getIndexedValue_x(const uint32_t i_Index_u32) const;

  /// Get encoded data element of lookup table entry with given index in lookup table (stub)
  ET getEncoded_x(const uint32_t i_Index_u32) const;

private:

  LookupTable<VGT, LTT, ET,CT>  lut_s;  //< lookup table config/info
  bool_t              lutConfigured_b;  //< flag indicating lookup table config status
  bool_t                  generated_b;  //< flag indicating whether lookup table has been generated

};


} // namespace model
} // namespace mecl

#include "ViewGen.hpp"

#endif

/// @}
/// @}
