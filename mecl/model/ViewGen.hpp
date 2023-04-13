//--------------------------------------------------------------------------
/// @file ViewGen.hpp
/// @brief Implementation of non-specialized class template methods of ViewGen.

/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
/// @date 12/18/2015
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{


#ifndef MODEL_VIEWGEN_HPP_
#define MODEL_VIEWGEN_HPP_

#include "ViewGen.h"

namespace mecl {
namespace model {

// -----------------------------------------------------------------------------
/// @brief Default constructor
///
/// Default constructor intitializes class private member to its default values
///
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indicating lookup table type
/// @tparam DT  Type of decoded lookup table data
/// @tparam ET  Type of encoded lookup table data
/// @tparam CT  Base type of camera model used
// -----------------------------------------------------------------------------
template <ViewGenType_e VGT,      // Type of the view generator
          LookupTableType_e LTT,  // Type of lookup table
          typename DT,            // Type of decoded lut entry
          typename ET,            // Type of encoded lut entry
          typename IT,            // Type of value encoded by index
          typename CT>            // Base type of camera model used
ViewGen<VGT, LTT, DT, ET, IT, CT>::ViewGen(void)
: lut_s()
, lutConfigured_b(false)
, generated_b(false)
{}

// -----------------------------------------------------------------------------
/// @brief Constructor with lookup table configuration data input
///
/// Set lookup table configuration according to input parameter \i_Config_rs
///
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indicating lookup table type
/// @tparam DT  Type of decoded lookup table data
/// @tparam ET  Type of encoded lookup table data
/// @tparam CT  Base type of camera model used
/// @param[in] i_Config_rs Reference to lookup table configuration data
// -----------------------------------------------------------------------------
template <ViewGenType_e VGT,      // Type of the view generator
          LookupTableType_e LTT,  // Type of lookup table
          typename DT,            // Type of decoded lut entry
          typename ET,            // Type of encoded lut entry
          typename IT,            // Type of value encoded by index
          typename CT>            // Base type of camera model used
ViewGen<VGT, LTT, DT, ET, IT, CT>::ViewGen(const typename LookupTable<VGT, LTT, ET,CT>::Config_s& i_Config_rs)
: lut_s()
, lutConfigured_b(true)
, generated_b(false)
{
  this->lut_s.config_s = i_Config_rs;
}
// -----------------------------------------------------------------------------
/// @brief Get lookup table configuration data
///
/// Returns the lookup table configuration data currently set
///
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indicating lookup table type
/// @tparam DT  Type of decoded lookup table data
/// @tparam ET  Type of encoded lookup table data
/// @tparam CT  Base type of camera model used
/// @return Lookup table configuration data currently set
// -----------------------------------------------------------------------------
// get lookup table config
template <ViewGenType_e VGT,              // Type of the view generator
          LookupTableType_e LTT,          // Type of lookup table
          typename DT,                    // Type of decoded lut entry
          typename ET,                    // Type of encoded lut entry
          typename IT,                    // Type of value encoded by index
          typename CT>                    // Base type of camera model used
const LookupTableConfig_s<VGT, LTT,DT,CT>& // Return type
ViewGen<VGT, LTT, DT, ET, IT, CT>::getLookupTableConfig_rs(void) const
{
    return this->lut_s.config_s;
}
// -----------------------------------------------------------------------------
/// @brief Get lookup table info data
///
/// Returns the valid lookup table info data, if lookup table was generated.
/// A struct with default values is returned, if lookup table has not been
/// generated yet.
///
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indicating lookup table type
/// @tparam DT  Type of decoded lookup table data
/// @tparam ET  Type of encoded lookup table data
/// @tparam CT  Base type of camera model used
/// @return Lookup table configuration data, valid only if lookup table has already been generated.
// -----------------------------------------------------------------------------
// get lookup table info
template <ViewGenType_e VGT,              // Type of the view generator
          LookupTableType_e LTT,          // Type of lookup table
          typename DT,                    // Type of decoded lut entry
          typename ET,                    // Type of encoded lut entry
          typename IT,                    // Type of value encoded by index
          typename CT>                    // Base type of camera model used
const LookupTableInfo_s<VGT, LTT>&        // Return type
ViewGen<VGT, LTT, DT, ET, IT, CT>::getLookupTableInfo_rs(void) const
{
  return this->lut_s.info_s;
}


// -----------------------------------------------------------------------------
/// @brief Get decoded lookup table entry with index
///
/// Returns decoded lookup table entry with given index \i_Index_u32
///
/// Default constructor intitializes class private member to its default values
///
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indicating lookup table type
/// @tparam DT  Type of decoded lookup table data
/// @tparam ET  Type of encoded lookup table data
/// @tparam CT  Base type of camera model used
/// @param[in]  i_Index_u32 Index of lookup table entry
/// @return     Decoded data of lookup table entry
// -----------------------------------------------------------------------------
template <ViewGenType_e VGT,              // Type of the view generator
          LookupTableType_e LTT,          // Type of lookup table
          typename DT,                    // Type of decoded lut entry
          typename ET,                    // Type of encoded lut entry
          typename IT,                    // Type of value encoded by index
          typename CT>                    // Base type of camera model used
DT                                        // Return type
ViewGen<VGT, LTT, DT, ET, IT, CT>::getDecoded_x(uint32_t i_Index_u32) const
{
  return this->decode_x( this->getEncoded_x(i_Index_u32) );
};

// -----------------------------------------------------------------------------
/// @brief Set lookup table configuration parameters
///
/// Set/Reset view generator parameters for generating an associated lookup table.
///
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indicating lookup table type
/// @tparam DT  Type of decoded lookup table data
/// @tparam ET  Type of encoded lookup table data
/// @tparam CT  Base type of camera model used
/// @param[in]  i_Config_rs Reference to struct of parameters to set
// -----------------------------------------------------------------------------
template <ViewGenType_e VGT,      // Type of the view generator
          LookupTableType_e LTT,  // Type of lookup table
          typename DT,            // Type of decoded lut entry
          typename ET,            // Type of encoded lut entry
          typename IT,            // Type of value encoded by index
          typename CT>            // Base type of camera model used
void ViewGen<VGT, LTT, DT, ET, IT, CT>::setLookupTableConfig_v(const LookupTableConfig_s<VGT, LTT, DT,CT>& i_LutConfig_rs)
{
   this->lut_s.config_s = i_LutConfig_rs;
   memcpy(&this->lut_s.config_s, i_LutConfig_rs, sizeof(LookupTableConfig_s<VGT, LTT, DT,CT>));
   this->lutConfigured_b = true;
   this->generated_b = false;
   return;
}

// -----------------------------------------------------------------------------
/// @brief Encode data by codec used by this view generator
///
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indicating lookup table type
/// @tparam DT  Type of decoded lookup table data
/// @tparam ET  Type of encoded lookup table data
/// @tparam CT  Base type of camera model used
/// @param[in]  i_Val_rx Data to encode (same type as of decoded lookup table entry)
/// @return Encoded data (same type as of encoded lookup table entry)
// -----------------------------------------------------------------------------
template <ViewGenType_e VGT,      // Type of the view generator
          LookupTableType_e LTT,  // Type of lookup table
          typename DT,            // Type of decoded lut entry
          typename ET,            // Type of encoded lut entry
          typename IT,            // Type of value encoded by index
          typename CT>            // Base type of camera model used
ET ViewGen<VGT, LTT, DT, ET, IT, CT>::encode_x(const DT& i_Val_rx) const
{
  ViewGenCodec<VGT, DT, ET> v_Codec_x;
  ET v_Encoded_x;
  v_Codec_x.encode_v(i_Val_rx, v_Encoded_x);
  return v_Encoded_x;
}

// -----------------------------------------------------------------------------
/// @brief Decode data by codec used by this view generator
///
/// @tparam VGT enum value indicating view generator type
/// @tparam LTT enum value indicating lookup table type
/// @tparam DT  Type of decoded lookup table data
/// @tparam ET  Type of encoded lookup table data
/// @tparam CT  Base type of camera model used
/// @param[in]  i_Val_rx Data to decode (same type of encoded lookup table entry)
/// @return Decoded data (same type as of encoded lookup table entry)
// -----------------------------------------------------------------------------
template <ViewGenType_e VGT,      // Type of the view generator
          LookupTableType_e LTT,  // Type of lookup table
          typename DT,            // Type of decoded lut entry
          typename ET,            // Type of encoded lut entry
          typename IT,            // Type of value encoded by index
          typename CT>            // Base type of camera model used
DT ViewGen<VGT, LTT, DT, ET, IT, CT>::decode_x(const ET& i_Val_rx) const
{
  ViewGenCodec<VGT, DT, ET> v_Codec_x;
  DT v_Decoded_x;
  v_Codec_x.decode_v(i_Val_rx, v_Decoded_x);
  return v_Decoded_x;
}


} // namespace model
} // namespace mecl

#endif

/// @}
/// @}

