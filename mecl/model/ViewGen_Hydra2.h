//----------------------------------------------------------------------------
/// @file ViewGen_Hydra2.h
/// @brief Type definitions for view hydra2 viewgen2 generator functionality.
/// The header file contains the type definitions of implemented classes defined
/// by template specialization in the referred file ViewGen_Hydra2.hpp.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
/// @date 12/15/2015
//  --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef MECL_MODEL_VIEWGEN_HYDRA2_H_
#define MECL_MODEL_VIEWGEN_HYDRA2_H_

#include "ViewGen.h"
#include "ViewGenCodec_Hydra2.h"

namespace mecl {
namespace model {

/// Codec type definition for Hydra2 compliant encoding/decoding of 3D cube coordinates
typedef class ViewGenCodec<e_Hydra2RadialLens, core::Point3D<float32_t>, uint32_t>    Vgc_Hydra2_P3f32_u32;

/// Codec type definition for Hydra2 compliant encoding/decoding of 2D point locations
typedef class ViewGenCodec<e_Hydra2RadialLens, core::Point2D<float32_t>, uint32_t>    Vgc_Hydra2_P2f32_u32;

// ----------------------------------------------------------------------------
/// @struct LookupTableConfig_s<e_Hydra2RadialLens, e_LensUndistortion, uint32_t, float32_t>
/// @brief Struct template specialization for configuration parameters of associated
/// view generator.
///
/// Structure defines the configuration parameters for initializing a Hydra2 compliant
/// view generator for an undistortion model. The associated view generator generates
/// a lookup table mapping metric 2D image points to metric 3D cube coordinates
/// determined by a lens undistortion model given by the camera model configured.
// ----------------------------------------------------------------------------
template<>
struct LookupTableConfig_s<e_Hydra2RadialLens, e_LensUndistortion, uint32_t, float32_t>
{
  uint32_t*       addr_pu32;        //< start address of (external) memory where lookup table is located
  uint32_t        maxSize_u32;      //< maximum size of (external) memory where lookup table is loocated
  uint32_t        stepSize_u32;     //< size of the support point grid (square) on 2D image area in pixel (e.g. 16)
  uint32_t        resHoriz_u32;     //< maximum size of a (visible) horizontal in pixel
  uint32_t        resVert_u32;      //< maximum size of a (visible) vertical line in pixel
  bool_t          calcError_b;      //< flag indicating whether error should be measured and updated in lookup table info struct

  Camera<float32_t>* camera_po;     //< Pointer reference to camera model on behalf of which the lookup table is computed
};

// ----------------------------------------------------------------------------
/// @struct LookupTableInfo_s<e_Hydra2RadialLens, e_LensUndistortion>
/// @brief Struct template specialization for lookup table info struct of associated view generator.
///
/// Structure defines lookup table info data given back after initialization of
/// associated Hydra 2 lens undistortion view generator.
/// The associated view generator generates a lookup table mapping metric to
/// 2D image points to metric 3D cube coordinates determined by a
/// lens undistortion model given by the camera model configured.
// ----------------------------------------------------------------------------
template<>
struct LookupTableInfo_s<e_Hydra2RadialLens, e_LensUndistortion>
{
  uint32_t* startAddr_pu32;         //< start address of lookup table within external memory
  uint32_t* endAddr_pu32;           //< end address of lookup table within external memory
  core::Point2D<float32_t>::Config_s pppCfg_f32; //< config of metric principal point 2d position
  uint32_t numSupportPoints_u32;    //< number of support points in a line
  uint32_t numSupportLines_u32;     //< number of support lines

  float32_t avgError_f32;           //< average error undistorted point <-> decoded from lut
  float32_t maxError_f32;           //< maximum error undistorted point <-> decoded from lut
  uint32_t  maxErrorIdx_u32;        //< index of support point with maximal error
};

// ----------------------------------------------------------------------------
/// @struct LookupTableConfig_s<e_Hydra2RadialLens, e_Distortion, uint32_t, float32_t>
/// @brief Struct template specialization for configuration parameters of associated
/// view generator.
///
/// Structure defines the configuration parameters for initializing a Hydra2 compliant
/// view generator for a distortion model. The associated view generator generates
/// a lookup table mapping metric 3D cube coordinates to metric 2D image coordinates
/// determined by a lens undistortion model given by the camera model configured.
// ----------------------------------------------------------------------------
template<>
struct LookupTableConfig_s<e_Hydra2RadialLens, e_LensDistortion, uint32_t, float32_t>
{
  uint32_t*  addr_pu32;       //< start address of (external) memory where lookup table is located
  uint32_t   maxSize_u32;     //< maximum size of (external) memory where lookup table is located
  uint32_t   spread_u32;      //< half the number of support points per cube face in horizontal/vertical direction
  bool_t     applySensor_b;   //< set true, if entries should be in pixel coordinates, otherwise metric
  bool_t     calcError_b;     //< flag indicating whether errors should be measured and updated in lookup table info struct

  Camera<float32_t>* camera_po;   //< pointer reference to camera model on behalf of which the lookup table is computed
};

// ----------------------------------------------------------------------------
/// @struct LookupTableInfo_s<e_Hydra2RadialLens, e_LensDistortion>
/// @brief Struct template specialization for lookup table info struct of associated view generator.
///
/// Structure defines lookup table info data given back after initialization of
/// associated Hydra 2 lens distortion view generator.
/// The associated view generator generates a lookup table mapping metric 3D cube
/// coordinates to metric 2D image coordinates determined by a lens distortion
/// given by the camera model configured.
// ----------------------------------------------------------------------------
template<>
struct LookupTableInfo_s<e_Hydra2RadialLens, e_LensDistortion>
{
  uint32_t* frontFaceStartAddr_pu32;    //< start address of front face lookup table = lookup table start adress
  uint32_t* sideFaceStartAddr_pu32;     //< start address of side face lookup table
  uint32_t* verticalFaceStartAddr_pu32; //< start adresss of vertical face lookup table
  float32_t stepWidth_f32;              //< length of an element of regular grid on unit cube in [mm]
  uint32_t* endAddr_pu32;               //< end address of lookup table = end address of vertical lookup table

  float32_t minHoriz_f32;                //< minimum horizontal coordinate
  float32_t maxHoriz_f32;                //< maximum horizontal coordinate
  float32_t minVert_f32;                 //< minimum vertical coordinate
  float32_t maxVert_f32;                 //< maximum vertical coordinate

  float32_t avgError_f32;               //< overall average error
  float32_t avgErrorFront_f32;          //< average error front face
  float32_t avgErrorSide_f32;           //< average error side faces
  float32_t avgErrorVertical_f32;       //< average error vertical faces
  float32_t avgErrorRear_f32;           //< average error rear
  float32_t maxError_f32;               //< overall maximum error
  uint32_t  maxErrorIdx_u32;            //< lookup table index of entry with maximum error
};

// ----------------------------------------------------------------------------
// Typedefs (specialized templates)

/// Lookup table configuration type definition for Hydra2 compliant view generator for lens distortion
typedef struct LookupTableConfig_s<e_Hydra2RadialLens, e_LensDistortion, uint32_t, float32_t>    LutCfg_Hydra2_Dist_u32;

/// Lookup table configuration type definition for Hydra2 compliant view generator for lens undistortion
typedef struct LookupTableConfig_s<e_Hydra2RadialLens, e_LensUndistortion, uint32_t, float32_t>  LutCfg_Hydra2_Undist_u32;

/// Lookup table info type definition for Hydra2 compliant view generator for lens distortion
typedef struct LookupTableInfo_s<e_Hydra2RadialLens, e_LensDistortion>                LutInfo_Hydra2_Dist_u32;

/// Lookup table info type defintion for Hydra2 compliant view generator for lens undistortion
typedef struct LookupTableInfo_s<e_Hydra2RadialLens, e_LensUndistortion>              LutInfo_Hydra2_Undist_u32;

/// Lookup table type definition for Hydra2 compliant view generator for lens distortion
typedef struct LookupTable<e_Hydra2RadialLens, e_LensDistortion, uint32_t, float32_t>          Lut_Hydra2_Dist_u32;

/// Lookup table type defintion for Hydra2 compliant view generator for lens undistortion
typedef struct LookupTable<e_Hydra2RadialLens, e_LensUndistortion, uint32_t, float32_t>        Lut_Hydra2_Undist_u32;

/// View generator type definition for Hydra2 compliant undistortion view generator
typedef class ViewGen< e_Hydra2RadialLens, e_LensUndistortion,
                       core::Point3D<float32_t>,
                       uint32_t,
                       core::Point2D<float32_t>, float32_t >                Vg_Hydra2_Undist_p3f32_u32_p2f32;

/// View generator type defintion for Hydra2 compliant distortion view generator
typedef class ViewGen< e_Hydra2RadialLens, e_LensDistortion,
                       core::Point2D<float32_t>,
                       uint32_t,
                       core::Point3D<float32_t>, float32_t >                Vg_Hydra2_Dist_p2f32_u32_p3f32;

}
}


#endif /* MECL_MODEL_VIEWGEN_HYDRA2_H_ */

/// @}
/// @}
