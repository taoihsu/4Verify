// --------------------------------------------------------------------------
/// @file ModelTypes.h
/// @brief Contains common camera model definitions.
///
/// --------------------------------------------------------------------------
/// @copyright MAGNA Electronics - C O N F I D E N T I A L <br>
/// This document in its entirety is CONFIDENTIAL and may not be disclosed,
/// disseminated or distributed to parties outside MAGNA Electronics
/// without written permission from MAGNA Electronics.
///
/// @author Helmut Zollner (helmut.zollner@magna.com)
///
// --------------------------------------------------------------------------
/// \addtogroup mecl
/// @{
/// \addtogroup model
/// @{

#ifndef MECL_MODEL_MODELTYPES_H_
#define MECL_MODEL_MODELTYPES_H_

#ifdef __cplusplus
namespace mecl
{
namespace model
{
#endif // __cplusplus

// --------------------------------------------------------------------------
/// @enum PreRoll_e 
/// @brief Camera pre-rotation enumeration type
// --------------------------------------------------------------------------
typedef enum PreRoll_e
{
  e_PreRoll0 = 0,     ///<   0 degrees pre-rotation about z-axis ( left camera)
  e_PreRoll90 ,       ///<  90 degrees pre-rotation about z-axis (front camera)
  e_PreRoll180,       ///< 180 degrees pre-rotation about z-axis (right camera)
  e_PreRoll270        ///< 270 degrees pre-rotation about z-axis ( rear camera)
} PreRoll_e; 

// --------------------------------------------------------------------------
/// @enum PitchOffset_e 
/// @brief Camera pitch offset enumeration type
// --------------------------------------------------------------------------
typedef enum PitchOffset_e
{
  e_PitchOffset180 = 180,     ///< 180 degrees pitch offset about x-axis
  e_PitchOffset270 = 270      ///< 270 degrees pitch offset about x-axis
} PitchOffset_e; 


// --------------------------------------------------------------------------
/// @enum SensorType_e
/// @brief Defines type of Sensor (template parameter)
// --------------------------------------------------------------------------
typedef enum
{
  e_Uniform = 0,      // Sensor has constant scaling (psz, psy) across clipping area
  e_VarScaleX = 1,    // Sensor has constant scaling only for x (psz) but not for y
  e_VarScaleY = 2,    // Sensor has constant scaling only for y (psy) bot not for x
  e_NonUniform = 3    // Sensor scaling varies across clipping area
} SensorType_e;

// --------------------------------------------------------------------------
/// @enum ImageOriginPosition_e
/// @brief Defines the (displayed) position of the image origin
typedef enum ImageOriginPosition_e
{
  e_UpperLeft = 0,            ///< origin is in upper left corner (default)
  e_UpperRight,               ///< origin is in upper right corner (image is flipped over y axis)
  e_LowerLeft,                ///< origin is in lower left corner (flipped over x axis)
  e_LowerRight                ///< origin is in lower right corner (flipped over x and y axis)
} ImageOriginPosition_e;

// --------------------------------------------------------------------------
/// @enum RotationType_e
/// @brief Defines type of rotation
typedef enum RotationType_e
{
  e_WorldToCamera = 0, //< aligns points in world coordinate system to camera coordinate system (DIN70000 automotive standard)
  e_CameraToWorld      //< aligns points in camera coordinate system to world coordinate system (CGI definition)
} RotationType_e;


// --------------------------------------------------------------------------
/// @enum ViewGenType_e
/// @brief Defines set of implemented view generators types
typedef enum { e_Hydra2RadialLens, e_Hydra2CylinderLens }
        ViewGenType_e;

// --------------------------------------------------------------------------
/// @enum LookupTableType_e
/// @brief Defines set of lookup table types used by view generators
typedef enum { e_LensDistortion,   e_LensUndistortion, e_RealToSynt, e_SyntToReal }
        LookupTableType_e;

// --------------------------------------------------------------------------
/// @enum Faces_e
/// @brief Defines MSB bit encoding of faces for encoded 3D cube coordinates
typedef enum {   // coded in 2 bit representation (MSB)
                 e_Front    = 0x00000000U,
                 e_Side     = 0x40000000U,
                 e_Vertical = 0x80000000U,
                 e_Rear     = 0xC0000000U,

                 // coded in 3 bit representation (MSB)
                 e_Left     = e_Side,
                 e_Right    = 0x60000000U,
                 e_Top      = e_Vertical,
                 e_Bottom   = 0xA0000000U,

                 // point is cube center (cannot be normalized)
                 e_Origin   = 0x20000000U,

                 // undefinded face
                 e_Undef    = 0xE0000000U }
        Faces_e;

/// @enum SideFaces_e
/// @brief Define set of side cube faces
typedef enum { e_LeftSide = e_Left, e_RightSide = e_Right } SideFaces_e;

/// @enum VerticalFaces_e
/// @brief Define set of vertical cube faces
typedef enum { e_TopSide = e_Top, e_BottomSide = e_Bottom } VerticalFaces_e;


#ifdef __cplusplus
} // namespace model
} // namespace mecl
#endif // __cplusplus

#endif // MECL_MODEL_MODELTYPES_H_
/// @}
/// @}
