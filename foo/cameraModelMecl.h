// ----------------------------------------------------------------------------
// --- Written by Ehsan Parvizi [11-May-2015]
// --- Modified by Ehsan Parvizi [12-May-2015]
// --- Copyright (c) Magna Vectrics (MEVC) 2015
// ----------------------------------------------------------------------------
// --- CameraModelMecl.h - CameraModel related parameter struct/methods
// ----------------------------------------------------------------------------
#ifndef __CAMERAMODEL_MECL_H_
#define __CAMERAMODEL_MECL_H_

#ifndef USE_SVSCM
#include "point_oc.h"
#include "mathOperations.h"
#include "mecl/mecl.h"

// ----------------------------------------------------------------------------
// Suppress the QACPP MISRA warning regarding use of floating point numbers
// PRQA S 3708 EOF

// CameraModelMecl class
namespace camera_model
{
class CameraModelMecl
{
public:
    explicit CameraModelMecl( tscApi::enuCameraID i_ID_t = tscApi::e_TscFrontCam, bool i_IsValid_b = false ):  \
        id_t( i_ID_t ),  \
        isValid_b( i_IsValid_b )        
    {
    }

    ~CameraModelMecl()
    {}

    tscApi::enuCameraID getID_t() const          { return id_t; }
    bool_t valid_b() const               { return isValid_b;}
    float64_t (&GetRstdPtr(void))[3][3]{ return rStd_af64; }
#ifdef ENABLE_SFM
    float64_t (&GetInvKPtr(void))[3][3]{ return invK; }
#endif
    mecl::model::Camera<float32_t> *getCameraObj_px(void) const { return cameraObj_px; }
    float64_t getPreRollKmcs_f64() const { return preRollKmcs_f64; }
    // setter
    void PutID(tscApi::enuCameraID i_CameraID_t) { id_t = i_CameraID_t; }

    bool_t LoadConfig(tscApi::CameraModelMeclCfg_s const * i_CameraModelMeclCfg_pt);
    // --- Unwarp
    bool Unwarp( const fc::Point& i_WarpedPt_rt, fc::Pointd& o_UnwarpedPt_rt ) const;
    // --- Warp: returns int-type point
    bool Warp( const fc::Pointd& i_UnwarpedPt_rt, fc::Point& o_WarpedPt_rt ) const;
    
    // --- Camera2Image
    // Converts point in camera coordinates (metric) to image coordinates (pixel)
    static void Camera2Image( const mecl::model::Camera<float32_t> *i_CameraObj_po, const fc::Pointd& i_CamPt_rx, fc::Pointd& o_ImgPt_rx );
    
    // --- Image2Camera
    // Converts point in image coordinates (pixel) to camera coordinates (metric)
    static void Image2Camera( const mecl::model::Camera<float32_t> *i_CameraObj_px, const fc::Pointd& i_ImgPt_rx, fc::Pointd& o_CamPt_rx );

    const mecl::model::Pinhole<float32_t>::Intrinsic_s& GetIntrinsics() const;
    const mecl::model::Pinhole<float32_t>::Extrinsic_s& GetExtrinsics() const;
    const mecl::model::Sensor<float32_t>& GetSensor() const;
    // --- GetRotationMatrix
    mecl::core::RotationMatrix<float32_t> getRotationMatrix_x() const;
    // --- GetProjectionMatrix
    mecl::core::Matrix<float32_t, 3, 4>& getProjectionMatrix_rx() const;
    // --- Project point in world coordinates onto image sensor coordinate system
    void ApplyFullProjection( const mecl::core::Point4D<float32_t>& i_WorldPt_rx, mecl::core::Point2D<float32_t>& o_ImPt_rx ) const;
    // --- Project point in image frame onto Z=0 plane in world coordinate system
    void ApplyZPlaneBackProjection( const mecl::core::Point2D<float32_t>& i_ImgPt_rx, mecl::core::Point4D<float32_t>& o_WorldPt_rx ) const;
    // --- Calculate Corresponding World Point
    bool_t computeWorldPt_b( const fc::Pointd& i_Pt1_rx, const fc::Pointd& i_Pt2_rx, const mecl::core::Matrix<float32_t, 4U, 4U>& i_MotionTrans_rx, fc::Point3d& o_WorldPt_rt ) const;

    bool_t isRollIn14Quadrants_b() const
    {
        float32_t v_RollDeg_f32 = extrinsicRoll_f32;
        return( (v_RollDeg_f32 >= -90.0) && (v_RollDeg_f32 < 90.0) );
    }
    bool_t isRollIn23Quadrants_b() const
    {
        float32_t v_RollDeg_f32 = extrinsicRoll_f32;
        return( ((v_RollDeg_f32 >= 90.0) && (v_RollDeg_f32 < 270.0)) || ((v_RollDeg_f32 >= -270.0) && (v_RollDeg_f32 < -90.0)) );
    }
    // --- Build Z Axis Rotation matrix
    static bool_t BuildZAxisRotationMatrix( const float64_t i_AngDeg_f64, float64_t (&o_R_raf64)[ 3][ 3 ] );
    // --- Build Y Axis Rotation matrix
    static bool_t BuildYAxisRotationMatrix( const float64_t i_AngDeg_f64, float64_t (&o_R_raf64)[ 3][ 3 ] );
    // --- Build X Axis Rotation matrix
    static bool_t BuildXAxisRotationMatrix( const float64_t i_AngDeg_f64, float64_t (&o_R_raf64)[ 3][ 3 ] );
    // --- Build Extrinsic Rotation matrix
    static bool_t BuildRotationMatrix( const float64_t (&Ra)[ 3][ 3 ], const float64_t (&Rb)[ 3][ 3 ], const float64_t (&Rg)[ 3][ 3 ], float64_t (&R)[ 3][ 3 ] );
    // --- Build Extrinsic Rotation matrix with pre-roll into account
    static bool_t BuildRotationMatrix( const float64_t (&Ra)[ 3][ 3 ], const float64_t (&Rb)[ 3][ 3 ], const float64_t (&Rg)[ 3][ 3 ], const float64_t (&RpreRoll)[ 3][ 3 ], float64_t (&R)[ 3][ 3 ] );
    // --- Build Translation matrix
    static bool BuildTranslationMatrix( float64_t i_Tx_f64, float64_t i_Ty_f64, float64_t i_Tz_f64, float64_t (&T)[ 3][ 4 ] );
    // --- Build Motion-Vector Transformation matrix
    static bool BuildMotionTransformation( float64_t i_DeltaX_f64, float64_t i_DeltaY_f64, float64_t i_DeltaPsiRad_f64, float64_t (&MotionTrans)[ 4][ 4 ] );
    // --- Build Motion-Vector Transformation matrix
    static bool BuildMotionTransformation( float32_t i_DeltaX_f32, float32_t i_DeltaY_f32, float32_t i_DeltaPsiRad_f32, mecl::core::Matrix<float32_t, 4, 4>& o_MotionTransMat_rx );
    // --- Build Projection Matrix
    static bool_t BuildProjectionMatrix( float64_t (&K)[ 3][ 3 ], float64_t (&R)[ 3][ 3 ], float64_t (&T)[ 3][ 4 ], float64_t (&P)[ 3][ 4 ] );

private:
    // --- camera ID
    tscApi::enuCameraID id_t;
    // --- Whether camera parameters are valid
    bool isValid_b;
    float64_t rStd_af64[3][3];
#ifdef ENABLE_SFM
    float64_t invK[3][3];
#endif
    float64_t preRollKmcs_f64;
    float32_t extrinsicRoll_f32;

    mecl::model::Camera<float32_t> *cameraObj_px;
    float32_t downsampleFactor_f32;
};

//-------------------------------------------------------------------------

inline bool CameraModelMecl::Unwarp( const fc::Point& i_WarpedPt_rt, fc::Pointd& o_UnwarpedPt_rt ) const
{
    mecl::core::Point2D<float32_t>::Config_s v_WarpedCfg_t = 
    {{
        static_cast<float32_t>(i_WarpedPt_rt.x_x) * downsampleFactor_f32, 
        static_cast<float32_t>(i_WarpedPt_rt.y_x) * downsampleFactor_f32 
    }};

    mecl::core::Point2D<float32_t> v_Warped_x( v_WarpedCfg_t );
    mecl::core::Point2D<float32_t> v_WarpedMetric_x;
    cameraObj_px->pixelToMetric_v( v_Warped_x, v_WarpedMetric_x );

    mecl::core::Point3D<float32_t> v_UnwarpedMetric_x;
    cameraObj_px->applyUndistortion_v( v_WarpedMetric_x, v_UnwarpedMetric_x );
    
    mecl::core::Point2D<float32_t> v_Unwarped_x;
    cameraObj_px->applyNormalization_v( v_UnwarpedMetric_x, 1.0F, v_Unwarped_x );
    cameraObj_px->metricToPixel_v( v_Unwarped_x, v_Unwarped_x );

    o_UnwarpedPt_rt.x_x = v_Unwarped_x.getPosX();
    o_UnwarpedPt_rt.y_x = v_Unwarped_x.getPosY();

    return true;
}

//-------------------------------------------------------------------------

inline bool CameraModelMecl::Warp( const fc::Pointd& i_UnwarpedPt_rt, fc::Point& o_WarpedPt_rt ) const
{
    mecl::core::Point2D<float32_t>::Config_s v_UnwarpedCfg_t = 
    {{
        static_cast<float32_t>(i_UnwarpedPt_rt.x_x), 
        static_cast<float32_t>(i_UnwarpedPt_rt.y_x) 
    }};
    mecl::core::Point2D<float32_t> v_Unwarped_x( v_UnwarpedCfg_t );
    mecl::core::Point2D<float32_t> v_UnwarpedMetric_x;
    cameraObj_px->pixelToMetric_v( v_Unwarped_x, v_UnwarpedMetric_x );

    mecl::core::Point3D<float32_t>::Config_s v_Unwarped3dCfg_t = 
    {{
        v_UnwarpedMetric_x.getPosX(), 
        v_UnwarpedMetric_x.getPosY(),
        1.0F
    }};
    mecl::core::Point3D<float32_t> v_UnwarpedMetric3d_x( v_Unwarped3dCfg_t );
    mecl::core::Point2D<float32_t> v_WarpedMetric_x;
    cameraObj_px->applyDistortion_v( v_UnwarpedMetric3d_x, 0, v_WarpedMetric_x);
    mecl::core::Point2D<float32_t> v_Warped_x;
    cameraObj_px->metricToPixel_v( v_WarpedMetric_x, v_Warped_x );

    // Convert to the nearest integer (hence: + 0.5)
    o_WarpedPt_rt.x_x = static_cast < sint32_t > ( (v_Warped_x.getPosX() / downsampleFactor_f32) + 0.5 );
    o_WarpedPt_rt.y_x = static_cast < sint32_t > ( (v_Warped_x.getPosY() / downsampleFactor_f32) + 0.5 );

    return true;
}

//-------------------------------------------------------------------------

inline void CameraModelMecl::Camera2Image( const mecl::model::Camera<float32_t> *i_CameraObj_po, const fc::Pointd& i_CamPt_rx, fc::Pointd& o_ImgPt_rx )
{
    mecl::core::Point2D<float32_t>::Config_s v_CamPtCfg_t = 
    {{
        static_cast<float32_t>(i_CamPt_rx.x_x), 
        static_cast<float32_t>(i_CamPt_rx.y_x)
    }};
    mecl::core::Point2D<float32_t> v_CamPtMM_x( v_CamPtCfg_t );
    mecl::core::Point2D<float32_t> v_ImgPtPX_x;
    i_CameraObj_po->metricToPixel_v( v_CamPtMM_x, v_ImgPtPX_x );

    o_ImgPt_rx.x_x = v_ImgPtPX_x.getPosX();
    o_ImgPt_rx.y_x = v_ImgPtPX_x.getPosY();

    return;
}

//-------------------------------------------------------------------------

inline void CameraModelMecl::Image2Camera( const mecl::model::Camera<float32_t> *i_CameraObj_px, const fc::Pointd& i_ImgPt_rx, fc::Pointd& o_CamPt_rx )
{
    mecl::core::Point2D<float32_t>::Config_s v_ImgPtCfg_t = 
    {{
        static_cast<float32_t>(i_ImgPt_rx.x_x), 
        static_cast<float32_t>(i_ImgPt_rx.y_x)
    }};

    mecl::core::Point2D<float32_t> v_ImgPtPX_x( v_ImgPtCfg_t );
    mecl::core::Point2D<float32_t> v_CamPtMM_x;
    i_CameraObj_px->pixelToMetric_v( v_ImgPtPX_x, v_CamPtMM_x );

    o_CamPt_rx.x_x = v_CamPtMM_x.getPosX();
    o_CamPt_rx.y_x = v_CamPtMM_x.getPosY();

    return;
}

//-------------------------------------------------------------------------

inline const mecl::model::Pinhole<float32_t>::Intrinsic_s& CameraModelMecl::GetIntrinsics() const
{
    return reinterpret_cast<mecl::model::Pinhole<float32_t>&>(cameraObj_px->getImager_rx()).getIntrinsic_rs();
}

//-------------------------------------------------------------------------

inline const mecl::model::Pinhole<float32_t>::Extrinsic_s& CameraModelMecl::GetExtrinsics() const
{
    return reinterpret_cast<mecl::model::Pinhole<float32_t>&>(cameraObj_px->getImager_rx()).getExtrinsic_rs();
}

//-------------------------------------------------------------------------

inline const mecl::model::Sensor<float32_t>& CameraModelMecl::GetSensor() const
{
    return reinterpret_cast<mecl::model::Sensor<float32_t>&>(cameraObj_px->getSensor_rx());
}

//-------------------------------------------------------------------------

inline mecl::core::RotationMatrix<float32_t> CameraModelMecl::getRotationMatrix_x() const
{
    return reinterpret_cast<mecl::model::Pinhole<float32_t>&>(cameraObj_px->getImager_rx()).getRotationMatrix_x();
}

//-------------------------------------------------------------------------

inline mecl::core::Matrix<float32_t, 3, 4>& CameraModelMecl::getProjectionMatrix_rx() const
{
  return reinterpret_cast<mecl::model::Pinhole<float32_t>&>(cameraObj_px->getImager_rx()).getProjectionMatrix_rx();
}

//-------------------------------------------------------------------------
inline void CameraModelMecl::ApplyFullProjection( 
    const mecl::core::Point4D<float32_t>& i_WorldPt_rx,
    mecl::core::Point2D<float32_t>& o_ImPt_rx ) const
{
    cameraObj_px->applyFullProjection_v( i_WorldPt_rx, 0.0F, o_ImPt_rx );
}

//-------------------------------------------------------------------------
inline void CameraModelMecl::ApplyZPlaneBackProjection( 
    const mecl::core::Point2D<float32_t>& i_ImgPt_rx,
    mecl::core::Point4D<float32_t>& o_WorldPt_rx ) const
{
    cameraObj_px->applyZPlaneBackProjection_v( i_ImgPt_rx, o_WorldPt_rx );
}

//-----------------------------------------------------------------------------
inline bool_t CameraModelMecl::computeWorldPt_b( const fc::Pointd& i_Pt1_rx, const fc::Pointd& i_Pt2_rx, const mecl::core::Matrix<float32_t, 4U, 4U>& i_MotionTrans_rx, fc::Point3d& o_WorldPt_rt ) const
{
  mecl::core::Matrix<float32_t, 3U, 4U> v_Proj1_x = this->getProjectionMatrix_rx();
  mecl::core::Matrix<float32_t, 3U, 4U> v_Proj2_x = v_Proj1_x % i_MotionTrans_rx;
  fc::Pointd v_CamPt1_x;
  Image2Camera( this->getCameraObj_px(), i_Pt1_rx, v_CamPt1_x );
  fc::Pointd v_CamPt2_x;
  Image2Camera( this->getCameraObj_px(), i_Pt2_rx, v_CamPt2_x );
  mecl::core::Matrix<float32_t, 4U, 3U> v_A_x;
  bool_t v_Return_b = true;
  float32_t v_Proj1Element_f32;
  float32_t v_Proj2Element_f32;

  for( uint32_t v_Col_u32 = 0U; v_Col_u32 < 3U; ++v_Col_u32 )
  {
    v_Proj1Element_f32 = v_Proj1_x( 2, v_Col_u32 );
    v_A_x( 0, v_Col_u32 ) = v_Proj1_x( 0, v_Col_u32 ) - static_cast<float32_t>(v_CamPt1_x.x_x) * v_Proj1Element_f32;
    v_A_x( 1, v_Col_u32 ) = v_Proj1_x( 1, v_Col_u32 ) - static_cast<float32_t>(v_CamPt1_x.y_x) * v_Proj1Element_f32;
    v_Proj2Element_f32 = v_Proj2_x( 2, v_Col_u32 );
    v_A_x( 2, v_Col_u32 ) = v_Proj2_x( 0, v_Col_u32 ) - static_cast<float32_t>(v_CamPt2_x.x_x) * v_Proj2Element_f32;
    v_A_x( 3, v_Col_u32 ) = v_Proj2_x( 1, v_Col_u32 ) - static_cast<float32_t>(v_CamPt2_x.y_x) * v_Proj2Element_f32;
  }

  v_Proj1Element_f32 = v_Proj1_x( 2, 3 );
  v_Proj2Element_f32 = v_Proj2_x( 2, 3 );
  mecl::core::Vector<float32_t, 4U>::Config_s v_VectCfg_t =
  {{
      - ( v_Proj1_x( 0, 3 ) - static_cast<float32_t>(v_CamPt1_x.x_x) * v_Proj1Element_f32 ),
      - ( v_Proj1_x( 1, 3 ) - static_cast<float32_t>(v_CamPt1_x.y_x) * v_Proj1Element_f32 ),
      - ( v_Proj2_x( 0, 3 ) - static_cast<float32_t>(v_CamPt2_x.x_x) * v_Proj2Element_f32 ),
      - ( v_Proj2_x( 1, 3 ) - static_cast<float32_t>(v_CamPt2_x.y_x) * v_Proj2Element_f32 )
  }};
  mecl::core::Vector<float32_t, 4U> v_B_x( v_VectCfg_t );
  mecl::core::Matrix<float32_t, 3, 1> v_Xw_x;
  // 1
  // --- AtA[3 x 3] = A' [3 x 4] x A [4 x 3]
  mecl::core::Matrix3x3<float32_t> v_AtA_x = v_A_x.t() % v_A_x;

  if(true == v_AtA_x.isRegular_b())
  {
    // 2
    // --- inv_AtA [3 x 3] = inv(A' x A)
    mecl::core::Matrix3x3<float32_t> v_InvAtA_x = v_AtA_x.inverse();
    // 3
    // Atb [3 x 1] = A' [3 x 4] x b [4 x 1]
    mecl::core::Matrix<float32_t, 3, 1> v_Atb_x = v_A_x.t() % v_B_x;
    // 4
    // Xw [3 x 1] = inv(A' x A) [3 x 3] x (A' x b) [3 x 1]
    v_Xw_x = v_InvAtA_x % v_Atb_x;
  }
  else
  {
    v_Xw_x(0) = mecl::math::numeric_limits<float32_t>::infinity_x();
    v_Xw_x(1) = mecl::math::numeric_limits<float32_t>::infinity_x();
    v_Xw_x(2) = mecl::math::numeric_limits<float32_t>::infinity_x();
    v_Return_b = false;
  }
  o_WorldPt_rt.x_x = v_Xw_x( 0 );
  o_WorldPt_rt.y_x = v_Xw_x( 1 );
  o_WorldPt_rt.z_x = v_Xw_x( 2 );

  return v_Return_b;
}

//-----------------------------------------------------------------------------
inline bool_t CameraModelMecl::BuildZAxisRotationMatrix( const float64_t i_AngDeg_f64, float64_t (&o_R_raf64)[ 3][ 3 ] )
{
    float64_t v_AngRad_f64 = tsc_math::Degrees2Radians( i_AngDeg_f64 );
    float64_t v_CosAngle_f64 = cos( v_AngRad_f64 );
    float64_t v_SinAngle_f64 = sin( v_AngRad_f64 );

    o_R_raf64[ 0][ 0 ] = v_CosAngle_f64;
    o_R_raf64[ 0][ 1 ] = v_SinAngle_f64;
    o_R_raf64[ 0][ 2 ] = 0;

    o_R_raf64[ 1][ 0 ] =  - v_SinAngle_f64;
    o_R_raf64[ 1][ 1 ] = v_CosAngle_f64;
    o_R_raf64[ 1][ 2 ] = 0;

    o_R_raf64[ 2][ 0 ] = 0;
    o_R_raf64[ 2][ 1 ] = 0;
    o_R_raf64[ 2][ 2 ] = 1.0;

    return true;
}

//-------------------------------------------------------------------------

inline bool_t CameraModelMecl::BuildYAxisRotationMatrix( const float64_t i_AngDeg_f64, float64_t (&o_R_raf64)[ 3][ 3 ] )
{
    float64_t v_AngRad_f64 = tsc_math::Degrees2Radians( i_AngDeg_f64 );
    float64_t v_CosAngle_f64 = cos( v_AngRad_f64 );
    float64_t v_SinAngle_f64 = sin( v_AngRad_f64 );

    o_R_raf64[ 0][ 0 ] = v_CosAngle_f64;
    o_R_raf64[ 0][ 1 ] = 0;
    o_R_raf64[ 0][ 2 ] =  - v_SinAngle_f64;

    o_R_raf64[ 1][ 0 ] = 0;
    o_R_raf64[ 1][ 1 ] = 1.0;
    o_R_raf64[ 1][ 2 ] = 0;

    o_R_raf64[ 2][ 0 ] = v_SinAngle_f64;
    o_R_raf64[ 2][ 1 ] = 0;
    o_R_raf64[ 2][ 2 ] = v_CosAngle_f64;

    return true;
}

//-------------------------------------------------------------------------

inline bool_t CameraModelMecl::BuildXAxisRotationMatrix( const float64_t i_AngDeg_f64, float64_t (&o_R_raf64)[ 3][ 3 ] )
{
    float64_t v_AngRad_f64 = tsc_math::Degrees2Radians( i_AngDeg_f64 );
    float64_t v_CosAngle_f64 = cos( v_AngRad_f64 );
    float64_t v_SinAngle_f64 = sin( v_AngRad_f64 );

    o_R_raf64[ 0][ 0 ] = 1.0;
    o_R_raf64[ 0][ 1 ] = 0;
    o_R_raf64[ 0][ 2 ] = 0;

    o_R_raf64[ 1][ 0 ] = 0;
    o_R_raf64[ 1][ 1 ] = v_CosAngle_f64;
    o_R_raf64[ 1][ 2 ] = v_SinAngle_f64;

    o_R_raf64[ 2][ 0 ] = 0;
    o_R_raf64[ 2][ 1 ] =  - v_SinAngle_f64;
    o_R_raf64[ 2][ 2 ] = v_CosAngle_f64;

    return true;
}

//-------------------------------------------------------------------------

inline bool_t CameraModelMecl::BuildRotationMatrix( const float64_t (&Ra)[ 3][ 3 ], const float64_t (&Rb)[ 3][ 3 ], const float64_t (&Rg)[ 3][ 3 ], float64_t (&R)[ 3][ 3 ] )
{
    bool_t v_Ret_b = true;
    // --- R[3x3] = Rg[3x3] x Rb[3x3]
    bool_t v_Result_b = tsc_math::MatrixMultiply( &Rg[ 0][ 0 ], 3, 3, false, &Rb[ 0][ 0 ], 3, 3, false, &R[ 0][ 0 ] );
    if( !v_Result_b )
    {
        v_Ret_b = false;
    }
    else
    {
        // --- R[3x3] = R[3x3] x Ra[3x3]
        v_Result_b = tsc_math::MatrixMultiply( &R[ 0][ 0 ], 3, 3, false, &Ra[ 0][ 0 ], 3, 3, false, &R[ 0][ 0 ] );
        v_Ret_b = v_Result_b;
    }

    return v_Ret_b;
}

//-------------------------------------------------------------------------

inline bool_t CameraModelMecl::BuildRotationMatrix( const float64_t (&Ra)[ 3][ 3 ], const float64_t (&Rb)[ 3][ 3 ], const float64_t (&Rg)[ 3][ 3 ], const float64_t (&RpreRoll)[3][3], float64_t (&R)[ 3][ 3 ] )
{
    bool_t v_Ret_b = true;
    // --- R[3x3] = Rg[3x3] x Rb[3x3]
    bool_t v_Result_b = tsc_math::MatrixMultiply( &Rg[ 0][ 0 ], 3, 3, false, &Rb[ 0][ 0 ], 3, 3, false, &R[ 0][ 0 ] );
    if( !v_Result_b )
    {
        v_Ret_b = false;
    }
    else
    {
        // --- R[3x3] = R[3x3] x Ra[3x3]
        v_Result_b = tsc_math::MatrixMultiply( &R[ 0][ 0 ], 3, 3, false, &Ra[ 0][ 0 ], 3, 3, false, &R[ 0][ 0 ] );
        if (! v_Result_b )
        {
            v_Ret_b = false;
        }
        else
        {
            v_Result_b = tsc_math::MatrixMultiply( &R[ 0][ 0 ], 3, 3, false, &RpreRoll[ 0][ 0 ], 3, 3, false, &R[ 0][ 0 ] );
            v_Ret_b = v_Result_b;
        }
    }

    return v_Ret_b;
}

//-------------------------------------------------------------------------
inline bool CameraModelMecl::BuildTranslationMatrix( float64_t i_Tx_f64, float64_t i_Ty_f64, float64_t i_Tz_f64, float64_t (&T)[ 3][ 4 ] )
{
    T[ 0][ 0 ] = 1.0;
    T[ 0][ 1 ] = 0.0;
    T[ 0][ 2 ] = 0.0;
    T[ 0][ 3 ] = -i_Tx_f64;

    T[ 1][ 0 ] = 0.0;
    T[ 1][ 1 ] = 1.0;
    T[ 1][ 2 ] = 0.0;
    T[ 1][ 3 ] = -i_Ty_f64;

    T[ 2][ 0 ] = 0.0;
    T[ 2][ 1 ] = 0.0;
    T[ 2][ 2 ] = 1.0;
    T[ 2][ 3 ] = -i_Tz_f64;

    return true;
}

//-------------------------------------------------------------------------
inline bool CameraModelMecl::BuildMotionTransformation( float64_t i_DeltaX_f64, float64_t i_DeltaY_f64, float64_t i_DeltaPsiRad_f64, float64_t (&MotionTrans)[ 4][ 4 ] )
{
    float64_t v_CosPsi_f64 = cos( i_DeltaPsiRad_f64 );
    float64_t v_SinPsi_f64 = sin( i_DeltaPsiRad_f64 );

    MotionTrans[ 0][ 0 ] = v_CosPsi_f64;
    MotionTrans[ 0][ 1 ] = v_SinPsi_f64;
    MotionTrans[ 0][ 2 ] = 0.0;
    MotionTrans[ 0][ 3 ] =  - i_DeltaX_f64 * v_CosPsi_f64 - i_DeltaY_f64 * v_SinPsi_f64;

    MotionTrans[ 1][ 0 ] =  - v_SinPsi_f64;
    MotionTrans[ 1][ 1 ] = v_CosPsi_f64;
    MotionTrans[ 1][ 2 ] = 0.0;
    MotionTrans[ 1][ 3 ] = i_DeltaX_f64 * v_SinPsi_f64 - i_DeltaY_f64 * v_CosPsi_f64;

    MotionTrans[ 2][ 0 ] = 0.0;
    MotionTrans[ 2][ 1 ] = 0.0;
    MotionTrans[ 2][ 2 ] = 1.0;
    MotionTrans[ 2][ 3 ] = 0.0;

    MotionTrans[ 3][ 0 ] = 0.0;
    MotionTrans[ 3][ 1 ] = 0.0;
    MotionTrans[ 3][ 2 ] = 0.0;
    MotionTrans[ 3][ 3 ] = 1.0;

    return true;
}

//-------------------------------------------------------------------------
inline bool CameraModelMecl::BuildMotionTransformation( 
    float32_t i_DeltaX_f32, float32_t i_DeltaY_f32, float32_t i_DeltaPsiRad_f32, 
    mecl::core::Matrix<float32_t, 4, 4>& o_MotionTransMat_rx )
{
    float32_t v_CosPsi_f32 = mecl::math::trigonometry< float32_t >::cos_x( i_DeltaPsiRad_f32 );
    float32_t v_SinPsi_f32 = mecl::math::trigonometry< float32_t >::sin_x( i_DeltaPsiRad_f32 );

    o_MotionTransMat_rx = mecl::core::Matrix<float32_t, 4, 4>::eye();

    o_MotionTransMat_rx( 0, 0 ) = v_CosPsi_f32;
    o_MotionTransMat_rx( 0, 1 ) = v_SinPsi_f32;
    o_MotionTransMat_rx( 0, 3 ) = - i_DeltaX_f32 * v_CosPsi_f32 - i_DeltaY_f32 * v_SinPsi_f32;

    o_MotionTransMat_rx( 1, 0 ) = - v_SinPsi_f32;
    o_MotionTransMat_rx( 1, 1 ) = v_CosPsi_f32;
    o_MotionTransMat_rx( 1, 3 ) = i_DeltaX_f32 * v_SinPsi_f32 - i_DeltaY_f32 * v_CosPsi_f32;

    return true;
}

//-------------------------------------------------------------------------
inline bool_t CameraModelMecl::BuildProjectionMatrix( float64_t (&K)[ 3][ 3 ], float64_t (&R)[ 3][ 3 ], float64_t (&T)[ 3][ 4 ], float64_t (&P)[ 3][ 4 ] )
{
    bool_t v_Ret_b = true;
    //
    // --- KR[3x3] = K[3x3] x R[3x3]  
    static float64_t v_KR_f64[ 3][ 3 ];
    bool_t v_Result_b = tsc_math::MatrixMultiply( &K[ 0][ 0 ], 3, 3, false, &R[ 0][ 0 ], 3, 3, false, &v_KR_f64[ 0][ 0 ] );

    if( !v_Result_b )
    {
      v_Ret_b = false;
    }
    else
    {
      //
      // --- P[3x4] = KR[3x3] x T[3x4]
      v_Result_b = tsc_math::MatrixMultiply( &v_KR_f64[ 0][ 0 ], 3, 3, false, &T[ 0][ 0 ], 3, 4, false, &P[ 0][ 0 ] );

      if( !v_Result_b )
      {
        v_Ret_b = false;
      }
    }
    return v_Ret_b;
}

//-------------------------------------------------------------------------

inline bool_t CameraModelMecl::LoadConfig(tscApi::CameraModelMeclCfg_s const * i_CameraModelMeclCfg_pt)
{
    bool_t v_Ret_b = true;

    isValid_b = false;
    
    if ( i_CameraModelMeclCfg_pt->camera_px != reinterpret_cast<mecl::model::Camera<float32_t>*>(tsc_cfg::CAMERAMODEL_MECL_OBJ_UNDEFINED) )
    {
    cameraObj_px = i_CameraModelMeclCfg_pt->camera_px;
    downsampleFactor_f32 = i_CameraModelMeclCfg_pt->downSampleFactor_f32;

    mecl::core::RotationMatrix<float32_t>::EulerAngles_s v_EulerAnglesRad_s = GetExtrinsics().eulerAngles_s;
    float32_t v_Pitch_f32 = mecl::math::toDegrees_x<float32_t>( v_EulerAnglesRad_s.pitch_x );
    float32_t v_Yaw_f32 = mecl::math::toDegrees_x<float32_t>( v_EulerAnglesRad_s.yaw_x );
    extrinsicRoll_f32 = mecl::math::toDegrees_x<float32_t>( v_EulerAnglesRad_s.roll_x );
    tsc_math::toRange<float32_t>(extrinsicRoll_f32, 180.0F);

    if( id_t == tscApi::e_TscRightCam )
    {
        preRollKmcs_f64 = 180.0;
    }
    else if( id_t == tscApi::e_TscFrontCam )
    {
        preRollKmcs_f64 = 270.0;
    }
    else if( id_t == tscApi::e_TscLeftCam )
    {
        preRollKmcs_f64 = 0.0;
    }
    else if( id_t == tscApi::e_TscRearCam )
    {
        preRollKmcs_f64 = 90.0;
    } else
    {
    }

    // --- update R_std for LFC
    float64_t v_Rg_af64[ 3][ 3 ];
    float64_t v_Rb_af64[ 3][ 3 ];
    float64_t v_Ra_af64[ 3][ 3 ];
    float64_t v_RMagna_af64[ 3][ 3 ];
    float64_t v_RpreRoll_af64[ 3][ 3 ];
    float64_t v_InvRpreRoll_af64[ 3][ 3 ];

    // Standard Coordinate System computation
    BuildXAxisRotationMatrix( v_Pitch_f32, v_Ra_af64 );
    BuildYAxisRotationMatrix( v_Yaw_f32, v_Rb_af64 );
    BuildZAxisRotationMatrix( extrinsicRoll_f32, v_Rg_af64 );

    BuildRotationMatrix( v_Ra_af64, v_Rb_af64, v_Rg_af64, v_RMagna_af64 );

    BuildZAxisRotationMatrix( preRollKmcs_f64, v_RpreRoll_af64 );
    BuildZAxisRotationMatrix( - preRollKmcs_f64, v_InvRpreRoll_af64 );

    BuildRotationMatrix( v_RpreRoll_af64, v_RMagna_af64, v_InvRpreRoll_af64, rStd_af64 );
    
#ifdef ENABLE_SFM
    float64_t K[3][3];
    K[ 0][ 0 ] =  - GetIntrinsics().foclX_x / GetSensor().getPsz_rx().getPosX();
    K[ 0][ 1 ] = 0;
    K[ 0][ 2 ] = GetSensor().getPpp_rx().getPosX();

    // --- K matrix
    K[ 1][ 0 ] = 0;
    K[ 1][ 1 ] = GetIntrinsics().foclY_x / GetSensor().getPsz_rx().getPosY();
    K[ 1][ 2 ] = GetSensor().getPpp_rx().getPosY();

    K[ 2][ 0 ] = 0;
    K[ 2][ 1 ] = 0;
    K[ 2][ 2 ] = 1;

    // --- compute the K inverse
    bool_t result = tsc_math::MatrixInvert( K, 3, invK );
    if( !result )
    {
        v_Ret_b = false;
    }
#endif

    isValid_b = v_Ret_b;
    }

    return v_Ret_b;
}
//-------------------------------------------------------------------------

}
#endif
#endif
