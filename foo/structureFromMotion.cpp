// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [08-Jul-2013]
// --- Modified by Ehsan Parvizi [29-May-2014]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include "tscAlg.h"
#include <cfloat>
#ifdef ENABLE_SFM   // PRQA S 1070
// ----------------------------------------------------------------------------
using fc::ValidFeature;
using tsc_trace::kCameraStrings;
// ----------------------------------------------------------------------------
namespace tsc
{
StructureFromMotion::StructureFromMotion( ): 
    m_cfg( NULL ), 
    m_hTracer ( 0 ),
    m_cameraModel( NULL ), 
    Rg(), Rb(), Ra(), R(), 
#ifdef USE_SVSCM
    P1(), P2(), MotionTrans(), F(),
#else
    PMat1(), PMat2(), MotionTransMat(),
#endif
    R_km(), t_km(), 
    R_c(), t_c(), 
    E()
{
    // --- collect the needed variables for convenient usage
    TRACE_2( m_hTracer, "%s [%s]: Structure From Motion object created", tsc_cfg::MODULE_NAME_SFM, kCameraStrings[ m_cameraID ].c_str() );

    // --- initialize implementation specific data structures
}

//-------------------------------------------------------------------------

StructureFromMotion::~StructureFromMotion()
{
    // --- cleanups
    TRACE_2( m_hTracer, "%s [%s]: Structure From Motion object destroyed", tsc_cfg::MODULE_NAME_SFM, kCameraStrings[ m_cameraID ].c_str() );
}

//-------------------------------------------------------------------------

bool_t StructureFromMotion::Init( tscApi::enuCameraID cameraID )
{
    m_cameraID = cameraID;
    m_cfg = TSCAlg::getInstance_rt().getModuleConfig_pt();
    m_hTracer = TSCAlg::getInstance_rt().getTracer_u64();
    size_t ind = static_cast<size_t>(cameraID);
    m_cameraModel = fc::FeatureCollection::getInstance_rt().getModuleInfo_rt().getMAddrCmrMdls_po();
    m_cameraModel += ind;
    TRACE_2( m_hTracer, "%s [%s]: Structure From Motion object initialized", tsc_cfg::MODULE_NAME_SFM, kCameraStrings[ m_cameraID ].c_str() );

    return true;
}

//-------------------------------------------------------------------------

bool_t StructureFromMotion::loadConfiguration_b(void)
{
    // --- load from m_cfg configuration specific to Structure From Motion (SfM) object
    return true;
}

// ----------------------------------------------------------------------------
bool_t StructureFromMotion::Process( fc::ValidFeatureCollection& validFeatureCollection, uint32_t frameNumber, fc::InitialGuess &ig ) // PRQA S 4327
{
    bool_t retValue = true;

    // --- process this SfM
    CREATE_TIMER( t );
    START_TIMER( t );
    TRACE_4( m_hTracer, "%s [%s] [%s]: Initiate processing frame %lu ...", 
        tsc_cfg::MODULE_NAME_SFM, __func__, kCameraStrings[ m_cameraID ].c_str(), frameNumber );

#ifndef USE_SVSCM
    // --- create local camera object for projection matrix calcaution based on initial guess angles
    m_cameraObj.setLens_v( m_cameraModel->GetCameraObj()->getLens_rx() );
    m_cameraObj.setSensor_v( m_cameraModel->GetCameraObj()->getSensor_rx() );
    // --- reset Extrinsics with initial guess angles
    mecl::model::Pinhole<float32_t> localPinhole = reinterpret_cast<mecl::model::Pinhole<float32_t>&>( m_cameraModel->GetCameraObj()->getImager_rx() );
    mecl::model::Pinhole<float32_t>::Extrinsic_s newExtrinsics( m_cameraModel->GetExtrinsics() );
    newExtrinsics.eulerAngles_s.pitch_x = mecl::math::toRadians_x< float32_t >( ig.GetPitch() );
    newExtrinsics.eulerAngles_s.yaw_x = mecl::math::toRadians_x< float32_t >( ig.GetYaw() );
    newExtrinsics.eulerAngles_s.roll_x = mecl::math::toRadians_x< float32_t >( ig.GetRoll() );
    localPinhole.setExtrinsic_v( newExtrinsics );
    localPinhole.init_v();   // recalculate projection matrix
    m_cameraObj.setImager_v( localPinhole );
#endif
   
    
    // --- build the camera rotation matrices based on initial guess angles 
    m_cameraModel->BuildZAxisRotationMatrix( ig.GetRoll(), Rg );
    m_cameraModel->BuildYAxisRotationMatrix( ig.GetYaw(), Rb );
    m_cameraModel->BuildXAxisRotationMatrix( ig.GetPitch(), Ra );

    m_cameraModel->BuildRotationMatrix( Ra, Rb, Rg, R );

#ifdef USE_SVSCM
    camera_model::CameraModel::BuildProjectionMatrix( m_cameraModel->GetIntrinsic().GetKPtr(), R, m_cameraModel->GetTPtr(), P1 );    
#else
    camera_model::CameraModelMecl::BuildTranslationMatrix( 0, 0, localPinhole.getExtrinsic_rs().pos_x.cVal_ax[2], T );

    PMat1 = localPinhole.getProjectionMatrix_rx();
#endif
    // --- actual implementation for SfM

    tempVF.clear_v();
    mecl::core::ArrayList < ValidFeature, tsc_cfg::NUM_VALID_FEATURES > ::iterator it = validFeatureCollection.GetVldFtrs().rwBegin_o();
    mecl::core::ArrayList < ValidFeature, tsc_cfg::NUM_VALID_FEATURES > ::const_iterator end = validFeatureCollection.GetVldFtrs().end_o();
    for( ; it != end; ++it )
    {
        ValidFeature& vf =  * it;
        
        // Remove outliers using fundamental matrix
#ifdef USE_SVSCM
        if( !IsFundMatInlier( vf ) )
#else
        if( !IsEssMatInlier( vf ) )
#endif
        {
            TRACE_4( m_hTracer, "%s [%s] [%s]: Removed outlier feature from ValidFeatureCollection. New size [%u]", 
                tsc_cfg::MODULE_NAME_SFM, __func__, kCameraStrings[ m_cameraID ].c_str(), validFeatureCollection.GetVldFtrs().size_u32() );
        }
        else
        {
            tempVF.pushBack_v( vf );
        }
    }

    validFeatureCollection.GetVldFtrs().clear_v();
    mecl::core::ArrayList < ValidFeature, tsc_cfg::NUM_VALID_FEATURES > ::iterator it2 = tempVF.rwBegin_o();
    for( ; it2 != tempVF.end_o(); ++it2 )
    {
        validFeatureCollection.GetVldFtrs().pushBack_v(*it2);
    }

    mecl::core::ArrayList < ValidFeature, tsc_cfg::NUM_VALID_FEATURES > ::iterator it3 = validFeatureCollection.GetVldFtrs().rwBegin_o();
    for( ; it3 != validFeatureCollection.GetVldFtrs().end_o(); ++it3 )
    {
        ValidFeature& vf =  * it3;

        if ( retValue == false )
        {
          break;
        }

        if( (!vf.GetTrackList()[ 0 ].GetIsUnWarped()) || (!vf.GetTrackList()[ vf.GetTrackList().size_u32() - 1 ].GetIsUnWarped()) )
        {
            TRACE_3( m_hTracer, "%s [%s] [%s]: Encountered a valid feature with no unwarped image point. Skipping...",  \
                tsc_cfg::MODULE_NAME_SFM, __func__, kCameraStrings[ m_cameraID ].c_str() );
            continue;
        }

#ifdef USE_SVSCM
        camera_model::CameraModel::BuildMotionTransformation( vf.GetMotionVector().GetX(), vf.GetMotionVector().GetY(), vf.GetMotionVector().GetPsi(), MotionTrans );

        bool_t result = tsc_math::MatrixMultiply( &P1[ 0][ 0 ], 3, 4, false, &MotionTrans[ 0][ 0 ], 4, 4, false, &P2[ 0][ 0 ] );
        if( !result )
        {
            TRACE_3( m_hTracer, "%s [%s] [%s]: Failed to multiply proj1 x MotionTrans", 
                tsc_cfg::MODULE_NAME_SFM, __func__, kCameraStrings[ m_cameraID ].c_str() );
            retValue = false;
            continue;
        }

        // Building A[4x3] and beta[4x1]

        fc::Point3d worldPt;
        result = CalculateCorrespondingWorldPt( vf.GetTrackList()[ 0 ].GetUnWarpedPt(), vf.GetTrackList()[ vf.GetTrackList().size_u32() - 1 ].GetUnWarpedPt(), P1, P2, worldPt );
        vf.PutWorldPt(worldPt);
        if( !result )
        {
            TRACE_3( m_hTracer, "%s [%s] [%s]: Failed to Calculate Corresponding World Point", 
                tsc_cfg::MODULE_NAME_SFM, __func__, kCameraStrings[ m_cameraID ].c_str() );
            retValue = false;
            continue;
        }
#else
        // transform motion vector to VCS
        float32_t s1 = 0;
        float32_t s2 = 0;
        if ( m_cameraID == tscApi::e_TscFrontCam )
        {
            s1 = static_cast<float32_t>( -vf.GetMotionVector().GetY() );
            s2 = static_cast<float32_t>(  vf.GetMotionVector().GetX() );
        }
        else if ( m_cameraID == tscApi::e_TscRearCam )
        {
            s1 = static_cast<float32_t>(  vf.GetMotionVector().GetY() );
            s2 = static_cast<float32_t>( -vf.GetMotionVector().GetX() );
        }
        else if ( m_cameraID == tscApi::e_TscLeftCam )
        {
            s1 = static_cast<float32_t>( -vf.GetMotionVector().GetX() );
            s2 = static_cast<float32_t>( -vf.GetMotionVector().GetY() );
        }
        else if ( m_cameraID == tscApi::e_TscRightCam )
        {
            s1 = static_cast<float32_t>( vf.GetMotionVector().GetX() );
            s2 = static_cast<float32_t>( vf.GetMotionVector().GetY() );
        }
        else
        {
            TRACE_3( m_hTracer, "%s [%s] [%s]: Failed to transform motion vector to VCS. Invalid Camera ID", tsc_cfg::MODULE_NAME_SFM, __func__, kCameraStrings[ m_cameraID ].c_str() );
            retValue = false;
            continue;
        }

        camera_model::CameraModelMecl::BuildMotionTransformation( s1, s2, static_cast<float32_t>(vf.GetMotionVector().GetPsi()), MotionTransMat );
        PMat2 = PMat1 % MotionTransMat;

        fc::Point3d worldPt;
        bool_t result = CalculateCorrespondingWorldPt( vf.GetTrackList()[ 0 ].GetUnWarpedPt(), vf.GetTrackList()[ vf.GetTrackList().size_u32() - 1 ].GetUnWarpedPt(), PMat1, PMat2, worldPt );
        vf.PutWorldPt(worldPt);
        if( !result )
        {
            TRACE_3( m_hTracer, "%s [%s] [%s]: Failed to Calculate Corresponding World Point", 
                tsc_cfg::MODULE_NAME_SFM, __func__, kCameraStrings[ m_cameraID ].c_str() );
            retValue = false;
            continue;
        }
#endif
    }

    if (retValue == true)
    {
    FilterByWorldDistance( validFeatureCollection );

    STOP_TIMER( t );
    TRACE_4( m_hTracer, "%s [%s] [%s]: Processed in Total [%.1f]ms", 
        tsc_cfg::MODULE_NAME_SFM, __func__, kCameraStrings[ m_cameraID ].c_str(), GET_ELAPSED_TIME( t ));
    }
    return retValue;
}

//-------------------------------------------------------------------------
void StructureFromMotion::FilterByWorldDistance( fc::ValidFeatureCollection& validFeatureCollection )
{
    tempVF.clear_v();
    mecl::core::ArrayList < ValidFeature, tsc_cfg::NUM_VALID_FEATURES > ::iterator it = validFeatureCollection.GetVldFtrs().rwBegin_o();
    mecl::core::ArrayList < ValidFeature, tsc_cfg::NUM_VALID_FEATURES > ::const_iterator end = validFeatureCollection.GetVldFtrs().end_o();
    for( ; it != end; ++it )
    {
        ValidFeature& vf =  * it;
        
        if( (fabs( vf.GetWorldPt().x_x ) > m_cfg->GetWorldPtLimitX()) || 
            (fabs( vf.GetWorldPt().y_x ) > m_cfg->GetWorldPtLimitY()) || 
            (fabs( vf.GetWorldPt().z_x ) > m_cfg->GetWorldPtLimitZ()) )
        {
            TRACE_3( m_hTracer, "[%s] [%s]: Removed feature from ValidFeatureCollection. New size [%u]",  \
                __func__, kCameraStrings[ m_cameraID ].c_str(), validFeatureCollection.GetVldFtrs().size_u32() );
        }
        else
        {
            tempVF.pushBack_v( vf );
        }
    }

    validFeatureCollection.GetVldFtrs().clear_v();
    mecl::core::ArrayList < ValidFeature, tsc_cfg::NUM_VALID_FEATURES > ::iterator it2 = tempVF.rwBegin_o();
    for( ; it2 != tempVF.end_o(); ++it2 )
    {
        validFeatureCollection.GetVldFtrs().pushBack_v(*it2);
    }
}

// ----------------------------------------------------------------------------
bool_t StructureFromMotion::BuildEssentialMatrix()
{
    bool_t retValue = true;

    E[ 0][ 0 ] = 0;
    E[ 0][ 1 ] = -t_c[2];
    E[ 0][ 2 ] = t_c[1];

    E[ 1][ 0 ] = t_c[2];
    E[ 1][ 1 ] = 0;
    E[ 1][ 2 ] = -t_c[0];

    E[ 2][ 0 ] = -t_c[1];
    E[ 2][ 1 ] = t_c[0];
    E[ 2][ 2 ] = 0;

    bool_t result = tsc_math::MatrixMultiply( &E[ 0][ 0 ], 3, 3, false, &R_c[ 0][ 0 ], 3, 3, false, &E[ 0][ 0 ] );
    if( !result )
    {
        TRACE_2( m_hTracer, "[%s] [%s]: Failed to multiply [t_c]_x by R_c'", __func__, kCameraStrings[ m_cameraID ].c_str() );
        retValue = false;
    }

    return retValue;
}

//-------------------------------------------------------------------------
#ifdef USE_SVSCM
bool_t StructureFromMotion::CalculateCorrespondingWorldPt( const fc::Pointd& pt1, const fc::Pointd& pt2, float64_t (&proj1)[ 3][ 4 ], float64_t (&proj2)[ 3][ 4 ], fc::Point3d& worldPt ) const
{
    bool_t retValue = true;

    float64_t A[ 4][ 3 ];
    float64_t b[ 4 ];

    for( uint32_t col = 0; col < 3; ++col )
    {
        A[ 0][ col ] = proj1[ 0][ col ] - pt1.x_x * proj1[ 2][ col ];
        A[ 1][ col ] = proj1[ 1][ col ] - pt1.y_x * proj1[ 2][ col ];
        A[ 2][ col ] = proj2[ 0][ col ] - pt2.x_x * proj2[ 2][ col ];
        A[ 3][ col ] = proj2[ 1][ col ] - pt2.y_x * proj2[ 2][ col ];
    }

    b[ 0 ] =  - ( proj1[ 0][ 3 ] - pt1.x_x * proj1[ 2][ 3 ] );
    b[ 1 ] =  - ( proj1[ 1][ 3 ] - pt1.y_x * proj1[ 2][ 3 ] );
    b[ 2 ] =  - ( proj2[ 0][ 3 ] - pt2.x_x * proj2[ 2][ 3 ] );
    b[ 3 ] =  - ( proj2[ 1][ 3 ] - pt2.y_x * proj2[ 2][ 3 ] );

    // 1
    // --- Atrans_A[3 x 3] = A' [3 x 4] x A [4 x 3]

    float64_t Atrans_A[ 3][ 3 ];

    bool_t result = tsc_math::MatrixMultiply( &A[ 0][ 0 ], 4, 3, true, &A[ 0][ 0 ], 4, 3, false, &Atrans_A[ 0][ 0 ] );

    if( !result )
    {
        TRACE_2( m_hTracer, "[%s] [%s]: Failed to calculate (A' x A).", __func__, kCameraStrings[ m_cameraID ].c_str() );
        retValue = false;
    }
    else
    {
      // 2
      // --- inv_AtransA [3 x 3]

      const uint8_t widthHeight = 3;
      float64_t inv_AtransA[ 3][ 3 ]; //dst
      //src
      bool_t status = tsc_math::MatrixInvert(  Atrans_A, widthHeight, inv_AtransA );

      if( status == false )
      {
        TRACE_2( m_hTracer, "[%s] [%s]: Failed to calculate inv(A' x A).", __func__, kCameraStrings[ m_cameraID ].c_str() );
        retValue = false;
      }
      else
      {

        // 3
        // Atrans_b

        // dst vector Atrans_b [ 3 x 1]
        float64_t Atrans_b[ 3 ];

        result = tsc_math::MatrixVectorMultiply( &A[ 0][ 0 ], 4, 3, true, &b[ 0 ], &Atrans_b[ 0 ] );

        if( !result )
        {
          TRACE_2( m_hTracer, "[%s] [%s]: Failed to calculate (A' x b).", __func__, kCameraStrings[ m_cameraID ].c_str() );
          retValue = false;
        }
        else
        {

          // 4
          // inv(A' x A) x (A' x b)

          // src1 matrix inv_AtransA [3 x 3]
          // src2 vector Atrans_b [ 3 x 1]
          // dst matrix Xw [ 3 x 1]
          float64_t Xw[ 3 ];
          result = tsc_math::MatrixVectorMultiply( &inv_AtransA[ 0][ 0 ], 3, 3, false, &Atrans_b[ 0 ], &Xw[ 0 ] );

          if( !result )
          {
            TRACE_2( m_hTracer, "[%s] [%s]: Failed to calculate inv(A' x A) x (A' x b).", __func__, kCameraStrings[ m_cameraID ].c_str() );
            retValue = false;
          }

          else
          {
            worldPt.x_x = Xw[ 0 ];
            worldPt.y_x = Xw[ 1 ];
            worldPt.z_x = Xw[ 2 ];
          }
        }
      }
    }

    return retValue;
}

//-------------------------------------------------------------------------
bool_t StructureFromMotion::IsFundMatInlier( ValidFeature& vf )
{
    bool_t retValue = true;

    uint32_t trackSz = vf.GetTrackList().size_u32();

    if( trackSz < 2 )
    {
        retValue = false;
    }
    else
    {

      fc::ImageFeature& lastFeature = vf.GetTrackList().back_ro();
      bool_t result;

      // compute t_km
      {
        t_km[0] = vf.GetMotionVector().GetX();
        t_km[1] = vf.GetMotionVector().GetY();
        t_km[2] = 0;

        float64_t norm = t_km[0] * t_km[0] + t_km[1] * t_km[1];
        norm = sqrt( norm );

        if( norm > FLT_EPSILON ) // PRQA S 5053
        {    
            t_km[0] /= norm;
            t_km[1] /= norm;
        }
      }

      // compute R_km

      camera_model::CameraModel::BuildZAxisRotationMatrix( tsc_math::Radians2Degrees( vf.GetMotionVector().GetPsi() ), R_km );

      // compute t_c = R_ig * t_km
      result = tsc_math::MatrixVectorMultiply( &R[ 0][ 0 ], 3, 3, false, &t_km[ 0 ], &t_c[ 0 ] );
      if( !result )
      {
          TRACE_2( m_hTracer, "[%s] [%s]: Failed to compute t_c = R x t_km", __func__, kCameraStrings[ m_cameraID ].c_str() );
          retValue = false;
      }
      else

      // calc R_c = R * R_km * R'
      {
        result = tsc_math::MatrixMultiply( &R[ 0][ 0 ], 3, 3, false, &R_km[ 0][ 0 ], 3, 3, false, &R_c[ 0][ 0 ] );
        if( !result )
        {
            TRACE_2( m_hTracer, "[%s] [%s]: Failed to multiply R x R_km", __func__, kCameraStrings[ m_cameraID ].c_str() );
            retValue = false;
        }
        else
        {

          result = tsc_math::MatrixMultiply( &R_c[ 0][ 0 ], 3, 3, false, &R[ 0][ 0 ], 3, 3, true, &R_c[ 0][ 0 ] );
          if( !result )
          {
            TRACE_2( m_hTracer, "[%s] [%s]: Failed to multiply (R x R_km) x R'", __func__, kCameraStrings[ m_cameraID ].c_str() );
            retValue = false;
          }
          else
          {

            // Essential Matrix
            result = BuildEssentialMatrix();
            if( !result )
            {
              TRACE_2( m_hTracer, "[%s] [%s]: Failed to BuildEssentialMatrix", __func__, kCameraStrings[ m_cameraID ].c_str() );
              retValue = false;
            }
            else
            {

              // Fundamental Matrix(
              result = BuildFundamentalMatrix();
              if( !result )
              {
                TRACE_2( m_hTracer, "[%s] [%s]: Failed to BuildFundamentalMatrix", __func__, kCameraStrings[ m_cameraID ].c_str() );
                retValue = false;
              }
              else
              {

                // test pt2 * F * pt1 < m_fundMatOutlierThresh
                const fc::ImageFeature& firstFeature = vf.GetTrackList().front_ro();

                float64_t firstPoint[3] = { firstFeature.GetUnWarpedPt().x_x, firstFeature.GetUnWarpedPt().y_x, 1.0 };

                float64_t rhs[3];
                result = tsc_math::MatrixVectorMultiply( &F[ 0][ 0 ], 3, 3, false, &firstPoint[0], &rhs[0] );
                if( !result )
                {
                  TRACE_2( m_hTracer, "[%s] [%s]: Failed to multiply F * firstPoint", __func__, kCameraStrings[ m_cameraID ].c_str() );
                  retValue = false;
                }
                else
                {
                  float64_t secondPoint[3] = { lastFeature.GetUnWarpedPt().x_x, lastFeature.GetUnWarpedPt().y_x, 1.0 };

                  float64_t fundMatError;
                  result = tsc_math::MatrixMultiply( &secondPoint[0], 1, 3, false, &rhs[0], 3, 1, false, &fundMatError );
                  lastFeature.PutFundMatErr( fundMatError );
                  if( !result )
                  {
                    TRACE_2( m_hTracer, "[%s] [%s]: Failed to multiply secondPoint * (F * firstPoint)", __func__, kCameraStrings[ m_cameraID ].c_str() );
                    retValue = false;
                  }
                  else
                  {

                    vf.PutFundMatError( lastFeature.GetFundMatErr() );
                    if ( fabs( vf.GetFundMatError() ) > m_cfg->GetFundMatOutlierThresh() )
                    {
                      // it's an outlier
                      retValue = false;
                    }
                  }
                }
              }
            }
          }
        }
      }
    }

    return retValue;
}

// ----------------------------------------------------------------------------
bool_t StructureFromMotion::BuildFundamentalMatrix()
{
    bool_t retValue = true;
    // F=(inv(K))' * E * inv(K)

    bool_t result = tsc_math::MatrixMultiply( reinterpret_cast<float64_t*>(m_cameraModel->GetIntrinsic().GetKPtr()), 3, 3, true, &E[ 0][ 0 ], 3, 3, false, &F[ 0][ 0 ] );
    if( !result )
    {
        TRACE_2( m_hTracer, "[%s] [%s]: Failed to multiply invK' by E matrix", __func__, kCameraStrings[ m_cameraID ].c_str() );
        retValue = false;
    }
    else
    {
      result = tsc_math::MatrixMultiply( &F[ 0][ 0 ], 3, 3, false, reinterpret_cast<float64_t*>(m_cameraModel->GetIntrinsic().GetKPtr()), 3, 3, false, &F[ 0][ 0 ] );
      if( !result )
      {
        TRACE_2( m_hTracer, "[%s] [%s]: Failed to multiply (invK' x E) by invK", __func__, kCameraStrings[ m_cameraID ].c_str() );
        retValue = false;
      }
    }

    return retValue;
}

#else

bool_t StructureFromMotion::CalculateCorrespondingWorldPt( const fc::Pointd& pt1, const fc::Pointd& pt2, 
                                                            const mecl::core::Matrix<float32_t, 3, 4>& proj1, 
                                                            const mecl::core::Matrix<float32_t, 3, 4>& proj2, 
                                                            fc::Point3d& worldPt ) const
{
    fc::Pointd camPt1;
    m_cameraModel->Image2Camera( &m_cameraObj, pt1, camPt1 );
    fc::Pointd camPt2;
    m_cameraModel->Image2Camera( &m_cameraObj, pt2, camPt2 );

    mecl::core::Matrix<float32_t, 4, 3> A;

    for( uint32_t col = 0; col < 3; ++col )
    {
        A( 0, col ) = proj1( 0, col ) - static_cast<float32_t>(camPt1.x_x) * proj1( 2, col );
        A( 1, col ) = proj1( 1, col ) - static_cast<float32_t>(camPt1.y_x) * proj1( 2, col );
        A( 2, col ) = proj2( 0, col ) - static_cast<float32_t>(camPt2.x_x) * proj2( 2, col );
        A( 3, col ) = proj2( 1, col ) - static_cast<float32_t>(camPt2.y_x) * proj2( 2, col );
    }

    mecl::core::Vector<float32_t, 4>::Config_s vectCfg = 
    {
        - ( proj1( 0, 3 ) - static_cast<float32_t>(camPt1.x_x) * proj1( 2, 3 ) ),
        - ( proj1( 1, 3 ) - static_cast<float32_t>(camPt1.y_x) * proj1( 2, 3 ) ),
        - ( proj2( 0, 3 ) - static_cast<float32_t>(camPt2.x_x) * proj2( 2, 3 ) ),
        - ( proj2( 1, 3 ) - static_cast<float32_t>(camPt2.y_x) * proj2( 2, 3 ) )
    };
    mecl::core::Vector<float32_t, 4> b( vectCfg );

    // 1
    // --- AtA[3 x 3] = A' [3 x 4] x A [4 x 3]
    mecl::core::Matrix3x3<float32_t> AtA = A.t() % A;

    // 2
    // --- inv_AtA [3 x 3] = inv(A' x A)
    mecl::core::Matrix3x3<float32_t> inv_AtA = AtA.inverse();

    // 3
    // Atb [3 x 1] = A' [3 x 4] x b [4 x 1] 
    mecl::core::Matrix<float32_t, 3, 1> Atb = A.t() % b;

    // 4
    // Xw [3 x 1] = inv(A' x A) [3 x 3] x (A' x b) [3 x 1]
    mecl::core::Matrix<float32_t, 3, 1> Xw = inv_AtA % Atb;

    worldPt.x_x = Xw( 0 );
    worldPt.y_x = Xw( 1 );
    worldPt.z_x = Xw( 2 );
    return true;
}

//-------------------------------------------------------------------------
bool_t StructureFromMotion::IsEssMatInlier( ValidFeature& vf )
{
    bool_t retValue = true;

    uint32_t trackSz = vf.GetTrackList().size_u32();

    if( trackSz < 2 )
    {
        retValue = false;
    }
    else
    {

      fc::ImageFeature& lastFeature = vf.GetTrackList().back_ro();
      bool_t result;

      // compute t_km
      {
        t_km[0] = vf.GetMotionVector().GetX();
        t_km[1] = vf.GetMotionVector().GetY();
        t_km[2] = 0;

        float64_t norm = t_km[0] * t_km[0] + t_km[1] * t_km[1];
        norm = sqrt( norm );

        if( norm > FLT_EPSILON ) // PRQA S 5053
        {    
            t_km[0] /= norm;
            t_km[1] /= norm;
        }
      }

      // compute R_km

      camera_model::CameraModelMecl::BuildZAxisRotationMatrix( tsc_math::Radians2Degrees( vf.GetMotionVector().GetPsi() ), R_km );

      // compute t_c = R_ig * t_km
      result = tsc_math::MatrixVectorMultiply( &R[ 0][ 0 ], 3, 3, false, &t_km[ 0 ], &t_c[ 0 ] );
      if( !result )
      {
          TRACE_2( m_hTracer, "[%s] [%s]: Failed to compute t_c = R x t_km", __func__, kCameraStrings[ m_cameraID ].c_str() );
          retValue = false;
      }
      else

      // calc R_c = R * R_km * R'
      {
        result = tsc_math::MatrixMultiply( &R[ 0][ 0 ], 3, 3, false, &R_km[ 0][ 0 ], 3, 3, false, &R_c[ 0][ 0 ] );
        if( !result )
        {
            TRACE_2( m_hTracer, "[%s] [%s]: Failed to multiply R x R_km", __func__, kCameraStrings[ m_cameraID ].c_str() );
            retValue = false;
        }
        else
        {

          result = tsc_math::MatrixMultiply( &R_c[ 0][ 0 ], 3, 3, false, &R[ 0][ 0 ], 3, 3, true, &R_c[ 0][ 0 ] );
          if( !result )
          {
            TRACE_2( m_hTracer, "[%s] [%s]: Failed to multiply (R x R_km) x R'", __func__, kCameraStrings[ m_cameraID ].c_str() );
            retValue = false;
          }
          else
          {

            // Essential Matrix
            result = BuildEssentialMatrix();
            if( !result )
            {
              TRACE_2( m_hTracer, "[%s] [%s]: Failed to BuildEssentialMatrix", __func__, kCameraStrings[ m_cameraID ].c_str() );
              retValue = false;
            }
            else
            {

                // test mc_2 * F * pt1 < m_fundMatOutlierThresh
                const fc::ImageFeature& firstFeature = vf.GetTrackList().front_ro();

                fc::Pointd tmpCamPt;
                camera_model::CameraModelMecl::Image2Camera( &m_cameraObj, firstFeature.GetUnWarpedPt(), tmpCamPt );
                float64_t camPt1[3] = { tmpCamPt.x_x, tmpCamPt.y_x, 1 };
                camera_model::CameraModelMecl::Image2Camera( &m_cameraObj, lastFeature.GetUnWarpedPt(), tmpCamPt );
                float64_t camPt2[3] = { tmpCamPt.x_x, tmpCamPt.y_x, 1 };

                float64_t rhs[3];
                result = tsc_math::MatrixVectorMultiply( &E[ 0][ 0 ], 3, 3, false, &camPt1[0], &rhs[0] );
                if( !result )
                {
                  TRACE_2( m_hTracer, "[%s] [%s]: Failed to multiply E * camPt1", __func__, kCameraStrings[ m_cameraID ].c_str() );
                  retValue = false;
                }
                else
                {
                  float64_t fundMatError;
                  result = tsc_math::MatrixMultiply( &camPt2[0], 1, 3, false, &rhs[0], 3, 1, false, &fundMatError );
                  lastFeature.PutFundMatErr( fundMatError );
                  if( !result )
                  {
                    TRACE_2( m_hTracer, "[%s] [%s]: Failed to multiply secondPoint * (E * camPt1)", __func__, kCameraStrings[ m_cameraID ].c_str() );
                    retValue = false;
                  }
                  else
                  {

                    vf.PutFundMatError( lastFeature.GetFundMatErr() );
                    if ( fabs( vf.GetFundMatError() ) > m_cfg->GetFundMatOutlierThresh() )
                    {
                      // it's an outlier
                      retValue = false;
                    }
                  }
                }
            }
          }
        }
      }
    }

    return retValue;
}
#endif

}
// ----------------------------------------------------------------------------
#endif
