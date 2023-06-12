// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [08-Jul-2013]
// --- Modified by Ehsan Parvizi [25-Jul-2013]
// --- Modified by Hany Kashif [25-Sep-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2013
// ----------------------------------------------------------------------------
#ifndef __STRUCTUREFORMOTION_H_
#define __STRUCTUREFORMOTION_H_
// ----------------------------------------------------------------------------
#include "featureCollectionStructs.h"
#include "tscAlgStructs.h"

#ifdef ENABLE_SFM   // PRQA S 1070
namespace tsc
{
class TSCConfig;
// ----------------------------------------------------------------------------
// --- StructureFromMotion
class StructureFromMotion
{
    public:
    StructureFromMotion( );
    ~StructureFromMotion();
    bool_t Init( tscApi::enuCameraID cameraID );
    bool_t Process( fc::ValidFeatureCollection& validFeatureCollection, uint32_t frameNumber, fc::InitialGuess& ig );

    private:
    // --- Calculate Corresponding World Point
    sint64_t m_hTracer;
    tscApi::enuCameraID m_cameraID;
    TSCConfig* m_cfg;
    static bool_t loadConfiguration_b( void );
    bool_t BuildEssentialMatrix();
    void FilterByWorldDistance( fc::ValidFeatureCollection& validFeatureCollection );
#ifdef USE_SVSCM
    bool_t CalculateCorrespondingWorldPt( const fc::Pointd& pt1, const fc::Pointd& pt2, float64_t (&proj1)[ 3][ 4 ], float64_t (&proj2)[ 3][ 4 ], fc::Point3d& worldPt ) const;
    bool_t IsFundMatInlier( fc::ValidFeature& vf );
    bool_t BuildFundamentalMatrix();
#else
    bool_t CalculateCorrespondingWorldPt( const fc::Pointd& pt1, const fc::Pointd& pt2, const mecl::core::Matrix<float32_t, 3, 4>& proj1, 
                                          const mecl::core::Matrix<float32_t, 3, 4>& proj2, fc::Point3d& worldPt ) const;
    bool_t IsEssMatInlier( fc::ValidFeature& vf );
#endif
    mecl::core::ArrayList < fc::ValidFeature, tsc_cfg::NUM_VALID_FEATURES > tempVF;

    // --- implementation specific data members
    tscApi::CalibrationParams m_initialCalibParams;

    float64_t Rg[ 3][ 3 ];
    float64_t Rb[ 3][ 3 ];
    float64_t Ra[ 3][ 3 ];
    float64_t R[ 3][ 3 ];
    float64_t T[ 3][ 4 ];

    float64_t R_km[3][3];
    float64_t t_km[3];
    float64_t R_c[3][3];
    float64_t t_c[3];

    float64_t E[3][3];

#ifdef USE_SVSCM
    camera_model::CameraModel* m_cameraModel;
    float64_t P1[ 3][ 4 ];
    float64_t P2[ 3][ 4 ];
    float64_t MotionTrans[ 4][ 4 ];
    float64_t F[3][3];
#else
    camera_model::CameraModelMecl* m_cameraModel;
    mecl::model::Camera<float32_t> m_cameraObj;  // local cameraObj
    mecl::core::Matrix<float32_t, 4, 4> MotionTransMat;
    mecl::core::Matrix<float32_t, 3, 4> PMat1;
    mecl::core::Matrix<float32_t, 3, 4> PMat2;
#endif
};
}
// ----------------------------------------------------------------------------
#endif
#endif
