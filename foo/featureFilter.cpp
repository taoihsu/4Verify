// ----------------------------------------------------------------------------
// --- Written by Rathi G. R. [08-Jul-2013]
// --- Modified by Ehsan Parvizi [09-Oct-2014]
// --- Modified by Hany Kashif [31-Oct-2014]
// --- Modified by Dmitri Kelbas [05-Nov-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#include "stdafx.h"
#include <stdlib.h> // rand
#include "mecl/mecl.h"
#include "mathOperations.h"
#include "featureCollection.h"
#include "localFeatureCollector.h"
#include "featureFilter.h"
#include "cameraModel.h"
#include <cfloat> // DBL_MIN
#include "ss.h"
// ----------------------------------------------------------------------------
// PRQA S 3063 EOF
// PRQA S 3708 EOF
using ff::FeatureFilter;
using fc::ImageFeature;
using fc::FeatureTrack;
using fc::LocalFeatureTrackCollection;
// SB using std::fabs;
// SB using std::sqrt;
// SB using std::atan;
// SB using std::asin;
// SB using std::sin;
// SB using std::cos;
// SB using std::string;
using tsc_math::MatrixVectorMultiply;
using tsc_math::Radians2Degrees;
using tsc_trace::kCameraStrings;
using namespace std;
// ----------------------------------------------------------------------------
FeatureFilter::FeatureFilter( ):
    rpo_po( NULL ),
    initOK_b( false ),
    c_Img_pu8( NULL ),
    hTracer_u64( 0 ),
    minPixelMotionThresh_u32( 0 ),
    currFrameNum_u32( 0 ),
    cameraModel_po( NULL ),
    trackLengthThresh_u32( 0 ),
    slopeDifferenceThresh_f64( 0 ),
    useSfmFilter_b( false ),
    maxHeightDiffMm_f64( 1000000.0 ),
    angleThreshIGDeg_f32( 5.0 ),
    deviationPercentageIG_u32( 100 )
{
    // --- initialize the data members
}

// ----------------------------------------------------------------------------
#ifdef DEBUG_TSC_ALG
bool_t FeatureFilter::Init( tscApi::enuCameraID i_CameraID_t, fc::ROI* i_ROI_po, const uint8_t* i_Img_pu8, fc::DiagnosticData* i_DiagnosticData_po, tscApi::DebugCounters* pDebugCounters)
#else
//PRQA S 6200 1 //const not added to input pointer variables, otherwise assignment produces error
bool_t FeatureFilter::Init( tscApi::enuCameraID i_CameraID_t, fc::ROI* i_ROI_po, const uint8_t* i_Img_pu8, fc::DiagnosticData* i_DiagnosticData_po)
#endif
{
    bool_t v_Ret_b = true;
    rpo_po = i_ROI_po;
    cameraID_t = i_CameraID_t;
    tscTargetCameraID_t = i_DiagnosticData_po->tscCameraTarget;
    c_Img_pu8 = i_Img_pu8;
    diagnosticData_po = i_DiagnosticData_po;
#ifdef DEBUG_TSC_ALG
    m_pDebugCounters = pDebugCounters;
#endif
    hTracer_u64 = fc::FeatureCollection::getInstance_rt().getTracer_u64();
    size_t v_Ind_t = static_cast<size_t>(i_CameraID_t);
    cameraModel_po = fc::FeatureCollection::getInstance_rt().getModuleInfo_rt().getMAddrCmrMdls_po();
    cameraModel_po += v_Ind_t;
    TRACE_2( hTracer_u64, "[%s] FF[%s] Feature Filter Object Initialized", kCameraStrings[ cameraID_t ].c_str(), tsc_cfg::MODULE_NAME_FEATUREFILTER );

    if( !loadConfiguration_b() )
    {
        initOK_b = false;
        TRACE_2( hTracer_u64, "[%s] FF[%s] Init: Failed to load configuration", kCameraStrings[ cameraID_t ].c_str(), tsc_cfg::MODULE_NAME_FEATUREFILTER );
        v_Ret_b = false;
    }
    else
    // --- collect the needed variables for convenient usage
    if( c_Img_pu8 == NULL )
    {
        initOK_b = false;
        TRACE_2( hTracer_u64, "[%s] FF[%s] Init: Failed to load the image buffer", kCameraStrings[ cameraID_t ].c_str(), tsc_cfg::MODULE_NAME_FEATUREFILTER );
        v_Ret_b = false;
    } else
    {
        // --- initialize implementation specific data structures
        initOK_b = true;
    }

    // Windows Logging
    std::string camera;

    switch( cameraID_t )
    {
        case 0:
            camera = "FRONT";
            break;

        case 1:
            camera = "LEFT";
            break;

        case 2:
            camera = "REAR";
            break;

        case 3:
            camera = "RIGHT";
            break;

        default:
            camera = "Unknown";
            break;
    }

    if( tscTargetCameraID_t == cameraID_t )
    {
        m_logFileName = camera + "_" + std::to_string( cameraID_t ) + "_FeatureFilter.csv";
        m_logHeader.clear();
        m_logHeader
                << "Frame, "
                << "Trk1_Pt1_x, "
                << "Trk1_Pt1_y, "
                << "Trk1_Pt2_x, "
                << "Trk1_Pt2_y, "
                << "Trk2_Pt1_x, "
                << "Trk2_Pt1_y, "
                << "Trk2_Pt2_x, "
                << "Trk2_Pt2_y, "
                << "thetaDeviation, "
                << "theta_car, "
                << "theta_avg, "
                << "S1, "
                << "Feature S1, "
                << "expected/actual, "
                << "\n";
    }

    return v_Ret_b;
}

//-------------------------------------------------------------------------

FeatureFilter::~FeatureFilter()
{
    if( tscTargetCameraID_t == cameraID_t )
    {
        std::stringstream header( "Elapsed Time\n" );
        AppCtrl::WriteToFile( "IG_ElapsedTime.csv", m_initialGuessTimeStream, header, true );
        AppCtrl::WriteToFile( m_logFileName, m_logStream, m_logHeader, false );
    }
}

// ----------------------------------------------------------------------------
bool_t FeatureFilter::Process( uint32_t i_CurrFrameNum_u32, lfc::LocalFeatureCollector *b_LFC_po )
{
    bool_t v_Ret_b = true;
    if( !initOK_b )
    {
        TRACE_2( hTracer_u64, "[%s] FF[%s] not initialized. Cannot process further", kCameraStrings[ cameraID_t ].c_str(), tsc_cfg::MODULE_NAME_FEATUREFILTER );
        v_Ret_b = false;
    }
    else
    {
        // --- store current frame number
        currFrameNum_u32 = i_CurrFrameNum_u32;

        trackLengthThresh_u32 = b_LFC_po->getTrackLengthThreshold_u32();
        LocalFeatureTrackCollection *v_LFT_po = b_LFC_po->getLocalFeatureTrackCollection_po();

        // --- process this FF
        CREATE_TIMER( t );
        START_TIMER( t );
        TRACE_3( hTracer_u64, "[%s] FF[%s] Initiate processing Frame %lu...", kCameraStrings[ cameraID_t ].c_str(), tsc_cfg::MODULE_NAME_FEATUREFILTER, currFrameNum_u32 );

        // --- actual implementation for FF

        // -- clear m_initialGuesses and validIndices_x from past
        validIndices_x.clear_v();
        initialGuesses_x.clear_v();

        size_t v_MaxIdx_t = v_LFT_po->getTrcks_rx().size_u32();
        for( size_t v_Idx_t = 0; v_Idx_t < v_MaxIdx_t; ++v_Idx_t )
        {
            FeatureTrack& v_Ft_ro = v_LFT_po->getTrcks_rx()[v_Idx_t];
            if( v_Ft_ro.getIsTerm_b() )
            {
                // --- First check if has minimum track length
                size_t v_TrackSz_t = v_Ft_ro.getTrckLst_rx().size_u32();
                if( v_TrackSz_t < trackLengthThresh_u32 )
                {
                    v_Ft_ro.PutIsValid(false);
                    diagnosticData_po->incInvalidFeatures_v();
#ifdef DEBUG_TSC_ALG
                    m_pDebugCounters->invalidTrckLen ++;
#endif
                    TRACE_4( hTracer_u64, "[%s] track starting at (%u, %u) does not have enough features, only got %u features",  \
                        tsc_cfg::MODULE_NAME_FEATUREFILTER, v_Ft_ro.getTrckLst_rx().front_ro().getPt_t().x_x, v_Ft_ro.getTrckLst_rx().front_ro().getPt_t().y_x, v_TrackSz_t );
                    continue;
                }

                // --- Filter by pixel motion
                bool_t v_HasMinPixelMotion_b = CheckMinPixelMotion( v_Ft_ro, minPixelMotionThresh_u32 );
                if( !v_HasMinPixelMotion_b )
                {
                    v_Ft_ro.PutIsValid(false);
                    diagnosticData_po->incInvalidFeatures_v();
#ifdef DEBUG_TSC_ALG
                    m_pDebugCounters->invalidPxMotion ++;
#endif
                    TRACE_5( hTracer_u64, "[%s] track starting at (%u, %u) ending at (%u, %u) does not meet min pixel motion thresh",
                        tsc_cfg::MODULE_NAME_FEATUREFILTER, v_Ft_ro.getTrckLst_rx().front_ro().getPt_t().x_x, v_Ft_ro.getTrckLst_rx().front_ro().getPt_t().y_x,
                                                            v_Ft_ro.getTrckLst_rx().back_ro().getPt_t().x_x, v_Ft_ro.getTrckLst_rx().back_ro().getPt_t().y_x );
                    continue;
                }

                // --- Unwarp and Filter by NaN zone
                if( !UnwarpFeatureTrack( v_Ft_ro ) )
                {
                    v_Ft_ro.PutIsValid(false);
                    diagnosticData_po->incInvalidFeatures_v();
                    continue;
                }

                // --- Filter by slope check
                if( !CheckValidSlope( v_Ft_ro, false ) )
                {
                    v_Ft_ro.PutIsValid(false);
                    diagnosticData_po->incInvalidFeatures_v();
#ifdef DEBUG_TSC_ALG
                    m_pDebugCounters->invalidValidSlope ++;
#endif
                    continue;
                }

                v_Ft_ro.getFltrdIndcs_rx().clear_v();
                v_Ft_ro.getFltrdIndcs_rx().pushBack_v( 0 );
                v_Ft_ro.getFltrdIndcs_rx().pushBack_v( v_TrackSz_t - 1 );
                validIndices_x.pushBack_v( v_Idx_t );
                // update its ROI performance
                b_LFC_po->updateROIPerformance_u32( v_Ft_ro.getRoiI_u32(), true );
                // protection of ArrayList assertion
                if (validIndices_x.size_u32() == 4)
                {
                    break;
                }
            }
        }

        if( useSfmFilter_b )
        {
            ApplySfMFilter( v_LFT_po->getTrcks_rx() );
        }

        // --- compute initial guess if at least there's a pair
        if ( validIndices_x.size_u32() < 2 )
        {
            diagnosticData_po->addIgnoredValidFeatures_v(validIndices_x.size_u32());
#ifdef DEBUG_TSC_ALG
            m_pDebugCounters->ignoredMissingPair ++;
#endif
            TRACE_4( hTracer_u64, "[%s] [%s]: Not enough filtered features [%lu] ending in this frame [%lu]",  \
                kCameraStrings[ cameraID_t ].c_str(), tsc_cfg::MODULE_NAME_FEATUREFILTER, validIndices_x.size_u32(), i_CurrFrameNum_u32 );
            validIndices_x.clear_v();
            STOP_TIMER( t );
            TRACE_4( hTracer_u64, "[%s] FF[%s]: Processed Frame %lu in Total [%.1f]ms",kCameraStrings[ cameraID_t ].c_str(), tsc_cfg::MODULE_NAME_FEATUREFILTER, currFrameNum_u32, GET_ELAPSED_TIME(t) );
            v_Ret_b = true;
        }else
        {
            // --- compute initial guess if at least there's a pair
            if( useInitialGuessCombinations_b )
            {
                bool_t v_Result_b;
                v_Result_b = ComputeInitialGuessCombinations( *v_LFT_po );
                if ( !v_Result_b )
                {
                    TRACE_4( hTracer_u64, "[%s] [%s]: No initial guess combinations was computed for the [%lu] filtered features ending in this frame [%lu]",  \
                       kCameraStrings[ cameraID_t ].c_str(), tsc_cfg::MODULE_NAME_FEATUREFILTER, validIndices_x.size_u32(), i_CurrFrameNum_u32 );

                    diagnosticData_po->addIgnoredValidFeatures_v(validIndices_x.size_u32());
                    validIndices_x.clear_v();
                }
            }
            else
            {
                bool_t v_Result_b;
                v_Result_b = ComputeInitialGuess( *v_LFT_po );
                if ( !v_Result_b )
                {
                    TRACE_4( hTracer_u64, "[%s] [%s]: No initial guess was computed for the [%lu] filtered features ending in this frame [%lu]",  \
                       kCameraStrings[ cameraID_t ].c_str(), tsc_cfg::MODULE_NAME_FEATUREFILTER, validIndices_x.size_u32(), i_CurrFrameNum_u32 );
                    diagnosticData_po->addIgnoredValidFeatures_v(validIndices_x.size_u32());
                    validIndices_x.clear_v();
                }
            }

            STOP_TIMER( t );
            TRACE_4( hTracer_u64, "[%s] FF[%s]: Processed Frame %lu in Total [%.1f]ms", kCameraStrings[ cameraID_t ].c_str(), tsc_cfg::MODULE_NAME_FEATUREFILTER, currFrameNum_u32, GET_ELAPSED_TIME(t) );
        }
    }

    return v_Ret_b;
}

//-------------------------------------------------------------------------

bool_t FeatureFilter::CheckMinPixelMotion( FeatureTrack& b_Ft_ro, uint32_t i_Thresh_u32 )
{
    bool_t v_Ret_b = true;
    const fc::Point &c_Front_rt = b_Ft_ro.getTrckLst_rx().front_ro().getPt_t();
    const fc::Point &c_Back_rt = b_Ft_ro.getTrckLst_rx().back_ro().getPt_t();
    uint32_t v_XShift_u32 = static_cast<uint32_t>(/*std::*/mecl::math::abs_x<float32_t>(static_cast<float32_t>(c_Back_rt.x_x - c_Front_rt.x_x)));
    uint32_t v_YShift_u32 = static_cast<uint32_t>(/*std::*/mecl::math::abs_x<float32_t>(static_cast<float32_t>(c_Back_rt.y_x - c_Front_rt.y_x)));

    if( ( v_XShift_u32 * v_XShift_u32 + v_YShift_u32 * v_YShift_u32 ) < (i_Thresh_u32 * i_Thresh_u32) )
    {
        v_Ret_b = false;
    }

    return v_Ret_b;
}

//-------------------------------------------------------------------------

bool_t FeatureFilter::UnwarpFeatureTrack( FeatureTrack& b_Ft_ro ) const
{
    bool_t v_Ret_b = true;
    mecl::core::ArrayList < ImageFeature, tsc_cfg::TRACK_LENGTH >::iterator v_It_t = b_Ft_ro.getTrckLst_rx().rwBegin_o();
    mecl::core::ArrayList < ImageFeature, tsc_cfg::TRACK_LENGTH >::const_iterator v_Endlp_t = b_Ft_ro.getTrckLst_rx().end_o();
    for( ; v_It_t != v_Endlp_t; ++v_It_t )
    {
        ImageFeature& v_Feature_ro = *v_It_t;
        if( !v_Feature_ro.getIsUnWarped_b() )
        {
            fc::Pointd v_TmpUnWarped_x;
            if( !cameraModel_po->Unwarp( v_Feature_ro.getPt_t(), v_TmpUnWarped_x ) )
            {
                TRACE_5( hTracer_u64, "[%s] FF[%s] Unable to unwarp feature #[%lu] at ( %d , %d )", \
                kCameraStrings[ cameraID_t ].c_str(), tsc_cfg::MODULE_NAME_FEATUREFILTER, 0, \
                v_Feature_ro.getPt_t().x_x, v_Feature_ro.getPt_t().y_x);
                v_Ret_b = false;
            }
            else
            {
                v_Feature_ro.PutIsUnWarped(true);
            }
            v_Feature_ro.PutUnWarpedPt(v_TmpUnWarped_x); 
        }
        if( v_Ret_b == false )
        {
            break;
        }
    }
    return v_Ret_b;
}

//-------------------------------------------------------------------------

bool_t FeatureFilter::CheckValidSlope( FeatureTrack& b_Ft_ro, bool_t i_UseRatio_b) const
{
    bool_t v_Ret_b = true;

    ImageFeature& v_First_ro = b_Ft_ro.getTrckLst_rx().front_ro();
    ImageFeature& v_Last_ro = b_Ft_ro.getTrckLst_rx().back_ro();
   
    float32_t v_Slope_f32 = static_cast<float32_t>( v_Last_ro.getUnWarpedPt_x().y_x - v_First_ro.getUnWarpedPt_x().y_x ) / static_cast<float32_t>( v_Last_ro.getUnWarpedPt_x().x_x - v_First_ro.getUnWarpedPt_x().x_x );
    v_First_ro.PutAngle( v_Slope_f32 );
    TRACE_3( hTracer_u64, "[%s] [CheckValidSlope]: Frame [%lu] Overall Track slope [0] = [%f]", kCameraStrings[ cameraID_t ].c_str(), currFrameNum_u32, v_First_ro.getAngle_f32() );

    size_t v_MaxIdx_t = b_Ft_ro.getTrckLst_rx().size_u32();
    for( size_t v_Idx_t = 1; v_Idx_t < v_MaxIdx_t; ++v_Idx_t )
    {
        const ImageFeature& c_Prev_ro = b_Ft_ro.getTrckLst_rx()[ v_Idx_t - 1 ];
        ImageFeature& v_Curr_ro = b_Ft_ro.getTrckLst_rx()[ v_Idx_t ];

        // if same point, it must be a mistrack
        if ( (v_Curr_ro.getPt_t().x_x == c_Prev_ro.getPt_t().x_x) && (v_Curr_ro.getPt_t().y_x == c_Prev_ro.getPt_t().y_x) )
        {
            v_Ret_b = false;
            TRACE_3( hTracer_u64, "[%s] FF CheckValidSlope failed at ( %d , %d ) for mistrack", \
            kCameraStrings[ cameraID_t ].c_str(), c_Prev_ro.getPt_t().x_x, c_Prev_ro.getPt_t().y_x);
        }
        else
        {
            // Calculate the slope between each point
            v_Slope_f32 = static_cast< float32_t >( v_Curr_ro.getUnWarpedPt_x().y_x - c_Prev_ro.getUnWarpedPt_x().y_x ) / static_cast< float32_t >( v_Curr_ro.getUnWarpedPt_x().x_x - c_Prev_ro.getUnWarpedPt_x().x_x );
            // --- store the slope into the feature angle (equivalent to slope)
            v_Curr_ro.PutAngle(v_Slope_f32);

            float32_t v_SlopeDiff_f32;
            if (i_UseRatio_b)
            {
                v_SlopeDiff_f32 = v_Curr_ro.getAngle_f32() / v_First_ro.getAngle_f32() - 1.0F;
                TRACE_4( hTracer_u64, "[%s] [CheckVaildSlope]: Frame [%lu] Track slope [%u] ratio = [%f]", kCameraStrings[ cameraID_t ].c_str(), currFrameNum_u32, v_Idx_t, v_SlopeDiff_f32 );
            }
            else
            {
                v_SlopeDiff_f32 = v_Curr_ro.getAngle_f32() - c_Prev_ro.getAngle_f32();
                TRACE_4( hTracer_u64, "[%s] [CheckVaildSlope]: Frame [%lu] Track slope [%u] [%f]", kCameraStrings[ cameraID_t ].c_str(), currFrameNum_u32, v_Idx_t, v_SlopeDiff_f32 );
            }

            // Now compare all the slopes to the Thresholds
            if  ( (mecl::math::abs_x<float32_t>( v_SlopeDiff_f32 ) > slopeDifferenceThresh_f64) )
            {
                v_Ret_b = false;
                TRACE_5( hTracer_u64, "[%s] [CheckValidSlope] failed at slope thresh starting ( %d , %d ) with angle (%.3f, %.3f)", \
                kCameraStrings[ cameraID_t ].c_str(), c_Prev_ro.getPt_t().x_x, c_Prev_ro.getPt_t().y_x, c_Prev_ro.getAngle_f32(), v_Curr_ro.getAngle_f32());
            }
        }
        if( !v_Ret_b )
        {
            break;
        }
    }

    if( v_Ret_b && (b_Ft_ro.getTrckLst_rx().size_u32() > 3) &&  \
        (mecl::math::abs_x<float32_t>( v_Last_ro.getAngle_f32() - b_Ft_ro.getTrckLst_rx()[1].getAngle_f32() ) > slopeDifferenceThresh_f64 ))
    {
        v_Ret_b = false;
        TRACE_5( hTracer_u64, "[%s] [CheckValidSlope] failed at slope thresh starting ( %d , %d ) with angle (%.3f, %.3f)", \
        kCameraStrings[ cameraID_t ].c_str(), b_Ft_ro.getTrckLst_rx().front_ro().getPt_t().x_x, b_Ft_ro.getTrckLst_rx().front_ro().getPt_t().y_x, b_Ft_ro.getTrckLst_rx()[1].getAngle_f32(), v_Last_ro.getAngle_f32());
    }


    return v_Ret_b;
}

template<typename T>
//-----------------------------------------------------------------------------
sint32_t FeatureFilter::givensEx(T (&i_A_ax)[2], T (&i_B_ax)[2], sint32_t i_N_s32, T i_C_x, T i_S_x) {
    sint32_t v_K_s32 = 0;
    T v_C20_x = i_C_x;
    T v_C21_x = i_C_x;
    T v_S20_x = i_S_x;
    T v_S21_x = i_S_x;

    for (v_K_s32 = 0; v_K_s32 <= (i_N_s32 - 2); v_K_s32 += 2) // n is always 2, hence 'k' can be omitted
    {
        T v_A00_x = i_A_ax[v_K_s32 + 0];
        T v_A01_x = i_A_ax[v_K_s32 + 1];
        T v_B00_x = i_B_ax[v_K_s32 + 0];
        T v_B01_x = i_B_ax[v_K_s32 + 1];

        T v_T00_x = v_A00_x * v_C20_x + v_B00_x * v_S20_x;
        T v_T01_x = v_A01_x * v_C21_x + v_B01_x * v_S21_x;
        T v_T10_x = v_B00_x * v_C20_x - v_A00_x * v_S20_x;
        T v_T11_x = v_B01_x * v_C21_x - v_A01_x * v_S21_x;

        i_A_ax[v_K_s32 + 0] = v_T00_x;
        i_A_ax[v_K_s32 + 1] = v_T01_x;
        i_B_ax[v_K_s32 + 0] = v_T10_x;
        i_B_ax[v_K_s32 + 1] = v_T11_x;
    }
    return v_K_s32;
}

template<typename T>
T FeatureFilter::hypotEx(T i_A_t, T i_B_t) {
    T v_RetVal_x = 0.0;

    i_A_t = mecl::math::abs_x<T>(i_A_t);
    i_B_t = mecl::math::abs_x<T>(i_B_t);
    if (i_A_t > i_B_t) {
        i_B_t /= i_A_t;
        v_RetVal_x = static_cast<T>(i_A_t * mecl::math::algebra<T>::sqrt_x(1.0 + i_B_t * i_B_t));
    }
    else if (i_B_t > 0) {
        i_A_t /= i_B_t;
        v_RetVal_x = static_cast<T>(i_B_t * mecl::math::algebra<T>::sqrt_x(1.0 + i_A_t * i_A_t));
    }
    else
    {
    }
    return v_RetVal_x;
}

// ----------------------------------------------------------------------------
//! computes norm of the selected array part
template<typename T>
T FeatureFilter::normEx(T const (&i_Src_rax)[3]) {
    T v_Result_x = 0.0;

    v_Result_x = i_Src_rax[0] * i_Src_rax[0] + i_Src_rax[1] * i_Src_rax[1] + i_Src_rax[2] * i_Src_rax[2];
    v_Result_x = sqrt(v_Result_x);
    return v_Result_x;
}

//! scales and shifts array elements so that either the specified norm (alpha) or the minimum (alpha) and maximum (beta) array values get the specified values
template<typename T>
void FeatureFilter::normalizeEx(T const (&i_Src_rax)[3], T (&o_Dst_ax)[3]) {
    T v_A_x = 1.;
    T v_Scale_x = 1.;

    v_Scale_x = normEx(i_Src_rax);
    v_Scale_x = (v_Scale_x > mecl::math::numeric_limits<T>::epsilon_x()) ? v_A_x / v_Scale_x : 0.; // DBL_EPSILON

    o_Dst_ax[0] = v_Scale_x * i_Src_rax[0];
    o_Dst_ax[1] = v_Scale_x * i_Src_rax[1];
    o_Dst_ax[2] = v_Scale_x * i_Src_rax[2];
}


// ----------------------------------------------------------------------------
template<typename T>
bool_t FeatureFilter::SVDcomputeEx(T const (&i_Aarr_ax)[2][3], T (&o_W_ax)[3]) {   // PRQA S 4212
    bool_t v_Ret_b = true;
    uint8_t v_M_u8 = 3;
    uint8_t v_N_u8 = 2;

    T v_Temp_ax[3][3] = { { 0., 0., 0. }, { 0., 0., 0. }, { 0., 0., 0. } };

    memcpy(&v_Temp_ax[0][0], &i_Aarr_ax[0][0], sizeof(T)*v_N_u8*v_M_u8);

    //-- JacobiSVD(temp_a, temp_w, temp_v, m=3, n=2, urows=3)
    {
        //-- static void JacobiSVD(double* At, size_t astep, double* W, double* Vt, size_t vstep, int m, int n, int n1=-1)
        //-- JacobiSVDImpl_(At, astep, W, Vt, vstep, m, n, n1, DBL_MIN, DBL_EPSILON*10);
        {
            T v_TempV_ax[2][2] = { { 0., 0. }, { 0., 0. } };
            uint8_t v_N1_u8 = 3;
            T v_Minval_x = DBL_MIN;
            T v_Eps_x = mecl::math::numeric_limits<T>::epsilon_x()  * 10.0; // DBL_EPSILON * 10 http://stackoverflow.com/questions/1566198/how-to-portably-get-dbl-epsilon-in-c-c
            T v_W_ax[2];
            uint8_t v_Index_u8 = 0;
            uint8_t v_InnerIndex_u8 = 0;
            uint8_t v_Idx_u8 = 0;
            uint8_t v_Iter_u8 = 0;
            uint8_t v_MaxIter_u8 = 3;
            T v_C_x = 0.0;
            T v_S_x = 0.0;
            T v_SD_x = 0.0;

            for (v_Index_u8 = 0; v_Index_u8 < v_N_u8; v_Index_u8++) {
                v_SD_x = 0;
                for (v_Idx_u8 = 0; v_Idx_u8 < v_M_u8; v_Idx_u8++) {
                    T v_T_x = v_Temp_ax[v_Index_u8][v_Idx_u8]; //-- At(i*astep + k)
                    v_SD_x += static_cast<T>(v_T_x * v_T_x);
                }
                v_W_ax[v_Index_u8] = v_SD_x;

                for (v_Idx_u8 = 0; v_Idx_u8 < v_N_u8; v_Idx_u8++) {
                    v_TempV_ax[v_Index_u8][v_Idx_u8] = 0; //-- Vt(i*vstep + k)
                }
                v_TempV_ax[v_Index_u8][v_Index_u8] = 1.0; //-- Vt(i*vstep + i)
            }

            for (v_Iter_u8 = 0; v_Iter_u8 < v_MaxIter_u8; v_Iter_u8++) {
                bool_t v_Changed_b = false;
                T v_A_x = v_W_ax[0];
                T v_B_x = v_W_ax[1];
                T v_P_x = 0.0;

                for (v_Idx_u8 = 0; v_Idx_u8 < v_M_u8; v_Idx_u8++) {
                    v_P_x += static_cast<T>(v_Temp_ax[0][v_Idx_u8] * v_Temp_ax[1][v_Idx_u8]);
                }
                if (mecl::math::abs_x<T>(v_P_x) <= (v_Eps_x * sqrt(static_cast<T>(v_A_x * v_B_x))))
                {
                    continue;
                }

                v_P_x *= 2.0;
                T v_Beta_x = v_A_x - v_B_x;
                T v_Gamma_x = hypotEx(static_cast<T>(v_P_x), v_Beta_x);
                if (v_Beta_x < 0) {
                    T v_Delta_x = (v_Gamma_x - v_Beta_x) * 0.5;
                    v_S_x = static_cast<T>( sqrt(v_Delta_x / v_Gamma_x));
                    v_C_x = static_cast<T>(v_P_x / (v_Gamma_x * v_S_x * 2.0));
                } else {
                    v_C_x = static_cast<T>(sqrt((v_Gamma_x + v_Beta_x) / (v_Gamma_x * 2.0)));
                    v_S_x = static_cast<T>(v_P_x / (v_Gamma_x * v_C_x * 2.0));
                }

                v_A_x = 0;
                v_B_x = 0;
                for (v_Idx_u8 = 0; v_Idx_u8 < v_M_u8; v_Idx_u8++) {
                    T v_T0_x = v_C_x * v_Temp_ax[0][v_Idx_u8] + v_S_x * v_Temp_ax[1][v_Idx_u8];
                    T v_T1_x = -v_S_x * v_Temp_ax[0][v_Idx_u8] + v_C_x * v_Temp_ax[1][v_Idx_u8];
                    v_Temp_ax[0][v_Idx_u8] = v_T0_x;
                    v_Temp_ax[1][v_Idx_u8] = v_T1_x;

                    v_A_x += v_T0_x * v_T0_x;
                    v_B_x += v_T1_x * v_T1_x;
                }
                v_W_ax[0] = v_A_x;
                v_W_ax[1] = v_B_x;

                v_Changed_b = true;

                T (&v_Vi_rax)[2] = v_TempV_ax[0]; //-- Vt(i*vstep)
                T (&v_Vj_rax)[2] = v_TempV_ax[1]; //-- Vt(j*vstep)

                for (v_Idx_u8 = givensEx(v_Vi_rax, v_Vj_rax, v_N_u8, v_C_x, v_S_x); v_Idx_u8 < v_N_u8; v_Idx_u8++) {
                    T v_T0_x = v_C_x * v_TempV_ax[0][v_Idx_u8] + v_S_x * v_TempV_ax[1][v_Idx_u8];
                    T v_T1_x = -v_S_x * v_TempV_ax[0][v_Idx_u8] + v_C_x * v_TempV_ax[1][v_Idx_u8];
                    v_TempV_ax[0][v_Idx_u8] = v_T0_x;
                    v_TempV_ax[1][v_Idx_u8] = v_T1_x;
                }
                if (!v_Changed_b) {
                    break;
                }
            }

            for (v_Index_u8 = 0; v_Index_u8 < v_N_u8; v_Index_u8++) {
                v_SD_x = 0;
                for (v_Idx_u8 = 0; v_Idx_u8 < v_M_u8; v_Idx_u8++) {
                    T v_T_x = v_Temp_ax[v_Index_u8][v_Idx_u8];
                    v_SD_x += v_T_x * v_T_x;
                }
                v_W_ax[v_Index_u8] = sqrt(v_SD_x);
            }

            for (v_Index_u8 = 0; v_Index_u8 < (v_N_u8 - 1); v_Index_u8++) {
                v_InnerIndex_u8 = v_Index_u8;
                for (v_Idx_u8 = v_Index_u8 + 1; v_Idx_u8 < v_N_u8; v_Idx_u8++) {
                    if (v_W_ax[v_InnerIndex_u8] < v_W_ax[v_Idx_u8]) {
                        v_InnerIndex_u8 = v_Idx_u8;
                    }
                }
                if (v_Index_u8 != v_InnerIndex_u8) {
                    T v_Tmp_x = 0.0;
                    v_Tmp_x = v_W_ax[v_Index_u8];
                    v_W_ax[v_Index_u8] = v_W_ax[v_InnerIndex_u8];
                    v_W_ax[v_InnerIndex_u8] = v_Tmp_x;

                    for (v_Idx_u8 = 0; v_Idx_u8 < v_M_u8; v_Idx_u8++) {
                        v_Tmp_x = v_Temp_ax[v_Index_u8][v_Idx_u8];
                        v_Temp_ax[v_Index_u8][v_Idx_u8] = v_Temp_ax[v_InnerIndex_u8][v_Idx_u8];
                        v_Temp_ax[v_InnerIndex_u8][v_Idx_u8] = v_Tmp_x;
                    }
                    for (v_Idx_u8 = 0; v_Idx_u8 < v_N_u8; v_Idx_u8++) {
                        v_Tmp_x = v_TempV_ax[v_Index_u8][v_Idx_u8];
                        v_TempV_ax[v_Index_u8][v_Idx_u8] = v_TempV_ax[v_InnerIndex_u8][v_Idx_u8];
                        v_TempV_ax[v_InnerIndex_u8][v_Idx_u8] = v_Tmp_x;
                    }
                }
            }

            for (v_Index_u8 = 0; v_Index_u8 < v_N1_u8; v_Index_u8++) // 0,1,2
            {
                v_SD_x = (v_Index_u8 < v_N_u8) ? v_W_ax[v_Index_u8] : 0;
                if (v_SD_x <= v_Minval_x) {
                    uint32_t v_LoopCount_u32 = 0;
                    while (v_LoopCount_u32 < 100) { // make maximum 100 to avoid endless loop, normally run once should be enough
                        // if we got a zero singular value, then in order to get the corresponding left singular vector
                        // we generate a random vector, project it to the previously computed left singular vectors,
                        // subtract the projection and normalize the difference.
                        const T c_Val0_x = 1. / static_cast<T>(v_M_u8);
                        for (v_Idx_u8 = 0; v_Idx_u8 < v_M_u8; v_Idx_u8++) // 0,1,2
                        {
                            uint32_t v_R_u32 = rand();
                            T v_Val_x = ((v_R_u32 & 256) != 0) ? c_Val0_x : -c_Val0_x; // bit8 randomly toggles between 0 and 1
                            v_Temp_ax[v_Index_u8][v_Idx_u8] = v_Val_x; //-- At(i*astep + k)
                        }
                        for (v_Iter_u8 = 0; v_Iter_u8 < 2; v_Iter_u8++) // 0,1
                        {
                            for (v_InnerIndex_u8 = 0; v_InnerIndex_u8 < v_Index_u8; v_InnerIndex_u8++) //
                            {
                                v_SD_x = 0;
                                for (v_Idx_u8 = 0; v_Idx_u8 < v_M_u8; v_Idx_u8++) {
                                    v_SD_x += v_Temp_ax[v_Index_u8][v_Idx_u8] * v_Temp_ax[v_InnerIndex_u8][v_Idx_u8]; //-- At(i*astep + k) * At(j*astep + k)
                                }
                                T v_Asum_x = 0;
                                for (v_Idx_u8 = 0; v_Idx_u8 < v_M_u8; v_Idx_u8++) {
                                    T v_T_x = v_Temp_ax[v_Index_u8][v_Idx_u8] - v_SD_x * v_Temp_ax[v_InnerIndex_u8][v_Idx_u8]; //-- At(i*astep + k) - sd*At[(*astep + k)
                                    v_Temp_ax[v_Index_u8][v_Idx_u8] = v_T_x; //-- At(i*astep + k)
                                    v_Asum_x += mecl::math::abs_x<T>(v_T_x);
                                }
                                v_Asum_x = ( v_Asum_x > 0 ) ? 1. / v_Asum_x : 0;
                                for (v_Idx_u8 = 0; v_Idx_u8 < v_M_u8; v_Idx_u8++) {
                                    v_Temp_ax[v_Index_u8][v_Idx_u8] *= v_Asum_x; //-- At(i*astep + k)
                                }
                            }
                        }
                        v_SD_x = 0;
                        for (v_Idx_u8 = 0; v_Idx_u8 < v_M_u8; v_Idx_u8++) {
                            T v_T_x = v_Temp_ax[v_Index_u8][v_Idx_u8]; //-- At(i*astep + k)
                            v_SD_x += static_cast<T>(v_T_x * v_T_x);
                        }
                        v_SD_x = sqrt(v_SD_x);
                        v_LoopCount_u32 ++;
                        if (v_SD_x > v_Minval_x) {
                            break;
                        }
                    }
                    if (v_SD_x <= v_Minval_x) {
                        v_Ret_b = false;
                        TRACE_2(hTracer_u64, "SVD loops %d times, sd value is %d", v_LoopCount_u32, v_SD_x);
                    }
                }

                v_S_x = 1. / v_SD_x;
                for (v_Idx_u8 = 0; v_Idx_u8 < v_M_u8; v_Idx_u8++) {
                    v_Temp_ax[v_Index_u8][v_Idx_u8] *= v_S_x; //-- At(i*astep + k)
                }
            }
        }
        //-- end of function JacobiSVDImpl_() body

        o_W_ax[0] = -v_Temp_ax[2][0];
        o_W_ax[1] = -v_Temp_ax[2][1];
        o_W_ax[2] = -v_Temp_ax[2][2];
    }

    return v_Ret_b;
}

#ifdef string
// SB
// PRQA S 4327 ++
void FeatureFilter::PrintPointPairOnError(std::string val, ImageFeature &imPair1Pt1, ImageFeature &imPair1Pt2, ImageFeature &imPair2Pt1, ImageFeature &imPair2Pt2) const   // PRQA S 4212
{
                TRACE_2( hTracer_u64, "[%s] [ComputeInitialGuess]: %s denominator is zero, continuing...", kCameraStrings[cameraID_t].c_str(), val.c_str());

                TRACE_5( hTracer_u64,    "[%s] [ComputeInitialGuess]: Pair1 : 1st Image Point = ( %.2f , %.2f ), 2nd Image Point = ( %.2f , %.2f )",
                        kCameraStrings[cameraID_t].c_str(), imPair1Pt1.getUnWarpedPt_x().x_x, imPair1Pt1.getUnWarpedPt_x().y_x, imPair1Pt2.getUnWarpedPt_x().x_x, imPair1Pt2.getUnWarpedPt_x().y_x);

                TRACE_5( hTracer_u64,    "[%s] [ComputeInitialGuess]: Pair2 : 1st Image Point = ( %.2f , %.2f ), 2nd Image Point = ( %.2f , %.2f )",
                        kCameraStrings[cameraID_t].c_str(), imPair2Pt1.getUnWarpedPt_x().x_x, imPair2Pt1.getUnWarpedPt_x().y_x, imPair2Pt2.getUnWarpedPt_x().x_x, imPair2Pt2.getUnWarpedPt_x().y_x);
}
// PRQA S 4327 --
#endif
#ifdef USE_SVSCM
void FeatureFilter::ComputeAmatRow(ImageFeature &imPair1Pt1, ImageFeature &imPair1Pt2, float64_t (&AMat)[2][3], uint8_t ind, float64_t (&pairCam)[2][3]) const
{
    float64_t invK[3][3];
    float64_t pair1ImPt1[3] = { imPair1Pt1.getUnWarpedPt_x().x_x, imPair1Pt1.getUnWarpedPt_x().y_x, 1.0 };
    float64_t pair1ImPt2[3] = { imPair1Pt2.getUnWarpedPt_x().x_x, imPair1Pt2.getUnWarpedPt_x().y_x, 1.0 };

    memcpy(&invK[0][0], &cameraModel_po->GetIntrinsic().GetKInvPtr(), sizeof(float64_t)*9);
    MatrixVectorMultiply(&invK[0][0], 3, 3, false, &pair1ImPt1[0], &pairCam[0][0]);
    MatrixVectorMultiply(&invK[0][0], 3, 3, false, &pair1ImPt2[0], &pairCam[1][0]);

    // populate the A matrix of A*x homogeneous system (1st row)
    AMat[ind][0] = pairCam[0][1] - pairCam[1][1];
    AMat[ind][1] = pairCam[1][0] - pairCam[0][0];
    AMat[ind][2] = (pairCam[0][0] * pairCam[1][1]) - (pairCam[0][1] * pairCam[1][0]);

}
#else
void FeatureFilter::ComputeAmatRow(const ImageFeature &i_ImPair1Pt1_ro, const ImageFeature &i_ImPair1Pt2_ro, float32_t (&o_AMat_rf32)[2][3], uint8_t i_Ind_u8, float32_t (&o_PairCam_rf32)[2][3]) const
{
    fc::Pointd v_TmpCamPt_x;

    cameraModel_po->Image2Camera( cameraModel_po->getCameraObj_px(), i_ImPair1Pt1_ro.getUnWarpedPt_x(), v_TmpCamPt_x );
    o_PairCam_rf32[0][0] = static_cast<float32_t>(v_TmpCamPt_x.x_x);
    o_PairCam_rf32[0][1] = static_cast<float32_t>(v_TmpCamPt_x.y_x);
    o_PairCam_rf32[0][2] = 1.0;

    cameraModel_po->Image2Camera( cameraModel_po->getCameraObj_px(), i_ImPair1Pt2_ro.getUnWarpedPt_x(), v_TmpCamPt_x );
    o_PairCam_rf32[1][0] = static_cast<float32_t>(v_TmpCamPt_x.x_x);
    o_PairCam_rf32[1][1] = static_cast<float32_t>(v_TmpCamPt_x.y_x);
    o_PairCam_rf32[1][2] = 1.0;
    // populate the A matrix of A*x homogeneous system (1st row)
    o_AMat_rf32[i_Ind_u8][0] = o_PairCam_rf32[0][1] - o_PairCam_rf32[1][1];
    o_AMat_rf32[i_Ind_u8][1] = o_PairCam_rf32[1][0] - o_PairCam_rf32[0][0];
    o_AMat_rf32[i_Ind_u8][2] = (o_PairCam_rf32[0][0] * o_PairCam_rf32[1][1]) - (o_PairCam_rf32[0][1] * o_PairCam_rf32[1][0]);

}
#endif

template<typename T>
bool_t FeatureFilter::ComputeEta(T *o_Eta_px, T (&i_PairCam_rx)[2][3], uint8_t i_Ind_u8, T (&i_RotColumn_rx)[3])   // PRQA S 4212
{
    bool_t v_Ret_b = true;
    T v_Denominator_x = i_PairCam_rx[1][i_Ind_u8] * i_RotColumn_rx[2] - i_RotColumn_rx[i_Ind_u8];
    if ( !mecl::math::isAboutZero_b< T >( v_Denominator_x ) )
    {
        (*o_Eta_px) = (i_PairCam_rx[0][i_Ind_u8] - i_PairCam_rx[1][i_Ind_u8]) / v_Denominator_x;
    }
    else
    {
        v_Ret_b = false;
    }
    return v_Ret_b;
}

template<typename T>
bool_t FeatureFilter::ComputeInitialGuessByPairs( const ImageFeature& i_ImPair1Pt1_ro, const ImageFeature& i_ImPair1Pt2_ro, const ImageFeature& i_ImPair2Pt1_ro, const ImageFeature& i_ImPair2Pt2_ro, T i_ThetaCar_x , T v_DirectionalMotion_f32 )
{
    bool_t v_Ret_b = false;
    // Compute the Rotation Matrix from 2 ground features in 2 frames
    T v_KnownRotColumn_ax[3] = {0.0,0.0,0.0};
    T v_R1_ax[3] = {0.0,0.0,0.0};
    T v_R2_ax[3] = {0.0,0.0,0.0};
    T v_AMat_ax[2][3];
    T v_Sol_ax[3];

    //CREATE_TIMER( timer );
    //START_TIMER( timer );

    T v_Pair1CamPts_ax[2][3] = {{ 0, 0, 0 }, { 0, 0, 0 }};
    ComputeAmatRow(i_ImPair1Pt1_ro, i_ImPair1Pt2_ro, v_AMat_ax, 0, v_Pair1CamPts_ax);

    T v_Pair2CamPts_ax[2][3] = {{ 0, 0, 0 }, { 0, 0, 0 }};
    ComputeAmatRow(i_ImPair2Pt1_ro, i_ImPair2Pt2_ro, v_AMat_ax, 1, v_Pair2CamPts_ax);

    // Solve singular linear system A*x = 0, svd.solveZ(Amat, sol)
    SVDcomputeEx(v_AMat_ax, v_Sol_ax);

    if ( (cameraID_t == tscApi::e_TscFrontCam) || (cameraID_t == tscApi::e_TscRearCam) )
    {
        // r2[2] = - cos(bRad) * sin(aRad) < 0
        if (v_Sol_ax[2] < 0) {
            for (uint32_t v_Index_u32 = 0; v_Index_u32 < 3; v_Index_u32++) {
                v_R2_ax[v_Index_u32] = v_Sol_ax[v_Index_u32];
                v_KnownRotColumn_ax[v_Index_u32] = v_R2_ax[v_Index_u32];
            }
        }
        else {
            for (uint32_t v_Index_u32 = 0; v_Index_u32 < 3; v_Index_u32++) {
                v_R2_ax[v_Index_u32] = -v_Sol_ax[v_Index_u32];
                v_KnownRotColumn_ax[v_Index_u32] = v_R2_ax[v_Index_u32];
            }
        }
    }
    else if ( (cameraID_t == tscApi::e_TscLeftCam) || (cameraID_t == tscApi::e_TscRightCam) )
    {
        // -- upside-up image gRad [-90, 90) r1[0] = cos(gRad) * cos(bRad) > 0
        // -- upside-down image gRad [90, 270) r1[0] = cose(gRad) * cos(bRad) < 0
        bool_t v_Positive_b = true;
#ifdef USE_SVSCM
        if ( cameraModel_po->GetExtrinsic().isRollIn14Quadrants_b() )
#else
        if ( cameraModel_po->isRollIn14Quadrants_b() )
#endif
        {
            v_Positive_b = (v_Sol_ax[0] > 0);
        }
#ifdef USE_SVSCM
        else if ( cameraModel_po->GetExtrinsic().isRollIn23Quadrants_b() )
#else
        else if ( cameraModel_po->isRollIn23Quadrants_b() )
#endif
        {
            v_Positive_b = (v_Sol_ax[0] < 0);
        }
        else
        {
            // as the range of roll has been check at the configuration
            // code shall not ever run into here
            // TODO: add error code
        }
        if (v_Positive_b) {
            v_R1_ax[0] = v_Sol_ax[0];
            v_R1_ax[1] = v_Sol_ax[1];
            v_R1_ax[2] = v_Sol_ax[2];
        } else {
            v_R1_ax[0] = -v_Sol_ax[0];
            v_R1_ax[1] = -v_Sol_ax[1];
            v_R1_ax[2] = -v_Sol_ax[2];
        }
        v_KnownRotColumn_ax[0] = v_R1_ax[0];
        v_KnownRotColumn_ax[1] = v_R1_ax[1];
        v_KnownRotColumn_ax[2] = v_R1_ax[2];
    }else
    {
        // as the camera id has been check at the TSC_Start()
        // code shall not ever run into here
        // TODO: add error code
    }

    T v_N1_ax[3];
    v_N1_ax[0] = -v_KnownRotColumn_ax[1];
    v_N1_ax[1] = v_KnownRotColumn_ax[0];
    v_N1_ax[2] = 0.0;
    normalizeEx(v_N1_ax, v_N1_ax);

    T v_N2_ax[3];
    v_N2_ax[0] = v_KnownRotColumn_ax[1]*v_N1_ax[2] - v_KnownRotColumn_ax[2]*v_N1_ax[1];
    v_N2_ax[1] = v_KnownRotColumn_ax[2]*v_N1_ax[0] - v_KnownRotColumn_ax[0]*v_N1_ax[2];
    v_N2_ax[2] = v_KnownRotColumn_ax[0]*v_N1_ax[1] - v_KnownRotColumn_ax[1]*v_N1_ax[0];

    T v_EtaX1_x;
    T v_EtaY1_x;
    T v_EtaX2_x;
    T v_EtaY2_x;
    if ( !ComputeEta(&v_EtaX1_x, v_Pair1CamPts_ax, 0, v_KnownRotColumn_ax) )
    {
        PrintPointPairOnError("eta_x1", i_ImPair1Pt1_ro, i_ImPair1Pt2_ro, i_ImPair2Pt1_ro, i_ImPair2Pt2_ro);
    } else
    if ( !ComputeEta(&v_EtaY1_x, v_Pair1CamPts_ax, 1, v_KnownRotColumn_ax) )
    {
        PrintPointPairOnError("eta_x1", i_ImPair1Pt1_ro, i_ImPair1Pt2_ro, i_ImPair2Pt1_ro, i_ImPair2Pt2_ro);
    } else
    if ( !ComputeEta(&v_EtaX2_x, v_Pair2CamPts_ax, 0, v_KnownRotColumn_ax) )
    {
        PrintPointPairOnError("eta_x2", i_ImPair1Pt1_ro, i_ImPair1Pt2_ro, i_ImPair2Pt1_ro, i_ImPair2Pt2_ro);
    } else
    if ( !ComputeEta(&v_EtaY2_x, v_Pair2CamPts_ax, 1, v_KnownRotColumn_ax) )
    {
        PrintPointPairOnError("eta_y2", i_ImPair1Pt1_ro, i_ImPair1Pt2_ro, i_ImPair2Pt1_ro, i_ImPair2Pt2_ro);
    } else
    { 

        T v_A_x = v_EtaX2_x * (v_N1_ax[0]*v_Pair1CamPts_ax[0][0]+v_N1_ax[1]*v_Pair1CamPts_ax[0][1]+v_N1_ax[2]*v_Pair1CamPts_ax[0][2])
                    - v_EtaX1_x * (v_N1_ax[0]*v_Pair2CamPts_ax[0][0]+v_N1_ax[1]*v_Pair2CamPts_ax[0][1]+v_N1_ax[2]*v_Pair2CamPts_ax[0][2]);

        T v_B_x = v_EtaX2_x * (v_N2_ax[0]*v_Pair1CamPts_ax[0][0]+v_N2_ax[1]*v_Pair1CamPts_ax[0][1]+v_N2_ax[2]*v_Pair1CamPts_ax[0][2])
                    - v_EtaX1_x * (v_N2_ax[0]*v_Pair2CamPts_ax[0][0]+v_N2_ax[1]*v_Pair2CamPts_ax[0][1]+v_N2_ax[2]*v_Pair2CamPts_ax[0][2]);

        if ( mecl::math::isAboutZero_b< float32_t >(v_B_x) )
        {
            PrintPointPairOnError("B", i_ImPair1Pt1_ro, i_ImPair1Pt2_ro, i_ImPair2Pt1_ro, i_ImPair2Pt2_ro);
        }
        else
        {
            T v_Alpha1_x = - atan( v_A_x / v_B_x );
            T v_R3_ax[3];

            v_R3_ax[0] = cos(v_Alpha1_x) * v_N1_ax[0] + sin(v_Alpha1_x) * v_N2_ax[0];
            v_R3_ax[1] = cos(v_Alpha1_x) * v_N1_ax[1] + sin(v_Alpha1_x) * v_N2_ax[1];
            v_R3_ax[2] = cos(v_Alpha1_x) * v_N1_ax[2] + sin(v_Alpha1_x) * v_N2_ax[2];
            // r3 = cos(bRad) *cos(aRad) > 0
            if (v_R3_ax[2] < 0) {
                v_R3_ax[0] = -v_R3_ax[0];
                v_R3_ax[1] = -v_R3_ax[1];
                v_R3_ax[2] = -v_R3_ax[2];
            }

            // now we can compute the third column of R matrix
            if ( (cameraID_t == tscApi::e_TscFrontCam) || (cameraID_t == tscApi::e_TscRearCam) )
            {
                v_R1_ax[0] = v_R2_ax[1]*v_R3_ax[2] - v_R2_ax[2]*v_R3_ax[1];
                v_R1_ax[1] = v_R2_ax[2]*v_R3_ax[0] - v_R2_ax[0]*v_R3_ax[2];
                v_R1_ax[2] = v_R2_ax[0]*v_R3_ax[1] - v_R2_ax[1]*v_R3_ax[0];
            }
            else if ( (cameraID_t == tscApi::e_TscLeftCam) || (cameraID_t == tscApi::e_TscRightCam) )
            {
                v_R2_ax[0] = -(v_R1_ax[1]*v_R3_ax[2] - v_R1_ax[2]*v_R3_ax[1]);
                v_R2_ax[1] = -(v_R1_ax[2]*v_R3_ax[0] - v_R1_ax[0]*v_R3_ax[2]);
                v_R2_ax[2] = -(v_R1_ax[0]*v_R3_ax[1] - v_R1_ax[1]*v_R3_ax[0]);
            }else
            {
            }

            T v_Mu1_x = v_R3_ax[0]*v_Pair1CamPts_ax[0][0]+v_R3_ax[1]*v_Pair1CamPts_ax[0][1]+v_R3_ax[2]*v_Pair1CamPts_ax[0][2];
            T v_Mu2_x = v_R3_ax[0]*v_Pair2CamPts_ax[0][0]+v_R3_ax[1]*v_Pair2CamPts_ax[0][1]+v_R3_ax[2]*v_Pair2CamPts_ax[0][2];
            T v_ThetaX1_x = v_EtaX1_x / v_Mu1_x;
            T v_ThetaX2_x = v_EtaX2_x / v_Mu2_x;
            T v_ThetaY1_x = v_EtaY1_x / v_Mu1_x;
            T v_ThetaY2_x = v_EtaY2_x / v_Mu2_x;

            T v_ThetaAvg_x = ( v_ThetaX1_x + v_ThetaX2_x + v_ThetaY1_x + v_ThetaY2_x ) / 4.0;

            // theta_avg and theta_car should have the same sign
            if ( ( v_ThetaAvg_x >= 0) != ( i_ThetaCar_x >=0 ) )
            {
                TRACE_3( hTracer_u64, "[%s] theta_avg [%f] and theta_car [%f] don't have the same sign", kCameraStrings[cameraID_t].c_str(), v_ThetaAvg_x, i_ThetaCar_x );
            }
            else
            {

            T v_ThetaDeviation_x = mecl::math::abs_x<T>( 100.0 * ( 1.0 - v_ThetaAvg_x / i_ThetaCar_x ) );
                m_logStream << currFrameNum_u32 << ", ";
                m_logStream << i_ImPair1Pt1_ro.getPt_t().x_x << ", ";
                m_logStream << i_ImPair1Pt1_ro.getPt_t().y_x << ", ";
                m_logStream << i_ImPair1Pt2_ro.getPt_t().x_x << ", ";
                m_logStream << i_ImPair1Pt2_ro.getPt_t().y_x << ", ";
                m_logStream << i_ImPair2Pt1_ro.getPt_t().x_x << ", ";
                m_logStream << i_ImPair2Pt1_ro.getPt_t().y_x << ", ";
                m_logStream << i_ImPair2Pt2_ro.getPt_t().x_x << ", ";
                m_logStream << i_ImPair2Pt2_ro.getPt_t().y_x << ", ";
                m_logStream << v_ThetaDeviation_x << ", ";
                m_logStream << i_ThetaCar_x << ", ";
                m_logStream << v_ThetaAvg_x << ", ";
                m_logStream << v_DirectionalMotion_f32 << ", ";
                m_logStream << v_ThetaAvg_x* cameraModel_po->GetExtrinsics().pos_x.cVal_ax[2] << ", ";
                m_logStream << i_ThetaCar_x / v_ThetaAvg_x << "\n";
                T v_Z_x = v_DirectionalMotion_f32 / v_ThetaAvg_x;
#if 0
                if( i_ImPair1Pt1_ro.getMV_o().getPsi_f64() > 0 || i_ImPair1Pt1_ro.getMV_o().getPsi_f64() < 0 )
                {
                    if( fc::FeatureCollection::getInstance_rt().selFeaturesSize < FEATSIZE )
                    {
                        fc::FeatureCollection::getInstance_rt().selFeatures[fc::FeatureCollection::getInstance_rt().selFeaturesSize].first =  i_ImPair1Pt1_ro ;
                        fc::FeatureCollection::getInstance_rt().selFeatures[fc::FeatureCollection::getInstance_rt().selFeaturesSize].second = ( i_ImPair1Pt2_ro );
                        fc::FeatureCollection::getInstance_rt().selFeaturesSize++;
                    }
                }

                if( i_ImPair2Pt1_ro.getMV_o().getPsi_f64() > 0 || i_ImPair2Pt1_ro.getMV_o().getPsi_f64() < 0 )
                {
                    if( fc::FeatureCollection::getInstance_rt().selFeaturesSize < FEATSIZE )
                    {
                        //fc::FeatureCollection::getInstance_rt().selFeatures[fc::FeatureCollection::getInstance_rt().selFeaturesSize].first = i_ImPair2Pt1_ro;
                        //fc::FeatureCollection::getInstance_rt().selFeatures[fc::FeatureCollection::getInstance_rt().selFeaturesSize].second = (i_ImPair2Pt2_ro);
                        //fc::FeatureCollection::getInstance_rt().selFeaturesSize++;
                    }
                }
#endif
            // --- validate against some conditions and then store them
            if ( v_ThetaDeviation_x <= static_cast<T>(deviationPercentageIG_u32) )
            {
                T v_BRad_x = mecl::math::trigonometry<T>::asin_x(v_R1_ax[2]);
                T v_ARad_x = mecl::math::trigonometry<T>::atan2_x(-v_R2_ax[2], v_R3_ax[2]);
                T v_GRad_x = mecl::math::trigonometry<T>::atan2_x(-v_R1_ax[1], v_R1_ax[0]);

                T v_PitchDeg_x = mecl::math::toDegrees_x<T>( v_ARad_x );
                T v_YawDeg_x = mecl::math::toDegrees_x<T>( v_BRad_x );
                T v_RollDeg_x = mecl::math::toDegrees_x<T>( v_GRad_x );
                    T v_Z_x;// = v_DirectionalMotion_f32 / v_ThetaAvg_x;
                T v_LimitDeg_x = 180.0;

                tsc_math::toRange(v_RollDeg_x, v_LimitDeg_x);

                // --- validate against some conditions and then store them
                T v_ExtrinsicPitchDeg_x;
                T v_ExtrinsicYawDeg_x;
                T v_ExtrinsicRollDeg_x;
#ifdef USE_SVSCM
                v_ExtrinsicPitchDeg_x = cameraModel_po->GetExtrinsic().GetPtchDg();
                v_ExtrinsicYawDeg_x = cameraModel_po->GetExtrinsic().GetYwDg();
                v_ExtrinsicRollDeg_x = cameraModel_po->GetExtrinsic().GetRllDg();
#else
                v_ExtrinsicPitchDeg_x = mecl::math::toDegrees_x( cameraModel_po->GetExtrinsics().eulerAngles_s.pitch_x );
                v_ExtrinsicYawDeg_x = mecl::math::toDegrees_x( cameraModel_po->GetExtrinsics().eulerAngles_s.yaw_x );
                v_ExtrinsicRollDeg_x = mecl::math::toDegrees_x( cameraModel_po->GetExtrinsics().eulerAngles_s.roll_x );
#endif
                if ( (v_ExtrinsicRollDeg_x < -90.0F) && (v_RollDeg_x > 90.0F) )
                {
                    v_RollDeg_x -= 360.0F;
                }
                if ( (v_ExtrinsicRollDeg_x > 90.0F) && (v_RollDeg_x < -90.0F) )
                {
                    v_RollDeg_x += 360.0F;
                }
                if ( (mecl::math::abs_x<T>( v_PitchDeg_x - v_ExtrinsicPitchDeg_x ) < angleThreshIGDeg_f32) &&
                     (mecl::math::abs_x<T>( v_YawDeg_x - v_ExtrinsicYawDeg_x ) < angleThreshIGDeg_f32) &&
                     (mecl::math::abs_x<T>( v_RollDeg_x - v_ExtrinsicRollDeg_x ) < angleThreshIGDeg_f32) )
                {
                	v_Z_x = v_DirectionalMotion_f32 / v_ThetaAvg_x;
                    fc::InitialGuess v_Ig_o(v_PitchDeg_x, v_YawDeg_x, v_RollDeg_x, false);// v_Z_x );
                    initialGuesses_x.pushBack_v( v_Ig_o );
                    v_Ret_b = true;
                    TRACE_5( hTracer_u64, "[%s]: Added initial guess #(%u): { %.2f, %.2f, %.2f }",  \
                             kCameraStrings[cameraID_t].c_str(), initialGuesses_x.size_u32(), v_PitchDeg_x, v_YawDeg_x, v_RollDeg_x );
                }
#ifdef DEBUG_TSC_ALG
                else
                {
                    m_pDebugCounters->ignoredIGThreshold ++;
                    TRACE_4( hTracer_u64, "[%s]: Ignored estimated angle: { %.2f, %.2f, %.2f }",  \
                             kCameraStrings[cameraID_t].c_str(), v_PitchDeg_x, v_YawDeg_x, v_RollDeg_x );
                }
#endif
            }
#ifdef DEBUG_TSC_ALG
            else
            {
                m_pDebugCounters->ignoredIGDeviation ++;
            }
#endif
            }
        }
    }

    //STOP_TIMER( timer );
    TRACE_5( hTracer_u64, "[%s] [%s]: Processed features ending in frame [%lu] producing [%u] IGs; Elapsed time [%.3f]ms",  \
        kCameraStrings[cameraID_t].c_str(), __func__, currFrameNum_u32, initialGuesses_x.size_u32(), GET_ELAPSED_TIME( timer ));
    return v_Ret_b;
}

bool_t FeatureFilter::ComputeInitialGuess( LocalFeatureTrackCollection& b_LocalTrackList_ro )
{
    bool_t v_Ret_b = true;
    //CREATE_TIMER( timer );
    //START_TIMER( timer );

    if( validIndices_x.size_u32() == 0)
    {
        v_Ret_b = false;
    }
    else
    {
    // capture the non-zero car movement
#ifdef USE_SVSCM
    float64_t v_DirectionalMotion_f32; // s1 or s2 depending on camera
#else
    float32_t v_DirectionalMotion_f32; // s1 or s2 depending on camera
#endif
    if ( (cameraID_t == tscApi::e_TscFrontCam) || (cameraID_t == tscApi::e_TscRearCam) )
    {
        v_DirectionalMotion_f32 = b_LocalTrackList_ro.getTrcks_rx()[ validIndices_x[0] ].getTotalMV_o().getY_f64();
    }
    else // for side cameras
    {
#ifdef USE_SVSCM
        v_DirectionalMotion_f32 = i_LocalTrackList_ro.getTrcks_rx()[ validIndices_x[0] ].getTotalMV_o().getX_f64();
#else
        v_DirectionalMotion_f32 = - b_LocalTrackList_ro.getTrcks_rx()[ validIndices_x[0] ].getTotalMV_o().getX_f64();
#endif
    }
#ifdef USE_SVSCM
    float64_t v_ThetaCar_f32 = v_DirectionalMotion_f32 / cameraModel_po->GetExtrinsic().GetZmm();
#else
    float32_t v_ThetaCar_f32 = v_DirectionalMotion_f32 / cameraModel_po->GetExtrinsics().pos_x.cVal_ax[2];
#endif

    size_t v_MaxIdx_t = validIndices_x.size_u32();
    for ( size_t v_Pair1_t = 0; v_Pair1_t < (v_MaxIdx_t - 1); ++v_Pair1_t )
    {
        uint32_t v_Idx1_u32 = validIndices_x[v_Pair1_t];
        FeatureTrack* v_Pair1Feature_po = &b_LocalTrackList_ro.getTrcks_rx()[ v_Idx1_u32 ];
        ImageFeature &v_Pair1ImageFeature1_ro = v_Pair1Feature_po->getTrckLst_rx().front_ro();
        ImageFeature &v_Pair1ImageFeature2_ro = v_Pair1Feature_po->getTrckLst_rx().back_ro();

        for ( size_t v_Pair2_t = v_Pair1_t + 1; v_Pair2_t < v_MaxIdx_t; ++v_Pair2_t )
        {
            uint32_t v_Idx2_u32 = validIndices_x[v_Pair2_t];
            FeatureTrack* v_Pair2Feature_po = &b_LocalTrackList_ro.getTrcks_rx()[ v_Idx2_u32 ];
            ImageFeature &v_Pair2ImageFeature1_ro = v_Pair2Feature_po->getTrckLst_rx().front_ro();
            ImageFeature &v_Pair2ImageFeature2_ro = v_Pair2Feature_po->getTrckLst_rx().back_ro();

            if( !ComputeInitialGuessByPairs( v_Pair1ImageFeature1_ro, v_Pair1ImageFeature2_ro, v_Pair2ImageFeature1_ro, v_Pair2ImageFeature2_ro, v_ThetaCar_f32, v_DirectionalMotion_f32 ) )
            {
                TRACE_4( hTracer_u64, "[%s] [%s]: No initialGuess from pair1 index[%u] and pair2 index[%u]", kCameraStrings[cameraID_t].c_str(), __func__, v_Idx1_u32, v_Idx2_u32);
            }

        }
    }

    if( initialGuesses_x.size_u32() == 0 )
    {
        v_Ret_b = false;
    }
    }

    //STOP_TIMER( timer );
    TRACE_5( hTracer_u64, "[%s] [%s]: Processed features ending in frame [%lu] producing [%u] IGs; Elapsed time [%.3f]ms",  \
        kCameraStrings[cameraID_t].c_str(), __func__, currFrameNum_u32, initialGuesses_x.size_u32(), GET_ELAPSED_TIME( timer ));

    return v_Ret_b;
}


// ----------------------------------------------------------------------------

bool_t FeatureFilter::ComputeInitialGuessCombinations( LocalFeatureTrackCollection& b_LocalTrackList_ro )
{
    bool_t v_Ret_b = true;
    //CREATE_TIMER( timer );
    //START_TIMER( timer );
    AppCtrl::Timer timer;
    timer.Start();
    bool_t IGValid_b[tsc_cfg::NUM_INITIAL_GUESSES_PER_FRAME];
    memset(&IGValid_b, 0, sizeof(IGValid_b));

    if( validIndices_x.size_u32() == 0)
    {
        v_Ret_b = false;
    }
    else
    {
        FeatureTrack& v_FirstTrack_ro = b_LocalTrackList_ro.getTrcks_rx()[ validIndices_x[0] ];
        for( size_t v_Pt1_t = 0; v_Pt1_t < (trackLengthThresh_u32 - 1); ++v_Pt1_t )
        {
#ifdef USE_SVSCM
            float64_t v_DirectionalMotion_f32 = 0; // s1 or s2 depending on camera
#else
            float32_t v_DirectionalMotion_f32 = 0; // s1 or s2 depending on camera
#endif

            for( size_t v_Pt2_t = v_Pt1_t + 1; v_Pt2_t < trackLengthThresh_u32; ++v_Pt2_t )
            {
                // capture the non-zero car movement
                if ( (cameraID_t == tscApi::e_TscFrontCam) || (cameraID_t == tscApi::e_TscRearCam) )
                {
                    v_DirectionalMotion_f32 += v_FirstTrack_ro.getTrckLst_rx()[ v_Pt2_t ].getMV_o().getY_f64();
                }
                else // for side cameras
                {
#ifdef USE_SVSCM
                    v_DirectionalMotion_f32 += v_FirstTrack_ro.getTrckLst_rx()[ v_Pt2_t ].getMV_o().getX_f64();
#else
                    v_DirectionalMotion_f32 -= v_FirstTrack_ro.getTrckLst_rx()[ v_Pt2_t ].getMV_o().getX_f64();
#endif
                }
#ifdef USE_SVSCM
                float64_t v_ThetaCar_f32 = v_DirectionalMotion_f32 / cameraModel_po->GetExtrinsic().GetZmm();
#else
                float32_t v_ThetaCar_f32 = v_DirectionalMotion_f32 / cameraModel_po->GetExtrinsics().pos_x.cVal_ax[2];
#endif

                size_t v_MaxIdx_t = validIndices_x.size_u32();
                for ( size_t v_Pair1_t = 0; v_Pair1_t < (v_MaxIdx_t - 1); ++v_Pair1_t )
                {
                    uint16_t v_Idx1_u16 = validIndices_x[v_Pair1_t];
                    FeatureTrack& v_Pair1Feature_ro = b_LocalTrackList_ro.getTrcks_rx()[ v_Idx1_u16 ];
                    ImageFeature &v_Pair1ImageFeature1_ro = v_Pair1Feature_ro.getTrckLst_rx()[v_Pt1_t];
                    ImageFeature &v_Pair1ImageFeature2_ro = v_Pair1Feature_ro.getTrckLst_rx()[v_Pt2_t];

                    for ( size_t v_Pair2_t = v_Pair1_t + 1; v_Pair2_t < v_MaxIdx_t; ++v_Pair2_t )
                    {
                        uint16_t v_Idx2_u16 = validIndices_x[v_Pair2_t];
                        FeatureTrack& v_Pair2Feature_ro = b_LocalTrackList_ro.getTrcks_rx()[ v_Idx2_u16 ];
                        if (v_Pair2Feature_ro.getRoiI_u32() != v_Pair1Feature_ro.getRoiI_u32())
                        {
                            ImageFeature& v_Pair2ImageFeature1_ro = v_Pair2Feature_ro.getTrckLst_rx()[v_Pt1_t];
                            ImageFeature& v_Pair2ImageFeature2_ro = v_Pair2Feature_ro.getTrckLst_rx()[v_Pt2_t];

                            if (!ComputeInitialGuessByPairs(v_Pair1ImageFeature1_ro, v_Pair1ImageFeature2_ro, v_Pair2ImageFeature1_ro, v_Pair2ImageFeature2_ro, v_ThetaCar_f32, v_DirectionalMotion_f32))
                            {
                                TRACE_4(hTracer_u64, "[%s] [%s]: No initialGuess from pair1 pt1[%u] and pair2 pt2[%u]", kCameraStrings[cameraID_t].c_str(), __func__, v_Pt1_t, v_Pt2_t);
                            }
                        }
                    } // end for ( size_t pair2 = pair1 + 1; pair2 < validIndices_x.size(); ++pair2 )
                } // end for ( size_t pair1 = 0; pair1 < validIndices_x.size() - 1; ++pair1 )
            } // end for( size_t pt2 = pt1 + 1; pt1 < m_trackLengthThresh; ++pt1 )
        } // end for( size_t pt1 = 0; pt1 < m_trackLengthThresh - 1; ++pt1 )

        if( initialGuesses_x.size_u32() != 0 )
        {
            size_t v_MaxIdx_t = initialGuesses_x.size_u32();
            for( size_t v_Index_t = 0; v_Index_t < (v_MaxIdx_t - 1); ++v_Index_t )
            {
                for( size_t v_InnerIndex_t = v_Index_t + 1; v_InnerIndex_t < v_MaxIdx_t; ++v_InnerIndex_t )
                {
                    fc::InitialGuess &v_Igi_ro = initialGuesses_x[ v_Index_t ];
                    fc::InitialGuess &v_Igj_ro = initialGuesses_x[ v_InnerIndex_t ];
                    float64_t v_PitchDiff_f64 = mecl::math::abs_x<float64_t>( v_Igi_ro.getPitch_f64() - v_Igj_ro.getPitch_f64() );
                    float64_t v_YawDiff_f64 = mecl::math::abs_x<float64_t>( v_Igi_ro.getYaw_f64() - v_Igj_ro.getYaw_f64() );
                    float64_t v_RollDiff_f64 = mecl::math::abs_x<float64_t>( v_Igi_ro.getRoll_f64() - v_Igj_ro.getRoll_f64() );

                    if( (v_PitchDiff_f64 > initialGuessDiffThresholdDeg_f32) ||  \
                        (v_YawDiff_f64 > initialGuessDiffThresholdDeg_f32) ||  \
                        (v_RollDiff_f64 > initialGuessDiffThresholdDeg_f32) )
                    {
                        //STOP_TIMER( timer );
                        TRACE_5( hTracer_u64, "[%s] [%s]: Intermediate initial guesses {%u} and {%u} are not close, discarding all; Elapsed time [%.3f]ms",  \
                            kCameraStrings[cameraID_t].c_str(), __func__, v_Index_t, v_InnerIndex_t, GET_ELAPSED_TIME( timer ) );
                        //v_Ret_b = false;
                        //break;
                    }
                    else
                    {
                        IGValid_b[v_Index_t] = true;
                        IGValid_b[v_InnerIndex_t] = true;

                    }

                }
                if( !v_Ret_b )
                {
                    break;
                }
            }
            tempinitialGuesses_x.clear_v();
            for (size_t v_Index_t = 0; v_Index_t < (v_MaxIdx_t-1); v_Index_t++)
            {
                if (IGValid_b[v_Index_t] == true)
                {
                    tempinitialGuesses_x.pushBack_v(initialGuesses_x[v_Index_t]);
                }
            }
            initialGuesses_x.clear_v();
            initialGuesses_x.copyAll_v(tempinitialGuesses_x);
        }
        

        if( v_Ret_b )
        {
            //STOP_TIMER( timer );
            TRACE_5( hTracer_u64, "[%s] [%s]: Processed features ending in frame [%lu] producing [%u] IGs; Elapsed time [%.3f]ms",  \
                kCameraStrings[cameraID_t].c_str(), __func__, currFrameNum_u32, initialGuesses_x.size_u32(), GET_ELAPSED_TIME( timer ));

            if( initialGuesses_x.size_u32() == 0 )
            {
                v_Ret_b = false;
            }
        }
    }
    return v_Ret_b;
}

// ----------------------------------------------------------------------------

void FeatureFilter::ApplySfMFilter( const mecl::core::ArrayList < FeatureTrack, tsc_cfg::NUM_TRACKS >& i_Tracks_rt )
{
  if( validIndices_x.size_u32() >= 2 )
  {
    bool_t v_Break_b = false;
    for( uint32_t v_Index_u32 = 0; v_Index_u32 < validIndices_x.size_u32(); ++v_Index_u32 )
    {
      for( uint32_t v_InnerIndex_u32 = v_Index_u32 + 1; v_InnerIndex_u32 < validIndices_x.size_u32(); ++v_InnerIndex_u32 )
      {
        float64_t v_Z1_f64 = i_Tracks_rt[validIndices_x[v_Index_u32]].getDesignWorldPt_t().z_x;
        float64_t v_Z2_f64 = i_Tracks_rt[validIndices_x[v_InnerIndex_u32]].getDesignWorldPt_t().z_x;

        if( mecl::core::abs_x<float64_t>( v_Z1_f64 - v_Z2_f64 ) > maxHeightDiffMm_f64 )
        {
          v_Break_b = true;
          break;
        }
      }
      if(true == v_Break_b)
      {
        break;
      }
    }
    if(true == v_Break_b)
    {
      validIndices_x.clear_v();
    }
  }
}

// ----------------------------------------------------------------------------
