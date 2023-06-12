// ----------------------------------------------------------------------------
// --- Written by Hany Kashif [31-Oct-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
#ifndef __FEATURECOLLECTIONSTRUCTURES_H_
#define __FEATURECOLLECTIONSTRUCTURES_H_
// ----------------------------------------------------------------------------
#include "mecl/mecl.h"
#include "tscApi.h"
#include "configuration.h"
#ifdef USE_SVSCM
#include "cameraModel.h"
#else
#include "cameraModelMecl.h"
#endif
#include "kinematicModel.h"

// Suppress QACPP warning regarding use of floating point numbers
// PRQA S 3708 EOF
//
// ----------------------------------------------------------------------------
// --- data structures with namespace fc (feature collector)
// ------------------------------------------------------------------------
namespace fc
{
    //-------------------------------------------------------------------------
    class MotionVector
    {
        public:
        MotionVector():
            dXMM_f64( 0 ),
            dYMM_f64( 0 ),
            dPsiRad_f64( 0 )
        {
        }

        MotionVector( float64_t i_DXMM_f64, float64_t i_DYMM_f64, float64_t i_DPsiRad_f64 ):
            dXMM_f64( i_DXMM_f64 ),
            dYMM_f64( i_DYMM_f64 ),
            dPsiRad_f64( i_DPsiRad_f64 )
        {
        }
        MotionVector& operator += ( const MotionVector& i_Param_rt );

        MotionVector& operator = ( const MotionVector& i_Mv_rt )
        {
            if (this != &i_Mv_rt) {
            dXMM_f64 = i_Mv_rt.dXMM_f64;
            dYMM_f64 = i_Mv_rt.dYMM_f64;
            dPsiRad_f64 = i_Mv_rt.dPsiRad_f64;
            }
            return  *this;
        }
        float64_t getX_f64(void) const { return dXMM_f64; }
        float64_t getY_f64(void) const { return dYMM_f64; }
        float64_t getPsi_f64(void) const { return dPsiRad_f64; }
        void PutX(float64_t i_Param_f64) { dXMM_f64 = i_Param_f64; }
        void PutY(float64_t i_Param_f64) { dYMM_f64 = i_Param_f64; }
        void PutPsi(float64_t i_Param_f64) { dPsiRad_f64 = i_Param_f64; }

      private:
        float64_t dXMM_f64; // --- x motion in mm
        float64_t dYMM_f64; // --- y motion in mm
        float64_t dPsiRad_f64; // --- angle motion in radians
    };

    inline MotionVector& MotionVector::operator += ( const MotionVector& i_Param_rt )
    {
        dXMM_f64 += i_Param_rt.dXMM_f64;
        dYMM_f64 += i_Param_rt.dYMM_f64;
        dPsiRad_f64 += i_Param_rt.dPsiRad_f64;
        return *this;
    }

    class ImageFeature
    {
      public:
        ImageFeature()
            : pt_t( Point() )
            , motionVector_o( MotionVector() )
            , frameNumber_u32( 0 )
            , size_f32( 0 )
            , angle_f32( static_cast<float32_t>(-1.0) )
            , response_f32( 0 )
            , octave_s32( 0 )
            , matchDistance_f32( 0 )
            , unwarpedPt_x( Pointd() )
            , isUnwarped_b( false )
            , predicted2ndImagePt_t( Point() )
            , fundMatError_f64( 0.0 )
        {
        }

        Point getPt_t(void) const { return pt_t; }
        float32_t getSize_f32( void ) const
        {
            return size_f32;
        }
        float32_t getAngle_f32(void) const { return angle_f32; }
        float32_t getResponse_f32( void ) const
        {
            return response_f32;
        }
        sint32_t getOctave_s32( void ) const
        {
            return octave_s32;
        }
        float32_t getMatchDistance_f32( void ) const
        {
            return matchDistance_f32;
        }
        Point get2ndIPt_t(void) const { return predicted2ndImagePt_t; }
        float64_t getFundMatErr_f64(void) const { return fundMatError_f64; }
        uint32_t getFn_u32(void) const { return frameNumber_u32; }
        Pointd getUnWarpedPt_x(void) const { return unwarpedPt_x; }
        bool_t getIsUnWarped_b(void) const { return isUnwarped_b; }
        MotionVector getMV_o(void) const { return motionVector_o; }
        MotionVector &getMVAddress_ro(void) { return motionVector_o; }

        void PutPt(sint32_t x, sint32_t y) { pt_t.x_x = x; pt_t.y_x = y; }
        void PutAngle(float32_t i_Param_f32) { angle_f32 = i_Param_f32; }
        void PutMatchDist(float32_t i_Param_f32) { matchDistance_f32 = i_Param_f32; }
        void Put2ndIPt(Point i_Param_t) { predicted2ndImagePt_t = i_Param_t; }
        void PutFundMatErr(float64_t i_Param_f64) { fundMatError_f64 = i_Param_f64; }
        void PutFn(uint32_t i_Param_u32) { frameNumber_u32 = i_Param_u32; }
        void PutUnWarpedPt(Pointd i_Param_x) { unwarpedPt_x = i_Param_x; }
        void PutIsUnWarped(bool_t i_Param_b) { isUnwarped_b = i_Param_b; }
        void PutMV(MotionVector i_Param_o) { motionVector_o = i_Param_o; }

        private:
        Point pt_t; // --- point location in the current image
        MotionVector motionVector_o; // --- vehicle motion vector for current frame
        /// Temp
        uint32_t frameNumber_u32; // --- current frame number
        float32_t size_f32; // --- diameter of the meaningful keypoint neighborhood
        float32_t angle_f32; // --- computed orientation of the keypoint (-1 if not applicable);
                     // --- it's in [0,360) degrees and measured relative to
                     // --- image coordinate system, ie in clockwise.
        float32_t response_f32; // --- the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling
        sint32_t octave_s32; // --- octave (pyramid layer) from which the keypoint has been extracted
        float32_t matchDistance_f32;
        Pointd unwarpedPt_x;
        bool_t isUnwarped_b;
        Point predicted2ndImagePt_t; // --- predicted corresponding point location in the second image
        float64_t fundMatError_f64; // --- the fundamental matrix test error for this feature and its correspondence
    };

    class FeatureTrack
    {
        public:
        FeatureTrack() :
              kinematicModel_po( NULL )
            , numFilteredImagePts_u32( 0 )
            , lastDescriptorIdx_u32( 0 )
            , trackedFrameNumber_u32( 0 )
            , isTerminated_b( false )
            , isValid_b( true )
            , trackFrameSkip_u32( 0 )
            , totalMotionVector_o( MotionVector() )
            , roiIdx_u32( static_cast<uint32_t>(-1) )
            , designWorldPt_t(Point3d())
        {
        }

        FeatureTrack( const FeatureTrack& i_Ft_rt )
            : kinematicModel_po(i_Ft_rt.kinematicModel_po),
            numFilteredImagePts_u32(i_Ft_rt.numFilteredImagePts_u32),
            lastDescriptorIdx_u32(i_Ft_rt.lastDescriptorIdx_u32),
            trackedFrameNumber_u32(i_Ft_rt.trackedFrameNumber_u32),
            isTerminated_b(i_Ft_rt.isTerminated_b),
            isValid_b(i_Ft_rt.isValid_b),
            trackFrameSkip_u32(i_Ft_rt.trackFrameSkip_u32),
            totalMotionVector_o(i_Ft_rt.totalMotionVector_o),
            roiIdx_u32(i_Ft_rt.roiIdx_u32),
            designWorldPt_t(i_Ft_rt.designWorldPt_t)
        {
            trackList_x.copy_v(i_Ft_rt.trackList_x, i_Ft_rt.trackList_x.size_u32());
            filteredIndices_x.copy_v(i_Ft_rt.filteredIndices_x, i_Ft_rt.filteredIndices_x.size_u32());
            lastDescriptor_x.copyAll_v(i_Ft_rt.lastDescriptor_x);
        }

        FeatureTrack& operator = ( const FeatureTrack& i_Ft_rt )
        {
            if (this != &i_Ft_rt) {
            kinematicModel_po = i_Ft_rt.kinematicModel_po;
            numFilteredImagePts_u32 = i_Ft_rt.numFilteredImagePts_u32;
            lastDescriptorIdx_u32 = i_Ft_rt.lastDescriptorIdx_u32;
            trackedFrameNumber_u32 = i_Ft_rt.trackedFrameNumber_u32;
            isTerminated_b = i_Ft_rt.isTerminated_b;
            isValid_b = i_Ft_rt.isValid_b;
            trackFrameSkip_u32 = i_Ft_rt.trackFrameSkip_u32;
            totalMotionVector_o = i_Ft_rt.totalMotionVector_o;
            roiIdx_u32 = i_Ft_rt.roiIdx_u32;
            designWorldPt_t = i_Ft_rt.designWorldPt_t;
            trackList_x.copy_v(i_Ft_rt.trackList_x, i_Ft_rt.trackList_x.size_u32());
            filteredIndices_x.copy_v(i_Ft_rt.filteredIndices_x, i_Ft_rt.filteredIndices_x.size_u32());
            lastDescriptor_x.copyAll_v(i_Ft_rt.lastDescriptor_x);
            }
            return *this;
        }

        km::KinematicModelImpl* getPKmaticMdl_po(void) const { return kinematicModel_po; }  // PRQA S 4628
        uint32_t getTrckFrmSkp_u32(void) const { return trackFrameSkip_u32; }
        uint32_t getRoiI_u32(void) const { return roiIdx_u32; }
        uint32_t getTrckFrmNum_u32(void) const { return trackedFrameNumber_u32; }
        bool_t getIsTerm_b(void) const { return isTerminated_b; }
        MotionVector getTotalMV_o(void) const { return totalMotionVector_o; }
        Point3d getDesignWorldPt_t(void) const {return designWorldPt_t; }
        mecl::core::ArrayList < uint32_t, tsc_cfg::NUM_FILTERED_INDICES > &getFltrdIndcs_rx() { return filteredIndices_x; } // --- indices of the valid filtered items inside a feature track
        mecl::core::ArrayList < ImageFeature, tsc_cfg::TRACK_LENGTH > &getTrckLst_rx() { return trackList_x; } // --- tracked list of feature across frames
        mecl::core::Array< uint8_t, tsc_cfg::NUM_DESCRIPTOR_LIST> &getLstDscrptr_rx() { return lastDescriptor_x; } // --- (dynamic) array containing last descriptor
        //PRQA S 6200 1 //const not added to input pointer variables, otherwise assignment produces error
        void PutPKmaticMdl(km::KinematicModelImpl* i_Param_po) { kinematicModel_po = i_Param_po; }
        void PutTrckFrmSkp(uint32_t i_Param_u32) { trackFrameSkip_u32 = i_Param_u32; }
        void PutRoiI(uint32_t i_Param_u32) { roiIdx_u32 = i_Param_u32; }
        void PutTrckFrmNum(uint32_t i_Param_u32) { trackedFrameNumber_u32 = i_Param_u32; }
        void PutIsTerm(bool_t i_Param_b) { isTerminated_b = i_Param_b; }
        void PutIsValid(bool_t i_Param_b) { isValid_b = i_Param_b; }
        void PutTotalMVPlus(MotionVector i_Param_o) { totalMotionVector_o += i_Param_o; }
        void PutDesignWorldPt(Point3d i_Param_t) { designWorldPt_t = i_Param_t; }
        bool_t getIsValid()
        {
            return isValid_b;
        }

        private:
        km::KinematicModelImpl* kinematicModel_po; // --- handle to kinematic model Impl object
        uint32_t numFilteredImagePts_u32; // --- No. of valid filtered items inside a feature track
        uint32_t lastDescriptorIdx_u32; // --- last feature descriptor index
        uint32_t trackedFrameNumber_u32; // --- the latest frame number associated with this track
        bool_t isTerminated_b; // --- Whether this track is terminated; by default is false
        bool_t isValid_b; // --- Whether this track is valid; by default is true
        uint32_t trackFrameSkip_u32; // --- the frame skip associated with this track
        MotionVector totalMotionVector_o; // --- total vehicle motion vector between track begin and end
        uint32_t roiIdx_u32; // --- index of the roi where this feature track came from
        Point3d designWorldPt_t;
        mecl::core::ArrayList < ImageFeature, tsc_cfg::TRACK_LENGTH > trackList_x; // --- tracked list of feature across frames
        mecl::core::ArrayList < uint32_t, tsc_cfg::NUM_FILTERED_INDICES > filteredIndices_x; // --- indices of the valid filtered items inside a feature track
        mecl::core::Array< uint8_t, tsc_cfg::NUM_DESCRIPTOR_LIST> lastDescriptor_x; // --- (dynamic) array containing last descriptor
    };

    // ------------------------------------------------------------------------
    class ROI
    {
        public:

        ROI& operator = ( const ROI& i_Roi_rt )
        {
            if (this != &i_Roi_rt) {
            id_u8 = i_Roi_rt.id_u8;
            rect_t = i_Roi_rt.rect_t;
            }
            return *this;
        }
        uint8_t getID_u8(void) const { return id_u8; }
        tsc_math::ROIRect getRect_t(void) const { return rect_t; }
        void PutID(uint8_t i_Param_u8) { id_u8 = i_Param_u8; }
        void PutRect(tsc_math::ROIRect i_Param_t) { rect_t = i_Param_t; }

        private:
        uint8_t id_u8;
        tsc_math::ROIRect rect_t;
    };

    //-------------------------------------------------------------------------
    class LocalFeatureTrackCollection
    {
        public:
        LocalFeatureTrackCollection() :
            roi_o( ROI() ),
            numValidTracks_u32( 0 ),
            numIgnoredValidTracks_u32( 0 ),
            numInvalidTracks_u32( 0 )
        {
        }
        ROI getRoi_o(void) const { return roi_o; }
        void PutRoi(const ROI& i_ROI_ro) { roi_o = i_ROI_ro; }
        void AddValidTracks(uint32_t i_Count_u32) { numValidTracks_u32+= i_Count_u32; }
        uint32_t getNumValidTracks_u32(void) const { return numValidTracks_u32; }
        void AddIgnoredValidTracks(uint32_t i_Count_u32) { numIgnoredValidTracks_u32+= i_Count_u32; }
        uint32_t getNumIgnoredValidTracks_u32(void) const { return numIgnoredValidTracks_u32; }
        void incInvalidTracks_v(void) { numInvalidTracks_u32++; }
        uint32_t getNumInvalidTracks_u32(void) const { return numInvalidTracks_u32; }
        mecl::core::ArrayList < FeatureTrack, tsc_cfg::NUM_TRACKS > &getTrcks_rx() { return tracks_x; }

        private:
        mecl::core::ArrayList < FeatureTrack, tsc_cfg::NUM_TRACKS > tracks_x;
        ROI roi_o;
        uint32_t numValidTracks_u32;
        uint32_t numIgnoredValidTracks_u32;
        uint32_t numInvalidTracks_u32;
    };

    //-------------------------------------------------------------------------
    class ValidFeature
    {
        public:
        ValidFeature()
            : motionVector_o( MotionVector() )
            , worldPt_t( Point3d() )
            , fundMatError_f64( 0.0 )
            , designWorldPt_t( Point3d() )
        {
        }
        ValidFeature( const ValidFeature& i_Vf_rt )
            : motionVector_o( i_Vf_rt.motionVector_o )
            , worldPt_t( i_Vf_rt.worldPt_t )
            , fundMatError_f64( i_Vf_rt.fundMatError_f64 )
            , designWorldPt_t( i_Vf_rt.designWorldPt_t )
        {
            trackList_x.copy_v(i_Vf_rt.trackList_x, i_Vf_rt.trackList_x.size_u32());
        }
        ValidFeature& operator = ( const ValidFeature& i_Vf_rt )
        {
            if (this != &i_Vf_rt) {
            motionVector_o = i_Vf_rt.motionVector_o;
            worldPt_t = i_Vf_rt.worldPt_t;
            fundMatError_f64 = i_Vf_rt.fundMatError_f64;
            designWorldPt_t = i_Vf_rt.designWorldPt_t;
            trackList_x.copy_v(i_Vf_rt.trackList_x, i_Vf_rt.trackList_x.size_u32());
            }
            return  *this;
        }

        explicit ValidFeature( FeatureTrack &b_FT_ro )
            : worldPt_t( Point3d() )
        {
            motionVector_o = b_FT_ro.getTotalMV_o();
            fundMatError_f64 = b_FT_ro.getTrckLst_rx().back_ro().getFundMatErr_f64();
            designWorldPt_t = b_FT_ro.getDesignWorldPt_t();
            trackList_x.copy_v(b_FT_ro.getTrckLst_rx(), b_FT_ro.getTrckLst_rx().size_u32());   // PRQA S 3223
        }
        ValidFeature( MotionVector i_MotionVector_o,
            const mecl::core::ArrayList < ImageFeature, tsc_cfg::TRACK_LENGTH >* i_TrackList_px,
            float64_t i_FundMatError_f64, Point3d i_DesignWorldPt_t )
            : motionVector_o( i_MotionVector_o )
            , worldPt_t( Point3d() )
            , fundMatError_f64( i_FundMatError_f64 )
            , designWorldPt_t( i_DesignWorldPt_t )
        {
            trackList_x.copy_v(*i_TrackList_px, i_TrackList_px->size_u32());
        }
        ValidFeature( MotionVector i_MotionVector_o,
            const mecl::core::ArrayList < ImageFeature, tsc_cfg::TRACK_LENGTH >* i_TrackList_px,
            float64_t i_FundMatError_f64 )
            : motionVector_o( i_MotionVector_o )
            , worldPt_t( Point3d() )
            , fundMatError_f64( i_FundMatError_f64 )
            , designWorldPt_t( Point3d() )
        {
            trackList_x.copy_v(*i_TrackList_px, i_TrackList_px->size_u32());
        }
        Point3d getWorldPt_t() const { return worldPt_t; }
        Point3d getDesignWorldPt_t() const { return designWorldPt_t; }
        void PutWorldPt( Point3d i_Param_t ) { worldPt_t = i_Param_t; }
        MotionVector getMotionVector_o() const { return motionVector_o; }
        float64_t getFundMatError_f64() const { return fundMatError_f64; }
        void PutFundMatError(float64_t i_Param_f64) { fundMatError_f64 = i_Param_f64; }
        void PutDesignWorldPt(Point3d i_Param_t) { designWorldPt_t = i_Param_t; }
        mecl::core::ArrayList <ImageFeature, tsc_cfg::TRACK_LENGTH > &getTrackList_rx() { return trackList_x; }

        private:
        MotionVector motionVector_o; // --- vehicle motion vector between start and end frame
        Point3d worldPt_t; // --- Corresponding world point of this ValidFeature
        float64_t fundMatError_f64; // --- the fundamental matrix test error between start and end pt
        Point3d designWorldPt_t;
        mecl::core::ArrayList < ImageFeature, tsc_cfg::TRACK_LENGTH > trackList_x; // --- tracked list of feature across frames
    };

    //-------------------------------------------------------------------------
    class DiagnosticData
    {
        public:
        DiagnosticData() :
            numValidFeatures_u32( 0 ),
            numIgnoredValidFeatures_u32( 0 ),
            numInvalidFeatures_u32( 0 ),
            numUndetectedFeatures_u32( 0 ),
            numSkippedFrames_u32( 0 ),
            numProcessedFrames_u32( 0 ),
            numValidFrames_u32( 0 )
        {
            overlays_x.clear_v();
        }
        void reset_v(void)
        {
            numValidFeatures_u32 = 0;
            numIgnoredValidFeatures_u32 = 0;
            numInvalidFeatures_u32 = 0;
            numUndetectedFeatures_u32 = 0;
            numSkippedFrames_u32 = 0;
            numProcessedFrames_u32 = 0;
            numValidFrames_u32 = 0;
            overlays_x.clear_v();
        }
        void restoreValidFeatures_v(uint32_t i_Features_u32) { numValidFeatures_u32 = i_Features_u32; }
        void restoreProcessedFrames_v(uint32_t i_Frames_u32) { numProcessedFrames_u32 = i_Frames_u32; }
        void addValidFeatures_v(uint32_t i_Count_u32) { numValidFeatures_u32+= i_Count_u32; }
        uint32_t getNumValidFeatures_u32(void) const { return numValidFeatures_u32; }
        void addIgnoredValidFeatures_v(uint32_t i_Count_u32) { numIgnoredValidFeatures_u32+= i_Count_u32; }
        uint32_t getNumIgnoredValidFeatures_u32(void) const { return numIgnoredValidFeatures_u32; }
        void incInvalidFeatures_v(void) { numInvalidFeatures_u32++; }
        uint32_t getNumInvalidFeatures_u32(void) const { return numInvalidFeatures_u32; }
        void incUndetectedFeatures_v(void) { numUndetectedFeatures_u32++; }
        uint32_t getNumUndetectedFeatures_u32(void) const { return numUndetectedFeatures_u32; }
        void incSkippedFrames_v(void) { numSkippedFrames_u32++; }
        uint32_t getNumSkippedFrames_u32(void) const { return numSkippedFrames_u32; }
        void incProcessedFrames_v(void) { numProcessedFrames_u32++; }
        uint32_t getNumProcessedFrames_u32(void) const { return numProcessedFrames_u32; }
        void putNumVldFrms_v(uint32_t i_Param_u32) { numValidFrames_u32 = i_Param_u32; }
        void incNumVldFrms_v(void) { ++numValidFrames_u32; }
        uint32_t getNumVldFrms_u32(void) const { return numValidFrames_u32; }
        mecl::core::ArrayList< tscApi::DebugOverlay_s, tsc_cfg::NUM_VALID_INDICES > &getOverlays_rx() { return overlays_x; }
//#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
        mecl::core::ArrayList< ValidFeature, tsc_cfg::NUM_VALID_INDICES > &getValidFeatures_u32() { return validFeatures; }
//#endif
        tscApi::enuCameraID tscCameraTarget;
        private:
        mecl::core::ArrayList< tscApi::DebugOverlay_s, tsc_cfg::NUM_VALID_INDICES > overlays_x;
//#if defined (DEBUG) && defined (TRACING) && defined (APP_CTRL)
        mecl::core::ArrayList < ValidFeature,tsc_cfg::NUM_VALID_INDICES > validFeatures;  //Valid Features per Frame
//#endif
        uint32_t numValidFeatures_u32;
        uint32_t numIgnoredValidFeatures_u32;
        uint32_t numInvalidFeatures_u32;
        uint32_t numUndetectedFeatures_u32;
        uint32_t numSkippedFrames_u32;
        uint32_t numProcessedFrames_u32;
        uint32_t numValidFrames_u32; // --- Number of frames contributing to the valid features
    };

    //-------------------------------------------------------------------------
//#ifdef ENABLE_SFM    // PRQA S 1070
    class ValidFeatureCollection
    {
        public:
        ValidFeatureCollection()  
        {
        }
        mecl::core::ArrayList < ValidFeature,tsc_cfg::NUM_VALID_FEATURES > &GetVldFtrs() { return validFeatures; }

        private:
        mecl::core::ArrayList < ValidFeature,tsc_cfg::NUM_VALID_FEATURES > validFeatures;
    };
//#endif

    //-----------------------------------------------------------------------
    class Descriptor
    {
        public:
        Descriptor()
            : numValidFrames_u32( 0 )
            , version_u16( tsc_cfg::TSC_SAVED_DATA_VER )
            , cameraID_t( static_cast<tscApi::enuCameraID>(0) )
            , numRawFrames_u32( 0 )
            , numValidFeatures_u32( 0 )
        {
        }
        void reset_v( void )
        {
            numValidFrames_u32 = 0;
            numRawFrames_u32 = 0;
            numValidFeatures_u32 = 0;
        }
        void SetCameraID( tscApi::enuCameraID i_ID_t ) { cameraID_t = i_ID_t; }
        void SetRawFrame( uint32_t i_Num_u32 ) { numRawFrames_u32 = i_Num_u32; }
        void SetValidFrame( uint32_t i_Num_u32) { numValidFrames_u32 = i_Num_u32; }
        void SetValidFeatures( uint32_t i_Num_u32) { numValidFeatures_u32 = i_Num_u32; }
        tscApi::enuCameraID getCameraID_t( void ) const { return cameraID_t; }
        uint16_t getVer_u16( void ) const { return version_u16; }
        uint32_t getValidFrames_u32( void ) const { return numValidFrames_u32; }
        uint32_t getRawFrames_u32( void ) const { return numRawFrames_u32; }
        uint32_t getValidFeatures_u32( void ) const { return numValidFeatures_u32; }

        private:
        uint32_t numValidFrames_u32;
        uint16_t version_u16;
        tscApi::enuCameraID cameraID_t;
        uint32_t numRawFrames_u32;
        uint32_t numValidFeatures_u32;
    };

    //-----------------------------------------------------------------------
    class InitialGuess
    {
        public:
        InitialGuess()
            : pitch_f64( 0 )
            , yaw_f64( 0 )
            , roll_f64( 0 )
//            , Z_f64( 0 )
        {
        }
        InitialGuess( float64_t i_Pitch_f64, float64_t i_Yaw_f64, float64_t i_Roll_f64, float64_t i_Z_f64 )
            : pitch_f64( i_Pitch_f64 )
            , yaw_f64( i_Yaw_f64 )
            , roll_f64( i_Roll_f64 )
//            , Z_f64( i_Z_f64 )
        {
        }
        
        float64_t getPitch_f64() const { return pitch_f64; }
        float64_t getYaw_f64() const { return yaw_f64; }
        float64_t getRoll_f64() const { return roll_f64; }
//        float64_t getZ_f64() const { return Z_f64; }
        void PutPitch(float64_t i_Pitch_f64) { pitch_f64 = i_Pitch_f64; }
        void PutYaw(float64_t i_Yaw_f64)     { yaw_f64 = i_Yaw_f64; }
        void PutRoll(float64_t i_Roll_f64)   { roll_f64 = i_Roll_f64; }
//        void PutZ( float64_t i_Z_f64 ) { Z_f64 = i_Z_f64; }
        void reset_v()
        {
            pitch_f64 = 0.0;
            yaw_f64 = 0.0;
            roll_f64 = 0.0;
//            Z_f64 = 0.0;;
        }

        private:
        float64_t pitch_f64;
        float64_t yaw_f64;
        float64_t roll_f64;
//        float64_t Z_f64;
    };

    //-----------------------------------------------------------------------
    class SavedRecord
    {
        public:
        SavedRecord()
            :numInitialGuesses_s16(0)
#ifdef ENABLE_SFM    // PRQA S 1070
            ,numValidFeatures_u32(0)
#endif
        {
#ifdef ENABLE_SFM    // PRQA S 1070
            memset(m_ValidFeatures, 0, sizeof(m_ValidFeatures));
#endif
            memset(initialGuesses_ao, 0, sizeof(initialGuesses_ao));
        }
        SavedRecord& operator = ( const SavedRecord& i_Sr_rt )  // PRQA S 4054
        {
            if (this != &i_Sr_rt) {
#ifdef ENABLE_SFM    // PRQA S 1070
            for (sint16_t i = 0; i < i_Sr_rt.numValidFeatures_u32; ++i)
            {
                m_ValidFeatures[i] = i_Sr_rt.m_ValidFeatures[i];
            }
            numValidFeatures_u32 = i_Sr_rt.numValidFeatures_u32;
#endif
            for (sint16_t v_Index_s16 = 0; v_Index_s16 < i_Sr_rt.numInitialGuesses_s16; ++v_Index_s16)
            {
                initialGuesses_ao[v_Index_s16] = i_Sr_rt.initialGuesses_ao[v_Index_s16];
            }
            numInitialGuesses_s16 = i_Sr_rt.numInitialGuesses_s16;
            }
            return *this;
        }
#ifdef ENABLE_SFM    // PRQA S 1070
        void AddValidFeature( ValidFeature vf )
        {
            if (numValidFeatures_u32 < tsc_cfg::NUM_VALID_INDICES)
            {
                m_ValidFeatures[numValidFeatures_u32] = vf;
                numValidFeatures_u32++;
            }
        }
#endif
        void AddInitialGuess( InitialGuess i_IG_o )
        {
            if (numInitialGuesses_s16 < tsc_cfg::NUM_INITIAL_GUESSES_PER_FRAME)
            {
                initialGuesses_ao[numInitialGuesses_s16] = i_IG_o;
                numInitialGuesses_s16++;
            }
        }
#ifdef ENABLE_SFM    // PRQA S 1070
        sint16_t getNumValidFeatures_u32() const { return numValidFeatures_u32; }
        ValidFeature& GetValidFeature(uint32_t i) { return m_ValidFeatures[i]; }
#endif
        sint16_t getNumInitialGuesses_s16() const { return numInitialGuesses_s16; }
        InitialGuess& getInitialGuess_ro(uint32_t i_Index_u32) { return initialGuesses_ao[i_Index_u32]; }
 
        private:
#ifdef ENABLE_SFM    // PRQA S 1070
        sint16_t numValidFeatures_u32;
        ValidFeature m_ValidFeatures[tsc_cfg::NUM_VALID_INDICES];
#endif
        sint16_t numInitialGuesses_s16;
        InitialGuess initialGuesses_ao[tsc_cfg::NUM_INITIAL_GUESSES_PER_FRAME];
     };

    //-----------------------------------------------------------------------
    class TSCSavedData
    {
        public:
        TSCSavedData()
            : descriptor_o( Descriptor() )
        {
            numRecord_u32 = 0;
            memset( savedRecords_ao, 0 , sizeof(savedRecords_ao));
        }
        void reset_v( void )
        {
            descriptor_o.reset_v();
            numRecord_u32 = 0;
            memset( savedRecords_ao, 0 , sizeof(savedRecords_ao));
        }
        bool_t AddRecord( const SavedRecord i_SR_o )
        {
            bool_t v_Ret_b = false;
            if (numRecord_u32 < tsc_cfg::MIN_NUM_RAW_FRAMES)
            {
                savedRecords_ao[numRecord_u32] = i_SR_o;
                numRecord_u32++;
                v_Ret_b = true;
            }
            return v_Ret_b;
        }
        bool_t removeLastRecord_b( void )
        {
            bool_t v_Ret_b = false;
            if (numRecord_u32 > 0)
            {
                numRecord_u32--;
                v_Ret_b = true;
            }
            return v_Ret_b;
        }
        Descriptor &getDescriptor_ro() { return descriptor_o; }
        uint32_t getNumOfRecord_u32() const { return numRecord_u32; }
        SavedRecord& getSavedRecord_ro(uint32_t i_Index_u32) { return savedRecords_ao[i_Index_u32]; }
        uint32_t getRecordLength_u32() const { return ( sizeof(savedRecords_ao[0]) ); }
        uint32_t getHeaderLength_u32() const { return ( reinterpret_cast<uint64_t>(&savedRecords_ao[0]) - reinterpret_cast<uint64_t>(&descriptor_o) ); }   // PRQA S 3044
        uint32_t getValidLength_u32() const { return getHeaderLength_u32() + numRecord_u32 * getRecordLength_u32(); }
        uint32_t getArraySize_u32() const { return sizeof(savedRecords_ao); }

        private:
        Descriptor descriptor_o;
        uint32_t numRecord_u32;
        SavedRecord savedRecords_ao[tsc_cfg::MIN_NUM_RAW_FRAMES];
    };
}
#endif
