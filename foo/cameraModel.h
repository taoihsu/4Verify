// ----------------------------------------------------------------------------
// --- Written by X.Phan [28-Jun-2012]
// --- Modified by Ehsan Parvizi [29-May-2014]
// --- Copyright (c) Magna Vectrics (MEVC) 2014
// ----------------------------------------------------------------------------
// --- CameraModel.h - CameraModel related parameter struct/methods
// ----------------------------------------------------------------------------
#ifndef __CAMERAMODEL_H_
#define __CAMERAMODEL_H_

#ifdef USE_SVSCM
// ----------------------------------------------------------------------------
#include <cstring>  //memset
#include "mecl/mecl.h"
#include <math.h>    //cos, sin
#include <climits>
#include "mathOperations.h"
#include "point_oc.h"
// ----------------------------------------------------------------------------
// Suppress the QACPP MISRA warning regarding use of floating point numbers
// PRQA S 3708 EOF

// CameraModel class
namespace camera_model
{
class CameraModel
{
    public:
    explicit CameraModel( tscApi::enuCameraID _id = tscApi::e_TscFrontCam, bool_t _isValid = false ):  \
        ID( _id ),  \
        isValid( _isValid ),  \
        intrinsic( IntrinsicParams() ),  \
        extrinsic( ExtrinsicParams() ),  \
        orientation( Orientation() ) 
    {
        // Using initialization lists
    }

    // --- camera ID
    // --- Intrinsic parameters
    class IntrinsicParams
    {
      public:
        IntrinsicParams():
                pixelSize(0.0),
                focalLength(0.0),
                R0(0.0),
                radialCoeff1(0.0),
                radialCoeff2(0.0),
                radialCoeff3(0.0),
                tangentialCoeff1(0.0),
                tangentialCoeff2(0.0),
                affineCoeff1(0.0),
                affineCoeff2(0.0),
                ppx(0.0),
                ppy(0.0),
                origX(0.0),
                origY(0.0),
                padLength(0.0),
                downsampleFactor(0),
                axisX(0.0),
                axisY(0.0)
        {
            memset( this, 0, sizeof( IntrinsicParams ) );
        }

        float64_t GetPixelS(void) const { return pixelSize; }
        float64_t GetFocalL(void) const { return focalLength; }
        float64_t GetR0(void) const { return R0; }
        float64_t GetRadialC1(void) const { return radialCoeff1; }
        float64_t GetRadialC2(void) const { return radialCoeff2; }
        float64_t GetRadialC3(void) const { return radialCoeff3; }
        float64_t GetTangentialC1(void) const { return tangentialCoeff1; }
        float64_t GetTangentialC2(void) const { return tangentialCoeff2; }
        float64_t GetAffineC1(void) const { return affineCoeff1; }
        float64_t GetAffineC2(void) const { return affineCoeff2; }
        float64_t GetPpx(void) const { return ppx; }
        float64_t GetPpy(void) const { return ppy; }
        float64_t GetOrigX(void) const { return origX; }
        float64_t GetOrigY(void) const { return origY; }
        float64_t GetPadLen(void) const { return padLength; }
        uint32_t GetDwnSmplFctr(void) const { return downsampleFactor; }
        float64_t GetAxisX(void) const { return axisX; }
        float64_t GetAxisY(void) const { return axisY; }

        float64_t (&GetKPtr(void))[3][3] { return K; }
        float64_t (&GetKInvPtr(void))[3][3] { return invK; }

        void PutPixelS(float64_t param) { pixelSize = param; }
        void PutFocalL(float64_t param) { focalLength = param; }
        void PutR0(float64_t param) { R0 = param; }
        void PutRadialC1(float64_t param) { radialCoeff1 = param; }
        void PutRadialC2(float64_t param) { radialCoeff2 = param; }
        void PutRadialC3(float64_t param) { radialCoeff3 = param; }
        void PutTangentialC1(float64_t param) { tangentialCoeff1 = param; }
        void PutTangentialC2(float64_t param) { tangentialCoeff2 = param; }
        void PutAffineC1(float64_t param) { affineCoeff1 = param; }
        void PutAffineC2(float64_t param) { affineCoeff2 = param; }
        void PutPpx(float64_t param) { ppx = param; }
        void PutPpy(float64_t param) { ppy = param; }
        void PutOrigX(float64_t param) { origX = param; }
        void PutOrigY(float64_t param) { origY = param; }
        void PutDwnSmplFctr(uint32_t param) { downsampleFactor = param; }
        void PutAxisX(float64_t param) { axisX = param; }
        void PutAxisY(float64_t param) { axisY = param; }
      private:
        float64_t pixelSize;
        float64_t focalLength;
        float64_t R0;
        float64_t radialCoeff1; // --- Radial distortion coefficients
        float64_t radialCoeff2; // --- Radial distortion coefficients
        float64_t radialCoeff3; // --- Radial distortion coefficients
        float64_t tangentialCoeff1; // --- Tangential distortion coefficients
        float64_t tangentialCoeff2; // --- Tangential distortion coefficients
        float64_t affineCoeff1; // --- Affine distortion coefficients
        float64_t affineCoeff2; // --- Affine distortion coefficients
        float64_t ppx; // --- internal principal point
        float64_t ppy; // --- internal principal point
        float64_t origX;
        float64_t origY;
        float64_t padLength;
        uint32_t downsampleFactor;
        float64_t axisX; // --- camera principal point (for K matrix)
        float64_t axisY; // --- camera principal point (for K matrix)
        float64_t K[ 3][ 3 ]; // --- derived K matrix
        float64_t invK[ 3][ 3 ]; // --- derived inverse of K matrix
    };

    // --- Extrinsic parameters
    class ExtrinsicParams
    {
      public:
        explicit ExtrinsicParams(  \
            float64_t _pitch_deg = 0, \
            float64_t _yaw_deg = 0, \
            float64_t _roll_deg = 0,  \
            float64_t _x_mm = 0, \
            float64_t _y_mm = 0, \
            float64_t _z_mm = 0, \
            bool_t _flipped = false ) :  \
            Pitch_deg( _pitch_deg ),  \
            Yaw_deg( _yaw_deg ),  \
            Roll_deg( _roll_deg ),  \
            X_mm( _x_mm ),  \
            Y_mm( _y_mm ),  \
            Z_mm( _z_mm ),  \
            flipped( _flipped )
        {
            // Using initialization lists
        }

        float64_t GetPtchDg(void) const { return Pitch_deg; }
        float64_t GetYwDg(void) const { return Yaw_deg; }
        float64_t GetRllDg(void) const { return Roll_deg; }
        float64_t GetXmm(void) const { return X_mm; }
        float64_t GetYmm(void) const { return Y_mm; }
        float64_t GetZmm(void) const { return Z_mm; }
        bool_t GetFlppd(void) const { return flipped; }

        void PutPtchDg(float64_t param) { Pitch_deg = param; }
        void PutYwDg(float64_t param) { Yaw_deg = param; }
        void PutRllDg(float64_t param) {
            Roll_deg = param;
            tsc_math::toRange(Roll_deg, 360.0);
        }
        void PutRllDgMinus(float64_t param) { Roll_deg -= param; }
        void PutXmm(float64_t param) { X_mm = param; }
        void PutYmm(float64_t param) { Y_mm = param; }
        void PutZmm(float64_t param) { Z_mm = param; }
        void PutFlppd(bool_t param) { flipped = param; }

        bool_t IsRollIn14Quadrants() const
        {
            return( ((Roll_deg >= -90.0) && (Roll_deg < 90.0)) || ((Roll_deg >= 270.0) && (Roll_deg < 360.0)) );
        }
        bool_t IsRollIn23Quadrants() const
        {
            return( ((Roll_deg >= 90.0) && (Roll_deg < 270.0)) || ((Roll_deg >= -270.0) && (Roll_deg < -90.0)) );
        }

      private:
        float64_t Pitch_deg;
        float64_t Yaw_deg;
        float64_t Roll_deg;
        float64_t X_mm;
        float64_t Y_mm;
        float64_t Z_mm;
        bool_t flipped; // --- whether the camera is flipped horizontally
    };

    // --- Orientation parameters
    class Orientation
    {
      public:
        Orientation():
                cameraXOffset_mm(0.0),
                cameraYOffset_mm(0.0),
                cameraPreRoll_deg(0.0)
        {
            memset( this, 0, sizeof( Orientation ) );
        }

        float64_t GetCmrXOffst(void) const { return cameraXOffset_mm; }
        float64_t GetCmrYOffst(void) const { return cameraYOffset_mm; }
        float64_t GetCmrPrRll(void) const { return cameraPreRoll_deg; }

        void PutCmrXOffst(float64_t param) { cameraXOffset_mm = param; }
        void PutCmrYOffst(float64_t param) { cameraYOffset_mm = param; }
        void PutCmrPrRll(float64_t param) { cameraPreRoll_deg = param; }

      private:
        // --- needed to set the kinematic model
        float64_t cameraXOffset_mm; // --- In vehicle coordinate system
        float64_t cameraYOffset_mm; // --- In vehicle coordinate system
        float64_t cameraPreRoll_deg; // --- In vehicle coordinate system
    };


    tscApi::enuCameraID GetID() const          { return ID; }
    bool_t Valid() const               { return isValid;}
    IntrinsicParams &GetIntrinsic()    { return intrinsic; }
    ExtrinsicParams &GetExtrinsic()    { return extrinsic; }
    Orientation &GetOrientation()      { return orientation; }
    float64_t (&GetRstdPtr(void))[3][3]{ return R_std; }
#ifdef ENABLE_SFM   // PRQA S 1070
    float64_t (&GetTPtr(void))[3][4]   { return T; }
#endif
    // setter
    void SetValid()   { isValid = true; }
    void ClearValid() { isValid = false; }
    void PutID(tscApi::enuCameraID cameraID) { ID = cameraID; }

    // --- load configuration
    bool_t LoadIntrinsicParamConfig(tscApi::cameraModelConfig_Type const * initCameraModelConfigPtr);
    bool_t LoadExtrinsicParamConfig( tscApi::cameraModelConfig_Type const * initCameraModelConfigPtr);
    bool_t LoadOrientationParamConfig( tscApi::cameraModelConfig_Type const * initCameraModelConfigPtr);
    // --- Unwarp
    bool_t Unwarp( const fc::Point& warpedPt, fc::Pointd& unwarpedPt ) const;
    // --- Unwarp
    bool_t Warp( const fc::Pointd& unwarpedPt, fc::Point& warpedPt ) const;

    // --- static helper extrinsic methods used in dependent modules

    // --- Build Z Axis Rotation matrix
    static bool_t BuildZAxisRotationMatrix( const float64_t angDeg, float64_t (&R)[ 3][ 3 ] );
    // --- Build Y Axis Rotation matrix
    static bool_t BuildYAxisRotationMatrix( const float64_t angDeg, float64_t (&R)[ 3][ 3 ] );
    // --- Build X Axis Rotation matrix
    static bool_t BuildXAxisRotationMatrix( const float64_t angDeg, float64_t (&R)[ 3][ 3 ] );
    // --- Build Extrinsic Rotation matrix
    static bool_t BuildRotationMatrix( const float64_t (&Ra)[ 3][ 3 ], const float64_t (&Rb)[ 3][ 3 ], const float64_t (&Rg)[ 3][ 3 ], float64_t (&R)[ 3][ 3 ] );
    // --- Build Translation matrix
    static bool_t BuildTranslationMatrix( const float64_t tx, const float64_t ty, const float64_t tz, float64_t (&T)[ 3][ 4 ] );
    // --- Build Motion-Vector Transformation matrix
    static bool_t BuildMotionTransformation( const float64_t deltaX, const float64_t deltaY, const float64_t deltaPsiRad, float64_t (&MotionTrans)[ 4][ 4 ] );
    // --- Build Projection Matrix
    static bool_t BuildProjectionMatrix( const float64_t (&K)[ 3][ 3 ], const float64_t (&R)[ 3][ 3 ], float64_t (&T)[ 3][ 4 ], float64_t (&P)[ 3][ 4 ] );

    private:
    // --- ApplyUndistortion
    bool_t ApplyUndistortion( fc::Pointd& ioPt, const bool_t inRotate ) const;
    // --- ApplyDistortion
    bool_t ApplyDistortion( fc::Pointd& ioPt, const uint8_t inLevel, const bool_t inRotate ) const;
    // --- ApplyLensCorrection
    void ApplyLensCorrection( float64_t *outOffsetX, float64_t *outOffsetY, const float64_t inX, const float64_t inY, const bool_t inRotate ) const;
    // --- PixelToMetric
    void PixelToMetric( float64_t& x, float64_t& y ) const;
    // --- MetricToPixel
    bool_t MetricToPixel( float64_t& x, float64_t& y ) const;
    tscApi::enuCameraID ID;
    // --- Whether camera parameters are valid
    bool_t isValid;
    
    IntrinsicParams intrinsic;
    ExtrinsicParams extrinsic;
    Orientation orientation;
    float64_t R_std[ 3][ 3 ];
#ifdef ENABLE_SFM   // PRQA S 1070
    float64_t T[ 3][ 4 ];
#endif
};

inline bool_t CameraModel::Warp( const fc::Pointd& unwarpedPt, fc::Point& warpedPt ) const
{
    fc::Pointd ioPt;
    bool_t retValue = true;

    if( extrinsic.GetFlppd() )
    {
        ioPt.x_x = ( intrinsic.GetOrigX() - 1.0 ) - ( unwarpedPt.x_x - ( static_cast<float64_t>(intrinsic.GetPadLen()) ));
        ioPt.y_x = ( intrinsic.GetOrigY() - 1.0 ) - ( unwarpedPt.y_x - ( static_cast<float64_t>(intrinsic.GetPadLen()) ));
    }
    else
    {
        ioPt.x_x = unwarpedPt.x_x - static_cast<float64_t>(intrinsic.GetPadLen());
        ioPt.y_x = unwarpedPt.y_x - static_cast<float64_t>(intrinsic.GetPadLen());
    }

    retValue = ApplyDistortion( ioPt, 0, false );

    if( extrinsic.GetFlppd() )
    {
        ioPt.x_x = ( intrinsic.GetOrigX() - 1.0 ) - ioPt.x_x;
        ioPt.y_x = ( intrinsic.GetOrigY() - 1.0 ) - ioPt.y_x;
    }

    ioPt.x_x /= static_cast<float64_t>(intrinsic.GetDwnSmplFctr());
    ioPt.y_x /= static_cast<float64_t>(intrinsic.GetDwnSmplFctr());

    // Convert to the nearest integer (hence: + 0.5)
    warpedPt.x_x = static_cast < sint32_t > ( ioPt.x_x + 0.5 );
    warpedPt.y_x = static_cast < sint32_t > ( ioPt.y_x + 0.5 );

    return retValue;
}

//-------------------------------------------------------------------------

inline bool_t CameraModel::Unwarp( const fc::Point& warpedPt, fc::Pointd& unwarpedPt ) const
{
    bool_t retValue = true;
    fc::Pointd ioPt(  \
        static_cast<float64_t>( warpedPt.x_x * intrinsic.GetDwnSmplFctr() ),  \
        static_cast<float64_t>( warpedPt.y_x * intrinsic.GetDwnSmplFctr() ) );

    if( extrinsic.GetFlppd() )
    {
        ioPt.x_x = ( intrinsic.GetOrigX() - 1.0 ) - ioPt.x_x;
        ioPt.y_x = ( intrinsic.GetOrigY() - 1.0 ) - ioPt.y_x;
    }

    retValue = ApplyUndistortion( ioPt, false );

    if ( retValue == true )
    {
        if( extrinsic.GetFlppd() )
        {
            unwarpedPt.x_x = ( intrinsic.GetOrigX() - 1.0 ) - ioPt.x_x + static_cast<float64_t>(intrinsic.GetPadLen());
            unwarpedPt.y_x = ( intrinsic.GetOrigY() - 1.0 ) - ioPt.y_x + static_cast<float64_t>(intrinsic.GetPadLen());
        }
        else
        {
            unwarpedPt.x_x = ioPt.x_x + static_cast<float64_t>(intrinsic.GetPadLen());
            unwarpedPt.y_x = ioPt.y_x + static_cast<float64_t>(intrinsic.GetPadLen());
        }
    }

    return retValue;
}

//-------------------------------------------------------------------------
inline bool_t CameraModel::ApplyDistortion( fc::Pointd& ioPt, const uint8_t inLevel, const bool_t inRotate ) const
{
    static const float64_t kDesignF = 0.895;
    bool_t retValue = true;

    // lens model polynomial coefficients
    static const float64_t kLensParams[ 6 ] =
    {
        0.000141931846539,  \
        0.886515027524824,  \
        0.113431880193239,  \
        - 0.060232115075806,  \
        0.229182402913814,  \
        - 0.103107427814684
    };

    float64_t X = ioPt.x_x;
    float64_t Y = ioPt.y_x;

    PixelToMetric( X, Y );

    float64_t offX = 0.0;
    float64_t offY = 0.0;

    /* get intrinsic correction offsets */
    ApplyLensCorrection( &offX, &offY, X, Y, inRotate );

    X += offX;
    Y += offY;

    /* get pinhole radius */
    float64_t pinRadius = sqrt( X *X + Y * Y );

    /* get pinhole angle */
    float64_t pinAngle1 = atan( pinRadius / kDesignF );
    float64_t pinAngle2 = pinAngle1 * pinAngle1;
    float64_t pinAngle3 = pinAngle2 * pinAngle1;

    /* get distorted radius */
    float64_t corRadius =  \
        kLensParams[ 5 ] *pinAngle3 * pinAngle2 +  \
        kLensParams[ 4 ] *pinAngle2 * pinAngle2 +  \
        kLensParams[ 3 ] *pinAngle3 +  \
        kLensParams[ 2 ] *pinAngle2 +  \
        kLensParams[ 1 ] *pinAngle1 +  \
        kLensParams[ 0 ];

    if( inLevel != 0U )
    {
        /* apply level of distortion */
        pinRadius -= ( pinRadius - corRadius ) / 100.0 * static_cast<float64_t>(inLevel);
    }

    /* get correction factor */
    float64_t scale = corRadius / ( pinRadius + 1e-20 ); /* + 1e-20f to avoid division by zero */

    /* apply correction factor to pinhole pixel */
    X *= scale;
    Y *= scale;

    retValue = MetricToPixel( X, Y );

    ioPt.x_x = X;
    ioPt.y_x = Y;
    return retValue;
}

//-------------------------------------------------------------------------

inline bool_t CameraModel::ApplyUndistortion( fc::Pointd& ioPt, const bool_t inRotate ) const
{
    static const float64_t kDesignF = 0.895;
    bool_t retValue = true;

    // lens model polynomial coefficients
    static const float64_t kLensParams[ 6 ] =
    {
        - 0.000404504875527,  \
        1.128639383470241,  \
        - 0.121149804170641,  \
        - 0.108510659717601,  \
        0.053988842139022,  \
        - 0.002115458183709
    };

    float64_t X = ioPt.x_x;
    float64_t Y = ioPt.y_x;

    PixelToMetric( X, Y );

    /* get distorted radius */
    float64_t corRadius2 = X * X + Y * Y;
    float64_t corRadius1 = sqrt( corRadius2 );
    float64_t corRadius3 = corRadius2 * corRadius1;

    float64_t pinAngle =  \
        kLensParams[ 5 ] *corRadius3 * corRadius2 +  \
        kLensParams[ 4 ] *corRadius2 * corRadius2 +  \
        kLensParams[ 3 ] *corRadius3 +  \
        kLensParams[ 2 ] *corRadius2 +  \
        kLensParams[ 1 ] *corRadius1 +  \
        kLensParams[ 0 ];

    /* get pinhole radius */
    float64_t pinRadius = tan( pinAngle ) *kDesignF;

    /* get correction factor */
    float64_t scale = pinRadius / ( corRadius1 + 1e-20 ); /* + 1e-20f to avoid division by zero */

    /* apply correction factor to distorted pixel */
    X *= scale;
    Y *= scale;

    float64_t border = 1e-5;
    float64_t diffX = border;
    float64_t diffY = border;

    float64_t newX = X;
    float64_t newY = Y;

    float64_t oldX = X;
    float64_t oldY = Y;

    float64_t offX;
    float64_t offY;

    uint16_t iter = 0U;
    uint16_t maxIterations = 100U;

    while( ( iter < maxIterations ) )
    {
        /* get intrinsic correction offsets */
        ApplyLensCorrection( &offX, &offY, newX, newY, inRotate );

        newX = X - offX;
        newY = Y - offY;
        diffX = mecl::math::abs_x<float64_t>( newX - oldX );
        diffY = mecl::math::abs_x<float64_t>( newY - oldY );
        oldX = newX;
        oldY = newY;
        iter++;
        if ( (!(diffX >= border )) && (!( diffY >= border ) ) )
        {
          break;
        }
    }

    retValue = MetricToPixel( newX, newY );

    ioPt.x_x = newX;
    ioPt.y_x = newY;
    return retValue;
}

//-------------------------------------------------------------------------


inline void CameraModel::PixelToMetric( float64_t& x, float64_t& y ) const
{
    /* Convert pixel to millimeter and center origin */
    x = ( x - intrinsic.GetAxisX() ) * intrinsic.GetPixelS();
    y = ( y - intrinsic.GetAxisY() ) * ( - intrinsic.GetPixelS() );
}

//-------------------------------------------------------------------------

inline bool_t CameraModel::MetricToPixel( float64_t& x, float64_t& y ) const
{
    bool_t retValue = true;
    /* Convert millimeter to pixel and center origin */
    // Trap the possibility of dividing by zero. Allow this division
    // to occur (as this appears to be the intent in the code)
    // But flag by returning false.
    if ( (intrinsic.GetPixelS() ) == 0.0 )  // PRQA S 3270
    {
      retValue = false;
    }
    x = x / intrinsic.GetPixelS() + intrinsic.GetAxisX();
    y = y / ( - intrinsic.GetPixelS() ) + intrinsic.GetAxisY();
    return retValue;
}

//-------------------------------------------------------------------------

inline void CameraModel::ApplyLensCorrection( float64_t *outOffsetX, float64_t *outOffsetY, const float64_t inX, const float64_t inY, const bool_t inRotate ) const
{
    float64_t R2 = intrinsic.GetR0() * intrinsic.GetR0();
    float64_t R4 = R2 * R2;
    float64_t R6 = R4 * R2;

    float64_t B1_2 = intrinsic.GetTangentialC1() * 2.0;
    float64_t B2_2 = intrinsic.GetTangentialC2() * 2.0;

    float64_t XY = inX * inY;
    float64_t X2 = inX * inX;
    float64_t Y2 = inY * inY;

    float64_t pixRad2 = X2 + Y2;
    float64_t pixRad4 = pixRad2 * pixRad2;
    float64_t pixRad6 = pixRad4 * pixRad2;

    /* Radial symmetric distortion */
    float64_t corRad =  \
        ( pixRad2 - R2 ) * intrinsic.GetRadialC1() +  \
        ( pixRad4 - R4 ) * intrinsic.GetRadialC2() +  \
        ( pixRad6 - R6 ) * intrinsic.GetRadialC3();

    *outOffsetX = inX * corRad;
    *outOffsetY = inY * corRad;

    /* Tangential asymmetric distortion */
    *outOffsetX += ( 2.0 * X2 + pixRad2 ) *intrinsic.GetTangentialC1() + XY * B2_2;
    *outOffsetY += ( 2.0 * Y2 + pixRad2 ) *intrinsic.GetTangentialC2() + XY * B1_2;

    /* Affinity and shear */
    *outOffsetX += inX * intrinsic.GetAffineC1() + inY * intrinsic.GetAffineC2();

    /* Principal */
    if( inRotate )
    {
        *outOffsetX -= intrinsic.GetPpx();
        *outOffsetY -= intrinsic.GetPpy();
    }
    else
    {
        *outOffsetX += intrinsic.GetPpx();
        *outOffsetY += intrinsic.GetPpy();
    }
}

inline bool_t CameraModel::BuildProjectionMatrix( const float64_t (&K)[ 3][ 3 ], const float64_t (&R)[ 3][ 3 ], float64_t (&T)[ 3][ 4 ], float64_t (&P)[ 3][ 4 ] )
{
    bool_t ret = true;
    //
    // --- KR[3x3] = K[3x3] x R[3x3]
    static float64_t KR[ 3][ 3 ];
    bool_t result = tsc_math::MatrixMultiply( &K[ 0][ 0 ], 3, 3, false, &R[ 0][ 0 ], 3, 3, false, &KR[ 0][ 0 ] );

    if( !result )
    {
        ret = false;
    }
    else
    {
        //
        // --- P[3x4] = KR[3x3] x T[3x4]
        result = tsc_math::MatrixMultiply( &KR[ 0][ 0 ], 3, 3, false, &T[ 0][ 0 ], 3, 4, false, &P[ 0][ 0 ] );
        ret = result;
    }
    return ret;
}

//-----------------------------------------------------------------------------
inline bool_t CameraModel::BuildZAxisRotationMatrix( const float64_t angDeg, float64_t (&R)[ 3][ 3 ] )
{
    float64_t angRad = tsc_math::Degrees2Radians( angDeg );
    float64_t cosAngle = cos( angRad );
    float64_t sinAngle = sin( angRad );

    R[ 0][ 0 ] = cosAngle;
    R[ 0][ 1 ] = sinAngle;
    R[ 0][ 2 ] = 0;

    R[ 1][ 0 ] =  - sinAngle;
    R[ 1][ 1 ] = cosAngle;
    R[ 1][ 2 ] = 0;

    R[ 2][ 0 ] = 0;
    R[ 2][ 1 ] = 0;
    R[ 2][ 2 ] = 1.0;

    return true;
}

//-------------------------------------------------------------------------

inline bool_t CameraModel::BuildYAxisRotationMatrix( const float64_t angDeg, float64_t (&R)[ 3][ 3 ] )
{
    float64_t angRad = tsc_math::Degrees2Radians( angDeg );
    float64_t cosAngle = cos( angRad );
    float64_t sinAngle = sin( angRad );

    R[ 0][ 0 ] = cosAngle;
    R[ 0][ 1 ] = 0;
    R[ 0][ 2 ] =  - sinAngle;

    R[ 1][ 0 ] = 0;
    R[ 1][ 1 ] = 1.0;
    R[ 1][ 2 ] = 0;

    R[ 2][ 0 ] = sinAngle;
    R[ 2][ 1 ] = 0;
    R[ 2][ 2 ] = cosAngle;

    return true;
}

//-------------------------------------------------------------------------

inline bool_t CameraModel::BuildXAxisRotationMatrix( const float64_t angDeg, float64_t (&R)[ 3][ 3 ] )
{
    float64_t angRad = tsc_math::Degrees2Radians( angDeg );
    float64_t cosAngle = cos( angRad );
    float64_t sinAngle = sin( angRad );

    R[ 0][ 0 ] = 1.0;
    R[ 0][ 1 ] = 0;
    R[ 0][ 2 ] = 0;

    R[ 1][ 0 ] = 0;
    R[ 1][ 1 ] = cosAngle;
    R[ 1][ 2 ] = sinAngle;

    R[ 2][ 0 ] = 0;
    R[ 2][ 1 ] =  - sinAngle;
    R[ 2][ 2 ] = cosAngle;

    return true;
}

//-------------------------------------------------------------------------

inline bool_t CameraModel::BuildRotationMatrix( const float64_t (&Ra)[ 3][ 3 ], const float64_t (&Rb)[ 3][ 3 ], const float64_t (&Rg)[ 3][ 3 ], float64_t (&R)[ 3][ 3 ] )
{
    bool_t ret = true;
    // --- R[3x3] = Rg[3x3] x Rb[3x3]
    bool_t result = tsc_math::MatrixMultiply( &Rg[ 0][ 0 ], 3, 3, false, &Rb[ 0][ 0 ], 3, 3, false, &R[ 0][ 0 ] );
    if( !result )
    {
        ret = false;
    }
    else
    {
        // --- R[3x3] = R[3x3] x Ra[3x3]
        result = tsc_math::MatrixMultiply( &R[ 0][ 0 ], 3, 3, false, &Ra[ 0][ 0 ], 3, 3, false, &R[ 0][ 0 ] );
        ret = result;
    }

    return ret;
}

//-------------------------------------------------------------------------

inline bool_t CameraModel::BuildTranslationMatrix( const float64_t tx, const float64_t ty, const float64_t tz, float64_t (&T)[ 3][ 4 ] )
{
    T[ 0][ 0 ] = 1.0;
    T[ 0][ 1 ] = 0;
    T[ 0][ 2 ] = 0;
    T[ 0][ 3 ] = -tx;

    T[ 1][ 0 ] = 0;
    T[ 1][ 1 ] = 1.0;
    T[ 1][ 2 ] = 0;
    T[ 1][ 3 ] = -ty;

    T[ 2][ 0 ] = 0;
    T[ 2][ 1 ] = 0;
    T[ 2][ 2 ] = 1.0;
    T[ 2][ 3 ] = -tz;

    return true;
}

//-------------------------------------------------------------------------

inline bool_t CameraModel::BuildMotionTransformation( const float64_t deltaX, const float64_t deltaY, const float64_t deltaPsiRad, float64_t (&MotionTrans)[ 4][ 4 ] )
{
    float64_t cosPsi = cos( deltaPsiRad );
    float64_t sinPsi = sin( deltaPsiRad );

    MotionTrans[ 0][ 0 ] = cosPsi;
    MotionTrans[ 0][ 1 ] = sinPsi;
    MotionTrans[ 0][ 2 ] = 0;
    MotionTrans[ 0][ 3 ] =  - deltaX * cosPsi - deltaY * sinPsi;

    MotionTrans[ 1][ 0 ] =  - sinPsi;
    MotionTrans[ 1][ 1 ] = cosPsi;
    MotionTrans[ 1][ 2 ] = 0;
    MotionTrans[ 1][ 3 ] = deltaX * sinPsi - deltaY * cosPsi;

    MotionTrans[ 2][ 0 ] = 0;
    MotionTrans[ 2][ 1 ] = 0;
    MotionTrans[ 2][ 2 ] = 1.0;
    MotionTrans[ 2][ 3 ] = 0;

    MotionTrans[ 3][ 0 ] = 0;
    MotionTrans[ 3][ 1 ] = 0;
    MotionTrans[ 3][ 2 ] = 0;
    MotionTrans[ 3][ 3 ] = 1.0;
    return true;
}

inline bool_t CameraModel::LoadIntrinsicParamConfig(tscApi::cameraModelConfig_Type const * initCameraModelConfigPtr)
{
    bool_t ret = true;

    /* Load the intrinsic camera parameters from the initial configuration */

    intrinsic.PutPixelS(initCameraModelConfigPtr->intrinsicParams.pixelSize);

    intrinsic.PutFocalL(initCameraModelConfigPtr->intrinsicParams.focalLength);

    float64_t radialZeroCrossing = initCameraModelConfigPtr->intrinsicParams.radialZeroCrossing;

    intrinsic.PutRadialC1(initCameraModelConfigPtr->intrinsicParams.radialCoeff1);

    intrinsic.PutRadialC2(initCameraModelConfigPtr->intrinsicParams.radialCoeff2);

    intrinsic.PutRadialC3(initCameraModelConfigPtr->intrinsicParams.radialCoeff3);

    intrinsic.PutTangentialC1(initCameraModelConfigPtr->intrinsicParams.tangentialCoeff1);

    intrinsic.PutTangentialC2(initCameraModelConfigPtr->intrinsicParams.tangentialCoeff2);

    intrinsic.PutAffineC1(initCameraModelConfigPtr->intrinsicParams.affineCoeff1);

    intrinsic.PutAffineC2(initCameraModelConfigPtr->intrinsicParams.affineCoeff2);

    float64_t ppx_pixel = initCameraModelConfigPtr->intrinsicParams.ppx;

    float64_t ppy_pixel = initCameraModelConfigPtr->intrinsicParams.ppy;

    intrinsic.PutOrigX(initCameraModelConfigPtr->intrinsicParams.origX);

    intrinsic.PutOrigY(initCameraModelConfigPtr->intrinsicParams.origY);

    intrinsic.PutAxisX(initCameraModelConfigPtr->intrinsicParams.axisX);

    intrinsic.PutAxisY(initCameraModelConfigPtr->intrinsicParams.axisY);

    intrinsic.PutDwnSmplFctr(initCameraModelConfigPtr->intrinsicParams.downsampleFactor);

    // --- ppx and ppy
    intrinsic.PutPpx(- intrinsic.GetPixelS() *( intrinsic.GetOrigX() / static_cast<float64_t>(2)-ppx_pixel ));
    intrinsic.PutPpy(- intrinsic.GetPixelS() *( intrinsic.GetOrigY() / static_cast<float64_t>(2)-ppy_pixel ));

    // --- R0
    intrinsic.PutR0(intrinsic.GetPixelS() * radialZeroCrossing);

    // PRQA S 3706 ++
    // PRQA S 3222 ++
    intrinsic.GetKPtr()[ 0][ 0 ] =  - intrinsic.GetFocalL() / intrinsic.GetPixelS();
    intrinsic.GetKPtr()[ 0][ 1 ] = 0;
    intrinsic.GetKPtr()[ 0][ 2 ] = intrinsic.GetAxisX();

    // --- K matrix
    intrinsic.GetKPtr()[ 1][ 0 ] = 0;
    intrinsic.GetKPtr()[ 1][ 1 ] = intrinsic.GetFocalL() / intrinsic.GetPixelS();
    intrinsic.GetKPtr()[ 1][ 2 ] = intrinsic.GetAxisY();

    intrinsic.GetKPtr()[ 2][ 0 ] = 0;
    intrinsic.GetKPtr()[ 2][ 1 ] = 0;
    intrinsic.GetKPtr()[ 2][ 2 ] = static_cast<float64_t>(1);
    // PRQA S 3706 --
    // PRQA S 3222 --

    // --- compute the K inverse
    ret = tsc_math::MatrixInvert( intrinsic.GetKPtr(), 3, intrinsic.GetKInvPtr() ); // PRQA S 3223

    return ret;
}

//-------------------------------------------------------------------------
inline bool_t CameraModel::LoadExtrinsicParamConfig( tscApi::cameraModelConfig_Type const * initCameraModelConfigPtr)
{
    /* Load the extrinsic camera parameters from the intiial configuration */
    extrinsic.PutRllDg(initCameraModelConfigPtr->extrinsicParams.Roll_deg);

    extrinsic.PutFlppd(initCameraModelConfigPtr->extrinsicParams.flipped);

    // if the field flipped in use, this will bring the Roll to range [-90, 90]
    if( extrinsic.GetFlppd() && mecl::math::abs_x<float64_t>( extrinsic.GetRllDg() ) > 90.0F )
    {
        extrinsic.PutRllDgMinus(180.0);
    }

    extrinsic.PutYwDg(initCameraModelConfigPtr->extrinsicParams.Yaw_deg);

    extrinsic.PutPtchDg(initCameraModelConfigPtr->extrinsicParams.Pitch_deg);

    extrinsic.PutXmm(initCameraModelConfigPtr->extrinsicParams.X_mm);

    extrinsic.PutYmm(initCameraModelConfigPtr->extrinsicParams.Y_mm);

    extrinsic.PutZmm(initCameraModelConfigPtr->extrinsicParams.Z_mm);

#ifdef ENABLE_SFM   // PRQA S 1070
    // --- build the translation matrix based on design camera positions
    BuildTranslationMatrix( extrinsic.GetXmm(), extrinsic.GetYmm(), extrinsic.GetZmm(), T );
#endif

    return true;
}

//-------------------------------------------------------------------------
inline bool_t CameraModel::LoadOrientationParamConfig( tscApi::cameraModelConfig_Type const * initCameraModelConfigPtr)
{
    /* Load the Orientation parameters from the initial configuration */
    orientation.PutCmrXOffst(initCameraModelConfigPtr->orientationParams.deltaX);

    orientation.PutCmrYOffst(initCameraModelConfigPtr->orientationParams.deltaY);

    orientation.PutCmrPrRll(initCameraModelConfigPtr->orientationParams.preRoll_deg);

    // --- update R_std for LFC
    {
        float64_t Rg[ 3][ 3 ];
        float64_t Rb[ 3][ 3 ];
        float64_t Ra[ 3][ 3 ];
        float64_t R_Magna[ 3][ 3 ];
        float64_t RpreRoll[ 3][ 3 ];
        float64_t InvRpreRoll[ 3][ 3 ];

        // Standard Coordinate System computation
        BuildXAxisRotationMatrix( extrinsic.GetPtchDg(), Ra );
        BuildYAxisRotationMatrix( extrinsic.GetYwDg(), Rb );
        BuildZAxisRotationMatrix( extrinsic.GetRllDg(), Rg );

        BuildRotationMatrix( Ra, Rb, Rg, R_Magna );

        BuildZAxisRotationMatrix( orientation.GetCmrPrRll(), RpreRoll );
        BuildZAxisRotationMatrix( - orientation.GetCmrPrRll(), InvRpreRoll );

        BuildRotationMatrix( RpreRoll, R_Magna, InvRpreRoll, R_std );
    }
    return true;
}

}
#endif
#endif
