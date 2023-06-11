#include "stdafx.h"
#include "DataProviderOC.h"


namespace oc
{
    DataProviderOC::DataProviderOC(
        ocdata::OcData_s* init_data,
        const class CAppCtrlInfo* pAppCtrlInfo,
        const struct  CVideoImageInfo::VideoImageProfile* pFrontObjVideoImage,
        const struct  CVideoImageInfo::VideoImageProfile* pLeftObjVideoImage,
        const struct  CVideoImageInfo::VideoImageProfile* pRearObjVideoImage,
        const struct  CVideoImageInfo::VideoImageProfile* pRightObjVideoImage,
        const class CCANTranslationInfo* pObjCANTranslation,
        const unsigned char* const frontInputImage,
        const unsigned char* const leftInputImage,
        const unsigned char* const rearInputImage,
        const unsigned char* const rightInputImage,
        bool Logging,
        unsigned long& ProcessedFrameNum_in,
        do_CameraParam& doFrontCameraParam,
        do_CameraParam& doLeftCameraParam,
        do_CameraParam& doRearCameraParam,
        do_CameraParam& doRightCameraParam,
        tscApi::enuCameraID ocCameraID,
        unsigned long hImpl

    ) : IDataProviderOC(),
        //initialization_configuration(init_config),
        initialisation_data( init_data ),
        m_pAppCtrlInfo( pAppCtrlInfo ),
        m_pFrontObjVideoImage( pFrontObjVideoImage ),
        m_pLeftObjVideoImage( pLeftObjVideoImage ),
        m_pRearObjVideoImage( pRearObjVideoImage ),
        m_pRightObjVideoImage( pRightObjVideoImage ),
        m_pObjCANTranslation( pObjCANTranslation ),
        m_pFrontInputImage( frontInputImage ),
        m_pLeftInputImage( leftInputImage ),
        m_pRearInputImage( rearInputImage ),
        m_pRightInputImage( rightInputImage ),
        Logging_enabled( Logging ),
        ProcessedFrameNum( ProcessedFrameNum_in ),
        m_aFrontCameraInfo( doFrontCameraParam ),
        m_aLeftCameraInfo( doLeftCameraParam ),
        m_aRearCameraInfo( doRearCameraParam ),
        m_aRightCameraInfo( doRightCameraParam ),
        targetCameraID_e( ocCameraID ),
        hOCImpl( hImpl )

    {
    }

    DataProviderOC::~DataProviderOC()
    {
    }

    ocdata::OcData_s* DataProviderOC::getOutData() const
    {
        return initialisation_data;
    }

    /* Camera_ID DataProviderOC::getCameraID() const
     {
         return m_CamID;
     }*/

    bool_t DataProviderOC::isColorCamera_b() const
    {
        return m_pAppCtrlInfo->m_VIDIfo.m_ImgType == IMG_RGB;
    }

    uint16_t DataProviderOC::getNumOfChannels_u16() const
    {
        return isColorCamera_b() ? 3 : 1;
    }

    uint32_t DataProviderOC::getFrameNum_u32() const
    {
        return m_pAppCtrlInfo->m_FrameNumber;
    }

    uint16_t DataProviderOC::getFrontVideoWidth_u16() const
    {
        return ( uint16_t )m_pFrontObjVideoImage->Width;
    }

    uint16_t DataProviderOC::getLeftVideoWidth_u16() const
    {
        return ( uint16_t )m_pLeftObjVideoImage->Width;
    }

    uint16_t DataProviderOC::getRearVideoWidth_u16() const
    {
        return ( uint16_t )m_pRearObjVideoImage->Width;
    }

    uint16_t DataProviderOC::getRightVideoWidth_u16() const
    {
        return ( uint16_t )m_pRightObjVideoImage->Width;
    }

    uint16_t DataProviderOC::getFrontVideoHeight_u16() const
    {
        return ( uint16_t )m_pFrontObjVideoImage->Height;
    }

    uint16_t DataProviderOC::getLeftVideoHeight_u16() const
    {
        return ( uint16_t )m_pLeftObjVideoImage->Height;
    }

    uint16_t DataProviderOC::getRearVideoHeight_u16() const
    {
        return ( uint16_t )m_pRearObjVideoImage->Height;
    }

    uint16_t DataProviderOC::getRightVideoHeight_u16() const
    {
        return ( uint16_t )m_pRightObjVideoImage->Height;
    }

    /*const uint8_t* DataProviderOC::getInputImage_pu8() const
    {
        return m_pInputImage;
    }*/

    float32_t DataProviderOC::getYawRateTimeStamp() const
    {
        return m_pObjCANTranslation->m_YawRateTimeStamp;
    }

    float32_t DataProviderOC::getSpeed_f32() const
    {
        return m_pObjCANTranslation->m_Speed;
    }

    float32_t DataProviderOC::getWheelAngle_f32() const
    {
        return m_pObjCANTranslation->m_WheelAngle;
    }

    bool DataProviderOC::getvideochanged() const
    {
        return m_pAppCtrlInfo->m_VideoChanged;
    };

    std::string DataProviderOC::getvideoname() const
    {
        return m_pAppCtrlInfo->m_VideoName;
    };

    bool DataProviderOC::getLoggingEnabled() const
    {
        return Logging_enabled;
    };

    unsigned long& DataProviderOC::getProcessedFrameNum()
    {
        return ProcessedFrameNum;
    }

    do_CameraParam& DataProviderOC::getFrontDoCameraParam()
    {
        return m_aFrontCameraInfo;
    }

    do_CameraParam& DataProviderOC::getLeftDoCameraParam()
    {
        return m_aLeftCameraInfo;
    }

    do_CameraParam& DataProviderOC::getRearDoCameraParam()
    {
        return m_aRearCameraInfo;
    }

    do_CameraParam& DataProviderOC::getRightDoCameraParam()
    {
        return m_aRightCameraInfo;
    }

    const uint8_t* DataProviderOC::getFrontInputImage() const
    {
        return m_pFrontInputImage;
    }

    const uint8_t* DataProviderOC::getLeftInputImage() const
    {
        return m_pLeftInputImage;
    }

    const uint8_t* DataProviderOC::getRearInputImage() const
    {
        return m_pRearInputImage;
    }

    const uint8_t* DataProviderOC::getRightInputImage() const
    {
        return m_pRightInputImage;
    }

    const tscApi::enuCameraID DataProviderOC::getCameraID_e() const
    {
        return targetCameraID_e;
    }



    unsigned long DataProviderOC::getOCImpl() const
    {
        return hOCImpl;
    }
    /* intg::CameraSource DataProviderOC::getObjCamSrc()
     {
         return m_ObjCamSrc;
     }*/
}