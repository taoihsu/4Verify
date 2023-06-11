#ifndef DATAPROVIDER_OC_H
#define DATAPROVIDER_OC_H

#include "IDataProviderOC.h"


// Copied from DataProviderPD.h
// Needs to be reworked for OC

class CAppCtrlInfo;
class CCANTranslationInfo;

namespace oc
{
    class DataProviderOC : public IDataProviderOC
    {
    public:
        explicit DataProviderOC(
            //pd::Setup& init_config,
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
            do_CameraParam& cameraParamFront,
            do_CameraParam& cameraParamLeft,
            do_CameraParam& cameraParamRear,
            do_CameraParam& cameraParamRight,
            tscApi::enuCameraID CameraID,
            unsigned long hImpl
        );

        virtual ~DataProviderOC();

        //pd::Setup& getSetupConfig() const override;
        // Does it make sense to have OutData ?
        // Nothing is being sent to MCU
        ocdata::OcData_s* getOutData() const override;
        //Camera_ID getCameraID() const override;
        bool_t isColorCamera_b() const override;
        uint16_t getNumOfChannels_u16() const override;

        uint32_t getFrameNum_u32() const override;

        uint16_t getFrontVideoWidth_u16() const override;
        uint16_t getLeftVideoWidth_u16() const override;
        uint16_t getRearVideoWidth_u16() const override;
        uint16_t getRightVideoWidth_u16() const override;

        uint16_t getFrontVideoHeight_u16() const override;
        uint16_t getLeftVideoHeight_u16() const override;
        uint16_t getRearVideoHeight_u16() const override;
        uint16_t getRightVideoHeight_u16() const override;

        //const uint8_t* getInputImage_pu8() const override;

        float32_t getYawRateTimeStamp() const override;
        float32_t getSpeed_f32() const override;
        float32_t getWheelAngle_f32() const override;
        bool getvideochanged() const override;
        std::string getvideoname() const override;
        bool getLoggingEnabled() const override;
        unsigned long& getProcessedFrameNum() override;
        //intg::CameraSource getObjCamSrc() override;

        do_CameraParam& getFrontDoCameraParam() override;
        do_CameraParam& getLeftDoCameraParam() override;
        do_CameraParam& getRearDoCameraParam() override;
        do_CameraParam& getRightDoCameraParam() override;

        const uint8_t* getFrontInputImage() const override;
        const uint8_t* getLeftInputImage() const override;
        const uint8_t* getRearInputImage() const override;
        const uint8_t* getRightInputImage() const override;
        const tscApi::enuCameraID getCameraID_e() const override;
        unsigned long getOCImpl() const override;

    private:

        ocdata::OcData_s* initialisation_data;
        do_CameraParam& m_aFrontCameraInfo;
        do_CameraParam& m_aLeftCameraInfo;
        do_CameraParam& m_aRearCameraInfo;
        do_CameraParam& m_aRightCameraInfo;

        // 640x400 FE Camera view pointers
        const unsigned char* const m_pFrontInputImage;
        const unsigned char* const m_pLeftInputImage;
        const unsigned char* const m_pRearInputImage;
        const unsigned char* const m_pRightInputImage;


        bool Logging_enabled;
        unsigned long& ProcessedFrameNum;

        const CAppCtrlInfo* const m_pAppCtrlInfo;
        const CCANTranslationInfo* const m_pObjCANTranslation;

        const struct CVideoImageInfo::VideoImageProfile* m_pFrontObjVideoImage;
        const struct CVideoImageInfo::VideoImageProfile* m_pLeftObjVideoImage;
        const struct CVideoImageInfo::VideoImageProfile* m_pRearObjVideoImage;
        const struct CVideoImageInfo::VideoImageProfile* m_pRightObjVideoImage;

        unsigned long hOCImpl;
        tscApi::enuCameraID targetCameraID_e;
    };
}


#endif // !DATAPROVIDER_OC_H

