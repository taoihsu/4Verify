#ifndef IDATAPROVIDER_OC_H
#define IDATAPROVIDER_OC_H

#include "mecl.h"
#include "meclcfg.h"
#include "Camera.h"
#include <string>
//#include "OCStructs.h" //not sure if I need a file like this or not
#include "OCData.h"
#include "tscApi.h"
#include "ObjectEntry.h"

// Copied from IDataProviderPD.h
// May need some significant changes to fit with OC

class DataCollectAgent;

namespace oc
{
    class IDataProviderOC
    {
    public:
        explicit IDataProviderOC()
        {
        }

        virtual ~IDataProviderOC()
        {
        }

        //virtual pd::Setup& getSetupConfig() const = 0;
        virtual ocdata::OcData_s* getOutData() const = 0;
        //virtual Camera_ID getCameraID() const = 0;
        virtual bool_t isColorCamera_b() const = 0;
        virtual uint16_t getNumOfChannels_u16() const = 0;

        virtual uint32_t getFrameNum_u32() const = 0;

        virtual uint16_t getFrontVideoWidth_u16() const = 0;
        virtual uint16_t getLeftVideoWidth_u16() const = 0;
        virtual uint16_t getRearVideoWidth_u16() const = 0;
        virtual uint16_t getRightVideoWidth_u16() const = 0;

        virtual uint16_t getFrontVideoHeight_u16() const = 0;
        virtual uint16_t getLeftVideoHeight_u16() const = 0;
        virtual uint16_t getRearVideoHeight_u16() const = 0;
        virtual uint16_t getRightVideoHeight_u16() const = 0;


        //virtual const uint8_t* getInputImage_pu8() const = 0;

        virtual float32_t getYawRateTimeStamp() const = 0;
        virtual float32_t getSpeed_f32() const = 0;
        virtual float32_t getWheelAngle_f32() const = 0;
        virtual bool getvideochanged() const = 0;
        virtual std::string getvideoname() const = 0;
        virtual bool getLoggingEnabled() const = 0;
        virtual unsigned long& getProcessedFrameNum() = 0;

        //virtual intg::CameraSource getObjCamSrc() = 0;
        virtual do_CameraParam& getFrontDoCameraParam() = 0;
        virtual do_CameraParam& getLeftDoCameraParam() = 0;
        virtual do_CameraParam& getRearDoCameraParam() = 0;
        virtual do_CameraParam& getRightDoCameraParam() = 0;

        // Placeholder functions for getting FE image pointers
        virtual const uint8_t* getFrontInputImage() const = 0;
        virtual const uint8_t* getLeftInputImage() const = 0;
        virtual const uint8_t* getRearInputImage() const = 0;
        virtual const uint8_t* getRightInputImage() const = 0;
        virtual const tscApi::enuCameraID getCameraID_e() const = 0;
        virtual unsigned long getOCImpl() const = 0;

    };
} // namespace oc


#endif // !IDATAPROVIDER_OC_H


