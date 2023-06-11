#ifndef OCSTRUCTS_H
#define OCSTRUCTS_H

#include "VideoImageIntf.h"

typedef std::map<Camera_ID, CVideoImageInfo::VideoImageProfile*>::iterator vidImageProfileItr;

struct CameraProfile
{
    do_CameraParam m_aCamIfo;
    Camera_ID m_camOrientation;

    unsigned char* m_pIncomingImagePtr;
};




#endif // !OCSTRUCTS_H

