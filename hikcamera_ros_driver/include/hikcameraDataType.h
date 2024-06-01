#ifndef _hikcameraDataType_H_
#define _hikcameraDataType_H_

#include "MvCameraControl.h" 

typedef struct _CAMERA_INFO_
{
    void* pUser;
    unsigned int nDataSize;
    MV_FRAME_OUT stImageInfo;

    unsigned char* pImageCache;

}CAMERA_INFO;


struct _GPRMC_TIME_STAMP_
{
    int64_t high;
    int64_t low;
};


#endif