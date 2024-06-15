#ifndef _hikcameraDataType_H_
#define _hikcameraDataType_H_

#include "MvCameraControl.h" 

typedef struct _CAMERA_INFO_
{
    void* pUser;
    unsigned int nDataSize;
    MV_FRAME_OUT stFrameOut;

    unsigned char* pImageCache;

}CAMERA_INFO;

enum class BaudRateDef {
  BR2400,
  BR4800,
  BR9600,
  BR19200,
  BR38400,
  BR57600,
  BR115200,
  BR230400,
  BR460800,
  BR500000,
  BR576000,
  BR921600,
  BR1152000,
  BR1500000,
  BR2000000,
  BR2500000,
  BR3000000,
  BR3500000,
  BR4000000,
  BRUnkown,
  BaudRateCount
};

const std::array<int, static_cast<int>(BaudRateDef::BaudRateCount)> baudRateValues = {
    2400,    // BR2400
    4800,    // BR4800
    9600,    // BR9600
    19200,   // BR19200
    38400,   // BR38400
    57600,   // BR57600
    115200,  // BR115200
    230400,  // BR230400
    460800,  // BR460800
    500000,  // BR500000
    576000,  // BR576000
    921600,  // BR921600
    1152000, // BR1152000
    1500000, // BR1500000
    2000000, // BR2000000
    2500000, // BR2500000
    3000000, // BR3000000
    3500000, // BR3500000
    4000000, // BR4000000
    -1       // BRUnkown
};

#endif