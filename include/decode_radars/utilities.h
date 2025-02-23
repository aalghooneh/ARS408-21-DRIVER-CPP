#ifndef _DECODER_UTILITIES_H
#define _DECODER_UTILITIES_H

#include <iostream>
#include <ros/ros.h>



#include "decode_radars/ClusterList.h"
#include "decode_radars/ClusterRadar.h"
#include "decode_radars/ContiList.h"
#include "decode_radars/ContiRadar.h"
#include "decode_radars/ContiRadarList.h"

enum modes{
    _cluster,
    _object,
};


union Byte
{
    unsigned char byte;


    struct
    {
        bool bit1:1;
        bool bit2:1;
        bool bit3:1;
        bool bit4:1;
        bool bit5:1;
        bool bit6:1;
        bool bit7:1;
        bool bit8:1;
    };
};


// cluster general information
const double TARGET_DIST_RES = 0.2;
const double TARGET_DIST_LONG_MIN = -500;
const double TARGET_DIST_LAT_MIN = -102.3;
const double TARGET_VREL_RES = 0.25;
const double TARGET_VREL_LONG_MIN = -128.0;
const double TARGET_VREL_LAT_MIN = -64.0;
const double TARGET_RCS_RES = 0.5;
const double TARGET_RCS_MIN = -64.0;


// Object general information
const double OBJECT_DIST_RES = 0.2;
const double OBJECT_DIST_LONG_MIN = -500;
const double OBJECT_DIST_LAT_MIN = -204.6;
const double OBJECT_VREL_RES = 0.25;
const double OBJECT_VREL_LONG_MIN = -128.0;
const double OBJECT_VREL_LAT_MIN = -64.0;
const double OBJECT_RCS_RES = 0.5;
const double OBJECT_RCS_MIN = -64.0;



#endif