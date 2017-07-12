#ifndef MESSAGE_H
#define MESSAGE_H

#include <inttypes.h>

typedef struct
{
    uint64_t millis;
    double heading;
    double pitch;
    double roll;
    uint8_t calib;
    double alt;
}
message_t;

#endif
