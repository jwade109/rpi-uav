#ifndef MESSAGE_H
#define MESSAGE_H

#include <inttypes.h>

typedef struct
{
    uint64_t millis;
    float heading;
    float pitch;
    float roll;
    uint8_t calib;
    float alt;
}
Message;

#endif
