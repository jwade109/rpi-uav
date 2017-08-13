#ifndef DTYPES_H
#define DTYPES_H

#include <inttypes.h>
#include <string>
#include <iostream>
#include <fstream>

namespace uav
{
    typedef struct
    {
        uint64_t t;                 // epoch time in millis

        float z1;                   // alt from arduino imu
        float z2;                   // alt from external bmp085
        float dz;                   // filtered altitude from home point

        float h, p, r;              // heading, pitch, roll
        uint8_t calib;              // calibration status

        float tz, th, tp, tr;       // targets for 4 degrees of freedom

        float zov, hov, pov, rov;   // respective pid response
        uint8_t motors[4];
    }
    State;

    typedef struct
    {
        uint8_t freq;               // frequency of updates in hz
        double z1h;                 // home point altitude from imu
        double z2h;                 // home point altitude from bmp085

        double zpidg[4];            // pid gains for altitude
        double hpidg[4];            // ' ' for yaw
        double ppidg[4];            // ' ' for pitch
        double rpidg[4];            // ' ' for roll

        double gz_lpf;              // gain for alt low-pass filter
        double gz_wam;              // weighted average gain towards alta
        uint16_t maxmrate;          // max motor thrust rate of change in hz
        double mg;                  // weight of vehicle as percent of max thrust
    }
    Param;

    int tobuffer(Param& prm, char* buffer);

    int tobuffer(State& it, char* buffer);

    int frombuffer(Param& prm, char* buffer);

    int frombuffer(State& it, char* buffer);

    std::string tostring(Param prm);

    std::string tostring(State it);
}

#endif // DTYPES_H
