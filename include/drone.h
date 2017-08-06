#ifndef DRONE_H
#define DRONE_H

#include <stdint.h>
#include <pid.h>
#include <filebuffer.h>
#include <filters.h>
#include <ardimu.h>
#include <bmp.h>

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
Iter;

typedef struct
{
    uint8_t freq;               // frequency of updates in hz
    float z1h;                  // home point altitude from imu
    float z2h;                  // home point altitude from bmp085

    float zpidg[4];             // pid gains for altitude
    float hpidg[4];             // ' ' for yaw
    float ppidg[4];             // ' ' for pitch
    float rpidg[4];             // ' ' for roll

    float gz_lpf;               // gain for alt low-pass filter
    float gz_wam;               // weighted average gain towards alta
    uint16_t maxmrate;          // max motor thrust rate of change in hz
    float mg;                   // weight of vehicle as percent of max thrust
}
Param;

class Drone
{
    public:

    Drone(Iter initial, Param cfg);
    ~Drone();

    int align();
    int iterate();
    
    Iter getstate();
    void setstate(Iter state);
    Param getparams();

    private:

    Arduino imu;
    BMP085 bmp;
    Iter curr, prev;
    Param prm;
    FileBuffer dat, err;
    PID zpid, hpid, ppid, rpid;
    LowPassFilter zlpf;
    RateLimiter mr1, mr2, mr3, mr4;

    void gettargets(Iter& state);
    void writeparams(Param cfg);
};

#endif // DRONE_H
