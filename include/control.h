#ifndef CONTROL_H
#define CONTROL_H

#include <cstdint>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

#include <pid.h>
#include <filters.h>
#include <ardimu.h>
#include <bmp.h>

namespace uav
{
    enum Freq : uint8_t
    {
        F10Hz = 10,
        F20Hz = 20,
        F25Hz = 25,
        F40Hz = 40,
        F50Hz = 50,
        F100Hz = 100,
        F125Hz = 125,
        F200Hz = 200,
        F250Hz = 250
    };

    typedef struct
    {
        uint64_t t;                 // epoch time in millis

        float z1;                   // alt from arduino imu
        float z2;                   // alt from external bmp085
        float dz;                   // filtered altitude from home point

        float h, p, r;              // heading, pitch, roll
        uint8_t calib;              // calibration status

        uint16_t tz, th, tp, tr;    // targets for 4 degrees of freedom

        float zov, hov, pov, rov;   // respective pid response
        uint8_t motors[4];
    }
    State;

    typedef struct
    {
        Freq freq;              // frequency of updates in hz
        double z1h;             // home point altitude from imu
        double z2h;             // home point altitude from bmp085

        double zpidg[4];        // pid gains for altitude
        double hpidg[4];        // ' ' for yaw
        double ppidg[4];        // ' ' for pitch
        double rpidg[4];        // ' ' for roll
        double gz_lpf;          // gain for alt low-pass filter
        double gz_wam;          // weighted average gain towards alta
        uint16_t maxmrate;      // max motor thrust rate of change in hz
        double mg;              // weight of vehicle as ratio of max thrust
    }
    Param;

    const std::string pheader("FREQ Z1H Z2H "
                        "ZPID[0] [1] [2] [3] HPID[0] [1] [2] [3] "
                        "PPID[0] [1] [2] [3] RPID[0] [1] [2] [3] "
                        "GZLPF GZWAM MAXM MG");

    const std::string sheader("Time    "
                              "z1h     "
                              "z2h         "
                              "dz          "
                              "hdg    "
                              "pitch  "
                              "roll   "
                              "cal "
                              "tz   "
                              "th   "
                              "tp   "
                              "tr   "
                              "zov         "
                              "hov         "
                              "pov         "
                              "rov         "
                              "m1  "
                              "m2  "
                              "m3  "
                              "m4");

    int tobuffer(Param& prm, char* buffer);

    int tobuffer(State& it, char* buffer);

    int frombuffer(Param& prm, char* buffer);

    int frombuffer(State& it, char* buffer);

    std::string to_string(Param prm);

    std::string to_string(State it);
    
    class Control
    {
        public:

        Control(uav::State initial, uav::Param cfg);
        ~Control();

        int align();
        int iterate(bool block = false);
    
        uav::State getstate();
        void setstate(uav::State state);
        uav::Param getparams();

        static bool debug();

        private:

        unsigned long long iters;
        std::chrono::steady_clock::time_point tstart;

        uav::Arduino imu;
        uav::BMP085 bmp;
        
        uav::State curr, prev;
        uav::Param prm;

        PID zpid, hpid, ppid, rpid;
        LowPassFilter zlpf;
        RateLimiter mr1, mr2, mr3, mr4;

        void gettargets(uav::State& state);
    };
}

#endif // CONTROL_H