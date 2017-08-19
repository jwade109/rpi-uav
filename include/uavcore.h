#ifndef UAV_CORE_H
#define UAV_CORE_H

#include <fstream>
#include <deque>

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
        uint64_t t;                 // time in millis

        float z1;                   // alt from arduino imu
        float z2;                   // alt from external bmp085
        float dz;                   // filtered altitude from home point

        float h, p, r;              // heading, pitch, roll
        uint8_t calib;              // calibration status

        uint16_t tz, th, tp, tr;    // targets for 4 degrees of freedom

        float zov, hov, pov, rov;   // respective pid response
        uint8_t motors[4];
        uint16_t err;               // bitmask for storing error codes
        // 0 | 1  | 2  | 3 | 4 | 5 | 6  | 7  | 8  | 9  | ...
        // t | z1 | z2 | h | p | r | tz | th | tp | tr | ...
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
        
        double gz_rc;           // RC time constant for alt low-pass filter
        double gz_wam;          // weighted average gain towards z1
        uint16_t maxmrate;      // max motor thrust rate of change in hz
        double mg;              // weight of vehicle as ratio of max thrust
    }
    Param;

    const size_t statefields = 21;  // number of fields in each
    const size_t paramfields = 23;

    const size_t statelen = 63;     // number of bytes of each
    const size_t paramlen = 171;    // respective member
    
    int to_buffer(Param& prm, char buffer[paramlen]);

    int to_buffer(State& it, char buffer[statelen]);

    int from_buffer(Param& prm, char buffer[paramlen]);

    int from_buffer(State& it, char buffer[statelen]);

    std::string pheader();

    std::string sheader(uint64_t mask);

    std::string to_string(Param prm);

    std::string to_string(State it, uint64_t mask);

    namespace log
    {
        extern std::deque<std::string> debug;
        extern std::deque<std::string> warn;
        extern std::deque<std::string> fatal;

        std::string ts(uint64_t ms);
    }
}

#endif // UAV_CORE_H
