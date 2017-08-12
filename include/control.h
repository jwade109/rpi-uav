#ifndef DRONE_H
#define DRONE_H

#include <cstdint>
#include <iostream>
#include <fstream>
#include <chrono>

#include <pid.h>
#include <filters.h>
#include <ardimu.h>
#include <bmp.h>
#include <dtypes.h>

namespace uav
{
    class Control
    {
        public:

        Control(uav::State initial, uav::Param cfg);
        ~Control();

        int align();
        int iterate();
    
        uav::State getstate();
        void setstate(uav::State state);
        uav::Param getparams();

        static bool debug();

        private:

        std::chrono::steady_clock::time_point start;
        
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

#endif // DRONE_H
