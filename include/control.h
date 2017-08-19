#ifndef CONTROL_H
#define CONTROL_H

#include <cstdint>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

#include <uavcore.h>
#include <pid.h>
#include <ardimu.h>
#include <bmp.h>

namespace uav
{
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

        std::chrono::steady_clock::time_point tstart;

        uav::Arduino imu;
        uav::BMP085 bmp;

        uav::State curr, prev;
        uav::Param prm;

        PID zpid, hpid, ppid, rpid;

        void gettargets(uav::State& state);
    };
}

#endif // CONTROL_H
