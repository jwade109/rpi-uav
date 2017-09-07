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

        Control(state initial, param cfg, bool debug = false);
        ~Control();

        int align();
        int iterate(bool block = false);
    
        state getstate();
        void setstate(state s);
        param getparams();

        private:

        bool debug;
        std::chrono::steady_clock::time_point tstart;

        Arduino imu;
        BMP085 bmp;

        state curr, prev;
        param prm;

        PID zpid, hpid, ppid, rpid;

        void gettargets();
    };
}

#endif // CONTROL_H
