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
    class controller
    {
        public:

        controller(state initial, param cfg, bool debug = false);
        ~controller();

        int align();
        int iterate(bool block);
    
        state getstate();
        void setstate(state s);
        param getparams();

        private:

        bool debug;
        std::chrono::steady_clock::time_point tstart;

        arduino imu;
        bmp085 bmp;

        state curr, prev;
        param prm;

        pid_controller zpid, hpid, ppid, rpid;

        void gettargets();
    };
}

#endif // CONTROL_H
