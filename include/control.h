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
#include <freebody.h>

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
        pid_vector<2> spid;
        dronebody simulator;

        void gettargets();
    };

    std::pair<imu::Vector<3>, imu::Vector<3>>
    traverse(imu::Vector<2> S, double heading, double tilt95, double maxtilt);
}

#endif // CONTROL_H
