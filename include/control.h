#ifndef CONTROL_H
#define CONTROL_H

#include <cstdint>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <bitset>

#include <uavcore.h>
#include <freebody.h>
#include <filters.h>
#include <pid.h>

namespace uav
{

class controller
{
    public:

    controller(state initial, param cfg);
    ~controller();

    int begin();
    int iterate(bool block);

    state getstate();
    void setstate(state s);
    param getparams();

    private:

    std::chrono::steady_clock::time_point tstart;

    state curr, prev;
    param prm;

    pid_controller xpid, ypid, zpid, hpid, ppid, rpid;
    dronebody simulator;

    void gettargets();
    static std::bitset<16> validate(const state& prev, state& curr);
    static std::pair<imu::Vector<3>, imu::Vector<3>> traverse(
        imu::Vector<2> S, double heading, double tilt95, double maxtilt);
};

} // namespace uav

#endif // CONTROL_H
