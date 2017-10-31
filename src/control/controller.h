#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <uav/algorithm>

namespace uav
{

struct drone_config
{
    enum : uint8_t
    {
        f1hz = 1, f10hz = 10, f20hz = 20, f25hz = 25,
        f40hz = 40, f50hz = 50, f100hz = 100, f125hz = 125,
        f200hz = 200, f250hz = 250,

        fdefault = f100hz
    };

    uint8_t freq; // frequency of updates in hz
    double mass;
    // pid gains for altitude, heading, pitch, roll
    std::array<double, 20> pid_gains;
};

class controller
{
    public:

    controller(param cfg);

    int step(const uav::raw_data& raw_data);

    state getstate();
    void setstate(state s);
    param getparams();

    private:

    uint64_t num_steps;
    std::chrono::steady_clock::time_point tstart;

    state curr, prev;
    param prm;

    range_accumulator hdg_acc, roll_acc;

    dronebody simulator;
    gps_baro_filter alt_filter;
    gps_position_filter pos_filter;
};

} // namespace uav

#endif // CONTROLLER_H
