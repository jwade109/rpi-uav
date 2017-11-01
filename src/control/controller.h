#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <uav/hardware>
#include <uav/algorithm>

namespace uav
{

struct param
{
    enum : uint8_t
    {
        f1hz = 1, f10hz = 10, f20hz = 20, f25hz = 25,
        f40hz = 40, f50hz = 50, f100hz = 100, f125hz = 125,
        f200hz = 200, f250hz = 250,

        fdefault = f100hz
    };

    uint8_t freq;
    double mass;
    std::array<double, 20> pid_gains;
};

struct state
{
    enum : uint8_t
    {
        null_status, align, no_vel, pos_seek,
        pos_hold, high_tilt, upside_down
    };

    std::array<uint64_t, 3> time;
    coordinate position;
    std::array<angle, 3> attitude;
    std::array<angle, 4> targets;
    std::array<double, 4> motors;
    uint16_t error;
    uint8_t status;
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
