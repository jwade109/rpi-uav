#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <uav/hardware>
#include <uav/algorithm>
#include "state.h"
#include "param.h"

namespace uav
{

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

    freebody simulator;
    gps_baro_filter alt_filter;
    gps_position_filter pos_filter;
};

} // namespace uav

#endif // CONTROLLER_H
