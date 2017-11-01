#include "controller.h"

#include <uav/math>
#include <uav/logging>

namespace uav
{

controller::controller(param cfg):
    num_steps(0),
    curr{0}, prev{0}, prm(cfg),
    hdg_acc(2*M_PI), roll_acc(2*M_PI),
    alt_filter(cfg.freq),
    pos_filter(cfg.freq) { }

int controller::step(const raw_data& raw)
{
    using namespace std::chrono;

    if (num_steps++ == 0) tstart = std::chrono::steady_clock::now();
    prev = curr;

    auto now = steady_clock::now();
    curr.time[0] = duration_cast<milliseconds>(now - tstart).count();
    curr.time[1] = duration_cast<milliseconds>
        (now.time_since_epoch()).count();

    if ((curr.status != uav::state::null_status) &&
            (curr.time[0] - prev.time[0]) != 1000/prm.freq)
        uav::error << "Timing error (" << prev.time[0]
            << " -> " << curr.time[0] << ")\n";

    double gps_alt = raw.gps.gga.altitude;
    double b1_alt = uav::altitude(raw.ard.pres);
    double b2_alt = uav::altitude(raw.bmp.pressure);

    curr.position = pos_filter(raw.gps.gga.pos);
    curr.position.altitude() = alt_filter.step(gps_alt, b1_alt, b2_alt);

    curr.attitude[0] = hdg_acc(angle::degrees(raw.ard.euler.x()));
    curr.attitude[1] = angle::degrees(raw.ard.euler.y());
    curr.attitude[2] = roll_acc(angle::degrees(raw.ard.euler.z()));

    auto comptime = steady_clock::now() - now;
    curr.time[2] = duration_cast<microseconds>(comptime).count();

    return 0;
}

state controller::getstate()
{
    return curr;
}

void controller::setstate(state s)
{
    curr = s;
}

} // namespace uav
