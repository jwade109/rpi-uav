#include "gps_position_filter.h"

namespace uav
{

gps_position_filter::gps_position_filter(uint8_t frequency,
    double kP, double kI, double kD, double minres, double maxres) :
    freq(frequency), kP(kP), kI(kI), kD(kD),
    minres(minres), maxres(maxres) { }

gps_position_filter::gps_position_filter(uint8_t freq) :
    gps_position_filter(freq, 10, 0.01, 10, 0.1, 0.2) { }

coordinate gps_position_filter::operator () (coordinate pos)
{
    if (first) { first = false; home = pos; }
    auto d = pos - home;
    return home + Eigen::Vector3d(
            lpfx.step(d(0), dt), lpfy.step(d(1), dt), pos.altitude());
}

} // namespace uav
