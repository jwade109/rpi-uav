#ifndef GPS_POS_FILTER_H
#define GPS_POS_FILTER_H

#include <uav/math>
#include "filters.h"

namespace uav
{

class gps_position_filter
{
    public:

    coordinate value;
    const uint8_t freq;
    const double dt, kP, kI, kD, minres, maxres;

    gps_position_filter(uint8_t frequency, double kP,
        double kI, double kD, double minres, double maxres);

    coordinate operator () (coordinate pos);

    private:

    bool first;
    coordinate home;
    low_pass lpfx, lpfy;
};

} // namespace uav

#endif // GPS_POS_FILTER_H
