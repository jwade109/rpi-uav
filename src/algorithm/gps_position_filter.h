#ifndef GPS_POS_FILTER_H
#define GPS_POS_FILTER_H

#include <uav/math>
#include <uav/filter>

namespace uav
{

class gps_position_filter
{
    public:

    coordinate value;
    const uint8_t freq;
    const double rc, dt;

    gps_position_filter(uint8_t frequency);
    gps_position_filter(uint8_t frequency,
                        double rc);

    coordinate operator () (coordinate pos);

    private:

    bool first;
    coordinate home;
    low_pass lpfx, lpfy;
};

} // namespace uav

#endif // GPS_POS_FILTER_H
