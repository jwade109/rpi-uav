#include "gps_position_filter.h"

namespace uav
{

gps_position_filter::gps_position_filter(uint8_t freq) :
    gps_position_filter(freq, 3) { }

gps_position_filter::gps_position_filter(uint8_t f, double rc) :
    freq(f), rc(rc), dt(1.0/f), first(true), lpfx(rc), lpfy(rc) { }

coordinate gps_position_filter::operator () (coordinate pos)
{
    if (first) { first = false; home = pos; }
    auto d = pos - home;
    return coordinate(lpfx.step(d.latitude(), dt),
                      lpfy.step(d.longitude(), dt), pos.altitude());
}

} // namespace uav
