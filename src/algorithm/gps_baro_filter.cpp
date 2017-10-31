#include "gps_baro_filter.h"

namespace uav
{

gps_baro_filter::gps_baro_filter(uint8_t f) :
    gps_baro_filter(f, 30, 30, 30, 0.7, 0.7) { }

gps_baro_filter::gps_baro_filter(uint8_t f, unsigned gst,
    unsigned ast, unsigned bst, double arc, double brc) :
    value(0), gps_samples(gst * f), ard_samples(ast * f), bmp_samples(bst * f),
    ard_rc(arc), bmp_rc(brc), dt(1.0/f), home_alt(NAN),
    u_g(gps_samples), u_b1(ard_samples),
    u_b2(bmp_samples), lpf1(ard_rc), lpf2(bmp_rc) { }

double gps_baro_filter::step(double gps, double ard, double bmp)
{
    if (isnan(home_alt)) home_alt = gps;

    u_g.step(gps);
    u_b1.step(ard);
    u_b2.step(bmp);

    double d1_est = u_b1.value - u_g.value;
    double d2_est = u_b2.value - u_g.value;
    double alt1_est = ard - d1_est - home_alt;
    double alt2_est = bmp - d2_est - home_alt;
    double alt1_smooth = lpf1.step(alt1_est, dt);
    double alt2_smooth = lpf2.step(alt2_est, dt);

    return (value = 0.5 * alt1_smooth + 0.5 * alt2_smooth);
}

} // namespace uav
