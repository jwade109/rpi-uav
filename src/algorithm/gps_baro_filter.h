#ifndef GPS_BARO_FILTER_H
#define GPS_BARO_FILTER_H

#include <uav/filter>

namespace uav
{

// based on the algorithm by zaliva and franchetti:
// http://www.crocodile.org/lord/baroaltitude.pdf

class gps_baro_filter
{
    public:

    double value;

    gps_baro_filter(uint8_t frequency);
    gps_baro_filter(uint8_t frequency,
                    unsigned gps_sample_time,
                    unsigned ard_sample_time,
                    unsigned bmp_sample_time,
                    double ard_rc,
                    double bmp_rc);

    double step(double gps, double ard, double bmp);

    private:

    const unsigned gps_samples, ard_samples, bmp_samples;
    const double ard_rc, bmp_rc, dt;
    double home_alt;

    uav::moving_average u_g, u_b1, u_b2;
    uav::low_pass lpf1, lpf2;
};

} // namespace uav

#endif // GPS_BARO_FILTER_H
