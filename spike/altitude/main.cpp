#include <iostream>

#include <filters.h>
#include <bmp.h>
#include <ardimu.h>
#include <gps.h>

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

gps_baro_filter::gps_baro_filter(uint8_t f) :
    gps_baro_filter(f, 30, 30, 30, 0.3, 0.3) { }

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

int main(int argc, char** argv)
{
    uav::arduino ard;
    uav::bmp085 bmp;
    uav::gps gps;

    int ret1 = ard.begin() + bmp.begin() + gps.begin();
    if (ret1 > 0)
    {
        std::cerr << "Something went wrong!" << std::endl;
        return 1;
    }
    while (gps.get().gga.num_sats == 0);

    gps_baro_filter gbf(50);

    if (argc == 1) std::cout << "  ";
    std::cout << std::fixed << "gps\t\tarduino\t\tbmp\t\tfused" << std::endl;

    while (1)
    {
        double g = gps.get().gga.altitude;
        double b1 = uav::bmp085::altitude(ard.get().pres/1000);
        double b2 = bmp.getAltitude();
        if (argc == 1) std::cout << "  " << g << "\t" << b1 << "\t"
            << b2 << "\t" << gbf.step(g, b1, b2) << "   \r" << std::flush;
        else std::cout << g << "\t" << b1 << "\t"
            << b2 << "\t" << gbf.step(g, b1, b2) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}
