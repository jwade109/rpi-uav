#ifndef CONTROL_H
#define CONTROL_H

#include <cstdint>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <bitset>
#include <array>

#include <uavcore.h>
#include <freebody.h>
#include <filters.h>
#include <sensors.h>
#include <pid.h>

namespace uav
{

class controller
{
    public:

    controller(state initial, param cfg);

    int step(const uav::raw_data& raw_data);

    state getstate();
    void setstate(state s);
    param getparams();

    private:

    uint64_t num_steps;
    std::chrono::steady_clock::time_point tstart;

    state curr, prev;
    param prm;

    pid_controller xpid, ypid, zpid, hpid, ppid, rpid;
    dronebody simulator;

    std::array<float, 4> pid2motor(std::array<float, 6> pidov, float hover);

    // void gettargets();
    static std::bitset<16> validate(const state& prev, state& curr);
    static std::pair<imu::Vector<3>, imu::Vector<3>> traverse(
        imu::Vector<2> S, double heading, double tilt95, double maxtilt);
};

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

class mass_estimator
{
    public:

    double value;
    const uint8_t freq;
    const double accel_rc, mavg_sample_time, epsilon;

    mass_estimator(uint8_t freq);
    mass_estimator(uint8_t freq, double accel_rc,
                   double mavg_sample_time, double epsilon_detect);

    double step(double Fz, double az);

    private:

    uav::low_pass accel_lpf;
    uav::running_average global;
    uav::moving_average moving;
    bool diverged = false;

    const double dt;
};

} // namespace uav

#endif // CONTROL_H
