#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>
#include <vector>
#include <bitset>

#include <uav/math>
#include <uav/algorithm>
#include <uav/hardware>

#include "kalman.h"

int main(int argc, char** argv)
{
    uav::sensor_hub sensors;
    if (sensors.begin()) return 1;

    std::this_thread::sleep_for(std::chrono::seconds(2));
    const uint8_t freq = 50;
    /*

    // M: measurements. position and velocity.
    // N: states. position and velocity.
    // U: control vector. only acceleration.
    const size_t M = 2, N = 2, U = 1, freq = 50;
    const double dt = 1.0/freq;
    kalman<M, N, U, double> kfx, kfy;

    kfx.R *= 50; // high gps measurement error
    kfx.Q *= 0.002;  // relatively lower process noise
    kfx.P *= 1;  // high initial certainty
    kfx.H << 1, 0, 0, 1; // observations map directly to state

    kfx.A << 1, dt, 0, 1; // state transitions w/ kinematics
    kfx.B << 0.5*dt*dt, dt; // acceleration to pos, vel

    kfy.R *= 50; // high gps measurement error
    kfy.Q *= 0.002;  // relatively lower process noise
    kfy.P *= 1;  // high initial certainty
    kfy.H << 1, 0, 0, 1; // observations map directly to state

    kfy.A << 1, dt, 0, 1; // state transitions w/ kinematics
    kfy.B << 0.5*dt*dt, dt; // acceleration to pos, vel

    */

    std::cout << std::left << std::fixed << std::setprecision(3);
    std::cerr << std::fixed << std::setprecision(3);
    std::cout << std::setw(12) << "timestamp"
              << std::setw(12) << "latitude"
              << std::setw(12) << "longitude"
              << std::setw(12) << "altitude"
              << std::setw(12) << "track_good"
              << std::setw(12) << "knots"
              << std::setw(12) << "HDOP"
              << std::setw(12) << "ax"
              << std::setw(12) << "ay"
              << std::setw(12) << "az"
              << std::setw(12) << "aE"
              << std::setw(12) << "aN"
              << std::setw(12) << "aU"
              << std::setw(12) << "heading"
              << std::setw(12) << "pitch"
              << std::setw(12) << "roll"
              << std::setw(12) << "calib"
              << std::setw(12) << "alt1"
              << std::setw(12) << "alt2" << std::endl;

    uint64_t counter = 0;
    // uint8_t sec = 0;
    // auto H = kfx.H;
    uav::freebody body;
    // uav::coordinate home = sensors.get().gps.rmc.pos;

    auto start = std::chrono::steady_clock::now(), now = start;
    auto delay = std::chrono::milliseconds(1000/freq),
         runtime = delay * 0;

    while (1)
    {
        while (now < start + runtime)
            now = std::chrono::steady_clock::now();

        auto raw = sensors.get();
        auto hdg = uav::angle::degrees(90 - raw.gps.rmc.track_angle);
        double mps = raw.gps.rmc.ground_speed/2;
        double vx = mps*std::cos(hdg), vy = mps*std::sin(hdg);

        // auto p_rel = raw.gps.rmc.pos - home;

        body.euler(Eigen::Vector3d(
            raw.ard.euler.x(), raw.ard.euler.y(), raw.ard.euler.z())*(M_PI/180));
        body.ba(Eigen::Vector3d(
            raw.ard.acc.x(), raw.ard.acc.y(), raw.ard.acc.z()));

        // decltype(kfx)::measure zx; zx << p_rel(0), vx;
        // decltype(kfx)::control ux; ux << body.a()(0);
        // decltype(kfy)::measure zy; zy << p_rel(1), vy;
        // decltype(kfy)::control uy; uy << body.a()(1);

        // auto x_hat = kfx.predict(ux), y_hat = kfy.predict(uy);
        // if (sec != raw.gps.rmc.utc.second)
        {
            // sec = raw.gps.rmc.utc.second;
            // x_hat = kfx.update(zx);
            // y_hat = kfy.update(zy);
        }

        // uav::coordinate filtered = home +
        //     Eigen::Vector3d(x_hat(0), y_hat(0), p_rel(2));

        using fsec = std::chrono::duration<double, std::ratio<1>>;
        static auto start(std::chrono::steady_clock::now());
        auto now = std::chrono::steady_clock::now();
        auto dec_seconds = std::chrono::duration_cast<fsec>(now - start);

        std::cout << std::setw(12) << dec_seconds.count() // "timestamp"
                  << std::setw(12) << raw.gps.gga.pos.latitude() // "pos_gnss"
                  << std::setw(12) << raw.gps.gga.pos.longitude()
                  << std::setw(12) << raw.gps.gga.pos.altitude()
                  << std::setw(12) << raw.gps.rmc.track_angle // "track_good"
                  << std::setw(12) << raw.gps.rmc.ground_speed // "knots"
                  << std::setw(12) << raw.gps.gga.hdop // "HDOP"
                  << std::setw(12) << raw.ard.acc.x() // "ax"
                  << std::setw(12) << raw.ard.acc.y() // "ay"
                  << std::setw(12) << raw.ard.acc.z() // "az"
                  << std::setw(12) << body.a()(0) // "aE"
                  << std::setw(12) << body.a()(1) // "aN"
                  << std::setw(12) << body.a()(2) // "aN"
                  << std::setw(12) << raw.ard.euler.x() // "heading"
                  << std::setw(12) << raw.ard.euler.y() // "pitch"
                  << std::setw(12) << raw.ard.euler.z() // "roll"
                  << std::setw(12) << std::bitset<8>(raw.ard.calib) // "calib"
                  << std::setw(12) << uav::altitude(raw.ard.pres)
                  << std::setw(12) << uav::altitude(raw.bmp.pressure);
        if (argc > 1) std::cout << "    \r" << std::flush;
        else std::cout << std::endl;

        if (argc == 1) std::cerr << "  " << dec_seconds.count()
            << "          \r" << std::flush;

        runtime += delay;
    }

    return 0;
}
