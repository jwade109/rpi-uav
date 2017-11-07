#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>
#include <vector>

#include <uav/math>
#include <uav/algorithm>
#include <uav/hardware>

#include "kalman.h"

int main()
{
    uav::sensor_hub sensors;
    if (sensors.begin()) return 1;

    // M: measurements. position and velocity.
    // N: states. position and velocity.
    // U: control vector. only acceleration.
    const size_t M = 2, N = 2, U = 1, freq = 50;
    const double dt = 1.0/freq;
    kalman<M, N, U, double> kfx;

    kfx.R *= 15; // high gps measurement error
    kfx.Q *= 1;  // relatively lower process noise
    kfx.P *= 1;  // high initial certainty
    kfx.H << 1, 0, 0, 1; // observations map directly to state

    kfx.A << 1, dt, 0, 1; // state transitions w/ kinematics
    kfx.B << 0.5*dt*dt, dt; // acceleration to pos, vel

    std::cout << std::left << std::fixed << std::setprecision(3);

    std::cout << std::setw(15) << "timestamp"
              << std::setw(50) << "pos_gnss"
              << std::setw(15) << "track_good"
              << std::setw(15) << "knots"
              << std::setw(15) << "ax"
              << std::setw(15) << "ay"
              << std::setw(15) << "aE"
              << std::setw(15) << "aN"
              << std::setw(15) << "heading"
              << std::setw(15) << "pitch"
              << std::setw(15) << "roll"
              << std::setw(50) << "pos_kalman"
              << std::setw(15) << "vE_kalman"
              << std::setw(15) << "vN_kalman" << std::endl;

    uint64_t counter = 0;
    uint8_t sec = 0;
    auto H = kfx.H;
    uav::freebody body;
    uav::coordinate home = sensors.get().gps.rmc.pos;

    while (1)
    {
        auto raw = sensors.get();
        auto hdg = uav::angle::degrees(90 - raw.gps.rmc.track_angle);
        double mps = raw.gps.rmc.ground_speed/2;
        double vx = mps*std::cos(hdg), vy = mps*std::sin(hdg);

        Eigen::Vector3d p_rel = raw.gps.rmc.pos - home;

        decltype(kfx)::measure z; z << p_rel(0), vx;
        decltype(kfx)::control u; u << raw.ard.acc.x();

        if (sec != raw.gps.gga.utc.second)
        {
            kfx.H = H;
            sec = raw.gps.rmc.utc.second;
        }
        else
        {
            kfx.H = decltype(kfx.H)::Zero();
        }
        auto x_hat = kfx.step(z, u);

        body.euler(Eigen::Vector3d(
            raw.ard.euler.x(), raw.ard.euler.y(), raw.ard.euler.z())*(M_PI/180));
        body.ba(Eigen::Vector3d(
            raw.ard.acc.x(), raw.ard.acc.y(), raw.ard.acc.z()));

        using fsec = std::chrono::duration<double, std::ratio<1>>;
        auto now = std::chrono::steady_clock::now();
        auto dec_seconds = std::chrono::duration_cast<fsec>
            (now.time_since_epoch());

        std::cout << std::setw(15) << dec_seconds.count() // "timestamp"
                  << std::setw(50) << raw.gps.rmc.pos // "pos_gnss"
                  << std::setw(15) << raw.gps.rmc.track_angle // "track_good"
                  << std::setw(15) << raw.gps.rmc.ground_speed // "knots"
                  << std::setw(15) << raw.ard.acc.x() // "ax"
                  << std::setw(15) << raw.ard.acc.y() // "ay"
                  << std::setw(15) << body.a()(0) // "aE"
                  << std::setw(15) << body.a()(1) // "aN"
                  << std::setw(15) << raw.ard.euler.x() // "heading"
                  << std::setw(15) << raw.ard.euler.y() // "pitch"
                  << std::setw(15) << raw.ard.euler.z() // "roll"
                  << std::setw(50) << 0 // "pos_kalman"
                  << std::setw(15) << 0 // "vE_kalman"
                  << std::setw(15) << 0 // "vN_kalman"
                  << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000/freq));
    }

    return 0;
}
