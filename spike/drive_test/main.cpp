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

int main(int argc, char** argv)
{
    uav::sensor_hub sensors;
    if (sensors.begin()) return 1;

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // M: measurements. position and velocity.
    // N: states. position and velocity.
    // U: control vector. only acceleration.
    const size_t M = 2, N = 2, U = 1, freq = 50;
    const double dt = 1.0/freq;
    kalman<M, N, U, double> kfx, kfy;

    kfx.R *= 15; // high gps measurement error
    kfx.Q *= 0.2;  // relatively lower process noise
    kfx.P *= 1;  // high initial certainty
    kfx.H << 1, 0, 0, 1; // observations map directly to state

    kfx.A << 1, dt, 0, 1; // state transitions w/ kinematics
    kfx.B << 0.5*dt*dt, dt; // acceleration to pos, vel

    kfy.R *= 15; // high gps measurement error
    kfy.Q *= 0.2;  // relatively lower process noise
    kfy.P *= 1;  // high initial certainty
    kfy.H << 1, 0, 0, 1; // observations map directly to state

    kfy.A << 1, dt, 0, 1; // state transitions w/ kinematics
    kfy.B << 0.5*dt*dt, dt; // acceleration to pos, vel

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

        auto p_rel = raw.gps.rmc.pos - home;

        body.euler(Eigen::Vector3d(
            raw.ard.euler.x(), raw.ard.euler.y(), raw.ard.euler.z())*(M_PI/180));
        body.ba(Eigen::Vector3d(
            raw.ard.acc.x(), raw.ard.acc.y(), raw.ard.acc.z()));

        decltype(kfx)::measure zx; zx << p_rel(0), vx;
        decltype(kfx)::control ux; ux << body.a()(0);
        decltype(kfy)::measure zy; zy << p_rel(1), vy;
        decltype(kfy)::control uy; uy << body.a()(1);

        auto x_hat = kfx.predict(ux), y_hat = kfy.predict(uy);
        if (sec != raw.gps.rmc.utc.second)
        {
            sec = raw.gps.rmc.utc.second;
            x_hat = kfx.update(zx);
            y_hat = kfy.update(zy);
        }

        uav::coordinate filtered = home +
            Eigen::Vector3d(x_hat(0), y_hat(0), p_rel(2));

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
                  << std::setw(50) << filtered // "pos_kalman"
                  << std::setw(15) << x_hat(1) // "vE_kalman"
                  << std::setw(15) << y_hat(1); // "vN_kalman"

        if (argc > 1) std::cout << "    \r" << std::flush;
        else std::cout << std::endl;

        if (argc == 1) std::cerr << dec_seconds.count()
            << "      \r" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000/freq));
    }

    return 0;
}
