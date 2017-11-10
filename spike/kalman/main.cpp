#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>
#include <vector>

#include <Eigen/Core>

#include <uav/math>
#include <uav/algorithm>
#include <uav/hardware>

#include "kalman.h"

using namespace std::chrono;
using namespace Eigen;

template <int M, int N, typename rep>
std::ostream& operator << (std::ostream& os, const Eigen::Matrix<rep, M, N>& m)
{
    std::stringstream ss;
    ss << std::fixed << std::left << std::setprecision(4);
    for (int i = 0; i < m.rows(); i++)
        for (int j = 0; j < m.cols(); j++)
            ss << std::setw(9) << m(i,j);
    return os << ss.str() << std::endl;
}

template <size_t M, size_t N, size_t U, typename rep>
std::ostream& operator << (std::ostream& os, const kalman<M, N, U, rep>& k)
{
    std::stringstream s;
    s << "x: " << k.x << "z: " << k.z << "u: " << k.u
      << "P: " << k.P << "K: " << k.K << "A: " << k.A
      << "B: " << k.B << "H: " << k.H << "R: " << k.R
      << "Q: " << k.Q;
    return os << s.str();
}

int main()
{
    uav::sensor_hub sensors;
    if (sensors.begin()) return 1;

    // M: measurements. position and velocity.
    // N: states. position and velocity.
    // U: control vector. only acceleration.
    const size_t M = 2, N = 2, U = 1, freq = 50;
    const double dt = 1.0/freq;
    kalman<M, N, U, double> kf;

    kf.R *= 100; // high gps measurement error
    kf.Q *= 1;  // relatively lower process noise
    kf.P *= 1;  // high initial certainty
    kf.H << 1, 0, 0, 1; // observations map directly to state

    kf.A << 1, dt, 0, 1; // state transitions w/ kinematics
    kf.B << 0.5*dt*dt, dt; // acceleration to pos, vel

    std::cout << kf << std::left << std::fixed << std::setprecision(3);

    auto home_point = sensors.get().gps.gga.pos;

    std::cout << std::setw(10) << "gps pos"
              << std::setw(10) << "x filt"
              << std::setw(10) << "gps vel"
              << std::setw(10) << "imu acc"
              << std::setw(10) << "x_hat" << std::endl;

    uav::low_pass xlpf(5);
    uint64_t counter = 0;
    uint8_t sec = 0;
    auto H = kf.H;
    while (1)
    {
        auto raw = sensors.get();
        imu::Vector<3> acc = raw.ard.acc;
        Eigen::Vector3d p_rel = raw.gps.gga.pos - home_point;

        auto hdg = uav::angle::degrees(90 - raw.gps.rmc.track_angle);
        double mps = raw.gps.rmc.ground_speed/2;
        double vx = mps*std::cos(hdg), vy = mps*std::sin(hdg);

        double xfilt = xlpf.step(p_rel(0), dt);

        decltype(kf)::measure z; z << p_rel(0), vx;
        decltype(kf)::control u; u << acc.x();

        if (sec != raw.gps.gga.utc.second)
        {
            kf.H = H;
            sec = raw.gps.gga.utc.second;
        }
        else
        {
            kf.H = decltype(kf.H)::Zero();
        }
        kf.predict(u);
        auto x_hat = kf.update(z);

        std::cout << std::setw(10) << p_rel.x()
                  << std::setw(10) << xfilt
                  // << std::setw(10) << p_rel.y()
                  // << std::setw(10) << p_rel.z()
                  << std::setw(10) << vx
                  << std::setw(10) << acc.x()
                  // << std::setw(10) << acc.y()
                  // << std::setw(10) << acc.z()
                  << std::setw(10) << x_hat(0, 0)
                  << "   \r" << std::flush;

        std::this_thread::sleep_for(milliseconds(1000/freq));
    }

    return 0;
}
