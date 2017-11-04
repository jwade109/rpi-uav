#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>
#include <vector>

#include <uav/math>
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
    if (!sensors.begin()) return 1;

    // M: measurements. position and velocity.
    // N: states. position and velocity.
    // U: control vector. only acceleration.
    const size_t M = 2, N = 2, U = 1, freq = 50;
    const double dt = 1.0/freq;
    kalman<M, N, U, double> kf;

    kf.R *= 10; // high gps measurement error
    kf.Q *= 1;  // relatively lower process noise
    kf.P *= 1;  // high initial certainty
    kf.H << 1, 0, 0, 1; // observations map directly to state

    kf.A << 1, dt, 0, 1; // state transitions w/ kinematics
    kf.B << 0.5*dt*dt, dt; // acceleration to pos, vel

    std::cout << kf;

    auto home_point = sensors.get().gps.gga.pos;

    while (1)
    {
        auto raw = sensors.get();
        auto acc = raw.ard.acc;
        auto p_rel = displacement(home_point, raw.gps.gga.pos);
        std::cout << p_rel << " " << acc << std::endl;
    }

    return 0;
}
