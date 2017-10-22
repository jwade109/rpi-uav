#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <random>

#include <angle.h>
#include <pid.h>

int main()
{
    using namespace uav::angle_literals;
    using namespace std::chrono_literals;

    const uint8_t freq = 50;
    const double dt = 1.0/freq;

    std::default_random_engine gen;
    std::uniform_real_distribution<double> gauss(-1, 1);

    pid_controller pid(freq, 45, 0, 12);
    uav::angle pos, setpoint = 0_deg, vel = 10, accel = 0;

    std::cout << std::left << std::fixed << std::setprecision(3)
        << std::setw(20) << "pos"
        << std::setw(20) << "sp" << std::endl;

    auto start = std::chrono::steady_clock::now();
    while (1)
    {
        accel = pid.seek(pos, setpoint);
        vel += accel * dt;
        pos += vel * dt;

        std::cout << std::setw(20) << pos
            << std::setw(20) << setpoint << std::endl;

        std::this_thread::sleep_for(20ms);

        if (std::chrono::steady_clock::now() - start > 3s)
        {
            auto target = uav::angle(gauss(gen) * 10);
            setpoint = uav::target_azimuth(pos, target);
            start = std::chrono::steady_clock::now();
        }
    }

    return 0;
}
