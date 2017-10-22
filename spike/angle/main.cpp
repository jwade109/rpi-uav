#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

#include <angle.h>
#include <pid.h>

int main()
{
    using namespace uav::angle_literals;
    using namespace std::chrono_literals;

    const uint8_t freq = 50;
    const double dt = 1.0/freq;

    pid_controller pid(freq, 20, 0, 6);

    uav::angle pos, setpoint = 120_deg;
    double vel = 0, accel = 0;
    std::cout << std::fixed << std::setprecision(3)
        << std::setw(10) << "accel"
        << std::setw(10) << "vel"
        << std::setw(10) << "pos"
        << std::setw(10) << "sp" << std::endl;
    
    auto start = std::chrono::steady_clock::now();
    while (1)
    {
        accel = pid.seek_linear(pos, setpoint);
        vel += accel * dt;
        pos += vel * dt;

        std::cout << std::setw(10) << accel
            << std::setw(10) << vel
            << std::setw(10) << pos
            << std::setw(10) << setpoint
            << std::setw(10) << pid.d_response
            << std::endl;

        std::this_thread::sleep_for(20ms);

        if (std::chrono::steady_clock::now() - start > 5s)
        {
            setpoint += 55_deg;
            start = std::chrono::steady_clock::now();
        }
    }

    return 0;
}
