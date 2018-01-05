#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

#include <Eigen/Core>
#include <uav/math>
#include <uav/algorithm>
#include "ins.h"

void print_quat(std::ostream& os, const Eigen::Quaterniond& q)
{
    os << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
}

int main()
{
    std::cout << std::fixed << std::setprecision(3);

    for (int a = 0; a <= 360; a+=30)
    {
        for (int b = -90; b <= 90; b+=10)
        {
            for (int g = -180; g <= 180; g+=15)
            {
                auto heading = uav::angle::degrees(a),
                     pitch = uav::angle::degrees(b),
                     roll = uav::angle::degrees(g);

                auto quat = uav::angle2quat(heading, pitch, roll);
                std::cout << heading << " " << pitch << " "
                          << roll << " ";
                print_quat(std::cout, quat);
                std::cout << std::endl;
            }
        }
    }

    return 0;

    const uint8_t freq = 200;
    // uav::dronebody db;
    uav::ins ins(freq);

    while (ins.tow().count() % (1000/freq) > 0);
    auto start = std::chrono::steady_clock::now();
    auto delay = std::chrono::milliseconds(1000/freq);
    auto runtime = delay * 0;
    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        while (now < start + runtime)
            now = std::chrono::steady_clock::now();

        uint64_t tow = ins.tow().count();
        std::cout << tow / 1000 << "."
                  << std::setw(3) << std::setfill('0')
                  << tow % 1000 << std::endl; // "\r" << std::flush;

        runtime += delay;
    }
}
