#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

#include <uav/math>
#include <uav/hardware>

int main()
{
    uav::sensor_hub sensors;
    if (sensors.begin() > 0) return 1;

    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << std::fixed << std::setprecision(2);

    uav::freebody fb;
    while (1)
    {
        auto data = sensors.get();
        auto accel = data.ard.acc;
        auto euler = data.ard.euler;
        Eigen::Vector3d a(accel.x(), accel.y(), accel.z());
        Eigen::Vector3d e(euler.x(), euler.y(), euler.z());
        e *= (M_PI/180);
        fb.euler(e);
        fb.ba(a);

        for (int i = 0; i < 3; i++)
            std::cout << std::setw(10) << fb.s()(i);
        for (int i = 0; i < 3; i++)
            std::cout << std::setw(10) << fb.euler()(i) * 180/M_PI;
        std::cout << "\r" << std::flush;

        fb.stepfor(50*1000, 1000);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
