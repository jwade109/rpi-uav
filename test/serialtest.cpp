#include <iostream>
#include <chrono>
#include <thread>

#include <ardimu.h>

int main(int argc, char** argv)
{
    using namespace std::chrono;

    const int rate = 100; // hz

    uav::Arduino imu;
    int init = imu.begin();
    if (init != 0)
    {
        std::cerr << "Error initializing IMU: " << init << std::endl;
        return 1;
    }
    std::this_thread::sleep_for(seconds(1));

    auto start = steady_clock::now();
    auto runtime = milliseconds(0);
    auto dt = milliseconds(1000/rate);

    while (runtime < minutes(5))
    {
        auto now = duration_cast<milliseconds>(
                system_clock::now().time_since_epoch());
        uav::Message m = imu.get();
        std::cout << now.count() << "\t" << m.heading << "\t"
                  << m.pitch << "\t" << m.roll << "\t" << m.alt
                  << std::endl;

        runtime+=dt;
        auto ptr = steady_clock::now();
        while (ptr < start + runtime)
            ptr = steady_clock::now();
    }
    return 0;
}
