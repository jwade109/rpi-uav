#include <stdio.h>
#include <ardimu.h>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    using namespace std::chrono;

    const int rate = 100; // hz

    uav::Arduino imu;
    int init = imu.begin();
    if (init != 0)
    {
        fprintf(stderr, "Error initializing IMU: %d\n", init);
        return 1;
    }
    std::this_thread::sleep_for(seconds(1));

    auto start = steady_clock::now();
    auto runtime = milliseconds(0);
    auto dt = milliseconds(1000/rate);

    while (runtime < minutes(5))
    {
        uint64_t now = duration_cast<milliseconds>(
                system_clock::now().time_since_epoch()).count();
        uav::Message m = imu.get();
        printf("%llu\t%.02f\t%.02f\t%.02f\t%.02f\n",
            now, m.heading, m.pitch, m.roll, m.alt);
        
        runtime+=dt;
        std::this_thread::sleep_until(start + runtime);
    }
    return 0;
}
