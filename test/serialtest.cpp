#include <stdio.h>
#include <ardimu.h>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    using namespace std::chrono;

    const int rate = 100; // hz

    uav::Arduino imu;
    int start = imu.begin();
    if (start != 0)
    {
        fprintf(stderr, "Error initializing IMU: %d\n", start);
        return 1;
    }
    std::this_thread::sleep_for(seconds(1));
    for (;;)
    {
        uav::Message m = imu.get();
        printf("%.02f\t%.02f\t%.02f\t%.02f\n",
            m.heading, m.pitch, m.roll, m.alt);
        std::this_thread::sleep_for(milliseconds(1000/rate));
    }
    return 0;
}
