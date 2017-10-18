#include <iostream>
#include <chrono>
#include <thread>

#include <ardimu.h>

int main()
{
    uav::arduino imu;
    int init = imu.begin();
    if (init != 0)
    {
        std::cerr << "Error initializing IMU: " << init << std::endl;
        return 1;
    }

    std::cout << "  ms\thdg\tpitch\troll\tcal\tpres\t"
        "ax\tay\taz" << std::endl;
    while (1)
    {
        uav::imu_packet m = imu.get();
        std::cout << "  " << m.millis << "\t"
                  << m.heading << "\t" << m.pitch << "\t"
                  << m.roll << "\t" << std::hex << (int) m.calib
                  << std::dec << "\t" << m.pres << "\t"
                  << m.ax << "\t" << m.ay << "\t"
                  << m.az << "\r" << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}
