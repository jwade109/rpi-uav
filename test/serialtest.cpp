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

    std::cout << std::fixed << std::setprecision(2)
        << "  time\thdg\tpitch\troll\tcal\tpres\tax\tay\taz" << std::endl;
    while (1)
    {
        uav::arduino_data m = imu.get();
        std::cout << "  " << m.millis/1000.0 << "\t"
                  << m.euler.x() << "\t" << m.euler.y() << "\t"
                  << m.euler.z() << "\t" << std::hex << (int) m.calib
                  << std::dec << "\t" << m.pres << "\t"
                  << m.acc.x() << "\t" << m.acc.y() << "\t"
                  << m.acc.z() << "\r" << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}
