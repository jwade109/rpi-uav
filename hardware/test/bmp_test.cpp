#include <iostream>
#include <chrono>
#include <thread>

#include <bmp.h>

int main()
{
    uav::bmp085 bmp;
    int status = bmp.begin();
    if (status)
    {
        printf("BMP init error: %d\n", status);
        return 1;
    }

    std::cout << "  temp\tpres\talt" << std::endl;

    while (1)
    {
        std::cout << "  " << bmp.getTemperature() << "\t"
            << bmp.getPressure() << "\t"
            << bmp.getAltitude() << "\r" << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}
