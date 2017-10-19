#include <iostream>
#include <chrono>

#include <vector.h>
#include <ardimu.h>
#include <gps.h>
#include <bmp.h>
#include <lsm.h>

namespace uav
{

struct raw_data
{
    arduino_data ard;
    gps_data gps;
    bmp085_data bmp;
    lsm303_data lsm;
};

class sensor_hub
{
    public:

    int begin();
    raw_data get();

    uav::arduino ard;
    uav::gps gps;
    uav::bmp085 bmp;
    uav::lsm303 lsm;
};

} // namespace uav

int uav::sensor_hub::begin()
{
    int ret = ard.begin() + gps.begin() + 
        bmp.begin() + lsm.begin();
    if (ret > 0)
    {
        std::cerr << "sensor hub: constituent sensor failure"
            << std::endl;
        return 1;
    }
    return 0;
}

uav::raw_data uav::sensor_hub::get()
{
    raw_data raw;
    raw.ard = ard.get();
    raw.gps = gps.get();
    raw.bmp.pressure = bmp.getPressure();
    raw.lsm.acc = lsm.accel;
    return raw;
}

int main()
{
    uav::sensor_hub hub;
    hub.begin();

    while (1)
    {
        auto a = hub.get();
        std::cout << std::setw(40) << a.lsm.acc <<
            std::setw(40) << a.ard.acc << "\r" << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
