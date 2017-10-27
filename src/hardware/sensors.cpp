#include <sensors.h>

int uav::sensor_hub::begin()
{
    int ret = ard.begin() + gps.begin() + bmp.begin();
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
    raw.bmp = bmp.get();
    return raw;
}

