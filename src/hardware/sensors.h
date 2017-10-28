#ifndef SENSORS_H
#define SENSORS_H

#include "ardimu.h"
#include "gps.h"
#include "bmp.h"

namespace uav
{

struct raw_data
{
    arduino_data ard;
    gps_data gps;
    bmp085_data bmp;
};

class sensor_hub
{
    public:

    int begin();
    raw_data get();

    uav::arduino ard;
    uav::gps gps;
    uav::bmp085 bmp;
};

} // namespace uav

#endif // SENSORS_H

