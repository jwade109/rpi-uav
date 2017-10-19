/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303DLHC Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

// modified for Raspberry Pi by Wade Foster on 10/18/2017

#ifndef LSM_H
#define LSM_H

#include <thread>
#include <adasensor.h>
#include <i2c.h>
#include <vector.h>

namespace uav
{

struct lsm303_data
{
    imu::Vector<3> acc;
};

class lsm303
{
    public:

    lsm303();
    ~lsm303();
    int begin();
    void update();
    imu::Vector<3> get();

    imu::Vector<3> accel;

    private:

    bool cont;
    i2cdev i2c;
    std::thread reader;
};

} // namespace uav

#endif // LSM_H
