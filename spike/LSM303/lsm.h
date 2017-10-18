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

enum lsm303MagGain : uint8_t
{
    LSM303_MAGGAIN_1_3              = 0x20,  // +/- 1.3
    LSM303_MAGGAIN_1_9              = 0x40,  // +/- 1.9
    LSM303_MAGGAIN_2_5              = 0x60,  // +/- 2.5
    LSM303_MAGGAIN_4_0              = 0x80,  // +/- 4.0
    LSM303_MAGGAIN_4_7              = 0xA0,  // +/- 4.7
    LSM303_MAGGAIN_5_6              = 0xC0,  // +/- 5.6
    LSM303_MAGGAIN_8_1              = 0xE0   // +/- 8.1
};

enum lsm303MagRate : uint8_t
{
    LSM303_MAGRATE_0_7              = 0x00,  // 0.75 Hz
    LSM303_MAGRATE_1_5              = 0x01,  // 1.5 Hz
    LSM303_MAGRATE_3_0              = 0x62,  // 3.0 Hz
    LSM303_MAGRATE_7_5              = 0x03,  // 7.5 Hz
    LSM303_MAGRATE_15               = 0x04,  // 15 Hz
    LSM303_MAGRATE_30               = 0x05,  // 30 Hz
    LSM303_MAGRATE_75               = 0x06,  // 75 Hz
    LSM303_MAGRATE_220              = 0x07   // 200 Hz
};

class lsm303
{
    public:

    lsm303();
    int begin();
    void update();

    void setMagGain(lsm303MagGain gain);
    void setMagRate(lsm303MagRate rate);

    imu::Vector<3> accel, mag;

    private:

    i2cdev acc_i2c, mag_i2c;
    std::thread reader;
};

} // namespace uav

#endif // LSM_H
