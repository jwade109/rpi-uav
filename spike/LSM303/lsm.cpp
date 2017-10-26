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

#include <iostream>

#include <lsm.h>

const uint8_t LSM303_ID = 0x4d;
const double lsm303Accel_MG_LSB = 0.001F; // 1, 2, 4 or 12 mg per lsb
const uint8_t LSM303_ADDRESS_ACCEL = 0x19;

enum lsm303AccelRegisters : uint8_t
{
    LSM303_REGISTER_ACCEL_CTRL_REG1_A         = 0x20,
    LSM303_REGISTER_ACCEL_CTRL_REG2_A         = 0x21,
    LSM303_REGISTER_ACCEL_CTRL_REG3_A         = 0x22,
    LSM303_REGISTER_ACCEL_CTRL_REG4_A         = 0x23,
    LSM303_REGISTER_ACCEL_CTRL_REG5_A         = 0x24,
    LSM303_REGISTER_ACCEL_CTRL_REG6_A         = 0x25,
    LSM303_REGISTER_ACCEL_REFERENCE_A         = 0x26,
    LSM303_REGISTER_ACCEL_STATUS_REG_A        = 0x27,
    LSM303_REGISTER_ACCEL_OUT_X_L_A           = 0x28,
    LSM303_REGISTER_ACCEL_OUT_X_H_A           = 0x29,
    LSM303_REGISTER_ACCEL_OUT_Y_L_A           = 0x2A,
    LSM303_REGISTER_ACCEL_OUT_Y_H_A           = 0x2B,
    LSM303_REGISTER_ACCEL_OUT_Z_L_A           = 0x2C,
    LSM303_REGISTER_ACCEL_OUT_Z_H_A           = 0x2D,
    LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A     = 0x2E,
    LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A      = 0x2F,
    LSM303_REGISTER_ACCEL_INT1_CFG_A          = 0x30,
    LSM303_REGISTER_ACCEL_INT1_SOURCE_A       = 0x31,
    LSM303_REGISTER_ACCEL_INT1_THS_A          = 0x32,
    LSM303_REGISTER_ACCEL_INT1_DURATION_A     = 0x33,
    LSM303_REGISTER_ACCEL_INT2_CFG_A          = 0x34,
    LSM303_REGISTER_ACCEL_INT2_SOURCE_A       = 0x35,
    LSM303_REGISTER_ACCEL_INT2_THS_A          = 0x36,
    LSM303_REGISTER_ACCEL_INT2_DURATION_A     = 0x37,
    LSM303_REGISTER_ACCEL_CLICK_CFG_A         = 0x38,
    LSM303_REGISTER_ACCEL_CLICK_SRC_A         = 0x39,
    LSM303_REGISTER_ACCEL_CLICK_THS_A         = 0x3A,
    LSM303_REGISTER_ACCEL_TIME_LIMIT_A        = 0x3B,
    LSM303_REGISTER_ACCEL_TIME_LATENCY_A      = 0x3C,
    LSM303_REGISTER_ACCEL_TIME_WINDOW_A       = 0x3D
};

uav::lsm303::lsm303() : cont(false) { }

uav::lsm303::~lsm303()
{
    cont = false;
    if (reader.joinable()) reader.join();
}

int uav::lsm303::begin()
{
    if (cont) return 0;

    if (!i2c.open(LSM303_ADDRESS_ACCEL))
    {
        std::cerr << "lsm303: accelerometer i2c failed" << std::endl;
        return 1;
    }
    
    i2c.write8(LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);
    uint8_t whoami = i2c.read8(LSM303_REGISTER_ACCEL_CTRL_REG1_A);
    
    if (whoami != 0x57)
    {
        std::cerr << "lsm303: no sensor at i2c addr: 0x"
            << std::hex << LSM303_ADDRESS_ACCEL << std::endl;
        return 3;
    }

    cont = true;
    reader = std::thread([this]() { while (cont) this->update(); });
    
    return 0;
}

void uav::lsm303::update()
{
    uint8_t xlo = i2c.read8(LSM303_REGISTER_ACCEL_OUT_X_L_A);
    uint8_t xhi = i2c.read8(LSM303_REGISTER_ACCEL_OUT_X_H_A);
    uint8_t ylo = i2c.read8(LSM303_REGISTER_ACCEL_OUT_Y_L_A);
    uint8_t yhi = i2c.read8(LSM303_REGISTER_ACCEL_OUT_Y_H_A);
    uint8_t zlo = i2c.read8(LSM303_REGISTER_ACCEL_OUT_Z_L_A);
    uint8_t zhi = i2c.read8(LSM303_REGISTER_ACCEL_OUT_Z_H_A);

    imu::Vector<3> accel_raw;
    accel_raw.x() = (int16_t) (xlo | (xhi << 8)) >> 4;
    accel_raw.y() = (int16_t) (ylo | (yhi << 8)) >> 4;
    accel_raw.z() = (int16_t) (zlo | (zhi << 8)) >> 4;

    accel.x() = (double) accel_raw.x() * lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
    accel.y() = (double) accel_raw.y() * lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
    accel.z() = (double) accel_raw.z() * lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
}

imu::Vector<3> uav::lsm303::get()
{
    update();
    return accel;
}
