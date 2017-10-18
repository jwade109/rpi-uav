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
const double lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
const double lsm303Mag_Gauss_LSB_XY = 1100.0F;  // Varies with gain
const double lsm303Mag_Gauss_LSB_Z  = 980.0F;   // Varies with gain

const uint8_t LSM303_ADDRESS_ACCEL = 0x19;
const uint8_t LSM303_ADDRESS_MAG = 0x1e;

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

enum lsm303MagRegisters : uint8_t
{
    LSM303_REGISTER_MAG_CRA_REG_M             = 0x00,
    LSM303_REGISTER_MAG_CRB_REG_M             = 0x01,
    LSM303_REGISTER_MAG_MR_REG_M              = 0x02,
    LSM303_REGISTER_MAG_OUT_X_H_M             = 0x03,
    LSM303_REGISTER_MAG_OUT_X_L_M             = 0x04,
    LSM303_REGISTER_MAG_OUT_Z_H_M             = 0x05,
    LSM303_REGISTER_MAG_OUT_Z_L_M             = 0x06,
    LSM303_REGISTER_MAG_OUT_Y_H_M             = 0x07,
    LSM303_REGISTER_MAG_OUT_Y_L_M             = 0x08,
    LSM303_REGISTER_MAG_SR_REG_Mg             = 0x09,
    LSM303_REGISTER_MAG_IRA_REG_M             = 0x0A,
    LSM303_REGISTER_MAG_IRB_REG_M             = 0x0B,
    LSM303_REGISTER_MAG_IRC_REG_M             = 0x0C,
    LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
    LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
};

uav::lsm303::lsm303() { }

void uav::lsm303::update()
{
    { // accelerometer update block

    // not sure if this line is right
    // accel_i2c.write8(LSM303_ADDRESS_ACCEL, LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
    // accel_i2c.write8(LSM303_REGISTER_ACCEL_OUT_X_L_A, 0x80);

    uint8_t xlo = acc_i2c.read8(LSM303_REGISTER_ACCEL_OUT_X_L_A);
    uint8_t xhi = acc_i2c.read8(LSM303_REGISTER_ACCEL_OUT_X_H_A);
    uint8_t ylo = acc_i2c.read8(LSM303_REGISTER_ACCEL_OUT_Y_L_A);
    uint8_t yhi = acc_i2c.read8(LSM303_REGISTER_ACCEL_OUT_Y_H_A);
    uint8_t zlo = acc_i2c.read8(LSM303_REGISTER_ACCEL_OUT_Z_L_A);
    uint8_t zhi = acc_i2c.read8(LSM303_REGISTER_ACCEL_OUT_Z_H_A);

    imu::Vector<3> accel_raw;
    accel_raw.x() = (int16_t)(xlo | (xhi << 8)) >> 4;
    accel_raw.y() = (int16_t)(ylo | (yhi << 8)) >> 4;
    accel_raw.z() = (int16_t)(zlo | (zhi << 8)) >> 4;

    accel.x() = (float) accel_raw.x() * lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
    accel.y() = (float) accel_raw.y() * lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
    accel.z() = (float) accel_raw.z() * lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
    
    } // accelerometer update block
    
    { // magnetometer update block
    
    // Wire.write(LSM303_REGISTER_MAG_OUT_X_H_M);
    // Wire.endTransmission();
    // Wire.requestFrom((uint8_t)LSM303_ADDRESS_MAG, (uint8_t)6);

    uint8_t xhi = mag_i2c.read8(LSM303_REGISTER_MAG_OUT_X_H_M);
    uint8_t xlo = mag_i2c.read8(LSM303_REGISTER_MAG_OUT_X_L_M);
    uint8_t zhi = mag_i2c.read8(LSM303_REGISTER_MAG_OUT_Z_H_M);
    uint8_t zlo = mag_i2c.read8(LSM303_REGISTER_MAG_OUT_Z_L_M);
    uint8_t yhi = mag_i2c.read8(LSM303_REGISTER_MAG_OUT_Y_H_M);
    uint8_t ylo = mag_i2c.read8(LSM303_REGISTER_MAG_OUT_Y_L_M);

    imu::Vector<3> mag_raw;
    // Shift values to create properly formed integer (low byte first)
    mag_raw.x() = (int16_t)(xlo | ((int16_t)xhi << 8));
    mag_raw.y() = (int16_t)(ylo | ((int16_t)yhi << 8));
    mag_raw.z() = (int16_t)(zlo | ((int16_t)zhi << 8));
    
    // mag.x() = ???
    
    } // magnetometer update block
}

int uav::lsm303::begin()
{
    if (!acc_i2c.open(LSM303_ADDRESS_ACCEL))
    {
        std::cerr << "lsm303: accelerometer i2c failed" << std::endl;
        return 1;
    }
    if (!mag_i2c.open(LSM303_ADDRESS_MAG))
    {
        std::cerr << "lsm303: magnetometer i2c failed" << std::endl;
        return 2;
    }
    
    acc_i2c.write8(LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x57);
    mag_i2c.write8(LSM303_REGISTER_MAG_MR_REG_M, 0x00);

    uint8_t whoami_acc = acc_i2c.read8(LSM303_REGISTER_ACCEL_CTRL_REG1_A);
    uint8_t whoami_mag = mag_i2c.read8(LSM303_REGISTER_MAG_CRA_REG_M);
    
    if (whoami_acc != 0x57)
    {
        std::cerr << "lsm303: no accelerometer at i2c addr: 0x"
            << std::hex << LSM303_ADDRESS_ACCEL << std::endl;
        return 3;
    }
    if (whoami_mag != 0x10)
    {
        std::cerr << "lsm303: no LSM303 magnetometer at i2c addr: 0x"
            << std::hex << LSM303_ADDRESS_MAG << std::endl;
        return 4;
    }
    
    setMagGain(LSM303_MAGGAIN_1_3);

    return 0;
}

void uav::lsm303::setMagRate(lsm303MagRate rate)
{
    uint8_t reg_m = (rate & 0x07) << 2;
    mag_i2c.write8(LSM303_REGISTER_MAG_CRA_REG_M, reg_m);
}

void uav::lsm303::setMagGain(lsm303MagGain gain)
{
    mag_i2c.write8(LSM303_REGISTER_MAG_CRB_REG_M, gain);
}
