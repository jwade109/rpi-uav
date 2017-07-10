#include "I2C.h"
#include <stdbool.h>
#include <wiringPiI2C.h>
#include <stdint.h>
#include <string.h>

I2C::I2C()
{
    i2c_addr = 0;
    fd = -1;
}

I2C::I2C(uint8_t addr)
{
    i2c_addr = addr;
    fd = wiringPiI2CSetup(addr);
}

I2C::~I2C() { }

bool I2C::ready()
{
    return fd > 0;
}

uint8_t I2C::addr()
{
    return i2c_addr;
}

void I2C::readLen(uint8_t reg, uint8_t* buf, uint8_t len)
{
    memset(buf, 0, len);
    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = read8(reg + i);
    }
}

uint8_t I2C::read8(uint8_t reg)
{
    return wiringPiI2CReadReg8(fd, reg);
}

uint16_t I2C::read16_LE(uint8_t reg)
{
    return wiringPiI2CReadReg16(fd, reg);
}

uint16_t I2C::read16_BE(uint8_t reg)
{
    // wiringPi read is little-endian
    uint16_t val = wiringPiI2CReadReg16(fd, reg);
    return (val >> 8) | (val << 8);
}

uint32_t I2C::read24_BE(uint8_t reg)
{
    uint32_t value = read16_BE(reg);
    value <<= 8;
    return value | wiringPiI2CReadReg8(fd, reg);
}

bool I2C::write8(uint8_t reg, uint8_t data)
{
    wiringPiI2CWriteReg8(fd, reg, data);
}
