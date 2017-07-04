#include "I2C.h"
#include <stdbool.h>
#include <wiringPi.h>

uint8_t i2c::read8(uint8_t fd, uint8_t reg)
{
    return wiringPiI2CReadReg8(fd, reg);
}

uint16_t i2c::read16_LE(uint8_t fd, uint8_t reg)
{
    return wiringPiI2CReadReg16(fd, reg);
}

uint16_t i2c::read16_BE(uint8_t fd, uint8_t reg)
{
    // wiringPi read is little-endian
    uint16_t val = wiringPiI2CReadReg16(fd, reg);
    return (val >> 8) | (val << 8);
}

uint32_t i2c::read24_BE(uint8_t fd, uint8_t reg)
{
    uint32_t value = read16_BE(fd, reg);
    value <<= 8;
    return value | wiringPiI2CReadReg8(fd, reg);
}

bool i2c::write8(uint8_t fd, uint8_t reg, uint8_t data)
{
    wiringPiI2CWriteReg8(fd, reg, data);
}
