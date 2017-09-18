#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <wiringPiI2C.h>
#include <i2c.h>

i2cdev::i2cdev()
{
    i2c_addr = 0;
    fd = -1;
}

i2cdev::i2cdev(uint8_t addr)
{
    i2c_addr = addr;
    fd = wiringPiI2CSetup(addr);
}

i2cdev::~i2cdev() { }

bool i2cdev::open(uint8_t addr)
{
    i2c_addr = addr;
    fd = wiringPiI2CSetup(addr);
    return ready();
}

bool i2cdev::ready()
{
    return fd > 0;
}

uint8_t i2cdev::addr()
{
    return i2c_addr;
}

void i2cdev::readlen(uint8_t reg, uint8_t* buf, uint8_t len)
{
    memset(buf, 0, len);
    for (uint8_t i = 0; i < len; i++)
    {
        buf[i] = read8(reg + i);
    }
}

uint8_t i2cdev::read8(uint8_t reg)
{
    return wiringPiI2CReadReg8(fd, reg);
}

uint16_t i2cdev::read16_LE(uint8_t reg)
{
    return wiringPiI2CReadReg16(fd, reg);
}

uint16_t i2cdev::read16_BE(uint8_t reg)
{
    // wiringPi read is little-endian
    uint16_t val = wiringPiI2CReadReg16(fd, reg);
    return (val >> 8) | (val << 8);
}

uint32_t i2cdev::read24_BE(uint8_t reg)
{
    uint32_t value = read16_BE(reg);
    value <<= 8;
    return value | wiringPiI2CReadReg8(fd, reg);
}

bool i2cdev::write8(uint8_t reg, uint8_t data)
{
    return wiringPiI2CWriteReg8(fd, reg, data);
}
