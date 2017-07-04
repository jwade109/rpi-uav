#ifndef I2C_H
#define I2C_H

#include <stdbool.h>
#include <stdint.h>

namespace i2c
{
    uint8_t read8(uint8_t fd, uint8_t reg);

    uint16_t read16_LE(uint8_t fd, uint8_t reg);

    uint16_t read16_BE(uint8_t fd, uint8_t reg);

    uint32_t read24_LE(uint8_t fd, uint8_t reg);

    uint32_t read24_BE(uint8_t fd, uint8_t reg);

    uint32_t read32_LE(uint8_t fd, uint8_t reg);

    uint32_t read32_BE(uint8_t fd, uint8_t reg);

    bool write8(uint8_t fd, uint8_t reg, uint8_t data);
}
