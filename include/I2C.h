#ifndef I2C_H
#define I2C_H

#include <stdbool.h>
#include <stdint.h>

class I2C
{
    public:

        I2C();
        I2C(uint8_t addr);
        ~I2C();

        bool ready();
        uint8_t addr();

        void readLen(uint8_t reg, uint8_t* buf, uint8_t len);
        uint8_t read8(uint8_t reg);
        uint16_t read16_LE(uint8_t reg);
        uint16_t read16_BE(uint8_t reg);
        uint32_t read24_LE(uint8_t reg);
        uint32_t read24_BE(uint8_t reg);

        bool write8(uint8_t reg, uint8_t data);

    private:

        uint8_t i2c_addr;
        int8_t fd;
};

#endif
