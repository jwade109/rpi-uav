#ifndef I2C_H
#define I2C_H

#include <stdint.h>

class i2cdev
{
    public:

    i2cdev();
    i2cdev(uint8_t addr);
    ~i2cdev();

    bool open(uint8_t addr);
    bool ready();
    uint8_t addr();

    void readlen(uint8_t reg, uint8_t* buf, uint8_t len);
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

#endif // I2C_H
