#ifndef BMP_H
#define BMP_H

#include <adasensor.h>
#include <i2c.h>
#include <stdlib.h>

#define BMP085_CHIPID (0x55)

enum
{
    BMP085_REGISTER_CAL_AC1            = 0xAA,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC2            = 0xAC,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC3            = 0xAE,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC4            = 0xB0,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC5            = 0xB2,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_AC6            = 0xB4,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_B1             = 0xB6,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_B2             = 0xB8,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_MB             = 0xBA,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_MC             = 0xBC,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CAL_MD             = 0xBE,  // R   Calibration data (16 bits)
    BMP085_REGISTER_CHIPID             = 0xD0,
    BMP085_REGISTER_VERSION            = 0xD1,
    BMP085_REGISTER_SOFTRESET          = 0xE0,
    BMP085_REGISTER_CONTROL            = 0xF4,
    BMP085_REGISTER_TEMPDATA           = 0xF6,
    BMP085_REGISTER_PRESSUREDATA       = 0xF6,
    BMP085_REGISTER_READTEMPCMD        = 0x2E,
    BMP085_REGISTER_READPRESSURECMD    = 0x34
};

typedef enum
{
    ULTRALOWPOWER          = 0,
    STANDARD               = 1,
    HIGHRES                = 2,
    ULTRAHIGHRES           = 3
}
bmp085_mode_t;

typedef struct
{
    int16_t  ac1;
    int16_t  ac2;
    int16_t  ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;
    int16_t  b1;
    int16_t  b2;
    int16_t  mb;
    int16_t  mc;
    int16_t  md;
}
bmp085_calib_data;

class BMP085
{
    public:

    BMP085();
    ~BMP085();

    int begin(uint8_t addr = 0x77, bmp085_mode_t mode = ULTRAHIGHRES);
    float getTemperature(void);
    float getPressure(void);
    float getAltitude(float seaLevelhPa = 1013.25);

    private:

    I2C i2c;
    bmp085_calib_data bmp085_coeffs;
    uint8_t bmp085Mode;
    float* mem;
    int child_pid;

    float updateTemperature();
    float updatePressure();

    int32_t readRawTemperature(void);
    int32_t readRawPressure(void);
    void readCoefficients(void);
    int32_t computeB5(int32_t ut);
};

#endif // BMP_H
