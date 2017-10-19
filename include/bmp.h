#ifndef BMP_H
#define BMP_H

#include <thread>
#include <atomic>
#include <adasensor.h>
#include <i2c.h>
#include <stdlib.h>

namespace uav
{

enum : uint8_t
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

enum bmp085_mode_t : uint8_t
{
    ULTRALOWPOWER          = 0,
    STANDARD               = 1,
    HIGHRES                = 2,
    ULTRAHIGHRES           = 3
};

struct bmp085_calib_data
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
};

struct bmp085_data
{
    double temp;
    double pressure;
};

class bmp085
{
    public:

    const double slp;

    bmp085();
    ~bmp085();

    int begin(uint8_t addr = 0x77, bmp085_mode_t mode = ULTRAHIGHRES);

    float getTemperature();
    float getPressure();
    float getAltitude(); // default slp 1013.25 hPa

    bmp085_data get() const;

    // takes pressure in kPa
    static double altitude(double p, double hp = 101.325);

    private:

    std::atomic<double> temp, press, alt;
    std::atomic<bmp085_data> data;
    std::atomic<bool> cont;
    std::thread reader;

    i2cdev i2c;
    bmp085_calib_data bmp085_coeffs;
    uint8_t bmp085Mode;

    float updateTemperature();
    float updatePressure();

    int32_t readRawTemperature();
    int32_t readRawPressure();
    void readCoefficients();
    int32_t computeB5(int32_t ut);

    void work();
};

} // namespace uav

#endif // BMP_H
