#ifndef __BMP280_H__
#define __BMP280_H__

#include <wiringPi.h>
#include <Adafruit_Sensor.h>

#define BMP280_ADDRESS                (0x77)
#define BMP280_CHIPID                 (0x58)

typedef uint8_t byte;

enum
{
    BMP280_REGISTER_DIG_T1              = 0x88,
    BMP280_REGISTER_DIG_T2              = 0x8A,
    BMP280_REGISTER_DIG_T3              = 0x8C,

    BMP280_REGISTER_DIG_P1              = 0x8E,
    BMP280_REGISTER_DIG_P2              = 0x90,
    BMP280_REGISTER_DIG_P3              = 0x92,
    BMP280_REGISTER_DIG_P4              = 0x94,
    BMP280_REGISTER_DIG_P5              = 0x96,
    BMP280_REGISTER_DIG_P6              = 0x98,
    BMP280_REGISTER_DIG_P7              = 0x9A,
    BMP280_REGISTER_DIG_P8              = 0x9C,
    BMP280_REGISTER_DIG_P9              = 0x9E,

    BMP280_REGISTER_CHIPID             = 0xD0,
    BMP280_REGISTER_VERSION            = 0xD1,
    BMP280_REGISTER_SOFTRESET          = 0xE0,

    BMP280_REGISTER_CAL26              = 0xE1,  // R calibration stored in 0xE1-0xF0

    BMP280_REGISTER_CONTROL            = 0xF4,
    BMP280_REGISTER_CONFIG             = 0xF5,
    BMP280_REGISTER_PRESSUREDATA       = 0xF7,
    BMP280_REGISTER_TEMPDATA           = 0xFA,
};

typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
}
bmp280_calib_data;

class BMP280
{
    public:

        BMP280();
        bool begin();
        float readTemperature(void);
        float readPressure(void);
        float readAltitude(float seaLevelhPa = 1013.25);

    private:

        void readCoefficients(void);
        void write8(byte reg, byte value);
        uint8_t read8(byte reg);
        uint16_t read16(byte reg);
        uint32_t read24(byte reg);
        int16_t readS16(byte reg);
        uint16_t read16_LE(byte reg);
        int16_t readS16_LE(byte reg);

        int32_t t_fine;
        uint8_t fd;

        bmp280_calib_data bmp280_calib;

};

#endif
