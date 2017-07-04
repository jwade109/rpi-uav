#include <wiringPiI2C.h>
#include <BMP280.h>
#include <math.h>
#include <stdint.h>

using namespace std;

BMP280::BMP280()
{
    i2c = I2C(0);
}

bool BMP280::begin(uint8_t addr)
{
    i2c = I2C(addr);
    if (!i2c.ready()) return false;
    if (i2c.read8(BMP280_REGISTER_CHIPID) != BMP280_CHIPID)
        return false;

    readCoefficients();
    i2c.write8(BMP280_REGISTER_CONTROL, 0x3F);
    return true;
}

void BMP280::readCoefficients(void)
{
    bmp280_calib.dig_T1 = i2c.read16_LE(BMP280_REGISTER_DIG_T1);
    bmp280_calib.dig_T2 = (int16_t) i2c.read16_LE(BMP280_REGISTER_DIG_T2);
    bmp280_calib.dig_T3 = (int16_t) i2c.read16_LE(BMP280_REGISTER_DIG_T3);

    bmp280_calib.dig_P1 = i2c.read16_LE(BMP280_REGISTER_DIG_P1);
    bmp280_calib.dig_P2 = (int16_t) i2c.read16_LE(BMP280_REGISTER_DIG_P2);
    bmp280_calib.dig_P3 = (int16_t) i2c.read16_LE(BMP280_REGISTER_DIG_P3);
    bmp280_calib.dig_P4 = (int16_t) i2c.read16_LE(BMP280_REGISTER_DIG_P4);
    bmp280_calib.dig_P5 = (int16_t) i2c.read16_LE(BMP280_REGISTER_DIG_P5);
    bmp280_calib.dig_P6 = (int16_t) i2c.read16_LE(BMP280_REGISTER_DIG_P6);
    bmp280_calib.dig_P7 = (int16_t) i2c.read16_LE(BMP280_REGISTER_DIG_P7);
    bmp280_calib.dig_P8 = (int16_t) i2c.read16_LE(BMP280_REGISTER_DIG_P8);
    bmp280_calib.dig_P9 = (int16_t) i2c.read16_LE(BMP280_REGISTER_DIG_P9);
}

float BMP280::readTemperature(void)
{
    int32_t var1, var2;
    int32_t adc_T = i2c.read24_BE(BMP280_REGISTER_TEMPDATA);
    adc_T >>= 4;
    var1  = ((((adc_T>>3) - ((int32_t) bmp280_calib.dig_T1 <<1))) *
        ((int32_t) bmp280_calib.dig_T2)) >> 11;
    var2  = (((((adc_T>>4) - ((int32_t) bmp280_calib.dig_T1)) *
        ((adc_T>>4) - ((int32_t) bmp280_calib.dig_T1))) >> 12) *
        ((int32_t) bmp280_calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    float T  = (t_fine * 5 + 128) >> 8;
    return T/100;
}

float BMP280::readPressure(void)
{
    int64_t var1, var2, p;
    readTemperature();
    int32_t adc_P = i2c.read24_BE(BMP280_REGISTER_PRESSUREDATA);
    adc_P >>= 4;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t) bmp280_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t) bmp280_calib.dig_P5)<<17);
    var2 = var2 + (((int64_t) bmp280_calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t) bmp280_calib.dig_P3)>>8) +
    ((var1 * (int64_t) bmp280_calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t) bmp280_calib.dig_P1)>>33;
    if (var1 == 0) return 0; // avoid division by zero
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t) bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t) bmp280_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t) bmp280_calib.dig_P7)<<4);
    return (float) p/256;
}

float BMP280::readAltitude(float seaLevelhPa)
{
    float altitude;
    float pressure = readPressure(); // in Si units for Pascal
    pressure /= 100;
    altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
    return altitude;
}
