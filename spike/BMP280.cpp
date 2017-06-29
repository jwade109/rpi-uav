#include <wiringPiI2C.h>
#include <BMP280.h>
#include <math.h>
#include <iostream>

using namespace std;

Adafruit_BMP280::Adafruit_BMP280()
  : _cs(-1), _mosi(-1), _miso(-1), _sck(-1)
{ }

Adafruit_BMP280::Adafruit_BMP280(int8_t cspin)
  : _cs(cspin), _mosi(-1), _miso(-1), _sck(-1)
{ }

Adafruit_BMP280::Adafruit_BMP280(int8_t cspin, int8_t mosipin, int8_t misopin, int8_t sckpin)
  : _cs(cspin), _mosi(mosipin), _miso(misopin), _sck(sckpin)
{ }


bool Adafruit_BMP280::begin()
{
    i2caddr = 0x77;
    fd = wiringPiI2CSetup(i2caddr);
    uint8_t chipid = 0x58;

    if (fd < 0) return false;
    if (read8(BMP280_REGISTER_CHIPID) != chipid) return false;

    readCoefficients();
    write8(BMP280_REGISTER_CONTROL, 0x3F);
    return true;
}

uint8_t Adafruit_BMP280::spixfer(uint8_t x)
{
    return 0;
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C/SPI
*/
/**************************************************************************/
void Adafruit_BMP280::write8(byte reg, byte value)
{
    wiringPiI2CWriteReg8(fd, reg, value);
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C/SPI
*/
/**************************************************************************/
uint8_t Adafruit_BMP280::read8(byte reg)
{
    return wiringPiI2CReadReg8(fd, reg);
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit value over I2C/SPI
*/
/**************************************************************************/
uint16_t Adafruit_BMP280::read16(byte reg)
{
    uint16_t val = wiringPiI2CReadReg16(fd, reg);
    return val; // (val >> 8) | (val << 8);
}

uint16_t Adafruit_BMP280::read16_LE(byte reg)
{
    return read16(reg);
}

/**************************************************************************/
/*!
    @brief  Reads a signed 16 bit value over I2C/SPI
*/
/**************************************************************************/
int16_t Adafruit_BMP280::readS16(byte reg)
{
    return (int16_t) read16(reg);
}

int16_t Adafruit_BMP280::readS16_LE(byte reg)
{
    return (int16_t) read16_LE(reg);
}


/**************************************************************************/
/*!
    @brief  Reads a 24 bit value over I2C/SPI
*/
/**************************************************************************/
uint32_t Adafruit_BMP280::read24(byte reg)
{
    uint32_t value;

    /*
    if (_cs == -1) {
    Wire.beginTransmission((uint8_t)_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2caddr, (byte)3);

    value = Wire.read();
    value <<= 8;
    value |= Wire.read();
    value <<= 8;
    value |= Wire.read();
    */
    value = read8(reg);
    value <<= 8;
    value |= read8(reg + 1);
    value <<= 8;
    value |= read8(reg + 2);

    return value;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void Adafruit_BMP280::readCoefficients(void)
{
    _bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    _bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    _bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

    _bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    _bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    _bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    _bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    _bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    _bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    _bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    _bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    _bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);

    /*
    cout << "dig_T1 = " << _bmp280_calib.dig_T1 << endl;
    cout << "dig_T2 = " << _bmp280_calib.dig_T2 << endl;
    cout << "dig_T3 = " << _bmp280_calib.dig_T3 << endl;
    cout << "dig_P1 = " << _bmp280_calib.dig_P1 << endl;
    cout << "dig_P2 = " << _bmp280_calib.dig_P2 << endl;
    cout << "dig_P3 = " << _bmp280_calib.dig_P3 << endl;
    cout << "dig_P4 = " << _bmp280_calib.dig_P4 << endl;
    cout << "dig_P5 = " << _bmp280_calib.dig_P5 << endl;
    cout << "dig_P6 = " << _bmp280_calib.dig_P6 << endl;
    cout << "dig_P7 = " << _bmp280_calib.dig_P7 << endl;
    cout << "dig_P8 = " << _bmp280_calib.dig_P8 << endl;
    cout << "dig_P9 = " << _bmp280_calib.dig_P9 << endl;
    */
}

float Adafruit_BMP280::readTemperature(void)
{
    int32_t var1, var2;

    int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
    adc_T >>= 4;

    var1  = ((((adc_T>>3) - ((int32_t)_bmp280_calib.dig_T1 <<1))) *
        ((int32_t)_bmp280_calib.dig_T2)) >> 11;

    var2  = (((((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1)) *
        ((adc_T>>4) - ((int32_t)_bmp280_calib.dig_T1))) >> 12) *
        ((int32_t)_bmp280_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    float T  = (t_fine * 5 + 128) >> 8;
    return T/100;
}

float Adafruit_BMP280::readPressure(void)
{
    int64_t var1, var2, p;

    // Must be done first to get the t_fine variable set up
    readTemperature();

    int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
    var2 = var2 + ((var1*(int64_t)_bmp280_calib.dig_P5)<<17);
    var2 = var2 + (((int64_t)_bmp280_calib.dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3)>>8) +
    ((var1 * (int64_t)_bmp280_calib.dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)_bmp280_calib.dig_P1)>>33;

    if (var1 == 0) return 0; // avoid division by zero

    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)_bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7)<<4);
    return (float)p/256;
}

float Adafruit_BMP280::readAltitude(float seaLevelhPa)
{
    float altitude;
    float pressure = readPressure(); // in Si units for Pascal
    pressure /= 100;
    altitude = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));
    return altitude;
}
