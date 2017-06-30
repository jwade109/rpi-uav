#include <wiringPiI2C.h>
#include <BMP280.h>
#include <math.h>
#include <iostream>

using namespace std;

BMP280::BMP280() { }

bool BMP280::begin()
{
    fd = wiringPiI2CSetup(BMP280_ADDRESS);
    if (fd < 0) return false;
    if (read8(BMP280_REGISTER_CHIPID) != BMP280_CHIPID)
        return false;

    // 0x3F
    readCoefficients();
    write8(BMP280_REGISTER_CONTROL, 0x3F);
    return true;
}


void BMP280::write8(byte reg, byte value)
{
    wiringPiI2CWriteReg8(fd, reg, value);
}

uint8_t BMP280::read8(byte reg)
{
    return wiringPiI2CReadReg8(fd, reg);
}

uint16_t BMP280::read16(byte reg) // big-endian
{
    // wiringPi read is little-endian
    uint16_t val = wiringPiI2CReadReg16(fd, reg);
    return (val >> 8) | (val << 8);
}

uint16_t BMP280::read16_LE(byte reg) // little-endian
{
    return wiringPiI2CReadReg16(fd, reg);
}

/*
int16_t BMP280::readS16(byte reg) // unused, BE
{
    return (int16_t) read16(reg);
}
*/

int16_t BMP280::readS16_LE(byte reg) // little-endian
{
    return (int16_t) read16_LE(reg);
}

uint32_t BMP280::read24(byte reg) // big-endian
{
    uint32_t value = read16(reg);
    value <<= 8;
    return value | read8(reg);
}

void BMP280::readCoefficients(void)
{
    bmp280_calib.dig_T1 = read16_LE(BMP280_REGISTER_DIG_T1);
    bmp280_calib.dig_T2 = readS16_LE(BMP280_REGISTER_DIG_T2);
    bmp280_calib.dig_T3 = readS16_LE(BMP280_REGISTER_DIG_T3);

    bmp280_calib.dig_P1 = read16_LE(BMP280_REGISTER_DIG_P1);
    bmp280_calib.dig_P2 = readS16_LE(BMP280_REGISTER_DIG_P2);
    bmp280_calib.dig_P3 = readS16_LE(BMP280_REGISTER_DIG_P3);
    bmp280_calib.dig_P4 = readS16_LE(BMP280_REGISTER_DIG_P4);
    bmp280_calib.dig_P5 = readS16_LE(BMP280_REGISTER_DIG_P5);
    bmp280_calib.dig_P6 = readS16_LE(BMP280_REGISTER_DIG_P6);
    bmp280_calib.dig_P7 = readS16_LE(BMP280_REGISTER_DIG_P7);
    bmp280_calib.dig_P8 = readS16_LE(BMP280_REGISTER_DIG_P8);
    bmp280_calib.dig_P9 = readS16_LE(BMP280_REGISTER_DIG_P9);

    /*
    cout << "dig_T1 = " << bmp280_calib.dig_T1 << endl;
    cout << "dig_T2 = " << bmp280_calib.dig_T2 << endl;
    cout << "dig_T3 = " << bmp280_calib.dig_T3 << endl;
    cout << "dig_P1 = " << bmp280_calib.dig_P1 << endl;
    cout << "dig_P2 = " << bmp280_calib.dig_P2 << endl;
    cout << "dig_P3 = " << bmp280_calib.dig_P3 << endl;
    cout << "dig_P4 = " << bmp280_calib.dig_P4 << endl;
    cout << "dig_P5 = " << bmp280_calib.dig_P5 << endl;
    cout << "dig_P6 = " << bmp280_calib.dig_P6 << endl;
    cout << "dig_P7 = " << bmp280_calib.dig_P7 << endl;
    cout << "dig_P8 = " << bmp280_calib.dig_P8 << endl;
    cout << "dig_P9 = " << bmp280_calib.dig_P9 << endl;
    */
}

float BMP280::readTemperature(void)
{
    // cout << "=== READ TEMP ===\n";
    int32_t var1, var2;

    int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA);
    adc_T >>= 4;

    // cout << "UT = " << adc_T << endl;

    var1  = ((((adc_T>>3) - ((int32_t) bmp280_calib.dig_T1 <<1))) *
        ((int32_t) bmp280_calib.dig_T2)) >> 11;

    // cout << "var1 = " << var1 << endl;

    var2  = (((((adc_T>>4) - ((int32_t) bmp280_calib.dig_T1)) *
        ((adc_T>>4) - ((int32_t) bmp280_calib.dig_T1))) >> 12) *
        ((int32_t) bmp280_calib.dig_T3)) >> 14;

    // cout << "var2 = " << var2 << endl;

    t_fine = var1 + var2;

    // cout << "t_fine = " << t_fine << endl;

    float T  = (t_fine * 5 + 128) >> 8;
    // cout << "=/= READ TEMP =/=\n";
    return T/100;
}

float BMP280::readPressure(void)
{
    // cout << "=== READ PRESSURE ===\n";
    int64_t var1, var2, p;

    // Must be done first to get the t_fine variable set up
    readTemperature();

    // cout << "t_fine = " << t_fine << endl;

    int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA);
    adc_P >>= 4;

    // cout << "UP = " << adc_P << endl;

    var1 = ((int64_t)t_fine) - 128000;
    // cout << "var1 = " << var1 << endl;

    var2 = var1 * var1 * (int64_t) bmp280_calib.dig_P6;
    // cout << "var2 = " << var2 << endl;

    var2 = var2 + ((var1*(int64_t) bmp280_calib.dig_P5)<<17);
    // cout << "var2 = " << var2 << endl;

    var2 = var2 + (((int64_t) bmp280_calib.dig_P4)<<35);
    // cout << "var2 = " << var2 << endl;

    var1 = ((var1 * var1 * (int64_t) bmp280_calib.dig_P3)>>8) +
    ((var1 * (int64_t) bmp280_calib.dig_P2)<<12);
    // cout << "var1 = " << var1 << endl;

    var1 = (((((int64_t)1)<<47)+var1))*((int64_t) bmp280_calib.dig_P1)>>33;
    // cout << "var1 = " << var1 << endl;

    if (var1 == 0) return 0; // avoid division by zero

    p = 1048576 - adc_P;
    // cout << "p = " << p << endl;

    p = (((p<<31) - var2)*3125) / var1;
    // cout << "p = " << p << endl;

    var1 = (((int64_t) bmp280_calib.dig_P9) * (p>>13) * (p>>13)) >> 25;
    // cout << "var1 = " << var1 << endl;

    var2 = (((int64_t) bmp280_calib.dig_P8) * p) >> 19;
    // cout << "var2 = " << var2 << endl;

    p = ((p + var1 + var2) >> 8) + (((int64_t) bmp280_calib.dig_P7)<<4);
    // cout << "p = " << p << endl;

    // cout << "=/= READ PRESSURE =/=\n";

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
