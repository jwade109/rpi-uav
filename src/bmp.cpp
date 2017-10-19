#include <iostream>
#include <thread>
#include <chrono>
#include <math.h>
#include <stdio.h>

#include <bmp.h>

const uint8_t BMP085_CHIPID = 0x55;

using namespace std::chrono;

uav::bmp085::bmp085(): slp(1013.25), temp(0),
    press(0), alt(0), data({0}), cont(false) { }

uav::bmp085::~bmp085()
{
    cont = false;
    if (reader.joinable()) reader.join();
}

void uav::bmp085::readCoefficients(void)
{
    bmp085_coeffs.ac1 = (int16_t) i2c.read16_BE(
            BMP085_REGISTER_CAL_AC1);
    bmp085_coeffs.ac2 = (int16_t) i2c.read16_BE(
            BMP085_REGISTER_CAL_AC2);
    bmp085_coeffs.ac3 = (int16_t) i2c.read16_BE(
            BMP085_REGISTER_CAL_AC3);
    bmp085_coeffs.ac4 = i2c.read16_BE(BMP085_REGISTER_CAL_AC4);
    bmp085_coeffs.ac5 = i2c.read16_BE(BMP085_REGISTER_CAL_AC5);
    bmp085_coeffs.ac6 = i2c.read16_BE(BMP085_REGISTER_CAL_AC6);
    bmp085_coeffs.b1 = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_B1);
    bmp085_coeffs.b2 = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_B2);
    bmp085_coeffs.mb = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_MB);
    bmp085_coeffs.mc = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_MC);
    bmp085_coeffs.md = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_MD);
}

int32_t uav::bmp085::readRawTemperature(void)
{
    i2c.write8(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);
    std::this_thread::sleep_for(milliseconds(5));
    return i2c.read16_BE(BMP085_REGISTER_TEMPDATA);
}

int32_t uav::bmp085::readRawPressure(void)
{
    i2c.write8(BMP085_REGISTER_CONTROL,
               BMP085_REGISTER_READPRESSURECMD + (bmp085Mode << 6));
    switch(bmp085Mode)
    {
        case ULTRALOWPOWER:
            std::this_thread::sleep_for(milliseconds(5));
            break;
        case STANDARD:
            std::this_thread::sleep_for(milliseconds(8));
            break;
        case HIGHRES:
            std::this_thread::sleep_for(milliseconds(14));
            break;
        case ULTRAHIGHRES:
        default:
            std::this_thread::sleep_for(milliseconds(26));
            break;
    }

    uint16_t p16 = i2c.read16_BE(BMP085_REGISTER_PRESSUREDATA);
    int32_t p32 = (uint32_t) p16 << 8;
    uint8_t p8 = i2c.read8(BMP085_REGISTER_PRESSUREDATA+2);
    p32 += p8;
    p32 >>= (8 - bmp085Mode);
    return p32;
}

int32_t uav::bmp085::computeB5(int32_t ut)
{
    int32_t X1 = (ut - (int32_t) bmp085_coeffs.ac6) *
        ((int32_t) bmp085_coeffs.ac5) >> 15;
    int32_t X2 = ((int32_t) bmp085_coeffs.mc << 11) /
        (X1 + (int32_t) bmp085_coeffs.md);
    return X1 + X2;
}

int uav::bmp085::begin(uint8_t addr, bmp085_mode_t mode)
{
    if (cont) return 0;

    int ret = i2c.open(addr);
    if (!ret)
    {
        std::cerr << "bmp085: I2C failed to init: "
                  << ret << std::endl;
        return 1;
    }

    if(i2c.read8(BMP085_REGISTER_CHIPID) != BMP085_CHIPID)
    {
        std::cerr << "bmp085: No bmp085 at i2c addr: 0x"
                  << std::hex << (int) addr << std::dec << std::endl;
        return 2;
    }

    if ((mode > ULTRAHIGHRES) || (mode < 0))
    {
        mode = ULTRAHIGHRES;
    }

    bmp085Mode = mode;
    readCoefficients();

    cont = true;
    reader = std::thread(&bmp085::work, this);

    return 0;
}

float uav::bmp085::updatePressure()
{
    int32_t  ut = 0, up = 0, compp = 0;
    int32_t  x1, x2, b5, b6, x3, b3, p;
    uint32_t b4, b7;

    /* Get the raw pressure and temperature values */
    ut = readRawTemperature();
    up = readRawPressure();

    /* Temperature compensation */
    b5 = computeB5(ut);

    /* Pressure compensation */
    b6 = b5 - 4000;
    x1 = (bmp085_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
    x2 = (bmp085_coeffs.ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t) bmp085_coeffs.ac1) * 4 + x3) << bmp085Mode) + 2) >> 2;
    x1 = (bmp085_coeffs.ac3 * b6) >> 13;
    x2 = (bmp085_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (bmp085_coeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
    b7 = ((uint32_t) (up - b3) * (50000 >> bmp085Mode));

    if (b7 < 0x80000000)
    {
        p = (b7 << 1) / b4;
    }
    else
    {
        p = (b7 / b4) << 1;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    compp = p + ((x1 + x2 + 3791) >> 4);

    /* Assign compensated pressure value */
    return compp;
}

float uav::bmp085::updateTemperature(void)
{
    int32_t UT = readRawTemperature();

    #ifdef BMP085_USE_DATASHEET_VALS
        // use datasheet numbers!
        UT = 27898;
        bmp085_coeffs.ac6 = 23153;
        bmp085_coeffs.ac5 = 32757;
        bmp085_coeffs.mc = -8711;
        bmp085_coeffs.md = 2868;
    #endif

    int32_t B5 = computeB5(UT);
    float t = (B5+8) >> 4;
    t /= 10;
    return t;
}

float uav::bmp085::getAltitude()
{
    return alt;
}

float uav::bmp085::getTemperature() 
{
    return temp;
}

float uav::bmp085::getPressure()
{
    return press;
}

void uav::bmp085::work()
{
    while (cont)
    {
        temp = updateTemperature();
        press = updatePressure();
        alt = 44330 * (1.0 - pow(0.01 * press / slp, 0.1903));
        data = bmp085_data{temp, press};
    }
}

double uav::bmp085::altitude(double p, double hp)
{
    return 44330 * (1.0 - pow(p / hp, 0.1903));
}
