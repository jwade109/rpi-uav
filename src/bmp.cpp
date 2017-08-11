#include <math.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <sys/prctl.h>
#include <smem.h>
#include <bmp.h>
#include <thread>
#include <chrono>

using namespace std::chrono;

BMP085::BMP085()
{
    i2c = I2C(0);
    child_pid = -1;
}

BMP085::~BMP085()
{
    if (child_pid > 0) kill(child_pid, SIGKILL);
}

void BMP085::readCoefficients(void)
{
    #ifdef BMP085_USE_DATASHEET_VALS
        bmp085_coeffs.ac1 = 408;
        bmp085_coeffs.ac2 = -72;
        bmp085_coeffs.ac3 = -14383;
        bmp085_coeffs.ac4 = 32741;
        bmp085_coeffs.ac5 = 32757;
        bmp085_coeffs.ac6 = 23153;
        bmp085_coeffs.b1  = 6190;
        bmp085_coeffs.b2  = 4;
        bmp085_coeffs.mb  = -32768;
        bmp085_coeffs.mc  = -8711;
        bmp085_coeffs.md  = 2868;
        bmp085Mode        = 0;
    #else
        bmp085_coeffs.ac1 = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_AC1);
        bmp085_coeffs.ac2 = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_AC2);
        bmp085_coeffs.ac3 = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_AC3);
        bmp085_coeffs.ac4 = i2c.read16_BE(BMP085_REGISTER_CAL_AC4);
        bmp085_coeffs.ac5 = i2c.read16_BE(BMP085_REGISTER_CAL_AC5);
        bmp085_coeffs.ac6 = i2c.read16_BE(BMP085_REGISTER_CAL_AC6);
        bmp085_coeffs.b1 = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_B1);
        bmp085_coeffs.b2 = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_B2);
        bmp085_coeffs.mb = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_MB);
        bmp085_coeffs.mc = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_MC);
        bmp085_coeffs.md = (int16_t) i2c.read16_BE(BMP085_REGISTER_CAL_MD);
    #endif
}

int32_t BMP085::readRawTemperature(void)
{
    #ifdef BMP085_USE_DATASHEET_VALS
        return 27898;
    #else
        i2c.write8(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);
        std::this_thread::sleep_for(milliseconds(5));
        return i2c.read16_BE(BMP085_REGISTER_TEMPDATA);
    #endif
}

int32_t BMP085::readRawPressure(void)
{
    #ifdef BMP085_USE_DATASHEET_VALS
        return 23843;
    #else
        i2c.write8(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READPRESSURECMD + (bmp085Mode << 6));
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
    #endif
}

int32_t BMP085::computeB5(int32_t ut)
{
    int32_t X1 = (ut - (int32_t) bmp085_coeffs.ac6) * ((int32_t) bmp085_coeffs.ac5) >> 15;
    int32_t X2 = ((int32_t) bmp085_coeffs.mc << 11) / (X1 + (int32_t) bmp085_coeffs.md);
    return X1 + X2;
}

int BMP085::begin(uint8_t addr, bmp085_mode_t mode)
{
    if (child_pid != -1)
    {
        fprintf(stderr, "BMP085: Child process already exists\n");
        return 1;
    }

    i2c = I2C(addr);
    int ret = i2c.ready();
    if (!ret)
    {
        fprintf(stderr, "BMP085: I2C failed to init, returned %d\n", ret);
        return 2;
    }

    if(i2c.read8(BMP085_REGISTER_CHIPID) != BMP085_CHIPID)
    {
        fprintf(stderr, "BMP085: No BMP085 at i2c addr: 0x%02x\n", addr);
        return 3;
    }

    mem = (float*) sharedmem(sizeof(float) * 2);
    memset(mem, 0, sizeof(float) * 2);

    int pid = fork();
    if (pid > 0)
    {
        child_pid = pid;
        return 0;
    }

    prctl(PR_SET_NAME, "bmp");

    if ((mode > ULTRAHIGHRES) || (mode < 0))
    {
        mode = ULTRAHIGHRES;
    }

    bmp085Mode = mode;
    readCoefficients();

    for (;;)
    {
        mem[0] = updateTemperature();
        mem[1] = updatePressure();
    }

    return 0;
}

float BMP085::updatePressure()
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

float BMP085::updateTemperature(void)
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

float BMP085::getAltitude(float seaLevelhPa)
{
    float press = mem[1]; // in Si units for Pascal
    press /= 100;
    float altitude = 44330 * (1.0 - pow(press / seaLevelhPa, 0.1903));
    return altitude;
}

float BMP085::getTemperature()
{
    return mem[0];
}

float BMP085::getPressure()
{
    return mem[1];
}

