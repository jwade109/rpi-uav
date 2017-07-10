#include <math.h>
#include <BMP085.h>
#include <I2C.h>
#include <stdint.h>
#include <TimeUtil.h>

#define BMP085_USE_DATASHEET_VALS (0) // Set to 1 for sanity check

BMP085::BMP085()
{
    i2c = I2C(0);
}

void BMP085::readCoefficients(void)
{
    #if BMP085_USE_DATASHEET_VALS
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
    #if BMP085_USE_DATASHEET_VALS
        return 27898;
    #else
        i2c.write8(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READTEMPCMD);
        // delay(5)
        waitFor(5, MILLI);
        return i2c.read16_BE(BMP085_REGISTER_TEMPDATA);
    #endif
}

int32_t BMP085::readRawPressure(void)
{
    #if BMP085_USE_DATASHEET_VALS
        return 23843;
    #else
        i2c.write8(BMP085_REGISTER_CONTROL, BMP085_REGISTER_READPRESSURECMD + (bmp085Mode << 6));
        switch(bmp085Mode)
        {
            case ULTRALOWPOWER:
                // delay(5);
                waitFor(5, MILLI);
                break;
            case STANDARD:
                // delay(8);
                waitFor(8, MILLI);
                break;
            case HIGHRES:
                // delay(14);
                waitFor(14, MILLI);
                break;
            case ULTRAHIGHRES:
            default:
                // delay(26);
                waitFor(26, MILLI);
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

bool BMP085::begin(uint8_t addr, bmp085_mode_t mode)
{
    i2c = I2C(addr);
    if (!i2c.ready()) return false;
    if ((mode > ULTRAHIGHRES) || (mode < 0))
    {
        mode = ULTRAHIGHRES;
    }

    if(i2c.read8(BMP085_REGISTER_CHIPID) != BMP085_CHIPID)
        return false;
    bmp085Mode = mode;
    readCoefficients();
    return true;
}

float BMP085::pressure()
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

float BMP085::temperature(void)
{
    int32_t UT = readRawTemperature();

    #if BMP085_USE_DATASHEET_VALS
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

float BMP085::altitude(float seaLevelhPa)
{
    float press = pressure(); // in Si units for Pascal
    press /= 100;
    float altitude = 44330 * (1.0 - pow(press / seaLevelhPa, 0.1903));
    return altitude;
}
