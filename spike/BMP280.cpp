#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <wiringPiI2C.h>
#include <errno.h>
#include <stdbool.h>

using namespace std;

enum
{
        ID            = 0xD0,
        RESET         = 0xE0
};

class BMP280
{
    public:

        BMP280();
        ~BMP280();
        bool init(uint8_t addr);
        uint8_t read(uint8_t reg);

    private:

        int filehandle;

};

BMP280::BMP280()
{
    filehandle = -1;
}

BMP280::~BMP280()
{
    // close the I2C device
}

bool BMP280::init(uint8_t addr)
{
    filehandle = wiringPiI2CSetup(addr);
    if (filehandle < 0) return false;
    uint8_t id = wiringPiI2CReadReg8(filehandle, ID);
    if (id != 0x58) return false;
    return true;
}

uint8_t BMP280::read(uint8_t reg)
{
    if (filehandle < 0) return 0;
    uint8_t dat = wiringPiI2CReadReg8(filehandle, reg);
    return dat;
}

// big-endian: low address, high significance
// little-endian: low address, least significant byte

// arrays are little endian
void to8BitArray(uint8_t val, bool array[8])
{
    uint8_t mask = 1;
    for (uint8_t i = 0; i < 8; i++)
    {
        array[i] = val & (mask << i);
    }
}

void to16BitArray(uint16_t val, bool array[16])
{
    uint8_t mask = 1;
    for (uint8_t i = 0; i < 16; i++)
    {
        array[i] = val & (mask << 1);
    }
}

uint8_t from8BitArray(bool array[8])
{
    uint8_t val = 0;
    for (int i = 0; i < 8; i++)
    {
        val += (array[i] << i);
    }
    return val;
}

uint16_t from16BitArray(bool array[16])
{
    uint16_t val = 0;
    for (int i = 0; i < 16; i++)
    {
        val += (array[i] << i);
    }
    return val;
}

int main(int argc, char** argv)
{
    bool arr[8];
    uint8_t val = 13; // 1101
    to8BitArray(val, arr);
    cout << (int) from8BitArray(arr) << endl;

    bool larr[16];
    uint16_t large = -1;
    to16BitArray(large, larr);
    for (int i = 0; i < 16; i++)
        cout << larr[i] << " ";
    cout << endl;
    cout << (int) from16BitArray(larr) << endl;

    cout << endl;
    BMP280 bmp;
    bool status = bmp.init(0x77);
    cout << status << endl;
    if (!status) return 1;
    for (int i = 0; i < 0xff; i++)
    {
        printf("%02x ", bmp.read(i));
        if (i % 16 == 15) cout << endl;
    }
    cout << endl;
    return 0;
}
