/******************************************************************************
i2ctest.cpp
Raspberry Pi I2C interface demo
Byron Jacquot @ SparkFun Electronics>
4/2/2014
https://github.com/sparkfun/Pi_Wedge

A brief demonstration of the Raspberry Pi I2C interface, using the SparkFun
Pi Wedge breakout board.

The I2C API is documented here:
https://projects.drogon.net/raspberry-pi/wiringpi/i2c-library/

The init call returns a standard file descriptor.  More detailed configuration
of the interface can be performed using ioctl calls on that descriptor.
See the wiringPi I2C implementation (wiringPi/wiringPiI2C.c) for some examples.
Parameters configurable with ioctl are documented here:
http://git.kernel.org/cgit/linux/kernel/git/torvalds/linux.git/tree/Documentation/i2c/dev-interface

To build this file, I use the command:
>  g++ i2ctest.cpp -lwiringPi

Then to run it, first the I2C kernel module needs to be loaded.
This can be done using the GPIO utility.
> gpio load i2c 400
> ./a.out

******************************************************************************/

#include <iostream>
#include <stdio.h>
#include <errno.h>
#include <wiringPiI2C.h>

using namespace std;

int main()
{
    int fd, result, k = 0;

    fd = wiringPiI2CSetup(0x77);

    cout << "Init result: "<< fd << endl;

    for (int i = 0x80; i < 0xff; i++)
    {
        result = wiringPiI2CReadReg8(fd, i);
        if (result > 0)
        {
            printf("%02x ", result);
            k++;
        }
        if (k == 16) { cout << endl; k = 0; }
        if (result == -1)
        {
            cout << "Error (" << errno << ")" << endl;
            i = 0xff;
        }
    }
    if (result != -1) cout << endl;
}
