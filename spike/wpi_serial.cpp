#include <iostream>
#include <stdio.h>

#include <wiringSerial.h>

int main()
{
    int fd;
    if ((fd = serialOpen("/dev/ttyACM0", 115200)) < 0)
    // if ((fd = serialOpen("/dev/ttyS0", 9600)) < 0)
    {
        std::cerr << "Failed to open" << std::endl;
        return 1;
    }
    for (;;)
    {
        std::cout << (char) serialGetchar(fd) << std::flush;
    }
    return 0;
}
