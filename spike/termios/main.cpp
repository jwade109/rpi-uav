#include <stdio.h>
#include <termios.h>
#include <fcntl.h>

int main()
{
    struct termios attribs;
    speed_t speed = B115200;
    int fd = open("/dev/ttyACM0", O_RDONLY);

    // get attributes from device
    int res = tcgetattr(fd, &attribs);
    if (res < 0)
    {
        printf("Error: 1\n");
        return 1;
    }

    // set speed in attributes
    res = cfsetispeed(&attribs, speed);
    res += cfsetospeed(&attribs, speed);
    if (res < 0)
    {
        printf("Error: 2\n");
        return 2;
    }

    // apply attributes
    res = tcsetattr(fd, TCSANOW, &attribs);
    if (res < 0)
    {
        printf("Error: 3\n");
        return 3;
    }

    return 0;
}
