#include <iostream>
#include <string>
#include <sstream>

#include <ardimu.h>
#include <wiringSerial.h>

uav::arduino::arduino(): data{0}, fd(-1), cont(true), status(0) { }

uav::arduino::~arduino()
{
    cont = false;
    if (parser.joinable()) parser.join();
    serialClose(fd);
}

int uav::arduino::begin()
{
    fd = serialOpen("/dev/ttyACM0", 115200);
    if (fd < 0)
    {
        std::cerr << "arduino: failed to open /dev/ttyACM0" << std::endl;
        return 1;
    }

    parser = std::thread(&arduino::parse, this);

    while (status == 0);
    if (status == -1)
    {
        std::cerr << "arduino: connection timed out" << std::endl;
        return 2;
    }

    return 0;
}

const uav::imu_packet& uav::arduino::get() const
{
    return data;
}

void uav::arduino::parse()
{
    bool recieved = false;
    char ch = 0;
    std::stringstream ss;

    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(3);

    while (cont)
    {
        if (start + timeout < std::chrono::steady_clock::now())
        {
            status = -1;
            return;
        }
        if (ch == '#') break;
        if (serialDataAvail(fd) > 0) ch = serialGetchar(fd);
    }

    status = 1;

    while (cont)
    {
        if (ch == '<')
        {
            recieved = true;
            ch = serialGetchar(fd);
        }
        else if (ch == '>')
        {
            recieved = false;
            imu_packet d;
            char* cursor;
            d.millis = strtol(ss.str().c_str(), &cursor, 10);
            d.heading = strtod(cursor, &cursor);
            d.pitch = strtod(cursor, &cursor);
            d.roll = strtod(cursor, &cursor);
            d.calib = strtol(cursor, &cursor, 10);
            d.pres = strtod(cursor, &cursor);
            d.ax = strtod(cursor, &cursor);
            d.ay = strtod(cursor, &cursor);
            d.az = strtod(cursor, &cursor);
            data = d;
            ss.str("");
            ss.clear();
        }
        if (recieved) ss << ch;
        ch = serialGetchar(fd);
    }
}
