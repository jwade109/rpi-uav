#include "ardimu.h"

#include <iostream>
#include <string>
#include <sstream>
#include <wiringSerial.h>

uav::arduino::arduino(): data({0}), cont(false), status(0), fd(-1) { }

uav::arduino::~arduino()
{
    cont = false;
    if (parser.joinable()) parser.join();
    serialClose(fd);
}

int uav::arduino::begin()
{
    if (cont) return 0;

    fd = serialOpen("/dev/ttyACM0", 115200);
    if (fd < 0)
    {
        std::cerr << "arduino: failed to open /dev/ttyACM0" << std::endl;
        return 1;
    }

    cont = true;
    parser = std::thread(&arduino::parse, this);

    while (status == 0);
    if (status == -1)
    {
        cont = false;
        if (parser.joinable()) parser.join();
        std::cerr << "arduino: connection timed out" << std::endl;
        return 2;
    }

    return 0;
}

uav::arduino_data uav::arduino::get() const
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
            arduino_data d;
            char* cursor;
            d.millis = strtol(ss.str().c_str(), &cursor, 10);
            d.euler.x() = strtod(cursor, &cursor);
            d.euler.y() = strtod(cursor, &cursor);
            d.euler.z() = strtod(cursor, &cursor);
            d.calib = strtol(cursor, &cursor, 10);
            d.pres = strtod(cursor, &cursor)/101325.0;
            d.acc.x() = strtod(cursor, &cursor);
            d.acc.y() = strtod(cursor, &cursor);
            d.acc.z() = strtod(cursor, &cursor);
            data = d;
            ss.str("");
            ss.clear();
        }
        if (recieved) ss << ch;
        ch = serialGetchar(fd);
        if (data.load().pres > 0 && status < 1) status = 1;
    }
}
