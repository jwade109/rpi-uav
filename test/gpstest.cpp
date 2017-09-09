#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>

#include <gps.h>

std::string to_string(const gps_data& data)
{
    auto deg = [](uint32_t f) { return f/10000000; };
    auto dec = [](uint32_t f) { return f%10000000; };

    std::stringstream ss;
    ss << (int) data.hour << ":" << (int) data.minute
        << ":" << (int) data.seconds
        << " " << deg(data.latitude_fixed)
        << "." << dec(data.latitude_fixed)
        << " " << data.lat
        << " " << deg(data.longitude_fixed)
        << "." << dec(data.longitude_fixed)
        << " " << data.lon
        << " " << data.altitude;
    return ss.str();
}

int main()
{
    gps r;
    int ret = r.begin();
    if (ret)
    {
        std::cerr << "Error: " << ret << std::endl;
        return 1;
    }
    while(1)
    {
        if (r.isnew())
        {
            auto data = r.get();
            std::cout << std::endl << to_string(data) << std::endl;
        }
    }
}
