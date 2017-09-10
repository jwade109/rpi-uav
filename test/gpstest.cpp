#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>

#include <gps.h>

double degf(uint32_t f) { return f/10000000.0; }

uint32_t deg(uint32_t f) { return f/10000000; }

uint32_t dec(uint32_t f) { return f%10000000; }

std::string to_string(const gps_data& data)
{
    using namespace std;

    stringstream ss;
    ss << (int) data.hour
        << ":" << setw(2) << setfill('0') << (int) data.minute
        << ":" << setw(2) << setfill('0') << (int) data.seconds
        << " " << deg(data.latitude_fixed)
        << "." << dec(data.latitude_fixed)
        << " " << data.lat
        << " " << deg(data.longitude_fixed)
        << "." << dec(data.longitude_fixed)
        << " " << data.lon
        << " " << data.altitude
        << " " << (int) data.fixquality
        << " " << data.HDOP;
    return ss.str();
}

int main()
{
    bool first = true;
    gps_data ref;

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
            if (first && data.fixquality)
            {
                first = false;
                ref = data;
            }

            // earth circumference/360 degrees
            int scale = 111320;

            std::cout << to_string(data) << " | "
                << (degf(data.latitude_fixed) -
                    degf(ref.latitude_fixed)) * scale
                << " " << (degf(data.longitude_fixed) -
                    degf(ref.longitude_fixed)) * scale
                << std::endl;
        }
    }
}
