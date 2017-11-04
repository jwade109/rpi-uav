#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>

#include <uav/hardware>
#include <uav/algorithm>

char load()
{
    static int count = 0;
    count = ++count > 3 ? 0 : count;
    switch (count)
    {
        case 0: return '-';
        case 1: return '\\';
        case 2: return '|';
        case 3: return '/';
    }
    return '?';
}

int main()
{
    uav::gps r;
    int ret = r.begin();
    if (ret)
    {
        std::cerr << "Error: " << ret << std::endl;
        return 1;
    }
    uav::gps_data gp{0};

    while (gp.gga.num_sats == 0)
    {
        std::cout << load() << "\r" << std::flush;
        r.update(gp);
    }

    while (1)
    {
        if (r.update(gp));
        {
            std::cout << (int) gp.rmc.month << "/"
                << (int) gp.rmc.day << "/"
                << (int) gp.rmc.year << " "
                << gp.gga.utc << " "
                << gp.gga.pos << " "
                << (int) gp.gga.num_sats << " "
                << "      \r" << std::flush;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}
