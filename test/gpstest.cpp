#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>

#include <uav/hardware>
#include <uav/filter>

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
    uav::coordinate home;

    uav::low_pass xlpf(2), ylpf(2);

    while (gp.gga.num_sats == 0)
    {
        std::cout << load() << "\r" << std::flush;
        r.update(gp);
    }

    home = gp.gga.pos;

    while (1)
    {
        if (r.update(gp));
        {
            imu::Vector<2> rel = gp.gga.pos - home;

            imu::Vector<2> rel_filt =
                {xlpf.step(rel.x(), 0.02), ylpf.step(rel.y(), 0.02)};

            std::cout << (int) gp.rmc.month << "/"
                << (int) gp.rmc.day << "/"
                << (int) gp.rmc.year << " "
                << gp.gga.utc << " "
                << gp.gga.pos.lat << " "
                << gp.gga.pos.lon << " "
                << gp.gga.altitude << " "
                << (int) gp.gga.num_sats << " "
                // << rel << " " << rel_filt
                << "      \r" << std::flush;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}
