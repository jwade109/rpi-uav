#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>

#include <uav/hardware>
#include <uav/algorithm>

int main()
{
    uav::gps r;
    int ret = r.begin();
    if (ret)
    {
        std::cerr << "Error: " << ret << std::endl;
        return 1;
    }
    std::cout << std::fixed << std::setprecision(2)
              << "utc tow latitude longitude altitude "
                 "fix gga rmc vx vy" << std::endl;
    while (1)
    {
        auto gp = r.get();
        if (true)
        {
            auto hdg = uav::angle::degrees(90 - gp.rmc.track_angle);
            double mps = gp.rmc.ground_speed/2;
            double vx = mps*std::cos(hdg), vy = mps*std::sin(hdg);

            uint64_t unix_ms = std::chrono::duration_cast
                <std::chrono::milliseconds>(std::chrono::system_clock::now()
                .time_since_epoch()).count();
            uint64_t gps_ms = unix_ms - 315964800000 + 20000;
            uint64_t gps_tow = gps_ms % 604800000;

            std::cout << gp.gga.utc << " "
                << gps_tow/1000.0 << " "
                << gp.gga.pos << " "
                << (int) gp.gga.fix_quality << " "
                << gp.gga.newflag << " "
                << gp.rmc.newflag << " "
                << vx << " " << vy << "      \r" << std::flush;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return 0;
}
