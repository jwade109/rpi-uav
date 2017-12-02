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
              << "time GMT latitude longitude altitude" << std::endl;
    while (1)
    {
        auto gp = r.get();
        if (true)
        {
            auto hdg = uav::angle::degrees(90 - gp.rmc.track_angle);
            double mps = gp.rmc.ground_speed/2;
            double vx = mps*std::cos(hdg), vy = mps*std::sin(hdg);

            std::cout << gp.gga.newflag << " "
                << gp.gga.utc << " "
                << gp.gga.pos << " "
                << (int) gp.gga.fix_quality << " "
                << gp.rmc.newflag << " "
                << vx << " " << vy << "      \r" << std::flush;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    return 0;
}
