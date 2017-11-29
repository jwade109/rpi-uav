#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <string>
#include <sstream>
#include <vector>
#include <bitset>
#include <signal.h>

#include <uav/math>
#include <uav/hardware>

bool cont = true;

void sigint(int sig)
{
    cont = false;
}

int main(int argc, char** argv)
{
    signal(SIGINT, sigint);

    uav::sensor_hub sensors;
    if (sensors.begin()) return 1;

    std::this_thread::sleep_for(std::chrono::seconds(2));
    const uint8_t freq = 50;

    std::stringstream out;
    out << std::left << std::fixed << std::setprecision(3);
    out << std::setw(12) << "timestamp"
        << std::setw(12) << "fix"
        << std::setw(45) << "position"
        << std::setw(12) << "gps heading"
        << std::setw(12) << "knots"
        << std::setw(12) << "mph"
        << std::setw(12) << "HDOP";

    std::cout << out.str() << std::endl;
    std::cerr << out.str() << "\n";
    out.str("");
    out.clear();
    
    uint64_t counter = 0;
    auto start = std::chrono::steady_clock::now(), now = start;
    auto delay = std::chrono::milliseconds(1000/freq),
         runtime = delay * 0;

    while (cont)
    {
        while (now < start + runtime)
            now = std::chrono::steady_clock::now();

        auto raw = sensors.get();
        double mph = raw.gps.rmc.ground_speed * 0.621371;

        using fsec = std::chrono::duration<double, std::ratio<1>>;
        static auto start(std::chrono::steady_clock::now());
        auto now = std::chrono::steady_clock::now();
        auto dec_seconds = std::chrono::duration_cast<fsec>(now - start);

        out << std::setw(12) << dec_seconds.count() // "timestamp"
            << std::setw(12) <<
                (raw.gps.gga.fix_quality == 2 ? "SBAS" : "SP") // "fix"
            << std::setw(45) << raw.gps.gga.pos // "position"
            << std::setw(12) << raw.gps.rmc.track_angle // "track_good"
            << std::setw(12) << raw.gps.rmc.ground_speed // "knots"
            << std::setw(12) << mph // "mph"
            << std::setw(12) << raw.gps.gga.hdop; // "HDOP"
        std::cout << out.str() << "\n";
        std::cerr << out.str() << "   \r" << std::flush;
        out.str("");
        out.clear();

        runtime += delay;
    }

    std::cout << std::flush;
    std::cerr << "\nDone.\n" << std::flush;
    return 0;
}
