#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>

#include <uav/hardware>

#include "ins.h"

uint8_t calculate_checksum(const std::string& msg)
{
    return std::accumulate(begin(msg), end(msg), 0,
            [] (uint8_t sum, char ch) { return sum^ch; });
}

std::string make_ins(uint64_t msow, double x, double y, double z,
        double vx, double vy, double vz,
        double hdg, double pitch, double roll,
        double va, double vb, double vg)
{
    std::stringstream ss;
    
    ss << std::setprecision(3) << std::fixed;
    ss << "INS," << (msow / 1000) << "."
       << std::setw(3) << std::setfill('0') << (msow % 1000) << ",";
    ss << x << "," << y << "," << z << ","
       << vx << "," << vy << "," << vz << ","
       << hdg << "," << pitch << "," << roll << ","
       << va << "," << vb << "," << vg;

    int checksum = calculate_checksum(ss.str());
    ss << "*" << std::hex << std::setw(2) << std::setfill('0') << checksum;

    return "$" + ss.str();
}

void print_vector(std::ostream& os, const Eigen::Vector3d& v)
{
    os << std::fixed << std::setprecision(2);
    for (int i = 0; i < 3; ++i)
    {
        os << std::setw(7) << v(i) << " ";
    }
}

int main()
{
    const uint8_t freq = 50;
    uav::ins ins(freq);
    if (!ins.begin()) return 1;

    while (true)
    {
        ins.update();
        std::cout << ins.tow().count()/1000.0 << " ";
        // std::cout << ins.position() << " ";
        print_vector(std::cout, ins.displacement());
        print_vector(std::cout, ins.velocity());
        print_vector(std::cout, ins.attitude());
        print_vector(std::cout, ins.turn_rate());
        std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;

    uav::sensor_hub sensors;
    if (sensors.begin() > 0) return 0;

    auto home = sensors.get().gps.gga.pos;

    while (true)
    {
        uint64_t unix_ms = std::chrono::duration_cast
            <std::chrono::milliseconds>(std::chrono::system_clock::now()
            .time_since_epoch()).count();
        uint64_t gps_ms = unix_ms - 315964800000 + 20000;
        uint64_t gps_tow = gps_ms % 604800000;        
        auto data = sensors.get();
        auto pos = data.gps.gga.pos - home;
        double x = pos(0), y = pos(1), z = pos(2);
        double vx = 0, vy = 0, vz = 0;
        double hdg = data.ard.euler.x(),
               pitch = data.ard.euler.y(),
               roll = data.ard.euler.z();
        double va = 0, vb = 0, vg = 0;
        auto message = make_ins(gps_tow, x, y, z, vx, vy, vz,
                hdg, pitch, roll, va, vb, vg);
        std::cout << "  " << message << "  \r" << std::flush;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
