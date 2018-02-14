#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>

#include <uav/hardware>

#include "navigator.h"

uint8_t calculate_checksum(const std::string& msg)
{
    return std::accumulate(begin(msg), end(msg), 0,
            [] (uint8_t sum, char ch) { return sum^ch; });
}

std::string make_nav(uint64_t msow, double x, double y, double z,
        double vx, double vy, double vz,
        double hdg, double pitch, double roll,
        double va, double vb, double vg)
{
    std::stringstream ss;
    
    ss << std::setprecision(3) << std::fixed;
    ss << "NAV," << (msow / 1000) << "."
       << std::setw(3) << std::setfill('0') << (msow % 1000) << ",";
    ss << x << "," << y << "," << z << ","
       << vx << "," << vy << "," << vz << ","
       << hdg << "," << pitch << "," << roll << ","
       << va << "," << vb << "," << vg;

    int checksum = calculate_checksum(ss.str());
    ss << "*" << std::hex << std::setw(2) << std::setfill('0') << checksum;

    return "$" + ss.str();
}

void print_quat(std::ostream& os, const Eigen::Quaterniond& q)
{
    os << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " ";
}

void print_vector(std::ostream& os, const Eigen::Vector3d& v)
{
    for (int i = 0; i < 3; ++i)
    {
        os << v(i) << " ";
    }
}

int main()
{
    const uint8_t freq = 50;
    uav::navigator nav(freq);
    // if (!nav.begin()) return 1;

    std::cout << std::fixed << std::setprecision(3) << std::boolalpha
              << "tow position displacement drift velocity "
              << "attitude turn_rate dynamic" << std::endl;

    while (nav.tow().count() % (1000/freq) > 0) { }
    auto start = std::chrono::steady_clock::now();
    auto runtime = std::chrono::milliseconds(0);
    auto delay = std::chrono::milliseconds(1000/freq);

    nav.reconcile_displacement({0, 0, 10});
    nav.body().velocity() = {0, 0, -5};
    
    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        while (now < start + runtime)
            now = std::chrono::steady_clock::now();

        // nav.update();
        std::cout << nav.tow().count()/1000.0 << " ";
        // std::cout << nav.position() << " ";
        print_vector(std::cout, nav.displacement());
        // print_vector(std::cout, nav.drift());
        print_vector(std::cout, nav.velocity());
        print_vector(std::cout, nav.attitude());
        // print_vector(std::cout, nav.turn_rate());
        std::cout << "\n" << std::flush;

        nav.predict({2.6, 2.6, 2.6, 2.6}, 1);
        
        runtime += delay;
    }

    return 0;
}
