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
#include <uav/algorithm>

bool cont = true;

void sigint(int sig)
{
    cont = false;
}

int main(int argc, char** argv)
{
    signal(SIGINT, sigint);

    uav::gps gps_rec;
    if (gps_rec.begin()) return 1;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    const uint8_t freq = 50;
    double dt = 1.0/freq;

    std::stringstream out;
    out << std::left << std::fixed << std::setprecision(6);
    out << std::setw(12) << "timestamp"
        << std::setw(5) << "gga"
        << std::setw(5) << "rmc"
        << std::setw(5) << "fix"
        << std::setw(12) << "x"
        << std::setw(12) << "y"
        << std::setw(12) << "vx"
        << std::setw(12) << "vy"
        << std::setw(12) << "gx"
        << std::setw(12) << "gy"
        << std::setw(12) << "sx"
        << std::setw(12) << "sy"
        << std::setw(12) << "svx"
        << std::setw(12) << "svy";

    std::cout << out.str() << std::endl;
    std::cerr << out.str() << "\n";
    out.str("");
    out.clear();
    
    uint64_t counter = 0;
    auto start = std::chrono::steady_clock::now(), now = start;
    auto delay = std::chrono::milliseconds(1000/freq),
         runtime = delay * 0;

    const double kP = 10, kI = 0.1, kD = 10;
    pid_controller easting(freq, kP, kI, kD), northing(freq, kP, kI, kD);

    auto home_point = gps_rec.get().gga.pos;
    double gx = 0, gy = 0, sx = 0, sy = 0, svx = 0, svy = 0;
    Eigen::Vector3d last_disp, disp_gps;

    while (cont)
    {
        while (now < start + runtime)
            now = std::chrono::steady_clock::now();

        last_disp = disp_gps;

        auto gps = gps_rec.get();
        disp_gps = gps.gga.pos - home_point;
        double mps = gps.rmc.ground_speed/2;
        auto hdg = uav::angle::degrees(90 - gps.rmc.track_angle);
        double vx = mps*std::cos(hdg), vy = mps*std::sin(hdg);

        const double maxres = 0.2; // meters
        
        gx += vx * dt; // propogate dead reckoning
        if (std::abs(disp_gps(0) - gx) > maxres) gx = disp_gps(0);
        
        gy += vy * dt; // propogate dead reckoning
        if (std::abs(disp_gps(1) - gy) > maxres) gy = disp_gps(1);

        const double correction = 0.02/(10 + 0.02);
        gx += (disp_gps(0) - gx) * correction;
        gy += (disp_gps(1) - gy) * correction;

        double ax = easting.seek(sx, gx) + kD * vx;
        double ay = northing.seek(sy, gy) + kD * vy;
        svx += ax * dt;
        svy += ay * dt;
        sx += svx * dt;
        sy += svy * dt;

        using fsec = std::chrono::duration<double, std::ratio<1>>;
        static auto start(std::chrono::steady_clock::now());
        auto now = std::chrono::steady_clock::now();
        auto dec_seconds = std::chrono::duration_cast<fsec>(now - start);

        out << std::setw(12) << dec_seconds.count() // timestamp
            << std::setw(5) << gps.gga.newflag // gga
            << std::setw(5) << gps.rmc.newflag // rmc
            << std::setw(5) << (gps.gga.fix_quality > 1 ? "SBAS" : "SP") // fix

            << std::setw(12) << disp_gps(0) // gps_x
            << std::setw(12) << disp_gps(1) // gps_y
            << std::setw(12) << vx
            << std::setw(12) << vy
            << std::setw(12) << gx // guess_x
            << std::setw(12) << gy // guess_y
            << std::setw(12) << sx // smooth_x
            << std::setw(12) << sy // smooth_y
            << std::setw(12) << svx
            << std::setw(12) << svy;

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
