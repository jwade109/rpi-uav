#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <sstream>
#include <bitset>
#include <chrono>

#include <uav/math>
#include "uavcore.h"
#include "serial.h"

namespace uav
{

double altitude(double atm)
{
    return 44330 * (1.0 - pow(atm, 0.1903));
}

bool param::operator==(const param& other)
{
    return serialize(*this) == serialize(other);
}

bool param::operator!=(const param& other)
{
    return !(*this == other);
}

bool state::operator==(const state& other)
{
    return serialize(*this) == serialize(other);
}

bool state::operator!=(const state& other)
{
    return !(*this == other);
}

param::bin serialize(const param& p)
{
    return bin(p.freq) + bin(p.mass) + bin(p.pid_gains);
}

state::bin serialize(const state& s)
{
    return bin(s.time) + bin(s.position) + bin(s.attitude) +
        bin(s.targets) + bin(s.motors) + bin(s.error) + bin(s.status);
}

param deserialize(const param::bin& b)
{
    param p;
    size_t rptr = 0;
    auto src = begin(b);
    bin(src, rptr, p.freq);
    bin(src, rptr, p.mass);
    bin(src, rptr, p.pid_gains);
    return p;
}

state deserialize(const state::bin& b)
{
    state s;
    size_t rptr = 0;
    auto src = begin(b);
    bin(src, rptr, s.time);
    bin(src, rptr, s.position);
    bin(src, rptr, s.attitude);
    bin(src, rptr, s.targets);
    bin(src, rptr, s.motors);
    bin(src, rptr, s.error);
    bin(src, rptr, s.status);
    return s;
}

std::string param::header()
{
    return "freq mass { s(0..3) z(0..3) h(0..3) p(0..3) r(0..3) }";
}

std::string state::header(fmt::bitmask_t b)
{
    using namespace std;
    stringstream line;
    line << left;

    if (b & fmt::time)
    {
        line << setw(15) << "time";
    }
    if (b & (fmt::time_full & ~fmt::time))
    {
        line << setw(15) << "t_abs" << setw(15) << "comp_us";
    }
    if (b & fmt::position)
    {
        line << setw(15) << "x" << setw(15) << "y" << setw(15) << "z";
    }
    if (b & fmt::attitude)
    {
        line << setw(15) << "hdg" << setw(15) << "pitch"
            << setw(15) << "roll";
    }
    if (b & fmt::quat)
    {
        line << setw(15) << "qw" << setw(15) << "qx"
             << setw(15) << "qy" << setw(15) << "qz";
    }
    if (b & fmt::targets)
    {
        line << setw(15) << "tx" << setw(15) << "ty"
             << setw(15) << "tz" << setw(15) << "th";
    }
    if (b & fmt::motors)
    {
        line << setw(15) << "m1" << setw(15) << "m2"
             << setw(15) << "m3" << setw(15) << "m4";
    }
    if (b & fmt::error)
    {
        line << setw(20) << "err";
    }
    if (b & fmt::status)
    {
        line << setw(15) << "status";
    }
    return line.str();
}

std::string to_string(const param& prm)
{
    std::stringstream line;

    line << (int) prm.freq << " ";
    line << prm.mass << " ";

    line << "[ ";
    for (double e : prm.pid_gains)
        line << e << " ";
    line.seekp(-1, line.cur);
    line << "]";

    return line.str();
}

std::string to_string(const state& it, fmt::bitmask_t b)
{
    using namespace std;

    stringstream line;
    line << left << fixed << setprecision(3);

    if (b & fmt::time)
    {
        line << setw(15) << it.time[0]/1000.0;
    }
    if (b & (fmt::time_full & ~fmt::time))
    {
        line << setw(15) << it.time[1]/1000.0
             << setw(15) << it.time[2]/1000.0;
    }
    if (b & fmt::position)
    {
        for (auto e : it.position)
            line << setw(15) << e;
    }
    if (b & fmt::attitude)
    {
        for (double e : it.attitude)
            line << setw(15) << e;
    }
    if (b & fmt::quat)
    {
        imu::Quaternion q;
        imu::Vector<3> euler(it.attitude[0],
                it.attitude[1], it.attitude[2]);
        q.fromMatrix(euler2matrix(euler));
        line << setw(15) << q.w() << setw(15) << q.x()
             << setw(15) << q.y() << setw(15) << q.z();
    }
    if (b & fmt::targets)
    {
        for (auto e : it.targets)
            line << setw(15) << e;
    }
    if (b & fmt::motors)
    {
        for (auto e : it.motors)
            line << setw(15) << e;
    }
    if (b & fmt::error)
    {
        line << setw(20) << std::bitset<16>(it.error);
    }
    if (b & fmt::status)
    {
        line << (int) it.status << "-";
        const char *str[] = {"NULL_STATUS", "ALIGNING", "NO_VEL",
            "POS_SEEK", "POS_HOLD", "UPSIDE_DOWN"};
        line << str[it.status];
    }
    return line.str();
}

} // namespace uav
