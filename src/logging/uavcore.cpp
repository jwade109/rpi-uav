#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <sstream>
#include <bitset>
#include <chrono>

#include <uav/math>
#include "uavcore.h"

namespace uav
{

angle angle::from_degrees(double degs) { return angle(degs*(M_PI/180), 0); }

angle angle::from_radians(double rads) { return angle(rads, 0); }

angle angle::from_rotations(int rots) { return angle(0, rots); }

angle::angle() : angle(0, 0) { }

angle::angle(double rads, int rots) :
    rotations(rots + std::lround(rads/(2*M_PI))),
    radians(rads - (std::lround(rads/(2*M_PI))*2*M_PI)) { }

angle::angle(const angle& a) : rotations(a.rot()), radians(a.rad()) { }

double angle::rad() const { return radians; }

double angle::deg() const { return (180/M_PI) * radians; }

int angle::rot() const { return rotations; }

angle angle::operator - ()
{
    return angle(-radians, -rotations);
}

angle& angle::operator = (const angle& a)
{
    radians = a.rad();
    rotations = a.rot();
    return *this;
}

angle angle::operator + (const angle& a) const
{
    return angle(radians + a.rad(), rotations + a.rot());
}

angle angle::operator - (const angle& a) const
{
    return angle(radians - a.rad(), rotations - a.rot());
}

angle& angle::operator += (const angle& a)
{
    return (*this = *this + a);
}

angle& angle::operator -= (const angle& a)
{
    return (*this = *this - a);
}

angle& angle::operator = (double rads)
{
    return *this = angle(rads);
}

angle angle::operator * (double scalar) const
{
    return angle((radians + 2*M_PI * rotations) * scalar);
}

angle angle::operator / (double divisor) const
{
    return *this * (1/divisor);
}

angle& angle::operator *= (double scalar)
{
    return (*this = *this * scalar);
}

angle& angle::operator /= (double divisor)
{
    return (*this = *this / divisor);
}

double angle::operator / (const angle& a) const
{
    return (radians + 2*M_PI*rotations) / (a.rad() + 2*M_PI*a.rot());
}

bool angle::operator == (const angle& a) const
{
    return radians == a.rad() && rotations == a.rot();
}

bool angle::operator != (const angle& a) const
{
    return !(*this == a);
}

bool angle::operator > (const angle& a) const
{
    return rotations*2*M_PI + radians - a.rot()*2*M_PI - a.rad() > 0;
}

bool angle::operator < (const angle& a) const
{
    return rotations*2*M_PI + radians - a.rot()*2*M_PI - a.rad() < 0;
}

bool angle::operator >= (const angle& a) const
{
    return (*this == a) || (*this > a);
}

bool angle::operator <= (const angle& a) const
{
    return (*this == a) || (*this < a);
}

angle::operator double () const
{
    return radians + rotations*2*M_PI;;
}

angle target_azimuth(angle current, angle desired)
{
    angle diff((desired - current).rad());
    return current + diff;
}

std::ostream& operator << (std::ostream& os, const angle& a)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << a.rot() << "*" << a.deg();
    return os << ss.str();
}

namespace angle_literals
{

angle operator "" _rad(unsigned long long radians)
{
    return angle(radians);
}

angle operator "" _deg(unsigned long long degrees)
{
    return angle(degrees * (M_PI/180));
}

angle operator "" _rad(long double radians)
{
    return angle(radians);
}

angle operator "" _deg(long double degrees)
{
    return angle(degrees * (M_PI/180));
}

} // namespace uav::angle_literals;

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
        for (angle e : it.attitude)
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

std::string timestamp()
{
    using namespace std;
    using namespace std::chrono;
    uint64_t time = duration_cast<microseconds>(steady_clock::now()
                  .time_since_epoch()).count();
    std::stringstream s;
    s.setf(ios::fixed);

    uint64_t sec = (time/1000000);
    uint64_t milli = (time/1000) % 1000;
    uint64_t micro = time % 1000;

    s << "[" << setw(6) << setfill('0') << sec << "."
        << setw(3) << setfill('0') << milli << "."
        << setw(3) << setfill('0') << micro << "] ";
    return s.str();
}

param paramlog;
std::deque<state> statelog;
std::deque<std::string> textlog;

logstream debug("DEBUG"), info("INFO"), error("ERROR");

logstream::logstream(const std::string& streamname) :
    name(streamname) { }

logstream& logstream::operator << (std::ostream& (*)(std::ostream& os))
{
    textlog.push_back("[" + name + "]\t" + timestamp() + ss.str());
    ss.str(std::string());
    return *this;
}

void logstream::operator () (const std::string& s)
{
    textlog.push_back("[" + name + "]\t" + timestamp() + s);
}

void reset()
{
    paramlog = {0};
    statelog.clear();
    textlog.clear();
}

void include(param p)
{
    paramlog = p;
}

void include(state s)
{
    statelog.push_back(s);
}

void flush()
{
    std::ofstream
        text("log/events.txt", std::ios::out),
        data("log/data.bin", std::ios::out | std::ios::binary);

    {
        param::bin b = serialize(paramlog);
        data.write(reinterpret_cast<const char*>(b.data()), param::size);
    }
    while (!statelog.empty())
    {
        state::bin b = serialize(statelog.front());
        statelog.pop_front();
        data.write(reinterpret_cast<const char*>(b.data()), state::size);
    }
    while (!textlog.empty())
    {
        text << textlog.front() << "\n";
        textlog.pop_front();
    }
}

} // namespace uav
