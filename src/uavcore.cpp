#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <sstream>
#include <bitset>
#include <chrono>
#include <cassert>

#include <uavcore.h>
#include <freebody.h>

bool uav::param::operator==(const param& other)
{
    return serialize(*this) == serialize(other);
}

bool uav::param::operator!=(const param& other)
{
    return !(*this == other);
}

bool uav::state::operator==(const state& other)
{
    return serialize(*this) == serialize(other);
}

bool uav::state::operator!=(const state& other)
{
    return !(*this == other);
}

uav::param::bin uav::serialize(const param& p)
{
    return bin(p.freq) + bin(p.spidg) + bin(p.zpidg) +
        bin(p.hpidg) + bin(p.ppidg) + bin(p.rpidg) +
        bin(p.tilt95) + bin(p.maxtilt) + bin(p.mg);
}

uav::state::bin uav::serialize(const state& s)
{
    return bin(s.t) + bin(s.t_abs) + bin(s.comptime) +
        bin(s.pres) + bin(s.pos) + bin(s.calib) +
        bin(s.targets) + bin(s.pidov) + bin(s.motors) +
        bin(s.err) + bin(s.status);
}

uav::param uav::deserialize(const param::bin& b)
{
    param p;
    size_t rptr = 0;
    auto src = begin(b);
    bin(src, rptr, p.freq);
    bin(src, rptr, p.spidg);
    bin(src, rptr, p.zpidg);
    bin(src, rptr, p.hpidg);
    bin(src, rptr, p.ppidg);
    bin(src, rptr, p.rpidg);
    bin(src, rptr, p.tilt95);
    bin(src, rptr, p.maxtilt);
    bin(src, rptr, p.mg);
    return p;
}

uav::state uav::deserialize(const state::bin& b)
{
    state s;
    size_t rptr = 0;
    auto src = begin(b);
    bin(src, rptr, s.t);
    bin(src, rptr, s.t_abs);
    bin(src, rptr, s.comptime);
    bin(src, rptr, s.pres);
    bin(src, rptr, s.pos);
    bin(src, rptr, s.calib);
    bin(src, rptr, s.targets);
    bin(src, rptr, s.pidov);
    bin(src, rptr, s.motors);
    bin(src, rptr, s.err);
    bin(src, rptr, s.status);
    return s;
}

std::string uav::param::header()
{
    return "freq spidg(0..3) zpidg(0..3) hpidg(0..3) ppidg(0..3) rpidg(0..3) "
           "tilt95 maxtilt mg";
}

std::string uav::state::header(fmt::bitmask_t b)
{
    using namespace std;
    stringstream line;
    line << left;

    if (b & fmt::time)
    {
        line << setw(15) << "time";
    }
    if (b & fmt::time_full)
    {
        line << setw(15) << "t_abs" << setw(15) << "comp_us";
    }
    if (b & fmt::pressure)
    {
        line << setw(15) << "p1" << setw(15) << "p2";
    }
    if (b & fmt::position)
    {
        line << setw(15) << "x" << setw(15) << "y" << setw(15) << "z";
    }
    if (b & fmt::attitude)
    {
        line << setw(15) << "α" << setw(15) << "β" << setw(15) << "γ";
    }
    if (b & fmt::calib)
    {
        line << setw(15) << "cal";
    }
    if (b & fmt::quat)
    {
        line << setw(15) << "qw" << setw(15) << "qx"
             << setw(15) << "qy" << setw(15) << "qz";
    }
    if (b & fmt::targets)
    {
        line << setw(15) << "tx" << setw(15) << "ty" << setw(15) << "tz"
             << setw(15) << "tα" << setw(15) << "tβ" << setw(15) << "tγ";
    }
    if (b & fmt::pid)
    {
        line << setw(15) << "xov" << setw(15) << "yov" << setw(15) << "zov"
             << setw(15) << "αov" << setw(15) << "βov" << setw(15) << "γov";
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

std::string uav::to_string(const param& prm)
{
    std::stringstream line;

    line << (int) prm.freq << " ";

    line << "[ ";
    for (int i = 0; i < 4; i++)
        line << prm.spidg[i] << " ";
    line << "] [ ";
    for (int i = 0; i < 4; i++)
        line << prm.zpidg[i] << " ";
    line << "] [ ";
    for (int i = 0; i < 4; i++)
        line << prm.hpidg[i] << " ";
    line << "] [ ";
    for (int i = 0; i < 4; i++)
        line << prm.ppidg[i] << " ";
    line << "] [ ";
    for (int i = 0; i < 4; i++)
        line << prm.rpidg[i] << " ";
    line << "] ";
    line << prm.tilt95 << " ";
    line << prm.maxtilt << " ";
    line << prm.mg << " ";

    std::string str = line.str();
    str.pop_back();
    return str;
}

std::string uav::to_string(const state& it, fmt::bitmask_t b)
{
    using namespace std;

    stringstream line;
    line << left << fixed << setprecision(3);

    if (b & fmt::time)
    {
        line << setw(15) << it.t/1000.0;
    }
    if (b & fmt::time_full)
    {
        line << setw(15) << it.t_abs/1000.0
             << setw(15) << it.comptime/1000.0;
    }
    if (b & fmt::pressure)
    {
        line << setw(15) << it.pres[0] << setw(15) << it.pres[1];
    }
    if (b & fmt::position)
    {
        for (int i = 0; i < 3; i++)
        line << setw(15) << it.pos[i];
    }
    if (b & fmt::attitude)
    {
        for (int i = 3; i < 6; i++)
        line << setw(15) << it.pos[i];
    }
    if (b & fmt::calib)
    {
        line << hex << setw(15) << (int) it.calib << dec;
    }
    if (b & fmt::quat)
    {
        imu::Quaternion q;
        imu::Vector<3> euler(it.pos[3], it.pos[4], it.pos[5]);
        euler.toRadians();
        q.fromMatrix(uav::euler2matrix(euler));
        line << setw(15) << q.w() << setw(15) << q.x()
             << setw(15) << q.y() << setw(15) << q.z();
    }
    if (b & fmt::targets)
    {
        for (int i = 0; i < 6; i++)
        line << setw(15) << it.targets[i];
    }
    if (b & fmt::pid)
    {
        for (int i = 0; i < 6; i++)
        line << setw(15) << it.pidov[i];
    }
    if (b & fmt::motors)
    {
        for (int i = 0; i < 4; i++)
        line << setw(15) << it.motors[i];
    }
    if (b & fmt::error)
    {
        line << setw(20) << std::bitset<16>(it.err);
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

std::string uav::timestamp()
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

uav::param paramlog;
std::deque<uav::state> statelog;
std::deque<std::string> textlog;

uav::logstream uav::debug("DEBUG"), uav::info("INFO"), uav::error("ERROR");

uav::logstream::logstream(const std::string& streamname) :
    name(streamname) { }

uav::logstream& uav::logstream::operator << (std::ostream& (*)(std::ostream& os))
{
    textlog.push_back("[" + name + "]\t" + timestamp() + ss.str());
    ss.str(std::string());
    return *this;
}

void uav::logstream::operator () (const std::string& s)
{
    textlog.push_back("[" + name + "]\t" + timestamp() + s);
}

void uav::reset()
{
    paramlog = {0};
    statelog.clear();
    textlog.clear();
}

void uav::include(uav::param p)
{
    paramlog = p;
}

void uav::include(uav::state s)
{
    statelog.push_back(s);
}

void uav::flush()
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
