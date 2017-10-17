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
    return bin(p.freq) + bin(p.p1h) + bin(p.p2h) + bin(p.spidg) +
        bin(p.zpidg) + bin(p.hpidg) + bin(p.ppidg) + bin(p.rpidg) +
        bin(p.gz_rc) + bin(p.gz_wam) + bin(p.tilt95) +
        bin(p.maxtilt) + bin(p.mg);
}

uav::state::bin uav::serialize(const state& s)
{
    return bin(s.t) + bin(s.t_abs) + bin(s.comptime) +
        bin(s.temp) + bin(s.pres) + bin(s.pos) +
        bin(s.calib) + bin(s.targets) + bin(s.pidov) +
        bin(s.motors) + bin(s.err) + bin(s.status);
}

uav::param uav::deserialize(const param::bin& b)
{
    param p;
    size_t rptr = 0;
    auto src = begin(b);
    bin(src, rptr, p.freq);
    bin(src, rptr, p.p1h);
    bin(src, rptr, p.p2h);
    bin(src, rptr, p.spidg);
    bin(src, rptr, p.zpidg);
    bin(src, rptr, p.hpidg);
    bin(src, rptr, p.ppidg);
    bin(src, rptr, p.rpidg);
    bin(src, rptr, p.gz_rc);
    bin(src, rptr, p.gz_wam);
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
    bin(src, rptr, s.temp);
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
    return "freq p1h p2h spidg(0..3) zpidg(0..3) hpidg(0..3) ppidg(0..3) rpidg(0..3) "
           "gz_rc gz_wam tilt95 maxtilt mg";
}

std::string uav::state::header(fmt::bitmask_t mask)
{
    using namespace std;

    std::bitset<state::size> b(mask);
    stringstream line;
    line << left;

    int i = 0;
    if (b[i++]) line << setw(15) << "time";
    if (b[i++]) line << setw(15) << "t_abs";
    if (b[i++]) line << setw(15) << "comp_us";

    if (b[i++]) line << setw(15) << "t1";
    if (b[i++]) line << setw(15) << "t2";
    if (b[i++]) line << setw(15) << "p1";
    if (b[i++]) line << setw(15) << "p2";
    if (b[i++]) line << setw(15) << "x";
    if (b[i++]) line << setw(15) << "y";
    if (b[i++]) line << setw(15) << "z";

    if (b[i++]) line << setw(15) << "hdg";
    if (b[i++]) line << setw(15) << "pitch";
    if (b[i++]) line << setw(15) << "roll";
    if (b[i++]) line << setw(15) << "cal";

    if (b[i++]) line << setw(15) << "qw";
    if (b[i++]) line << setw(15) << "qx";
    if (b[i++]) line << setw(15) << "qy";
    if (b[i++]) line << setw(15) << "qz";

    if (b[i++]) line << setw(15) << "tx";
    if (b[i++]) line << setw(15) << "ty";
    if (b[i++]) line << setw(15) << "tz";
    if (b[i++]) line << setw(15) << "th";
    if (b[i++]) line << setw(15) << "tp";
    if (b[i++]) line << setw(15) << "tr";

    if (b[i++]) line << setw(15) << "xov";
    if (b[i++]) line << setw(15) << "yov";
    if (b[i++]) line << setw(15) << "zov";
    if (b[i++]) line << setw(15) << "hov";
    if (b[i++]) line << setw(15) << "pov";
    if (b[i++]) line << setw(15) << "rov";

    if (b[i++]) line << setw(15) << "m1(CW)";
    if (b[i++]) line << setw(15) << "m2(CCW)";
    if (b[i++]) line << setw(15) << "m3(CW)";
    if (b[i++]) line << setw(15) << "m4(CCW)";

    if (b[i++]) line << setw(20) << "err";
    if (b[i++]) line << setw(15) << "status";

    return line.str();
}

std::string uav::to_string(const param& prm)
{
    std::stringstream line;

    line << (int) prm.freq << " ";
    line << prm.p1h << " ";
    line << prm.p2h << " ";

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
    line << prm.gz_rc << " ";
    line << prm.gz_wam << " ";
    line << prm.tilt95 << " ";
    line << prm.maxtilt << " ";
    line << prm.mg << " ";

    std::string str = line.str();
    str.pop_back();
    return str;
}

std::string uav::to_string(const state& it, fmt::bitmask_t mask)
{
    using namespace std;

    std::bitset<64> b(mask);
    stringstream line;
    line << left << fixed << setprecision(3);

    int i = 0;
    if (b[i++]) line << setw(15) << it.t/1000.0;
    if (b[i++]) line << setw(15) << it.t_abs/1000.0;
    if (b[i++]) line << setw(15) << it.comptime/1000.0;
    if (b[i++]) line << setw(15) << it.temp[0];
    if (b[i++]) line << setw(15) << it.temp[1];
    if (b[i++]) line << setw(15) << it.pres[0];
    if (b[i++]) line << setw(15) << it.pres[1];

    for (int j = 0; j < 6; j++)
        if (b[i++]) line << setw(15) << it.pos[j];
    
    if (b[i++]) line << hex << setw(15) << (int) it.calib << dec;

    imu::Quaternion q;
    imu::Vector<3> euler(it.pos[3], it.pos[4], it.pos[5]);
    euler.toRadians();
    q.fromMatrix(uav::euler2matrix(euler));
    if (b[i++]) line << setw(15) << q.w();
    if (b[i++]) line << setw(15) << q.x();
    if (b[i++]) line << setw(15) << q.y();
    if (b[i++]) line << setw(15) << q.z();

    for (int j = 0; j < 6; j++)
        if (b[i++]) line << setw(15) << it.targets[j];

    for (int j = 0; j < 6; j++)
        if (b[i++]) line << setw(15) << it.pidov[j];

    for (int j = 0; j < 4; j++)
        if (b[i++]) line << setw(15) << it.motors[j];

    if (b[i++]) line << setw(20) << std::bitset<16>(it.err);
    if (b[i++])
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

uav::logstream uav::debugstream(uav::debug),
               uav::infostream(uav::info),
               uav::errorstream(uav::error);

uav::logstream::logstream(void (* logfunc) (std::string s)) :
    log(logfunc) { }

uav::logstream& uav::logstream::operator << (std::ostream& (*)(std::ostream& os))
{
    log(ss.str());
    ss.str(std::string());
    return *this;
}

uav::param paramlog;
std::deque<uav::state> statelog;
std::deque<std::string> textlog;

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

void uav::debug(std::string s)
{
    textlog.push_back("[DEBUG] " + timestamp() + s);
}

void uav::info(std::string s)
{
    textlog.push_back("[INFO]  " + timestamp() + s);
}

void uav::error(std::string s)
{
    textlog.push_back("[ERROR] " + timestamp() + s);
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

int uav::tests::uavcore()
{
    param p{ f125hz, 0, 0, { 0.12, 0.3, 0.5, -1 }, { 2.3, 1.4, 0.015, -1 },
        { 0.1, 01.8, 0.02, -1 }, { 0.1, -0.45, 0.02, -1 }, 0.1, 0.65, 500, 41 };

    state s{ 45, 123, 2001, {10132.7, 10100.3}, {45.4, 45.7},
        {0, 0, 0.73, 32.0, 2.3, -1.6}, 0x4f, {1, 4, 23, 3, -12, 102},
        {0.3, -3.4, 0.45F, 0.23, -0.34, 1.45}, {43, 27, 32, 51}, 12 };

    std::cout << to_string(p) << std::endl;
    std::cout << to_string(s, fmt::standard) << std::endl;

    auto pnew = deserialize(serialize(p));
    auto snew = deserialize(serialize(s));

    std::cout << to_string(pnew) << std::endl;
    std::cout << to_string(snew, fmt::standard) << std::endl;

    if (p != pnew) return 1;
    if (s != snew) return 2;

    return 0;
}
