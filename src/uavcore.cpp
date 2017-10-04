#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <sstream>
#include <bitset>
#include <chrono>
#include <cassert>

#include "uavcore.h"

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
    return bin(p.freq) + bin(p.p1h) + bin(p.p2h) + bin(p.zpidg) + 
        bin(p.hpidg) + bin(p.ppidg) + bin(p.rpidg) + bin(p.gz_rc) + 
        bin(p.gz_wam) + bin(p.maxmrate) + bin(p.mg);
}

uav::state::bin uav::serialize(const state& s)
{
    return bin(s.t) + bin(s.t_abs) + bin(s.comptime) +
        bin(s.temp) + bin(s.pres) + bin(s.dz) + bin(s.h) +
        bin(s.p) + bin(s.r) + bin(s.calib) + bin(s.tz) +
        bin(s.th) + bin(s.tp) + bin(s.tr) + bin(s.zov) +
        bin(s.hov) + bin(s.pov) + bin(s.rov) + bin(s.motors) +
        bin(s.err);
}

uav::param uav::deserialize(const param::bin& b)
{
    param p;
    size_t rptr(0);
    auto src = begin(b);
    bin(src, rptr, p.freq);
    bin(src, rptr, p.p1h);
    bin(src, rptr, p.p2h);
    bin(src, rptr, p.zpidg);
    for (int i = 0; i < 4; i++)
        bin(src, rptr, p.hpidg[i]);
    for (int i = 0; i < 4; i++)
        bin(src, rptr, p.ppidg[i]);
    for (int i = 0; i < 4; i++)
        bin(src, rptr, p.rpidg[i]);
    bin(src, rptr, p.gz_rc);
    bin(src, rptr, p.gz_wam);
    bin(src, rptr, p.maxmrate);
    bin(src, rptr, p.mg);
    return p;
}

uav::state uav::deserialize(const state::bin& b)
{
    state s;
    size_t rptr(0);
    const uint8_t* src = begin(b);
    bin(src, rptr, s.t);
    bin(src, rptr, s.t_abs);
    bin(src, rptr, s.comptime);
    bin(src, rptr, s.temp[0]);
    bin(src, rptr, s.temp[1]);
    bin(src, rptr, s.pres[0]);
    bin(src, rptr, s.pres[1]);
    bin(src, rptr, s.dz);
    bin(src, rptr, s.h);
    bin(src, rptr, s.p);
    bin(src, rptr, s.r);
    bin(src, rptr, s.calib);
    bin(src, rptr, s.tz);
    bin(src, rptr, s.th);
    bin(src, rptr, s.tp);
    bin(src, rptr, s.tr);
    bin(src, rptr, s.zov);
    bin(src, rptr, s.hov);
    bin(src, rptr, s.pov);
    bin(src, rptr, s.rov);
    for (int i = 0; i < 4; i++)
        bin(src, rptr, s.motors[i]);
    bin(src, rptr, s.err);
    return s;
}

std::string uav::param::header()
{
    return "freq p1h p2h zpidg(0..3) hpidg(0..3) ppidg(0..3) rpidg(0..3) "
           "gz_rc gz_wam maxmrate mg";
}

std::string uav::state::header(fmt::bitmask_t mask)
{
    static_assert(state::fields == 25, "CHECK_ASSUMED_SIZE_OF_STATE");

    using namespace std;

    std::bitset<state::size> b(mask);
    stringstream line;
    line << left;

    int i = 0;
    if (b[i++]) line << setw(10) << "time";
    if (b[i++]) line << setw(15) << "t_abs";
    if (b[i++]) line << setw(10) << "comp";

    if (b[i++]) line << setw(9) << "t1";
    if (b[i++]) line << setw(9) << "t2";
    if (b[i++]) line << setw(12) << "p1";
    if (b[i++]) line << setw(12) << "p2";
    if (b[i++]) line << setw(9) << "dz";

    if (b[i++]) line << setw(9) << "hdg";
    if (b[i++]) line << setw(9) << "pitch";
    if (b[i++]) line << setw(9) << "roll";
    if (b[i++]) line << setw(4) << "cal";

    if (b[i++]) line << setw(5) << "tz";
    if (b[i++]) line << setw(5) << "th";
    if (b[i++]) line << setw(5) << "tp";
    if (b[i++]) line << setw(5) << "tr";

    if (b[i++]) line << setw(12) << "zov";
    if (b[i++]) line << setw(12) << "hov";
    if (b[i++]) line << setw(12) << "pov";
    if (b[i++]) line << setw(12) << "rov";

    if (b[i++]) line << setw(9) << "m1(CW)";
    if (b[i++]) line << setw(9) << "m2(CCW)";
    if (b[i++]) line << setw(9) << "m3(CW)";
    if (b[i++]) line << setw(9) << "m4(CCW)";

    if (b[i++]) line << setw(10) << "err";

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
    line << prm.maxmrate << " ";
    line << prm.mg << " ";

    std::string str = line.str();
    str.pop_back();
    return str;
}

std::string uav::to_string(const state& it, fmt::bitmask_t mask)
{
    static_assert(state::fields == 25, "CHECK_ASSUMED_SIZE_OF_STATE");

    using namespace std;

    std::bitset<state::fields> b(mask);
    stringstream line;
    line << left;
    line.setf(ios::fixed);

    line << setprecision(3);

    int i = 0;
    if (b[i++]) line << setw(10) << it.t/1000.0;
    if (b[i++]) line << setw(15) << it.t_abs/1000.0;
    if (b[i++]) line << setw(10) << it.comptime;
    if (b[i++]) line << setw(9) << it.temp[0];
    if (b[i++]) line << setw(9) << it.temp[1];
    if (b[i++]) line << setw(12) << it.pres[0];
    if (b[i++]) line << setw(12) << it.pres[1];
    if (b[i++]) line << setw(9) << it.dz;

    if (b[i++]) line << setw(9) << it.h;
    if (b[i++]) line << setw(9) << it.p;
    if (b[i++]) line << setw(9) << it.r;
    if (b[i++]) line << hex << setw(4) << (int) it.calib << dec;

    if (b[i++]) line << setw(5) << (int) it.tz;
    if (b[i++]) line << setw(5) << (int) it.th;
    if (b[i++]) line << setw(5) << (int) it.tp;
    if (b[i++]) line << setw(5) << (int) it.tr;

    if (b[i++]) line << setw(12) << it.zov;
    if (b[i++]) line << setw(12) << it.hov;
    if (b[i++]) line << setw(12) << it.pov;
    if (b[i++]) line << setw(12) << it.rov;

    if (b[i++]) line << setw(9) << it.motors[0];
    if (b[i++]) line << setw(9) << it.motors[1];
    if (b[i++]) line << setw(9) << it.motors[2];
    if (b[i++]) line << setw(9) << it.motors[3];

    if (b[i++]) line << setw(10) << std::bitset<16>(it.err);

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
        << setw(3) << setfill('0') << micro << "]   ";
    return s.str();
}

uav::param paramlog;
std::deque<uav::state> statelog;
std::deque<std::string> textlog;

void uav::reset()
{
    paramlog = { 0 };
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
    textlog.push_back("[INFO ] " + timestamp() + s);
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

    state s{ 45, 123, 2001, 10132.7F, 10100.3F, 45.4F, 45.7F, 0.73F, 32.0F, 2.3F, -1.6F,
        0x4f, 23, 3, -12, 102, 0.45F, 0.23F, -0.34F, 1.45F, 43, 27, 32, 51, 12 };

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
