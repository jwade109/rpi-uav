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
    static_assert(param::fields == 23, "THIS FUNCTION MIGHT BE OUT OF DATE");

    return freq == other.freq && p1h == other.p1h && p2h == other.p2h &&
        zpidg[0] == other.zpidg[0] && zpidg[1] == other.zpidg[1] &&
        zpidg[2] == other.zpidg[2] && zpidg[3] == other.zpidg[3] &&
        hpidg[0] == other.hpidg[0] && hpidg[1] == other.hpidg[1] &&
        hpidg[2] == other.hpidg[2] && hpidg[3] == other.hpidg[3] &&
        ppidg[0] == other.ppidg[0] && ppidg[1] == other.ppidg[1] &&
        ppidg[2] == other.ppidg[2] && ppidg[3] == other.ppidg[3] &&
        rpidg[0] == other.rpidg[0] && rpidg[1] == other.rpidg[1] &&
        rpidg[2] == other.rpidg[2] && rpidg[3] == other.rpidg[3] &&
        gz_rc == other.gz_rc && gz_wam == other.gz_wam &&
        maxmrate == other.maxmrate && mg == other.mg;
}

bool uav::param::operator!=(const param& other)
{
    return !(*this == other);
}

bool uav::state::operator==(const state& other)
{
    static_assert(state::fields == 25, "THIS FUNCTION MIGHT BE OUT OF DATE");

    return t == other.t && t_abs == other.t_abs &&
        comptime == other.comptime &&
        pres[0] == other.pres[0] && pres[1] == other.pres[1] &&
        temp[0] == other.temp[0] && temp[1] == other.temp[1] &&
        dz == other.dz && h == other.h && p == other.p &&
        r == other.r && calib == other.calib &&
        tz == other.tz && th == other.th && tp == other.tp &&
        tr == other.tr && zov == other.zov && hov == other.hov &&
        pov == other.pov && rov == other.rov &&
        motors[0] == other.motors[0] && motors[1] == other.motors[1] &&
        motors[2] == other.motors[2] && motors[3] == other.motors[3] &&
        err == other.err;
}

bool uav::state::operator!=(const state& other)
{
    return !(*this == other);
}

uav::param::bin uav::serialize(const param& p)
{
    static_assert(param::fields == 23, "CHECK_ASSUMED_SIZE_OF_PARAM");
    static_assert(param::size == 171, "CHECK_ASSUMED_SIZE_OF_PARAM");

    param::bin b;
    b.fill(0);
    byte *wptr = b.data();
    memcpy(wptr, &p.freq, sizeof(p.freq));
    wptr += sizeof(p.freq);
    memcpy(wptr, &p.p1h, sizeof(p.p1h));
    wptr += sizeof(p.p1h);
    memcpy(wptr, &p.p2h, sizeof(p.p2h));
    wptr += sizeof(p.p2h);
    memcpy(wptr, p.zpidg, sizeof(p.zpidg[0]) * 4);
    wptr += (sizeof(p.zpidg[0]) * 4);
    memcpy(wptr, p.hpidg, sizeof(p.hpidg[0]) * 4);
    wptr += (sizeof(p.hpidg[0]) * 4);
    memcpy(wptr, p.ppidg, sizeof(p.ppidg[0]) * 4);
    wptr += (sizeof(p.ppidg[0]) * 4);
    memcpy(wptr, p.rpidg, sizeof(p.rpidg[0]) * 4);
    wptr += (sizeof(p.rpidg[0]) * 4);
    memcpy(wptr, &p.gz_rc, sizeof(p.gz_rc));
    wptr += sizeof(p.gz_rc);
    memcpy(wptr, &p.gz_wam, sizeof(p.gz_wam));
    wptr += sizeof(p.gz_wam);
    memcpy(wptr, &p.maxmrate, sizeof(p.maxmrate));
    wptr += sizeof(p.maxmrate);
    memcpy(wptr, &p.mg, sizeof(p.mg));
    wptr += sizeof(p.mg);
    return b;
}

uav::state::bin uav::serialize(const state& s)
{
    static_assert(state::fields == 25, "CHECK_ASSUMED_SIZE_OF_STATE");
    static_assert(state::size == 87, "CHECK_ASSUMED_SIZE_OF_STATE");

    state::bin b;
    b.fill(0);
    byte *wptr = b.data();
    memcpy(wptr, &s.t, sizeof(s.t));
    wptr += sizeof(s.t);
    memcpy(wptr, &s.t_abs, sizeof(s.t_abs));
    wptr += sizeof(s.t_abs);
    memcpy(wptr, &s.comptime, sizeof(s.comptime));
    wptr += sizeof(s.comptime);
    memcpy(wptr, s.temp, sizeof(s.temp[0]) * 2);
    wptr += (sizeof(s.temp[0]) * 2);
    memcpy(wptr, s.pres, sizeof(s.pres[0]) * 2);
    wptr += (sizeof(s.pres[0]) * 2);
    memcpy(wptr, &s.dz, sizeof(s.dz));
    wptr += sizeof(s.dz);
    memcpy(wptr, &s.h, sizeof(s.h));
    wptr += sizeof(s.h);
    memcpy(wptr, &s.p, sizeof(s.p));
    wptr += sizeof(s.p);
    memcpy(wptr, &s.r, sizeof(s.r));
    wptr += sizeof(s.r);
    memcpy(wptr, &s.calib, sizeof(s.calib));
    wptr += sizeof(s.calib);
    memcpy(wptr, &s.tz, sizeof(s.tz));
    wptr += sizeof(s.tz);
    memcpy(wptr, &s.th, sizeof(s.th));
    wptr += sizeof(s.th);
    memcpy(wptr, &s.tp, sizeof(s.tp));
    wptr += sizeof(s.tp);
    memcpy(wptr, &s.tr, sizeof(s.tr));
    wptr += sizeof(s.tr);
    memcpy(wptr, &s.zov, sizeof(s.zov));
    wptr += sizeof(s.zov);
    memcpy(wptr, &s.hov, sizeof(s.hov));
    wptr += sizeof(s.hov);
    memcpy(wptr, &s.pov, sizeof(s.pov));
    wptr += sizeof(s.pov);
    memcpy(wptr, &s.rov, sizeof(s.rov));
    wptr += sizeof(s.rov);
    memcpy(wptr, s.motors, sizeof(s.motors[0]) * 4);
    wptr += (sizeof(s.motors[0]) * 4);
    memcpy(wptr, &s.err, sizeof(s.err));
    wptr += sizeof(s.err);
    return b;
}

uav::param uav::deserialize(const param::bin& b)
{
    static_assert(param::fields == 23, "CHECK_ASSUMED_SIZE_OF_PARAM");
    static_assert(param::size == 171, "CHECK_ASSUMED_SIZE_OF_PARAM");

    param p;
    const byte *rptr = b.data();
    memcpy(&p.freq, rptr, sizeof(p.freq));
    rptr += sizeof(p.freq);
    memcpy(&p.p1h, rptr, sizeof(p.p1h));
    rptr += sizeof(p.p1h);
    memcpy(&p.p2h, rptr, sizeof(p.p2h));
    rptr += sizeof(p.p2h);
    memcpy(p.zpidg, rptr, sizeof(p.zpidg[0]) * 4);
    rptr += (sizeof(p.zpidg[0]) * 4);
    memcpy(p.hpidg, rptr, sizeof(p.hpidg[0]) * 4);
    rptr += (sizeof(p.hpidg[0]) * 4);
    memcpy(p.ppidg, rptr, sizeof(p.ppidg[0]) * 4);
    rptr += (sizeof(p.ppidg[0]) * 4);
    memcpy(p.rpidg, rptr, sizeof(p.rpidg[0]) * 4);
    rptr += (sizeof(p.rpidg[0]) * 4);
    memcpy(&p.gz_rc, rptr, sizeof(p.gz_rc));
    rptr += sizeof(p.gz_rc);
    memcpy(&p.gz_wam, rptr, sizeof(p.gz_wam));
    rptr += sizeof(p.gz_wam);
    memcpy(&p.maxmrate, rptr, sizeof(p.maxmrate));
    rptr += sizeof(p.maxmrate);
    memcpy(&p.mg, rptr, sizeof(p.mg));
    rptr += sizeof(p.mg);
    return p;
}

uav::state uav::deserialize(const state::bin& b)
{
    static_assert(state::fields == 25, "CHECK_ASSUMED_SIZE_OF_STATE");
    static_assert(state::size == 87, "CHECK_ASSUMED_SIZE_OF_STATE");

    state s;
    const byte *rptr = b.data();
    memcpy(&s.t, rptr, sizeof(s.t));
    rptr += sizeof(s.t);
    memcpy(&s.t_abs, rptr, sizeof(s.t_abs));
    rptr += sizeof(s.t_abs);
    memcpy(&s.comptime, rptr, sizeof(s.comptime));
    rptr += sizeof(s.comptime);
    memcpy(s.temp, rptr, sizeof(s.temp[0]) * 2);
    rptr += (sizeof(s.temp[0]) * 2);
    memcpy(s.pres, rptr, sizeof(s.pres[0]) * 2);
    rptr += (sizeof(s.pres[0]) * 2);
    memcpy(&s.dz, rptr, sizeof(s.dz));
    rptr += sizeof(s.dz);
    memcpy(&s.h, rptr, sizeof(s.h));
    rptr += sizeof(s.h);
    memcpy(&s.p, rptr, sizeof(s.p));
    rptr += sizeof(s.p);
    memcpy(&s.r, rptr, sizeof(s.r));
    rptr += sizeof(s.r);
    memcpy(&s.calib, rptr, sizeof(s.calib));
    rptr += sizeof(s.calib);
    memcpy(&s.tz, rptr, sizeof(s.tz));
    rptr += sizeof(s.tz);
    memcpy(&s.th, rptr, sizeof(s.th));
    rptr += sizeof(s.th);
    memcpy(&s.tp, rptr, sizeof(s.tp));
    rptr += sizeof(s.tp);
    memcpy(&s.tr, rptr, sizeof(s.tr));
    rptr += sizeof(s.tr);
    memcpy(&s.zov, rptr, sizeof(s.zov));
    rptr += sizeof(s.zov);
    memcpy(&s.hov, rptr, sizeof(s.hov));
    rptr += sizeof(s.hov);
    memcpy(&s.pov, rptr, sizeof(s.pov));
    rptr += sizeof(s.pov);
    memcpy(&s.rov, rptr, sizeof(s.rov));
    rptr += sizeof(s.rov);
    memcpy(s.motors, rptr, sizeof(s.motors[0]) * 4);
    rptr += (sizeof(s.motors[0]) * 4);
    memcpy(&s.err, rptr, sizeof(s.err));
    rptr += sizeof(s.err);
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

    if (b[i++]) line << setw(4) << "m1";
    if (b[i++]) line << setw(4) << "m2";
    if (b[i++]) line << setw(4) << "m3";
    if (b[i++]) line << setw(4) << "m4";

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

    int i = 0;
    if (b[i++]) line << setw(10) << setprecision(3) << it.t/1000.0;
    if (b[i++]) line << setw(15) << setprecision(3) << it.t_abs/1000.0;
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

    if (b[i++]) line << setw(4) << (int) it.motors[0];
    if (b[i++]) line << setw(4) << (int) it.motors[1];
    if (b[i++]) line << setw(4) << (int) it.motors[2];
    if (b[i++]) line << setw(4) << (int) it.motors[3];

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
