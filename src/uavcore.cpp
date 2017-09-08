#include <iomanip>
#include <string>
#include <cstring>
#include <sstream>
#include <bitset>
#include <chrono>

#include <uavcore.h>

uav::param uav::param::zero()
{
    static_assert(param_fields == 23, "CHECK_ASSUMED_SIZE_OF_PARAM");

    param p{f1hz, 0, 0, {0, 0, 0, 0}, {0, 0, 0, 0},
        {0, 0, 0, 0}, {0, 0, 0, 0}, 0, 0, 0, 0};

    return p;
}

uav::param_bin uav::to_binary(const param& p)
{
    static_assert(param_fields == 23, "CHECK_ASSUMED_SIZE_OF_PARAM");
    static_assert(param_size == 171, "CHECK_ASSUMED_SIZE_OF_PARAM"); 

    param_bin b;
    b.fill(0);
    uint8_t *wptr = b.begin();
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

uav::state_bin uav::to_binary(const state& s)
{
    static_assert(state_fields == 24, "CHECK_ASSUMED_SIZE_OF_STATE");
    static_assert(state_size == 79, "CHECK_ASSUMED_SIZE_OF_STATE"); 

    state_bin b;
    b.fill(0);
    uint8_t *wptr = b.begin();
    memcpy(wptr, &s.t, sizeof(s.t));
    wptr += sizeof(s.t);
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

uav::param uav::from_binary(const param_bin& b)
{
    static_assert(param_fields == 23, "CHECK_ASSUMED_SIZE_OF_PARAM");
    static_assert(param_size == 171, "CHECK_ASSUMED_SIZE_OF_PARAM"); 

    param p;
    const uint8_t *rptr = b.begin();
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

uav::state uav::from_binary(const state_bin& b)
{
    static_assert(state_fields == 24, "CHECK_ASSUMED_SIZE_OF_STATE");
    static_assert(state_size == 79, "CHECK_ASSUMED_SIZE_OF_STATE");

    state s;
    const uint8_t *rptr = b.begin();
    memcpy(&s.t, rptr, sizeof(s.t));
    rptr += sizeof(s.t);
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

std::string uav::pheader()
{
    return "freq p1h p2h zpidg(0..3) hpidg(0..3) ppidg(0..3) rpidg(0..3) "
           "gz_rc gz_wam maxmrate mg";
}

std::string uav::sheader(uint64_t mask)
{
    static_assert(state_fields == 24, "CHECK_ASSUMED_SIZE_OF_STATE");

    using namespace std;

    std::bitset<uav::state_size> b(mask);
    stringstream line;
    line << left;

    if (b[0]) line << setw(10) << "time";
    if (b[1]) line << setw(10) << "comp";

    if (b[2]) line << setw(9) << "t1";
    if (b[3]) line << setw(9) << "t2";
    if (b[4]) line << setw(12) << "p1";
    if (b[5]) line << setw(12) << "p2";
    if (b[6]) line << setw(9) << "dz";

    if (b[7]) line << setw(9) << "hdg";
    if (b[8]) line << setw(9) << "pitch";
    if (b[9]) line << setw(9) << "roll";
    if (b[10]) line << setw(4) << "cal";

    if (b[11]) line << setw(5) << "tz";
    if (b[12]) line << setw(5) << "th";
    if (b[13]) line << setw(5) << "tp";
    if (b[14]) line << setw(5) << "tr";

    if (b[15]) line << setw(12) << "zov";
    if (b[16]) line << setw(12) << "hov";
    if (b[17]) line << setw(12) << "pov";
    if (b[18]) line << setw(12) << "rov";

    for (int i = 0; i < 4; i++)
        if (b[19 + i])
            line << setw(4) << "m" + std::to_string(i + 1);

    if (b[23]) line << setw(10) << "err";

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

std::string uav::to_string(const state& it, uint64_t mask)
{ 
    static_assert(state_fields == 24, "CHECK_ASSUMED_SIZE_OF_STATE");

    using namespace std;

    std::bitset<state_fields> b(mask);
    stringstream line;
    line << left;
    line.setf(ios::fixed);

    if (b[0]) line << setw(10) << setprecision(3) << it.t/1000.0;
    if (b[1]) line << setw(10) << it.comptime;
    if (b[2]) line << setw(9) << it.temp[0];
    if (b[3]) line << setw(9) << it.temp[1];
    if (b[4]) line << setw(12) << it.pres[0];
    if (b[5]) line << setw(12) << it.pres[1];
    if (b[6]) line << setw(9) << it.dz;

    if (b[7]) line << setw(9) << it.h;
    if (b[8]) line << setw(9) << it.p;
    if (b[9]) line << setw(9) << it.r;
    if (b[10]) line << hex << setw(4) << (int) it.calib << dec;

    if (b[11]) line << setw(5) << (int) it.tz;
    if (b[12]) line << setw(5) << (int) it.th;
    if (b[13]) line << setw(5) << (int) it.tp;
    if (b[14]) line << setw(5) << (int) it.tr;

    if (b[15]) line << setw(12) << it.zov;
    if (b[16]) line << setw(12) << it.hov;
    if (b[17]) line << setw(12) << it.pov;
    if (b[18]) line << setw(12) << it.rov;

    for (int i = 0; i < 4; i++)
        if (b[19 + i]) line << setw(4) << (int) it.motors[i];

    if (b[23]) line << setw(10) << std::bitset<16>(it.err);

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

    uint64_t sec = time/1000000;
    uint64_t milli = (time/1000) % 1000;
    uint64_t micro = time % 1000;

    s << "[" << setw(7) << sec << "."
        << setw(3) << setfill('0') << milli << "'"
        << setw(3) << setfill('0') << micro << "] ";
    return s.str();
}

uav::param paramlog;
std::deque<uav::state> statelog;
std::deque<std::string> debuglog, infolog, errorlog;

void uav::reset()
{
    paramlog = param::zero();
    statelog.clear();
    debuglog.clear();
    infolog.clear();
    errorlog.clear();
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
    debuglog.push_back(timestamp() + s);
}

void uav::info(std::string s)
{
    infolog.push_back(timestamp() + s);
}

void uav::error(std::string s)
{
    errorlog.push_back(timestamp() + s);
}

void uav::flush()
{
    std::ofstream debug, info, error,
        data("log/data.bin", std::ios::out | std::ios::binary);
    {
        param_bin pbin = uav::to_binary(paramlog);
        data.write((const char*) pbin.data(), uav::param_size);
    }
    while (!statelog.empty())
    {
        state_bin b = uav::to_binary(statelog.front());
        statelog.pop_front();
        data.write((const char*) b.begin(), uav::state_size);
    }
    if (!debuglog.empty())
        debug.open("log/debug.txt", std::ios::out);
    while (!debuglog.empty())
    {
        debug << debuglog.front() << "\n";
        debuglog.pop_front();
    }
    if (!infolog.empty())
        info.open("log/info.txt", std::ios::out);
    while (!infolog.empty())
    {
        info << infolog.front() << "\n";
        infolog.pop_front();
    }
    if (!errorlog.empty())
        error.open("log/error.txt", std::ios::out);
    while (!errorlog.empty())
    {
        error << errorlog.front() << "\n";
        errorlog.pop_front();
    }
}
