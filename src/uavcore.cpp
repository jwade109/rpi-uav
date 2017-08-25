#include <iomanip>
#include <string>
#include <cstring>
#include <sstream>
#include <bitset>

#include <uavcore.h>

namespace uav
{
    int to_buffer(uav::Param& prm, char buffer[paramlen])
    {
        int wptr = 0;
        memcpy(buffer, &prm.freq, sizeof(prm.freq));
        wptr += sizeof(prm.freq);
        memcpy(buffer + wptr, &prm.p1h, sizeof(prm.p1h));
        wptr += sizeof(prm.p1h);
        memcpy(buffer + wptr, &prm.p2h, sizeof(prm.p2h));
        wptr += sizeof(prm.p2h);
        memcpy(buffer + wptr, prm.zpidg, sizeof(prm.zpidg[0]) * 4);
        wptr += (sizeof(prm.zpidg[0]) * 4);
        memcpy(buffer + wptr, prm.hpidg, sizeof(prm.hpidg[0]) * 4);
        wptr += (sizeof(prm.hpidg[0]) * 4);
        memcpy(buffer + wptr, prm.ppidg, sizeof(prm.ppidg[0]) * 4);
        wptr += (sizeof(prm.ppidg[0]) * 4);
        memcpy(buffer + wptr, prm.rpidg, sizeof(prm.rpidg[0]) * 4);
        wptr += (sizeof(prm.rpidg[0]) * 4);
        memcpy(buffer + wptr, &prm.gz_rc, sizeof(prm.gz_rc));
        wptr += sizeof(prm.gz_rc);
        memcpy(buffer + wptr, &prm.gz_wam, sizeof(prm.gz_wam));
        wptr += sizeof(prm.gz_wam);
        memcpy(buffer + wptr, &prm.maxmrate, sizeof(prm.maxmrate));
        wptr += sizeof(prm.maxmrate);
        memcpy(buffer + wptr, &prm.mg, sizeof(prm.mg));
        wptr += sizeof(prm.mg);
        return wptr;
    }

    int to_buffer(uav::State& it, char buffer[statelen])
    {
        int wptr = 0;
        memcpy(buffer, &it.t, sizeof(it.t));
        wptr += sizeof(it.t);
        memcpy(buffer + wptr, it.temp, sizeof(it.temp[0]) * 2);
        wptr += (sizeof(it.temp[0]) * 2);
        memcpy(buffer + wptr, it.pres, sizeof(it.pres[0]) * 2);
        wptr += (sizeof(it.pres[0]) * 2);
        memcpy(buffer + wptr, &it.dz, sizeof(it.dz));
        wptr += sizeof(it.dz);
        memcpy(buffer + wptr, &it.h, sizeof(it.h));
        wptr += sizeof(it.h);
        memcpy(buffer + wptr, &it.p, sizeof(it.p));
        wptr += sizeof(it.p);
        memcpy(buffer + wptr, &it.r, sizeof(it.r));
        wptr += sizeof(it.r);
        memcpy(buffer + wptr, &it.calib, sizeof(it.calib));
        wptr += sizeof(it.calib);
        memcpy(buffer + wptr, &it.tz, sizeof(it.tz));
        wptr += sizeof(it.tz);
        memcpy(buffer + wptr, &it.th, sizeof(it.th));
        wptr += sizeof(it.th);
        memcpy(buffer + wptr, &it.tp, sizeof(it.tp));
        wptr += sizeof(it.tp);
        memcpy(buffer + wptr, &it.tr, sizeof(it.tr));
        wptr += sizeof(it.tr);
        memcpy(buffer + wptr, &it.zov, sizeof(it.zov));
        wptr += sizeof(it.zov);
        memcpy(buffer + wptr, &it.hov, sizeof(it.hov));
        wptr += sizeof(it.hov);
        memcpy(buffer + wptr, &it.pov, sizeof(it.pov));
        wptr += sizeof(it.pov);
        memcpy(buffer + wptr, &it.rov, sizeof(it.rov));
        wptr += sizeof(it.rov);
        memcpy(buffer + wptr, it.motors, sizeof(it.motors[0]) * 4);
        wptr += (sizeof(it.motors[0]) * 4);
        memcpy(buffer + wptr, &it.err, sizeof(it.err));
        wptr += sizeof(it.err);
        return wptr;
    }

    int from_buffer(uav::Param& prm, char buffer[paramlen])
    {
        int rptr = 0;
        memcpy(&prm.freq, buffer, sizeof(prm.freq));
        rptr += sizeof(prm.freq);
        memcpy(&prm.p1h, buffer + rptr, sizeof(prm.p1h));
        rptr += sizeof(prm.p1h);
        memcpy(&prm.p2h, buffer + rptr, sizeof(prm.p2h));
        rptr += sizeof(prm.p2h);
        memcpy(prm.zpidg, buffer + rptr, sizeof(prm.zpidg[0]) * 4);
        rptr += (sizeof(prm.zpidg[0]) * 4);
        memcpy(prm.hpidg, buffer + rptr, sizeof(prm.hpidg[0]) * 4);
        rptr += (sizeof(prm.hpidg[0]) * 4);
        memcpy(prm.ppidg, buffer + rptr, sizeof(prm.ppidg[0]) * 4);
        rptr += (sizeof(prm.ppidg[0]) * 4);
        memcpy(prm.rpidg, buffer + rptr, sizeof(prm.rpidg[0]) * 4);
        rptr += (sizeof(prm.rpidg[0]) * 4);
        memcpy(&prm.gz_rc, buffer + rptr, sizeof(prm.gz_rc));
        rptr += sizeof(prm.gz_rc);
        memcpy(&prm.gz_wam, buffer + rptr, sizeof(prm.gz_wam));
        rptr += sizeof(prm.gz_wam);
        memcpy(&prm.maxmrate, buffer + rptr, sizeof(prm.maxmrate));
        rptr += sizeof(prm.maxmrate);
        memcpy(&prm.mg, buffer + rptr, sizeof(prm.mg));
        rptr += sizeof(prm.mg);
        return rptr;
    }

    int from_buffer(uav::State& it, char buffer[statelen])
    {
        int rptr = 0;
        memcpy(&it.t, buffer, sizeof(it.t));
        rptr += sizeof(it.t);
        memcpy(it.temp, buffer + rptr, sizeof(it.temp[0]) * 2);
        rptr += (sizeof(it.temp[0]) * 2);
        memcpy(it.pres, buffer + rptr, sizeof(it.pres[0]) * 2);
        rptr += (sizeof(it.pres[0]) * 2);
        memcpy(&it.dz, buffer + rptr, sizeof(it.dz));
        rptr += sizeof(it.dz);
        memcpy(&it.h, buffer + rptr, sizeof(it.h));
        rptr += sizeof(it.h);
        memcpy(&it.p, buffer + rptr, sizeof(it.p));
        rptr += sizeof(it.p);
        memcpy(&it.r, buffer + rptr, sizeof(it.r));
        rptr += sizeof(it.r);
        memcpy(&it.calib, buffer + rptr, sizeof(it.calib));
        rptr += sizeof(it.calib);
        memcpy(&it.tz, buffer + rptr, sizeof(it.tz));
        rptr += sizeof(it.tz);
        memcpy(&it.th, buffer + rptr, sizeof(it.th));
        rptr += sizeof(it.th);
        memcpy(&it.tp, buffer + rptr, sizeof(it.tp));
        rptr += sizeof(it.tp);
        memcpy(&it.tr, buffer + rptr, sizeof(it.tr));
        rptr += sizeof(it.tr);
        memcpy(&it.zov, buffer + rptr, sizeof(it.zov));
        rptr += sizeof(it.zov);
        memcpy(&it.hov, buffer + rptr, sizeof(it.hov));
        rptr += sizeof(it.hov);
        memcpy(&it.pov, buffer + rptr, sizeof(it.pov));
        rptr += sizeof(it.pov);
        memcpy(&it.rov, buffer + rptr, sizeof(it.rov));
        rptr += sizeof(it.rov);
        memcpy(it.motors, buffer + rptr, sizeof(it.motors[0]) * 4);
        rptr += (sizeof(it.motors[0]) * 4);
        memcpy(&it.err, buffer + rptr, sizeof(it.err));
        rptr += sizeof(it.err);
        return rptr;
    }

    std::string pheader()
    {
        return "freq p1h p2h zpidg(0..3) hpidg(0..3) ppidg(0..3) rpidg(0..3) "
               "gz_rc gz_wam maxmrate mg";
    }

    std::string sheader(uint64_t mask)
    {
        using namespace std;

        std::bitset<uav::statelen> b(mask);
        stringstream line;
        line << left;

        if (b[0]) line << setw(10) << "time";

        if (b[1]) line << setw(9) << "t1";
        if (b[2]) line << setw(9) << "t2";
        if (b[3]) line << setw(12) << "p1";
        if (b[4]) line << setw(12) << "p2";
        if (b[5]) line << setw(9) << "dz";

        if (b[6]) line << setw(9) << "hdg";
        if (b[7]) line << setw(9) << "pitch";
        if (b[8]) line << setw(9) << "roll";
        if (b[9]) line << setw(4) << "cal";

        if (b[10]) line << setw(5) << "tz";
        if (b[11]) line << setw(5) << "th";
        if (b[12]) line << setw(5) << "tp";
        if (b[13]) line << setw(5) << "tr";

        if (b[14]) line << setw(12) << "zov";
        if (b[15]) line << setw(12) << "hov";
        if (b[16]) line << setw(12) << "pov";
        if (b[17]) line << setw(12) << "rov";

        for (int i = 0; i < 4; i++)
            if (b[18 + i])
                line << setw(4) << "m" + std::to_string(i + 1);

        if (b[22]) line << setw(10) << "err";

        return line.str();

    }

    std::string to_string(uav::Param prm)
    {
        std::stringstream line;

        line << (int) prm.freq << " ";
        line << prm.p1h << " ";
        line << prm.p2h << " ";

        for (int i = 0; i < 4; i++)
            line << prm.zpidg[i] << " ";

        for (int i = 0; i < 4; i++)
            line << prm.hpidg[i] << " ";

        for (int i = 0; i < 4; i++)
            line << prm.ppidg[i] << " ";

        for (int i = 0; i < 4; i++)
            line << prm.rpidg[i] << " ";

        line << prm.gz_rc << " ";
        line << prm.gz_wam << " ";
        line << prm.maxmrate << " ";
        line << prm.mg << " ";

        std::string str = line.str();
        str.pop_back();
        return str;
    }

    std::string to_string(uav::State it, uint64_t mask)
    {
        using namespace std;

        std::bitset<uav::statelen> b(mask);
        stringstream line;
        line << left;

        if (b[0])
        {
            line.setf(ios::fixed);
            line << setw(10) << setprecision(3) << it.t/1000.0;
            line.unsetf(ios::fixed);
        }

        line.setf(ios::fixed);

        if (b[1]) line << setw(9) << it.temp[0];
        if (b[2]) line << setw(9) << it.temp[1];
        if (b[3]) line << setw(12) << it.pres[0];
        if (b[4]) line << setw(12) << it.pres[1];
        if (b[5]) line << setw(9) << it.dz;

        if (b[6]) line << setw(9) << it.h;
        if (b[7]) line << setw(9) << it.p;
        if (b[8]) line << setw(9) << it.r;
        if (b[9]) line << hex << setw(4) << (int) it.calib << dec;

        if (b[10]) line << setw(5) << (int) it.tz;
        if (b[11]) line << setw(5) << (int) it.th;
        if (b[12]) line << setw(5) << (int) it.tp;
        if (b[13]) line << setw(5) << (int) it.tr;

        if (b[14]) line << setw(12) << it.zov;
        if (b[15]) line << setw(12) << it.hov;
        if (b[16]) line << setw(12) << it.pov;
        if (b[17]) line << setw(12) << it.rov;

        for (int i = 0; i < 4; i++)
            if (b[18 + i]) line << setw(4) << (int) it.motors[i];

        if (b[22]) line << setw(10) << std::bitset<16>(it.err);

        return line.str();
    }

    std::deque<std::string> debug, info, error;

    std::string ts(uint64_t ms)
    {
        using namespace std;
        std::stringstream s;
        s.setf(ios::fixed);
        s << "[" << setprecision(3) << ms/1000.0 << "]";
        return s.str();
    }
}
