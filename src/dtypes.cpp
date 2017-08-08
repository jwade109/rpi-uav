
#include <dtypes.h>
#include <string.h>

int tobuffer(Param& prm, char* buffer)
{
    int wptr = 0;
    memcpy(buffer, &prm.freq, sizeof(prm.freq));
    wptr += sizeof(prm.freq);
    memcpy(buffer + wptr, &prm.z1h, sizeof(prm.z1h));
    wptr += sizeof(prm.z1h);
    memcpy(buffer + wptr, &prm.z2h, sizeof(prm.z2h));
    wptr += sizeof(prm.z2h);
    memcpy(buffer + wptr, prm.zpidg, sizeof(prm.zpidg[0]) * 4);
    wptr += (sizeof(prm.zpidg[0]) * 4);
    memcpy(buffer + wptr, prm.hpidg, sizeof(prm.hpidg[0]) * 4);
    wptr += (sizeof(prm.hpidg[0]) * 4);
    memcpy(buffer + wptr, prm.ppidg, sizeof(prm.ppidg[0]) * 4);
    wptr += (sizeof(prm.ppidg[0]) * 4);
    memcpy(buffer + wptr, prm.rpidg, sizeof(prm.rpidg[0]) * 4);
    wptr += (sizeof(prm.rpidg[0]) * 4);
    memcpy(buffer + wptr, &prm.gz_lpf, sizeof(prm.gz_lpf));
    wptr += sizeof(prm.gz_lpf);
    memcpy(buffer + wptr, &prm.gz_wam, sizeof(prm.gz_wam));
    wptr += sizeof(prm.gz_wam);
    memcpy(buffer + wptr, &prm.maxmrate, sizeof(prm.maxmrate));
    wptr += sizeof(prm.maxmrate);
    memcpy(buffer + wptr, &prm.mg, sizeof(prm.mg));
    wptr += sizeof(prm.mg);
    return wptr;
}

int tobuffer(Iter& it, char* buffer)
{
    int wptr = 0;
    memcpy(buffer, &it.t, sizeof(it.t));
    wptr += sizeof(it.t);
    memcpy(buffer + wptr, &it.z1, sizeof(it.z1));
    wptr += sizeof(it.z1);
    memcpy(buffer + wptr, &it.z2, sizeof(it.z2));
    wptr += sizeof(it.z2);
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
    return wptr;
}

int frombuffer(Param& prm, char* buffer)
{
    int rptr = 0;
    memcpy(&prm.freq, buffer, sizeof(prm.freq));
    rptr += sizeof(prm.freq);
    memcpy(&prm.z1h, buffer + rptr, sizeof(prm.z1h));
    rptr += sizeof(prm.z1h);
    memcpy(&prm.z2h, buffer + rptr, sizeof(prm.z2h));
    rptr += sizeof(prm.z2h);
    memcpy(prm.zpidg, buffer + rptr, sizeof(prm.zpidg[0]) * 4);
    rptr += (sizeof(prm.zpidg[0]) * 4);
    memcpy(prm.hpidg, buffer + rptr, sizeof(prm.hpidg[0]) * 4);
    rptr += (sizeof(prm.hpidg[0]) * 4);
    memcpy(prm.ppidg, buffer + rptr, sizeof(prm.ppidg[0]) * 4);
    rptr += (sizeof(prm.ppidg[0]) * 4);
    memcpy(prm.rpidg, buffer + rptr, sizeof(prm.rpidg[0]) * 4);
    rptr += (sizeof(prm.rpidg[0]) * 4);
    memcpy(&prm.gz_lpf, buffer + rptr, sizeof(prm.gz_lpf));
    rptr += sizeof(prm.gz_lpf);
    memcpy(&prm.gz_wam, buffer + rptr, sizeof(prm.gz_wam));
    rptr += sizeof(prm.gz_wam);
    memcpy(&prm.maxmrate, buffer + rptr, sizeof(prm.maxmrate));
    rptr += sizeof(prm.maxmrate);
    memcpy(&prm.mg, buffer + rptr, sizeof(prm.mg));
    rptr += sizeof(prm.mg);
    return rptr;
}

int frombuffer(Iter& it, char* buffer)
{
    int rptr = 0;
    memcpy(&it.t, buffer, sizeof(it.t));
    rptr += sizeof(it.t);
    memcpy(&it.z1, buffer + rptr, sizeof(it.z1));
    rptr += sizeof(it.z1);
    memcpy(&it.z2, buffer + rptr, sizeof(it.z2));
    rptr += sizeof(it.z2);
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
    return rptr;
}

std::string tostring(Param& prm, pformat_t f)
{
    std::string line;
    if (f & FREQ)
        line += std::to_string(prm.freq) + " ";
    if (f & HOMEALT)
    {
        line += std::to_string(prm.z1h) + " ";
        line += std::to_string(prm.z2h) + " ";
    }
    if (f & GAINS)
    {
        line += std::to_string(prm.zpidg[0]) + " ";
        line += std::to_string(prm.zpidg[1]) + " ";
        line += std::to_string(prm.zpidg[2]) + " ";
        line += std::to_string(prm.zpidg[3]) + " ";
        line += std::to_string(prm.zpidg[0]) + " ";
        line += std::to_string(prm.zpidg[1]) + " ";
        line += std::to_string(prm.zpidg[2]) + " ";
        line += std::to_string(prm.zpidg[3]) + " ";
        line += std::to_string(prm.zpidg[0]) + " ";
        line += std::to_string(prm.zpidg[1]) + " ";
        line += std::to_string(prm.zpidg[2]) + " ";
        line += std::to_string(prm.zpidg[3]) + " ";
        line += std::to_string(prm.zpidg[0]) + " ";
        line += std::to_string(prm.zpidg[1]) + " ";
        line += std::to_string(prm.zpidg[2]) + " ";
        line += std::to_string(prm.zpidg[3]) + " ";
    }
    if (f & GAINS)
    {
        line += std::to_string(prm.gz_lpf) + " ";
        line += std::to_string(prm.gz_wam) + " ";
        line += std::to_string(prm.maxmrate) + " ";
        line += std::to_string(prm.mg) + " ";
    }
    line.pop_back();
    return line;
}

std::string tostring(Iter& it)
{
    std::string line;
    line += std::to_string(it.t) + " ";
    line += std::to_string(it.z1) + " ";
    line += std::to_string(it.z2) + " ";
    line += std::to_string(it.dz) + " ";
    line += std::to_string(it.h) + " ";
    line += std::to_string(it.p) + " ";
    line += std::to_string(it.r) + " ";
    line += std::to_string(it.calib) + " ";
    line += std::to_string(it.tz) + " ";
    line += std::to_string(it.th) + " ";
    line += std::to_string(it.tp) + " ";
    line += std::to_string(it.tr) + " ";
    line += std::to_string(it.zov) + " ";
    line += std::to_string(it.hov) + " ";
    line += std::to_string(it.pov) + " ";
    line += std::to_string(it.rov) + " ";
    line += std::to_string(it.motors[0]) + " ";
    line += std::to_string(it.motors[1]) + " ";
    line += std::to_string(it.motors[2]) + " ";
    line += std::to_string(it.motors[3]) + " ";
    line.pop_back();
    return line;
}
