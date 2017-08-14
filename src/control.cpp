#include <cassert>
#include <cstring>
#include <ctime>
#include <cmath>
#include <thread>
#include <chrono>

#include <control.h>

// #define DEBUG // if this is defined, external sensors are disabled

namespace chrono = std::chrono;

namespace uav
{
    int tobuffer(uav::Param& prm, char* buffer)
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

    int tobuffer(uav::State& it, char* buffer)
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

    int frombuffer(uav::Param& prm, char* buffer)
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

    int frombuffer(uav::State& it, char* buffer)
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

    std::string tostring(uav::Param prm)
    {
        std::string line;
        line += std::to_string((int) prm.freq) + " ";
        line += std::to_string(prm.z1h) + " ";
        line += std::to_string(prm.z2h) + " ";
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
        line += std::to_string(prm.gz_lpf) + " ";
        line += std::to_string(prm.gz_wam) + " ";
        line += std::to_string(prm.maxmrate) + " ";
        line += std::to_string(prm.mg);
        return line;
    }

    std::string tostring(uav::State it)
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
        line += std::to_string(it.motors[3]);
        return line;
    }
    
    Control::Control(uav::State initial, uav::Param cfg):

        zpid(cfg.zpidg[0], cfg.zpidg[1],
            cfg.zpidg[2], (uint16_t) cfg.zpidg[3]),
        hpid(cfg.zpidg[0], cfg.hpidg[1],
            cfg.hpidg[2], (uint16_t) cfg.hpidg[3]),
        ppid(cfg.zpidg[0], cfg.ppidg[1],
            cfg.ppidg[2], (uint16_t) cfg.ppidg[3]),
        rpid(cfg.rpidg[0], cfg.rpidg[1],
            cfg.rpidg[2], (uint16_t) cfg.rpidg[3]),

        zlpf(cfg.gz_lpf, initial.dz),

        mr1(cfg.maxmrate, initial.motors[0]),
        mr2(cfg.maxmrate, initial.motors[1]),
        mr3(cfg.maxmrate, initial.motors[2]),
        mr4(cfg.maxmrate, initial.motors[3])
    {
        prm = cfg;
        curr = initial;
        prev = {0};
        iters = 0;
    }

    Control::~Control() { }

    int Control::align()
    {
        const int samples = 100; // altitude samples for home point
        const auto wait = chrono::milliseconds(10);

        #ifndef DEBUG
        int ret1 = imu.begin();
        int ret2 = bmp.begin();
        if (ret1 | ret2)
        {
            fprintf(stderr, "Drone alignment failure "
                    "(IMU: %d, BMP: %d)\n", ret1, ret2);
            return 1;
        }
        std::this_thread::sleep_for(chrono::seconds(3));
        #endif

        prm.z1h = 0;
        prm.z2h = 0;

        #ifndef DEBUG
        auto start = chrono::steady_clock::now();
        for (int i = 0; i < samples; i++)
        {
            // need to verify that these are valid readings
            // and that overflow does not occur
            prm.z1h += imu.get().alt;
            prm.z2h += bmp.getAltitude();
            start+=wait;
            std::this_thread::sleep_until(start);
        }
        prm.z1h /= samples;
        prm.z2h /= samples;
        #endif

        return 0;
    }

    int Control::iterate()
    {
        prev = curr;
        curr = {0};

        auto now = chrono::steady_clock::now();
        if (iters == 0) tstart = now;

        curr.t = chrono::duration_cast<chrono::milliseconds>(
                 now - tstart).count();

        double dt = chrono::milliseconds(
                curr.t - prev.t).count()/1000.0;

        if (iters == 0 || dt == 0)
        {
            iters++;
            return 1;
        }
        
        #ifndef DEBUG
        // need to add NaN and sanity checks here
        imu.get(curr.h, curr.p, curr.r, curr.z1, curr.calib);
        curr.z2 = bmp.getAltitude();
        #else
        curr.h = 10;
        curr.p = 4;
        curr.r = -3;
        curr.z1 = 11;
        curr.z2 = 9;
        #endif

        // once readings are verified, filter altitude
        float dz1 = curr.z1 - prm.z1h;
        float dz2 = curr.z2 - prm.z2h;
        float zavg = dz1 * prm.gz_wam + dz2 * (1 - prm.gz_wam);
        curr.dz = zlpf.step(zavg);

        // get target position and attitude from controller
        gettargets(curr);
        
        // assumed that at this point, z, h, r, and p are
        // all trustworthy. process pid controller responses
        curr.hov = hpid.seek(curr.h,  curr.th, dt);
        curr.pov = ppid.seek(curr.p,  curr.tp, dt);
        curr.rov = rpid.seek(curr.r,  curr.tr, dt);
        curr.zov = zpid.seek(curr.dz, curr.tz, dt);

        // get default hover thrust
        float hover = prm.mg / (cos(curr.p * M_PI / 180) *
                                cos(curr.r * M_PI / 180));

        // get raw motor responses by summing pid output variables
        // (linear combination dependent on motor layout)
        float raw[4];
        raw[0] = -curr.hov - curr.pov + curr.rov + curr.zov + hover;
        raw[1] =  curr.hov + curr.pov + curr.rov + curr.zov + hover;
        raw[2] = -curr.hov + curr.pov - curr.rov + curr.zov + hover;
        raw[3] =  curr.hov - curr.pov - curr.rov + curr.zov + hover;

        // for each raw response, trim to [0, 100] and limit rate
        for (int i = 0; i < 4; i++)
            raw[i] = raw[i] > 100 ? 100 : raw[i] < 0 ? 0 : raw[i];

        curr.motors[0] = mr1.step(raw[0], dt);
        curr.motors[1] = mr2.step(raw[1], dt);
        curr.motors[2] = mr3.step(raw[2], dt);
        curr.motors[3] = mr4.step(raw[3], dt);

        iters++;

        return 0;
    }
        
    uav::State Control::getstate()
    {
        return curr;
    }

    void Control::setstate(uav::State state)
    {
        curr = state;
    }

    uav::Param Control::getparams()
    {
        return prm;
    }

    bool Control::debug()
    {
        #ifdef DEBUG
        return true;
        #else
        return false;
        #endif
    }

    void Control::gettargets(uav::State& state)
    {
        // arbitrary targets until a true controller is implemented
        state.tz = 0;
        state.th = 150;
        state.tp = 0;
        state.tr = 0;
    }
}
