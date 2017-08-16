#include <cassert>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <thread>
#include <chrono>
#include <bitset>

#include <monitor.h>
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
        memcpy(buffer + wptr, &it.err, sizeof(it.err));
        wptr += sizeof(it.err);
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
        memcpy(&it.err, buffer + rptr, sizeof(it.err));
        rptr += sizeof(it.err);
        return rptr;
    }

    std::string to_string(uav::Param prm)
    {
        std::stringstream line;
        line << (int) prm.freq << " " << prm.z1h << " "
             << prm.z2h << " " << prm.zpidg[0] << " "
             << prm.zpidg[1] << " " << prm.zpidg[2] << " "
             << prm.zpidg[3] << " " << prm.hpidg[0] << " "
             << prm.hpidg[1] << " " << prm.hpidg[2] << " "
             << prm.hpidg[3] << " " << prm.ppidg[0] << " "
             << prm.ppidg[1] << " " << prm.ppidg[2] << " "
             << prm.ppidg[3] << " " << prm.rpidg[0] << " "
             << prm.rpidg[1] << " " << prm.rpidg[2] << " "
             << prm.rpidg[3] << " " << prm.gz_lpf << " "
             << prm.gz_wam << " " << prm.maxmrate << " " << prm.mg;
        return line.str();
    }

    std::string to_string(uav::State it)
    {
        using namespace std;
        stringstream line;
        line << left
             << setw(8) << (uint64_t) it.t
             << setw(15) << it.z1
             << setw(15) << it.z2
             << setw(15) << it.dz
             << setw(7) << it.h
             << setw(7) << it.p
             << setw(7) << it.r
             << hex
             << setw(4) << (int) it.calib
             << dec
             // << setw(5) << left << (int) it.tz
             // << setw(5) << left << (int) it.th
             // << setw(5) << left << (int) it.tp
             // << setw(5) << left << (int) it.tr
             // << setw(12) << left << it.zov
             // << setw(12) << left << it.hov
             // << setw(12) << left << it.pov
             // << setw(12) << left << it.rov
             << setw(4) << (int) it.motors[0]
             << setw(4) << (int) it.motors[1]
             << setw(4) << (int) it.motors[2]
             << setw(4) << (int) it.motors[3]
             << setw(10) << left << std::bitset<9>(it.err);
        return line.str();
    }
    
    Control::Control(uav::State initial, uav::Param cfg):

        zpid(cfg.zpidg[0], cfg.zpidg[1],
            cfg.zpidg[2], (uint16_t) cfg.zpidg[3]),
        hpid(cfg.hpidg[0], cfg.hpidg[1],
            cfg.hpidg[2], (uint16_t) cfg.hpidg[3]),
        ppid(cfg.ppidg[0], cfg.ppidg[1],
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

        #ifdef DEBUG
        uav::log::events.put("Starting in debug mode.\n");
        #endif
        uav::log::events.put("Alignment completed.\n");

        return 0;
    }

    int Control::iterate(bool block)
    {
        static int count;
        static bool first(true);
        std::bitset<16> error(0);
        prev = curr;

        auto now = chrono::steady_clock::now();
        if (first) tstart = now;
        else if (block)
        {
            uint64_t ts = prev.t + 1000/prm.freq;
            while (now < tstart + chrono::milliseconds(ts))
                now = chrono::steady_clock::now();
        }

        curr.t = chrono::duration_cast<chrono::milliseconds>(
                 now - tstart).count();

        double dt = chrono::milliseconds(
                curr.t - prev.t).count()/1000.0;

        if (!first && dt <= 0)
        {
            uav::log::events.put(std::to_string(curr.t) + 
                                 " Error: dt <= 0\n");
            error[0] = 1;
            return 1;
        }
        
        #ifndef DEBUG
        float z1raw;
        imu.get(curr.h, curr.p, curr.r, z1raw, curr.calib);
        count++;
        curr.z1 = z1raw - prm.z1h;
        curr.z2 = bmp.getAltitude() - prm.z2h;
        #else
        curr.h = 10;
        curr.p = 4;
        curr.r = -3;
        curr.z1 = 3;
        curr.z2 = 1;
        #endif

        // if an altitude measurement is deemed invalid, use the
        // other barometer's measurement, or the most recent valid
        // measurement
        //
        // for invalid attitude readings, use the previous measurement
        
        // assume during normal operation, neither altitude will stray
        // more than 50 meters from its respective home point
        if (curr.z1 < -50 || curr.z1 > 50)
        {
            error[1] = 1;
            uav::log::events.put(std::to_string(curr.t) +
                    " Error! z1 = " + std::to_string(curr.z1) + "\n");
        }
        if (curr.z2 < -50 || curr.z2 > 50)
        {
            error[2] = 1;
            uav::log::events.put(std::to_string(curr.t) +
                    " Error! z2 = " + std::to_string(curr.z2) + "\n");
        }
        // decision tree for correcting bad alt measurements
        if (error[1] && error[2]) // uh-oh, both altimeters are bad
        {
            curr.z1 = prev.z1;
            curr.z2 = prev.z2;
        }
        else if (error[1]) // z1 is bad, z2 is good
            curr.z1 = curr.z2;
        else if (error[2]) // z2 is bad, z1 is good
            curr.z2 = curr.z1;

        // verify that all attitudes are normal or zero
        // expected values for curr.h are (-180,+180)
        if (curr.h <= -180 || curr.h >= 180 || !std::isfinite(curr.h))
        {
            error[3] = 1;
            uav::log::events.put(std::to_string(curr.t) +
                    " Error! hdg = " + std::to_string(curr.h) + "\n");
            curr.h = prev.h;
        }
        // curr.p is expected to be [-90,+90]
        if (curr.p < -90 || curr.p > 90 || !std::isfinite(curr.p))
        {
            error[4] = 1;
            uav::log::events.put(std::to_string(curr.t) +
                    " Error! pitch = " + std::to_string(curr.p) + "\n");
            curr.p = prev.p;
        }
        // curr.r should be (-180,+180)
        if (curr.r <= -180 || curr.r >= 180 || !std::isfinite(curr.r))
        {
            error[5] = 1;
            uav::log::events.put(std::to_string(curr.t) +
                    " Error! roll = " + std::to_string(curr.r) + "\n");
            curr.r = prev.r;
        }

        // once readings are verified, filter altitude
        float zavg = curr.z1 * prm.gz_wam + curr.z2 * (1 - prm.gz_wam);
        curr.dz = zlpf.step(zavg);

        // get target position and attitude from controller
        gettargets(curr);

        // targets should not exceed the normal range for measured values
        if (curr.tz < -50 || curr.tz > 50)
        {
            error[6] = 1;
            uav::log::events.put(std::to_string(curr.t) +
                    " Error! alt target = " + std::to_string(curr.th) + "\n");
            curr.tz = prev.tz;
        }
        if (curr.th <= -180 || curr.th >= 180)
        {
            error[7] = 1;
            uav::log::events.put(std::to_string(curr.t) +
                    " Error! hdg target = " + std::to_string(curr.th) + "\n");
            curr.th = prev.th;
        }
        if (curr.tp < -90 || curr.tp > 90)
        {
            error[8] = 1;
            uav::log::events.put(std::to_string(curr.t) +
                    " Error! pitch target = " + std::to_string(curr.th) + "\n");
            curr.tp = prev.tp;
        }
        if (curr.tr <= -180 || curr.tr >= 180)
        {
            error[9] = 1;
            uav::log::events.put(std::to_string(curr.t) +
                    " Error! roll target = " + std::to_string(curr.tr) + "\n");
            curr.tr = prev.tr;
        }

        if (first)
        {
            first = false;
            curr.err = (uint16_t) error.to_ulong();
            uav::log::events.put(std::to_string(curr.t) +
                                 " First iter\n");
            return 2;
        }
        
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

        curr.err = (uint16_t) error.to_ulong();

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
