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

#define DEBUG // if this is defined, external sensors are disabled

namespace chrono = std::chrono;

namespace uav
{
    Control::Control(uav::State initial, uav::Param cfg):

        zpid(cfg.zpidg[0], cfg.zpidg[1],
            cfg.zpidg[2], (uint16_t) cfg.zpidg[3]),
        hpid(cfg.hpidg[0], cfg.hpidg[1],
            cfg.hpidg[2], (uint16_t) cfg.hpidg[3]),
        ppid(cfg.ppidg[0], cfg.ppidg[1],
            cfg.ppidg[2], (uint16_t) cfg.ppidg[3]),
        rpid(cfg.rpidg[0], cfg.rpidg[1],
            cfg.rpidg[2], (uint16_t) cfg.rpidg[3])
    {
        prm = cfg;
        curr = initial;
        prev = {0};
    }

    Control::~Control() { }

    int Control::align()
    {
        #ifndef DEBUG
        const int samples = 100; // altitude samples for home point
        const auto wait = chrono::milliseconds(10);

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
        curr.z1 = 25;
        curr.z2 = 19;
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
        {
	    double zavg = curr.z1 * prm.gz_wam + curr.z2 * (1 - prm.gz_wam);
            double a = dt/(prm.gz_rc + dt);
            curr.dz = a * zavg + (1 - a) * prev.dz;
	}

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
        {
            raw[i] = raw[i] > 100 ? 100 : raw[i] < 0 ? 0 : raw[i];
            if (raw[i] > prev.motors[i] + prm.maxmrate * dt)
                curr.motors[i] = prev.motors[i] + prm.maxmrate * dt;
            else if (raw[i] < prev.motors[i] - prm.maxmrate * dt)
                curr.motors[i] = prev.motors[i] - prm.maxmrate * dt;
            else
                curr.motors[i] = raw[i];
        }

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
