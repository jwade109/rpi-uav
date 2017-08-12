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
    Control::Control(uav::State initial, uav::Param cfg):

        zpid(cfg.zpidg[0], cfg.zpidg[1], cfg.zpidg[2], (uint16_t) cfg.zpidg[3]),
        hpid(cfg.zpidg[0], cfg.hpidg[1], cfg.hpidg[2], (uint16_t) cfg.hpidg[3]),
        ppid(cfg.zpidg[0], cfg.ppidg[1], cfg.ppidg[2], (uint16_t) cfg.ppidg[3]),
        rpid(cfg.rpidg[0], cfg.rpidg[1], cfg.rpidg[2], (uint16_t) cfg.rpidg[3]),

        zlpf(cfg.gz_lpf, initial.dz),

        mr1(cfg.maxmrate, initial.motors[0]),
        mr2(cfg.maxmrate, initial.motors[1]),
        mr3(cfg.maxmrate, initial.motors[2]),
        mr4(cfg.maxmrate, initial.motors[3])
    {
        prm = cfg;
        curr = initial;
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
            fprintf(stderr, "Drone alignment failure (IMU: %d, BMP: %d)\n",
                    ret1, ret2);
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

        start = chrono::steady_clock::now();

        return 0;
    }

    int Control::iterate()
    {
        prev = curr;
        curr = {0};

        curr.t = chrono::duration_cast<chrono::milliseconds>(
                 chrono::steady_clock::now() - start).count();

        double dt = chrono::duration_cast<chrono::duration<double,
               std::ratio<1>>>(chrono::milliseconds(curr.t - prev.t)).count();
        
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
        float hover = prm.mg / (cos(curr.p*M_PI/180) * cos(curr.r*M_PI/180));

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
