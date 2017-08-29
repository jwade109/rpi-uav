#include <cassert>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <thread>
#include <chrono>
#include <bitset>
#include <random>

#include <control.h>

namespace chrono = std::chrono;

unsigned seed = chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine gen(seed);
std::normal_distribution<double> gaussian(0.0, 1.0);

namespace uav
{
    Control::Control(uav::State initial, uav::Param cfg, bool debug):

        debug(debug),
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
        const int samples = 100; // altitude samples for home point
        const auto wait = chrono::milliseconds(10);

        if (!debug)
        {
            int ret1 = imu.begin();
            int ret2 = bmp.begin();
            if (ret1 | ret2)
            {
                fprintf(stderr, "Drone alignment failure "
                    "(IMU: %d, BMP: %d)\n", ret1, ret2);
                return 1;
            }
            std::this_thread::sleep_for(chrono::seconds(3));
        }

        prm.p1h = 0;
        prm.p2h = 0;

        auto start = chrono::steady_clock::now();
        for (int i = 0; i < samples; i++)
        {
            if (!debug)
            {
                prm.p1h += imu.get().pres;
                prm.p2h += bmp.getPressure();
            }
            else
            {
                prm.p1h += 101200 + gaussian(gen) * 10;
                prm.p2h += 101150 + gaussian(gen) * 10;
            }
            start+=wait;
            std::this_thread::sleep_until(start);
        }
        prm.p1h /= samples;
        prm.p2h /= samples;

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
            error[0] = 1;
            return 1;
        }

        if (!debug)
        {
            Message m = imu.get();
            curr.h = m.heading;
            curr.p = m.pitch;
            curr.r = m.roll;

            curr.temp[0] = m.temp;
            curr.temp[1] = bmp.getTemperature();
            curr.pres[0] = m.pres;
            curr.pres[1] = bmp.getPressure();
        }
        else
        {
            curr.h = prev.h + gaussian(gen);
            curr.p = prev.p + gaussian(gen);
            curr.r = prev.r + gaussian(gen);
            curr.pres[0] = prm.p1h + gaussian(gen) * 10;
            curr.pres[1] = prm.p2h + gaussian(gen) * 10;
            curr.temp[0] = curr.temp[1] = 25.6 + gaussian(gen) * 0.1;
        }

        // if a pressure measurement is deemed invalid, use the
        // other barometer's measurement, or the most recent valid
        // measurement
        //
        // for invalid attitude readings, use the previous measurement
        
        // assume during normal operation, pressure will be atmost
        // 101,325 Pa (0 m MSL), and no less than 80,000 Pa (~2000 m MSL)
        if (curr.pres[0] < 80000 || curr.pres[0] > 101325)
        {
            error[1] = 1;
        }
        if (curr.pres[1] < 80000 || curr.pres[1] > 101325)
        {
            error[2] = 1;
        }
        // decision tree for correcting bad alt measurements
        if (error[1] && error[2]) // uh-oh, both altimeters are bad
        {
            curr.pres[0] = prev.pres[0];
            curr.pres[1] = prev.pres[1];
        }
        else if (error[1]) // p1 is bad, p2 is good
            curr.pres[0] = curr.pres[1];
        else if (error[2]) // p2 is bad, p1 is good
            curr.pres[1] = curr.pres[0];

        // verify that all attitudes are normal or zero
        // expected values for curr.h are (-180,+180)
        if (curr.h <= -180 || curr.h >= 180 || !std::isfinite(curr.h))
        {
            error[3] = 1;
            curr.h = prev.h;
        }
        // curr.p is expected to be [-90,+90]
        if (curr.p < -90 || curr.p > 90 || !std::isfinite(curr.p))
        {
            error[4] = 1;
            curr.p = prev.p;
        }
        // curr.r should be (-180,+180)
        if (curr.r <= -180 || curr.r >= 180 || !std::isfinite(curr.r))
        {
            error[5] = 1;
            curr.r = prev.r;
        }

        // once readings are verified, filter altitude
        {
            auto alt = [](float p, float hp)
                { return 44330 * (1.0 - pow(p/hp, 0.1903)); };

            
            double z1 = alt(curr.pres[0], prm.p1h);
            double z2 = alt(curr.pres[1], prm.p2h);
            double zavg = z1 * prm.gz_wam + z2 * (1 - prm.gz_wam);
            double a = dt/(prm.gz_rc + dt);
            curr.dz = a * zavg + (1 - a) * prev.dz;
        }

        // get target position and attitude from controller
        gettargets();

        // targets should not exceed the normal range for measured values
        if (curr.tz < -50 || curr.tz > 50)
        {
            std::stringstream e;
            e << uav::ts(curr.t) << " curr.tz = " << curr.tz;
            uav::error.push_back(e.str());
            error[6] = 1;
            curr.tz = prev.tz;
        }
        if (curr.th <= -180 || curr.th >= 180)
        {
            std::stringstream e;
            e << uav::ts(curr.t) << " curr.th = " << curr.th;
            uav::error.push_back(e.str());
            error[7] = 1;
            curr.th = prev.th;
        }
        if (curr.tp < -90 || curr.tp > 90)
        {
            std::stringstream e;
            e << uav::ts(curr.t) << " curr.tp = " << curr.tp;
            uav::error.push_back(e.str());
            error[8] = 1;
            curr.tp = prev.tp;
        }
        if (curr.tr <= -180 || curr.tr >= 180)
        {
            std::stringstream e;
            e << uav::ts(curr.t) << " curr.tr = " << curr.tr;
            uav::error.push_back(e.str());
            error[9] = 1;
            curr.tr = prev.tr;
        }

        if (first)
        {
            first = false;
            curr.err = (uint16_t) error.to_ulong();
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

        uav::debug.push_back(std::to_string(hover));

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
            /*
            if (raw[i] > prev.motors[i] + prm.maxmrate * dt)
                curr.motors[i] = prev.motors[i] + prm.maxmrate * dt;
            else if (raw[i] < prev.motors[i] - prm.maxmrate * dt)
                curr.motors[i] = prev.motors[i] - prm.maxmrate * dt;
            else
            */
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

    void Control::gettargets()
    {
        // arbitrary targets until a true controller is implemented
        if (debug)
        {
            curr.tz = prev.tz + (int) gaussian(gen);
            curr.th = prev.th + (int) gaussian(gen);
            curr.tp = prev.tp + (int) gaussian(gen);
            curr.tr = prev.tr + (int) gaussian(gen);
        }
        else curr.tz = curr.th = curr.tp = curr.tr = 0;
    }
}
