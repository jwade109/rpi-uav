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
std::normal_distribution<double> gaussian(-1.0, 1.0);
std::uniform_real_distribution<double> uniform(-1.0, 1.0);

namespace uav
{
    controller::controller(state initial, param cfg, bool debug):

        debug(debug),
        zpid(cfg.zpidg[0], cfg.zpidg[1],
            cfg.zpidg[2], (uint16_t) cfg.zpidg[3]),
        hpid(cfg.hpidg[0], cfg.hpidg[1],
            cfg.hpidg[2], (uint16_t) cfg.hpidg[3]),
        ppid(cfg.ppidg[0], cfg.ppidg[1],
            cfg.ppidg[2], (uint16_t) cfg.ppidg[3]),
        rpid(cfg.rpidg[0], cfg.rpidg[1],
            cfg.rpidg[2], (uint16_t) cfg.rpidg[3]),
        spid(cfg.spidg[0], cfg.spidg[1],
            cfg.spidg[2], {cfg.spidg[3], cfg.spidg[3], cfg.spidg[3]})
    {
        prm = cfg;
        curr = initial;
        prev = {0};
    }

    controller::~controller() { }

    int controller::align()
    {
        uav::info("RNG seed: " + std::to_string(seed));
        const int samples = 100; // altitude samples for home point
        const auto wait = chrono::milliseconds(10);

        if (!debug)
        {
            int ret1 = imu.begin();
            int ret2 = bmp.begin();
            if (ret1 | ret2)
            {
                std::stringstream ss;
                ss << "Drone alignment failure (IMU: " << ret1
                    << ", BMP: " << ret2 << ")";
                std::cerr << ss.str() << "\n";
                uav::error(ss.str());
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

    int controller::iterate(bool block)
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

        auto stopwatch = chrono::steady_clock::now();
        curr.t = chrono::duration_cast<chrono::milliseconds>(
                now - tstart).count();
        curr.t_abs = chrono::duration_cast<chrono::milliseconds>(
                now.time_since_epoch()).count();

        double dt = (curr.t - prev.t)/1000.0;

        if (!first && (curr.t - prev.t) != 1000/prm.freq)
        {
            uav::error("Timing error (" +
                    std::to_string(prev.t) + " -> " +
                    std::to_string(curr.t) + ")");
            error[0] = 1;
        }

        if (!debug)
        {
            imu_packet m = imu.get();
            curr.pos[3] = m.heading;
            curr.pos[4] = m.pitch;
            curr.pos[5] = m.roll;

            curr.temp[0] = m.temp;
            curr.temp[1] = bmp.getTemperature();
            curr.pres[0] = m.pres;
            curr.pres[1] = bmp.getPressure();
        }
        else
        {
            imu::Vector<3> euler = simulator.euler;
            curr.pos[3] = euler.x();
            curr.pos[4] = euler.y();
            curr.pos[5] = euler.z();

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
        // expected values for heading are (-180,+180)
        if (curr.pos[3] <= -180 || curr.pos[3] >= 180 ||
            !std::isfinite(curr.pos[3]))
        {
            error[3] = 1;
            curr.pos[3] = prev.pos[3];
        }
        // roll is expected to be [-90,+90]
        if (curr.pos[4] < -90 || curr.pos[4] > 90 ||
            !std::isfinite(curr.pos[4]))
        {
            error[4] = 1;
            curr.pos[4] = prev.pos[4];
        }
        // pitch should be (-180,+180)
        if (curr.pos[5] <= -180 || curr.pos[5] >= 180 ||
            !std::isfinite(curr.pos[5]))
        {
            error[5] = 1;
            curr.pos[5] = prev.pos[5];
        }

        // once readings are verified, filter altitude
        {
            auto alt = [](float p, float hp)
                { return 44330 * (1.0 - pow(p/hp, 0.1903)); };

            double z1 = alt(curr.pres[0], prm.p1h);
            double z2 = alt(curr.pres[1], prm.p2h);
            double zavg = z1 * prm.gz_wam + z2 * (1 - prm.gz_wam);
            double a = dt/(prm.gz_rc + dt);
            curr.pos[2] = a * zavg + (1 - a) * prev.pos[2];

            if (debug)
            {
                auto pos = simulator.pos;
                curr.pos[0] = pos.x();
                curr.pos[1] = pos.y();
                curr.pos[2] = pos.z();
            }
        }

        // get target position and attitude from controller
        gettargets();

        // targets should not exceed the normal range for measured values
        if (curr.targets[2] < -50 || curr.targets[2] > 50)
        {
            uav::error("curr.targets[2] = " + std::to_string(curr.targets[2]));
            error[6] = 1;
            curr.targets[2] = prev.targets[2];
        }
        if (curr.targets[3] <= -180 || curr.targets[3] >= 180)
        {
            uav::error("curr.targets[3] = " + std::to_string(curr.targets[3]));
            error[7] = 1;
            curr.targets[3] = prev.targets[3];
        }
        if (curr.targets[4] < -90 || curr.targets[4] > 90)
        {
            uav::error("curr.targets[4] = " + std::to_string(curr.targets[4]));
            error[8] = 1;
            curr.targets[4] = prev.targets[4];
        }
        if (curr.targets[5] <= -180 || curr.targets[5] >= 180)
        {
            uav::error("curr.targets[5] = " + std::to_string(curr.targets[5]));
            error[9] = 1;
            curr.targets[5] = prev.targets[5];
        }

        if (first)
        {
            first = false;
            curr.err = (uint16_t) error.to_ulong();
            curr.comptime = chrono::duration_cast<chrono::nanoseconds>(
                chrono::steady_clock::now() - stopwatch).count();
            return 2;
        }
        
        // assumed that at this point, z, h, r, and p are
        // all trustworthy. process pid controller responses

        auto deg2rad = [](double deg) { return deg * M_PI / 180.0; };
        auto circular_err = [](double h, double t)
        {
            double dist = t - h;
            if (dist > 180) return dist - 360;
            if (dist < -180) return dist + 360;
            return dist;
        };

        imu::Vector<2> P(curr.pos[0], curr.pos[1]),
            T(curr.targets[0], curr.targets[1]);
        imu::Vector<2> Sp = spid.seek(P, T, dt);
        
        imu::Vector<3> euler = traverse(Sp, curr.pos[3], 10, 30).first;
        curr.targets[4] = euler.y();
        curr.targets[5] = euler.z();

        std::stringstream ss;
        ss << P << " " << T << " " << euler;
        uav::debug(ss.str());

        curr.pidov[2] = zpid.seek(deg2rad(curr.pos[2]),
            deg2rad(curr.targets[2]), dt);
        curr.pidov[3] = hpid.seek(0, deg2rad(circular_err(
            curr.pos[3], curr.targets[3])), dt);
        curr.pidov[4] = ppid.seek(deg2rad(curr.pos[4]),
            deg2rad(curr.targets[4]), dt);
        curr.pidov[5] = rpid.seek(0, deg2rad(circular_err(
            curr.pos[5], curr.targets[5])), dt);

        // get default hover thrust
        float hover = prm.mg / (cos(deg2rad(curr.pos[4])) *
            cos(deg2rad(curr.pos[5])));
        static float maxhover(prm.mg / pow(cos(M_PI/6), 2));
        if (hover > maxhover) hover = maxhover;
        if (hover < 0) hover = 0;

        // get raw motor responses by summing pid output variables
        // (linear combination dependent on motor layout)
        double raw[4];
        raw[0] = -curr.pidov[3] + curr.pidov[4] - curr.pidov[5] +
                  curr.pidov[2] + hover;
        raw[1] =  curr.pidov[3] + curr.pidov[4] + curr.pidov[5] +
                  curr.pidov[2] + hover;
        raw[2] = -curr.pidov[3] - curr.pidov[4] + curr.pidov[5] +
                  curr.pidov[2] + hover;
        raw[3] =  curr.pidov[3] - curr.pidov[4] - curr.pidov[5] +
                  curr.pidov[2] + hover;

        for (double& e : raw) e = e > 5 ? 5 : e < 0 ? 0 : e;
        for (int i = 0; i < 4; i++) curr.motors[i] = raw[i];

        simulator.set(raw);
        simulator.stepfor(1000000/prm.freq, 1000);

        curr.err = static_cast<uint16_t>(error.to_ulong());
        curr.comptime = chrono::duration_cast<chrono::nanoseconds>(
            chrono::steady_clock::now() - stopwatch).count();

        return 0;
    }
        
    state controller::getstate()
    {
        return curr;
    }

    void controller::setstate(state s)
    {
        curr = s;
    }

    param controller::getparams()
    {
        return prm;
    }

    void controller::gettargets()
    {
        // arbitrary targets until a true controller is implemented
        static uint64_t last(0);
        if (debug)
        {
            if ((curr.t - last >= 20000) || curr.t == 0)
            {
                if (last != 0) return;
                last = curr.t;

                curr.targets[0] = uniform(gen) * 100;
                curr.targets[1] = uniform(gen) * 100;
                curr.targets[2] = uniform(gen) * 25 + 25;
                curr.targets[3] = uniform(gen) * 180;
                curr.targets[4] = uniform(gen) * 30;
                curr.targets[5] = uniform(gen) * 30;
            }
        }
        else curr.targets.fill(0);
    }
}

std::pair<imu::Vector<3>, imu::Vector<3>>
uav::traverse(imu::Vector<2> S, double heading, double tilt95, double maxtilt)
{
    auto f = [=](double d)
    {
        return 2 * maxtilt / (1 + exp((log(2/1.95 - 1) / tilt95 * d))) - maxtilt;
    };

    const imu::Vector<3> X(1, 0, 0), Y(0, 1, 0), Z(0, 0, 1);
    imu::Vector<3> T(Z.cross({S.x(), S.y(), 0})),
        Xp(X.rotate(Z, heading * M_PI/180)), Yp(Z.cross(Xp));

    double tilt = f(S.magnitude());

    imu::Vector<3> Xpp(Xp.rotate(T, tilt * M_PI/180)),
        Zpp(Z.rotate(T, tilt * M_PI/180)),
        Ypp(Zpp.cross(Xpp)),
        Zp(Xp.cross(Ypp));

    double pitch = asin(Ypp.z()) * 180/M_PI;
    double roll = atan2(Xp.dot(Zpp), Zp.dot(Zpp)) * 180/M_PI;
    return {{heading, pitch, roll}, Zpp};
}
