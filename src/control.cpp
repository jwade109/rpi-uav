#include <cassert>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <cmath>
#include <thread>
#include <random>

#include <control.h>
#include <filters.h>

namespace chrono = std::chrono;

unsigned seed = chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine gen(seed);
std::normal_distribution<double> gaussian(-1.0, 1.0);
std::uniform_real_distribution<double> uniform(-1.0, 1.0);

uav::controller::controller(state initial, param cfg):

    xpid(cfg.spidg[0], cfg.spidg[1], cfg.spidg[2], cfg.spidg[3]),
    ypid(cfg.spidg[0], cfg.spidg[1], cfg.spidg[2], cfg.spidg[3]),
    zpid(cfg.zpidg[0], cfg.zpidg[1], cfg.zpidg[2], cfg.zpidg[3]),
    hpid(cfg.hpidg[0], cfg.hpidg[1], cfg.hpidg[2], cfg.hpidg[3]),
    ppid(cfg.ppidg[0], cfg.ppidg[1], cfg.ppidg[2], cfg.ppidg[3]),
    rpid(cfg.rpidg[0], cfg.rpidg[1], cfg.rpidg[2], cfg.rpidg[3])
{
    prm = cfg;
    curr = initial;
    prev = {0};
}

uav::controller::~controller() { }

int uav::controller::begin()
{
    auto start = chrono::steady_clock::now();
    return 0;
}

int uav::controller::iterate(bool block)
{
    std::bitset<16> error(0);
    prev = curr;

    {
        auto now = chrono::steady_clock::now();
        if (curr.status == uav::null_status) tstart = now;
        else if (block)
        {
            uint64_t ts = prev.t + 1000/prm.freq;
            while (now < tstart + chrono::milliseconds(ts))
                now = chrono::steady_clock::now();
        }
        curr.t = chrono::duration_cast<chrono::milliseconds>(
                now - tstart).count();
        curr.t_abs = chrono::duration_cast<chrono::milliseconds>(
                now.time_since_epoch()).count();
    }

    auto stopwatch = chrono::steady_clock::now();
    double dt = (curr.t - prev.t)/1000.0;

    if ((curr.status != uav::null_status) && (curr.t - prev.t) != 1000/prm.freq)
    {
        uav::errorstream << "Timing error ("
            << prev.t << " -> " << curr.t << ")" << std::endl;
        error[0] = 1;
    }

    /*
    imu_packet m = imu.get();
    curr.pos[3] = m.heading;
    curr.pos[4] = m.pitch;
    curr.pos[5] = m.roll;

    curr.pres[0] = m.pres;
    curr.pres[1] = bmp.getPressure();
    */

    error |= validate(prev, curr);

    // get target position and attitude from controller
    gettargets();

    // TODO add altitude filter here

    // end here if this is the first iteration
    if (curr.status == uav::null_status)
    {
        curr.status = uav::no_vel;
        simulator.stepfor(1000000/prm.freq, 1000);
        curr.err = (uint16_t) error.to_ulong();
        curr.comptime = chrono::duration_cast<chrono::nanoseconds>(
            chrono::steady_clock::now() - stopwatch).count();
        return 2;
    }
    
    // determine convergence on target position
    {
        const double epsilon = 0.1;
        bool converged = true;
        for (int i = 0; i < 6 && converged; i++)
        {
            converged &= ((std::abs(curr.targets[i] - curr.pos[i]) < epsilon)
                & (std::abs(curr.pidov[i]) < epsilon));
        }
        curr.status = converged ? uav::pos_hold : uav::pos_seek;
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

    curr.pidov[0] = xpid.seek(curr.pos[0], curr.targets[0], dt);
    curr.pidov[1] = xpid.seek(curr.pos[1], curr.targets[1], dt);

    imu::Vector<3> euler = traverse({curr.pidov[0], curr.pidov[1]},
            curr.pos[3], prm.tilt95, prm.maxtilt).first;

    curr.targets[4] = euler.y();
    curr.targets[5] = euler.z();

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
    
uav::state uav::controller::getstate()
{
    return curr;
}

void uav::controller::setstate(state s)
{
    curr = s;
}

uav::param uav::controller::getparams()
{
    return prm;
}

void uav::controller::gettargets()
{
    // arbitrary targets until a true controller is implemented
    static uint64_t last(0);
    if (curr.status != uav::pos_hold) last = curr.t;
    if ((curr.t - last >= 5000) || curr.t == 0)
    {
        last = curr.t;

        curr.targets[0] = uniform(gen) * 100;
        curr.targets[1] = uniform(gen) * 100;
        curr.targets[2] = uniform(gen) * 25 + 25;
        curr.targets[3] = uniform(gen) * 180;
    }
}

std::bitset<16> uav::controller::validate(
    const uav::state& prev, uav::state& curr)
{
    std::bitset<16> error;
    // if a pressure measurement is deemed invalid, use the
    // other barometer's measurement, or the most recent valid
    // measurement
    //
    // for invalid attitude readings, use the previous measurement

    // assume during normal operation, pressure will be atmost
    // 101,325 Pa (0 m MSL), and no less than 80,000 Pa (~2000 m MSL)
    if (curr.pres[0] < 80000 || curr.pres[0] > 101325)
    {
        uav::errorstream << "pres[0] = " << curr.pres[0] << std::endl;
        error[1] = 1;
    }
    if (curr.pres[1] < 80000 || curr.pres[1] > 101325)
    {
        uav::errorstream << "pres[1] = " << curr.pres[1] << std::endl;
        error[2] = 1;
    }
    // decision tree for correcting bad alt measurements
    if (error[1] && error[2]) // uh-oh, both altimeters are bad
    {
        uav::info("Using previous pressure measurements");
        curr.pres[0] = prev.pres[0];
        curr.pres[1] = prev.pres[1];
    }
    else if (error[1]) // p1 is bad, p2 is good
    {
        uav::info("using pres[1] for value of pres[0]");
        curr.pres[0] = curr.pres[1];
    }
    else if (error[2]) // p2 is bad, p1 is good
    {
        uav::info("using pres[0] for value of pres[1]");
        curr.pres[1] = curr.pres[0];
    }

    // verify that all attitudes are normal or zero
    // expected values for heading are (-180,+180)
    if (curr.pos[3] <= -180 || curr.pos[3] >= 180 ||
        !std::isfinite(curr.pos[3]))
    {
        uav::errorstream << "pos[3] = " << curr.pos[3] << std::endl;
        error[3] = 1;
        uav::info("Using previous value of pos[3]");
        curr.pos[3] = prev.pos[3];
    }
    // roll is expected to be [-90,+90]
    if (curr.pos[4] < -90 || curr.pos[4] > 90 ||
        !std::isfinite(curr.pos[4]))
    {
        uav::errorstream << "pos[4] = " << curr.pos[4] << std::endl;
        error[4] = 1;
        uav::info("Using previous value of pos[4]");
        curr.pos[4] = prev.pos[4];
    }
    // pitch should be (-180,+180)
    if (curr.pos[5] <= -180 || curr.pos[5] >= 180 ||
        !std::isfinite(curr.pos[5]))
    {
        uav::errorstream << "pos[5] = " << curr.pos[5] << std::endl;
        error[5] = 1;
        uav::info("Using previous value of pos[5]");
        curr.pos[5] = prev.pos[5];
    }

    // targets should not exceed the normal range for measured values
    if (curr.targets[2] < -50 || curr.targets[2] > 50)
    {
        uav::errorstream << "curr.targets[2] = " << curr.targets[2] << std::endl;
        error[6] = 1;
        uav::info("Using previous value of targets[2]");
        curr.targets[2] = prev.targets[2];
    }
    if (curr.targets[3] <= -180 || curr.targets[3] >= 180)
    {
        uav::errorstream << "curr.targets[3] = " << curr.targets[3] << std::endl;
        error[7] = 1;
        uav::info("Using previous value of targets[3]");
        curr.targets[3] = prev.targets[3];
    }
    if (curr.targets[4] < -90 || curr.targets[4] > 90)
    {
        uav::errorstream << "curr.targets[4] = " << curr.targets[4] << std::endl;
        error[8] = 1;
        uav::info("Using previous value of targets[4]");
        curr.targets[4] = prev.targets[4];
    }
    if (curr.targets[5] <= -180 || curr.targets[5] >= 180)
    {
        uav::errorstream << "curr.targets[5] = " << curr.targets[5] << std::endl;
        error[9] = 1;
        uav::info("Using previous value of targets[5]");
        curr.targets[5] = prev.targets[5];
    }
    return error;
}

std::pair<imu::Vector<3>, imu::Vector<3>> uav::controller::traverse(
    imu::Vector<2> S, double heading, double tilt95, double maxtilt)
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
