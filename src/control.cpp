#include <cassert>
#include <cstring>
#include <sstream>
#include <cmath>

#include <control.h>

uav::controller::controller(state initial, param cfg):

    num_steps(0),
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

int uav::controller::step(const uav::raw_data& raw)
{
    if (num_steps == 0) tstart = std::chrono::steady_clock::now();
    std::bitset<16> error(0);
    prev = curr;

    auto stopwatch = std::chrono::steady_clock::now();
    double dt = (curr.t - prev.t)/1000.0;

    if ((curr.status != uav::null_status) && (curr.t - prev.t) != 1000/prm.freq)
    {
        uav::error << "Timing error ("
            << prev.t << " -> " << curr.t << ")" << std::endl;
        error[0] = 1;
    }

    error |= validate(prev, curr);

    // end here if this is the first iteration
    if (curr.status == uav::null_status)
    {
        curr.status = uav::no_vel;
        simulator.stepfor(1000000/prm.freq, 1000);
        curr.err = (uint16_t) error.to_ulong();
        curr.comptime = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - stopwatch).count();
        return 2;
    }

    // assumed that at this point, z, h, r, and p are
    // all trustworthy. process pid controller responses

    curr.pidov[0] = xpid.seek_linear(curr.pos[0], curr.targets[0], dt);
    curr.pidov[1] = ypid.seek_linear(curr.pos[1], curr.targets[1], dt);

    imu::Vector<3> euler = traverse({curr.pidov[0], curr.pidov[1]},
            curr.pos[3], prm.tilt95, prm.maxtilt).first;

    curr.targets[4] = euler.y();
    curr.targets[5] = euler.z();

    curr.pidov[2] = zpid.seek_linear(curr.pos[2], curr.targets[2], dt);
    curr.pidov[3] = hpid.seek_degrees(curr.pos[3], curr.targets[3], dt);
    curr.pidov[4] = ppid.seek_degrees(curr.pos[4], curr.targets[4], dt);
    curr.pidov[5] = rpid.seek_degrees(curr.pos[5], curr.targets[5], dt);

    // get default hover thrust
    float hover = prm.mg / (cos(deg2rad(curr.pos[4])) *
        cos(deg2rad(curr.pos[5])));
    static float maxhover(prm.mg / pow(cos(M_PI/6), 2));
    if (hover > maxhover) hover = maxhover;
    if (hover < 0) hover = 0;

    auto mout = pid2motor(curr.pidov, hover);

    simulator.set(mout[0], mout[1], mout[2], mout[3]);
    simulator.stepfor(1000000/prm.freq, 1000);

    curr.err = static_cast<uint16_t>(error.to_ulong());
    curr.comptime = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - stopwatch).count();

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

std::array<float, 4> uav::controller::pid2motor(
    std::array<float, 6> pidov, float hover)
{
    std::array<float, 4> motor_out;
    motor_out[0] = -pidov[3] + pidov[4] - pidov[5] + pidov[2] + hover;
    motor_out[1] =  pidov[3] + pidov[4] + pidov[5] + pidov[2] + hover;
    motor_out[2] = -pidov[3] - pidov[4] + pidov[5] + pidov[2] + hover;
    motor_out[3] =  pidov[3] - pidov[4] - pidov[5] + pidov[2] + hover;

    for (auto& e : motor_out) e = e > 5 ? 5 : e < 0 ? 0 : e;
    return motor_out;
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
        uav::error << "pres[0] = " << curr.pres[0] << std::endl;
        error[1] = 1;
    }
    if (curr.pres[1] < 80000 || curr.pres[1] > 101325)
    {
        uav::error << "pres[1] = " << curr.pres[1] << std::endl;
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
        uav::error << "pos[3] = " << curr.pos[3] << std::endl;
        error[3] = 1;
        uav::info("Using previous value of pos[3]");
        curr.pos[3] = prev.pos[3];
    }
    // roll is expected to be [-90,+90]
    if (curr.pos[4] < -90 || curr.pos[4] > 90 ||
        !std::isfinite(curr.pos[4]))
    {
        uav::error << "pos[4] = " << curr.pos[4] << std::endl;
        error[4] = 1;
        uav::info("Using previous value of pos[4]");
        curr.pos[4] = prev.pos[4];
    }
    // pitch should be (-180,+180)
    if (curr.pos[5] <= -180 || curr.pos[5] >= 180 ||
        !std::isfinite(curr.pos[5]))
    {
        uav::error << "pos[5] = " << curr.pos[5] << std::endl;
        error[5] = 1;
        uav::info("Using previous value of pos[5]");
        curr.pos[5] = prev.pos[5];
    }

    // targets should not exceed the normal range for measured values
    if (curr.targets[2] < -50 || curr.targets[2] > 50)
    {
        uav::error << "curr.targets[2] = " << curr.targets[2] << std::endl;
        error[6] = 1;
        uav::info("Using previous value of targets[2]");
        curr.targets[2] = prev.targets[2];
    }
    if (curr.targets[3] <= -180 || curr.targets[3] >= 180)
    {
        uav::error << "curr.targets[3] = " << curr.targets[3] << std::endl;
        error[7] = 1;
        uav::info("Using previous value of targets[3]");
        curr.targets[3] = prev.targets[3];
    }
    if (curr.targets[4] < -90 || curr.targets[4] > 90)
    {
        uav::error << "curr.targets[4] = " << curr.targets[4] << std::endl;
        error[8] = 1;
        uav::info("Using previous value of targets[4]");
        curr.targets[4] = prev.targets[4];
    }
    if (curr.targets[5] <= -180 || curr.targets[5] >= 180)
    {
        uav::error << "curr.targets[5] = " << curr.targets[5] << std::endl;
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

uav::gps_baro_filter::gps_baro_filter(uint8_t f) :
    gps_baro_filter(f, 30, 30, 30, 0.3, 0.3) { }

uav::gps_baro_filter::gps_baro_filter(uint8_t f, unsigned gst,
    unsigned ast, unsigned bst, double arc, double brc) :
    value(0), gps_samples(gst * f), ard_samples(ast * f), bmp_samples(bst * f),
    ard_rc(arc), bmp_rc(brc), dt(1.0/f), home_alt(NAN),
    u_g(gps_samples), u_b1(ard_samples),
    u_b2(bmp_samples), lpf1(ard_rc), lpf2(bmp_rc) { }

double uav::gps_baro_filter::step(double gps, double ard, double bmp)
{
    if (isnan(home_alt)) home_alt = gps;

    u_g.step(gps);
    u_b1.step(ard);
    u_b2.step(bmp);

    double d1_est = u_b1.value - u_g.value;
    double d2_est = u_b2.value - u_g.value;
    double alt1_est = ard - d1_est - home_alt;
    double alt2_est = bmp - d2_est - home_alt;
    double alt1_smooth = lpf1.step(alt1_est, dt);
    double alt2_smooth = lpf2.step(alt2_est, dt);

    return (value = 0.5 * alt1_smooth + 0.5 * alt2_smooth);
}

uav::mass_estimator::mass_estimator(uint8_t freq) :
    mass_estimator(freq, 0.2, 1.5, 0.2) { }

uav::mass_estimator::mass_estimator(uint8_t f, double acc_rc,
    double mavg_st, double eps) :
    value(0), freq(f), accel_rc(acc_rc), mavg_sample_time(mavg_st),
    epsilon(eps), accel_lpf(acc_rc),
    moving(freq * mavg_st), diverged(false), dt(1.0/f) { }

double uav::mass_estimator::step(double Fz, double az)
{
    if (az == 9.81) return value;
    double smooth_accel = accel_lpf.step(az, dt);
    double m_hat = Fz/(smooth_accel + 9.81);

    double local_m_hat = moving.step(m_hat);
    double global_m_hat = global.step(m_hat);

    bool large_gap = std::abs(global_m_hat - local_m_hat) > epsilon;
    if (large_gap && !diverged) global = uav::running_average();
    diverged = large_gap;

    return value = diverged ? local_m_hat : global_m_hat;
}
