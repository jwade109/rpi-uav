#include <cassert>
#include <cstring>
#include <sstream>
#include <cmath>

#include <control.h>

namespace uav
{

controller::controller(state initial, param cfg):
    num_steps(0),
    curr(initial), prev{0}, prm(cfg) { }

int controller::step(const raw_data& raw)
{
    if (num_steps == 0) tstart = std::chrono::steady_clock::now();
    prev = curr;

    if ((curr.status != uav::null_status) && (curr.t - prev.t) != 1000/prm.freq)
        uav::error << "Timing error (" << prev.t
            << " -> " << curr.t << ")" << std::endl;

    curr.pos[3] = raw.ard.euler.x();
    curr.pos[4] = raw.ard.euler.y();
    curr.pos[5] = raw.ard.euler.z();

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

std::bitset<16> controller::validate(
    const state& prev, state& curr)
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

gps_baro_filter::gps_baro_filter(uint8_t f) :
    gps_baro_filter(f, 30, 30, 30, 0.3, 0.3) { }

gps_baro_filter::gps_baro_filter(uint8_t f, unsigned gst,
    unsigned ast, unsigned bst, double arc, double brc) :
    value(0), gps_samples(gst * f), ard_samples(ast * f), bmp_samples(bst * f),
    ard_rc(arc), bmp_rc(brc), dt(1.0/f), home_alt(NAN),
    u_g(gps_samples), u_b1(ard_samples),
    u_b2(bmp_samples), lpf1(ard_rc), lpf2(bmp_rc) { }

double gps_baro_filter::step(double gps, double ard, double bmp)
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

mass_estimator::mass_estimator(uint8_t freq) :
    mass_estimator(freq, 0.2, 1.5, 0.2) { }

mass_estimator::mass_estimator(uint8_t f, double acc_rc,
    double mavg_st, double eps) :
    value(0), freq(f), accel_rc(acc_rc), mavg_sample_time(mavg_st),
    epsilon(eps), accel_lpf(acc_rc),
    moving(freq * mavg_st), diverged(false), dt(1.0/f) { }

double mass_estimator::step(double Fz, double az)
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

motor_director::motor_director(uint8_t f, double mthrust,
    double t95, double mtilt, std::array<double, 24> g) :

    command{0}, freq(f), max_thrust(mthrust),
    tilt_95(t95), max_tilt(mtilt), gains(g),
    xpid(f, g[0], g[1], g[2], g[3]),
    ypid(f, g[4], g[5], g[6], g[7]),
    zpid(f, g[8], g[9], g[10], g[11]),
    hpid(f, g[12], g[13], g[14], g[15]),
    ppid(f, g[16], g[17], g[18], g[19]),
    rpid(f, g[20], g[21], g[22], g[23]) { }

std::array<double, 4> motor_director::step(double mass,
    std::array<double, 6> position, std::array<double, 4> targets)
{
    double xov = xpid.seek(position[0], targets[0]);
    double yov = ypid.seek(position[1], targets[1]);

    imu::Vector<3> euler = traverse({xov, yov},
            position[3], tilt_95, max_tilt).first;

    angle pitch_target = euler.y();
    angle roll_target = euler.z();
    angle heading_target = target_azimuth(position[3], targets[3]);

    double zov = zpid.seek(position[2], targets[2]);
    double hov = hpid.seek(position[3], heading_target);
    double pov = ppid.seek(position[4], pitch_target);
    double rov = rpid.seek(position[5], roll_target);

    // get default hover thrust
    double hover = (mass / (cos(position[4]) * cos(position[5])))/4;

    std::array<double, 4> command;
    command[0] = -hov + pov - rov + zov + hover;
    command[1] =  hov + pov + rov + zov + hover;
    command[2] = -hov - pov + rov + zov + hover;
    command[3] =  hov - pov - rov + zov + hover;

    for (double & e : command) e = e > 5 ? 5 : e < 0 ? 0 : e;
    return command;
}

std::pair<imu::Vector<3>, imu::Vector<3>> uav::motor_director::traverse(
    imu::Vector<2> S, double heading, double tilt95, double maxtilt)
{
    static auto f = [=](double d)
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

} // namespace uav
