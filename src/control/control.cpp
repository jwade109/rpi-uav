#include <cassert>
#include <cstring>
#include <sstream>
#include <cmath>

#include "control.h"

namespace uav
{

controller::controller(state initial, param cfg):
    num_steps(0),
    curr(initial), prev{0}, prm(cfg),
    hdg_acc(2*M_PI), roll_acc(2*M_PI),
    alt_filter(cfg.freq),
    pos_filter(cfg.freq) { }

int controller::step(const raw_data& raw)
{
    using namespace std::chrono;

    if (num_steps++ == 0) tstart = std::chrono::steady_clock::now();
    prev = curr;

    auto now = steady_clock::now();
    curr.time[0] = duration_cast<milliseconds>(now - tstart).count();
    curr.time[1] = duration_cast<milliseconds>
        (now.time_since_epoch()).count();

    if ((curr.status != uav::null_status) &&
            (curr.time[0] - prev.time[0]) != 1000/prm.freq)
        uav::error << "Timing error (" << prev.time[0]
            << " -> " << curr.time[0] << ")" << std::endl;

    double gps_alt = raw.gps.gga.altitude;
    double b1_alt = altitude(raw.ard.pres);
    double b2_alt = altitude(raw.bmp.pressure);

    auto pos = pos_filter(raw.gps.gga.pos);
    curr.position[0] = pos.x();
    curr.position[1] = pos.y();
    curr.position[2] = alt_filter.step(gps_alt, b1_alt, b2_alt);

    curr.attitude[0] = hdg_acc(angle::from_degrees(raw.ard.euler.x()));
    curr.attitude[1] = angle::from_degrees(raw.ard.euler.y());
    curr.attitude[2] = roll_acc(angle::from_degrees(raw.ard.euler.z()));

    auto comptime = steady_clock::now() - now;
    curr.time[2] = duration_cast<microseconds>(comptime).count();

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

gps_baro_filter::gps_baro_filter(uint8_t f) :
    gps_baro_filter(f, 30, 30, 30, 0.7, 0.7) { }

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

gps_position_filter::gps_position_filter(uint8_t freq) :
    gps_position_filter(freq, 3) { }

gps_position_filter::gps_position_filter(uint8_t f, double rc) :
    freq(f), rc(rc), dt(1.0/f), first(true), lpfx(rc), lpfy(rc) { }

imu::Vector<2> gps_position_filter::operator () (coordinate pos)
{
    if (first) { first = false; home = pos; }
    auto d = pos - home;
    return value = {lpfx.step(d.x(), dt), lpfy.step(d.y(), dt)};
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
    double t95, double mtilt, std::array<double, 20> g) :

    command{0}, freq(f), max_thrust(mthrust),
    tilt_95(t95), max_tilt(mtilt), gains(g),
    xpid(f, g[0], g[1], g[2], g[3]),
    ypid(f, g[0], g[1], g[2], g[3]),
    zpid(f, g[4], g[5], g[6], g[7]),
    hpid(f, g[8], g[9], g[10], g[11]),
    ppid(f, g[12], g[13], g[14], g[15]),
    rpid(f, g[16], g[17], g[18], g[19]) { }

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
