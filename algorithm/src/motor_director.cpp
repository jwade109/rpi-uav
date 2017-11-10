#include "motor_director.h"

namespace uav
{

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
