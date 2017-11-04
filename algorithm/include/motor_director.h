#ifndef MOTOR_DIRECTOR_H
#define MOTOR_DIRECTOR_H

#include <array>
#include <uav/math>
#include "pid.h"

namespace uav
{

class motor_director
{
    public:

    std::array<double, 4> command;
    const uint8_t freq;
    const double max_thrust;
    const double tilt_95;
    const double max_tilt;
    const std::array<double, 20> gains;

    motor_director(uint8_t freq,
                   double max_thrust,
                   double tilt_95,
                   double max_tilt,
                   std::array<double, 20> gains);

    std::array<double, 4> step(double mass,
                               std::array<double, 6> position,
                               std::array<double, 4> targets);

    private:

    static std::pair<imu::Vector<3>, imu::Vector<3>> traverse(
        imu::Vector<2> S, double heading, double tilt95, double maxtilt);

    pid_controller xpid, ypid, zpid, hpid, ppid, rpid;
};

} // namespace uav

#endif // CONTROL_H
