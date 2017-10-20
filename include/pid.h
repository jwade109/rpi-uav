#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <limits>
#include <cmath>
#include <vector.h>
#include <filters.h>

class pid_controller
{
    public:

    const uint8_t freq;
    const double Kp, Ki, Kd;
    const double max_int;
    double p_response, i_response, d_response;

    pid_controller(uint8_t freq, double Kp, double Ki,
                   double Kd, double max = INFINITY);
    void reset();
    double seek_linear(double actual, double setpoint, double dt);
    double seek_degrees(double actual, double setpoint, double dt);

    private:

    uav::derivative<1> error_rate;
    uav::integral<1> steady_state;
};

#endif
